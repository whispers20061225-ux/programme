from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from graspgen_geometry import load_gripper_geometry, reconstruct_surface_contacts


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float32).reshape((3,))
    length = float(np.linalg.norm(values))
    if length <= 1e-6:
        return np.zeros((3,), dtype=np.float32)
    return values / length


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    quaternion /= norm
    return quaternion.astype(float).tolist()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Query a GraspGen ZMQ server and convert grasps to Programme proposal JSON",
    )
    parser.add_argument("--repo-root", required=True, help="Path to the GraspGen repo root")
    parser.add_argument("--cloud-npy", required=True, help="Input target cloud .npy path")
    parser.add_argument("--out-json", required=True, help="Output proposal JSON path")
    parser.add_argument("--frame-id", required=True, help="Frame id for emitted proposals")
    parser.add_argument("--host", default="127.0.0.1", help="GraspGen ZMQ server host")
    parser.add_argument("--port", type=int, default=5556, help="GraspGen ZMQ server port")
    parser.add_argument("--timeout-sec", type=float, default=12.0, help="Server request timeout")
    parser.add_argument("--gripper-config", default="", help="Optional explicit GraspGen gripper config path")
    parser.add_argument("--grasp-threshold", type=float, default=-1.0)
    parser.add_argument("--num-grasps", type=int, default=200)
    parser.add_argument("--topk-num-grasps", type=int, default=20)
    parser.add_argument("--min-grasps", type=int, default=40)
    parser.add_argument("--max-tries", type=int, default=6)
    parser.add_argument("--pregrasp-offset-m", type=float, default=0.06)
    parser.add_argument("--max-gripper-width-m", type=float, default=0.06)
    parser.add_argument("--task-constraint-tag", default="pick")
    parser.add_argument("--semantic-score", type=float, default=1.0)
    parser.add_argument("--remove-outliers", action="store_true")
    return parser


def build_rotation_from_axes(
    *,
    closing_direction: np.ndarray,
    approach_direction: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    closing = normalize_vector(closing_direction)
    approach = normalize_vector(approach_direction)
    if not np.any(closing):
        closing = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    if not np.any(approach):
        approach = np.array([0.0, 0.0, 1.0], dtype=np.float32)

    y_axis = normalize_vector(np.cross(approach, closing))
    if not np.any(y_axis):
        fallback_axis = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        if abs(float(np.dot(fallback_axis, approach))) > 0.95:
            fallback_axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        y_axis = normalize_vector(np.cross(approach, fallback_axis))

    closing = normalize_vector(np.cross(y_axis, approach))
    if not np.any(closing):
        closing = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    rotation = np.column_stack(
        [
            closing.astype(np.float64),
            y_axis.astype(np.float64),
            approach.astype(np.float64),
        ]
    )
    return rotation, closing, approach


def main() -> int:
    args = build_parser().parse_args()
    repo_root = Path(args.repo_root).expanduser().resolve()
    cloud_path = Path(args.cloud_npy).expanduser().resolve()
    out_path = Path(args.out_json).expanduser().resolve()

    if not repo_root.is_dir():
        print(f"invalid GraspGen repo root: {repo_root}", file=sys.stderr)
        return 2
    if not cloud_path.is_file():
        print(f"missing input cloud: {cloud_path}", file=sys.stderr)
        return 2

    sys.path.insert(0, str(repo_root))
    try:
        from grasp_gen.serving.zmq_client import GraspGenClient
    except Exception as exc:  # noqa: BLE001
        print(f"failed to import GraspGen client: {exc}", file=sys.stderr)
        return 3

    try:
        points = np.load(cloud_path).astype(np.float32)
    except Exception as exc:  # noqa: BLE001
        print(f"failed to load input cloud: {exc}", file=sys.stderr)
        return 4
    if points.ndim != 2 or points.shape[1] != 3:
        print(f"input cloud must be shaped (N, 3), got {points.shape}", file=sys.stderr)
        return 4

    metadata: dict[str, Any] = {}
    try:
        with GraspGenClient(
            host=args.host,
            port=int(args.port),
            timeout_ms=max(1, int(float(args.timeout_sec) * 1000.0)),
            wait_for_server=False,
        ) as client:
            metadata = client.get_metadata()
            infer_start = time.monotonic()
            grasps, confidences = client.infer(
                points,
                grasp_threshold=float(args.grasp_threshold),
                num_grasps=max(1, int(args.num_grasps)),
                topk_num_grasps=max(1, int(args.topk_num_grasps)),
                min_grasps=max(1, int(args.min_grasps)),
                max_tries=max(1, int(args.max_tries)),
                remove_outliers=bool(args.remove_outliers),
            )
            infer_sec = time.monotonic() - infer_start
    except Exception as exc:  # noqa: BLE001
        print(f"GraspGen inference failed: {exc}", file=sys.stderr)
        return 5

    gripper_config_raw = str(args.gripper_config or metadata.get("gripper_config") or "").strip()
    gripper_name = str(metadata.get("gripper_name") or "").strip()
    if not gripper_config_raw and not gripper_name:
        print("GraspGen metadata did not include a gripper config path", file=sys.stderr)
        return 6
    gripper_config_path = (
        Path(gripper_config_raw).expanduser().resolve()
        if gripper_config_raw
        else None
    )

    try:
        gripper_geometry = load_gripper_geometry(
            repo_root=repo_root,
            gripper_config_path=gripper_config_path,
            gripper_name=gripper_name,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"failed to load gripper geometry: {exc}", file=sys.stderr)
        return 7

    grasps_np = np.asarray(grasps, dtype=np.float32)
    conf_np = np.asarray(confidences, dtype=np.float32).reshape((-1,))
    proposals: list[dict[str, Any]] = []

    for index, grasp_pose in enumerate(grasps_np, start=1):
        if grasp_pose.shape != (4, 4):
            continue
        grasp_contacts = reconstruct_surface_contacts(
            grasp_pose,
            gripper_geometry,
            points,
            max_gripper_width_m=float(args.max_gripper_width_m),
        )
        contact_point_1 = np.asarray(
            grasp_contacts["surface_contact_point_1"], dtype=np.float32
        ).reshape((3,))
        contact_point_2 = np.asarray(
            grasp_contacts["surface_contact_point_2"], dtype=np.float32
        ).reshape((3,))
        grasp_center = np.asarray(
            grasp_contacts["surface_grasp_center"], dtype=np.float32
        ).reshape((3,))
        closing_direction = np.asarray(
            grasp_contacts["closing_direction"], dtype=np.float32
        ).reshape((3,))
        if not np.any(closing_direction):
            closing_direction = normalize_vector(np.asarray(grasp_contacts["rotation"], dtype=np.float32)[:, 0])

        approach_direction = np.asarray(
            grasp_contacts["approach_direction"], dtype=np.float32
        ).reshape((3,))
        if not np.any(approach_direction):
            approach_direction = normalize_vector(np.asarray(grasp_contacts["rotation"], dtype=np.float32)[:, 2])

        orientation_matrix, closing_direction, approach_direction = build_rotation_from_axes(
            closing_direction=closing_direction,
            approach_direction=approach_direction,
        )
        quaternion_xyzw = rotation_matrix_to_quaternion_xyzw(orientation_matrix)
        pregrasp_position = grasp_center - approach_direction * float(args.pregrasp_offset_m)
        grasp_width_m = float(grasp_contacts["effective_grasp_width_m"])
        surface_width_m = float(grasp_contacts["surface_width_m"])

        proposal = {
            "frame_id": str(args.frame_id),
            "contact_point_1": contact_point_1.astype(float).tolist(),
            "contact_point_2": contact_point_2.astype(float).tolist(),
            "grasp_center": grasp_center.astype(float).tolist(),
            "closing_direction": closing_direction.astype(float).tolist(),
            "approach_direction": approach_direction.astype(float).tolist(),
            "grasp_pose_position": grasp_center.astype(float).tolist(),
            "grasp_pose_orientation": quaternion_xyzw,
            "pregrasp_pose_position": pregrasp_position.astype(float).tolist(),
            "pregrasp_pose_orientation": quaternion_xyzw,
            "grasp_width_m": grasp_width_m,
            "surface_width_m": surface_width_m,
            "max_gripper_width_m": float(args.max_gripper_width_m),
            "confidence_score": float(conf_np[index - 1]) if (index - 1) < conf_np.size else 0.0,
            "semantic_score": float(args.semantic_score),
            "candidate_rank": int(index),
            "task_constraint_tag": str(args.task_constraint_tag),
        }
        proposals.append(proposal)

    selected_index = 0
    if proposals and conf_np.size:
        selected_index = int(np.argmax(conf_np[: len(proposals)]))

    payload = {
        "frame_id": str(args.frame_id),
        "selected_index": int(selected_index),
        "proposals": proposals,
            "debug": {
                "backend": "graspgen_zmq",
                "infer_sec": float(infer_sec),
                "proposal_count": len(proposals),
                "gripper_name": str(metadata.get("gripper_name") or ""),
                "model_name": str(metadata.get("model_name") or ""),
                "gripper_config": str(gripper_geometry["path"]),
                "input_points": int(points.shape[0]),
            },
        }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=True, separators=(",", ":"))

    print(f"PROPOSAL_COUNT {len(proposals)}")
    if proposals:
        print(f"SELECTED_INDEX {selected_index}")
        print(f"TOP_CONFIDENCE {float(conf_np[selected_index]) if conf_np.size else 0.0}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
