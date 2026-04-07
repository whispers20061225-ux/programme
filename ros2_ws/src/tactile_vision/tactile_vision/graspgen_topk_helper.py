from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from graspgen_geometry import load_gripper_geometry, reconstruct_surface_contacts


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Query a GraspGen ZMQ server and dump top-k grasp geometry to .npz",
    )
    parser.add_argument("--repo-root", required=True, help="Path to the GraspGen repo root")
    parser.add_argument("--cloud-npy", required=True, help="Input target cloud .npy path")
    parser.add_argument("--out-npy", required=True, help="Output top-k geometry .npz path")
    parser.add_argument("--host", default="127.0.0.1", help="GraspGen ZMQ server host")
    parser.add_argument("--port", type=int, default=5556, help="GraspGen ZMQ server port")
    parser.add_argument("--timeout-sec", type=float, default=12.0, help="Server request timeout")
    parser.add_argument("--grasp-threshold", type=float, default=-1.0)
    parser.add_argument("--num-grasps", type=int, default=200)
    parser.add_argument("--topk-num-grasps", type=int, default=50)
    parser.add_argument("--min-grasps", type=int, default=40)
    parser.add_argument("--max-tries", type=int, default=6)
    parser.add_argument("--max-gripper-width-m", type=float, default=0.06)
    parser.add_argument(
        "--remove-outliers",
        action="store_true",
        help="Let the GraspGen server remove point-cloud outliers before inference",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).expanduser().resolve()
    cloud_path = Path(args.cloud_npy).expanduser().resolve()
    out_path = Path(args.out_npy).expanduser().resolve()

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

    metadata: dict[str, object] = {}
    try:
        with GraspGenClient(
            host=args.host,
            port=int(args.port),
            timeout_ms=max(1, int(float(args.timeout_sec) * 1000.0)),
            wait_for_server=False,
        ) as client:
            metadata = client.get_metadata()
            grasps, confidences = client.infer(
                points,
                grasp_threshold=float(args.grasp_threshold),
                num_grasps=max(1, int(args.num_grasps)),
                topk_num_grasps=max(1, int(args.topk_num_grasps)),
                min_grasps=max(1, int(args.min_grasps)),
                max_tries=max(1, int(args.max_tries)),
                remove_outliers=bool(args.remove_outliers),
            )
    except Exception as exc:  # noqa: BLE001
        print(f"GraspGen inference failed: {exc}", file=sys.stderr)
        return 5

    gripper_config_raw = str(metadata.get("gripper_config") or "").strip()
    gripper_name = str(metadata.get("gripper_name") or "").strip()
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
        return 6

    grasp_array = np.asarray(grasps, dtype=np.float32)
    score_array = np.asarray(confidences, dtype=np.float32).reshape((-1,))
    if grasp_array.ndim == 3 and grasp_array.shape[1:] == (4, 4):
        contact_point_1 = []
        contact_point_2 = []
        centers = []
        for grasp_pose in grasp_array:
            contacts = reconstruct_surface_contacts(
                grasp_pose,
                gripper_geometry,
                points,
                max_gripper_width_m=float(args.max_gripper_width_m),
            )
            contact_point_1.append(contacts["surface_contact_point_1"])
            contact_point_2.append(contacts["surface_contact_point_2"])
            centers.append(contacts["surface_grasp_center"])
        centers = np.asarray(centers, dtype=np.float32).reshape((-1, 3))
        contact_point_1 = np.asarray(contact_point_1, dtype=np.float32).reshape((-1, 3))
        contact_point_2 = np.asarray(contact_point_2, dtype=np.float32).reshape((-1, 3))
    else:
        centers = np.empty((0, 3), dtype=np.float32)
        contact_point_1 = np.empty((0, 3), dtype=np.float32)
        contact_point_2 = np.empty((0, 3), dtype=np.float32)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        out_path,
        centers=centers.reshape((-1, 3)),
        contact_point_1=contact_point_1.reshape((-1, 3)),
        contact_point_2=contact_point_2.reshape((-1, 3)),
        confidences=score_array.reshape((-1,)),
    )

    print(f"TOPK_COUNT {int(centers.shape[0])}")
    if centers.size > 0:
        print(f"TOP1_CENTER {centers[0].tolist()}")
        print(f"TOP1_CONFIDENCE {float(score_array[0]) if score_array.size else 0.0}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
