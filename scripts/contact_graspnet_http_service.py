#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
import math
import multiprocessing as mp
import queue
import sys
import threading
import time
import traceback
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterable, Optional

import cv2
import numpy as np


def decode_png_base64(payload_b64: str, flags: int = cv2.IMREAD_UNCHANGED) -> np.ndarray:
    encoded = base64.b64decode(payload_b64)
    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), flags)
    if image is None:
        raise ValueError("failed to decode PNG payload")
    return image


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def to_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    return default


def normalize_vector(values: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64).reshape((3,))
    norm = np.linalg.norm(vector)
    if norm <= 1e-8:
        return np.asarray(fallback, dtype=np.float64).reshape((3,))
    return vector / norm


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s

    quat = np.asarray([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm <= 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    quat /= norm
    return quat.astype(float).tolist()


def load_npz_object(value: Any) -> Any:
    if isinstance(value, np.ndarray) and value.dtype == object:
        if value.shape == ():
            return value.item()
        if value.size == 1:
            return value.reshape(()).item()
    return value


def confidence_from_score(score: float) -> float:
    if 0.0 <= score <= 1.0:
        return float(score)
    return float(1.0 / (1.0 + math.exp(-score)))


def segment_key_matches(key: Any, preferred_segmap_id: int) -> bool:
    if key == preferred_segmap_id:
        return True
    try:
        return int(round(float(key))) == int(preferred_segmap_id)
    except (TypeError, ValueError):
        normalized = str(key).strip()
        return normalized in {str(preferred_segmap_id), f"{preferred_segmap_id}.0"}


def _prepend_contact_graspnet_paths(contact_graspnet_root: Path) -> None:
    path_candidates = [
        str((contact_graspnet_root / "contact_graspnet").resolve()),
        str((contact_graspnet_root / "pointnet2" / "utils").resolve()),
        str(contact_graspnet_root.resolve()),
    ]
    for candidate in reversed(path_candidates):
        if candidate in sys.path:
            sys.path.remove(candidate)
        sys.path.insert(0, candidate)


def _worker_log_lines(debug: dict[str, Any]) -> list[str]:
    segment_counts = debug.get("segment_point_counts", {})
    segment_summary = ", ".join(
        f"{key}:{value}" for key, value in sorted(segment_counts.items(), key=lambda item: str(item[0]))
    ) or "none"
    return [
        f"full_depth_shape={debug.get('full_depth_shape')}",
        (
            "full_valid_depth_pixels="
            f"{debug.get('full_valid_depth_pixels')} full_mask_pixels={debug.get('full_mask_pixels')} "
            f"full_target_valid_depth_pixels={debug.get('full_target_valid_depth_pixels')} "
            f"full_target_z_valid_depth_pixels={debug.get('full_target_z_valid_depth_pixels')}"
        ),
        (
            "pc_counts="
            f"full={debug.get('pc_full_count')} "
            f"selected_segment={debug.get('selected_segment_point_count')} "
            f"all_segments={segment_summary}"
        ),
        (
            "inference_config="
            f"segmap_id={debug.get('segmap_id')} "
            f"local_regions={debug.get('local_regions')} "
            f"filter_grasps={debug.get('filter_grasps')} "
            f"forward_passes={debug.get('forward_passes')} "
            f"z_range={debug.get('z_range')}"
        ),
        (
            "grasp_output="
            f"grasp_set_keys={debug.get('grasp_set_keys')} "
            f"selected_grasp_count={debug.get('selected_grasp_count')}"
        ),
    ]


def _debug_summary(debug: dict[str, Any]) -> str:
    return (
        f"full_valid_depth_pixels={debug.get('full_valid_depth_pixels')} "
        f"full_target_valid_depth_pixels={debug.get('full_target_valid_depth_pixels')} "
        f"full_target_z_valid_depth_pixels={debug.get('full_target_z_valid_depth_pixels')} "
        f"pc_full_count={debug.get('pc_full_count')} "
        f"selected_segment_point_count={debug.get('selected_segment_point_count')} "
        f"selected_grasp_count={debug.get('selected_grasp_count')}"
    )


def _load_resident_runtime(
    *,
    contact_graspnet_root: Path,
    checkpoint_dir: Path,
    forward_passes: int,
) -> dict[str, Any]:
    _prepend_contact_graspnet_paths(contact_graspnet_root)

    import config_utils
    import tensorflow.compat.v1 as tf
    from contact_grasp_estimator import GraspEstimator
    from visualization_utils import visualize_grasps

    tf.disable_eager_execution()
    try:
        physical_devices = tf.config.experimental.list_physical_devices("GPU")
    except Exception:
        physical_devices = []
    for device in physical_devices:
        try:
            tf.config.experimental.set_memory_growth(device, True)
        except Exception:
            pass

    global_config = config_utils.load_config(
        str(checkpoint_dir),
        batch_size=int(forward_passes),
        arg_configs=[],
    )
    grasp_estimator = GraspEstimator(global_config)
    grasp_estimator.build_network()

    saver = tf.train.Saver(save_relative_paths=True)
    session_config = tf.ConfigProto()
    session_config.gpu_options.allow_growth = True
    session_config.allow_soft_placement = True
    sess = tf.Session(config=session_config)
    grasp_estimator.load_weights(sess, saver, str(checkpoint_dir), mode="test")

    return {
        "tf": tf,
        "sess": sess,
        "grasp_estimator": grasp_estimator,
        "visualize_grasps": visualize_grasps,
        "forward_passes": int(forward_passes),
    }


def _run_resident_inference(
    *,
    runtime: dict[str, Any],
    payload: dict[str, Any],
) -> dict[str, Any]:
    debug: dict[str, Any] = {}
    log_lines: list[str] = []
    try:
        depth_m = np.asarray(payload["depth_m"], dtype=np.float32)
        segmap = np.asarray(payload["segmap"], dtype=np.uint8)
        k_matrix = np.asarray(payload["K"], dtype=np.float32).reshape((3, 3))
        segmap_id = int(payload["segmap_id"])
        local_regions = bool(payload["local_regions"])
        filter_grasps = bool(payload["filter_grasps"])
        skip_border_objects = bool(payload["skip_border_objects"])
        forward_passes = int(payload["forward_passes"])
        visualize = bool(payload["visualize"])
        z_range_raw = payload["z_range"]
        z_range = (
            float(min(z_range_raw[0], z_range_raw[1])),
            float(max(z_range_raw[0], z_range_raw[1])),
        )
        if forward_passes != int(runtime["forward_passes"]):
            raise ValueError(
                "resident Contact-GraspNet worker was initialized for "
                f"forward_passes={runtime['forward_passes']} but request asked for {forward_passes}"
            )

        valid_depth_mask = np.isfinite(depth_m) & (depth_m > 0.0)
        full_mask = segmap > 0
        target_valid_depth_mask = valid_depth_mask & full_mask
        target_z_valid_depth_mask = target_valid_depth_mask & (depth_m >= z_range[0]) & (depth_m <= z_range[1])
        debug.update(
            {
                "full_depth_shape": [int(depth_m.shape[0]), int(depth_m.shape[1])],
                "full_valid_depth_pixels": int(np.count_nonzero(valid_depth_mask)),
                "full_mask_pixels": int(np.count_nonzero(full_mask)),
                "full_target_valid_depth_pixels": int(np.count_nonzero(target_valid_depth_mask)),
                "full_target_z_valid_depth_pixels": int(np.count_nonzero(target_z_valid_depth_mask)),
                "segmap_id": segmap_id,
                "local_regions": local_regions,
                "filter_grasps": filter_grasps,
                "forward_passes": forward_passes,
                "z_range": [float(z_range[0]), float(z_range[1])],
            }
        )
        print(
            "[contact_graspnet_http_service] worker preflight: "
            f"shape={debug['full_depth_shape'][1]}x{debug['full_depth_shape'][0]} "
            f"full_valid_depth_pixels={debug['full_valid_depth_pixels']} "
            f"full_mask_pixels={debug['full_mask_pixels']} "
            f"full_target_valid_depth_pixels={debug['full_target_valid_depth_pixels']} "
            f"full_target_z_valid_depth_pixels={debug['full_target_z_valid_depth_pixels']} "
            f"z_range={debug['z_range']}"
        , flush=True)

        grasp_estimator = runtime["grasp_estimator"]
        sess = runtime["sess"]
        pc_full, pc_segments, _ = grasp_estimator.extract_point_clouds(
            depth_m,
            k_matrix,
            segmap=segmap,
            rgb=None,
            z_range=list(z_range),
            segmap_id=segmap_id,
            skip_border_objects=skip_border_objects,
        )

        segment_counts = {
            str(int(round(float(key)))): int(np.asarray(value).shape[0])
            for key, value in pc_segments.items()
        }
        selected_segment = np.asarray(pc_segments.get(segmap_id, np.empty((0, 3), dtype=np.float32)))
        debug.update(
            {
                "pc_full_count": int(np.asarray(pc_full).shape[0]),
                "segment_point_counts": segment_counts,
                "selected_segment_point_count": int(selected_segment.shape[0]),
            }
        )
        print(
            "[contact_graspnet_http_service] worker point_clouds: "
            f"pc_full_count={debug['pc_full_count']} "
            f"selected_segment_point_count={debug['selected_segment_point_count']} "
            f"segment_point_counts={segment_counts}"
        , flush=True)

        if int(debug["pc_full_count"]) <= 0:
            raise ValueError("empty pc_full after extract_point_clouds")
        if int(debug["selected_segment_point_count"]) <= 0:
            raise ValueError("empty target segment after extract_point_clouds")

        pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(
            sess,
            pc_full,
            pc_segments=pc_segments,
            local_regions=local_regions,
            filter_grasps=filter_grasps,
            forward_passes=forward_passes,
        )
        if visualize:
            runtime["visualize_grasps"](
                pc_full,
                pred_grasps_cam,
                scores,
                plot_opencv_cam=True,
                pc_colors=None,
            )

        if isinstance(pred_grasps_cam, dict):
            grasp_set_keys = [str(key) for key in pred_grasps_cam.keys()]
            selected_grasp_count = int(
                np.asarray(next(iter(pred_grasps_cam.values()), np.empty((0, 4, 4)))).shape[0]
            )
            for key, value in pred_grasps_cam.items():
                if segment_key_matches(key, segmap_id):
                    selected_grasp_count = int(np.asarray(value).shape[0])
                    break
        else:
            grasp_set_keys = [str(segmap_id)]
            selected_grasp_count = int(np.asarray(pred_grasps_cam).shape[0])

        debug.update(
            {
                "grasp_set_keys": grasp_set_keys,
                "selected_grasp_count": selected_grasp_count,
            }
        )
        print(
            "[contact_graspnet_http_service] worker grasps: "
            f"grasp_set_keys={grasp_set_keys} selected_grasp_count={selected_grasp_count}"
        , flush=True)
        log_lines = _worker_log_lines(debug)
        return {
            "ok": True,
            "pred_grasps_cam": pred_grasps_cam,
            "scores": scores,
            "contact_pts": contact_pts,
            "debug": debug,
            "log_lines": log_lines,
            "traceback": "",
        }
    except Exception as exc:  # noqa: BLE001
        log_lines = _worker_log_lines(debug) if debug else []
        return {
            "ok": False,
            "error": str(exc),
            "debug": debug,
            "log_lines": log_lines,
            "traceback": traceback.format_exc(),
        }


def _resident_worker_main(
    request_queue: Any,
    response_queue: Any,
    *,
    contact_graspnet_root: str,
    checkpoint_dir: str,
    default_forward_passes: int,
) -> None:
    runtime: Optional[dict[str, Any]] = None
    try:
        runtime = _load_resident_runtime(
            contact_graspnet_root=Path(contact_graspnet_root),
            checkpoint_dir=Path(checkpoint_dir),
            forward_passes=int(default_forward_passes),
        )
        response_queue.put(
            {
                "type": "ready",
                "forward_passes": int(default_forward_passes),
            }
        )
        while True:
            message = request_queue.get()
            if message is None:
                break
            request_id = str(message.get("request_id") or "")
            payload = dict(message.get("payload") or {})
            result = _run_resident_inference(runtime=runtime, payload=payload)
            response_queue.put(
                {
                    "type": "result",
                    "request_id": request_id,
                    "result": result,
                }
            )
    except Exception as exc:  # noqa: BLE001
        response_queue.put(
            {
                "type": "startup_error",
                "error": str(exc),
                "traceback": traceback.format_exc(),
            }
        )
    finally:
        if runtime is not None:
            sess = runtime.get("sess")
            if sess is not None:
                try:
                    sess.close()
                except Exception:
                    pass


class ContactGraspNetService:
    def __init__(
        self,
        *,
        contact_graspnet_root: Path,
        checkpoint_dir: Path,
        python_bin: str,
        default_pregrasp_offset_m: float,
        default_max_proposals: int,
        default_z_range: tuple[float, float],
        default_forward_passes: int,
        default_visualize: bool,
        inference_timeout_sec: float,
        log_dir: Optional[Path],
    ) -> None:
        self.contact_graspnet_root = contact_graspnet_root
        self.checkpoint_dir = checkpoint_dir
        self.python_bin = python_bin
        self.default_pregrasp_offset_m = default_pregrasp_offset_m
        self.default_max_proposals = default_max_proposals
        self.default_z_range = default_z_range
        self.default_forward_passes = default_forward_passes
        self.default_visualize = default_visualize
        self.inference_timeout_sec = max(10.0, float(inference_timeout_sec))
        self.log_dir = log_dir
        self._request_lock = threading.Lock()
        self._active_request_id: Optional[str] = None
        self._worker_lock = threading.Lock()
        self._mp_ctx = mp.get_context("spawn")
        self._worker_process: Optional[mp.Process] = None
        self._worker_request_queue: Any = None
        self._worker_response_queue: Any = None
        if self.log_dir is not None:
            self.log_dir.mkdir(parents=True, exist_ok=True)
        self.inference_module_dir = contact_graspnet_root / "contact_graspnet"
        if not (self.inference_module_dir / "contact_grasp_estimator.py").is_file():
            raise FileNotFoundError(
                f"contact_grasp_estimator.py not found at {self.inference_module_dir / 'contact_grasp_estimator.py'}"
            )
        self._start_worker()

    def try_begin_request(self, request_id: str) -> tuple[bool, str]:
        if not self._request_lock.acquire(blocking=False):
            return False, str(self._active_request_id or "")
        self._active_request_id = request_id
        return True, ""

    def finish_request(self, request_id: str) -> None:
        if self._active_request_id == request_id:
            self._active_request_id = None
        if self._request_lock.locked():
            self._request_lock.release()

    def worker_alive(self) -> bool:
        return self._worker_process is not None and self._worker_process.is_alive()

    def _start_worker(self) -> None:
        with self._worker_lock:
            if self._worker_process is not None and self._worker_process.is_alive():
                return
            self._worker_request_queue = self._mp_ctx.Queue(maxsize=1)
            self._worker_response_queue = self._mp_ctx.Queue(maxsize=1)
            self._worker_process = self._mp_ctx.Process(
                target=_resident_worker_main,
                kwargs={
                    "request_queue": self._worker_request_queue,
                    "response_queue": self._worker_response_queue,
                    "contact_graspnet_root": str(self.contact_graspnet_root),
                    "checkpoint_dir": str(self.checkpoint_dir),
                    "default_forward_passes": int(self.default_forward_passes),
                },
                daemon=True,
            )
            print(
                "[contact_graspnet_http_service] worker boot: "
                f"ckpt={self.checkpoint_dir} forward_passes={self.default_forward_passes}"
            )
            self._worker_process.start()
            self._await_worker_ready()

    def _await_worker_ready(self) -> None:
        deadline = time.time() + max(120.0, self.inference_timeout_sec)
        while time.time() < deadline:
            process = self._worker_process
            if process is None:
                break
            if not process.is_alive():
                break
            try:
                message = self._worker_response_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if message.get("type") == "ready":
                print(
                    "[contact_graspnet_http_service] worker ready: "
                    f"pid={process.pid} forward_passes={message.get('forward_passes')}"
                )
                return
            if message.get("type") == "startup_error":
                raise RuntimeError(
                    "Contact-GraspNet worker failed to start: "
                    f"{message.get('error')}\n{message.get('traceback', '').strip()}"
                )
        raise TimeoutError("Contact-GraspNet worker did not become ready in time")

    def _stop_worker(self) -> None:
        with self._worker_lock:
            process = self._worker_process
            request_queue = self._worker_request_queue
            if request_queue is not None:
                try:
                    request_queue.put_nowait(None)
                except Exception:
                    pass
            if process is not None and process.is_alive():
                process.terminate()
                process.join(timeout=5.0)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=2.0)
            self._worker_process = None
            self._worker_request_queue = None
            self._worker_response_queue = None

    def _restart_worker(self, reason: str) -> None:
        print(f"[contact_graspnet_http_service] worker restart: reason={reason}")
        self._stop_worker()
        self._start_worker()

    def run(self, payload: dict[str, Any], *, request_id: str) -> dict[str, Any]:
        started = time.time()
        depth_png_b64 = str(payload.get("depth_png_b64") or "").strip()
        segmap_png_b64 = str(payload.get("segmap_png_b64") or "").strip()
        if not depth_png_b64 or not segmap_png_b64:
            raise ValueError("depth_png_b64 and segmap_png_b64 are required")

        depth_scale = float(payload.get("depth_scale_m_per_unit", 0.001) or 0.001)
        depth_raw = decode_png_base64(depth_png_b64, cv2.IMREAD_UNCHANGED)
        segmap_raw = decode_png_base64(segmap_png_b64, cv2.IMREAD_UNCHANGED)
        if depth_raw.ndim != 2:
            raise ValueError("depth image must be single-channel")
        if segmap_raw.ndim == 3:
            segmap_raw = segmap_raw[:, :, 0]
        if segmap_raw.shape[:2] != depth_raw.shape[:2]:
            raise ValueError("depth image and segmap shape mismatch")

        depth_m = depth_raw.astype(np.float32) * depth_scale
        segmap = np.asarray(segmap_raw, dtype=np.uint8)

        k_values = payload.get("K")
        if not isinstance(k_values, list) or len(k_values) != 9:
            raise ValueError("K must be a 9-element list")
        k_matrix = np.asarray(k_values, dtype=np.float32).reshape((3, 3))

        segmap_id = int(payload.get("segmap_id", 1) or 1)
        local_regions = bool(payload.get("local_regions", True))
        filter_grasps = bool(payload.get("filter_grasps", True))
        skip_border_objects = bool(payload.get("skip_border_objects", False))
        forward_passes = max(1, int(payload.get("forward_passes", self.default_forward_passes)))
        visualize = to_bool(payload.get("visualize"), self.default_visualize)
        z_range_raw = payload.get("z_range", list(self.default_z_range))
        if not isinstance(z_range_raw, list) or len(z_range_raw) < 2:
            z_range_raw = list(self.default_z_range)
        z_range = (float(z_range_raw[0]), float(z_range_raw[1]))
        pregrasp_offset_m = max(
            0.01,
            float(payload.get("pregrasp_offset_m", self.default_pregrasp_offset_m)),
        )
        requested_max_proposals = int(payload.get("max_proposals", self.default_max_proposals))
        max_proposals = max(0, requested_max_proposals)
        task_constraint_tag = str(payload.get("task_constraint_tag") or "pick")
        frame_id = str(payload.get("frame_id") or "")
        print(
            "[contact_graspnet_http_service] request start: "
            f"id={request_id} frame_id={frame_id or 'n/a'} segmap_id={segmap_id} "
            f"depth_shape={depth_raw.shape[1]}x{depth_raw.shape[0]} "
            f"mask_pixels={int(np.count_nonzero(segmap))} visualize={visualize}"
        )
        worker_payload = {
            "depth_m": depth_m.astype(np.float32),
            "segmap": np.where(segmap > 0, segmap_id, 0).astype(np.uint8),
            "K": k_matrix.astype(np.float32),
            "segmap_id": segmap_id,
            "local_regions": local_regions,
            "filter_grasps": filter_grasps,
            "skip_border_objects": skip_border_objects,
            "forward_passes": forward_passes,
            "z_range": [float(z_range[0]), float(z_range[1])],
            "visualize": visualize,
        }
        stdout_path = self._request_log_path(request_id, "stdout.log")
        stderr_path = self._request_log_path(request_id, "stderr.log")
        try:
            result = self._run_worker_request(
                request_id=request_id,
                payload=worker_payload,
            )
        except Exception as exc:
            self._write_request_log(
                stdout_path,
                "\n".join(
                    [
                        f"request_id={request_id}",
                        f"depth_shape={depth_raw.shape[1]}x{depth_raw.shape[0]}",
                        f"mask_pixels={int(np.count_nonzero(segmap))}",
                        f"segmap_id={segmap_id}",
                        f"local_regions={local_regions}",
                        f"filter_grasps={filter_grasps}",
                        f"forward_passes={forward_passes}",
                        f"z_range={[float(z_range[0]), float(z_range[1])]}",
                    ]
                ),
            )
            self._write_request_log(stderr_path, f"{type(exc).__name__}: {exc}")
            raise
        self._write_request_log(stdout_path, "\n".join(result.get("log_lines", [])))
        self._write_request_log(stderr_path, str(result.get("traceback", "")))
        if not bool(result.get("ok")):
            raise RuntimeError(
                f"{result.get('error', 'resident Contact-GraspNet inference failed')} | "
                f"{_debug_summary(dict(result.get('debug') or {}))}"
            )

        pred_grasps_cam = result.get("pred_grasps_cam")
        scores = result.get("scores")
        contact_pts = result.get("contact_pts")
        proposals = self._build_proposals(
            pred_grasps_cam=pred_grasps_cam,
            scores=scores,
            contact_pts=contact_pts,
            preferred_segmap_id=segmap_id,
            pregrasp_offset_m=pregrasp_offset_m,
            max_proposals=max_proposals,
            task_constraint_tag=task_constraint_tag,
            frame_id=frame_id,
        )
        source_segment_ids = sorted(
            {int(item["source_segment_id"]) for item in proposals if "source_segment_id" in item}
        )
        worker_debug = dict(result.get("debug") or {})
        print(
            "[contact_graspnet_http_service] inference exit: "
            f"id={request_id} proposals={len(proposals)} elapsed={time.time() - started:.2f}s "
            f"{_debug_summary(worker_debug)}"
        )

        return {
            "frame_id": frame_id,
            "proposals": proposals,
            "debug": {
                "proposal_count": len(proposals),
                "selected_segment_ids": source_segment_ids,
                "selected_object_only": True,
                "local_regions": local_regions,
                "filter_grasps": filter_grasps,
                "forward_passes": forward_passes,
                "segmap_pixels": int(np.count_nonzero(segmap)),
                "segmap_id": segmap_id,
                "visualize": visualize,
                "request_id": request_id,
                "elapsed_sec": round(time.time() - started, 3),
                **worker_debug,
            },
        }

    def _run_worker_request(self, *, request_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        self._start_worker()
        request_queue = self._worker_request_queue
        response_queue = self._worker_response_queue
        process = self._worker_process
        if request_queue is None or response_queue is None or process is None:
            raise RuntimeError("Contact-GraspNet worker is not available")
        print(
            "[contact_graspnet_http_service] inference dispatch: "
            f"id={request_id} worker_pid={process.pid} timeout={self.inference_timeout_sec:.1f}s"
        )
        request_queue.put({"request_id": request_id, "payload": payload})
        deadline = time.time() + self.inference_timeout_sec
        while time.time() < deadline:
            if not process.is_alive():
                self._restart_worker("worker died during inference")
                raise RuntimeError("Contact-GraspNet worker died during inference")
            remaining = deadline - time.time()
            try:
                message = response_queue.get(timeout=min(1.0, max(0.05, remaining)))
            except queue.Empty:
                continue
            if message.get("type") != "result":
                continue
            if str(message.get("request_id") or "") != request_id:
                continue
            return dict(message.get("result") or {})
        self._restart_worker(f"inference timeout for request {request_id}")
        raise TimeoutError(f"resident Contact-GraspNet inference timed out after {self.inference_timeout_sec:.1f}s")

    def _request_log_path(self, request_id: str, suffix: str) -> Optional[Path]:
        if self.log_dir is None:
            return None
        return self.log_dir / f"{request_id}.{suffix}"

    def _write_request_log(self, path: Optional[Path], content: str) -> None:
        if path is None:
            return
        path.write_text(content or "", encoding="utf-8")

    def _build_proposals(
        self,
        *,
        pred_grasps_cam: Any,
        scores: Any,
        contact_pts: Any,
        preferred_segmap_id: int,
        pregrasp_offset_m: float,
        max_proposals: int,
        task_constraint_tag: str,
        frame_id: str,
    ) -> list[dict[str, Any]]:
        candidates: list[dict[str, Any]] = []
        for key, grasps, score_values, contact_values in self._iterate_grasp_sets(
            pred_grasps_cam=pred_grasps_cam,
            scores=scores,
            contact_pts=contact_pts,
            preferred_segmap_id=preferred_segmap_id,
        ):
            grasp_array = np.asarray(grasps, dtype=np.float64)
            if grasp_array.ndim == 2 and grasp_array.shape == (4, 4):
                grasp_array = grasp_array[np.newaxis, ...]
            score_array = np.asarray(score_values, dtype=np.float64).reshape((-1,))
            contact_array = None
            if contact_values is not None:
                contact_array = np.asarray(contact_values, dtype=np.float64)
                if contact_array.ndim == 1 and contact_array.shape[0] == 3:
                    contact_array = contact_array[np.newaxis, ...]

            count = min(grasp_array.shape[0], score_array.shape[0])
            for idx in range(count):
                transform = np.asarray(grasp_array[idx], dtype=np.float64).reshape((4, 4))
                score = float(score_array[idx])
                contact_point = None
                if contact_array is not None and idx < int(contact_array.shape[0]):
                    candidate_contact = np.asarray(contact_array[idx], dtype=np.float64).reshape((-1,))
                    if candidate_contact.size >= 3:
                        contact_point = candidate_contact[:3]

                proposal = self._normalize_grasp(
                    transform=transform,
                    score=score,
                    contact_point=contact_point,
                    pregrasp_offset_m=pregrasp_offset_m,
                    task_constraint_tag=task_constraint_tag,
                    frame_id=frame_id,
                    source_segment_id=key,
                )
                if proposal is not None:
                    candidates.append(proposal)

        candidates.sort(key=lambda item: float(item["confidence_score"]), reverse=True)
        trimmed = candidates if max_proposals <= 0 else candidates[:max_proposals]
        for rank, item in enumerate(trimmed, start=1):
            item["candidate_rank"] = rank
        return trimmed

    def _iterate_grasp_sets(
        self,
        *,
        pred_grasps_cam: Any,
        scores: Any,
        contact_pts: Any,
        preferred_segmap_id: int,
    ) -> Iterable[tuple[int, Any, Any, Any]]:
        pred_grasps_cam = load_npz_object(pred_grasps_cam)
        scores = load_npz_object(scores)
        contact_pts = load_npz_object(contact_pts)

        if isinstance(pred_grasps_cam, dict):
            keys = list(pred_grasps_cam.keys())
            preferred_keys = [key for key in keys if segment_key_matches(key, preferred_segmap_id)]
            if not preferred_keys and len(keys) == 1:
                preferred_keys = list(keys)
            for key in preferred_keys:
                key_scores = scores.get(key, []) if isinstance(scores, dict) else scores
                key_contacts = contact_pts.get(key, None) if isinstance(contact_pts, dict) else contact_pts
                try:
                    normalized_key = int(round(float(key)))
                except (TypeError, ValueError):
                    normalized_key = preferred_segmap_id
                yield normalized_key, pred_grasps_cam[key], key_scores, key_contacts
            return

        yield preferred_segmap_id, pred_grasps_cam, scores, contact_pts

    def _normalize_grasp(
        self,
        *,
        transform: np.ndarray,
        score: float,
        contact_point: Optional[np.ndarray],
        pregrasp_offset_m: float,
        task_constraint_tag: str,
        frame_id: str,
        source_segment_id: int,
    ) -> Optional[dict[str, Any]]:
        if transform.shape != (4, 4):
            return None

        rotation = transform[:3, :3]
        position = transform[:3, 3]
        closing_direction = normalize_vector(rotation[:, 0], np.array([1.0, 0.0, 0.0]))
        approach_direction = normalize_vector(rotation[:, 2], np.array([0.0, 0.0, 1.0]))

        default_half_width = 0.025
        if contact_point is not None:
            contact_point = np.asarray(contact_point, dtype=np.float64).reshape((3,))
            projected_half_width = abs(float(np.dot(contact_point - position, closing_direction)))
            half_width = max(0.005, projected_half_width)
        else:
            half_width = default_half_width

        contact_point_1 = position - closing_direction * half_width
        contact_point_2 = position + closing_direction * half_width
        grasp_center = 0.5 * (contact_point_1 + contact_point_2)
        pregrasp_position = position - approach_direction * pregrasp_offset_m
        quaternion_xyzw = rotation_matrix_to_quaternion_xyzw(rotation)
        confidence_score = clamp(confidence_from_score(score), 0.0, 1.0)

        return {
            "frame_id": frame_id,
            "contact_point_1": contact_point_1.astype(float).tolist(),
            "contact_point_2": contact_point_2.astype(float).tolist(),
            "grasp_center": grasp_center.astype(float).tolist(),
            "closing_direction": closing_direction.astype(float).tolist(),
            "approach_direction": approach_direction.astype(float).tolist(),
            "grasp_pose_position": position.astype(float).tolist(),
            "grasp_pose_orientation": quaternion_xyzw,
            "pregrasp_pose_position": pregrasp_position.astype(float).tolist(),
            "pregrasp_pose_orientation": quaternion_xyzw,
            "grasp_width_m": float(2.0 * half_width),
            "confidence_score": confidence_score,
            "semantic_score": 1.0,
            "task_constraint_tag": task_constraint_tag,
            "source_segment_id": int(source_segment_id),
            "raw_score": float(score),
        }


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "ContactGraspNetHTTP/0.1"

    def do_GET(self) -> None:  # noqa: N802
        if self.path.rstrip("/") != "/health":
            self.send_error(HTTPStatus.NOT_FOUND, "unknown path")
            return
        healthy = bool(self.server.service.worker_alive())  # type: ignore[attr-defined]
        status = HTTPStatus.OK if healthy else HTTPStatus.SERVICE_UNAVAILABLE
        body = json.dumps({"status": "ok" if healthy else "worker_unavailable"}, ensure_ascii=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:  # noqa: N802
        if self.path.rstrip("/") != "/infer":
            self.send_error(HTTPStatus.NOT_FOUND, "unknown path")
            return

        started = time.time()
        request_id = f"cg_{int(started * 1000)}_{uuid.uuid4().hex[:8]}"
        acquired = False
        try:
            content_length = int(self.headers.get("Content-Length", "0") or 0)
            raw_body = self.rfile.read(content_length)
            acquired, active_request_id = self.server.service.try_begin_request(request_id)  # type: ignore[attr-defined]
            if not acquired:
                body = json.dumps(
                    {
                        "status": "busy",
                        "reason": "Contact-GraspNet sidecar is busy",
                        "active_request_id": active_request_id,
                    },
                    ensure_ascii=True,
                ).encode("utf-8")
                self.send_response(HTTPStatus.SERVICE_UNAVAILABLE)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                print(
                    "[contact_graspnet_http_service] infer busy: "
                    f"id={request_id} active_request_id={active_request_id}"
                )
                return
            payload = json.loads(raw_body.decode("utf-8"))
            print(
                "[contact_graspnet_http_service] infer received: "
                f"id={request_id} bytes={content_length} keys={sorted(payload.keys())}"
            )
            response_payload = self.server.service.run(payload, request_id=request_id)  # type: ignore[attr-defined]
            response_body = json.dumps(response_payload, ensure_ascii=True).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response_body)))
            self.end_headers()
            self.wfile.write(response_body)
            print(
                "[contact_graspnet_http_service] infer ok: "
                f"id={request_id} segmap_id={payload.get('segmap_id')} "
                f"proposals={len(response_payload.get('proposals', []))} "
                f"elapsed={time.time() - started:.2f}s"
            )
        except Exception as exc:  # noqa: BLE001
            print(
                "[contact_graspnet_http_service] infer failed: "
                f"id={request_id} elapsed={time.time() - started:.2f}s error={exc}"
            )
            traceback.print_exc()
            body = json.dumps(
                {"status": "error", "reason": str(exc)},
                ensure_ascii=True,
            ).encode("utf-8")
            self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        finally:
            if acquired:
                self.server.service.finish_request(request_id)  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        print(fmt % args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HTTP sidecar for Contact-GraspNet inference")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--python-bin", default="python")
    parser.add_argument("--contact-graspnet-root", required=True)
    parser.add_argument("--ckpt-dir", required=True)
    parser.add_argument("--default-pregrasp-offset-m", type=float, default=0.06)
    parser.add_argument("--default-max-proposals", type=int, default=16)
    parser.add_argument("--default-z-min", type=float, default=0.2)
    parser.add_argument("--default-z-max", type=float, default=1.8)
    parser.add_argument("--default-forward-passes", type=int, default=1)
    parser.add_argument("--default-visualize", action="store_true")
    parser.add_argument("--inference-timeout-sec", type=float, default=10.0)
    parser.add_argument("--log-dir", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    service = ContactGraspNetService(
        contact_graspnet_root=Path(args.contact_graspnet_root).resolve(),
        checkpoint_dir=Path(args.ckpt_dir).resolve(),
        python_bin=args.python_bin,
        default_pregrasp_offset_m=float(args.default_pregrasp_offset_m),
        default_max_proposals=max(0, int(args.default_max_proposals)),
        default_z_range=(float(args.default_z_min), float(args.default_z_max)),
        default_forward_passes=max(1, int(args.default_forward_passes)),
        default_visualize=bool(args.default_visualize),
        inference_timeout_sec=float(args.inference_timeout_sec),
        log_dir=Path(args.log_dir).resolve() if str(args.log_dir).strip() else None,
    )
    server = ThreadingHTTPServer((args.host, int(args.port)), RequestHandler)
    server.service = service  # type: ignore[attr-defined]
    print(
        f"contact_graspnet_http_service listening on http://{args.host}:{args.port}/infer "
        f"(health=http://{args.host}:{args.port}/health, root={service.contact_graspnet_root}, "
        f"ckpt={service.checkpoint_dir}, timeout={service.inference_timeout_sec:.1f}s, "
        f"log_dir={service.log_dir or 'disabled'})"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service._stop_worker()


if __name__ == "__main__":
    main()
