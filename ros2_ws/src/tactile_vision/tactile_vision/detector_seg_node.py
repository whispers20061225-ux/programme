from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import Bool, String
from tactile_interfaces.msg import DetectionResult, SemanticTask

from tactile_vision.modular_common import (
    coerce_bbox,
    coerce_point,
    compact_json,
    decode_color_image,
    decode_png_base64_mask,
    encode_image_to_base64_jpeg,
    extract_first_json_object,
    extract_message_text,
    make_mono8_image,
    make_rgb8_image,
)

COLOR_HINT_TOKENS = {
    "black",
    "blue",
    "brown",
    "gray",
    "green",
    "grey",
    "orange",
    "pink",
    "purple",
    "red",
    "white",
    "yellow",
}

SPATIAL_DIRECTION_TOKENS = {
    "left": {"left", "leftmost", "on the left", "\u5de6", "\u5de6\u8fb9", "\u5de6\u4fa7"},
    "right": {"right", "rightmost", "on the right", "\u53f3", "\u53f3\u8fb9", "\u53f3\u4fa7"},
    "center": {"center", "middle", "in the middle", "\u4e2d\u95f4", "\u4e2d\u592e"},
}

SEMANTIC_ALIAS_LABELS = {
    "container": {"bottle", "can", "cup", "vase", "wine glass"},
    "cylinder": {"bottle", "can", "cup", "vase", "wine glass"},
    "cylindrical": {"bottle", "can", "cup", "vase", "wine glass"},
    "drinkware": {"bottle", "can", "cup", "wine glass"},
    "tube": {"bottle", "can", "cup", "vase"},
}

DEFAULT_VLM_AMBIGUOUS_LABELS = {
    "remote",
    "cup",
    "bottle",
    "can",
    "vase",
    "wine glass",
}

APPEARANCE_HINT_TOKENS = {
    "block",
    "cylinder",
    "cylindrical",
    "handle",
    "paper",
    "plastic",
    "rectangular",
    "round",
    "silver",
    "tool",
    "tube",
}

GENERIC_VLM_LABELS = {
    "container",
    "cup",
    "bottle",
    "can",
    "object",
    "item",
    "thing",
    "tool",
    "vase",
    "wine glass",
}


class DetectorSegNode(Node):
    def __init__(self) -> None:
        super().__init__("detector_seg_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("detection_result_topic", "/perception/detection_result")
        self.declare_parameter(
            "candidate_visible_topic", "/sim/perception/target_candidate_visible"
        )
        self.declare_parameter("detection_debug_topic", "/perception/detection_debug")
        self.declare_parameter(
            "detection_debug_overlay_topic", "/perception/detection_debug_overlay"
        )
        self.declare_parameter("backend", "ultralytics_local")
        self.declare_parameter("backend_url", "")
        self.declare_parameter("request_timeout_sec", 10.0)
        self.declare_parameter("max_inference_rate_hz", 2.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("ultralytics_model_path", "yolo11s-seg.pt")
        self.declare_parameter("ultralytics_runtime_preference", "auto")
        self.declare_parameter("ultralytics_device", "")
        self.declare_parameter("ultralytics_imgsz", 960)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.55)
        self.declare_parameter("candidate_complete_edge_margin_px", 16)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("publish_debug_overlay", True)
        self.declare_parameter("debug_top_k_candidates", 5)
        self.declare_parameter("debug_candidate_confidence_floor", 0.05)
        self.declare_parameter("semantic_match_confidence_floor", 0.05)
        self.declare_parameter("vlm_relabel_enabled", True)
        self.declare_parameter("vlm_relabel_endpoint", "http://127.0.0.1:8000/v1/chat/completions")
        self.declare_parameter("vlm_relabel_model_name", "Qwen/Qwen2.5-VL-3B-Instruct-AWQ")
        self.declare_parameter("vlm_relabel_api_key", "EMPTY")
        self.declare_parameter("vlm_relabel_request_timeout_sec", 20.0)
        self.declare_parameter("vlm_relabel_top_k", 3)
        self.declare_parameter("vlm_relabel_min_confidence", 0.05)
        self.declare_parameter("vlm_relabel_crop_margin_px", 12)
        self.declare_parameter("vlm_relabel_resize_max_side_px", 384)
        self.declare_parameter("vlm_relabel_jpeg_quality", 88)
        self.declare_parameter("vlm_relabel_cache_ttl_sec", 30.0)
        self.declare_parameter("vlm_relabel_min_interval_sec", 2.5)
        self.declare_parameter("vlm_relabel_track_cooldown_sec", 5.0)
        self.declare_parameter("vlm_relabel_low_confidence_threshold", 0.35)
        self.declare_parameter(
            "vlm_relabel_ambiguous_labels",
            sorted(DEFAULT_VLM_AMBIGUOUS_LABELS),
        )
        self.declare_parameter("track_iou_threshold", 0.2)
        self.declare_parameter("track_center_distance_px", 120.0)
        self.declare_parameter("track_stale_sec", 10.0)
        self.declare_parameter("track_label_hold_sec", 2.0)
        self.declare_parameter("track_stable_frames_required", 2)
        self.declare_parameter("log_interval_sec", 10.0)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.candidate_visible_topic = str(self.get_parameter("candidate_visible_topic").value)
        self.detection_debug_topic = str(self.get_parameter("detection_debug_topic").value)
        self.detection_debug_overlay_topic = str(
            self.get_parameter("detection_debug_overlay_topic").value
        )
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.backend_url = str(self.get_parameter("backend_url").value).strip()
        self.request_timeout_sec = max(
            0.5, float(self.get_parameter("request_timeout_sec").value)
        )
        self.max_inference_rate_hz = max(
            0.2, float(self.get_parameter("max_inference_rate_hz").value)
        )
        self.enabled = bool(self.get_parameter("enabled").value)
        self.ultralytics_model_path = str(
            self.get_parameter("ultralytics_model_path").value
        ).strip()
        self.ultralytics_runtime_preference = str(
            self.get_parameter("ultralytics_runtime_preference").value
        ).strip().lower()
        self.ultralytics_device = str(self.get_parameter("ultralytics_device").value).strip()
        self.ultralytics_imgsz = max(320, int(self.get_parameter("ultralytics_imgsz").value))
        self.confidence_threshold = max(
            0.0, min(1.0, float(self.get_parameter("confidence_threshold").value))
        )
        self.iou_threshold = max(
            0.05, min(0.95, float(self.get_parameter("iou_threshold").value))
        )
        self.candidate_complete_edge_margin_px = max(
            0, int(self.get_parameter("candidate_complete_edge_margin_px").value)
        )
        self.jpeg_quality = max(50, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.publish_debug_overlay = bool(self.get_parameter("publish_debug_overlay").value)
        self.debug_top_k_candidates = max(
            1, int(self.get_parameter("debug_top_k_candidates").value)
        )
        self.debug_candidate_confidence_floor = max(
            0.0,
            min(1.0, float(self.get_parameter("debug_candidate_confidence_floor").value)),
        )
        self.semantic_match_confidence_floor = max(
            0.0,
            min(1.0, float(self.get_parameter("semantic_match_confidence_floor").value)),
        )
        self.vlm_relabel_enabled = bool(self.get_parameter("vlm_relabel_enabled").value)
        self.vlm_relabel_endpoint = str(
            self.get_parameter("vlm_relabel_endpoint").value or ""
        ).strip()
        self.vlm_relabel_model_name = str(
            self.get_parameter("vlm_relabel_model_name").value or ""
        ).strip()
        self.vlm_relabel_api_key = str(
            self.get_parameter("vlm_relabel_api_key").value or ""
        ).strip()
        self.vlm_relabel_request_timeout_sec = max(
            2.0, float(self.get_parameter("vlm_relabel_request_timeout_sec").value)
        )
        self.vlm_relabel_top_k = max(
            1, int(self.get_parameter("vlm_relabel_top_k").value)
        )
        self.vlm_relabel_min_confidence = max(
            0.0,
            min(1.0, float(self.get_parameter("vlm_relabel_min_confidence").value)),
        )
        self.vlm_relabel_crop_margin_px = max(
            0, int(self.get_parameter("vlm_relabel_crop_margin_px").value)
        )
        self.vlm_relabel_resize_max_side_px = max(
            128, int(self.get_parameter("vlm_relabel_resize_max_side_px").value)
        )
        self.vlm_relabel_jpeg_quality = max(
            50, min(100, int(self.get_parameter("vlm_relabel_jpeg_quality").value))
        )
        self.vlm_relabel_cache_ttl_sec = max(
            2.0, float(self.get_parameter("vlm_relabel_cache_ttl_sec").value)
        )
        self.vlm_relabel_min_interval_sec = max(
            0.0, float(self.get_parameter("vlm_relabel_min_interval_sec").value)
        )
        self.vlm_relabel_track_cooldown_sec = max(
            0.0, float(self.get_parameter("vlm_relabel_track_cooldown_sec").value)
        )
        self.vlm_relabel_low_confidence_threshold = max(
            0.0,
            min(1.0, float(self.get_parameter("vlm_relabel_low_confidence_threshold").value)),
        )
        self.vlm_relabel_ambiguous_labels = {
            str(item or "").strip().lower()
            for item in list(self.get_parameter("vlm_relabel_ambiguous_labels").value or [])
            if str(item or "").strip()
        }
        if not self.vlm_relabel_ambiguous_labels:
            self.vlm_relabel_ambiguous_labels = set(DEFAULT_VLM_AMBIGUOUS_LABELS)
        self.track_iou_threshold = max(
            0.0, min(1.0, float(self.get_parameter("track_iou_threshold").value))
        )
        self.track_center_distance_px = max(
            8.0, float(self.get_parameter("track_center_distance_px").value)
        )
        self.track_stale_sec = max(
            0.5, float(self.get_parameter("track_stale_sec").value)
        )
        self.track_label_hold_sec = max(
            0.5, float(self.get_parameter("track_label_hold_sec").value)
        )
        self.track_stable_frames_required = max(
            1, int(self.get_parameter("track_stable_frames_required").value)
        )
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._session = requests.Session()
        self._vlm_session = requests.Session()
        self._request_lock = threading.Lock()
        self._semantic_lock = threading.Lock()
        self._color_lock = threading.Lock()
        self._vlm_relabel_lock = threading.Lock()
        self._track_lock = threading.Lock()
        self._request_in_flight = False
        self._pending_request = False
        self._vlm_relabel_in_flight = False
        self._vlm_relabel_last_started = 0.0
        self._tracks: dict[int, dict[str, Any]] = {}
        self._next_track_id = 1
        self._latest_color_msg: Optional[Image] = None
        self._semantic_task: Optional[SemanticTask] = None
        self._last_terminal_summary = ""
        self._last_candidate_summary = ""
        self._model = None
        self._resolved_ultralytics_model_path = ""
        self._ultralytics_runtime = "torch"
        self._effective_ultralytics_imgsz = 0

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(Image, self.color_topic, self._on_color_image, qos_sensor)
        self.create_subscription(
            SemanticTask, self.semantic_task_topic, self._on_semantic_task, qos_reliable
        )

        self.detection_pub = self.create_publisher(
            DetectionResult, self.detection_result_topic, qos_reliable
        )
        self.candidate_visible_pub = self.create_publisher(
            Bool, self.candidate_visible_topic, qos_reliable
        )
        self.debug_pub = self.create_publisher(String, self.detection_debug_topic, qos_reliable)
        self.debug_overlay_pub = None
        if self.publish_debug_overlay:
            self.debug_overlay_pub = self.create_publisher(
                Image,
                self.detection_debug_overlay_topic,
                qos_reliable,
            )
        if self.backend in ("http_json", "ultralytics_local"):
            self.create_timer(1.0 / self.max_inference_rate_hz, self._maybe_run_inference)
        if self.backend == "ultralytics_local":
            self._prepare_ultralytics_runtime_env()
            self._log_ultralytics_runtime_status()

        self.get_logger().info(
            "detector_seg_node started: "
            f"backend={self.backend}, result={self.detection_result_topic}, "
            f"candidate_visible={self.candidate_visible_topic}, "
            f"vlm_relabel={'on' if self.vlm_relabel_enabled else 'off'}"
        )

    def _log_ultralytics_runtime_status(self) -> None:
        try:
            import ultralytics  # noqa: F401
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "detector backend is ultralytics_local, but ultralytics is unavailable: "
                f"{exc}. Install it before launching YOLO11-seg."
            )
            return

        self.get_logger().info(
            "ultralytics_local backend ready: "
            f"model={self.ultralytics_model_path}, device={self.ultralytics_device or 'auto'}, "
            f"runtime_pref={self.ultralytics_runtime_preference or 'auto'}"
        )

    def _prepare_ultralytics_runtime_env(self) -> None:
        os.environ.setdefault("ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS", "1")
        if not self._should_prefer_onnx_runtime():
            return
        for key in (
            "CUDA_VISIBLE_DEVICES",
            "HIP_VISIBLE_DEVICES",
            "ROCR_VISIBLE_DEVICES",
            "MESA_D3D12_DEFAULT_ADAPTER_NAME",
        ):
            os.environ.pop(key, None)

    def _should_prefer_onnx_runtime(self) -> bool:
        if self.ultralytics_runtime_preference == "onnx":
            return True
        if self.ultralytics_runtime_preference == "torch":
            return False
        device = self.ultralytics_device.strip().lower()
        return device in {"", "cpu"}

    def _resolve_ultralytics_model_path(self) -> tuple[str, str]:
        model_path = Path(self.ultralytics_model_path).expanduser()
        if not model_path.is_absolute():
            model_path = (Path.cwd() / model_path).resolve()
        else:
            model_path = model_path.resolve()

        if not model_path.exists():
            raise FileNotFoundError(f"ultralytics model not found: {model_path}")

        if model_path.suffix.lower() == ".onnx":
            return model_path.as_posix(), "onnx"
        if model_path.suffix.lower() != ".pt" or not self._should_prefer_onnx_runtime():
            return model_path.as_posix(), "torch"

        onnx_path = model_path.with_suffix(".onnx")
        if onnx_path.exists():
            return onnx_path.as_posix(), "onnx"

        try:
            import onnx  # noqa: F401
            import onnxruntime  # noqa: F401
            from ultralytics import YOLO
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "ONNX runtime fallback unavailable; staying on PyTorch inference: "
                f"{exc}"
            )
            return model_path.as_posix(), "torch"

        self.get_logger().info(
            "exporting ONNX segmentation model for stable CPU inference: "
            f"{model_path.name} -> {onnx_path.name}"
        )
        exported_path = YOLO(model_path.as_posix()).export(
            format="onnx",
            imgsz=self.ultralytics_imgsz,
            simplify=False,
        )
        resolved_export = Path(str(exported_path)).expanduser().resolve()
        return resolved_export.as_posix(), "onnx"

    def _resolve_onnx_input_imgsz(self, model_path: str) -> int:
        try:
            import onnxruntime as ort
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"failed to inspect ONNX input size for {model_path}: {exc}"
            )
            return int(self.ultralytics_imgsz)

        try:
            session = ort.InferenceSession(
                model_path,
                providers=["CPUExecutionProvider"],
            )
            inputs = session.get_inputs()
            if not inputs:
                return int(self.ultralytics_imgsz)
            shape = list(inputs[0].shape)
            if len(shape) < 4:
                return int(self.ultralytics_imgsz)
            height = shape[2]
            width = shape[3]
            if isinstance(height, int) and isinstance(width, int) and height == width and height > 0:
                return int(height)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"failed to read ONNX model metadata from {model_path}: {exc}"
            )
        return int(self.ultralytics_imgsz)

    def _on_color_image(self, msg: Image) -> None:
        with self._color_lock:
            self._latest_color_msg = msg
        if self.backend in ("http_json", "ultralytics_local"):
            self._pending_request = True

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        with self._semantic_lock:
            self._semantic_task = msg
        if self.backend == "disabled":
            self._publish_detection(
                self._build_empty_result(
                    task=msg.task,
                    target_label=msg.target_label,
                    reason="detector backend disabled; configure YOLO11 segmentation backend",
                )
            )
        else:
            self._pending_request = True

    def _maybe_run_inference(self) -> None:
        if not self.enabled or self.backend not in ("http_json", "ultralytics_local"):
            return
        if not self._pending_request:
            return

        with self._request_lock:
            if self._request_in_flight:
                return
            self._request_in_flight = True
            self._pending_request = False

        with self._color_lock:
            color_msg = self._latest_color_msg
        with self._semantic_lock:
            semantic_task = self._semantic_task

        self._run_inference(color_msg, semantic_task)

    def _run_inference(
        self,
        color_msg: Optional[Image],
        semantic_task: Optional[SemanticTask],
    ) -> None:
        try:
            if color_msg is None:
                raise ValueError("missing color image for detector inference")
            image_rgb = decode_color_image(color_msg)
            if image_rgb is None:
                raise ValueError(f"unsupported color image encoding: {color_msg.encoding}")
            image_rgb = np.array(image_rgb, dtype=np.uint8, copy=True, order="C")

            if self.backend == "http_json":
                payload = self._run_http_inference(color_msg, image_rgb, semantic_task)
            elif self.backend == "ultralytics_local":
                payload = self._run_ultralytics_inference(color_msg, image_rgb, semantic_task)
            else:
                payload = self._build_empty_result(
                    task=semantic_task.task if semantic_task is not None else "pick",
                    target_label=semantic_task.target_label if semantic_task is not None else "",
                    reason=f"unsupported detector backend: {self.backend}",
                    image_width=int(color_msg.width),
                    image_height=int(color_msg.height),
                    frame_id=color_msg.header.frame_id,
                )

            self._publish_detection(payload)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"detector inference failed: {exc}")
            self._publish_detection(
                self._build_empty_result(
                    task=semantic_task.task if semantic_task is not None else "pick",
                    target_label=semantic_task.target_label if semantic_task is not None else "",
                    reason=str(exc),
                    image_width=int(color_msg.width) if color_msg is not None else 0,
                    image_height=int(color_msg.height) if color_msg is not None else 0,
                    frame_id=color_msg.header.frame_id if color_msg is not None else "",
                )
            )
        finally:
            with self._request_lock:
                self._request_in_flight = False

    def _run_http_inference(
        self,
        color_msg: Image,
        image_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask],
    ) -> dict[str, Any]:
        if not self.backend_url:
            raise ValueError("detector backend_url is empty")

        image_b64 = encode_image_to_base64_jpeg(image_rgb, self.jpeg_quality)
        semantic_payload = self._semantic_payload(semantic_task)
        response = self._session.post(
            self.backend_url,
            json={
                "image_jpeg_b64": image_b64,
                "semantic_task": semantic_payload,
            },
            timeout=self.request_timeout_sec,
        )
        response.raise_for_status()
        parsed = response.json()
        if not isinstance(parsed, dict):
            raise ValueError("detector response is not a JSON object")

        bbox_xyxy = coerce_bbox(parsed.get("bbox_xyxy", parsed.get("bbox")), int(color_msg.width), int(color_msg.height))
        point_px = coerce_point(parsed.get("point_px", parsed.get("point")), int(color_msg.width), int(color_msg.height))
        if point_px is None and bbox_xyxy is not None:
            point_px = self._bbox_center_point(bbox_xyxy)

        mask = None
        mask_png_b64 = parsed.get("mask_png_b64")
        if isinstance(mask_png_b64, str) and mask_png_b64.strip():
            mask = decode_png_base64_mask(mask_png_b64)
            if mask is not None and mask.shape[:2] != image_rgb.shape[:2]:
                mask = cv2.resize(
                    mask,
                    (image_rgb.shape[1], image_rgb.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )

        return self._build_detection_payload(
            backend="http_json",
            task=str(parsed.get("task") or semantic_payload["task"]),
            target_label=str(
                parsed.get("target_label")
                or parsed.get("target")
                or parsed.get("label")
                or semantic_payload["target_label"]
                or semantic_payload["target_hint"]
            ).strip(),
            confidence=float(parsed.get("confidence", parsed.get("score", 0.0)) or 0.0),
            need_human_confirm=bool(parsed.get("need_human_confirm", False)),
            reason=str(parsed.get("reason") or ""),
            image_width=int(color_msg.width),
            image_height=int(color_msg.height),
            frame_id=color_msg.header.frame_id,
            bbox_xyxy=bbox_xyxy,
            point_px=point_px,
            mask_u8=mask,
        )

    def _ensure_ultralytics_model(self):
        if self._model is not None:
            return self._model
        try:
            from ultralytics import YOLO
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "ultralytics is not installed; install it before using backend=ultralytics_local"
            ) from exc

        self._resolved_ultralytics_model_path, self._ultralytics_runtime = (
            self._resolve_ultralytics_model_path()
        )
        self._effective_ultralytics_imgsz = int(self.ultralytics_imgsz)
        if self._ultralytics_runtime == "onnx":
            self._effective_ultralytics_imgsz = self._resolve_onnx_input_imgsz(
                self._resolved_ultralytics_model_path
            )
        self._model = YOLO(self._resolved_ultralytics_model_path)
        self.get_logger().info(
            "ultralytics model loaded: "
            f"runtime={self._ultralytics_runtime}, "
            f"model={self._resolved_ultralytics_model_path}, "
            f"imgsz={self._effective_ultralytics_imgsz}"
        )
        return self._model

    def _run_ultralytics_inference(
        self,
        color_msg: Image,
        image_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask],
    ) -> dict[str, Any]:
        model = self._ensure_ultralytics_model()
        prediction_kwargs: dict[str, Any] = {
            "source": image_rgb,
            "imgsz": self._effective_ultralytics_imgsz or self.ultralytics_imgsz,
            "conf": min(
                self.confidence_threshold,
                self.debug_candidate_confidence_floor,
                self.semantic_match_confidence_floor,
            ),
            "iou": self.iou_threshold,
            "verbose": False,
        }
        if self._ultralytics_runtime != "onnx" and self.ultralytics_device:
            prediction_kwargs["device"] = self.ultralytics_device
        prediction = model.predict(**prediction_kwargs)
        if not prediction:
            with self._track_lock:
                self._reap_stale_tracks(time.time())
            return self._build_empty_result(
                task=semantic_task.task if semantic_task is not None else "pick",
                target_label=semantic_task.target_label if semantic_task is not None else "",
                reason="no detections returned by YOLO11-seg",
                image_width=int(color_msg.width),
                image_height=int(color_msg.height),
                frame_id=color_msg.header.frame_id,
            )

        result = prediction[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            with self._track_lock:
                self._reap_stale_tracks(time.time())
            return self._build_empty_result(
                task=semantic_task.task if semantic_task is not None else "pick",
                target_label=semantic_task.target_label if semantic_task is not None else "",
                reason="YOLO11-seg found no candidate instances",
                image_width=int(color_msg.width),
                image_height=int(color_msg.height),
                frame_id=color_msg.header.frame_id,
            )

        names = getattr(result, "names", {})
        masks_data = None
        masks_obj = getattr(result, "masks", None)
        if masks_obj is not None and getattr(masks_obj, "data", None) is not None:
            masks_data = masks_obj.data
            if hasattr(masks_data, "cpu"):
                masks_data = masks_data.cpu().numpy()
            else:
                masks_data = np.asarray(masks_data)

        candidates: list[dict[str, Any]] = []
        raw_candidates: list[dict[str, Any]] = []
        for idx in range(len(boxes)):
            cls_tensor = boxes.cls[idx]
            conf_tensor = boxes.conf[idx]
            cls_id = int(cls_tensor.item() if hasattr(cls_tensor, "item") else cls_tensor)
            confidence = float(
                conf_tensor.item() if hasattr(conf_tensor, "item") else conf_tensor
            )
            label = (
                str(names.get(cls_id, cls_id))
                if isinstance(names, dict)
                else str(names[cls_id])
            )
            bbox_xyxy = self._extract_bbox_xyxy(
                boxes,
                idx,
                int(color_msg.width),
                int(color_msg.height),
            )
            mask_u8 = self._extract_mask_u8(masks_data, idx, image_rgb.shape[:2])
            mask_pixels = (
                int(np.count_nonzero(mask_u8)) if isinstance(mask_u8, np.ndarray) else 0
            )

            candidate = {
                "index": int(idx),
                "class_id": int(cls_id),
                "label": label,
                "raw_label": label,
                "canonical_label": label,
                "label_zh": "",
                "display_label": label,
                "label_source": "yolo",
                "confidence": float(confidence),
                "semantic_bonus": 0.0,
                "score": float(confidence),
                "confidence_floor": float(self.confidence_threshold),
                "bbox_xyxy": bbox_xyxy,
                "mask_u8": mask_u8,
                "mask_pixels": int(mask_pixels),
                "image_width": int(color_msg.width),
                "image_height": int(color_msg.height),
                "status": "selectable",
            }
            raw_candidates.append(candidate)

        self._associate_candidates_to_tracks(raw_candidates)
        self._schedule_candidate_relabels(image_rgb, raw_candidates, semantic_task)

        for candidate in raw_candidates:
            semantic_bonus = self._candidate_semantic_bonus(candidate, semantic_task)
            score = float(candidate.get("confidence", 0.0) or 0.0) + float(semantic_bonus)
            confidence_floor = self._candidate_confidence_floor(semantic_bonus, semantic_task)
            status = "selectable"
            if float(candidate.get("confidence", 0.0) or 0.0) < confidence_floor:
                status = "filtered_low_confidence"
            elif semantic_bonus <= -1.0:
                status = "filtered_excluded_label"
            candidate["semantic_bonus"] = float(semantic_bonus)
            candidate["score"] = float(score)
            candidate["confidence_floor"] = float(confidence_floor)
            candidate["status"] = status
            if status == "selectable":
                candidates.append(candidate)

        if not candidates:
            top_candidates = self._select_debug_candidates(raw_candidates, None)
            payload = self._build_empty_result(
                task=semantic_task.task if semantic_task is not None else "pick",
                target_label=semantic_task.target_label if semantic_task is not None else "",
                reason="YOLO11-seg found detections, but none passed selection filters",
                image_width=int(color_msg.width),
                image_height=int(color_msg.height),
                frame_id=color_msg.header.frame_id,
            )
            payload["semantic_target"] = self._semantic_payload(semantic_task)
            payload["debug_candidates"] = [
                self._candidate_debug_public(candidate) for candidate in top_candidates
            ]
            payload["selected_candidate"] = None
            payload["candidate_summary"] = self._format_candidate_summary(
                semantic_task,
                top_candidates,
                None,
            )
            if self.publish_debug_overlay:
                payload["debug_overlay_rgb"] = self._render_debug_overlay(
                    image_rgb,
                    top_candidates,
                    None,
                    semantic_task,
                )
            return payload

        if self._semantic_task_requires_match(semantic_task):
            semantically_relevant = [
                item for item in candidates if float(item["semantic_bonus"]) > 0.0
            ]
            if semantically_relevant:
                candidates = semantically_relevant
            else:
                top_candidates = self._select_debug_candidates(raw_candidates, None)
                payload = self._build_empty_result(
                    task=semantic_task.task if semantic_task is not None else "pick",
                    target_label=semantic_task.target_label if semantic_task is not None else "",
                    reason="YOLO11-seg found detections, but none matched the semantic target",
                    image_width=int(color_msg.width),
                    image_height=int(color_msg.height),
                    frame_id=color_msg.header.frame_id,
                )
                payload["semantic_target"] = self._semantic_payload(semantic_task)
                payload["debug_candidates"] = [
                    self._candidate_debug_public(candidate) for candidate in top_candidates
                ]
                payload["selected_candidate"] = None
                payload["candidate_summary"] = self._format_candidate_summary(
                    semantic_task,
                    top_candidates,
                    None,
                )
                if self.publish_debug_overlay:
                    payload["debug_overlay_rgb"] = self._render_debug_overlay(
                        image_rgb,
                        top_candidates,
                        None,
                        semantic_task,
                    )
                return payload

        selected_candidate = max(candidates, key=lambda item: float(item["score"]))
        top_candidates = self._select_debug_candidates(raw_candidates, selected_candidate)
        payload = self._build_detection_payload(
            backend=(
                f"ultralytics_{self._ultralytics_runtime}:"
                f"{Path(self._resolved_ultralytics_model_path or self.ultralytics_model_path).name}"
            ),
            task=semantic_task.task if semantic_task is not None else "pick",
            target_label=self._candidate_preferred_label(selected_candidate),
            confidence=float(selected_candidate["confidence"]),
            need_human_confirm=False,
            reason="",
            image_width=int(color_msg.width),
            image_height=int(color_msg.height),
            frame_id=color_msg.header.frame_id,
            bbox_xyxy=selected_candidate["bbox_xyxy"],
            point_px=self._bbox_center_point(selected_candidate["bbox_xyxy"]),
            mask_u8=selected_candidate["mask_u8"],
        )
        payload["semantic_target"] = self._semantic_payload(semantic_task)
        payload["debug_candidates"] = [
            self._candidate_debug_public(candidate) for candidate in top_candidates
        ]
        payload["selected_candidate"] = self._candidate_debug_public(selected_candidate)
        payload["candidate_summary"] = self._format_candidate_summary(
            semantic_task,
            top_candidates,
            selected_candidate,
        )
        if self.publish_debug_overlay:
            payload["debug_overlay_rgb"] = self._render_debug_overlay(
                image_rgb,
                top_candidates,
                selected_candidate,
                semantic_task,
            )
        return payload

    def _normalize_short_label(self, value: Any, *, max_chars: int = 48) -> str:
        text = str(value or "").replace("\n", " ").replace("\r", " ").strip()
        if not text:
            return ""
        while "  " in text:
            text = text.replace("  ", " ")
        text = text.strip(" \t,.;:!?/|")
        return text[:max_chars].strip()

    def _compose_candidate_display_label(
        self,
        *,
        canonical_label: str,
        label_zh: str,
        raw_label: str,
    ) -> str:
        canonical = self._normalize_short_label(canonical_label)
        zh = self._normalize_short_label(label_zh)
        raw = self._normalize_short_label(raw_label)
        if zh and canonical and zh.lower() != canonical.lower():
            return f"{zh} / {canonical}"
        if zh:
            return zh
        if canonical:
            return canonical
        return raw

    def _candidate_preferred_label(self, candidate: Optional[dict[str, Any]]) -> str:
        if not isinstance(candidate, dict):
            return ""
        return self._normalize_short_label(
            candidate.get("canonical_label")
            or candidate.get("label_zh")
            or candidate.get("label")
            or candidate.get("raw_label")
            or ""
        )

    def _candidate_preferred_display_label(self, candidate: Optional[dict[str, Any]]) -> str:
        if not isinstance(candidate, dict):
            return ""
        return self._normalize_short_label(
            candidate.get("display_label")
            or candidate.get("label_zh")
            or candidate.get("canonical_label")
            or candidate.get("label")
            or candidate.get("raw_label")
            or ""
        )

    def _candidate_has_vlm_label(self, candidate: Optional[dict[str, Any]]) -> bool:
        if not isinstance(candidate, dict):
            return False
        source = str(candidate.get("label_source", "") or "").strip().lower()
        return source in {"vlm", "vlm_track"}

    def _candidate_semantic_texts(self, candidate: dict[str, Any]) -> list[str]:
        seen: set[str] = set()
        texts: list[str] = []
        keys = ["canonical_label", "label_zh", "display_label"]
        if not self._candidate_has_vlm_label(candidate):
            keys.extend(["label", "raw_label"])
        for key in keys:
            text = self._normalize_short_label(candidate.get(key, ""))
            lowered = text.lower()
            if not lowered or lowered in seen:
                continue
            seen.add(lowered)
            texts.append(text)
        return texts

    def _candidate_semantic_bonus(
        self,
        candidate: dict[str, Any],
        semantic_task: Optional[SemanticTask],
    ) -> float:
        texts = self._candidate_semantic_texts(candidate)
        if not texts:
            return 0.0
        text_bonus = max(
            self._semantic_match_bonus(text, semantic_task)
            for text in texts
        )
        if text_bonus <= -1.0:
            return text_bonus
        return text_bonus + self._candidate_spatial_bonus(candidate, semantic_task)

    def _normalized_label_key(self, value: Any) -> str:
        return self._normalize_short_label(value, max_chars=96).lower()

    def _semantic_signature(self, semantic_task: Optional[SemanticTask]) -> str:
        if semantic_task is None:
            return ""
        task = str(semantic_task.task or "").strip().lower()
        label = str(semantic_task.target_label or "").strip().lower()
        hint = str(semantic_task.target_hint or "").strip().lower()
        excluded = ",".join(
            sorted(str(item or "").strip().lower() for item in semantic_task.excluded_labels if str(item or "").strip())
        )
        return "|".join([task, label, hint, excluded]).strip("|")

    def _bbox_center(self, bbox_xyxy: Optional[list[int]]) -> tuple[float, float]:
        if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
            return (0.0, 0.0)
        return (
            (float(bbox_xyxy[0]) + float(bbox_xyxy[2])) * 0.5,
            (float(bbox_xyxy[1]) + float(bbox_xyxy[3])) * 0.5,
        )

    def _center_distance_px(
        self,
        left_center: tuple[float, float],
        right_center: tuple[float, float],
    ) -> float:
        return float(
            np.hypot(
                float(left_center[0]) - float(right_center[0]),
                float(left_center[1]) - float(right_center[1]),
            )
        )

    def _bbox_iou(
        self,
        left_bbox_xyxy: Optional[list[int]],
        right_bbox_xyxy: Optional[list[int]],
    ) -> float:
        if (
            not isinstance(left_bbox_xyxy, list)
            or len(left_bbox_xyxy) != 4
            or not isinstance(right_bbox_xyxy, list)
            or len(right_bbox_xyxy) != 4
        ):
            return 0.0
        left_x1, left_y1, left_x2, left_y2 = [float(value) for value in left_bbox_xyxy]
        right_x1, right_y1, right_x2, right_y2 = [float(value) for value in right_bbox_xyxy]
        inter_x1 = max(left_x1, right_x1)
        inter_y1 = max(left_y1, right_y1)
        inter_x2 = min(left_x2, right_x2)
        inter_y2 = min(left_y2, right_y2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area <= 0.0:
            return 0.0
        left_area = max(0.0, left_x2 - left_x1) * max(0.0, left_y2 - left_y1)
        right_area = max(0.0, right_x2 - right_x1) * max(0.0, right_y2 - right_y1)
        union = left_area + right_area - inter_area
        if union <= 0.0:
            return 0.0
        return inter_area / union

    def _make_track_state(
        self,
        candidate: dict[str, Any],
        now_sec: float,
    ) -> dict[str, Any]:
        track_id = int(self._next_track_id)
        self._next_track_id += 1
        raw_label = self._normalize_short_label(candidate.get("raw_label") or candidate.get("label"))
        center_xy = self._bbox_center(candidate.get("bbox_xyxy"))
        return {
            "track_id": track_id,
            "created_at": now_sec,
            "first_seen": now_sec,
            "last_seen": now_sec,
            "seen_count": 0,
            "consecutive_hits": 0,
            "bbox_xyxy": list(candidate.get("bbox_xyxy") or []),
            "center_xy": center_xy,
            "class_id": int(candidate.get("class_id", -1)),
            "raw_label": raw_label,
            "canonical_label": raw_label,
            "label_zh": "",
            "display_label": raw_label,
            "label_source": "yolo",
            "confidence": float(candidate.get("confidence", 0.0) or 0.0),
            "mask_pixels": int(candidate.get("mask_pixels", 0) or 0),
            "relabel_in_flight": False,
            "last_relabel_requested_at": 0.0,
            "last_relabel_completed_at": 0.0,
            "last_relabel_reason": "",
            "last_semantic_signature": "",
        }

    def _update_track_from_candidate(
        self,
        track: dict[str, Any],
        candidate: dict[str, Any],
        now_sec: float,
    ) -> None:
        raw_label = self._normalize_short_label(candidate.get("raw_label") or candidate.get("label"))
        track["last_seen"] = now_sec
        track["seen_count"] = int(track.get("seen_count", 0)) + 1
        track["consecutive_hits"] = int(track.get("consecutive_hits", 0)) + 1
        track["bbox_xyxy"] = list(candidate.get("bbox_xyxy") or [])
        track["center_xy"] = self._bbox_center(candidate.get("bbox_xyxy"))
        track["class_id"] = int(candidate.get("class_id", -1))
        track["raw_label"] = raw_label
        track["confidence"] = float(candidate.get("confidence", 0.0) or 0.0)
        track["mask_pixels"] = int(candidate.get("mask_pixels", 0) or 0)
        if str(track.get("label_source", "yolo") or "yolo") != "vlm":
            track["canonical_label"] = raw_label
            track["display_label"] = raw_label
            track["label_zh"] = ""
            track["label_source"] = "yolo"

    def _reap_stale_tracks(self, now_sec: float) -> None:
        stale_ids = [
            track_id
            for track_id, track in self._tracks.items()
            if (now_sec - float(track.get("last_seen", 0.0) or 0.0)) > self.track_stale_sec
        ]
        for track_id in stale_ids:
            self._tracks.pop(track_id, None)

    def _track_match_score(
        self,
        track: dict[str, Any],
        candidate: dict[str, Any],
    ) -> Optional[float]:
        candidate_bbox = candidate.get("bbox_xyxy")
        track_bbox = track.get("bbox_xyxy")
        if (
            not isinstance(candidate_bbox, list)
            or len(candidate_bbox) != 4
            or not isinstance(track_bbox, list)
            or len(track_bbox) != 4
        ):
            return None
        candidate_center = self._bbox_center(candidate_bbox)
        track_center = track.get("center_xy") or self._bbox_center(track_bbox)
        iou = self._bbox_iou(track_bbox, candidate_bbox)
        distance_px = self._center_distance_px(track_center, candidate_center)
        if iou < self.track_iou_threshold and distance_px > self.track_center_distance_px:
            return None
        score = iou * 2.5 + max(
            0.0,
            1.0 - (distance_px / max(1.0, self.track_center_distance_px)),
        )
        raw_label = self._normalized_label_key(candidate.get("raw_label") or candidate.get("label"))
        if raw_label and raw_label == self._normalized_label_key(track.get("raw_label")):
            score += 0.15
        if int(candidate.get("class_id", -1)) == int(track.get("class_id", -2)):
            score += 0.1
        return score

    def _track_has_vlm_label(self, track: dict[str, Any]) -> bool:
        return bool(
            str(track.get("canonical_label", "") or "").strip()
            and str(track.get("label_source", "") or "").strip() == "vlm"
        )

    def _track_has_recent_vlm_label(self, track: dict[str, Any], now_sec: float) -> bool:
        if not self._track_has_vlm_label(track):
            return False
        updated_at = float(track.get("last_relabel_completed_at", 0.0) or 0.0)
        if updated_at <= 0.0:
            return False
        return (now_sec - updated_at) <= self.track_label_hold_sec

    def _apply_track_state_to_candidate(
        self,
        candidate: dict[str, Any],
        track: dict[str, Any],
        *,
        is_new_track: bool,
        now_sec: float,
    ) -> None:
        candidate["track_id"] = int(track.get("track_id", -1))
        candidate["track_new"] = bool(is_new_track)
        candidate["track_seen_count"] = int(track.get("seen_count", 0) or 0)
        candidate["track_consecutive_hits"] = int(track.get("consecutive_hits", 0) or 0)
        candidate["track_age_sec"] = max(
            0.0,
            now_sec - float(track.get("first_seen", now_sec) or now_sec),
        )
        candidate["track_last_seen_sec"] = float(track.get("last_seen", now_sec) or now_sec)
        label_age_sec = 0.0
        if float(track.get("last_relabel_completed_at", 0.0) or 0.0) > 0.0:
            label_age_sec = max(0.0, now_sec - float(track["last_relabel_completed_at"]))
        candidate["track_label_age_sec"] = label_age_sec
        candidate["track_relabel_reason"] = str(track.get("last_relabel_reason", "") or "")

        if not self._track_has_vlm_label(track):
            return

        candidate["canonical_label"] = self._normalize_short_label(track.get("canonical_label"))
        candidate["label_zh"] = self._normalize_short_label(track.get("label_zh"))
        candidate["display_label"] = self._normalize_short_label(
            track.get("display_label")
            or self._compose_candidate_display_label(
                canonical_label=candidate.get("canonical_label", ""),
                label_zh=candidate.get("label_zh", ""),
                raw_label=candidate.get("raw_label", ""),
            )
        )
        candidate["label_source"] = "vlm" if self._track_has_recent_vlm_label(track, now_sec) else "vlm_track"

    def _associate_candidates_to_tracks(
        self,
        raw_candidates: list[dict[str, Any]],
    ) -> None:
        if not raw_candidates:
            return
        now_sec = time.time()
        ranked_candidates = sorted(
            raw_candidates,
            key=lambda item: float(item.get("confidence", 0.0) or 0.0),
            reverse=True,
        )
        with self._track_lock:
            self._reap_stale_tracks(now_sec)
            assigned_track_ids: set[int] = set()
            for candidate in ranked_candidates:
                best_track: Optional[dict[str, Any]] = None
                best_score = -1.0
                for track in self._tracks.values():
                    track_id = int(track.get("track_id", -1))
                    if track_id in assigned_track_ids:
                        continue
                    score = self._track_match_score(track, candidate)
                    if score is None or score <= best_score:
                        continue
                    best_score = score
                    best_track = track
                is_new_track = best_track is None
                if best_track is None:
                    best_track = self._make_track_state(candidate, now_sec)
                    self._tracks[int(best_track["track_id"])] = best_track
                self._update_track_from_candidate(best_track, candidate, now_sec)
                assigned_track_ids.add(int(best_track["track_id"]))
                self._apply_track_state_to_candidate(
                    candidate,
                    best_track,
                    is_new_track=is_new_track,
                    now_sec=now_sec,
                )

            for track in self._tracks.values():
                if int(track.get("track_id", -1)) not in assigned_track_ids:
                    track["consecutive_hits"] = 0
            self._reap_stale_tracks(now_sec)

    def _track_semantic_texts(
        self,
        track: dict[str, Any],
        candidate: Optional[dict[str, Any]] = None,
    ) -> list[str]:
        seen: set[str] = set()
        texts: list[str] = []
        for source in (track, candidate or {}):
            keys = ["canonical_label", "label_zh", "display_label"]
            if source is candidate and not self._candidate_has_vlm_label(candidate):
                keys.extend(["raw_label", "label"])
            elif source is track and not self._track_has_vlm_label(track):
                keys.extend(["raw_label", "label"])
            for key in keys:
                text = self._normalize_short_label(source.get(key, ""))
                lowered = text.lower()
                if not lowered or lowered in seen:
                    continue
                seen.add(lowered)
                texts.append(text)
        return texts

    def _track_matches_semantic_target(
        self,
        track: dict[str, Any],
        semantic_task: Optional[SemanticTask],
        candidate: Optional[dict[str, Any]] = None,
    ) -> bool:
        if semantic_task is None:
            return False
        return any(
            self._semantic_match_bonus(text, semantic_task) > 0.0
            for text in self._track_semantic_texts(track, candidate)
        )

    def _is_ambiguous_label(self, label_text: Any) -> bool:
        label = self._normalized_label_key(label_text)
        return bool(label and label in self.vlm_relabel_ambiguous_labels)

    def _label_specificity_score(self, label_text: Any) -> float:
        normalized = self._normalized_label_key(label_text)
        if not normalized:
            return 0.0
        tokens = self._semantic_tokens(normalized)
        score = min(1.4, 0.35 * len(tokens))
        if any(token in COLOR_HINT_TOKENS for token in tokens):
            score += 0.8
        if any(token in APPEARANCE_HINT_TOKENS for token in tokens):
            score += 0.8
        if not self._is_ambiguous_label(normalized):
            score += 0.4
        return score

    def _should_preserve_existing_vlm_label(
        self,
        track: dict[str, Any],
        new_canonical_label: str,
        raw_label: str,
    ) -> bool:
        if not self._track_has_vlm_label(track):
            return False
        previous_canonical = self._normalize_short_label(track.get("canonical_label"))
        if not previous_canonical:
            return False
        candidate_label = self._normalize_short_label(new_canonical_label or raw_label)
        if not candidate_label:
            return True
        previous_specificity = self._label_specificity_score(previous_canonical)
        candidate_specificity = self._label_specificity_score(candidate_label)
        if (
            not self._is_ambiguous_label(previous_canonical)
            and self._is_ambiguous_label(candidate_label)
            and previous_specificity >= candidate_specificity
        ):
            return True
        if (
            self._normalized_label_key(candidate_label) == self._normalized_label_key(raw_label)
            and previous_specificity > candidate_specificity
        ):
            return True
        return False

    def _label_needs_refinement(self, label_text: Any, *, raw_label: Any = "") -> bool:
        normalized = self._normalized_label_key(label_text)
        if not normalized:
            return True
        if normalized == self._normalized_label_key(raw_label):
            return True
        if normalized in GENERIC_VLM_LABELS:
            return True
        if self._is_ambiguous_label(normalized):
            return True
        tokens = self._semantic_tokens(normalized)
        has_color = any(token in COLOR_HINT_TOKENS for token in tokens)
        has_shape = any(token in APPEARANCE_HINT_TOKENS for token in tokens)
        if len(tokens) <= 1 and not (has_color or has_shape):
            return True
        return False

    def _track_vlm_label_needs_refinement(
        self,
        track: dict[str, Any],
        candidate: Optional[dict[str, Any]] = None,
    ) -> bool:
        if not self._track_has_vlm_label(track):
            return True
        raw_label = ""
        if isinstance(candidate, dict):
            raw_label = self._normalize_short_label(candidate.get("raw_label") or candidate.get("label"))
        if not raw_label:
            raw_label = self._normalize_short_label(track.get("raw_label"))
        canonical_label = self._normalize_short_label(track.get("canonical_label"))
        display_label = self._normalize_short_label(track.get("display_label"))
        return (
            self._label_needs_refinement(canonical_label, raw_label=raw_label)
            and self._label_needs_refinement(display_label, raw_label=raw_label)
        )

    def _track_relabel_reason(
        self,
        track: dict[str, Any],
        candidate: dict[str, Any],
        semantic_task: Optional[SemanticTask],
        now_sec: float,
    ) -> str:
        if bool(track.get("relabel_in_flight", False)):
            return ""
        last_requested_at = float(track.get("last_relabel_requested_at", 0.0) or 0.0)
        if (now_sec - last_requested_at) < self.vlm_relabel_track_cooldown_sec:
            return ""

        semantic_signature = self._semantic_signature(semantic_task)
        semantic_changed = bool(
            semantic_signature
            and semantic_signature != str(track.get("last_semantic_signature", "") or "")
        )
        has_vlm_label = self._track_has_vlm_label(track)
        needs_vlm_refinement = self._track_vlm_label_needs_refinement(track, candidate)
        is_ambiguous = self._is_ambiguous_label(
            candidate.get("raw_label") or candidate.get("label") or track.get("raw_label")
        )
        is_low_confidence = float(candidate.get("confidence", 0.0) or 0.0) < self.vlm_relabel_low_confidence_threshold
        is_geometrically_stable = int(track.get("seen_count", 0) or 0) >= self.track_stable_frames_required
        semantic_requested = bool(
            semantic_task is not None
            and (
                str(semantic_task.target_label or "").strip()
                or str(semantic_task.target_hint or "").strip()
            )
        )

        if semantic_requested and (
            semantic_changed
            or not self._track_matches_semantic_target(track, semantic_task, candidate)
        ):
            return "semantic_target"
        if is_ambiguous and (not has_vlm_label or semantic_changed or needs_vlm_refinement):
            return "ambiguous_raw_label"
        if has_vlm_label and needs_vlm_refinement and is_geometrically_stable:
            return "refine_vlm_label"
        if is_low_confidence and is_geometrically_stable and (not has_vlm_label or semantic_changed):
            return "low_confidence_stable"
        if bool(candidate.get("track_new")) and not has_vlm_label:
            return "new_track"
        return ""

    def _track_relabel_priority(
        self,
        reason: str,
        candidate: dict[str, Any],
    ) -> float:
        base = {
            "semantic_target": 4.0,
            "ambiguous_raw_label": 3.0,
            "refine_vlm_label": 2.5,
            "low_confidence_stable": 2.0,
            "new_track": 1.0,
        }.get(reason, 0.0)
        return base + float(candidate.get("confidence", 0.0) or 0.0)

    def _extract_candidate_crop_rgb(
        self,
        image_rgb: np.ndarray,
        candidate: dict[str, Any],
    ) -> Optional[np.ndarray]:
        bbox_xyxy = candidate.get("bbox_xyxy")
        if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
            return None
        margin = self.vlm_relabel_crop_margin_px
        x1 = max(0, int(bbox_xyxy[0]) - margin)
        y1 = max(0, int(bbox_xyxy[1]) - margin)
        x2 = min(int(image_rgb.shape[1]), int(bbox_xyxy[2]) + margin)
        y2 = min(int(image_rgb.shape[0]), int(bbox_xyxy[3]) + margin)
        if x2 <= x1 or y2 <= y1:
            return None
        crop_rgb = np.asarray(image_rgb[y1:y2, x1:x2], dtype=np.uint8).copy()
        if crop_rgb.size == 0:
            return None
        max_side = max(int(crop_rgb.shape[0]), int(crop_rgb.shape[1]))
        if max_side > self.vlm_relabel_resize_max_side_px:
            scale = float(self.vlm_relabel_resize_max_side_px) / float(max_side)
            new_width = max(1, int(round(float(crop_rgb.shape[1]) * scale)))
            new_height = max(1, int(round(float(crop_rgb.shape[0]) * scale)))
            crop_rgb = cv2.resize(
                crop_rgb,
                (new_width, new_height),
                interpolation=cv2.INTER_AREA,
            )
        return crop_rgb

    def _render_vlm_relabel_context_rgb(
        self,
        image_rgb: np.ndarray,
        candidates: list[dict[str, Any]],
    ) -> np.ndarray:
        canvas = np.asarray(image_rgb, dtype=np.uint8).copy()
        palette = [
            (255, 99, 71),
            (64, 224, 208),
            (255, 215, 0),
            (50, 205, 50),
            (30, 144, 255),
        ]
        for order, candidate in enumerate(candidates):
            bbox_xyxy = candidate.get("bbox_xyxy")
            if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
                continue
            x1, y1, x2, y2 = [int(value) for value in bbox_xyxy]
            color = palette[order % len(palette)]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            box_id = str(int(candidate.get("index", order)))
            label = f"#{box_id}"
            (text_width, text_height), baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                2,
            )
            text_x = max(0, x1)
            text_y = max(text_height + baseline + 4, y1)
            cv2.rectangle(
                canvas,
                (text_x, text_y - text_height - baseline - 4),
                (text_x + text_width + 8, text_y + 4),
                color,
                thickness=-1,
            )
            cv2.putText(
                canvas,
                label,
                (text_x + 4, text_y - baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (15, 15, 15),
                2,
                cv2.LINE_AA,
            )
        return canvas

    def _query_vlm_relabel_batch(
        self,
        specs: list[dict[str, Any]],
        overview_image_url: str,
    ) -> dict[int, dict[str, Any]]:
        if not specs:
            return {}

        headers = {"Content-Type": "application/json"}
        api_key = self.vlm_relabel_api_key
        if api_key and api_key.upper() != "EMPTY":
            headers["Authorization"] = f"Bearer {api_key}"

        system_prompt = (
            "You relabel numbered object proposals from a robot camera. "
            "Return exactly one JSON object with no markdown. "
            "Schema: {\"candidates\":[{\"index\":number,\"label_en\":string,\"label_zh\":string}]}. "
            "Use very short noun phrases, not sentences, with 1 to 4 words per label. "
            "The first image is the full scene with numbered boxes. Later images are crops of those numbered boxes. "
            "Use the crop as primary evidence and the full scene only as context. "
            "Ignore the detector raw label if it looks wrong. "
            "If you are not clearly confident in a standard object class, describe the visible object by appearance instead: "
            "blue cylinder, white paper cup, white block, black tool handle, silver can. "
            "Prefer color-plus-shape descriptions over generic class guesses. "
            "If the object looks like a smooth cylinder and you cannot clearly see a cup rim, handle, bottle neck, or can top, label it as cylinder instead of cup, bottle, vase, or can. "
            "Do not answer with vague words like object, item, thing. "
            "Do not use dataset artifacts like remote, mouse, keyboard, cell phone unless the visible features clearly support that label."
        )

        user_content: list[dict[str, Any]] = [
            {
                "type": "text",
                "text": (
                    "Relabel each numbered candidate. "
                    "Keep label_en in short English and label_zh in short Chinese. "
                    "Prefer physical object names or color-plus-shape descriptions."
                ),
            }
        ]
        if overview_image_url:
            user_content.append(
                {
                    "type": "text",
                    "text": "Overview image with numbered boxes for context.",
                }
            )
            user_content.append(
                {
                    "type": "image_url",
                    "image_url": {"url": overview_image_url},
                }
            )
        for spec in specs:
            user_content.append(
                {
                    "type": "text",
                    "text": (
                        f"Candidate #{int(spec['index'])}: "
                        f"raw_label={str(spec.get('raw_label', '') or '')}, "
                        f"detector_confidence={float(spec.get('confidence', 0.0) or 0.0):.2f}. "
                        "Use the crop below to decide the label."
                    ),
                }
            )
            user_content.append(
                {
                    "type": "image_url",
                    "image_url": {"url": str(spec["image_url"])},
                }
            )

        payload = {
            "model": self.vlm_relabel_model_name,
            "messages": [
                {"role": "system", "content": [{"type": "text", "text": system_prompt}]},
                {"role": "user", "content": user_content},
            ],
            "temperature": 0.1,
            "max_tokens": 320,
            "response_format": {"type": "json_object"},
        }
        response = self._vlm_session.post(
            self.vlm_relabel_endpoint,
            headers=headers,
            json=payload,
            timeout=self.vlm_relabel_request_timeout_sec,
        )
        if response.status_code in {400, 404, 422}:
            fallback_payload = dict(payload)
            fallback_payload.pop("response_format", None)
            response = self._vlm_session.post(
                self.vlm_relabel_endpoint,
                headers=headers,
                json=fallback_payload,
                timeout=self.vlm_relabel_request_timeout_sec,
            )
        response.raise_for_status()
        parsed = extract_first_json_object(extract_message_text(response.json()))
        items = parsed.get("candidates", [])
        if not isinstance(items, list):
            raise ValueError("vlm relabel response missing candidates array")

        relabeled: dict[int, dict[str, Any]] = {}
        for item in items:
            if not isinstance(item, dict):
                continue
            try:
                index = int(item.get("index"))
            except (TypeError, ValueError):
                continue
            label_en = self._normalize_short_label(
                item.get("canonical_label") or item.get("label_en") or item.get("label")
            )
            label_zh = self._normalize_short_label(item.get("label_zh"))
            relabeled[index] = {
                "canonical_label": label_en,
                "label_en": label_en,
                "label_zh": label_zh,
            }
        return relabeled

    def _run_vlm_relabel_batch(
        self,
        specs: list[dict[str, Any]],
        overview_image_url: str,
    ) -> None:
        try:
            relabeled = self._query_vlm_relabel_batch(specs, overview_image_url)
            now_sec = time.time()
            with self._track_lock:
                for spec in specs:
                    track_id = int(spec.get("track_id", -1))
                    track = self._tracks.get(track_id)
                    if not isinstance(track, dict):
                        continue
                    item = relabeled.get(int(spec["index"]), {})
                    raw_label = self._normalize_short_label(spec.get("raw_label"))
                    canonical_label = self._normalize_short_label(item.get("canonical_label"))
                    label_zh = self._normalize_short_label(item.get("label_zh"))
                    if self._should_preserve_existing_vlm_label(track, canonical_label, raw_label):
                        canonical_label = self._normalize_short_label(track.get("canonical_label"))
                        label_zh = self._normalize_short_label(track.get("label_zh"))
                    elif not canonical_label and self._track_has_vlm_label(track):
                        canonical_label = self._normalize_short_label(track.get("canonical_label"))
                        label_zh = self._normalize_short_label(track.get("label_zh"))
                    elif not canonical_label:
                        canonical_label = raw_label
                    display_label = self._compose_candidate_display_label(
                        canonical_label=canonical_label or raw_label,
                        label_zh=label_zh,
                        raw_label=raw_label,
                    )
                    if canonical_label:
                        track["canonical_label"] = canonical_label
                        track["label_zh"] = label_zh
                        track["display_label"] = display_label
                        track["label_source"] = "vlm"
                    track["last_relabel_completed_at"] = now_sec
                    track["last_relabel_reason"] = str(spec.get("relabel_reason", "") or "")
                    track["last_semantic_signature"] = str(spec.get("semantic_signature", "") or "")
                    track["relabel_in_flight"] = False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"candidate relabel via VLM failed: {exc}")
            with self._track_lock:
                for spec in specs:
                    track_id = int(spec.get("track_id", -1))
                    track = self._tracks.get(track_id)
                    if not isinstance(track, dict):
                        continue
                    track["relabel_in_flight"] = False
        finally:
            with self._vlm_relabel_lock:
                self._vlm_relabel_in_flight = False
            self._pending_request = True

    def _rollback_pending_track_relabels(
        self,
        specs: list[dict[str, Any]],
    ) -> None:
        if not specs:
            return
        with self._track_lock:
            for spec in specs:
                track_id = int(spec.get("track_id", -1))
                track = self._tracks.get(track_id)
                if not isinstance(track, dict):
                    continue
                track["relabel_in_flight"] = False
                track["last_relabel_requested_at"] = float(
                    spec.get("previous_relabel_requested_at", 0.0) or 0.0
                )
                track["last_relabel_reason"] = str(spec.get("previous_relabel_reason", "") or "")

    def _schedule_candidate_relabels(
        self,
        image_rgb: np.ndarray,
        raw_candidates: list[dict[str, Any]],
        semantic_task: Optional[SemanticTask],
    ) -> None:
        if (
            not self.vlm_relabel_enabled
            or not self.vlm_relabel_endpoint
            or not self.vlm_relabel_model_name
            or not raw_candidates
        ):
            return

        now_sec = time.time()
        eligible: list[tuple[float, dict[str, Any], dict[str, Any], str]] = []
        with self._track_lock:
            for candidate in raw_candidates:
                confidence = float(candidate.get("confidence", 0.0) or 0.0)
                if confidence < self.vlm_relabel_min_confidence:
                    continue
                track_id = int(candidate.get("track_id", -1) or -1)
                if track_id < 0:
                    continue
                track = self._tracks.get(track_id)
                if not isinstance(track, dict):
                    continue
                reason = self._track_relabel_reason(track, candidate, semantic_task, now_sec)
                if not reason:
                    continue
                priority = self._track_relabel_priority(reason, candidate)
                eligible.append((priority, candidate, track, reason))

        if not eligible:
            return

        eligible.sort(key=lambda item: item[0], reverse=True)
        pending_specs: list[dict[str, Any]] = []
        pending_candidates: list[dict[str, Any]] = []
        semantic_signature = self._semantic_signature(semantic_task)
        with self._track_lock:
            for _, candidate, track, reason in eligible:
                if len(pending_specs) >= self.vlm_relabel_top_k:
                    break
                track_id = int(track.get("track_id", -1))
                active_track = self._tracks.get(track_id)
                if not isinstance(active_track, dict):
                    continue
                if bool(active_track.get("relabel_in_flight", False)):
                    continue
                crop_rgb = self._extract_candidate_crop_rgb(image_rgb, candidate)
                if crop_rgb is None or crop_rgb.size == 0:
                    continue
                image_url = (
                    "data:image/jpeg;base64,"
                    f"{encode_image_to_base64_jpeg(crop_rgb, self.vlm_relabel_jpeg_quality)}"
                )
                previous_requested_at = float(active_track.get("last_relabel_requested_at", 0.0) or 0.0)
                previous_reason = str(active_track.get("last_relabel_reason", "") or "")
                active_track["relabel_in_flight"] = True
                active_track["last_relabel_requested_at"] = now_sec
                active_track["last_relabel_reason"] = str(reason or "")
                pending_specs.append(
                    {
                        "index": int(candidate.get("index", -1)),
                        "track_id": track_id,
                        "raw_label": self._normalize_short_label(
                            candidate.get("raw_label") or candidate.get("label")
                        ),
                        "confidence": float(candidate.get("confidence", 0.0) or 0.0),
                        "image_url": image_url,
                        "relabel_reason": str(reason or ""),
                        "semantic_signature": semantic_signature,
                        "previous_relabel_requested_at": previous_requested_at,
                        "previous_relabel_reason": previous_reason,
                    }
                )
                pending_candidates.append(candidate)

        if not pending_specs:
            return

        overview_image_url = ""
        if pending_candidates:
            overview_rgb = self._render_vlm_relabel_context_rgb(image_rgb, pending_candidates)
            overview_image_url = (
                "data:image/jpeg;base64,"
                f"{encode_image_to_base64_jpeg(overview_rgb, self.vlm_relabel_jpeg_quality)}"
            )
        with self._vlm_relabel_lock:
            if self._vlm_relabel_in_flight:
                self._rollback_pending_track_relabels(pending_specs)
                return
            if (time.time() - self._vlm_relabel_last_started) < self.vlm_relabel_min_interval_sec:
                self._rollback_pending_track_relabels(pending_specs)
                return
            self._vlm_relabel_in_flight = True
            self._vlm_relabel_last_started = time.time()

        try:
            threading.Thread(
                target=self._run_vlm_relabel_batch,
                args=(pending_specs, overview_image_url),
                daemon=True,
            ).start()
        except Exception:  # noqa: BLE001
            self._rollback_pending_track_relabels(pending_specs)
            with self._vlm_relabel_lock:
                self._vlm_relabel_in_flight = False
            raise

    def _semantic_payload(self, semantic_task: Optional[SemanticTask]) -> dict[str, Any]:
        return {
            "task": semantic_task.task if semantic_task is not None else "pick",
            "target_label": semantic_task.target_label if semantic_task is not None else "",
            "target_hint": semantic_task.target_hint if semantic_task is not None else "",
            "constraints": list(semantic_task.constraints) if semantic_task is not None else [],
            "excluded_labels": list(semantic_task.excluded_labels)
            if semantic_task is not None
            else [],
        }

    def _semantic_match_bonus(
        self,
        detected_label: str,
        semantic_task: Optional[SemanticTask],
    ) -> float:
        label = detected_label.strip().lower()
        if not label or semantic_task is None:
            return 0.0

        bonus = 0.0
        target_label = str(semantic_task.target_label or "").strip().lower()
        target_hint = str(semantic_task.target_hint or "").strip().lower()
        excluded = {str(item).strip().lower() for item in semantic_task.excluded_labels}
        if label in excluded:
            return -1.0
        if target_label:
            if target_label == label:
                bonus += 0.35
            elif label in target_label or target_label in label:
                bonus += 0.20
        if target_hint:
            hint_tokens = self._semantic_tokens(target_hint)
            label_tokens = set(self._semantic_tokens(label))
            overlap = sum(1 for token in hint_tokens if token in label_tokens)
            bonus += min(0.15, 0.05 * overlap)
        alias_labels = self._semantic_alias_labels(target_label, target_hint)
        if any(self._labels_related(label, alias) for alias in alias_labels):
            bonus += 0.25
        if (target_label or target_hint) and bonus == 0.0:
            bonus -= 0.05
        return bonus

    def _candidate_spatial_bonus(
        self,
        candidate: dict[str, Any],
        semantic_task: Optional[SemanticTask],
    ) -> float:
        if semantic_task is None:
            return 0.0
        requested_directions = self._semantic_direction_hints(
            str(semantic_task.target_label or ""),
            str(semantic_task.target_hint or ""),
        )
        if not requested_directions:
            return 0.0
        candidate_direction = self._candidate_horizontal_region(candidate)
        if not candidate_direction:
            return 0.0
        if candidate_direction in requested_directions:
            return 0.18
        if "center" in requested_directions and candidate_direction == "center":
            return 0.12
        return -0.12

    def _semantic_task_requires_match(
        self,
        semantic_task: Optional[SemanticTask],
    ) -> bool:
        if semantic_task is None:
            return False
        return bool(
            str(semantic_task.target_label or "").strip()
            or str(semantic_task.target_hint or "").strip()
        )

    def _candidate_confidence_floor(
        self,
        semantic_bonus: float,
        semantic_task: Optional[SemanticTask],
    ) -> float:
        if self._semantic_task_requires_match(semantic_task) and semantic_bonus > 0.0:
            return min(self.confidence_threshold, self.semantic_match_confidence_floor)
        return self.confidence_threshold

    def _semantic_tokens(self, text: str) -> list[str]:
        normalized = text.replace(",", " ").replace("-", " ").lower()
        return [token for token in normalized.split() if token]

    def _semantic_alias_labels(self, *semantic_texts: str) -> set[str]:
        aliases: set[str] = set()
        for text in semantic_texts:
            for token in self._semantic_tokens(text):
                aliases.update(SEMANTIC_ALIAS_LABELS.get(token, set()))
        return aliases

    def _semantic_direction_hints(self, *semantic_texts: str) -> set[str]:
        lowered = " ".join(str(text or "").strip().lower() for text in semantic_texts if str(text or "").strip())
        hints: set[str] = set()
        if not lowered:
            return hints
        for direction, keywords in SPATIAL_DIRECTION_TOKENS.items():
            if any(keyword in lowered for keyword in keywords):
                hints.add(direction)
        return hints

    def _candidate_horizontal_region(self, candidate: dict[str, Any]) -> str:
        bbox_xyxy = candidate.get("bbox_xyxy")
        if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
            return ""
        image_width = int(candidate.get("image_width", 0) or 0)
        if image_width <= 0:
            return ""
        center_x = (float(bbox_xyxy[0]) + float(bbox_xyxy[2])) * 0.5
        ratio = center_x / float(image_width)
        if ratio <= 0.4:
            return "left"
        if ratio >= 0.6:
            return "right"
        return "center"

    def _labels_related(self, detected_label: str, semantic_label: str) -> bool:
        left = detected_label.strip().lower()
        right = semantic_label.strip().lower()
        if not left or not right:
            return False
        return bool(left == right or left in right or right in left)

    def _extract_bbox_xyxy(
        self,
        boxes: Any,
        index: int,
        image_width: int,
        image_height: int,
    ) -> Optional[list[int]]:
        xyxy_tensor = boxes.xyxy[index]
        bbox_xyxy = [
            float(v)
            for v in (
                xyxy_tensor.cpu().numpy().tolist()
                if hasattr(xyxy_tensor, "cpu")
                else list(xyxy_tensor)
            )
        ]
        return coerce_bbox(bbox_xyxy, image_width, image_height)

    def _extract_mask_u8(
        self,
        masks_data: Optional[np.ndarray],
        index: int,
        image_shape: tuple[int, int],
    ) -> Optional[np.ndarray]:
        if masks_data is None or index >= int(masks_data.shape[0]):
            return None
        mask = np.asarray(masks_data[index], dtype=np.float32)
        if mask.shape[:2] != image_shape:
            mask = cv2.resize(
                mask,
                (int(image_shape[1]), int(image_shape[0])),
                interpolation=cv2.INTER_NEAREST,
            )
        return np.where(mask > 0.5, 255, 0).astype(np.uint8)

    def _select_debug_candidates(
        self,
        raw_candidates: list[dict[str, Any]],
        selected_candidate: Optional[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        ranked = sorted(
            raw_candidates,
            key=lambda item: (
                float(item["confidence"]),
                float(item["score"]),
            ),
            reverse=True,
        )
        top_candidates = ranked[: self.debug_top_k_candidates]
        if (
            selected_candidate is not None and
            all(int(item["index"]) != int(selected_candidate["index"]) for item in top_candidates)
        ):
            remaining = [
                item
                for item in ranked
                if int(item["index"]) != int(selected_candidate["index"])
            ]
            top_candidates = [selected_candidate] + remaining[: self.debug_top_k_candidates - 1]
        return top_candidates

    def _candidate_debug_public(self, candidate: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
        if candidate is None:
            return None
        bbox_xyxy = candidate.get("bbox_xyxy")
        bbox_list = list(bbox_xyxy) if isinstance(bbox_xyxy, list) else None
        canonical_label = self._candidate_preferred_label(candidate)
        raw_label = self._normalize_short_label(candidate.get("raw_label") or candidate.get("label"))
        label_zh = self._normalize_short_label(candidate.get("label_zh"))
        display_label = self._candidate_preferred_display_label(candidate)
        return {
            "index": int(candidate.get("index", -1)),
            "track_id": int(candidate.get("track_id", -1)),
            "track_new": bool(candidate.get("track_new", False)),
            "track_seen_count": int(candidate.get("track_seen_count", 0) or 0),
            "track_consecutive_hits": int(candidate.get("track_consecutive_hits", 0) or 0),
            "track_age_sec": round(float(candidate.get("track_age_sec", 0.0) or 0.0), 3),
            "track_label_age_sec": round(float(candidate.get("track_label_age_sec", 0.0) or 0.0), 3),
            "track_relabel_reason": str(candidate.get("track_relabel_reason", "") or ""),
            "label": canonical_label or raw_label,
            "raw_label": raw_label,
            "canonical_label": canonical_label or raw_label,
            "label_zh": label_zh,
            "display_label": display_label or canonical_label or raw_label,
            "label_source": str(candidate.get("label_source", "yolo") or "yolo"),
            "side": self._candidate_horizontal_region(candidate),
            "confidence": round(float(candidate.get("confidence", 0.0)), 4),
            "confidence_floor": round(float(candidate.get("confidence_floor", 0.0)), 4),
            "semantic_bonus": round(float(candidate.get("semantic_bonus", 0.0)), 4),
            "score": round(float(candidate.get("score", 0.0)), 4),
            "bbox_xyxy": bbox_list,
            "mask_pixels": int(candidate.get("mask_pixels", 0)),
            "status": str(candidate.get("status", "")),
        }

    def _format_candidate_summary(
        self,
        semantic_task: Optional[SemanticTask],
        top_candidates: list[dict[str, Any]],
        selected_candidate: Optional[dict[str, Any]],
    ) -> str:
        target_hint = (
            str(semantic_task.target_hint or "").strip()
            if semantic_task is not None
            else ""
        )
        target_label = (
            str(semantic_task.target_label or "").strip()
            if semantic_task is not None
            else ""
        )
        selected_label = (
            self._candidate_preferred_label(selected_candidate)
            if selected_candidate is not None
            else "<none>"
        )
        parts: list[str] = []
        for rank, candidate in enumerate(top_candidates[: self.debug_top_k_candidates], start=1):
            label_text = self._candidate_preferred_display_label(candidate) or self._candidate_preferred_label(candidate)
            raw_label = self._normalize_short_label(candidate.get("raw_label") or candidate.get("label"))
            raw_suffix = f",raw={raw_label}" if raw_label and raw_label.lower() != label_text.lower() else ""
            track_suffix = ""
            track_id = int(candidate.get("track_id", -1) or -1)
            if track_id >= 0:
                track_suffix = f",track={track_id}"
            parts.append(
                f"#{rank}:{label_text}"
                f"(c={float(candidate['confidence']):.3f},"
                f"b={float(candidate['semantic_bonus']):+.3f},"
                f"s={float(candidate['score']):.3f},"
                f"status={str(candidate['status'])}{track_suffix}{raw_suffix})"
            )
        return (
            "detector top-k: "
            f"target_label={target_label or '<none>'} "
            f"target_hint={target_hint or '<none>'} "
            f"selected={selected_label or '<none>'} "
            f"candidates=[{'; '.join(parts)}]"
        )

    def _render_debug_overlay(
        self,
        image_rgb: np.ndarray,
        top_candidates: list[dict[str, Any]],
        selected_candidate: Optional[dict[str, Any]],
        semantic_task: Optional[SemanticTask],
    ) -> np.ndarray:
        canvas = np.asarray(image_rgb, dtype=np.uint8).copy()
        selected_index = int(selected_candidate["index"]) if selected_candidate is not None else -1

        for candidate in top_candidates:
            color = self._candidate_overlay_color(candidate, selected_index)
            mask_u8 = candidate.get("mask_u8")
            if isinstance(mask_u8, np.ndarray) and mask_u8.shape[:2] == canvas.shape[:2]:
                mask = mask_u8 > 0
                if np.any(mask):
                    color_array = np.asarray(color, dtype=np.float32)
                    blended = canvas[mask].astype(np.float32) * 0.65 + color_array * 0.35
                    canvas[mask] = np.clip(blended, 0.0, 255.0).astype(np.uint8)

        for rank, candidate in enumerate(top_candidates[: self.debug_top_k_candidates], start=1):
            bbox_xyxy = candidate.get("bbox_xyxy")
            if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
                continue
            x1, y1, x2, y2 = [int(value) for value in bbox_xyxy]
            color = self._candidate_overlay_color(candidate, selected_index)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            display_label = self._candidate_preferred_display_label(candidate) or self._candidate_preferred_label(candidate)
            track_id = int(candidate.get("track_id", -1) or -1)
            track_prefix = f"T{track_id} " if track_id >= 0 else ""
            label_text = (
                f"#{rank} {track_prefix}{display_label} "
                f"c={float(candidate['confidence']):.2f} "
                f"s={float(candidate['score']):.2f}"
            )
            text_y = y1 - 8 if y1 > 20 else y1 + 18
            cv2.putText(
                canvas,
                label_text,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

        header_lines = [
            (
                "target="
                f"{str(getattr(semantic_task, 'target_hint', '') or '<none>')}"
                f" selected={self._candidate_preferred_label(selected_candidate) if selected_candidate else '<none>'}"
            ),
            "top-k="
            + " | ".join(
                f"{self._candidate_preferred_label(candidate)}:{float(candidate['confidence']):.2f}"
                for candidate in top_candidates[: self.debug_top_k_candidates]
            ),
        ]
        bar_height = 24 * len(header_lines) + 8
        cv2.rectangle(
            canvas,
            (0, 0),
            (int(canvas.shape[1]), int(bar_height)),
            (0, 0, 0),
            thickness=-1,
        )
        for line_index, line in enumerate(header_lines, start=1):
            cv2.putText(
                canvas,
                line,
                (8, 20 * line_index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        return canvas

    def _candidate_overlay_color(
        self,
        candidate: dict[str, Any],
        selected_index: int,
    ) -> tuple[int, int, int]:
        if int(candidate.get("index", -1)) == selected_index:
            return (32, 255, 96)
        status = str(candidate.get("status", ""))
        if status == "filtered_low_confidence":
            return (180, 180, 180)
        if status == "filtered_excluded_label":
            return (255, 96, 96)
        return (255, 176, 64)

    def _bbox_center_point(self, bbox_xyxy: Optional[list[int]]) -> Optional[list[int]]:
        if bbox_xyxy is None:
            return None
        return [
            int(round((bbox_xyxy[0] + bbox_xyxy[2]) * 0.5)),
            int(round((bbox_xyxy[1] + bbox_xyxy[3]) * 0.5)),
        ]

    def _is_candidate_complete(
        self,
        bbox_xyxy: Optional[list[int]],
        image_width: int,
        image_height: int,
    ) -> bool:
        if bbox_xyxy is None:
            return False
        margin = self.candidate_complete_edge_margin_px
        x1, y1, x2, y2 = bbox_xyxy
        return bool(
            x1 > margin
            and y1 > margin
            and x2 < (image_width - margin)
            and y2 < (image_height - margin)
        )

    def _build_detection_payload(
        self,
        *,
        backend: str,
        task: str,
        target_label: str,
        confidence: float,
        need_human_confirm: bool,
        reason: str,
        image_width: int,
        image_height: int,
        frame_id: str,
        bbox_xyxy: Optional[list[int]],
        point_px: Optional[list[int]],
        mask_u8: Optional[np.ndarray],
    ) -> dict[str, Any]:
        accepted = bool(target_label and bbox_xyxy is not None and confidence > 0.0)
        return {
            "backend": backend,
            "accepted": accepted,
            "candidate_visible": accepted,
            "candidate_complete": self._is_candidate_complete(
                bbox_xyxy, image_width, image_height
            ),
            "task": str(task or "pick"),
            "target_label": str(target_label or ""),
            "confidence": max(0.0, min(1.0, float(confidence))),
            "need_human_confirm": bool(need_human_confirm),
            "reason": str(reason or ""),
            "image_width": int(image_width),
            "image_height": int(image_height),
            "frame_id": str(frame_id or ""),
            "bbox_xyxy": bbox_xyxy,
            "point_px": point_px,
            "mask_u8": mask_u8,
        }

    def _build_empty_result(
        self,
        *,
        task: str,
        target_label: str,
        reason: str,
        image_width: int = 0,
        image_height: int = 0,
        frame_id: str = "",
    ) -> dict[str, Any]:
        return {
            "backend": self.backend,
            "accepted": False,
            "candidate_visible": False,
            "candidate_complete": False,
            "task": str(task or "pick"),
            "target_label": str(target_label or ""),
            "confidence": 0.0,
            "need_human_confirm": True,
            "reason": str(reason or ""),
            "image_width": int(image_width),
            "image_height": int(image_height),
            "frame_id": str(frame_id or ""),
            "bbox_xyxy": None,
            "point_px": None,
            "mask_u8": None,
        }

    def _publish_detection(self, payload: dict[str, Any]) -> None:
        detection_msg = DetectionResult()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = str(payload.get("frame_id", "") or "")
        detection_msg.backend = str(payload.get("backend", ""))
        detection_msg.accepted = bool(payload.get("accepted", False))
        detection_msg.candidate_visible = bool(payload.get("candidate_visible", False))
        detection_msg.candidate_complete = bool(payload.get("candidate_complete", False))
        detection_msg.task = str(payload.get("task", ""))
        detection_msg.target_label = str(payload.get("target_label", ""))
        detection_msg.confidence = float(payload.get("confidence", 0.0))
        detection_msg.need_human_confirm = bool(payload.get("need_human_confirm", False))
        detection_msg.reason = str(payload.get("reason", ""))
        detection_msg.image_width = int(payload.get("image_width", 0))
        detection_msg.image_height = int(payload.get("image_height", 0))

        bbox_xyxy = payload.get("bbox_xyxy")
        if isinstance(bbox_xyxy, list) and len(bbox_xyxy) == 4:
            detection_msg.has_bbox = True
            roi = RegionOfInterest()
            roi.x_offset = int(bbox_xyxy[0])
            roi.y_offset = int(bbox_xyxy[1])
            roi.width = max(0, int(bbox_xyxy[2]) - int(bbox_xyxy[0]))
            roi.height = max(0, int(bbox_xyxy[3]) - int(bbox_xyxy[1]))
            roi.do_rectify = False
            detection_msg.bbox = roi
        else:
            detection_msg.has_bbox = False

        point_px = payload.get("point_px")
        if isinstance(point_px, list) and len(point_px) == 2:
            detection_msg.has_point = True
            detection_msg.point_px = [int(point_px[0]), int(point_px[1])]
        else:
            detection_msg.has_point = False
            detection_msg.point_px = [0, 0]

        mask_u8 = payload.get("mask_u8")
        if isinstance(mask_u8, np.ndarray) and mask_u8.size > 0:
            detection_msg.has_mask = True
            detection_msg.mask = make_mono8_image(
                mask_u8,
                frame_id=detection_msg.header.frame_id,
                stamp=detection_msg.header.stamp,
            )
        else:
            detection_msg.has_mask = False
            detection_msg.mask = Image()

        self.detection_pub.publish(detection_msg)
        self.candidate_visible_pub.publish(
            Bool(data=bool(payload.get("candidate_visible", False)))
        )

        debug_payload = {
            "backend": detection_msg.backend,
            "accepted": detection_msg.accepted,
            "label": detection_msg.target_label,
            "display_label": self._candidate_preferred_display_label(
                payload.get("selected_candidate")
            )
            or detection_msg.target_label,
            "confidence": round(float(detection_msg.confidence), 4),
            "has_bbox": bool(detection_msg.has_bbox),
            "has_mask": bool(detection_msg.has_mask),
            "reason": detection_msg.reason,
            "semantic_target": payload.get("semantic_target", {}),
            "selected_candidate": payload.get("selected_candidate"),
            "top_candidates": payload.get("debug_candidates", []),
            "candidate_summary": str(payload.get("candidate_summary", "") or ""),
        }
        debug_msg = String()
        debug_msg.data = compact_json(debug_payload)
        self.debug_pub.publish(debug_msg)

        if self.debug_overlay_pub is not None:
            debug_overlay_rgb = payload.get("debug_overlay_rgb")
            if isinstance(debug_overlay_rgb, np.ndarray) and debug_overlay_rgb.size > 0:
                self.debug_overlay_pub.publish(
                    make_rgb8_image(
                        np.asarray(debug_overlay_rgb, dtype=np.uint8),
                        frame_id=detection_msg.header.frame_id,
                        stamp=detection_msg.header.stamp,
                    )
                )

        summary = (
            "detector decision: "
            f"accepted={detection_msg.accepted} "
            f"label={detection_msg.target_label or '<none>'} "
            f"visible={bool(payload.get('candidate_visible', False))} "
            f"mask={detection_msg.has_mask}"
        )
        if summary != self._last_terminal_summary:
            self._last_terminal_summary = summary
            self.get_logger().info(summary)

        candidate_summary = str(payload.get("candidate_summary", "") or "")
        if candidate_summary and candidate_summary != self._last_candidate_summary:
            self._last_candidate_summary = candidate_summary
            self.get_logger().info(candidate_summary)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
