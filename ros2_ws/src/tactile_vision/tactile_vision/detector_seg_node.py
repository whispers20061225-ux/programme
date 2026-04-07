from __future__ import annotations

import json
import os
import threading
import time
import traceback
import copy
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
from tactile_interfaces.msg import DetectionResult, SemanticTask, TaskExecutionStatus

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

OPEN_VOCAB_ALIAS_PROMPTS = {
    "圆柱体": ["cylinder", "blue cylinder", "container"],
    "蓝色圆柱体": ["blue cylinder", "cylinder", "container"],
    "蓝色柱体": ["blue cylinder", "cylinder", "container"],
    "cylinder": ["cylinder", "blue cylinder", "container"],
    "blue cylinder": ["blue cylinder", "cylinder", "container"],
    "方块": ["block", "cube", "box"],
    "立方体": ["cube", "block", "box"],
    "红色方块": ["red block", "red cube", "block"],
    "block": ["block", "cube", "box"],
    "cube": ["cube", "block", "box"],
    "charger": ["charger", "power adapter"],
    "充电器": ["charger", "power adapter"],
    "适配器": ["power adapter", "charger"],
    "cup": ["cup", "mug", "container"],
    "mug": ["mug", "cup", "container"],
    "bottle": ["bottle", "container"],
    "can": ["can", "container"],
}

OPEN_VOCAB_TEXT_REPLACEMENTS = (
    ("蓝色", "blue "),
    ("红色", "red "),
    ("白色", "white "),
    ("黑色", "black "),
    ("绿色", "green "),
    ("黄色", "yellow "),
    ("圆柱体", "cylinder"),
    ("柱体", "cylinder"),
    ("方块", "block"),
    ("立方体", "cube"),
    ("充电器", "charger"),
    ("适配器", "power adapter"),
)


SCENE_PROMPT_SYSTEM_PROMPT = (
    "You generate a reusable open-vocabulary detector prompt set for a robot tabletop scene. "
    "Return exactly one JSON object and no markdown. "
    "Schema: {\"scene_prompt_classes\":[string,...],\"negative_labels\":[string,...],"
    "\"attributes\":[string,...],\"scene_summary\":string}. "
    "scene_prompt_classes must be short English detector phrases, 1 to 4 words each. "
    "Prefer visually grounded object labels and color-plus-shape phrases when color is obvious. "
    "Focus on salient pickable foreground objects. "
    "Ignore the robot arm, gripper, table surface, shadows, reflections, and UI text. "
    "Do not output sentences, verbs, or vague words like object, item, thing. "
    "If a shape is clearer than a fine-grained category, prefer the clearer physical description."
)


def _normalized_secret(value: Any) -> str:
    secret = str(value or "").strip()
    if not secret or secret.upper() in {"EMPTY", "NONE", "NULL"}:
        return ""
    return secret


def _normalize_openai_compatible_endpoint(value: Any) -> str:
    endpoint = str(value or "").strip()
    if not endpoint:
        return ""
    if endpoint.endswith("/v1"):
        return endpoint + "/chat/completions"
    if endpoint.endswith("/chat/completions"):
        return endpoint
    if endpoint.endswith("/"):
        endpoint = endpoint[:-1]
    if endpoint.endswith("/compatible-mode"):
        return endpoint + "/v1/chat/completions"
    if endpoint.endswith("/v1/models"):
        return endpoint[: -len("/models")] + "/chat/completions"
    return endpoint


def _load_remote_vlm_file() -> dict[str, str]:
    env_file = os.path.expanduser("~/.config/programme/remote_vlm.env")
    values: dict[str, str] = {}
    try:
        with open(env_file, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export ") :].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip()
                if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
                    value = value[1:-1]
                if key and value:
                    values[key] = value
    except OSError:
        return {}
    return values


def _resolve_runtime_model_config(
    *,
    endpoint: Any,
    model_name: Any,
    api_key: Any,
) -> tuple[str, str, str]:
    resolved_endpoint = _normalize_openai_compatible_endpoint(endpoint)
    resolved_model = str(model_name or "").strip()
    resolved_api_key = _normalized_secret(api_key)
    file_values = _load_remote_vlm_file()

    env_api_key = _normalized_secret(
        os.getenv("PROGRAMME_SCENE_PROMPT_API_KEY")
        or file_values.get("PROGRAMME_SCENE_PROMPT_API_KEY")
        or os.getenv("PROGRAMME_DIALOG_API_KEY")
        or file_values.get("PROGRAMME_DIALOG_API_KEY")
        or os.getenv("PROGRAMME_REMOTE_VLM_API_KEY")
        or file_values.get("PROGRAMME_REMOTE_VLM_API_KEY")
        or os.getenv("DASHSCOPE_API_KEY")
        or file_values.get("DASHSCOPE_API_KEY")
        or os.getenv("OPENAI_API_KEY")
        or file_values.get("OPENAI_API_KEY")
    )
    env_endpoint = _normalize_openai_compatible_endpoint(
        os.getenv("PROGRAMME_SCENE_PROMPT_MODEL_ENDPOINT")
        or file_values.get("PROGRAMME_SCENE_PROMPT_MODEL_ENDPOINT")
        or os.getenv("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or file_values.get("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or os.getenv("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or file_values.get("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or os.getenv("DASHSCOPE_BASE_URL")
        or file_values.get("DASHSCOPE_BASE_URL")
        or os.getenv("OPENAI_BASE_URL")
        or file_values.get("OPENAI_BASE_URL")
    )
    env_model = str(
        os.getenv("PROGRAMME_SCENE_PROMPT_MODEL_NAME")
        or file_values.get("PROGRAMME_SCENE_PROMPT_MODEL_NAME")
        or os.getenv("PROGRAMME_DIALOG_MODEL_NAME")
        or file_values.get("PROGRAMME_DIALOG_MODEL_NAME")
        or os.getenv("PROGRAMME_REMOTE_VLM_MODEL")
        or file_values.get("PROGRAMME_REMOTE_VLM_MODEL")
        or os.getenv("DASHSCOPE_MODEL")
        or file_values.get("DASHSCOPE_MODEL")
        or os.getenv("OPENAI_MODEL")
        or file_values.get("OPENAI_MODEL")
        or ""
    ).strip()

    if env_api_key:
        resolved_api_key = env_api_key
    if env_endpoint:
        resolved_endpoint = env_endpoint
    if env_model:
        resolved_model = env_model

    if resolved_api_key and not resolved_endpoint:
        resolved_endpoint = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"
    if resolved_api_key and not resolved_model:
        resolved_model = "qwen-vl-max-latest"

    return resolved_endpoint, resolved_model, resolved_api_key


class DetectorSegNode(Node):
    def __init__(self) -> None:
        super().__init__("detector_seg_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("task_execution_status_topic", "/task/execution_status")
        self.declare_parameter("pick_status_topic", "/sim/task/pick_status")
        self.declare_parameter("detection_result_topic", "/perception/detection_result")
        self.declare_parameter(
            "candidate_visible_topic", "/sim/perception/target_candidate_visible"
        )
        self.declare_parameter("detection_debug_topic", "/perception/detection_debug")
        self.declare_parameter(
            "detection_debug_overlay_topic", "/perception/detection_debug_overlay"
        )
        # Phase8 defaults to the semantic-driven open-vocabulary segmentation path.
        self.declare_parameter("backend", "yoloe_local")
        self.declare_parameter("backend_url", "")
        self.declare_parameter("request_timeout_sec", 10.0)
        self.declare_parameter("max_inference_rate_hz", 2.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("open_vocab_model_path", "yoloe-11s-seg.pt")
        self.declare_parameter(
            "open_vocab_default_classes",
            [
                "cup",
                "bottle",
                "can",
                "container",
                "block",
                "cube",
                "charger",
                "power adapter",
            ],
        )
        self.declare_parameter("open_vocab_max_classes", 6)
        self.declare_parameter("scene_prompt_enabled", False)
        self.declare_parameter("scene_prompt_endpoint", "")
        self.declare_parameter("scene_prompt_model_name", "")
        self.declare_parameter("scene_prompt_api_key", "")
        self.declare_parameter("scene_prompt_request_timeout_sec", 20.0)
        self.declare_parameter("scene_prompt_resize_max_side_px", 896)
        self.declare_parameter("scene_prompt_jpeg_quality", 85)
        self.declare_parameter("scene_prompt_max_classes", 4)
        self.declare_parameter("scene_prompt_cache_path", "~/.cache/programme/scene_prompt_cache.json")
        self.declare_parameter("scene_prompt_refresh_on_scene_change", True)
        self.declare_parameter("scene_prompt_refresh_cooldown_sec", 5.0)
        self.declare_parameter("scene_prompt_change_confirm_frames", 3)
        self.declare_parameter("scene_prompt_change_settle_sec", 0.6)
        self.declare_parameter("scene_prompt_change_hash_threshold", 8)
        self.declare_parameter("scene_prompt_change_hist_threshold", 0.18)
        self.declare_parameter("scene_prompt_monitor_max_side_px", 96)
        self.declare_parameter("scene_prompt_track_change_hold_sec", 0.8)
        self.declare_parameter("ultralytics_model_path", "ros2_ws/yolo11s-seg.onnx")
        self.declare_parameter("ultralytics_runtime_preference", "auto")
        self.declare_parameter("ultralytics_device", "")
        self.declare_parameter("ultralytics_imgsz", 960)
        self.declare_parameter("ultralytics_startup_warmup_enabled", True)
        self.declare_parameter("ultralytics_startup_warmup_delay_sec", 0.25)
        self.declare_parameter("ultralytics_startup_warmup_runs", 2)
        self.declare_parameter("ultralytics_startup_warmup_pause_sec", 0.05)
        self.declare_parameter("ultralytics_roi_enabled", True)
        self.declare_parameter("ultralytics_roi_padding_scale", 1.0)
        self.declare_parameter("ultralytics_roi_min_size_px", 256)
        self.declare_parameter("ultralytics_roi_max_stale_sec", 2.0)
        self.declare_parameter("ultralytics_roi_full_frame_interval_sec", 1.25)
        self.declare_parameter("ultralytics_roi_min_track_hits", 2)
        self.declare_parameter("ultralytics_roi_require_semantic_task", True)
        self.declare_parameter("bbox_tracking_enabled", True)
        self.declare_parameter("bbox_tracking_max_rate_hz", 12.0)
        self.declare_parameter("bbox_tracking_max_stale_sec", 2.0)
        self.declare_parameter("bbox_tracking_search_margin_px", 96)
        self.declare_parameter("bbox_tracking_template_padding_px", 12)
        self.declare_parameter("bbox_tracking_min_template_px", 20)
        self.declare_parameter("bbox_tracking_match_threshold", 0.42)
        self.declare_parameter("bbox_tracking_template_update_threshold", 0.65)
        self.declare_parameter("mask_publish_rate_hz", 1.0)
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
        self.declare_parameter("vlm_relabel_focus_top_k", 1)
        self.declare_parameter("vlm_relabel_focus_track_cooldown_sec", 1.25)
        self.declare_parameter("vlm_relabel_focus_without_overview", True)
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
        self.declare_parameter("selection_conf_qwen_weight", 0.6)
        self.declare_parameter("selection_conf_yolo_weight", 0.25)
        self.declare_parameter("selection_conf_track_weight", 0.15)
        self.declare_parameter("selection_conf_smoothing_frames", 3)
        self.declare_parameter("vlm_relabel_default_semantic_confidence", 0.65)
        self.declare_parameter("suppress_robot_self_candidates", True)
        self.declare_parameter("robot_self_inference_mask_bottom_band_px", 40)
        self.declare_parameter("robot_self_bottom_touch_tolerance_px", 2)
        self.declare_parameter("robot_self_candidate_min_y_ratio", 0.90)
        self.declare_parameter("robot_self_candidate_max_height_px", 52)
        self.declare_parameter("robot_self_candidate_max_width_px", 96)
        self.declare_parameter("robot_self_candidate_max_mask_pixels", 2500)
        self.declare_parameter("log_interval_sec", 10.0)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.task_execution_status_topic = str(
            self.get_parameter("task_execution_status_topic").value
        )
        self.pick_status_topic = str(self.get_parameter("pick_status_topic").value)
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
        self.open_vocab_model_path = str(
            self.get_parameter("open_vocab_model_path").value
        ).strip()
        self.open_vocab_default_classes = [
            self._normalize_short_label(item, max_chars=80)
            for item in list(self.get_parameter("open_vocab_default_classes").value or [])
            if self._normalize_short_label(item, max_chars=80)
        ]
        self.open_vocab_max_classes = max(
            1, int(self.get_parameter("open_vocab_max_classes").value)
        )
        self.scene_prompt_enabled = bool(self.get_parameter("scene_prompt_enabled").value)
        self.scene_prompt_endpoint = str(
            self.get_parameter("scene_prompt_endpoint").value or ""
        ).strip()
        self.scene_prompt_model_name = str(
            self.get_parameter("scene_prompt_model_name").value or ""
        ).strip()
        self.scene_prompt_api_key = str(
            self.get_parameter("scene_prompt_api_key").value or ""
        ).strip()
        self.scene_prompt_request_timeout_sec = max(
            2.0, float(self.get_parameter("scene_prompt_request_timeout_sec").value)
        )
        self.scene_prompt_resize_max_side_px = max(
            256, int(self.get_parameter("scene_prompt_resize_max_side_px").value)
        )
        self.scene_prompt_jpeg_quality = max(
            50, min(100, int(self.get_parameter("scene_prompt_jpeg_quality").value))
        )
        self.scene_prompt_max_classes = max(
            1, int(self.get_parameter("scene_prompt_max_classes").value)
        )
        self.scene_prompt_cache_path = os.path.expanduser(
            str(self.get_parameter("scene_prompt_cache_path").value or "").strip()
        )
        self.scene_prompt_refresh_on_scene_change = bool(
            self.get_parameter("scene_prompt_refresh_on_scene_change").value
        )
        self.scene_prompt_refresh_cooldown_sec = max(
            0.0, float(self.get_parameter("scene_prompt_refresh_cooldown_sec").value)
        )
        self.scene_prompt_change_confirm_frames = max(
            1, int(self.get_parameter("scene_prompt_change_confirm_frames").value)
        )
        self.scene_prompt_change_settle_sec = max(
            0.0, float(self.get_parameter("scene_prompt_change_settle_sec").value)
        )
        self.scene_prompt_change_hash_threshold = max(
            1, int(self.get_parameter("scene_prompt_change_hash_threshold").value)
        )
        self.scene_prompt_change_hist_threshold = max(
            0.0,
            min(1.0, float(self.get_parameter("scene_prompt_change_hist_threshold").value)),
        )
        self.scene_prompt_monitor_max_side_px = max(
            32, int(self.get_parameter("scene_prompt_monitor_max_side_px").value)
        )
        self.scene_prompt_track_change_hold_sec = max(
            0.0, float(self.get_parameter("scene_prompt_track_change_hold_sec").value)
        )
        (
            self.scene_prompt_endpoint,
            self.scene_prompt_model_name,
            self.scene_prompt_api_key,
        ) = _resolve_runtime_model_config(
            endpoint=self.scene_prompt_endpoint,
            model_name=self.scene_prompt_model_name,
            api_key=self.scene_prompt_api_key,
        )
        self.ultralytics_model_path = str(
            self.get_parameter("ultralytics_model_path").value
        ).strip()
        self.ultralytics_runtime_preference = str(
            self.get_parameter("ultralytics_runtime_preference").value
        ).strip().lower()
        self.ultralytics_device = str(self.get_parameter("ultralytics_device").value).strip()
        self.ultralytics_imgsz = max(320, int(self.get_parameter("ultralytics_imgsz").value))
        self.ultralytics_startup_warmup_enabled = bool(
            self.get_parameter("ultralytics_startup_warmup_enabled").value
        )
        self.ultralytics_startup_warmup_delay_sec = max(
            0.0, float(self.get_parameter("ultralytics_startup_warmup_delay_sec").value)
        )
        self.ultralytics_startup_warmup_runs = max(
            1, int(self.get_parameter("ultralytics_startup_warmup_runs").value)
        )
        self.ultralytics_startup_warmup_pause_sec = max(
            0.0, float(self.get_parameter("ultralytics_startup_warmup_pause_sec").value)
        )
        self.ultralytics_roi_enabled = bool(
            self.get_parameter("ultralytics_roi_enabled").value
        )
        self.ultralytics_roi_padding_scale = max(
            0.0, float(self.get_parameter("ultralytics_roi_padding_scale").value)
        )
        self.ultralytics_roi_min_size_px = max(
            64, int(self.get_parameter("ultralytics_roi_min_size_px").value)
        )
        self.ultralytics_roi_max_stale_sec = max(
            0.1, float(self.get_parameter("ultralytics_roi_max_stale_sec").value)
        )
        self.ultralytics_roi_full_frame_interval_sec = max(
            0.1, float(self.get_parameter("ultralytics_roi_full_frame_interval_sec").value)
        )
        self.ultralytics_roi_min_track_hits = max(
            1, int(self.get_parameter("ultralytics_roi_min_track_hits").value)
        )
        self.ultralytics_roi_require_semantic_task = bool(
            self.get_parameter("ultralytics_roi_require_semantic_task").value
        )
        self.bbox_tracking_enabled = bool(self.get_parameter("bbox_tracking_enabled").value)
        self.bbox_tracking_max_rate_hz = max(
            0.5, float(self.get_parameter("bbox_tracking_max_rate_hz").value)
        )
        self.bbox_tracking_max_stale_sec = max(
            0.1, float(self.get_parameter("bbox_tracking_max_stale_sec").value)
        )
        self.bbox_tracking_search_margin_px = max(
            8, int(self.get_parameter("bbox_tracking_search_margin_px").value)
        )
        self.bbox_tracking_template_padding_px = max(
            0, int(self.get_parameter("bbox_tracking_template_padding_px").value)
        )
        self.bbox_tracking_min_template_px = max(
            8, int(self.get_parameter("bbox_tracking_min_template_px").value)
        )
        self.bbox_tracking_match_threshold = max(
            0.0, min(1.0, float(self.get_parameter("bbox_tracking_match_threshold").value))
        )
        self.bbox_tracking_template_update_threshold = max(
            0.0,
            min(1.0, float(self.get_parameter("bbox_tracking_template_update_threshold").value)),
        )
        self.mask_publish_rate_hz = max(
            0.1, float(self.get_parameter("mask_publish_rate_hz").value)
        )
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
        self.vlm_relabel_focus_top_k = max(
            1, int(self.get_parameter("vlm_relabel_focus_top_k").value)
        )
        self.vlm_relabel_focus_track_cooldown_sec = max(
            0.0,
            float(self.get_parameter("vlm_relabel_focus_track_cooldown_sec").value),
        )
        self.vlm_relabel_focus_without_overview = bool(
            self.get_parameter("vlm_relabel_focus_without_overview").value
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
        self.selection_conf_qwen_weight = max(
            0.0, float(self.get_parameter("selection_conf_qwen_weight").value)
        )
        self.selection_conf_yolo_weight = max(
            0.0, float(self.get_parameter("selection_conf_yolo_weight").value)
        )
        self.selection_conf_track_weight = max(
            0.0, float(self.get_parameter("selection_conf_track_weight").value)
        )
        selection_conf_weight_sum = (
            self.selection_conf_qwen_weight
            + self.selection_conf_yolo_weight
            + self.selection_conf_track_weight
        )
        if selection_conf_weight_sum <= 1e-6:
            self.selection_conf_qwen_weight = 0.6
            self.selection_conf_yolo_weight = 0.25
            self.selection_conf_track_weight = 0.15
            selection_conf_weight_sum = 1.0
        self.selection_conf_qwen_weight /= selection_conf_weight_sum
        self.selection_conf_yolo_weight /= selection_conf_weight_sum
        self.selection_conf_track_weight /= selection_conf_weight_sum
        self.selection_conf_smoothing_frames = max(
            1, int(self.get_parameter("selection_conf_smoothing_frames").value)
        )
        self.vlm_relabel_default_semantic_confidence = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter("vlm_relabel_default_semantic_confidence").value),
            ),
        )
        self.suppress_robot_self_candidates = bool(
            self.get_parameter("suppress_robot_self_candidates").value
        )
        self.robot_self_inference_mask_bottom_band_px = max(
            0, int(self.get_parameter("robot_self_inference_mask_bottom_band_px").value)
        )
        self.robot_self_bottom_touch_tolerance_px = max(
            0, int(self.get_parameter("robot_self_bottom_touch_tolerance_px").value)
        )
        self.robot_self_candidate_min_y_ratio = max(
            0.0,
            min(1.0, float(self.get_parameter("robot_self_candidate_min_y_ratio").value)),
        )
        self.robot_self_candidate_max_height_px = max(
            1, int(self.get_parameter("robot_self_candidate_max_height_px").value)
        )
        self.robot_self_candidate_max_width_px = max(
            1, int(self.get_parameter("robot_self_candidate_max_width_px").value)
        )
        self.robot_self_candidate_max_mask_pixels = max(
            1, int(self.get_parameter("robot_self_candidate_max_mask_pixels").value)
        )
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._session = requests.Session()
        self._vlm_session = requests.Session()
        self._request_lock = threading.Lock()
        self._semantic_lock = threading.Lock()
        self._color_lock = threading.Lock()
        self._scene_prompt_lock = threading.Lock()
        self._vlm_relabel_lock = threading.Lock()
        self._track_lock = threading.Lock()
        self._bbox_tracking_lock = threading.Lock()
        self._model_lock = threading.RLock()
        self._request_in_flight = False
        self._pending_request = False
        self._scene_prompt_request_in_flight = False
        self._scene_prompt_bootstrap_attempted = False
        self._scene_prompt_generated_at = 0.0
        self._scene_prompt_last_error = ""
        self._scene_prompt_cache: dict[str, Any] = {}
        self._scene_prompt_force_bootstrap_pending = False
        self._scene_prompt_last_refresh_reason = ""
        self._scene_prompt_last_refresh_started_at = 0.0
        self._scene_prompt_reference: Optional[dict[str, Any]] = None
        self._scene_change_observed_frames = 0
        self._scene_change_first_observed_at = 0.0
        self._scene_change_last_reason = ""
        self._scene_change_last_metrics: dict[str, Any] = {}
        self._vlm_relabel_in_flight = False
        self._vlm_relabel_last_started = 0.0
        self._tracks: dict[int, dict[str, Any]] = {}
        self._next_track_id = 1
        self._latest_color_msg: Optional[Image] = None
        self._semantic_task: Optional[SemanticTask] = None
        self._semantic_task_received_at = 0.0
        self._task_execution_phase = "idle"
        self._task_execution_active = False
        self._task_execution_updated_at = 0.0
        self._pick_phase = "idle"
        self._last_terminal_summary = ""
        self._last_candidate_summary = ""
        self._model = None
        self._resolved_ultralytics_model_path = ""
        self._ultralytics_runtime = "torch"
        self._effective_ultralytics_imgsz = 0
        self._open_vocab_classes_signature = ""
        self._startup_warmup_started = False
        self._startup_warmup_completed = False
        self._startup_warmup_failed_reason = ""
        self._roi_state: Optional[dict[str, Any]] = None
        self._last_full_frame_inference_at = 0.0
        self._last_roi_inference_at = 0.0
        self._bbox_tracking_state: Optional[dict[str, Any]] = None
        self._last_bbox_tracking_publish_at = 0.0
        self._last_bbox_tracking_frame_key: Optional[tuple[int, int, str]] = None
        self._last_mask_publish_at = 0.0
        self._last_selected_candidate_debug: Optional[dict[str, Any]] = None

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
        self.create_subscription(
            TaskExecutionStatus,
            self.task_execution_status_topic,
            self._on_task_execution_status,
            qos_reliable,
        )
        self.create_subscription(String, self.pick_status_topic, self._on_pick_status, qos_reliable)

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
        if self.backend in ("http_json", "ultralytics_local", "yoloe_local"):
            self.create_timer(1.0 / self.max_inference_rate_hz, self._maybe_run_inference)
        if self.backend in ("ultralytics_local", "yoloe_local") and self.bbox_tracking_enabled:
            self.create_timer(
                1.0 / self.bbox_tracking_max_rate_hz,
                self._maybe_publish_tracked_detection,
            )
        if self.backend in ("ultralytics_local", "yoloe_local"):
            self._prepare_ultralytics_runtime_env()
            self._log_ultralytics_runtime_status()
            self._maybe_start_ultralytics_startup_warmup()
        self._load_scene_prompt_cache_from_file()
        if self.scene_prompt_enabled:
            self._scene_prompt_force_bootstrap_pending = True

        self.get_logger().info(
            "detector_seg_node started: "
            f"backend={self.backend}, result={self.detection_result_topic}, "
            f"candidate_visible={self.candidate_visible_topic}, "
            f"scene_prompt={'on' if self.scene_prompt_enabled else 'off'}, "
            f"vlm_relabel={'on' if self.vlm_relabel_enabled else 'off'}"
        )

    def _log_ultralytics_runtime_status(self) -> None:
        try:
            import ultralytics  # noqa: F401
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"detector backend {self.backend} requires ultralytics, but it is unavailable: "
                f"{exc}. Install ultralytics before launching the segmentation detector."
            )
            return

        model_ref = self.open_vocab_model_path if self.backend == "yoloe_local" else self.ultralytics_model_path
        self.get_logger().info(
            f"{self.backend} backend ready: "
            f"model={model_ref}, device={self.ultralytics_device or 'auto'}, "
            f"runtime_pref={self.ultralytics_runtime_preference or 'auto'}"
        )

    def _prepare_ultralytics_runtime_env(self) -> None:
        os.environ.setdefault("ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS", "1")
        if self.backend == "yoloe_local" or not self._should_prefer_onnx_runtime():
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
            if not self._should_prefer_onnx_runtime():
                torch_path = model_path.with_suffix(".pt")
                if torch_path.exists():
                    self.get_logger().info(
                        "detector configured with ONNX path but GPU/Torch runtime is requested; "
                        f"using sibling PyTorch model {torch_path.name}"
                    )
                    return torch_path.as_posix(), "torch"
                self.get_logger().warn(
                    "detector requested GPU/Torch runtime, but only an ONNX model path is configured; "
                    "staying on ONNX inference"
                )
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

    def _resolve_open_vocab_model_reference(self) -> str:
        model_ref = str(self.open_vocab_model_path or "").strip() or "yoloe-11s-seg.pt"
        candidate = Path(model_ref).expanduser()
        if candidate.is_absolute() or "/" in model_ref or "\\" in model_ref:
            resolved = candidate.resolve()
            if not resolved.exists():
                raise FileNotFoundError(f"open-vocabulary model not found: {resolved}")
            return resolved.as_posix()
        candidate_in_cwd = (Path.cwd() / candidate).resolve()
        if candidate_in_cwd.exists():
            return candidate_in_cwd.as_posix()
        return model_ref

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
        self._maybe_start_scene_prompt_bootstrap(msg)
        if self.backend in ("http_json", "ultralytics_local", "yoloe_local"):
            self._pending_request = True

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        semantic_signature = self._semantic_signature(msg)
        with self._semantic_lock:
            self._semantic_task = msg
            self._semantic_task_received_at = time.time()
        if (
            isinstance(self._roi_state, dict)
            and semantic_signature
            and str(self._roi_state.get("semantic_signature", "") or "") != semantic_signature
        ):
            self._roi_state = None
        with self._bbox_tracking_lock:
            if (
                isinstance(self._bbox_tracking_state, dict)
                and semantic_signature
                and str(self._bbox_tracking_state.get("semantic_signature", "") or "")
                != semantic_signature
            ):
                self._bbox_tracking_state = None
        if self.backend == "disabled":
            self._publish_detection(
                self._build_empty_result(
                    task=msg.task,
                    target_label=msg.target_label,
                    reason="detector backend disabled; configure a segmentation detector backend",
                )
            )
        else:
            self._pending_request = True

    def _on_task_execution_status(self, msg: TaskExecutionStatus) -> None:
        phase = str(getattr(msg, "phase", "") or "").strip().lower() or "idle"
        active = bool(getattr(msg, "active", False))
        self._task_execution_phase = phase
        self._task_execution_active = active
        self._task_execution_updated_at = time.time()
        # Keep the current target-conditioned guidance alive across execution
        # failures so the detector can recover back to the same target lock
        # instead of immediately degrading to scene-level classes.
        if phase in {"completed", "cancelled"} and self._semantic_guidance_terminalized():
            self._clear_target_conditioned_runtime_state()

    def _on_pick_status(self, msg: String) -> None:
        phase = "idle"
        try:
            payload = json.loads(str(msg.data or "{}"))
            phase = str(payload.get("phase") or "").strip().lower() or "idle"
        except Exception:  # noqa: BLE001
            phase = "idle"
        self._pick_phase = phase

    def _semantic_guidance_terminalized(self) -> bool:
        if self._task_execution_active:
            return False
        return self._task_execution_phase in {"idle", "completed", "cancelled"}

    def _semantic_guidance_suppressed(
        self,
        semantic_task: Optional[SemanticTask],
    ) -> bool:
        if not self._semantic_task_requested(semantic_task):
            return False
        if self._task_execution_phase == "error":
            return False
        if not self._semantic_guidance_terminalized():
            return False
        return (
            float(self._task_execution_updated_at or 0.0) > 0.0
            and float(self._task_execution_updated_at or 0.0)
            >= float(self._semantic_task_received_at or 0.0)
        )

    def _effective_semantic_task(
        self,
        semantic_task: Optional[SemanticTask],
    ) -> Optional[SemanticTask]:
        if self._semantic_guidance_suppressed(semantic_task):
            return None
        return semantic_task

    def _clear_target_conditioned_runtime_state(self) -> None:
        self._roi_state = None
        self._last_full_frame_inference_at = 0.0
        self._last_roi_inference_at = 0.0
        self._last_selected_candidate_debug = None
        with self._bbox_tracking_lock:
            self._bbox_tracking_state = None
        self._last_bbox_tracking_publish_at = 0.0
        self._last_bbox_tracking_frame_key = None
        self._pending_request = True

    def _color_msg_frame_key(self, msg: Image) -> tuple[int, int, str]:
        stamp = getattr(msg, "header", None)
        stamp_msg = getattr(stamp, "stamp", None)
        sec = int(getattr(stamp_msg, "sec", 0) or 0)
        nanosec = int(getattr(stamp_msg, "nanosec", 0) or 0)
        frame_id = str(getattr(stamp, "frame_id", "") or "")
        return (sec, nanosec, frame_id)

    def _maybe_run_inference(self) -> None:
        if not self.enabled or self.backend not in ("http_json", "ultralytics_local", "yoloe_local"):
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
            semantic_task = self._effective_semantic_task(self._semantic_task)

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
            elif self.backend in ("ultralytics_local", "yoloe_local"):
                payload = self._run_ultralytics_inference(color_msg, image_rgb, semantic_task)
            else:
                payload = self._build_empty_result(
                    task=semantic_task.task if semantic_task is not None else "pick",
                    target_label=semantic_task.target_label if semantic_task is not None else "",
                    reason=f"unsupported detector backend: {self.backend}",
                    image_width=int(color_msg.width),
                    image_height=int(color_msg.height),
                    frame_id=color_msg.header.frame_id,
                    stamp=color_msg.header.stamp,
                )

            self._publish_detection(payload)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                "detector inference failed: "
                f"{exc}\n{traceback.format_exc()}"
            )
            self._publish_detection(
                self._build_empty_result(
                    task=semantic_task.task if semantic_task is not None else "pick",
                    target_label=semantic_task.target_label if semantic_task is not None else "",
                    reason=str(exc),
                    image_width=int(color_msg.width) if color_msg is not None else 0,
                    image_height=int(color_msg.height) if color_msg is not None else 0,
                    frame_id=color_msg.header.frame_id if color_msg is not None else "",
                    stamp=color_msg.header.stamp if color_msg is not None else None,
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
            stamp=color_msg.header.stamp,
            bbox_xyxy=bbox_xyxy,
            point_px=point_px,
            mask_u8=mask,
        )

    def _scene_prompt_cache_snapshot(self) -> dict[str, Any]:
        with self._scene_prompt_lock:
            snapshot = copy.deepcopy(self._scene_prompt_cache)
            if self._scene_prompt_generated_at > 0.0:
                snapshot["updated_at"] = float(self._scene_prompt_generated_at)
            if self._scene_prompt_last_error:
                snapshot["last_error"] = str(self._scene_prompt_last_error)
            if self._scene_prompt_last_refresh_reason:
                snapshot["last_refresh_reason"] = str(self._scene_prompt_last_refresh_reason)
            if self._scene_prompt_last_refresh_started_at > 0.0:
                snapshot["last_refresh_started_at"] = float(self._scene_prompt_last_refresh_started_at)
            snapshot["request_in_flight"] = bool(self._scene_prompt_request_in_flight)
            snapshot["force_refresh_pending"] = bool(self._scene_prompt_force_bootstrap_pending)
            if isinstance(self._scene_prompt_reference, dict):
                snapshot["reference"] = copy.deepcopy(self._scene_prompt_reference)
            if (
                self._scene_change_observed_frames > 0
                or self._scene_change_last_reason
                or self._scene_change_last_metrics
            ):
                snapshot["change_monitor"] = {
                    "observed_frames": int(self._scene_change_observed_frames),
                    "first_observed_at": float(self._scene_change_first_observed_at or 0.0),
                    "reason": str(self._scene_change_last_reason or ""),
                    "metrics": copy.deepcopy(self._scene_change_last_metrics),
                }
            return snapshot

    def _scene_prompt_spec(self) -> dict[str, Any]:
        payload = self._scene_prompt_cache_snapshot()
        return {
            "prompt_classes": self._coerce_short_text_list(
                payload.get("prompt_classes") or [],
                max_chars=80,
            ),
            "negative_labels": self._coerce_short_text_list(
                payload.get("negative_labels") or [],
                max_chars=80,
            ),
            "attributes": self._coerce_short_text_list(
                payload.get("attributes") or [],
                max_chars=48,
            ),
            "scene_summary": self._normalize_short_label(
                payload.get("scene_summary") or "",
                max_chars=240,
            ),
        }

    def _normalize_scene_prompt_reference(
        self,
        payload: Any,
    ) -> Optional[dict[str, Any]]:
        if not isinstance(payload, dict):
            return None
        normalized: dict[str, Any] = {}
        hash_hex = str(payload.get("hash_hex") or "").strip().lower()
        if hash_hex.startswith("0x"):
            hash_hex = hash_hex[2:]
        if hash_hex and all(ch in "0123456789abcdef" for ch in hash_hex):
            normalized["hash_hex"] = hash_hex[-16:].rjust(16, "0")
        histogram: list[float] = []
        for item in list(payload.get("histogram") or []):
            try:
                histogram.append(round(max(0.0, float(item)), 6))
            except Exception:  # noqa: BLE001
                continue
            if len(histogram) >= 16:
                break
        if histogram:
            normalized["histogram"] = histogram
        track_signature: list[str] = []
        seen_signature_keys: set[str] = set()
        for item in list(payload.get("track_signature") or []):
            text = self._normalize_short_label(item, max_chars=96)
            key = text.lower()
            if not text or key in seen_signature_keys:
                continue
            seen_signature_keys.add(key)
            track_signature.append(text)
        if track_signature:
            normalized["track_signature"] = track_signature
            normalized["track_count"] = len(track_signature)
        try:
            track_count = int(payload.get("track_count", normalized.get("track_count", 0)) or 0)
            if track_count >= 0:
                normalized["track_count"] = track_count
        except Exception:  # noqa: BLE001
            pass
        try:
            captured_at = float(payload.get("captured_at", 0.0) or 0.0)
            if captured_at > 0.0:
                normalized["captured_at"] = captured_at
        except Exception:  # noqa: BLE001
            pass
        if not normalized:
            return None
        return normalized

    def _load_scene_prompt_cache_from_file(self) -> None:
        cache_path_text = str(self.scene_prompt_cache_path or "").strip()
        if not cache_path_text:
            return
        cache_path = Path(cache_path_text).expanduser()
        if not cache_path.exists():
            return
        try:
            raw_payload = json.loads(cache_path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"failed to read scene prompt cache {cache_path}: {exc}")
            return
        if not isinstance(raw_payload, dict):
            self.get_logger().warn(f"scene prompt cache {cache_path} did not contain a JSON object")
            return
        prompt_payload = raw_payload.get("prompt") if isinstance(raw_payload.get("prompt"), dict) else raw_payload
        try:
            normalized_prompt = self._normalize_scene_prompt_payload(dict(prompt_payload))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"scene prompt cache {cache_path} was invalid: {exc}")
            return
        reference = self._normalize_scene_prompt_reference(raw_payload.get("reference"))
        updated_at = float(
            prompt_payload.get("updated_at", raw_payload.get("saved_at", time.time())) or time.time()
        )
        normalized_prompt["updated_at"] = updated_at
        with self._scene_prompt_lock:
            self._scene_prompt_cache = normalized_prompt
            self._scene_prompt_generated_at = updated_at
            self._scene_prompt_reference = reference
            self._scene_prompt_last_error = ""
            self._scene_prompt_last_refresh_reason = str(
                raw_payload.get("last_refresh_reason") or ""
            ).strip()
        self.get_logger().info(
            "loaded scene prompt cache: "
            + ", ".join(normalized_prompt.get("prompt_classes", []))
        )

    def _save_scene_prompt_cache_to_file(
        self,
        payload: dict[str, Any],
        *,
        reference: Optional[dict[str, Any]],
        reason: str,
    ) -> None:
        cache_path_text = str(self.scene_prompt_cache_path or "").strip()
        if not cache_path_text:
            return
        cache_path = Path(cache_path_text).expanduser()
        payload_doc = copy.deepcopy(payload)
        reference_doc = self._normalize_scene_prompt_reference(reference)
        document: dict[str, Any] = {
            "version": 1,
            "saved_at": float(time.time()),
            "prompt": payload_doc,
        }
        if reference_doc is not None:
            document["reference"] = reference_doc
        if reason:
            document["last_refresh_reason"] = str(reason)
        try:
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            temp_path = cache_path.with_name(cache_path.name + ".tmp")
            temp_path.write_text(
                json.dumps(document, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            temp_path.replace(cache_path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"failed to persist scene prompt cache {cache_path}: {exc}")

    def _reset_scene_change_observation(self) -> None:
        with self._scene_prompt_lock:
            self._scene_change_observed_frames = 0
            self._scene_change_first_observed_at = 0.0
            self._scene_change_last_reason = ""
            self._scene_change_last_metrics = {}

    def _scene_monitor_gray_image(self, image_rgb: np.ndarray) -> np.ndarray:
        masked_rgb = self._mask_robot_self_inference_region(image_rgb)
        monitor_rgb = np.asarray(masked_rgb, dtype=np.uint8)
        max_side = max(int(monitor_rgb.shape[0]), int(monitor_rgb.shape[1]))
        if max_side > self.scene_prompt_monitor_max_side_px:
            scale = float(self.scene_prompt_monitor_max_side_px) / float(max_side)
            new_width = max(8, int(round(float(monitor_rgb.shape[1]) * scale)))
            new_height = max(8, int(round(float(monitor_rgb.shape[0]) * scale)))
            monitor_rgb = cv2.resize(
                monitor_rgb,
                (new_width, new_height),
                interpolation=cv2.INTER_AREA,
            )
        return cv2.cvtColor(monitor_rgb, cv2.COLOR_RGB2GRAY)

    def _scene_monitor_hash_hex(self, image_gray: np.ndarray) -> str:
        resized = cv2.resize(image_gray, (9, 8), interpolation=cv2.INTER_AREA)
        diff = resized[:, 1:] > resized[:, :-1]
        bits = 0
        for value in diff.flatten():
            bits = (bits << 1) | int(bool(value))
        return f"{bits:016x}"

    def _scene_monitor_histogram(self, image_gray: np.ndarray) -> list[float]:
        hist = cv2.calcHist([image_gray], [0], None, [16], [0, 256]).flatten()
        total = float(hist.sum())
        if total > 0.0:
            hist = hist / total
        return [round(float(value), 6) for value in hist.tolist()]

    def _scene_monitor_hash_distance(self, left_hash: Any, right_hash: Any) -> int:
        try:
            left_value = int(str(left_hash or "0").strip(), 16)
            right_value = int(str(right_hash or "0").strip(), 16)
        except Exception:  # noqa: BLE001
            return 0
        return int((left_value ^ right_value).bit_count())

    def _scene_monitor_hist_distance(
        self,
        left_hist: list[float],
        right_hist: list[float],
    ) -> float:
        if not left_hist or not right_hist:
            return 0.0
        size = min(len(left_hist), len(right_hist))
        distance = 0.0
        for index in range(size):
            distance += abs(float(left_hist[index]) - float(right_hist[index]))
        return min(1.0, distance * 0.5)

    def _scene_track_signature(
        self,
        *,
        image_width: int,
        image_height: int,
        now_sec: float,
    ) -> list[str]:
        recent_window_sec = max(
            self.scene_prompt_track_change_hold_sec,
            1.5 / max(0.5, float(self.max_inference_rate_hz)),
        )
        with self._track_lock:
            tracks = [copy.deepcopy(track) for track in self._tracks.values() if isinstance(track, dict)]
        signature: list[str] = []
        for track in tracks:
            if int(track.get("seen_count", 0) or 0) < self.track_stable_frames_required:
                continue
            last_present_at = max(
                float(track.get("last_seen", 0.0) or 0.0),
                float(track.get("last_tracked_at", 0.0) or 0.0),
            )
            if (now_sec - last_present_at) > recent_window_sec:
                continue
            bbox_xyxy = self._normalize_bbox_xyxy(track.get("bbox_xyxy"), image_width, image_height)
            center_xy = self._bbox_center(bbox_xyxy)
            if bbox_xyxy is None or center_xy is None:
                continue
            x_bin = min(7, max(0, int((float(center_xy[0]) / float(max(1, image_width))) * 8.0)))
            y_bin = min(7, max(0, int((float(center_xy[1]) / float(max(1, image_height))) * 8.0)))
            bbox_area = max(1, int(bbox_xyxy[2] - bbox_xyxy[0])) * max(1, int(bbox_xyxy[3] - bbox_xyxy[1]))
            area_ratio = float(bbox_area) / float(max(1, image_width * image_height))
            area_bin = min(7, max(0, int(round(min(1.0, area_ratio) * 7.0))))
            label = self._normalized_label_key(track.get("canonical_label") or track.get("raw_label"))
            if not label or label in GENERIC_VLM_LABELS:
                label = "unknown"
            signature.append(f"{label}:{x_bin}:{y_bin}:{area_bin}")
        signature.sort()
        return signature

    def _scene_monitor_metrics_from_image(self, image_rgb: np.ndarray) -> dict[str, Any]:
        now_sec = time.time()
        image_height = int(image_rgb.shape[0])
        image_width = int(image_rgb.shape[1])
        image_gray = self._scene_monitor_gray_image(image_rgb)
        track_signature = self._scene_track_signature(
            image_width=image_width,
            image_height=image_height,
            now_sec=now_sec,
        )
        return {
            "hash_hex": self._scene_monitor_hash_hex(image_gray),
            "histogram": self._scene_monitor_histogram(image_gray),
            "track_signature": track_signature,
            "track_count": len(track_signature),
            "captured_at": now_sec,
        }

    def _scene_prompt_change_reason(
        self,
        scene_metrics: dict[str, Any],
    ) -> tuple[str, dict[str, Any]]:
        with self._scene_prompt_lock:
            reference = copy.deepcopy(self._scene_prompt_reference)
        normalized_reference = self._normalize_scene_prompt_reference(reference)
        if normalized_reference is None:
            return "", {}
        current_reference = self._normalize_scene_prompt_reference(scene_metrics) or {}
        hash_distance = self._scene_monitor_hash_distance(
            current_reference.get("hash_hex"),
            normalized_reference.get("hash_hex"),
        )
        hist_distance = self._scene_monitor_hist_distance(
            list(current_reference.get("histogram") or []),
            list(normalized_reference.get("histogram") or []),
        )
        current_track_signature = list(current_reference.get("track_signature") or [])
        reference_track_signature = list(normalized_reference.get("track_signature") or [])
        current_track_count = int(current_reference.get("track_count", len(current_track_signature)) or 0)
        reference_track_count = int(
            normalized_reference.get("track_count", len(reference_track_signature)) or 0
        )
        image_changed = (
            hash_distance >= self.scene_prompt_change_hash_threshold
            and hist_distance >= self.scene_prompt_change_hist_threshold
        ) or hash_distance >= (self.scene_prompt_change_hash_threshold * 2)
        track_changed = (
            bool(current_track_signature or reference_track_signature)
            and current_track_signature != reference_track_signature
        ) or current_track_count != reference_track_count
        reasons: list[str] = []
        if image_changed:
            reasons.append("image")
        if track_changed:
            reasons.append("track_set")
        metrics = {
            "hash_distance": int(hash_distance),
            "hist_distance": round(float(hist_distance), 4),
            "track_count": int(current_track_count),
            "reference_track_count": int(reference_track_count),
        }
        if reasons:
            metrics["track_signature"] = current_track_signature
            metrics["reference_track_signature"] = reference_track_signature
        return "+".join(reasons), metrics

    def _scene_change_ready(
        self,
        *,
        reason: str,
        metrics: dict[str, Any],
        now_sec: float,
    ) -> bool:
        with self._scene_prompt_lock:
            if not reason:
                self._scene_change_observed_frames = 0
                self._scene_change_first_observed_at = 0.0
                self._scene_change_last_reason = ""
                self._scene_change_last_metrics = {}
                return False
            if self._scene_change_observed_frames <= 0:
                self._scene_change_first_observed_at = now_sec
                self._scene_change_observed_frames = 1
            else:
                self._scene_change_observed_frames += 1
            self._scene_change_last_reason = str(reason)
            self._scene_change_last_metrics = copy.deepcopy(metrics)
            return bool(
                self._scene_change_observed_frames >= self.scene_prompt_change_confirm_frames
                and (now_sec - float(self._scene_change_first_observed_at or now_sec))
                >= self.scene_prompt_change_settle_sec
            )

    def _normalize_scene_prompt_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        raw_classes = (
            payload.get("scene_prompt_classes")
            or payload.get("prompt_classes")
            or payload.get("classes")
            or []
        )
        blocked_tokens = {
            "arm",
            "base",
            "gripper",
            "robot",
            "shadow",
            "shadows",
            "surface",
            "table",
            "ui",
        }
        prompt_classes: list[str] = []
        seen_prompt_keys: set[str] = set()
        for item in list(raw_classes or []):
            prompt = self._normalize_short_label(item, max_chars=80)
            key = prompt.lower()
            tokens = set(self._semantic_tokens(key))
            if (
                not prompt
                or key in seen_prompt_keys
                or key in GENERIC_VLM_LABELS
                or self._scene_prompt_label_is_too_generic(prompt)
                or bool(tokens.intersection(blocked_tokens))
            ):
                continue
            seen_prompt_keys.add(key)
            prompt_classes.append(prompt)
            if len(prompt_classes) >= self.scene_prompt_max_classes:
                break
        if not prompt_classes:
            raise ValueError("scene prompt response did not contain usable prompt classes")
        negative_labels = self._coerce_short_text_list(
            payload.get("negative_labels") or payload.get("excluded_labels") or [],
            max_chars=80,
        )[:8]
        attributes = self._coerce_short_text_list(
            payload.get("attributes") or payload.get("scene_attributes") or [],
            max_chars=48,
        )[:8]
        scene_summary = self._normalize_short_label(
            payload.get("scene_summary") or payload.get("summary") or "",
            max_chars=240,
        )
        return {
            "prompt_classes": prompt_classes,
            "negative_labels": negative_labels,
            "attributes": attributes,
            "scene_summary": scene_summary,
            "updated_at": float(time.time()),
        }

    def _scene_prompt_label_is_too_generic(self, value: Any) -> bool:
        normalized = self._normalized_label_key(value)
        if not normalized:
            return True
        tokens = self._semantic_tokens(normalized)
        if not tokens:
            return True
        return tokens[-1] in {"object", "item", "thing"}

    def _query_scene_prompt(self, color_msg: Image) -> dict[str, Any]:
        if not self.scene_prompt_endpoint or not self.scene_prompt_model_name:
            raise ValueError("scene prompt model config is incomplete")
        image_rgb = decode_color_image(color_msg)
        if image_rgb is None:
            raise ValueError(f"unsupported color image encoding: {color_msg.encoding}")
        image_rgb = np.asarray(image_rgb, dtype=np.uint8)
        max_side = max(int(image_rgb.shape[0]), int(image_rgb.shape[1]))
        if max_side > self.scene_prompt_resize_max_side_px:
            scale = float(self.scene_prompt_resize_max_side_px) / float(max_side)
            new_width = max(1, int(round(float(image_rgb.shape[1]) * scale)))
            new_height = max(1, int(round(float(image_rgb.shape[0]) * scale)))
            image_rgb = cv2.resize(
                image_rgb,
                (new_width, new_height),
                interpolation=cv2.INTER_AREA,
            )
        image_b64 = encode_image_to_base64_jpeg(image_rgb, self.scene_prompt_jpeg_quality)

        headers = {"Content-Type": "application/json"}
        api_key = _normalized_secret(self.scene_prompt_api_key)
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"

        user_content = [
            {
                "type": "text",
                "text": (
                    "Generate a compact open-vocabulary detector prompt set for this scene. "
                    "Return 3 to 4 short English detector phrases that together cover the salient "
                    "pickable foreground objects. Include color-plus-shape phrases when visually obvious. "
                    "Also include optional negative_labels for obvious distractors such as robot gripper."
                ),
            },
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
            },
        ]
        payload = {
            "model": self.scene_prompt_model_name,
            "messages": [
                {
                    "role": "system",
                    "content": [{"type": "text", "text": SCENE_PROMPT_SYSTEM_PROMPT}],
                },
                {"role": "user", "content": user_content},
            ],
            "temperature": 0.0,
            "max_tokens": 192,
            "response_format": {"type": "json_object"},
        }
        response = self._vlm_session.post(
            self.scene_prompt_endpoint,
            headers=headers,
            json=payload,
            timeout=self.scene_prompt_request_timeout_sec,
        )
        if response.status_code in {400, 404, 422}:
            fallback_payload = dict(payload)
            fallback_payload.pop("response_format", None)
            response = self._vlm_session.post(
                self.scene_prompt_endpoint,
                headers=headers,
                json=fallback_payload,
                timeout=self.scene_prompt_request_timeout_sec,
            )
        response.raise_for_status()
        parsed = extract_first_json_object(extract_message_text(response.json()))
        return self._normalize_scene_prompt_payload(parsed)

    def _run_scene_prompt_bootstrap(
        self,
        color_msg: Image,
        *,
        reason: str,
        scene_metrics: Optional[dict[str, Any]],
    ) -> None:
        try:
            payload = self._query_scene_prompt(color_msg)
            reference = self._normalize_scene_prompt_reference(scene_metrics)
            if reference is None:
                image_rgb = decode_color_image(color_msg)
                if image_rgb is not None:
                    reference = self._normalize_scene_prompt_reference(
                        self._scene_monitor_metrics_from_image(np.asarray(image_rgb, dtype=np.uint8))
                    )
            with self._scene_prompt_lock:
                self._scene_prompt_cache = payload
                self._scene_prompt_generated_at = float(payload.get("updated_at", time.time()) or time.time())
                self._scene_prompt_last_error = ""
                self._scene_prompt_force_bootstrap_pending = False
                self._scene_prompt_last_refresh_reason = str(reason or "")
                if reference is not None:
                    self._scene_prompt_reference = reference
            self._save_scene_prompt_cache_to_file(
                payload,
                reference=reference,
                reason=reason,
            )
            self._reset_scene_change_observation()
            self.get_logger().info(
                "scene prompt ready"
                + (f" ({reason})" if reason else "")
                + ": "
                + ", ".join(payload.get("prompt_classes", []))
            )
            self._pending_request = True
        except Exception as exc:  # noqa: BLE001
            with self._scene_prompt_lock:
                self._scene_prompt_last_error = str(exc)
                if not self._scene_prompt_cache:
                    self._scene_prompt_force_bootstrap_pending = True
            self.get_logger().warn(f"scene prompt refresh failed ({reason or 'startup'}): {exc}")
        finally:
            with self._scene_prompt_lock:
                self._scene_prompt_request_in_flight = False

    def _maybe_start_scene_prompt_refresh(
        self,
        *,
        color_msg: Image,
        reason: str,
        scene_metrics: Optional[dict[str, Any]],
    ) -> bool:
        if not self.scene_prompt_endpoint or not self.scene_prompt_model_name:
            with self._scene_prompt_lock:
                should_warn = not self._scene_prompt_bootstrap_attempted
                self._scene_prompt_bootstrap_attempted = True
                self._scene_prompt_last_error = "scene prompt model config is incomplete"
                self._scene_prompt_force_bootstrap_pending = False
            if should_warn:
                self.get_logger().warn(
                    "scene prompt refresh skipped: missing model endpoint or model name"
                )
            return False
        now_sec = time.time()
        with self._scene_prompt_lock:
            last_refresh_anchor = max(
                float(self._scene_prompt_generated_at or 0.0),
                float(self._scene_prompt_last_refresh_started_at or 0.0),
            )
            if self._scene_prompt_request_in_flight:
                return False
            if (
                self.scene_prompt_refresh_cooldown_sec > 0.0
                and last_refresh_anchor > 0.0
                and (now_sec - last_refresh_anchor) < self.scene_prompt_refresh_cooldown_sec
            ):
                return False
            self._scene_prompt_request_in_flight = True
            self._scene_prompt_bootstrap_attempted = True
            self._scene_prompt_force_bootstrap_pending = False
            self._scene_prompt_last_refresh_reason = str(reason or "")
            self._scene_prompt_last_refresh_started_at = now_sec
            self._scene_change_observed_frames = 0
            self._scene_change_first_observed_at = 0.0
            self._scene_change_last_reason = ""
            self._scene_change_last_metrics = {}
        threading.Thread(
            target=self._run_scene_prompt_bootstrap,
            kwargs={
                "color_msg": color_msg,
                "reason": reason,
                "scene_metrics": copy.deepcopy(scene_metrics)
                if isinstance(scene_metrics, dict)
                else None,
            },
            daemon=True,
        ).start()
        return True

    def _maybe_start_scene_prompt_bootstrap(self, color_msg: Optional[Image] = None) -> None:
        if (
            self.backend != "yoloe_local"
            or not self.scene_prompt_enabled
        ):
            return
        if color_msg is None:
            with self._color_lock:
                color_msg = self._latest_color_msg
        if color_msg is None:
            return
        image_rgb = decode_color_image(color_msg)
        if image_rgb is None:
            return
        scene_metrics = self._scene_monitor_metrics_from_image(
            np.asarray(image_rgb, dtype=np.uint8)
        )
        with self._semantic_lock:
            semantic_task = self._effective_semantic_task(self._semantic_task)
        refresh_suspended = bool(
            self._semantic_task_requested(semantic_task)
            or str(self._pick_phase or "").strip().lower() not in {"", "idle"}
        )
        with self._scene_prompt_lock:
            has_cache = bool(self._scene_prompt_cache)
            if has_cache and self._scene_prompt_reference is None:
                self._scene_prompt_reference = self._normalize_scene_prompt_reference(scene_metrics)
            force_refresh_pending = bool(self._scene_prompt_force_bootstrap_pending)
        if refresh_suspended and has_cache:
            self._reset_scene_change_observation()
            return
        if force_refresh_pending:
            self._maybe_start_scene_prompt_refresh(
                color_msg=color_msg,
                reason="startup" if not has_cache else "scene_refresh_pending",
                scene_metrics=scene_metrics,
            )
            return
        if not has_cache or not self.scene_prompt_refresh_on_scene_change:
            self._reset_scene_change_observation()
            return
        reason, metrics = self._scene_prompt_change_reason(scene_metrics)
        if not reason:
            self._reset_scene_change_observation()
            return
        if self._scene_change_ready(reason=reason, metrics=metrics, now_sec=time.time()):
            self._maybe_start_scene_prompt_refresh(
                color_msg=color_msg,
                reason=reason,
                scene_metrics=scene_metrics,
            )
        return

    def _maybe_start_ultralytics_startup_warmup(self) -> None:
        if (
            self.backend not in ("ultralytics_local", "yoloe_local")
            or not self.ultralytics_startup_warmup_enabled
            or self._startup_warmup_started
        ):
            return
        self._startup_warmup_started = True
        threading.Thread(
            target=self._run_ultralytics_startup_warmup,
            daemon=True,
        ).start()

    def _run_ultralytics_startup_warmup(self) -> None:
        delay_sec = float(self.ultralytics_startup_warmup_delay_sec)
        if delay_sec > 0.0:
            time.sleep(delay_sec)
        try:
            model = self._ensure_ultralytics_model()
            warmup_size = int(self._effective_ultralytics_imgsz or self.ultralytics_imgsz)
            warmup_rgb = np.zeros((warmup_size, warmup_size, 3), dtype=np.uint8)
            self.get_logger().info(
                "ultralytics startup warmup started: "
                f"runs={self.ultralytics_startup_warmup_runs}, "
                f"imgsz={warmup_size}, runtime={self._ultralytics_runtime}"
            )
            for warmup_index in range(self.ultralytics_startup_warmup_runs):
                self._predict_ultralytics(model, warmup_rgb)
                if (
                    warmup_index + 1 < self.ultralytics_startup_warmup_runs
                    and self.ultralytics_startup_warmup_pause_sec > 0.0
                ):
                    time.sleep(self.ultralytics_startup_warmup_pause_sec)
            self._startup_warmup_completed = True
            self.get_logger().info("ultralytics startup warmup complete")
        except Exception as exc:  # noqa: BLE001
            self._startup_warmup_failed_reason = str(exc)
            self.get_logger().warn(f"ultralytics startup warmup failed: {exc}")

    def _ensure_ultralytics_model(self):
        with self._model_lock:
            if self._model is not None:
                return self._model
            if self.backend == "yoloe_local":
                try:
                    from ultralytics.models import YOLOE
                except Exception as exc:  # noqa: BLE001
                    raise RuntimeError(
                        "ultralytics YOLOE is unavailable; install/upgrade ultralytics before using backend=yoloe_local"
                    ) from exc
                self._resolved_ultralytics_model_path = self._resolve_open_vocab_model_reference()
                self._ultralytics_runtime = "torch"
                self._effective_ultralytics_imgsz = int(self.ultralytics_imgsz)
                self._model = YOLOE(self._resolved_ultralytics_model_path, task="segment")
            else:
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
                f"backend={self.backend}, runtime={self._ultralytics_runtime}, "
                f"model={self._resolved_ultralytics_model_path}, "
                f"imgsz={self._effective_ultralytics_imgsz}"
            )
            return self._model

    def _predict_ultralytics(
        self,
        model: Any,
        detector_input_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask] = None,
    ) -> list[Any]:
        # Ultralytics expects numpy-image inputs in OpenCV's BGR channel order.
        # The ROS decode path above normalizes images to RGB, so convert here before
        # handing the array to model.predict(). File-path sources do not need this.
        detector_input_bgr = cv2.cvtColor(detector_input_rgb, cv2.COLOR_RGB2BGR)
        prediction_kwargs: dict[str, Any] = {
            "source": detector_input_bgr,
            "imgsz": self._effective_ultralytics_imgsz or self.ultralytics_imgsz,
            "conf": min(
                self.confidence_threshold,
                self.debug_candidate_confidence_floor,
                self.semantic_match_confidence_floor,
            ),
            "iou": self.iou_threshold,
            "verbose": False,
        }
        if self.backend == "yoloe_local":
            class_names = self._open_vocab_prompt_classes(semantic_task)
            class_signature = "|".join(class_names)
            with self._model_lock:
                if class_signature != self._open_vocab_classes_signature:
                    model.set_classes(class_names)
                    self._open_vocab_classes_signature = class_signature
                    self.get_logger().info(
                        "open-vocabulary classes updated: " + ", ".join(class_names)
                    )
                if self.ultralytics_device:
                    prediction_kwargs["device"] = self.ultralytics_device
                return model.predict(**prediction_kwargs)
        if self._ultralytics_runtime != "onnx" and self.ultralytics_device:
            prediction_kwargs["device"] = self.ultralytics_device
        with self._model_lock:
            return model.predict(**prediction_kwargs)

    def _normalize_bbox_xyxy(
        self,
        bbox_xyxy: Any,
        image_width: int,
        image_height: int,
    ) -> Optional[list[int]]:
        if not isinstance(bbox_xyxy, (list, tuple)) or len(bbox_xyxy) != 4:
            return None
        try:
            values = [int(value) for value in bbox_xyxy]
        except Exception:  # noqa: BLE001
            return None
        return coerce_bbox(values, image_width, image_height)

    def _expand_roi_bbox(
        self,
        bbox_xyxy: Any,
        image_width: int,
        image_height: int,
    ) -> Optional[list[int]]:
        normalized_bbox = self._normalize_bbox_xyxy(bbox_xyxy, image_width, image_height)
        if normalized_bbox is None:
            return None
        x1, y1, x2, y2 = normalized_bbox
        bbox_width = max(1, x2 - x1)
        bbox_height = max(1, y2 - y1)
        target_width = min(
            image_width,
            max(
                self.ultralytics_roi_min_size_px,
                int(round(float(bbox_width) * (1.0 + self.ultralytics_roi_padding_scale))),
            ),
        )
        target_height = min(
            image_height,
            max(
                self.ultralytics_roi_min_size_px,
                int(round(float(bbox_height) * (1.0 + self.ultralytics_roi_padding_scale))),
            ),
        )
        center_x = (float(x1) + float(x2)) * 0.5
        center_y = (float(y1) + float(y2)) * 0.5
        roi_x1 = int(round(center_x - float(target_width) * 0.5))
        roi_y1 = int(round(center_y - float(target_height) * 0.5))
        roi_x2 = roi_x1 + int(target_width)
        roi_y2 = roi_y1 + int(target_height)
        if roi_x1 < 0:
            roi_x2 -= roi_x1
            roi_x1 = 0
        if roi_y1 < 0:
            roi_y2 -= roi_y1
            roi_y1 = 0
        if roi_x2 > image_width:
            roi_x1 -= roi_x2 - image_width
            roi_x2 = image_width
        if roi_y2 > image_height:
            roi_y1 -= roi_y2 - image_height
            roi_y2 = image_height
        roi_bbox = [
            max(0, roi_x1),
            max(0, roi_y1),
            min(image_width, max(max(0, roi_x1) + 1, roi_x2)),
            min(image_height, max(max(0, roi_y1) + 1, roi_y2)),
        ]
        return self._normalize_bbox_xyxy(roi_bbox, image_width, image_height)

    def _select_ultralytics_roi_spec(
        self,
        image_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask],
        now_sec: float,
    ) -> Optional[dict[str, Any]]:
        if not self.ultralytics_roi_enabled:
            return None
        if self.ultralytics_roi_require_semantic_task and not self._semantic_task_requested(
            semantic_task
        ):
            return None
        if (now_sec - float(self._last_full_frame_inference_at or 0.0)) >= float(
            self.ultralytics_roi_full_frame_interval_sec
        ):
            return None
        roi_state = self._roi_state
        if not isinstance(roi_state, dict):
            return None
        if (now_sec - float(roi_state.get("updated_at", 0.0) or 0.0)) > float(
            self.ultralytics_roi_max_stale_sec
        ):
            return None
        semantic_signature = self._semantic_signature(semantic_task)
        stored_signature = str(roi_state.get("semantic_signature", "") or "")
        if semantic_signature and stored_signature and semantic_signature != stored_signature:
            return None

        bbox_xyxy = list(roi_state.get("bbox_xyxy") or [])
        track_hits = int(roi_state.get("track_seen_count", 0) or 0)
        track_id = int(roi_state.get("track_id", -1) or -1)
        if track_id >= 0:
            with self._track_lock:
                track = self._tracks.get(track_id)
                if not isinstance(track, dict):
                    return None
                if (now_sec - float(track.get("last_seen", 0.0) or 0.0)) > float(
                    self.ultralytics_roi_max_stale_sec
                ):
                    return None
                if self._semantic_task_requested(semantic_task) and not self._track_matches_semantic_target(
                    track, semantic_task
                ):
                    return None
                bbox_xyxy = list(track.get("bbox_xyxy") or bbox_xyxy)
                track_hits = int(track.get("seen_count", track_hits) or track_hits)
        if track_hits < self.ultralytics_roi_min_track_hits:
            return None
        roi_bbox_xyxy = self._expand_roi_bbox(
            bbox_xyxy,
            int(image_rgb.shape[1]),
            int(image_rgb.shape[0]),
        )
        if roi_bbox_xyxy is None:
            return None
        return {
            "bbox_xyxy": roi_bbox_xyxy,
            "track_id": track_id,
        }

    def _translate_bbox_from_roi(
        self,
        bbox_xyxy: Optional[list[int]],
        roi_bbox_xyxy: list[int],
        image_width: int,
        image_height: int,
    ) -> Optional[list[int]]:
        if bbox_xyxy is None:
            return None
        roi_x1, roi_y1, _, _ = roi_bbox_xyxy
        translated = [
            int(bbox_xyxy[0]) + int(roi_x1),
            int(bbox_xyxy[1]) + int(roi_y1),
            int(bbox_xyxy[2]) + int(roi_x1),
            int(bbox_xyxy[3]) + int(roi_y1),
        ]
        return coerce_bbox(translated, image_width, image_height)

    def _project_mask_from_roi(
        self,
        mask_u8: Optional[np.ndarray],
        roi_bbox_xyxy: list[int],
        full_shape: tuple[int, int],
    ) -> Optional[np.ndarray]:
        if not isinstance(mask_u8, np.ndarray):
            return None
        roi_x1, roi_y1, roi_x2, roi_y2 = [int(value) for value in roi_bbox_xyxy]
        roi_width = max(1, roi_x2 - roi_x1)
        roi_height = max(1, roi_y2 - roi_y1)
        if mask_u8.shape[:2] != (roi_height, roi_width):
            mask_u8 = cv2.resize(
                mask_u8,
                (roi_width, roi_height),
                interpolation=cv2.INTER_NEAREST,
            )
        projected = np.zeros(full_shape, dtype=np.uint8)
        projected[roi_y1:roi_y2, roi_x1:roi_x2] = mask_u8
        return projected

    def _update_roi_state_from_candidate(
        self,
        selected_candidate: Optional[dict[str, Any]],
        semantic_task: Optional[SemanticTask],
        now_sec: float,
    ) -> None:
        if not self.ultralytics_roi_enabled or not isinstance(selected_candidate, dict):
            return
        bbox_xyxy = self._normalize_bbox_xyxy(
            selected_candidate.get("bbox_xyxy"),
            int(selected_candidate.get("image_width", 0) or 0),
            int(selected_candidate.get("image_height", 0) or 0),
        )
        if bbox_xyxy is None:
            return
        self._roi_state = {
            "bbox_xyxy": bbox_xyxy,
            "track_id": int(selected_candidate.get("track_id", -1) or -1),
            "track_seen_count": int(selected_candidate.get("track_seen_count", 0) or 0),
            "semantic_signature": self._semantic_signature(semantic_task),
            "updated_at": float(now_sec),
        }

    def _extract_tracking_template(
        self,
        image_rgb: np.ndarray,
        bbox_xyxy: list[int],
    ) -> Optional[dict[str, Any]]:
        normalized_bbox = self._normalize_bbox_xyxy(
            bbox_xyxy,
            int(image_rgb.shape[1]),
            int(image_rgb.shape[0]),
        )
        if normalized_bbox is None:
            return None
        x1, y1, x2, y2 = normalized_bbox
        pad = int(self.bbox_tracking_template_padding_px)
        tpl_x1 = max(0, x1 - pad)
        tpl_y1 = max(0, y1 - pad)
        tpl_x2 = min(int(image_rgb.shape[1]), x2 + pad)
        tpl_y2 = min(int(image_rgb.shape[0]), y2 + pad)
        if tpl_x2 <= tpl_x1 or tpl_y2 <= tpl_y1:
            return None
        template_rgb = np.asarray(image_rgb[tpl_y1:tpl_y2, tpl_x1:tpl_x2], dtype=np.uint8).copy()
        if template_rgb.size == 0:
            return None
        if (
            int(template_rgb.shape[0]) < self.bbox_tracking_min_template_px
            or int(template_rgb.shape[1]) < self.bbox_tracking_min_template_px
        ):
            return None
        template_gray = cv2.cvtColor(template_rgb, cv2.COLOR_RGB2GRAY)
        return {
            "template_rgb": template_rgb,
            "template_gray": template_gray,
            "template_bbox_xyxy": [tpl_x1, tpl_y1, tpl_x2, tpl_y2],
            "template_padding_px": pad,
        }

    def _update_bbox_tracking_state(
        self,
        image_rgb: np.ndarray,
        selected_candidate: Optional[dict[str, Any]],
        semantic_task: Optional[SemanticTask],
        now_sec: float,
    ) -> None:
        if not self.bbox_tracking_enabled or not isinstance(selected_candidate, dict):
            return
        bbox_xyxy = self._normalize_bbox_xyxy(
            selected_candidate.get("bbox_xyxy"),
            int(selected_candidate.get("image_width", 0) or image_rgb.shape[1]),
            int(selected_candidate.get("image_height", 0) or image_rgb.shape[0]),
        )
        if bbox_xyxy is None:
            return
        template = self._extract_tracking_template(image_rgb, bbox_xyxy)
        if template is None:
            return
        state = {
            "bbox_xyxy": bbox_xyxy,
            "track_id": int(selected_candidate.get("track_id", -1) or -1),
            "semantic_signature": self._semantic_signature(semantic_task),
            "updated_at": float(now_sec),
            "label": self._candidate_preferred_label(selected_candidate),
            "display_label": self._candidate_preferred_display_label(selected_candidate),
            "label_source": str(selected_candidate.get("label_source", "yolo") or "yolo"),
            "raw_label": self._normalize_short_label(
                selected_candidate.get("raw_label") or selected_candidate.get("label")
            ),
            "label_zh": self._normalize_short_label(selected_candidate.get("label_zh")),
            "confidence": float(
                selected_candidate.get(
                    "selection_confidence_smoothed",
                    selected_candidate.get("selection_confidence", selected_candidate.get("confidence", 0.0)),
                )
                or 0.0
            ),
            "track_seen_count": int(selected_candidate.get("track_seen_count", 0) or 0),
            "track_consecutive_hits": int(
                selected_candidate.get("track_consecutive_hits", 0) or 0
            ),
            "semantic_bonus": float(selected_candidate.get("semantic_bonus", 0.0) or 0.0),
            "score": float(selected_candidate.get("score", 0.0) or 0.0),
            "image_width": int(image_rgb.shape[1]),
            "image_height": int(image_rgb.shape[0]),
            **template,
        }
        with self._bbox_tracking_lock:
            self._bbox_tracking_state = state

    def _maybe_publish_mask_for_now(self, now_sec: float) -> bool:
        min_interval_sec = 1.0 / max(0.1, float(self.mask_publish_rate_hz))
        return (now_sec - float(self._last_mask_publish_at or 0.0)) >= min_interval_sec

    def _match_bbox_tracking_state(
        self,
        image_rgb: np.ndarray,
        state: dict[str, Any],
    ) -> Optional[dict[str, Any]]:
        bbox_xyxy = self._normalize_bbox_xyxy(
            state.get("bbox_xyxy"),
            int(image_rgb.shape[1]),
            int(image_rgb.shape[0]),
        )
        template_gray = state.get("template_gray")
        template_rgb = state.get("template_rgb")
        template_bbox_xyxy = state.get("template_bbox_xyxy")
        if (
            bbox_xyxy is None
            or not isinstance(template_gray, np.ndarray)
            or not isinstance(template_rgb, np.ndarray)
            or not isinstance(template_bbox_xyxy, list)
            or len(template_bbox_xyxy) != 4
        ):
            return None
        x1, y1, x2, y2 = bbox_xyxy
        pad = int(state.get("template_padding_px", self.bbox_tracking_template_padding_px) or 0)
        search_margin = int(self.bbox_tracking_search_margin_px)
        search_x1 = max(0, x1 - search_margin - pad)
        search_y1 = max(0, y1 - search_margin - pad)
        search_x2 = min(int(image_rgb.shape[1]), x2 + search_margin + pad)
        search_y2 = min(int(image_rgb.shape[0]), y2 + search_margin + pad)
        search_rgb = np.asarray(image_rgb[search_y1:search_y2, search_x1:search_x2], dtype=np.uint8)
        if search_rgb.size == 0:
            return None
        if (
            int(search_rgb.shape[0]) < int(template_gray.shape[0])
            or int(search_rgb.shape[1]) < int(template_gray.shape[1])
        ):
            return None
        search_gray = cv2.cvtColor(search_rgb, cv2.COLOR_RGB2GRAY)
        result = cv2.matchTemplate(search_gray, template_gray, cv2.TM_CCOEFF_NORMED)
        _, max_score, _, max_loc = cv2.minMaxLoc(result)
        if float(max_score) < float(self.bbox_tracking_match_threshold):
            return None
        tpl_h, tpl_w = template_gray.shape[:2]
        matched_tpl_x1 = int(search_x1 + max_loc[0])
        matched_tpl_y1 = int(search_y1 + max_loc[1])
        matched_bbox = [
            matched_tpl_x1 + pad,
            matched_tpl_y1 + pad,
            matched_tpl_x1 + tpl_w - pad,
            matched_tpl_y1 + tpl_h - pad,
        ]
        normalized_bbox = self._normalize_bbox_xyxy(
            matched_bbox,
            int(image_rgb.shape[1]),
            int(image_rgb.shape[0]),
        )
        if normalized_bbox is None:
            return None
        updated_template = None
        if float(max_score) >= float(self.bbox_tracking_template_update_threshold):
            updated_template = self._extract_tracking_template(image_rgb, normalized_bbox)
        return {
            "bbox_xyxy": normalized_bbox,
            "match_score": float(max_score),
            "updated_template": updated_template,
        }

    def _build_tracking_payload(
        self,
        color_msg: Image,
        image_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask],
        state: dict[str, Any],
        tracked: dict[str, Any],
        *,
        display_candidates: Optional[list[dict[str, Any]]] = None,
    ) -> dict[str, Any]:
        bbox_xyxy = list(tracked.get("bbox_xyxy") or [])
        label = str(state.get("label", "") or "")
        selected_candidate = {
            "index": -1,
            "track_id": int(state.get("track_id", -1) or -1),
            "track_new": False,
            "track_seen_count": int(state.get("track_seen_count", 0) or 0),
            "track_consecutive_hits": int(state.get("track_consecutive_hits", 0) or 0),
            "track_age_sec": 0.0,
            "track_label_age_sec": 0.0,
            "track_relabel_reason": "bbox_track",
            "label": label,
            "raw_label": str(state.get("raw_label", "") or ""),
            "canonical_label": label,
            "label_zh": str(state.get("label_zh", "") or ""),
            "display_label": str(state.get("display_label", label) or label),
            "label_source": str(state.get("label_source", "bbox_track") or "bbox_track"),
            "side": self._candidate_horizontal_region(
                {
                    "bbox_xyxy": bbox_xyxy,
                    "image_width": int(color_msg.width),
                }
            ),
            "confidence": round(float(state.get("confidence", 0.0) or 0.0), 4),
            "yolo_confidence": round(float(state.get("confidence", 0.0) or 0.0), 4),
            "semantic_confidence": round(float(state.get("confidence", 0.0) or 0.0), 4),
            "track_stability": 1.0,
            "selection_confidence": round(float(state.get("confidence", 0.0) or 0.0), 4),
            "selection_confidence_smoothed": round(float(state.get("confidence", 0.0) or 0.0), 4),
            "confidence_floor": round(float(self.confidence_threshold), 4),
            "semantic_bonus": round(float(state.get("semantic_bonus", 0.0) or 0.0), 4),
            "score": round(float(state.get("score", state.get("confidence", 0.0)) or 0.0), 4),
            "instance_match": False,
            "bbox_xyxy": bbox_xyxy,
            "mask_pixels": 0,
            "status": "tracked",
            "match_score": round(float(tracked.get("match_score", 0.0) or 0.0), 4),
        }
        payload = self._build_detection_payload(
            backend=f"{self.backend}+bbox_track",
            task=semantic_task.task if semantic_task is not None else "pick",
            target_label=label,
            confidence=float(state.get("confidence", 0.0) or 0.0),
            need_human_confirm=False,
            reason="",
            image_width=int(color_msg.width),
            image_height=int(color_msg.height),
            frame_id=color_msg.header.frame_id,
            stamp=color_msg.header.stamp,
            bbox_xyxy=bbox_xyxy,
            point_px=self._bbox_center_point(bbox_xyxy),
            mask_u8=None,
        )
        payload["semantic_target"] = self._semantic_payload(semantic_task)
        payload["selected_candidate"] = selected_candidate
        payload["debug_candidates"] = [selected_candidate]
        payload["display_candidates"] = (
            copy.deepcopy(display_candidates)
            if isinstance(display_candidates, list) and display_candidates
            else [selected_candidate]
        )
        payload["candidate_summary"] = (
            "detector top-k: "
            f"target_label={str(getattr(semantic_task, 'target_label', '') or '<none>')} "
            f"target_hint={str(getattr(semantic_task, 'target_hint', '') or '<none>')} "
            f"selected={label or '<none>'} "
            f"candidates=[#1:{selected_candidate['display_label']}"
            f"(c={float(state.get('confidence', 0.0) or 0.0):.3f},"
            f"b={float(state.get('semantic_bonus', 0.0) or 0.0):+.3f},"
            f"s={float(state.get('score', state.get('confidence', 0.0)) or 0.0):.3f},"
            f"status=tracked,track={int(state.get('track_id', -1) or -1)})]"
        )
        payload["inference_scope"] = "bbox_track"
        payload["roi_bbox_xyxy"] = None
        if self.publish_debug_overlay:
            payload["debug_overlay_rgb"] = self._render_debug_overlay(
                image_rgb,
                [selected_candidate],
                selected_candidate,
                semantic_task,
            )
        return payload

    def _maybe_publish_tracked_detection(self) -> None:
        if (
            not self.enabled
            or self.backend not in ("ultralytics_local", "yoloe_local")
            or not self.bbox_tracking_enabled
        ):
            return
        now_sec = time.time()
        if (now_sec - float(self._last_bbox_tracking_publish_at or 0.0)) < (
            1.0 / max(0.5, float(self.bbox_tracking_max_rate_hz))
        ):
            return
        with self._color_lock:
            color_msg = self._latest_color_msg
        if color_msg is None:
            return
        with self._semantic_lock:
            semantic_task = self._effective_semantic_task(self._semantic_task)
        with self._bbox_tracking_lock:
            state = dict(self._bbox_tracking_state) if isinstance(self._bbox_tracking_state, dict) else None
        frame_key = self._color_msg_frame_key(color_msg)
        if frame_key == self._last_bbox_tracking_frame_key:
            return
        image_rgb = decode_color_image(color_msg)
        if image_rgb is None:
            return
        image_rgb = np.array(image_rgb, dtype=np.uint8, copy=True, order="C")
        selected_candidate: Optional[dict[str, Any]] = None
        pending_payload: Optional[dict[str, Any]] = None
        if isinstance(state, dict) and (now_sec - float(state.get("updated_at", 0.0) or 0.0)) <= float(
            self.bbox_tracking_max_stale_sec
        ):
            semantic_signature = self._semantic_signature(semantic_task)
            stored_signature = str(state.get("semantic_signature", "") or "")
            if not (semantic_signature and stored_signature and semantic_signature != stored_signature):
                if self._semantic_task_requested(semantic_task) and not self._track_matches_semantic_target(
                    state,
                    semantic_task,
                ):
                    self.get_logger().info(
                        "bbox tracking state mismatched semantic target; clearing stale tracked candidate"
                    )
                    self._clear_target_conditioned_runtime_state()
                    return
                tracked = self._match_bbox_tracking_state(image_rgb, state)
                if tracked is not None:
                    pending_payload = self._build_tracking_payload(
                        color_msg, image_rgb, semantic_task, state, tracked
                    )
                    selected_candidate = copy.deepcopy(pending_payload.get("selected_candidate"))
                    updated_template = tracked.get("updated_template")
                    with self._bbox_tracking_lock:
                        active_state = self._bbox_tracking_state
                        if isinstance(active_state, dict) and int(
                            active_state.get("track_id", -1) or -1
                        ) == int(state.get("track_id", -2) or -2):
                            active_state["bbox_xyxy"] = list(tracked["bbox_xyxy"])
                            active_state["updated_at"] = float(now_sec)
                            if isinstance(updated_template, dict):
                                active_state.update(updated_template)
        display_candidates = self._display_candidates_from_tracks(image_rgb)
        if isinstance(pending_payload, dict):
            if display_candidates:
                pending_payload["display_candidates"] = copy.deepcopy(display_candidates)
            elif isinstance(selected_candidate, dict):
                pending_payload["display_candidates"] = [copy.deepcopy(selected_candidate)]
            self._publish_detection(pending_payload)
        if display_candidates or isinstance(selected_candidate, dict) or isinstance(
            self._last_selected_candidate_debug, dict
        ):
            self._publish_display_tracking_debug(
                color_msg,
                semantic_task,
                display_candidates,
                selected_candidate=self._resolve_display_selected_candidate(
                    display_candidates,
                    preferred_candidate=selected_candidate,
                ),
            )
        self._last_bbox_tracking_publish_at = now_sec
        self._last_bbox_tracking_frame_key = frame_key

    def _publish_debug_state(self, debug_payload: dict[str, Any]) -> None:
        selected_candidate = debug_payload.get("selected_candidate")
        if isinstance(selected_candidate, dict):
            self._last_selected_candidate_debug = copy.deepcopy(selected_candidate)
        elif selected_candidate is None:
            self._last_selected_candidate_debug = None
        debug_msg = String()
        debug_msg.data = compact_json(debug_payload)
        self.debug_pub.publish(debug_msg)

    def _publish_display_tracking_debug(
        self,
        color_msg: Image,
        semantic_task: Optional[SemanticTask],
        display_candidates: list[dict[str, Any]],
        *,
        selected_candidate: Optional[dict[str, Any]],
    ) -> None:
        top_candidates = list(display_candidates[: self.debug_top_k_candidates])
        selected_label = (
            self._candidate_preferred_label(selected_candidate)
            if isinstance(selected_candidate, dict)
            else ""
        )
        selected_confidence = (
            float(selected_candidate.get("confidence", 0.0) or 0.0)
            if isinstance(selected_candidate, dict)
            else 0.0
        )
        debug_payload = {
            "backend": f"{self.backend}+display_track",
            "accepted": False,
            "label": selected_label,
            "display_label": self._candidate_preferred_display_label(selected_candidate)
            if isinstance(selected_candidate, dict)
            else "",
            "confidence": round(float(selected_confidence), 4),
            "has_bbox": bool(
                isinstance(selected_candidate, dict)
                and isinstance(selected_candidate.get("bbox_xyxy"), list)
                and len(selected_candidate.get("bbox_xyxy", [])) == 4
            ),
            "has_mask": False,
            "reason": "",
            "semantic_target": self._semantic_payload(semantic_task),
            "selected_candidate": selected_candidate,
            "debug_candidates": display_candidates,
            "top_candidates": top_candidates,
            "candidate_summary": self._format_candidate_summary(
                semantic_task, top_candidates, selected_candidate
            ),
            "inference_scope": "display_track",
            "roi_bbox_xyxy": None,
            "image_width": int(color_msg.width),
            "image_height": int(color_msg.height),
            "scene_prompt": self._scene_prompt_cache_snapshot(),
        }
        self._publish_debug_state(debug_payload)

    def _run_ultralytics_inference_once(
        self,
        color_msg: Image,
        image_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask],
        *,
        roi_spec: Optional[dict[str, Any]],
    ) -> dict[str, Any]:
        model = self._ensure_ultralytics_model()
        backend_display_name = "YOLOE-seg" if self.backend == "yoloe_local" else "YOLO11-seg"
        image_height = int(color_msg.height)
        image_width = int(color_msg.width)
        # Preserve the full object silhouette for segmentation; self-robot suppression
        # for tiny bottom-edge false positives still happens at candidate filtering time.
        detector_input_rgb = np.asarray(image_rgb, dtype=np.uint8)
        roi_bbox_xyxy: Optional[list[int]] = None
        inference_rgb = detector_input_rgb
        inference_scope = "full_frame"
        if roi_spec is not None:
            roi_bbox_xyxy = self._normalize_bbox_xyxy(
                roi_spec.get("bbox_xyxy"),
                image_width,
                image_height,
            )
            if roi_bbox_xyxy is not None:
                roi_x1, roi_y1, roi_x2, roi_y2 = roi_bbox_xyxy
                inference_rgb = np.asarray(
                    detector_input_rgb[roi_y1:roi_y2, roi_x1:roi_x2],
                    dtype=np.uint8,
                ).copy()
                inference_scope = "roi"
        prediction = self._predict_ultralytics(model, inference_rgb, semantic_task)
        now_sec = time.time()
        if inference_scope == "roi":
            self._last_roi_inference_at = now_sec
        else:
            self._last_full_frame_inference_at = now_sec
        if not prediction:
            with self._track_lock:
                self._reap_stale_tracks(now_sec)
            payload = self._build_empty_result(
                task=semantic_task.task if semantic_task is not None else "pick",
                target_label=semantic_task.target_label if semantic_task is not None else "",
                reason=f"no detections returned by {backend_display_name}",
                image_width=image_width,
                image_height=image_height,
                frame_id=color_msg.header.frame_id,
                stamp=color_msg.header.stamp,
            )
            payload["inference_scope"] = inference_scope
            payload["roi_bbox_xyxy"] = roi_bbox_xyxy
            payload["semantic_target"] = self._semantic_payload(semantic_task)
            self._attach_display_debug_payload(
                payload,
                image_rgb=image_rgb,
                semantic_task=semantic_task,
            )
            return payload

        result = prediction[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            with self._track_lock:
                self._reap_stale_tracks(now_sec)
            payload = self._build_empty_result(
                task=semantic_task.task if semantic_task is not None else "pick",
                target_label=semantic_task.target_label if semantic_task is not None else "",
                reason=f"{backend_display_name} found no candidate instances",
                image_width=image_width,
                image_height=image_height,
                frame_id=color_msg.header.frame_id,
                stamp=color_msg.header.stamp,
            )
            payload["inference_scope"] = inference_scope
            payload["roi_bbox_xyxy"] = roi_bbox_xyxy
            payload["semantic_target"] = self._semantic_payload(semantic_task)
            self._attach_display_debug_payload(
                payload,
                image_rgb=image_rgb,
                semantic_task=semantic_task,
            )
            return payload

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
        candidate_count = self._candidate_count_for_result(boxes, masks_data)
        if candidate_count <= 0:
            with self._track_lock:
                self._reap_stale_tracks(now_sec)
            payload = self._build_empty_result(
                task=semantic_task.task if semantic_task is not None else "pick",
                target_label=semantic_task.target_label if semantic_task is not None else "",
                reason=f"{backend_display_name} returned inconsistent candidate tensors",
                image_width=image_width,
                image_height=image_height,
                frame_id=color_msg.header.frame_id,
                stamp=color_msg.header.stamp,
            )
            payload["inference_scope"] = inference_scope
            payload["roi_bbox_xyxy"] = roi_bbox_xyxy
            payload["semantic_target"] = self._semantic_payload(semantic_task)
            self._attach_display_debug_payload(
                payload,
                image_rgb=image_rgb,
                semantic_task=semantic_task,
            )
            return payload

        inference_height = int(inference_rgb.shape[0])
        inference_width = int(inference_rgb.shape[1])
        for idx in range(candidate_count):
            cls_tensor = self._safe_result_item(getattr(boxes, "cls", None), idx)
            conf_tensor = self._safe_result_item(getattr(boxes, "conf", None), idx)
            if cls_tensor is None or conf_tensor is None:
                continue
            cls_id = int(cls_tensor.item() if hasattr(cls_tensor, "item") else cls_tensor)
            confidence = float(
                conf_tensor.item() if hasattr(conf_tensor, "item") else conf_tensor
            )
            label = self._lookup_result_label(names, cls_id)
            bbox_xyxy = self._extract_bbox_xyxy(
                boxes,
                idx,
                inference_width,
                inference_height,
            )
            if roi_bbox_xyxy is not None:
                bbox_xyxy = self._translate_bbox_from_roi(
                    bbox_xyxy,
                    roi_bbox_xyxy,
                    image_width,
                    image_height,
                )
            if bbox_xyxy is None:
                continue
            mask_u8 = self._extract_mask_u8(masks_data, idx, inference_rgb.shape[:2])
            if roi_bbox_xyxy is not None:
                mask_u8 = self._project_mask_from_roi(
                    mask_u8,
                    roi_bbox_xyxy,
                    image_rgb.shape[:2],
                )
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
                "label_source": "open_vocab" if self.backend == "yoloe_local" else "yolo",
                "confidence": float(confidence),
                "semantic_bonus": 0.0,
                "score": float(confidence),
                "confidence_floor": float(self.confidence_threshold),
                "bbox_xyxy": bbox_xyxy,
                "mask_u8": mask_u8,
                "mask_pixels": int(mask_pixels),
                "image_width": image_width,
                "image_height": image_height,
                "status": "selectable",
            }
            if self._is_robot_self_candidate(candidate):
                continue
            raw_candidates.append(candidate)

        self._associate_candidates_to_tracks(raw_candidates, image_rgb=image_rgb)
        if self.backend == "ultralytics_local":
            self._schedule_candidate_relabels(image_rgb, raw_candidates, semantic_task)

        instance_lock_active = self._semantic_target_instance_active(semantic_task)
        for candidate in raw_candidates:
            self._apply_selection_confidence(candidate)
            selection_confidence = float(
                candidate.get(
                    "selection_confidence_smoothed",
                    candidate.get("selection_confidence", candidate.get("confidence", 0.0)),
                )
                or 0.0
            )
            semantic_bonus = self._candidate_semantic_bonus(candidate, semantic_task)
            instance_match = self._candidate_matches_instance_lock(candidate, semantic_task)
            instance_bonus = self._candidate_instance_bonus(candidate, semantic_task)
            score = float(selection_confidence) + float(semantic_bonus) + float(instance_bonus)
            confidence_floor = self._candidate_confidence_floor(semantic_bonus, semantic_task)
            status = "selectable"
            if instance_lock_active and not instance_match:
                status = "filtered_instance_mismatch"
            elif float(selection_confidence) < confidence_floor:
                status = "filtered_low_confidence"
            elif semantic_bonus <= -1.0:
                status = "filtered_excluded_label"
            candidate["instance_match"] = bool(instance_lock_active and instance_match)
            candidate["semantic_bonus"] = float(semantic_bonus)
            candidate["score"] = float(score)
            candidate["confidence_floor"] = float(confidence_floor)
            candidate["status"] = status
            if status == "selectable":
                candidates.append(candidate)
        self._update_track_selection_debug_from_candidates(raw_candidates)
        display_candidates = self._display_candidates_from_raw_candidates(image_rgb, raw_candidates)

        if not candidates and instance_lock_active:
            candidates = self._recover_instance_locked_candidates(raw_candidates, semantic_task)

        if not candidates:
            empty_reason = f"{backend_display_name} found detections, but none passed selection filters"
            if instance_lock_active:
                empty_reason = (
                    f"{backend_display_name} found detections, but none matched the requested target instance"
                )
            payload = self._build_empty_result(
                task=semantic_task.task if semantic_task is not None else "pick",
                target_label=semantic_task.target_label if semantic_task is not None else "",
                reason=empty_reason,
                image_width=image_width,
                image_height=image_height,
                frame_id=color_msg.header.frame_id,
                stamp=color_msg.header.stamp,
            )
            payload["semantic_target"] = self._semantic_payload(semantic_task)
            payload["inference_scope"] = inference_scope
            payload["roi_bbox_xyxy"] = roi_bbox_xyxy
            self._attach_display_debug_payload(
                payload,
                image_rgb=image_rgb,
                semantic_task=semantic_task,
                detected_candidates=raw_candidates,
            )
            return payload

        if self._semantic_task_requires_match(semantic_task):
            semantically_relevant = [
                item for item in candidates if float(item["semantic_bonus"]) > 0.0
            ]
            if semantically_relevant:
                candidates = semantically_relevant
            else:
                payload = self._build_empty_result(
                    task=semantic_task.task if semantic_task is not None else "pick",
                    target_label=semantic_task.target_label if semantic_task is not None else "",
                    reason=f"{backend_display_name} found detections, but none matched the semantic target",
                    image_width=image_width,
                    image_height=image_height,
                    frame_id=color_msg.header.frame_id,
                    stamp=color_msg.header.stamp,
                )
                payload["semantic_target"] = self._semantic_payload(semantic_task)
                payload["inference_scope"] = inference_scope
                payload["roi_bbox_xyxy"] = roi_bbox_xyxy
                self._attach_display_debug_payload(
                    payload,
                    image_rgb=image_rgb,
                    semantic_task=semantic_task,
                    detected_candidates=raw_candidates,
                )
                return payload

        selected_candidate = max(candidates, key=lambda item: float(item["score"]))
        if inference_scope == "full_frame":
            self._update_roi_state_from_candidate(selected_candidate, semantic_task, now_sec)
            self._update_bbox_tracking_state(image_rgb, selected_candidate, semantic_task, now_sec)
        mask_u8_for_payload = (
            selected_candidate["mask_u8"]
            if self._maybe_publish_mask_for_now(now_sec)
            else None
        )
        payload = self._build_detection_payload(
            backend=(
                f"ultralytics_{self._ultralytics_runtime}:"
                f"{Path(self._resolved_ultralytics_model_path or self.ultralytics_model_path).name}"
                f"+{inference_scope}"
            ),
            task=semantic_task.task if semantic_task is not None else "pick",
            target_label=self._candidate_preferred_label(selected_candidate),
            confidence=float(
                selected_candidate.get(
                    "selection_confidence_smoothed",
                    selected_candidate.get("selection_confidence", selected_candidate["confidence"]),
                )
            ),
            need_human_confirm=False,
            reason="",
            image_width=image_width,
            image_height=image_height,
            frame_id=color_msg.header.frame_id,
            stamp=color_msg.header.stamp,
            bbox_xyxy=selected_candidate["bbox_xyxy"],
            point_px=self._bbox_center_point(selected_candidate["bbox_xyxy"]),
            mask_u8=mask_u8_for_payload,
        )
        payload["semantic_target"] = self._semantic_payload(semantic_task)
        payload["inference_scope"] = inference_scope
        payload["roi_bbox_xyxy"] = roi_bbox_xyxy
        self._attach_display_debug_payload(
            payload,
            image_rgb=image_rgb,
            semantic_task=semantic_task,
            detected_candidates=raw_candidates,
            preferred_candidate=selected_candidate,
        )
        return payload

    def _run_ultralytics_inference(
        self,
        color_msg: Image,
        image_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask],
    ) -> dict[str, Any]:
        now_sec = time.time()
        roi_spec = self._select_ultralytics_roi_spec(image_rgb, semantic_task, now_sec)
        if roi_spec is not None:
            roi_payload = self._run_ultralytics_inference_once(
                color_msg,
                image_rgb,
                semantic_task,
                roi_spec=roi_spec,
            )
            if bool(roi_payload.get("accepted", False)) or bool(
                roi_payload.get("candidate_visible", False)
            ):
                return roi_payload
        return self._run_ultralytics_inference_once(
            color_msg,
            image_rgb,
            semantic_task,
            roi_spec=None,
        )

    def _mask_robot_self_inference_region(self, image_rgb: np.ndarray) -> np.ndarray:
        if (
            not self.suppress_robot_self_candidates
            or self.robot_self_inference_mask_bottom_band_px <= 0
            or image_rgb.size == 0
        ):
            return image_rgb
        masked = np.asarray(image_rgb, dtype=np.uint8).copy()
        height = int(masked.shape[0])
        band_px = min(height, int(self.robot_self_inference_mask_bottom_band_px))
        if band_px <= 0:
            return image_rgb
        masked[height - band_px :, :, :] = 0
        return masked

    def _is_robot_self_candidate(self, candidate: dict[str, Any]) -> bool:
        if not self.suppress_robot_self_candidates:
            return False
        bbox_xyxy = candidate.get("bbox_xyxy")
        if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
            return False
        image_height = int(candidate.get("image_height", 0) or 0)
        if image_height <= 0:
            return False
        x1, y1, x2, y2 = [int(value) for value in bbox_xyxy]
        bbox_width = max(0, x2 - x1)
        bbox_height = max(0, y2 - y1)
        touches_bottom = (image_height - y2) <= self.robot_self_bottom_touch_tolerance_px
        near_bottom = y1 >= int(round(float(image_height) * self.robot_self_candidate_min_y_ratio))
        small_bbox = (
            bbox_height <= self.robot_self_candidate_max_height_px
            and bbox_width <= self.robot_self_candidate_max_width_px
        )
        small_mask = int(candidate.get("mask_pixels", 0) or 0) <= self.robot_self_candidate_max_mask_pixels
        return touches_bottom and near_bottom and small_bbox and small_mask

    def _normalize_short_label(self, value: Any, *, max_chars: int = 48) -> str:
        text = str(value or "").replace("\n", " ").replace("\r", " ").strip()
        if not text:
            return ""
        while "  " in text:
            text = text.replace("  ", " ")
        text = text.strip(" \t,.;:!?/|")
        return text[:max_chars].strip()

    def _lookup_result_label(self, names: Any, cls_id: int) -> str:
        try:
            normalized_cls_id = int(cls_id)
        except (TypeError, ValueError):
            normalized_cls_id = -1
        if isinstance(names, dict):
            return self._normalize_short_label(names.get(normalized_cls_id, normalized_cls_id))
        if isinstance(names, (list, tuple)):
            if 0 <= normalized_cls_id < len(names):
                return self._normalize_short_label(names[normalized_cls_id])
            return f"class_{normalized_cls_id}" if normalized_cls_id >= 0 else "unknown"
        return self._normalize_short_label(names) or (
            f"class_{normalized_cls_id}" if normalized_cls_id >= 0 else "unknown"
        )

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

    def _coerce_short_text_list(self, value: Any, *, max_chars: int = 80) -> list[str]:
        if not isinstance(value, (list, tuple)):
            return []
        items: list[str] = []
        seen: set[str] = set()
        for item in value:
            text = self._normalize_short_label(item, max_chars=max_chars)
            key = text.lower()
            if not text or key in seen:
                continue
            seen.add(key)
            items.append(text)
        return items

    def _translate_open_vocab_text(self, value: Any) -> str:
        text = self._normalize_short_label(value, max_chars=96)
        if not text:
            return ""
        translated = text
        for source, target in OPEN_VOCAB_TEXT_REPLACEMENTS:
            translated = translated.replace(source, target)
        while "  " in translated:
            translated = translated.replace("  ", " ")
        translated = translated.strip(" \t,.;:!?/|")
        if not translated:
            return ""
        if any(ord(char) > 127 for char in translated):
            return ""
        return translated

    def _expand_open_vocab_alias_prompts(self, value: Any) -> list[str]:
        base = self._normalize_short_label(value, max_chars=96)
        if not base:
            return []
        prompts: list[str] = []
        seen: set[str] = set()
        candidates = [base]
        translated = self._translate_open_vocab_text(base)
        if translated and translated.lower() != base.lower():
            candidates.append(translated)
        for candidate in candidates:
            alias_key = candidate.lower()
            aliases = OPEN_VOCAB_ALIAS_PROMPTS.get(alias_key, [])
            expanded = [candidate, *aliases]
            for item in expanded:
                prompt = self._normalize_short_label(item, max_chars=80)
                key = prompt.lower()
                if not prompt or key in seen:
                    continue
                seen.add(key)
                prompts.append(prompt)
        return prompts

    def _semantic_detection_spec(self, semantic_task: Optional[SemanticTask]) -> dict[str, Any]:
        scene_spec = self._scene_prompt_spec()
        scene_prompt_classes = list(scene_spec.get("prompt_classes") or [])
        scene_negative_labels = list(scene_spec.get("negative_labels") or [])
        scene_attributes = list(scene_spec.get("attributes") or [])
        prompt_classes: list[str] = []
        negative_labels: list[str] = []
        attributes: list[str] = []
        primary_label = ""
        raw_payload = str(getattr(semantic_task, "raw_json", "") or "").strip()
        parsed_payload: dict[str, Any] = {}
        if raw_payload:
            try:
                parsed_payload = extract_first_json_object(raw_payload)
            except Exception:
                parsed_payload = {}
        detection_spec = parsed_payload.get("detection_spec")
        if isinstance(detection_spec, dict):
            primary_label = self._normalize_short_label(
                detection_spec.get("primary_label")
                or detection_spec.get("canonical_label")
                or detection_spec.get("target_label")
                or "",
                max_chars=80,
            )
            prompt_classes.extend(
                self._coerce_short_text_list(
                    detection_spec.get("prompt_classes")
                    or detection_spec.get("classes")
                    or detection_spec.get("det_prompt")
                    or [],
                    max_chars=80,
                )
            )
            prompt_classes.extend(
                self._coerce_short_text_list(detection_spec.get("synonyms") or [], max_chars=80)
            )
            negative_labels.extend(
                self._coerce_short_text_list(
                    detection_spec.get("negative_labels") or [],
                    max_chars=80,
                )
            )
            attributes.extend(
                self._coerce_short_text_list(detection_spec.get("attributes") or [], max_chars=48)
            )
        if semantic_task is not None:
            primary_label = primary_label or self._normalize_short_label(
                semantic_task.target_label or semantic_task.target_hint or "",
                max_chars=80,
            )
            prompt_classes.extend(self._expand_open_vocab_alias_prompts(semantic_task.target_label))
            prompt_classes.extend(self._expand_open_vocab_alias_prompts(semantic_task.target_hint))
            negative_labels.extend(self._coerce_short_text_list(semantic_task.excluded_labels, max_chars=80))
        if scene_prompt_classes:
            prompt_classes.extend(scene_prompt_classes)
            negative_labels.extend(scene_negative_labels)
            attributes.extend(scene_attributes)
        if not prompt_classes:
            prompt_classes = list(self.open_vocab_default_classes)
        deduped_prompts: list[str] = []
        seen_prompt_keys: set[str] = set()
        for item in prompt_classes:
            prompt = self._normalize_short_label(item, max_chars=80)
            key = prompt.lower()
            if not prompt or key in seen_prompt_keys:
                continue
            seen_prompt_keys.add(key)
            deduped_prompts.append(prompt)
            if len(deduped_prompts) >= self.open_vocab_max_classes:
                break
        deduped_negatives = self._coerce_short_text_list(negative_labels, max_chars=80)
        return {
            "primary_label": primary_label or (deduped_prompts[0] if deduped_prompts else ""),
            "prompt_classes": deduped_prompts or list(self.open_vocab_default_classes),
            "negative_labels": deduped_negatives,
            "attributes": self._coerce_short_text_list(attributes, max_chars=48),
        }

    def _open_vocab_prompt_classes(self, semantic_task: Optional[SemanticTask]) -> list[str]:
        spec = self._semantic_detection_spec(semantic_task)
        classes = [
            self._normalize_short_label(item, max_chars=80)
            for item in spec.get("prompt_classes", []) or []
            if self._normalize_short_label(item, max_chars=80)
        ]
        return classes or list(self.open_vocab_default_classes or ["object"])

    def _semantic_signature(self, semantic_task: Optional[SemanticTask]) -> str:
        if semantic_task is None:
            return ""
        task = str(semantic_task.task or "").strip().lower()
        label = str(semantic_task.target_label or "").strip().lower()
        hint = str(semantic_task.target_hint or "").strip().lower()
        target_instance = self._semantic_target_instance(semantic_task) or {}
        instance_track = str(int(target_instance.get("track_id", 0) or 0))
        instance_bbox = ",".join(str(int(item)) for item in list(target_instance.get("bbox_xyxy", []) or []))
        instance_point = ",".join(str(int(item)) for item in list(target_instance.get("point_px", []) or []))
        excluded = ",".join(
            sorted(str(item or "").strip().lower() for item in semantic_task.excluded_labels if str(item or "").strip())
        )
        return "|".join([task, label, hint, excluded, instance_track, instance_bbox, instance_point]).strip("|")

    def _semantic_task_requested(self, semantic_task: Optional[SemanticTask]) -> bool:
        if semantic_task is None:
            return False
        return bool(
            str(semantic_task.target_label or "").strip()
            or str(semantic_task.target_hint or "").strip()
        )

    def _semantic_target_instance(
        self,
        semantic_task: Optional[SemanticTask],
    ) -> Optional[dict[str, Any]]:
        if semantic_task is None:
            return None
        try:
            track_id = int(getattr(semantic_task, "target_instance_track_id", 0) or 0)
        except (TypeError, ValueError):
            track_id = 0
        bbox_xyxy = [
            int(value)
            for value in list(getattr(semantic_task, "target_instance_bbox_xyxy", []) or [])
        ]
        if len(bbox_xyxy) != 4:
            bbox_xyxy = []
        point_px = [
            int(value)
            for value in list(getattr(semantic_task, "target_instance_point_px", []) or [])
        ]
        if len(point_px) != 2:
            point_px = self._bbox_center_point(bbox_xyxy) or []
        source = str(getattr(semantic_task, "target_instance_source", "") or "").strip()
        if track_id <= 0 and not bbox_xyxy and not point_px:
            return None
        return {
            "track_id": track_id,
            "bbox_xyxy": bbox_xyxy,
            "point_px": point_px,
            "source": source,
        }

    def _semantic_target_instance_active(
        self,
        semantic_task: Optional[SemanticTask],
    ) -> bool:
        return self._semantic_target_instance(semantic_task) is not None

    def _candidate_matches_instance_lock(
        self,
        candidate: Optional[dict[str, Any]],
        semantic_task: Optional[SemanticTask],
    ) -> bool:
        target_instance = self._semantic_target_instance(semantic_task)
        if target_instance is None:
            return True
        if not isinstance(candidate, dict):
            return False
        try:
            candidate_track_id = int(candidate.get("track_id", -1) or -1)
        except (TypeError, ValueError):
            candidate_track_id = -1
        target_track_id = int(target_instance.get("track_id", 0) or 0)
        if target_track_id > 0 and candidate_track_id == target_track_id:
            return True

        candidate_bbox = candidate.get("bbox_xyxy")
        target_bbox = target_instance.get("bbox_xyxy")
        if (
            isinstance(candidate_bbox, list)
            and len(candidate_bbox) == 4
            and isinstance(target_bbox, list)
            and len(target_bbox) == 4
        ):
            if self._bbox_iou(candidate_bbox, target_bbox) >= max(0.18, self.track_iou_threshold):
                return True
            if self._center_distance_px(
                self._bbox_center(candidate_bbox),
                self._bbox_center(target_bbox),
            ) <= min(64.0, self.track_center_distance_px):
                return True

        target_point = target_instance.get("point_px")
        if isinstance(target_point, list) and len(target_point) == 2:
            candidate_center = self._bbox_center(candidate_bbox)
            if self._center_distance_px(
                candidate_center,
                (float(target_point[0]), float(target_point[1])),
            ) <= min(56.0, self.track_center_distance_px):
                return True
        return False

    def _candidate_instance_bonus(
        self,
        candidate: Optional[dict[str, Any]],
        semantic_task: Optional[SemanticTask],
    ) -> float:
        target_instance = self._semantic_target_instance(semantic_task)
        if target_instance is None or not self._candidate_matches_instance_lock(candidate, semantic_task):
            return 0.0
        try:
            candidate_track_id = int((candidate or {}).get("track_id", -1) or -1)
        except (TypeError, ValueError):
            candidate_track_id = -1
        if int(target_instance.get("track_id", 0) or 0) > 0 and candidate_track_id == int(
            target_instance.get("track_id", 0) or 0
        ):
            return 1.25
        return 0.8

    def _recover_instance_locked_candidates(
        self,
        raw_candidates: list[dict[str, Any]],
        semantic_task: Optional[SemanticTask],
    ) -> list[dict[str, Any]]:
        if not self._semantic_target_instance_active(semantic_task):
            return []

        eligible: list[dict[str, Any]] = []
        for candidate in raw_candidates:
            confidence = float(candidate.get("confidence", 0.0) or 0.0)
            confidence_floor = float(
                candidate.get("confidence_floor", self.confidence_threshold) or self.confidence_threshold
            )
            semantic_bonus = float(candidate.get("semantic_bonus", 0.0) or 0.0)
            if confidence < confidence_floor:
                continue
            if semantic_bonus <= -1.0:
                continue
            eligible.append(candidate)

        if not eligible:
            return []

        semantically_relevant = [
            candidate for candidate in eligible if float(candidate.get("semantic_bonus", 0.0) or 0.0) > 0.0
        ]
        fallback_pool = semantically_relevant or eligible
        if len(fallback_pool) != 1:
            return []

        recovered = fallback_pool[0]
        recovered["status"] = "reacquired_instance"
        recovered["instance_match"] = False
        recovered["instance_reacquired"] = True
        return [recovered]

    def _vlm_focus_phase_active(self) -> bool:
        return self._pick_phase in {"waiting_execute", "planning", "executing"}

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

    def _bbox_overlap_ratio(
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
        smaller_area = min(left_area, right_area)
        if smaller_area <= 0.0:
            return 0.0
        return inter_area / smaller_area

    def _candidate_priority_key(self, candidate: dict[str, Any]) -> tuple[float, float, float]:
        return (
            float(
                candidate.get(
                    "selection_confidence_smoothed",
                    candidate.get("selection_confidence", candidate.get("confidence", 0.0)),
                )
                or 0.0
            ),
            float(candidate.get("score", candidate.get("confidence", 0.0)) or 0.0),
            float(candidate.get("confidence", 0.0) or 0.0),
        )

    def _candidate_duplicate_preference_key(
        self,
        candidate: dict[str, Any],
    ) -> tuple[float, int, float]:
        return (
            float(candidate.get("confidence", 0.0) or 0.0),
            int(candidate.get("mask_pixels", 0) or 0),
            float(candidate.get("score", candidate.get("confidence", 0.0)) or 0.0),
        )

    def _deduplicate_candidates_by_bbox(
        self,
        candidates: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        ranked = sorted(candidates, key=self._candidate_priority_key, reverse=True)
        deduped: list[dict[str, Any]] = []
        for candidate in ranked:
            bbox_xyxy = candidate.get("bbox_xyxy")
            if not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
                deduped.append(candidate)
                continue
            candidate_label = self._normalized_label_key(
                self._candidate_preferred_label(candidate)
            )
            candidate_tokens = set(self._semantic_tokens(candidate_label))
            candidate_center = self._bbox_center(bbox_xyxy)
            duplicate_index = -1
            for kept_index, kept in enumerate(deduped):
                kept_bbox = kept.get("bbox_xyxy")
                if not isinstance(kept_bbox, list) or len(kept_bbox) != 4:
                    continue
                if (
                    self._bbox_overlap_ratio(bbox_xyxy, kept_bbox) >= 0.82
                    or self._bbox_iou(bbox_xyxy, kept_bbox) >= 0.55
                ):
                    duplicate_index = kept_index
                    break
                kept_label = self._normalized_label_key(
                    self._candidate_preferred_label(kept)
                )
                if candidate_label and candidate_label == kept_label:
                    overlap_ratio = self._bbox_overlap_ratio(bbox_xyxy, kept_bbox)
                    center_distance = self._center_distance_px(
                        candidate_center,
                        self._bbox_center(kept_bbox),
                    )
                    if overlap_ratio >= 0.45 or (
                        overlap_ratio >= 0.2 and center_distance <= 64.0
                    ):
                        duplicate_index = kept_index
                        break
                kept_tokens = set(self._semantic_tokens(kept_label))
                labels_compatible = bool(
                    candidate_tokens
                    and kept_tokens
                    and (
                        candidate_tokens.issubset(kept_tokens)
                        or kept_tokens.issubset(candidate_tokens)
                    )
                )
                if labels_compatible:
                    overlap_ratio = self._bbox_overlap_ratio(bbox_xyxy, kept_bbox)
                    center_distance = self._center_distance_px(
                        candidate_center,
                        self._bbox_center(kept_bbox),
                    )
                    if overlap_ratio >= 0.45 or (
                        overlap_ratio >= 0.2 and center_distance <= 64.0
                    ):
                        duplicate_index = kept_index
                        break
            if duplicate_index < 0:
                deduped.append(candidate)
                continue

            kept = deduped[duplicate_index]
            candidate_conf = float(candidate.get("confidence", 0.0) or 0.0)
            kept_conf = float(kept.get("confidence", 0.0) or 0.0)
            candidate_mask_pixels = int(candidate.get("mask_pixels", 0) or 0)
            kept_mask_pixels = int(kept.get("mask_pixels", 0) or 0)
            if (
                candidate_mask_pixels > kept_mask_pixels
                and (candidate_conf + 0.05) >= kept_conf
                and self._candidate_duplicate_preference_key(candidate)
                >= self._candidate_duplicate_preference_key(kept)
            ):
                deduped[duplicate_index] = candidate
        return deduped

    def _make_track_state(
        self,
        candidate: dict[str, Any],
        now_sec: float,
        image_rgb: Optional[np.ndarray] = None,
    ) -> dict[str, Any]:
        track_id = int(self._next_track_id)
        self._next_track_id += 1
        raw_label = self._normalize_short_label(candidate.get("raw_label") or candidate.get("label"))
        center_xy = self._bbox_center(candidate.get("bbox_xyxy"))
        track = {
            "track_id": track_id,
            "created_at": now_sec,
            "first_seen": now_sec,
            "last_seen": now_sec,
            "last_tracked_at": 0.0,
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
            "semantic_confidence": float(candidate.get("confidence", 0.0) or 0.0),
            "mask_pixels": int(candidate.get("mask_pixels", 0) or 0),
            "selection_conf_history": [],
            "selection_confidence": float(candidate.get("confidence", 0.0) or 0.0),
            "selection_confidence_smoothed": float(candidate.get("confidence", 0.0) or 0.0),
            "last_selection_status": "",
            "last_semantic_bonus": 0.0,
            "last_score": float(candidate.get("confidence", 0.0) or 0.0),
            "last_instance_match": False,
            "last_confidence_floor": float(self.confidence_threshold),
            "relabel_in_flight": False,
            "last_relabel_requested_at": 0.0,
            "last_relabel_completed_at": 0.0,
            "last_relabel_reason": "",
            "last_semantic_signature": "",
        }
        if isinstance(image_rgb, np.ndarray):
            template = self._extract_tracking_template(image_rgb, track["bbox_xyxy"])
            if isinstance(template, dict):
                track.update(template)
        return track

    def _update_track_from_candidate(
        self,
        track: dict[str, Any],
        candidate: dict[str, Any],
        now_sec: float,
        image_rgb: Optional[np.ndarray] = None,
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
        if isinstance(image_rgb, np.ndarray):
            template = self._extract_tracking_template(image_rgb, track.get("bbox_xyxy"))
            if isinstance(template, dict):
                track.update(template)

    def _estimate_semantic_confidence(
        self,
        canonical_label: str,
        label_zh: str,
        raw_label: str,
    ) -> float:
        normalized_label = self._normalize_short_label(canonical_label or raw_label)
        if not normalized_label:
            return self.vlm_relabel_default_semantic_confidence
        score = self.vlm_relabel_default_semantic_confidence
        tokens = self._semantic_tokens(normalized_label)
        if any(token in COLOR_HINT_TOKENS for token in tokens):
            score += 0.12
        if any(token in APPEARANCE_HINT_TOKENS for token in tokens):
            score += 0.08
        if label_zh:
            score += 0.04
        if self._is_ambiguous_label(normalized_label):
            score -= 0.12
        if self._normalized_label_key(normalized_label) == self._normalized_label_key(raw_label):
            score -= 0.08
        return max(0.05, min(0.95, score))

    def _track_stability_confidence(self, track: Optional[dict[str, Any]]) -> float:
        if not isinstance(track, dict):
            return 0.0
        consecutive_hits = max(0, int(track.get("consecutive_hits", 0) or 0))
        return max(
            0.0,
            min(1.0, float(consecutive_hits) / float(max(1, self.selection_conf_smoothing_frames))),
        )

    def _apply_selection_confidence(self, candidate: dict[str, Any]) -> None:
        yolo_confidence = max(0.0, min(1.0, float(candidate.get("confidence", 0.0) or 0.0)))
        semantic_confidence = yolo_confidence
        track_stability = 0.0
        selection_confidence = (
            self.selection_conf_qwen_weight * semantic_confidence
            + self.selection_conf_yolo_weight * yolo_confidence
        )
        smoothed_selection_confidence = selection_confidence
        track_id = int(candidate.get("track_id", -1) or -1)
        with self._track_lock:
            track = self._tracks.get(track_id)
            if isinstance(track, dict):
                if self._track_has_vlm_label(track):
                    semantic_confidence = max(
                        0.0,
                        min(1.0, float(track.get("semantic_confidence", yolo_confidence) or yolo_confidence)),
                    )
                track_stability = self._track_stability_confidence(track)
                selection_confidence = (
                    self.selection_conf_qwen_weight * semantic_confidence
                    + self.selection_conf_yolo_weight * yolo_confidence
                    + self.selection_conf_track_weight * track_stability
                )
                history = list(track.get("selection_conf_history", []) or [])
                history.append(float(selection_confidence))
                history = history[-self.selection_conf_smoothing_frames :]
                smoothed_selection_confidence = float(sum(history) / float(len(history)))
                track["selection_conf_history"] = history
                track["selection_confidence"] = float(selection_confidence)
                track["selection_confidence_smoothed"] = float(smoothed_selection_confidence)
        candidate["semantic_confidence"] = float(semantic_confidence)
        candidate["track_stability"] = float(track_stability)
        candidate["selection_confidence"] = float(selection_confidence)
        candidate["selection_confidence_smoothed"] = float(smoothed_selection_confidence)

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
        candidate["semantic_confidence"] = float(
            track.get("semantic_confidence", candidate.get("confidence", 0.0)) or 0.0
        )
        candidate["track_stability"] = self._track_stability_confidence(track)
        candidate["selection_confidence"] = float(
            track.get("selection_confidence", candidate.get("confidence", 0.0)) or 0.0
        )
        candidate["selection_confidence_smoothed"] = float(
            track.get(
                "selection_confidence_smoothed",
                candidate.get("selection_confidence", candidate.get("confidence", 0.0)),
            )
            or 0.0
        )

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
        image_rgb: Optional[np.ndarray] = None,
    ) -> None:
        if not raw_candidates:
            return
        raw_candidates[:] = self._deduplicate_candidates_by_bbox(raw_candidates)
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
                    best_track = self._make_track_state(candidate, now_sec, image_rgb=image_rgb)
                    self._tracks[int(best_track["track_id"])] = best_track
                self._update_track_from_candidate(best_track, candidate, now_sec, image_rgb=image_rgb)
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
        if self._semantic_target_instance_active(semantic_task):
            return self._candidate_matches_instance_lock(candidate or track, semantic_task)
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
        cooldown_sec = self.vlm_relabel_track_cooldown_sec
        if self._is_vlm_focus_candidate(track, candidate, semantic_task):
            cooldown_sec = min(cooldown_sec, self.vlm_relabel_focus_track_cooldown_sec)
        if (now_sec - last_requested_at) < cooldown_sec:
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
        track: dict[str, Any],
        reason: str,
        candidate: dict[str, Any],
        semantic_task: Optional[SemanticTask],
    ) -> float:
        base = {
            "semantic_target": 4.0,
            "ambiguous_raw_label": 3.0,
            "refine_vlm_label": 2.5,
            "low_confidence_stable": 2.0,
            "new_track": 1.0,
        }.get(reason, 0.0)
        if self._is_vlm_focus_candidate(track, candidate, semantic_task):
            base += 10.0
        return base + float(candidate.get("confidence", 0.0) or 0.0)

    def _is_vlm_focus_candidate(
        self,
        track: dict[str, Any],
        candidate: dict[str, Any],
        semantic_task: Optional[SemanticTask],
    ) -> bool:
        if not self._semantic_task_requested(semantic_task):
            return False
        if not self._track_matches_semantic_target(track, semantic_task, candidate):
            return False
        if self._vlm_focus_phase_active():
            return True
        return bool(int(track.get("seen_count", 0) or 0) >= self.track_stable_frames_required)

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
            "Schema: {\"candidates\":[{\"index\":number,\"label_en\":string,\"label_zh\":string,\"semantic_confidence\":number}]}. "
            "Use very short noun phrases, not sentences, with 1 to 4 words per label. "
            "The first image is the full scene with numbered boxes. Later images are crops of those numbered boxes. "
            "Use the crop as primary evidence and the full scene only as context. "
            "Ignore the detector raw label if it looks wrong. "
            "If you are not clearly confident in a standard object class, describe the visible object by appearance instead: "
            "blue cylinder, white paper cup, white block, black tool handle, silver can. "
            "Prefer color-plus-shape descriptions over generic class guesses. "
            "If the object looks like a smooth cylinder and you cannot clearly see a cup rim, handle, bottle neck, or can top, label it as cylinder instead of cup, bottle, vase, or can. "
            "Do not answer with vague words like object, item, thing. "
            "Do not use dataset artifacts like remote, mouse, keyboard, cell phone unless the visible features clearly support that label. "
            "Also return semantic_confidence as a number from 0.0 to 1.0 for each candidate."
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
            "max_tokens": 128,
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
                "semantic_confidence": item.get("semantic_confidence"),
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
                    semantic_confidence_raw = item.get("semantic_confidence")
                    try:
                        semantic_confidence = float(semantic_confidence_raw)
                    except (TypeError, ValueError):
                        semantic_confidence = self._estimate_semantic_confidence(
                            canonical_label=canonical_label or raw_label,
                            label_zh=label_zh,
                            raw_label=raw_label,
                        )
                    semantic_confidence = max(0.0, min(1.0, semantic_confidence))
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
                        track["semantic_confidence"] = float(semantic_confidence)
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
                priority = self._track_relabel_priority(
                    track,
                    reason,
                    candidate,
                    semantic_task,
                )
                eligible.append((priority, candidate, track, reason))

        if not eligible:
            return

        eligible.sort(key=lambda item: item[0], reverse=True)
        focus_mode = False
        focus_phase_active = self._vlm_focus_phase_active()
        if self._semantic_task_requested(semantic_task):
            focused = [
                item
                for item in eligible
                if self._is_vlm_focus_candidate(item[2], item[1], semantic_task)
            ]
            if focused:
                eligible = focused
                focus_mode = True
            elif focus_phase_active:
                return
        pending_specs: list[dict[str, Any]] = []
        pending_candidates: list[dict[str, Any]] = []
        semantic_signature = self._semantic_signature(semantic_task)
        candidate_limit = (
            self.vlm_relabel_focus_top_k if focus_mode else self.vlm_relabel_top_k
        )
        with self._track_lock:
            for _, candidate, track, reason in eligible:
                if len(pending_specs) >= candidate_limit:
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
        skip_overview = len(pending_specs) == 1 or (
            focus_mode and self.vlm_relabel_focus_without_overview
        )
        if pending_candidates and not skip_overview:
            overview_rgb = self._render_vlm_relabel_context_rgb(image_rgb, pending_candidates)
            overview_image_url = (
                "data:image/jpeg;base64,"
                f"{encode_image_to_base64_jpeg(overview_rgb, self.vlm_relabel_jpeg_quality)}"
            )
        self.get_logger().info(
            "vlm relabel batch: focus_mode=%s candidates=%d overview=%s semantic_target=%s pick_phase=%s"
            % (
                str(focus_mode).lower(),
                len(pending_specs),
                "yes" if overview_image_url else "no",
                semantic_signature or "<none>",
                self._pick_phase,
            )
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
            "target_instance": self._semantic_target_instance(semantic_task),
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
        xyxy_tensor = self._safe_result_item(getattr(boxes, "xyxy", None), index)
        if xyxy_tensor is None:
            return None
        try:
            bbox_xyxy = [
                float(v)
                for v in (
                    xyxy_tensor.cpu().numpy().tolist()
                    if hasattr(xyxy_tensor, "cpu")
                    else list(xyxy_tensor)
                )
            ]
        except Exception:  # noqa: BLE001
            return None
        return coerce_bbox(bbox_xyxy, image_width, image_height)

    def _extract_mask_u8(
        self,
        masks_data: Optional[np.ndarray],
        index: int,
        image_shape: tuple[int, int],
    ) -> Optional[np.ndarray]:
        if masks_data is None:
            return None
        shape = tuple(int(dim) for dim in getattr(masks_data, "shape", ()) or ())
        if not shape or index < 0 or index >= shape[0]:
            return None
        try:
            mask = np.asarray(masks_data[index], dtype=np.float32)
        except Exception:  # noqa: BLE001
            return None
        if mask.shape[:2] != image_shape:
            mask = cv2.resize(
                mask,
                (int(image_shape[1]), int(image_shape[0])),
                interpolation=cv2.INTER_NEAREST,
            )
        return np.where(mask > 0.5, 255, 0).astype(np.uint8)

    def _safe_result_length(self, value: Any) -> int:
        if value is None:
            return 0
        try:
            return max(0, int(len(value)))
        except Exception:  # noqa: BLE001
            shape = getattr(value, "shape", None)
            if shape is None:
                return 0
            try:
                return max(0, int(shape[0]))
            except Exception:  # noqa: BLE001
                return 0

    def _safe_result_item(self, value: Any, index: int) -> Any:
        if value is None or index < 0:
            return None
        if index >= self._safe_result_length(value):
            return None
        try:
            return value[index]
        except Exception:  # noqa: BLE001
            return None

    def _candidate_count_for_result(
        self,
        boxes: Any,
        masks_data: Optional[np.ndarray],
    ) -> int:
        counts = [
            self._safe_result_length(boxes),
            self._safe_result_length(getattr(boxes, "cls", None)),
            self._safe_result_length(getattr(boxes, "conf", None)),
            self._safe_result_length(getattr(boxes, "xyxy", None)),
        ]
        positive_counts = [count for count in counts if count > 0]
        if not positive_counts:
            return 0
        candidate_count = min(positive_counts)
        mask_count = self._safe_result_length(masks_data)
        if mask_count > 0:
            candidate_count = min(candidate_count, mask_count)
        return max(0, int(candidate_count))

    def _select_debug_candidates(
        self,
        raw_candidates: list[dict[str, Any]],
        selected_candidate: Optional[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        ranked = sorted(
            raw_candidates,
            key=lambda item: (
                float(
                    item.get(
                        "selection_confidence_smoothed",
                        item.get("selection_confidence", item["confidence"]),
                    )
                ),
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
            "confidence": round(
                float(
                    candidate.get(
                        "selection_confidence_smoothed",
                        candidate.get("selection_confidence", candidate.get("confidence", 0.0)),
                    )
                ),
                4,
            ),
            "yolo_confidence": round(float(candidate.get("confidence", 0.0)), 4),
            "semantic_confidence": round(
                float(candidate.get("semantic_confidence", candidate.get("confidence", 0.0))),
                4,
            ),
            "track_stability": round(float(candidate.get("track_stability", 0.0)), 4),
            "selection_confidence": round(
                float(candidate.get("selection_confidence", candidate.get("confidence", 0.0))),
                4,
            ),
            "selection_confidence_smoothed": round(
                float(
                    candidate.get(
                        "selection_confidence_smoothed",
                        candidate.get("selection_confidence", candidate.get("confidence", 0.0)),
                    )
                ),
                4,
            ),
            "confidence_floor": round(float(candidate.get("confidence_floor", 0.0)), 4),
            "semantic_bonus": round(float(candidate.get("semantic_bonus", 0.0)), 4),
            "score": round(float(candidate.get("score", 0.0)), 4),
            "instance_match": bool(candidate.get("instance_match", False)),
            "bbox_xyxy": bbox_list,
            "mask_pixels": int(candidate.get("mask_pixels", 0)),
            "status": str(candidate.get("status", "")),
            "display_status": str(
                candidate.get("display_status", candidate.get("status", ""))
            ),
            "selection_status": str(candidate.get("selection_status", "")),
        }

    def _track_debug_public(
        self,
        track: Optional[dict[str, Any]],
        *,
        bbox_xyxy: Optional[list[int]],
        image_width: int,
        image_height: int,
        status: str,
        selection_status: str = "",
        match_score: float = 0.0,
        now_sec: Optional[float] = None,
    ) -> Optional[dict[str, Any]]:
        if not isinstance(track, dict):
            return None
        current_now = float(now_sec if now_sec is not None else time.time())
        canonical_label = self._normalize_short_label(
            track.get("canonical_label") or track.get("display_label") or track.get("raw_label")
        )
        raw_label = self._normalize_short_label(track.get("raw_label") or canonical_label)
        label_zh = self._normalize_short_label(track.get("label_zh"))
        display_candidate = {
            "index": int(track.get("track_id", -1) or -1),
            "track_id": int(track.get("track_id", -1) or -1),
            "track_new": False,
            "track_seen_count": int(track.get("seen_count", 0) or 0),
            "track_consecutive_hits": int(track.get("consecutive_hits", 0) or 0),
            "track_age_sec": max(
                0.0,
                current_now - float(track.get("first_seen", current_now) or current_now),
            ),
            "track_label_age_sec": max(
                0.0,
                current_now - float(track.get("last_relabel_completed_at", 0.0) or 0.0),
            )
            if float(track.get("last_relabel_completed_at", 0.0) or 0.0) > 0.0
            else 0.0,
            "track_relabel_reason": str(track.get("last_relabel_reason", "") or ""),
            "label": canonical_label or raw_label,
            "raw_label": raw_label,
            "canonical_label": canonical_label or raw_label,
            "label_zh": label_zh,
            "display_label": self._normalize_short_label(
                track.get("display_label") or canonical_label or raw_label
            ),
            "label_source": str(track.get("label_source", "track") or "track"),
            "confidence": float(
                track.get(
                    "selection_confidence_smoothed",
                    track.get("selection_confidence", track.get("confidence", 0.0)),
                )
                or 0.0
            ),
            "yolo_confidence": float(track.get("confidence", 0.0) or 0.0),
            "semantic_confidence": float(
                track.get("semantic_confidence", track.get("confidence", 0.0)) or 0.0
            ),
            "track_stability": float(self._track_stability_confidence(track)),
            "selection_confidence": float(
                track.get("selection_confidence", track.get("confidence", 0.0)) or 0.0
            ),
            "selection_confidence_smoothed": float(
                track.get(
                    "selection_confidence_smoothed",
                    track.get("selection_confidence", track.get("confidence", 0.0)),
                )
                or 0.0
            ),
            "confidence_floor": float(
                track.get("last_confidence_floor", self.confidence_threshold)
                or self.confidence_threshold
            ),
            "semantic_bonus": float(track.get("last_semantic_bonus", 0.0) or 0.0),
            "score": float(
                track.get(
                    "last_score",
                    track.get(
                        "selection_confidence_smoothed",
                        track.get("selection_confidence", track.get("confidence", 0.0)),
                    ),
                )
                or 0.0
            ),
            "instance_match": bool(track.get("last_instance_match", False)),
            "bbox_xyxy": list(bbox_xyxy or []),
            "mask_pixels": int(track.get("mask_pixels", 0) or 0),
            "status": str(status or "tracked"),
            "display_status": str(status or "tracked"),
            "selection_status": str(selection_status or track.get("last_selection_status", "") or ""),
            "image_width": int(image_width),
            "image_height": int(image_height),
        }
        public = self._candidate_debug_public(display_candidate)
        if isinstance(public, dict):
            public["match_score"] = round(float(match_score or 0.0), 4)
        return public

    def _update_track_selection_debug_from_candidates(
        self,
        raw_candidates: list[dict[str, Any]],
    ) -> None:
        if not raw_candidates:
            return
        with self._track_lock:
            for candidate in raw_candidates:
                track_id = int(candidate.get("track_id", -1) or -1)
                if track_id < 0:
                    continue
                track = self._tracks.get(track_id)
                if not isinstance(track, dict):
                    continue
                track["last_selection_status"] = str(candidate.get("status", "") or "")
                track["last_semantic_bonus"] = float(candidate.get("semantic_bonus", 0.0) or 0.0)
                track["last_score"] = float(
                    candidate.get("score", candidate.get("confidence", 0.0)) or 0.0
                )
                track["last_instance_match"] = bool(candidate.get("instance_match", False))
                track["last_confidence_floor"] = float(
                    candidate.get("confidence_floor", self.confidence_threshold)
                    or self.confidence_threshold
                )

    def _display_candidates_from_raw_candidates(
        self,
        image_rgb: np.ndarray,
        raw_candidates: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        return self._display_candidates_from_tracks(
            image_rgb,
            detected_candidates=raw_candidates,
        )

    def _display_candidates_from_tracks(
        self,
        image_rgb: np.ndarray,
        *,
        detected_candidates: Optional[list[dict[str, Any]]] = None,
    ) -> list[dict[str, Any]]:
        now_sec = time.time()
        image_height = int(image_rgb.shape[0])
        image_width = int(image_rgb.shape[1])
        detection_fresh_sec = 1.5 / max(0.5, float(self.max_inference_rate_hz))
        detected_by_track_id: dict[int, dict[str, Any]] = {}
        for candidate in list(detected_candidates or []):
            if not isinstance(candidate, dict):
                continue
            track_id = int(candidate.get("track_id", -1) or -1)
            if track_id < 0:
                continue
            previous = detected_by_track_id.get(track_id)
            if previous is None or self._candidate_priority_key(candidate) > self._candidate_priority_key(previous):
                detected_by_track_id[track_id] = candidate
        with self._track_lock:
            tracks = [copy.deepcopy(track) for track in self._tracks.values() if isinstance(track, dict)]
        display_candidates: list[dict[str, Any]] = []
        track_updates: list[tuple[int, dict[str, Any]]] = []
        for track in tracks:
            track_id = int(track.get("track_id", -1) or -1)
            bbox_xyxy = self._normalize_bbox_xyxy(track.get("bbox_xyxy"), image_width, image_height)
            if track_id < 0 or bbox_xyxy is None:
                continue
            last_seen = float(track.get("last_seen", 0.0) or 0.0)
            last_tracked_at = float(track.get("last_tracked_at", 0.0) or 0.0)
            if (now_sec - max(last_seen, last_tracked_at)) > float(self.bbox_tracking_max_stale_sec):
                continue
            match_score = 0.0
            status = "track_hold"
            selection_status = ""
            detected_candidate = detected_by_track_id.get(track_id)
            if isinstance(detected_candidate, dict):
                detected_bbox = self._normalize_bbox_xyxy(
                    detected_candidate.get("bbox_xyxy"),
                    image_width,
                    image_height,
                )
                if detected_bbox is not None:
                    bbox_xyxy = detected_bbox
                status = "detected"
                selection_status = str(detected_candidate.get("status", "selectable") or "selectable")
            else:
                tracked = self._match_bbox_tracking_state(image_rgb, track)
                if tracked is not None:
                    bbox_xyxy = list(tracked.get("bbox_xyxy") or bbox_xyxy)
                    match_score = float(tracked.get("match_score", 0.0) or 0.0)
                    status = "tracked"
                    track_updates.append(
                        (
                            track_id,
                            {
                                "bbox_xyxy": bbox_xyxy,
                                "center_xy": self._bbox_center(bbox_xyxy),
                                "last_tracked_at": now_sec,
                                "updated_template": tracked.get("updated_template"),
                            },
                        )
                    )
                elif (now_sec - last_seen) <= detection_fresh_sec:
                    status = "detected"
            public = self._track_debug_public(
                track,
                bbox_xyxy=bbox_xyxy,
                image_width=image_width,
                image_height=image_height,
                status=status,
                selection_status=selection_status,
                match_score=match_score,
                now_sec=now_sec,
            )
            if isinstance(public, dict):
                display_candidates.append(public)
        if track_updates:
            with self._track_lock:
                for track_id, update in track_updates:
                    active_track = self._tracks.get(track_id)
                    if not isinstance(active_track, dict):
                        continue
                    active_track["bbox_xyxy"] = list(update["bbox_xyxy"])
                    active_track["center_xy"] = update["center_xy"]
                    active_track["last_tracked_at"] = float(update["last_tracked_at"])
                    updated_template = update.get("updated_template")
                    if isinstance(updated_template, dict):
                        active_track.update(updated_template)
        display_candidates.sort(
            key=lambda item: (
                float(item.get("confidence", 0.0) or 0.0),
                float(item.get("score", 0.0) or 0.0),
                int(item.get("track_id", -1) or -1),
            ),
            reverse=True,
        )
        display_candidates = self._deduplicate_candidates_by_bbox(display_candidates)
        display_candidates.sort(key=self._candidate_priority_key, reverse=True)
        return display_candidates

    def _attach_display_debug_payload(
        self,
        payload: dict[str, Any],
        *,
        image_rgb: np.ndarray,
        semantic_task: Optional[SemanticTask],
        detected_candidates: Optional[list[dict[str, Any]]] = None,
        preferred_candidate: Optional[dict[str, Any]] = None,
    ) -> None:
        display_candidates = self._display_candidates_from_tracks(
            image_rgb,
            detected_candidates=detected_candidates,
        )
        preferred_public = (
            self._candidate_debug_public(preferred_candidate)
            if isinstance(preferred_candidate, dict)
            else None
        )
        selected_candidate = self._resolve_display_selected_candidate(
            display_candidates,
            preferred_candidate=preferred_public,
        )
        debug_candidates = list(display_candidates[: self.debug_top_k_candidates])
        payload["debug_candidates"] = copy.deepcopy(debug_candidates)
        payload["display_candidates"] = copy.deepcopy(display_candidates)
        payload["selected_candidate"] = copy.deepcopy(selected_candidate)
        payload["candidate_summary"] = self._format_candidate_summary(
            semantic_task,
            debug_candidates,
            selected_candidate,
        )
        if self.publish_debug_overlay:
            payload["debug_overlay_rgb"] = self._render_debug_overlay(
                image_rgb,
                debug_candidates,
                selected_candidate,
                semantic_task,
            )

    def _resolve_display_selected_candidate(
        self,
        display_candidates: list[dict[str, Any]],
        preferred_candidate: Optional[dict[str, Any]] = None,
    ) -> Optional[dict[str, Any]]:
        preferred = (
            copy.deepcopy(preferred_candidate)
            if isinstance(preferred_candidate, dict)
            else copy.deepcopy(self._last_selected_candidate_debug)
            if isinstance(self._last_selected_candidate_debug, dict)
            else None
        )
        if not isinstance(preferred, dict):
            return None
        target_track_id = int(preferred.get("track_id", -1) or -1)
        if target_track_id >= 0:
            for candidate in display_candidates:
                if int(candidate.get("track_id", -1) or -1) == target_track_id:
                    return copy.deepcopy(candidate)
        return preferred

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
        target_instance = self._semantic_target_instance(semantic_task) or {}
        target_instance_summary = ""
        if int(target_instance.get("track_id", 0) or 0) > 0:
            target_instance_summary = f"target_instance=T{int(target_instance['track_id'])} "
        elif list(target_instance.get("bbox_xyxy", []) or []):
            target_instance_summary = "target_instance=bbox "
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
                f"(c={float(candidate.get('selection_confidence_smoothed', candidate.get('selection_confidence', candidate['confidence']))):.3f},"
                f"b={float(candidate['semantic_bonus']):+.3f},"
                f"s={float(candidate['score']):.3f},"
                f"status={str(candidate['status'])}{track_suffix}{raw_suffix})"
            )
        return (
            "detector top-k: "
            f"target_label={target_label or '<none>'} "
            f"target_hint={target_hint or '<none>'} "
            f"{target_instance_summary}"
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
                f"c={float(candidate.get('selection_confidence_smoothed', candidate.get('selection_confidence', candidate['confidence']))):.2f} "
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
        if status == "filtered_instance_mismatch":
            return (255, 128, 64)
        return (255, 176, 64)

    def _bbox_center_point(self, bbox_xyxy: Optional[list[int]]) -> Optional[list[int]]:
        if bbox_xyxy is None or not isinstance(bbox_xyxy, list) or len(bbox_xyxy) != 4:
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
        stamp: Any = None,
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
            "stamp": stamp,
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
        stamp: Any = None,
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
            "stamp": stamp,
            "bbox_xyxy": None,
            "point_px": None,
            "mask_u8": None,
        }

    def _publish_detection(self, payload: dict[str, Any]) -> None:
        detection_msg = DetectionResult()
        stamp = payload.get("stamp")
        if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
            detection_msg.header.stamp = stamp
        else:
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
        if detection_msg.has_mask:
            self._last_mask_publish_at = time.time()

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
            "debug_candidates": payload.get(
                "display_candidates", payload.get("debug_candidates", [])
            ),
            "top_candidates": payload.get("debug_candidates", []),
            "candidate_summary": str(payload.get("candidate_summary", "") or ""),
            "inference_scope": str(payload.get("inference_scope", "") or ""),
            "roi_bbox_xyxy": payload.get("roi_bbox_xyxy"),
            "image_width": int(detection_msg.image_width),
            "image_height": int(detection_msg.image_height),
            "scene_prompt": self._scene_prompt_cache_snapshot(),
        }
        self._publish_debug_state(debug_payload)

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
            f"mask={detection_msg.has_mask} "
            f"scope={str(payload.get('inference_scope', '') or 'unknown')}"
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
