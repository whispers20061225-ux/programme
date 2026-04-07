import ast
import base64
import asyncio
import copy
import hashlib
import json
import os
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional
from urllib.parse import urlsplit, urlunsplit
from uuid import uuid4

import cv2
import numpy as np
import requests
import rclpy
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from rclpy.action import ActionClient
try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except Exception:  # noqa: BLE001
    PackageNotFoundError = RuntimeError
    get_package_share_directory = None
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from tactile_interfaces.action import ExecuteTask
from tactile_interfaces.msg import (
    ArmState,
    DetectionResult,
    GraspProposalArray,
    SemanticTask,
    SystemHealth,
    TaskExecutionStatus,
    TaskGoal,
    TactileRaw,
)
from tactile_interfaces.srv import MoveArmJoints


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


def _resolve_dialog_runtime_config(
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
        os.getenv("PROGRAMME_DIALOG_API_KEY")
        or file_values.get("PROGRAMME_DIALOG_API_KEY")
        or os.getenv("PROGRAMME_REMOTE_VLM_API_KEY")
        or file_values.get("PROGRAMME_REMOTE_VLM_API_KEY")
        or os.getenv("DASHSCOPE_API_KEY")
        or file_values.get("DASHSCOPE_API_KEY")
        or os.getenv("OPENAI_API_KEY")
        or file_values.get("OPENAI_API_KEY")
    )
    env_endpoint = _normalize_openai_compatible_endpoint(
        os.getenv("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or file_values.get("PROGRAMME_DIALOG_MODEL_ENDPOINT")
        or os.getenv("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or file_values.get("PROGRAMME_REMOTE_VLM_ENDPOINT")
        or os.getenv("DASHSCOPE_BASE_URL")
        or file_values.get("DASHSCOPE_BASE_URL")
        or os.getenv("OPENAI_BASE_URL")
        or file_values.get("OPENAI_BASE_URL")
    )
    env_model = str(
        os.getenv("PROGRAMME_DIALOG_MODEL_NAME")
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


def _safe_json_loads(raw_text: str) -> dict[str, Any]:
    try:
        parsed = json.loads(raw_text)
    except Exception:
        return {"raw": raw_text}
    if isinstance(parsed, dict):
        return parsed
    return {"value": parsed}


def _compact_json(payload: Any) -> str:
    return json.dumps(_json_compatible(payload), ensure_ascii=False, separators=(",", ":"))


def _extract_model_message_text(payload: dict[str, Any]) -> str:
    choices = payload.get("choices") or []
    if not choices:
        raise ValueError("response missing choices")

    first_choice = choices[0] or {}
    message = first_choice.get("message") or {}
    content = message.get("content")
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        text_parts: list[str] = []
        for item in content:
            if not isinstance(item, dict):
                continue
            text = item.get("text")
            if isinstance(text, str) and text.strip():
                text_parts.append(text.strip())
        if text_parts:
            return "\n".join(text_parts)

    fallback_text = first_choice.get("text")
    if isinstance(fallback_text, str):
        return fallback_text.strip()
    raise ValueError("response missing textual content")


def _extract_first_json_object(text: str) -> dict[str, Any]:
    candidate = str(text or "").strip()
    if not candidate:
        raise ValueError("empty model response")
    decoder = json.JSONDecoder()
    for index, char in enumerate(candidate):
        if char != "{":
            continue
        try:
            parsed, _ = decoder.raw_decode(candidate[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed
    raise ValueError("response does not contain a JSON object")


def _json_compatible(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, deque)):
        return [_json_compatible(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_compatible(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    return value


def _now_sec() -> float:
    return time.time()


def _msg_stamp_sec(msg: Any) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0.0
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9


def _point_to_dict(point: Any) -> dict[str, float]:
    return {
        "x": float(getattr(point, "x", 0.0)),
        "y": float(getattr(point, "y", 0.0)),
        "z": float(getattr(point, "z", 0.0)),
    }


def _vector_to_dict(vector: Any) -> dict[str, float]:
    return {
        "x": float(getattr(vector, "x", 0.0)),
        "y": float(getattr(vector, "y", 0.0)),
        "z": float(getattr(vector, "z", 0.0)),
    }


def _pose_to_dict(pose: Any) -> dict[str, Any]:
    return {
        "position": _point_to_dict(getattr(pose, "position", None)),
        "orientation": {
            "x": float(getattr(getattr(pose, "orientation", None), "x", 0.0)),
            "y": float(getattr(getattr(pose, "orientation", None), "y", 0.0)),
            "z": float(getattr(getattr(pose, "orientation", None), "z", 0.0)),
            "w": float(getattr(getattr(pose, "orientation", None), "w", 1.0)),
        },
    }


def _coerce_int_list(value: Any, *, expected_len: Optional[int] = None) -> list[int]:
    if not isinstance(value, (list, tuple)):
        return []
    try:
        items = [int(round(float(item))) for item in value]
    except (TypeError, ValueError):
        return []
    if expected_len is not None and len(items) != expected_len:
        return []
    return items


def _bbox_center_point_xyxy(bbox_xyxy: Any) -> list[int]:
    bbox = _coerce_int_list(bbox_xyxy, expected_len=4)
    if len(bbox) != 4:
        return []
    return [
        int(round((float(bbox[0]) + float(bbox[2])) * 0.5)),
        int(round((float(bbox[1]) + float(bbox[3])) * 0.5)),
    ]


def _target_instance_hint_dict(
    *,
    track_id: Any = 0,
    bbox_xyxy: Any = None,
    point_px: Any = None,
    source: Any = "",
) -> Optional[dict[str, Any]]:
    try:
        resolved_track_id = int(track_id or 0)
    except (TypeError, ValueError):
        resolved_track_id = 0
    if resolved_track_id < 0:
        resolved_track_id = 0
    bbox = _coerce_int_list(bbox_xyxy, expected_len=4)
    point = _coerce_int_list(point_px, expected_len=2)
    if not point and bbox:
        point = _bbox_center_point_xyxy(bbox)
    resolved_source = str(source or "").strip()
    if resolved_track_id <= 0 and not bbox and not point:
        return None
    payload: dict[str, Any] = {}
    if resolved_track_id > 0:
        payload["track_id"] = resolved_track_id
    if bbox:
        payload["bbox_xyxy"] = bbox
    if point:
        payload["point_px"] = point
    if resolved_source:
        payload["source"] = resolved_source
    return payload or None


def _target_instance_from_payload(payload: Any) -> Optional[dict[str, Any]]:
    if not isinstance(payload, dict):
        return None
    nested = payload.get("target_instance")
    if isinstance(nested, dict):
        return _target_instance_hint_dict(
            track_id=nested.get("track_id", payload.get("target_instance_track_id", 0)),
            bbox_xyxy=nested.get(
                "bbox_xyxy", payload.get("target_instance_bbox_xyxy", [])
            ),
            point_px=nested.get(
                "point_px", payload.get("target_instance_point_px", [])
            ),
            source=nested.get(
                "source", payload.get("target_instance_source", "")
            ),
        )
    return _target_instance_hint_dict(
        track_id=payload.get("target_instance_track_id", 0),
        bbox_xyxy=payload.get("target_instance_bbox_xyxy", []),
        point_px=payload.get("target_instance_point_px", []),
        source=payload.get("target_instance_source", ""),
    )


def _target_instance_from_candidate(
    candidate: Any,
    *,
    source: str = "",
) -> Optional[dict[str, Any]]:
    if not isinstance(candidate, dict):
        return None
    return _target_instance_hint_dict(
        track_id=candidate.get("track_id", 0),
        bbox_xyxy=candidate.get("bbox_xyxy", []),
        point_px=candidate.get("point_px", []),
        source=source or candidate.get("source", ""),
    )


def _semantic_task_to_dict(msg: SemanticTask) -> dict[str, Any]:
    constraints = [str(item) for item in msg.constraints]
    return {
        "task": str(msg.task or "pick"),
        "target_label": str(msg.target_label or ""),
        "target_hint": str(msg.target_hint or ""),
        "target_instance": _target_instance_hint_dict(
            track_id=getattr(msg, "target_instance_track_id", 0),
            bbox_xyxy=list(getattr(msg, "target_instance_bbox_xyxy", []) or []),
            point_px=list(getattr(msg, "target_instance_point_px", []) or []),
            source=str(getattr(msg, "target_instance_source", "") or ""),
        ),
        "constraints": constraints,
        "gripper": constraints[0] if constraints else "",
        "excluded_labels": [str(item) for item in msg.excluded_labels],
        "confidence": float(msg.confidence),
        "need_human_confirm": bool(msg.need_human_confirm),
        "reason": str(msg.reason or ""),
        "prompt_text": str(msg.prompt_text or ""),
        "raw_json": str(msg.raw_json or ""),
        "updated_at": _now_sec(),
        "stamp_sec": _msg_stamp_sec(msg),
    }


def _semantic_payload_defaults(semantic: dict[str, Any]) -> dict[str, Any]:
    constraints = [
        str(item).strip()
        for item in list(semantic.get("constraints", []) or [])
        if str(item).strip()
    ]
    return {
        "task": str(semantic.get("task", "pick") or "pick"),
        "target_label": str(semantic.get("target_label", "") or ""),
        "target_hint": str(semantic.get("target_hint", "") or ""),
        "target_instance": _target_instance_from_payload(semantic),
        "constraints": constraints,
        "excluded_labels": [
            str(item).strip()
            for item in list(semantic.get("excluded_labels", []) or [])
            if str(item).strip()
        ],
        "confidence": float(semantic.get("confidence", 1.0) or 1.0),
        "need_human_confirm": bool(semantic.get("need_human_confirm", False)),
        "reason": str(semantic.get("reason", "") or ""),
    }


def _task_goal_to_dict(msg: TaskGoal) -> dict[str, Any]:
    return {
        "schema_version": str(getattr(msg, "schema_version", "") or ""),
        "goal_id": str(msg.goal_id or ""),
        "parent_goal_id": str(getattr(msg, "parent_goal_id", "") or ""),
        "step_id": str(getattr(msg, "step_id", "") or ""),
        "step_index": int(getattr(msg, "step_index", 0) or 0),
        "goal_type": str(msg.goal_type or ""),
        "task": str(msg.task or "pick"),
        "target_label": str(msg.target_label or ""),
        "target_hint": str(msg.target_hint or ""),
        "target_attributes": [str(item) for item in getattr(msg, "target_attributes", [])],
        "target_relations": [str(item) for item in getattr(msg, "target_relations", [])],
        "target_instance": _target_instance_hint_dict(
            track_id=getattr(msg, "target_instance_track_id", 0),
            bbox_xyxy=list(getattr(msg, "target_instance_bbox_xyxy", []) or []),
            point_px=list(getattr(msg, "target_instance_point_px", []) or []),
            source=str(getattr(msg, "target_instance_source", "") or ""),
        ),
        "target_part_query": str(getattr(msg, "target_part_query", "") or ""),
        "preferred_part_tags": [str(item) for item in getattr(msg, "preferred_part_tags", [])],
        "forbidden_part_tags": [str(item) for item in getattr(msg, "forbidden_part_tags", [])],
        "grasp_region": str(msg.grasp_region or ""),
        "preferred_grasp_family": str(getattr(msg, "preferred_grasp_family", "") or ""),
        "placement_label": str(msg.placement_label or ""),
        "preferred_approach_dirs": [
            str(item) for item in getattr(msg, "preferred_approach_dirs", [])
        ],
        "forbidden_approach_dirs": [
            str(item) for item in getattr(msg, "forbidden_approach_dirs", [])
        ],
        "transport_constraints": [str(item) for item in msg.transport_constraints],
        "execution_constraints": [str(item) for item in msg.execution_constraints],
        "excluded_labels": [str(item) for item in msg.excluded_labels],
        "reserved_next_skills": [str(item) for item in getattr(msg, "reserved_next_skills", [])],
        "confidence": float(msg.confidence),
        "min_affordance_score": float(getattr(msg, "min_affordance_score", 0.0) or 0.0),
        "min_grasp_quality": float(getattr(msg, "min_grasp_quality", 0.0) or 0.0),
        "max_forbidden_overlap": float(getattr(msg, "max_forbidden_overlap", 0.0) or 0.0),
        "max_target_staleness_sec": float(
            getattr(msg, "max_target_staleness_sec", 0.0) or 0.0
        ),
        "need_human_confirm": bool(msg.need_human_confirm),
        "start_search_sweep": bool(msg.start_search_sweep),
        "allow_instance_grounding": bool(msg.allow_instance_grounding),
        "require_target_lock": bool(msg.require_target_lock),
        "allow_replan": bool(msg.allow_replan),
        "allow_rescan": bool(msg.allow_rescan),
        "strict_part_match": bool(getattr(msg, "strict_part_match", False)),
        "strict_approach_match": bool(getattr(msg, "strict_approach_match", False)),
        "max_retries": int(msg.max_retries),
        "planning_timeout_sec": float(msg.planning_timeout_sec),
        "execution_timeout_sec": float(msg.execution_timeout_sec),
        "success_criteria": str(msg.success_criteria or ""),
        "failure_policy": str(msg.failure_policy or ""),
        "verify_policy": str(getattr(msg, "verify_policy", "") or ""),
        "recovery_policy": str(getattr(msg, "recovery_policy", "") or ""),
        "handoff_context_json": str(getattr(msg, "handoff_context_json", "") or ""),
        "reason": str(msg.reason or ""),
        "prompt_text": str(msg.prompt_text or ""),
        "raw_json": str(msg.raw_json or ""),
        "updated_at": _now_sec(),
        "stamp_sec": _msg_stamp_sec(msg),
    }


def _task_execution_status_to_dict(msg: TaskExecutionStatus) -> dict[str, Any]:
    return {
        "goal_id": str(msg.goal_id or ""),
        "parent_goal_id": str(getattr(msg, "parent_goal_id", "") or ""),
        "step_id": str(getattr(msg, "step_id", "") or ""),
        "phase": str(msg.phase or "idle"),
        "current_skill": str(msg.current_skill or ""),
        "skill_status_code": str(getattr(msg, "skill_status_code", "") or ""),
        "message": str(msg.message or ""),
        "error_code": str(msg.error_code or ""),
        "target_label": str(msg.target_label or ""),
        "target_hint": str(msg.target_hint or ""),
        "target_part_query": str(getattr(msg, "target_part_query", "") or ""),
        "grasp_region": str(msg.grasp_region or ""),
        "active": bool(msg.active),
        "success": bool(msg.success),
        "requires_confirmation": bool(msg.requires_confirmation),
        "target_locked": bool(msg.target_locked),
        "target_candidate_visible": bool(msg.target_candidate_visible),
        "pick_active": bool(msg.pick_active),
        "retry_count": int(msg.retry_count),
        "max_retries": int(msg.max_retries),
        "grounded_track_id": int(msg.grounded_track_id),
        "selected_candidate_id": int(getattr(msg, "selected_candidate_id", 0) or 0),
        "progress": float(msg.progress),
        "grounding_confidence": float(getattr(msg, "grounding_confidence", 0.0) or 0.0),
        "best_grasp_quality": float(getattr(msg, "best_grasp_quality", 0.0) or 0.0),
        "best_affordance_score": float(getattr(msg, "best_affordance_score", 0.0) or 0.0),
        "verification_score": float(getattr(msg, "verification_score", 0.0) or 0.0),
        "recommended_recovery": str(getattr(msg, "recommended_recovery", "") or ""),
        "updated_at": _now_sec(),
        "stamp_sec": _msg_stamp_sec(msg),
    }


def _debug_candidates_from_payload(detection_debug: dict[str, Any]) -> list[dict[str, Any]]:
    empty_candidates: list[dict[str, Any]] = []
    for key in ("display_candidates", "debug_candidates", "top_candidates"):
        items = detection_debug.get(key)
        if not isinstance(items, list):
            continue
        candidates = [item for item in items if isinstance(item, dict)]
        if candidates:
            return candidates
        empty_candidates = candidates
    return empty_candidates


def _detection_result_to_dict(msg: DetectionResult) -> dict[str, Any]:
    bbox = None
    if msg.has_bbox:
        bbox = {
            "x_offset": int(msg.bbox.x_offset),
            "y_offset": int(msg.bbox.y_offset),
            "width": int(msg.bbox.width),
            "height": int(msg.bbox.height),
            "xyxy": [
                int(msg.bbox.x_offset),
                int(msg.bbox.y_offset),
                int(msg.bbox.x_offset + msg.bbox.width),
                int(msg.bbox.y_offset + msg.bbox.height),
            ],
        }
    return {
        "backend": str(msg.backend or ""),
        "accepted": bool(msg.accepted),
        "candidate_visible": bool(msg.candidate_visible),
        "candidate_complete": bool(msg.candidate_complete),
        "task": str(msg.task or ""),
        "target_label": str(msg.target_label or ""),
        "confidence": float(msg.confidence),
        "need_human_confirm": bool(msg.need_human_confirm),
        "reason": str(msg.reason or ""),
        "image_width": int(msg.image_width),
        "image_height": int(msg.image_height),
        "bbox": bbox,
        "point_px": list(msg.point_px) if msg.has_point else None,
        "has_mask": bool(msg.has_mask),
        "updated_at": _now_sec(),
        "stamp_sec": _msg_stamp_sec(msg),
    }


def _proposal_to_dict(proposal: Any) -> dict[str, Any]:
    return {
        "contact_point_1": _point_to_dict(proposal.contact_point_1),
        "contact_point_2": _point_to_dict(proposal.contact_point_2),
        "grasp_center": _point_to_dict(proposal.grasp_center),
        "approach_direction": _vector_to_dict(proposal.approach_direction),
        "closing_direction": _vector_to_dict(proposal.closing_direction),
        "grasp_pose": _pose_to_dict(proposal.grasp_pose),
        "pregrasp_pose": _pose_to_dict(proposal.pregrasp_pose),
        "grasp_width_m": float(proposal.grasp_width_m),
        "confidence_score": float(proposal.confidence_score),
        "semantic_score": float(proposal.semantic_score),
        "candidate_rank": int(proposal.candidate_rank),
        "task_constraint_tag": str(proposal.task_constraint_tag or ""),
    }


def _arm_state_to_dict(msg: ArmState) -> dict[str, Any]:
    return {
        "connected": bool(msg.connected),
        "moving": bool(msg.moving),
        "error": bool(msg.error),
        "error_message": str(msg.error_message or ""),
        "battery_voltage": float(msg.battery_voltage),
        "joint_positions": [float(item) for item in msg.joint_positions],
        "joint_angles": [float(item) for item in msg.joint_angles],
        "updated_at": _now_sec(),
        "stamp_sec": _msg_stamp_sec(msg),
    }


@dataclass
class StreamFrame:
    jpeg: Optional[bytes] = None
    updated_at: float = 0.0
    width: int = 0
    height: int = 0
    encoding: str = ""
    source_stamp_sec: float = 0.0


@dataclass
class RawImageFrame:
    data: bytes
    width: int
    height: int
    step: int
    encoding: str
    source_stamp_sec: float


def _frame_rate_from_times(update_times: list[float]) -> float:
    if len(update_times) < 2:
        return 0.0
    span = max(1e-6, update_times[-1] - update_times[0])
    return float(len(update_times) - 1) / span


def _image_msg_to_raw_frame(msg: Image) -> RawImageFrame:
    return RawImageFrame(
        data=bytes(msg.data),
        width=int(msg.width),
        height=int(msg.height),
        step=int(getattr(msg, "step", 0) or 0),
        encoding=str(msg.encoding or ""),
        source_stamp_sec=_msg_stamp_sec(msg),
    )


def _raw_image_to_jpeg(frame: RawImageFrame, jpeg_quality: int) -> bytes:
    width = int(frame.width)
    height = int(frame.height)
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image dimensions: {width}x{height}")

    encoding = str(frame.encoding or "")
    row_bytes = 0
    channels = 0
    convert_code: Optional[int] = None
    grayscale = False

    if encoding in ("rgb8", "bgr8"):
        channels = 3
        row_bytes = width * channels
        convert_code = cv2.COLOR_RGB2BGR if encoding == "rgb8" else None
    elif encoding in ("rgba8", "bgra8"):
        channels = 4
        row_bytes = width * channels
        convert_code = cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
    elif encoding in ("mono8", "8UC1"):
        channels = 1
        row_bytes = width
        grayscale = True
    else:
        raise ValueError(f"unsupported image encoding for MJPEG stream: {encoding}")

    step = int(frame.step or row_bytes)
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than row bytes {row_bytes}")
    required_bytes = step * height
    if len(frame.data) < required_bytes:
        raise ValueError(
            f"image buffer too small: got {len(frame.data)} expected at least {required_bytes}"
        )

    flat = np.frombuffer(frame.data, dtype=np.uint8, count=required_bytes)
    rows = flat.reshape((height, step))
    pixels = rows[:, :row_bytes]
    if step != row_bytes:
        pixels = pixels.copy()
    if grayscale:
        array = cv2.cvtColor(pixels.reshape((height, width)), cv2.COLOR_GRAY2BGR)
    else:
        array = pixels.reshape((height, width, channels))
        if convert_code is not None:
            array = cv2.cvtColor(array, convert_code)

    ok, encoded = cv2.imencode(
        ".jpg",
        array,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise ValueError("cv2.imencode returned false")
    return encoded.tobytes()


def _jpeg_bytes_to_data_url(jpeg: bytes) -> str:
    return "data:image/jpeg;base64," + base64.b64encode(jpeg).decode("ascii")


def _jpeg_to_bgr_image(jpeg: bytes) -> np.ndarray:
    flat = np.frombuffer(jpeg, dtype=np.uint8)
    image = cv2.imdecode(flat, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode jpeg bytes")
    return image


def _resize_bgr_image(image_bgr: np.ndarray, max_side: int) -> np.ndarray:
    height, width = image_bgr.shape[:2]
    limit = max(1, int(max_side))
    current_max_side = max(height, width)
    if current_max_side <= limit:
        return image_bgr
    scale = float(limit) / float(current_max_side)
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    return cv2.resize(image_bgr, (resized_width, resized_height), interpolation=cv2.INTER_AREA)


def _bgr_image_to_jpeg(image_bgr: np.ndarray, jpeg_quality: int) -> bytes:
    ok, encoded = cv2.imencode(
        ".jpg",
        image_bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise ValueError("cv2.imencode returned false")
    return encoded.tobytes()


def _candidate_bbox_xyxy(candidate: dict[str, Any]) -> Optional[tuple[int, int, int, int]]:
    bbox = candidate.get("bbox_xyxy", [])
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return None
    try:
        x1, y1, x2, y2 = [int(round(float(item))) for item in bbox]
    except (TypeError, ValueError):
        return None
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _normalize_dialog_label_text(value: Any) -> str:
    text = str(value or "").replace("\n", " ").replace("\r", " ").strip()
    while "  " in text:
        text = text.replace("  ", " ")
    return text


def _dialog_text_tokens(text: str) -> set[str]:
    normalized = _normalize_dialog_label_text(text).lower()
    for mark in (",", "/", "-", "(", ")", ":", ";", "|"):
        normalized = normalized.replace(mark, " ")
    return {token for token in normalized.split() if token}


_DIALOG_TERM_ALIASES = {
    "blue": ["blue", "\u84dd\u8272"],
    "red": ["red", "\u7ea2\u8272"],
    "green": ["green", "\u7eff\u8272"],
    "left": ["left", "\u5de6\u8fb9", "\u5de6\u4fa7"],
    "right": ["right", "\u53f3\u8fb9", "\u53f3\u4fa7"],
    "center": ["center", "middle", "\u4e2d\u95f4", "\u4e2d\u592e"],
    "cup": ["cup", "\u676f\u5b50", "\u676f"],
    "remote": ["remote", "\u9065\u63a7\u5668"],
    "bottle": ["bottle", "\u74f6\u5b50"],
    "cylinder": ["cylinder", "\u5706\u67f1\u4f53", "\u5706\u67f1"],
    "block": ["block", "\u65b9\u5757", "\u5757"],
}


def _dialog_primary_non_ascii_alias(token: str) -> str:
    aliases = list(_DIALOG_TERM_ALIASES.get(token, []))
    for alias in aliases:
        if any(ord(char) > 127 for char in alias):
            return alias
    return ""


def _dialog_text_aliases(text: str) -> list[str]:
    normalized = _normalize_dialog_label_text(text).lower()
    if not normalized:
        return []
    aliases: list[str] = []
    seen: set[str] = {normalized}
    direct_aliases = list(_DIALOG_TERM_ALIASES.get(normalized, []))
    for alias in direct_aliases:
        lowered = alias.lower()
        if lowered not in seen:
            seen.add(lowered)
            aliases.append(alias)
    tokens = list(_dialog_text_tokens(normalized))
    chinese_parts: list[str] = []
    if tokens:
        for token in tokens:
            for alias in _DIALOG_TERM_ALIASES.get(token, []):
                lowered = alias.lower()
                if lowered not in seen:
                    seen.add(lowered)
                    aliases.append(alias)
            chinese_alias = _dialog_primary_non_ascii_alias(token)
            if chinese_alias:
                chinese_parts.append(chinese_alias)
        if chinese_parts and len(chinese_parts) == len(tokens):
            combined = "".join(chinese_parts)
            if combined.lower() not in seen:
                aliases.append(combined)
    return aliases


_DIALOG_LEFT_KEYWORDS = (
    "left",
    "leftmost",
    "on the left",
    "\u5de6",
    "\u5de6\u8fb9",
    "\u5de6\u4fa7",
    "\u5de6\u8fb9\u90a3\u4e2a",
)

_DIALOG_RIGHT_KEYWORDS = (
    "right",
    "rightmost",
    "on the right",
    "\u53f3",
    "\u53f3\u8fb9",
    "\u53f3\u4fa7",
    "\u53f3\u8fb9\u90a3\u4e2a",
)

_DIALOG_CENTER_KEYWORDS = (
    "center",
    "middle",
    "in the middle",
    "\u4e2d\u95f4",
    "\u4e2d\u95f4\u90a3\u4e2a",
    "\u4e2d\u592e",
)


def _dialog_extract_direction_hints(text: str) -> set[str]:
    lowered = _normalize_dialog_label_text(text).lower()
    if not lowered:
        return set()
    hints: set[str] = set()
    if any(keyword in lowered for keyword in _DIALOG_LEFT_KEYWORDS):
        hints.add("left")
    if any(keyword in lowered for keyword in _DIALOG_RIGHT_KEYWORDS):
        hints.add("right")
    if any(keyword in lowered for keyword in _DIALOG_CENTER_KEYWORDS):
        hints.add("center")
    return hints


def _dialog_candidate_side(candidate: dict[str, Any]) -> str:
    bbox = _candidate_bbox_xyxy(candidate)
    if bbox is None:
        return ""
    image_width = int(candidate.get("image_width", 0) or candidate.get("frame_width", 0) or 0)
    if image_width <= 0:
        return ""
    center_x = (float(bbox[0]) + float(bbox[2])) * 0.5
    ratio = center_x / float(image_width)
    if ratio <= 0.4:
        return "left"
    if ratio >= 0.6:
        return "right"
    return "center"


def _dialog_candidate_primary_label(candidate: Optional[dict[str, Any]]) -> str:
    if not isinstance(candidate, dict):
        return ""
    for key in ("display_label", "canonical_label", "label_zh", "label", "raw_label"):
        text = _normalize_dialog_label_text(candidate.get(key, ""))
        if text:
            return text
    return ""


def _dialog_candidate_texts(candidate: Optional[dict[str, Any]]) -> list[str]:
    if not isinstance(candidate, dict):
        return []
    texts: list[str] = []
    seen: set[str] = set()
    for key in ("display_label", "canonical_label", "label_zh", "label", "raw_label", "target_hint"):
        text = _normalize_dialog_label_text(candidate.get(key, ""))
        lowered = text.lower()
        if not lowered or lowered in seen:
            continue
        seen.add(lowered)
        texts.append(text)
        for alias in _dialog_text_aliases(text):
            alias_lowered = alias.lower()
            if alias_lowered and alias_lowered not in seen:
                seen.add(alias_lowered)
                texts.append(alias)
    side = _dialog_candidate_side(candidate)
    if side and side not in seen:
        texts.append(side)
    return texts


def _dialog_focus_target_hint(focus: Optional[dict[str, Any]]) -> str:
    if not isinstance(focus, dict):
        return ""
    target_hint = _normalize_dialog_label_text(focus.get("target_hint", ""))
    canonical = _normalize_dialog_label_text(
        focus.get("canonical_label") or focus.get("label") or focus.get("display_label")
    )
    base = canonical or target_hint
    side = _normalize_dialog_label_text(focus.get("side", "")).lower()
    if side in {"left", "right", "center"} and base:
        tokens = _dialog_text_tokens(base)
        if side not in tokens:
            return f"{side} {base}".strip()
    return target_hint or base


def _dialog_labels_related(left: str, right: str) -> bool:
    left_text = _normalize_dialog_label_text(left).lower()
    right_text = _normalize_dialog_label_text(right).lower()
    if not left_text or not right_text:
        return False
    if left_text == right_text or left_text in right_text or right_text in left_text:
        return True
    return bool(_dialog_text_tokens(left_text).intersection(_dialog_text_tokens(right_text)))


_DIALOG_VISUAL_QUERY_KEYWORDS = (
    "see",
    "look",
    "visible",
    "what",
    "where",
    "recognize",
    "identify",
    "show",
    "image",
    "picture",
    "frame",
    "scene",
    "object",
    "thing",
    "看到",
    "看见",
    "能看到",
    "看得到",
    "能不能看到",
    "有没有",
    "图像",
    "画面",
    "图片",
    "画里",
    "场景",
    "物体",
    "目标",
    "识别",
    "在哪",
    "是什么",
)

_DIALOG_DEICTIC_KEYWORDS = (
    "this",
    "that",
    "it",
    "this one",
    "that one",
    "这个",
    "那个",
    "它",
    "这一个",
    "那一个",
)


def _dialog_mentions_visual_query(text: str) -> bool:
    lowered = str(text or "").strip().lower()
    if not lowered:
        return False
    return any(keyword in lowered for keyword in _DIALOG_VISUAL_QUERY_KEYWORDS)


def _dialog_mentions_deictic_reference(text: str) -> bool:
    lowered = str(text or "").strip().lower()
    if not lowered:
        return False
    return any(keyword in lowered for keyword in _DIALOG_DEICTIC_KEYWORDS)


_DIALOG_ANSWER_QUERY_KEYWORDS = (
    "why",
    "how",
    "status",
    "state",
    "reason",
    "explain",
    "describe",
    "what happened",
    "why not",
    "\u4e3a\u4ec0\u4e48",
    "\u600e\u4e48",
    "\u89e3\u91ca",
    "\u8bf4\u660e",
    "\u539f\u56e0",
    "\u72b6\u6001",
    "\u53d1\u751f\u4e86\u4ec0\u4e48",
    "\u4e3a\u4ec0\u4e48\u6ca1",
)

_DIALOG_EXECUTE_KEYWORDS = (
    "pick",
    "grab",
    "grasp",
    "take",
    "lift",
    "execute",
    "run",
    "go pick",
    "\u6293",
    "\u6293\u53d6",
    "\u5939",
    "\u5939\u53d6",
    "\u62ff",
    "\u62ff\u8d77",
    "\u6267\u884c",
    "\u53bb\u6293",
    "\u5e2e\u6211\u6293",
)

_DIALOG_CANCEL_KEYWORDS = (
    "cancel",
    "stop",
    "abort",
    "never mind",
    "don't",
    "do not",
    "\u53d6\u6d88",
    "\u505c\u6b62",
    "\u5148\u522b",
    "\u4e0d\u8981",
    "\u7b97\u4e86",
)


def _dialog_mentions_answer_query(text: str) -> bool:
    lowered = str(text or "").strip().lower()
    if not lowered:
        return False
    return (
        _dialog_mentions_visual_query(lowered)
        or any(keyword in lowered for keyword in _DIALOG_ANSWER_QUERY_KEYWORDS)
        or lowered.endswith("?")
        or lowered.endswith("\uFF1F")
    )


def _dialog_mentions_execute_intent(text: str) -> bool:
    lowered = str(text or "").strip().lower()
    if not lowered:
        return False
    return any(keyword in lowered for keyword in _DIALOG_EXECUTE_KEYWORDS)


def _dialog_mentions_cancel_intent(text: str) -> bool:
    lowered = str(text or "").strip().lower()
    if not lowered:
        return False
    return any(keyword in lowered for keyword in _DIALOG_CANCEL_KEYWORDS)


def _clean_dialog_assistant_text(text: str) -> str:
    cleaned = str(text or "").strip()
    if cleaned.startswith("```"):
        lines = cleaned.splitlines()
        if lines:
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        cleaned = "\n".join(lines).strip()
    return cleaned


def _dialog_assistant_text_is_low_signal(text: str) -> bool:
    lowered = _clean_dialog_assistant_text(text).strip().lower().rstrip(".!?")
    return lowered in {"", "yes", "no", "ok", "okay", "sure"}


def _dialog_assistant_text_mentions_execution(text: str) -> bool:
    lowered = _clean_dialog_assistant_text(text).strip().lower()
    return any(
        keyword in lowered
        for keyword in (
            "pick",
            "grab",
            "grasp",
            "execute",
            "take",
            "\u6293",
            "\u6293\u53d6",
            "\u6267\u884c",
            "\u5939",
            "\u62ff",
        )
    )


def _normalize_dialog_reply_language(value: Any) -> str:
    return "en" if str(value or "").strip().lower() == "en" else "zh"


def _dialog_coerce_text_value(value: Any) -> str:
    if isinstance(value, (list, tuple, set)):
        for item in value:
            text = _dialog_coerce_text_value(item)
            if text:
                return text
        return ""
    text = str(value or "").strip()
    if not text:
        return ""
    if text.startswith("[") and text.endswith("]"):
        try:
            parsed = ast.literal_eval(text)
        except Exception:
            parsed = None
        if parsed is not None and parsed is not value:
            coerced = _dialog_coerce_text_value(parsed)
            if coerced:
                return coerced
    return text


def _dialog_coerce_text_list(value: Any) -> list[str]:
    items: list[Any]
    if isinstance(value, (list, tuple, set)):
        items = list(value)
    else:
        text = str(value or "").strip()
        if not text:
            return []
        parsed = None
        if text.startswith("[") and text.endswith("]"):
            try:
                parsed = ast.literal_eval(text)
            except Exception:
                parsed = None
        if isinstance(parsed, (list, tuple, set)):
            items = list(parsed)
        else:
            items = [value]
    normalized: list[str] = []
    seen: set[str] = set()
    for item in items:
        text = _dialog_coerce_text_value(item)
        if not text:
            continue
        key = text.strip().lower()
        if key in seen:
            continue
        seen.add(key)
        normalized.append(text)
    return normalized


def _dialog_reply_text(reply_language: str, zh_text: str, en_text: str) -> str:
    return str(en_text if _normalize_dialog_reply_language(reply_language) == "en" else zh_text)


def _dialog_text_language_scores(text: str) -> tuple[int, int]:
    cleaned = _clean_dialog_assistant_text(text)
    zh_score = sum(1 for char in cleaned if "\u4e00" <= char <= "\u9fff")
    en_score = sum(1 for char in cleaned if char.isascii() and char.isalpha())
    return zh_score, en_score


def _dialog_text_matches_reply_language(text: str, reply_language: str) -> bool:
    normalized = _normalize_dialog_reply_language(reply_language)
    zh_score, en_score = _dialog_text_language_scores(text)
    if normalized == "zh":
        if zh_score >= 2:
            return True
        return zh_score > 0 and zh_score >= en_score
    if en_score >= 4 and zh_score == 0:
        return True
    return en_score > 0 and en_score >= (zh_score * 2)


def _dialog_target_hint_is_low_signal(text: str) -> bool:
    lowered = str(text or "").strip().lower()
    if not lowered:
        return True
    if any(mark in lowered for mark in (".", "!", "?", "\u3002", "\uFF01", "\uFF1F")):
        return True
    return lowered in {
        "yes",
        "no",
        "object",
        "thing",
        "visible object",
        "unknown",
        "low confidence",
        "high confidence",
    } or "confidence" in lowered or len(lowered) > 32


def _infer_dialog_requested_action(user_text: str) -> str:
    if _dialog_mentions_cancel_intent(user_text):
        return "cancel"
    if _dialog_mentions_execute_intent(user_text):
        return "execute"
    if _dialog_mentions_answer_query(user_text):
        return "answer"
    return "update_task"


class MjpegFrameBuffer:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame = StreamFrame()
        self._frame_count = 0
        self._changed_frame_count = 0
        self._last_digest = ""
        self._update_times: deque[float] = deque(maxlen=180)

    def update(
        self,
        jpeg: bytes,
        *,
        width: int,
        height: int,
        encoding: str,
        source_stamp_sec: float,
    ) -> None:
        with self._condition:
            now_sec = _now_sec()
            digest = hashlib.sha1(jpeg).hexdigest()
            self._frame_count += 1
            if digest != self._last_digest:
                self._changed_frame_count += 1
                self._last_digest = digest
            self._update_times.append(now_sec)
            self._frame = StreamFrame(
                jpeg=jpeg,
                updated_at=now_sec,
                width=int(width),
                height=int(height),
                encoding=str(encoding or ""),
                source_stamp_sec=float(source_stamp_sec),
            )
            self._condition.notify_all()

    def wait_next(self, previous_update: float, timeout_sec: float = 1.0) -> StreamFrame:
        with self._condition:
            if self._frame.updated_at <= previous_update:
                self._condition.wait(timeout_sec)
            return self._frame

    def latest(self) -> StreamFrame:
        with self._condition:
            return self._frame

    def stats(self) -> dict[str, Any]:
        with self._condition:
            frame = self._frame
            update_times = list(self._update_times)
            frame_count = int(self._frame_count)
            changed_frame_count = int(self._changed_frame_count)
        fps = _frame_rate_from_times(update_times)
        changed_fps = 0.0
        if update_times:
            span = max(1e-6, update_times[-1] - update_times[0]) if len(update_times) >= 2 else 1.0
            changed_fps = float(min(changed_frame_count, len(update_times))) / span
        now_sec = _now_sec()
        return {
            "updated_at": float(frame.updated_at),
            "age_sec": max(0.0, now_sec - float(frame.updated_at)) if frame.updated_at else None,
            "frame_count": frame_count,
            "changed_frame_count": changed_frame_count,
            "approx_fps": fps,
            "approx_changed_fps": changed_fps,
            "width": int(frame.width),
            "height": int(frame.height),
            "encoding": str(frame.encoding or ""),
            "jpeg_bytes": len(frame.jpeg) if frame.jpeg is not None else 0,
            "source_stamp_sec": float(frame.source_stamp_sec),
            "source_age_sec": max(0.0, now_sec - float(frame.source_stamp_sec))
            if frame.source_stamp_sec
            else None,
        }


class AsyncImageEncoder:
    def __init__(
        self,
        *,
        stream_name: str,
        logger: Any,
        jpeg_quality: int,
        buffer: MjpegFrameBuffer,
    ) -> None:
        self._stream_name = str(stream_name)
        self._logger = logger
        self._jpeg_quality = int(jpeg_quality)
        self._buffer = buffer
        self._condition = threading.Condition()
        self._pending: Optional[RawImageFrame] = None
        self._stopped = False
        self._received_count = 0
        self._dropped_pending_count = 0
        self._encoded_count = 0
        self._receive_times: deque[float] = deque(maxlen=180)
        self._thread = threading.Thread(
            target=self._run,
            name=f"{self._stream_name}-jpeg-encoder",
            daemon=True,
        )
        self._thread.start()

    def submit(self, msg: Image) -> None:
        frame = _image_msg_to_raw_frame(msg)
        with self._condition:
            self._received_count += 1
            self._receive_times.append(_now_sec())
            if self._pending is not None:
                self._dropped_pending_count += 1
            self._pending = frame
            self._condition.notify()

    def close(self) -> None:
        with self._condition:
            self._stopped = True
            self._condition.notify_all()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def stats(self) -> dict[str, Any]:
        base_stats = self._buffer.stats()
        with self._condition:
            receive_times = list(self._receive_times)
            received_count = int(self._received_count)
            dropped_pending_count = int(self._dropped_pending_count)
            encoded_count = int(self._encoded_count)
            has_pending = self._pending is not None
        return {
            **base_stats,
            "input_frame_count": received_count,
            "input_approx_fps": _frame_rate_from_times(receive_times),
            "encoded_frame_count": encoded_count,
            "dropped_pending_count": dropped_pending_count,
            "has_pending_frame": has_pending,
        }

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._stopped and self._pending is None:
                    self._condition.wait(timeout=1.0)
                if self._stopped:
                    return
                frame = self._pending
                self._pending = None
            if frame is None:
                continue
            try:
                jpeg = _raw_image_to_jpeg(frame, self._jpeg_quality)
            except Exception as exc:  # noqa: BLE001
                self._logger.warn(
                    f"failed to encode {self._stream_name} image for MJPEG stream: {exc}"
                )
                continue
            self._buffer.update(
                jpeg,
                width=frame.width,
                height=frame.height,
                encoding=frame.encoding,
                source_stamp_sec=frame.source_stamp_sec,
            )
            with self._condition:
                self._encoded_count += 1


def _dialog_status_label(status: str) -> str:
    labels = {
        "idle": "Idle",
        "thinking": "Thinking",
        "ready": "Ready",
        "awaiting_lock": "Waiting for Target Lock",
        "auto_executing": "Auto Executing",
        "error": "Error",
    }
    return labels.get(status, status or "Idle")


def _dialog_message_summary(
    semantic_payload: Optional[dict[str, Any]],
    requested_action: str,
) -> str:
    if not semantic_payload:
        return f"Action: {requested_action or 'review'}"
    target = str(
        semantic_payload.get("target_label")
        or semantic_payload.get("target_hint")
        or "--"
    )
    constraints = list(semantic_payload.get("constraints", []) or [])
    gripper = str(constraints[0] if constraints else "--")
    return (
        f"Task: {semantic_payload.get('task', 'pick')} | "
        f"Target: {target} | "
        f"Gripper: {gripper} | "
        f"Action: {requested_action or 'update_task'}"
    )


class TactileWebGateway(Node):
    def __init__(self) -> None:
        super().__init__("tactile_web_gateway")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("frontend_dist_dir", "")
        self.declare_parameter("live_state_period_sec", 0.15)
        self.declare_parameter("prompt_topic", "/qwen/user_prompt")
        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("semantic_result_topic", "/qwen/semantic_result")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detection_result_topic", "/perception/detection_result")
        self.declare_parameter("detection_debug_topic", "/perception/detection_debug")
        self.declare_parameter(
            "detection_debug_overlay_topic", "/perception/detection_debug_overlay"
        )
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter("pick_active_topic", "/sim/task/pick_active")
        self.declare_parameter("pick_status_topic", "/sim/task/pick_status")
        self.declare_parameter(
            "candidate_grasp_proposal_array_topic", "/grasp/candidate_grasp_proposals"
        )
        self.declare_parameter("grasp_backend_debug_topic", "/grasp/backend_debug")
        self.declare_parameter("grasp_overlay_topic", "/grasp/ggcnn/grasp_overlay")
        self.declare_parameter("tactile_topic", "/tactile/raw")
        self.declare_parameter("health_topic", "/system/health")
        self.declare_parameter("arm_state_topic", "/arm/state")
        self.declare_parameter("execute_task_action", "/task/execute_task")
        self.declare_parameter("task_goal_topic", "/task/goal")
        self.declare_parameter("task_execution_status_topic", "/task/execution_status")
        self.declare_parameter("execute_pick_service", "/task/execute_pick")
        self.declare_parameter("reset_pick_session_service", "/task/reset_pick_session")
        self.declare_parameter("return_home_service", "/task/return_home")
        self.declare_parameter("start_search_sweep_service", "/task/start_search_sweep")
        self.declare_parameter("stop_search_sweep_service", "/task/stop_search_sweep")
        self.declare_parameter("control_arm_enable_service", "/control/arm/enable")
        self.declare_parameter("control_arm_move_joints_service", "/control/arm/move_joints")
        self.declare_parameter("scene_reset_service", "/sim/world/reset")
        self.declare_parameter("scene_set_pose_service", "/sim/world/set_pose")
        self.declare_parameter("scene_pose_info_topic", "/sim/world/pose/info")
        self.declare_parameter("scene_target_model_name", "target_cylinder")
        self.declare_parameter("scene_reset_use_world_reset", False)
        self.declare_parameter("scene_reset_delay_sec", 0.75)
        self.declare_parameter("scene_reset_return_home_after_reset", True)
        self.declare_parameter(
            "scene_target_reset_pose",
            [0.50, 0.08, 0.44, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("debug_open_gazebo_gui", True)
        self.declare_parameter("debug_open_rviz", True)
        self.declare_parameter("debug_open_views_script", "")
        self.declare_parameter("return_home_joint_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter(
            "return_home_joint_angles_deg",
            [3.0, -58.0, 89.405064, 96.0, 0.0, -68.754936],
        )
        self.declare_parameter("return_home_duration_ms", 3500)
        self.declare_parameter("return_home_wait_for_completion", True)
        # Startup-home ownership belongs to the motion/search layer, not the web gateway.
        self.declare_parameter("startup_prepare_home", False)
        self.declare_parameter("startup_prepare_home_delay_sec", 6.0)
        self.declare_parameter("event_log_capacity", 240)
        self.declare_parameter("feedback_event_capacity", 48)
        self.declare_parameter("stream_jpeg_quality", 95)
        self.declare_parameter("dialog_model_endpoint", "http://127.0.0.1:8000/v1/chat/completions")
        self.declare_parameter("dialog_model_name", "Qwen/Qwen2.5-VL-3B-Instruct-AWQ")
        self.declare_parameter("dialog_api_key", "EMPTY")
        self.declare_parameter("dialog_request_timeout_sec", 25.0)
        self.declare_parameter("dialog_temperature", 0.1)
        self.declare_parameter("dialog_max_tokens", 384)
        self.declare_parameter("dialog_history_turns", 8)
        self.declare_parameter("dialog_auto_execute_confidence_threshold", 0.55)
        self.declare_parameter("dialog_use_visual_context", True)
        # Production default: let the dialog VLM see raw RGB and use detector output
        # only through structured context, not baked into the image.
        self.declare_parameter("dialog_visual_input_mode", "raw")
        self.declare_parameter("dialog_visual_max_side_px", 896)
        self.declare_parameter("dialog_visual_jpeg_quality", 85)
        self.declare_parameter("dialog_visual_focus_timeout_sec", 20.0)
        self.declare_parameter("dialog_default_reply_language", "zh")
        self.declare_parameter("default_task", "pick")
        self.declare_parameter("default_target_hint", "blue cylinder")
        self.declare_parameter("default_constraints", ["parallel_gripper"])

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.frontend_dist_dir = str(self.get_parameter("frontend_dist_dir").value or "").strip()
        self.live_state_period_sec = max(
            0.05, float(self.get_parameter("live_state_period_sec").value)
        )
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.semantic_result_topic = str(self.get_parameter("semantic_result_topic").value)
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.detection_debug_topic = str(self.get_parameter("detection_debug_topic").value)
        self.detection_debug_overlay_topic = str(
            self.get_parameter("detection_debug_overlay_topic").value
        )
        self.target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.pick_active_topic = str(self.get_parameter("pick_active_topic").value)
        self.pick_status_topic = str(self.get_parameter("pick_status_topic").value)
        self.candidate_grasp_proposal_array_topic = str(
            self.get_parameter("candidate_grasp_proposal_array_topic").value
        )
        self.grasp_backend_debug_topic = str(
            self.get_parameter("grasp_backend_debug_topic").value
        )
        self.grasp_overlay_topic = str(self.get_parameter("grasp_overlay_topic").value)
        self.tactile_topic = str(self.get_parameter("tactile_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.arm_state_topic = str(self.get_parameter("arm_state_topic").value)
        self.execute_task_action = str(self.get_parameter("execute_task_action").value)
        self.task_goal_topic = str(self.get_parameter("task_goal_topic").value)
        self.task_execution_status_topic = str(
            self.get_parameter("task_execution_status_topic").value
        )
        self.execute_pick_service = str(self.get_parameter("execute_pick_service").value)
        self.reset_pick_session_service = str(
            self.get_parameter("reset_pick_session_service").value
        )
        self.return_home_service = str(self.get_parameter("return_home_service").value)
        self.start_search_sweep_service = str(
            self.get_parameter("start_search_sweep_service").value
        )
        self.stop_search_sweep_service = str(
            self.get_parameter("stop_search_sweep_service").value
        )
        self.control_arm_enable_service = str(
            self.get_parameter("control_arm_enable_service").value
        )
        self.control_arm_move_joints_service = str(
            self.get_parameter("control_arm_move_joints_service").value
        )
        self.scene_reset_service = str(self.get_parameter("scene_reset_service").value)
        self.scene_set_pose_service = str(self.get_parameter("scene_set_pose_service").value)
        self.scene_pose_info_topic = str(self.get_parameter("scene_pose_info_topic").value)
        self.scene_target_model_name = (
            str(self.get_parameter("scene_target_model_name").value).strip() or "target_cylinder"
        )
        self.scene_reset_use_world_reset = bool(
            self.get_parameter("scene_reset_use_world_reset").value
        )
        self.scene_reset_delay_sec = max(
            0.0, float(self.get_parameter("scene_reset_delay_sec").value)
        )
        self.scene_reset_return_home_after_reset = bool(
            self.get_parameter("scene_reset_return_home_after_reset").value
        )
        scene_target_reset_pose = [
            float(item)
            for item in list(self.get_parameter("scene_target_reset_pose").value)
        ]
        if len(scene_target_reset_pose) != 7:
            scene_target_reset_pose = [0.50, 0.08, 0.44, 0.0, 0.0, 0.0, 1.0]
        self.scene_target_reset_pose = {
            "position": {
                "x": float(scene_target_reset_pose[0]),
                "y": float(scene_target_reset_pose[1]),
                "z": float(scene_target_reset_pose[2]),
            },
            "orientation": {
                "x": float(scene_target_reset_pose[3]),
                "y": float(scene_target_reset_pose[4]),
                "z": float(scene_target_reset_pose[5]),
                "w": float(scene_target_reset_pose[6]),
            },
        }
        self.debug_open_gazebo_gui = bool(self.get_parameter("debug_open_gazebo_gui").value)
        self.debug_open_rviz = bool(self.get_parameter("debug_open_rviz").value)
        self.debug_open_views_script = str(
            self.get_parameter("debug_open_views_script").value or ""
        ).strip()
        self.return_home_joint_ids = [
            int(item) for item in list(self.get_parameter("return_home_joint_ids").value)
        ]
        self.return_home_joint_angles_deg = [
            float(item)
            for item in list(self.get_parameter("return_home_joint_angles_deg").value)
        ]
        self.return_home_duration_ms = max(
            500, int(self.get_parameter("return_home_duration_ms").value)
        )
        self.return_home_wait_for_completion = bool(
            self.get_parameter("return_home_wait_for_completion").value
        )
        self.startup_prepare_home = bool(self.get_parameter("startup_prepare_home").value)
        self.startup_prepare_home_delay_sec = max(
            0.0, float(self.get_parameter("startup_prepare_home_delay_sec").value)
        )
        self.event_log_capacity = max(50, int(self.get_parameter("event_log_capacity").value))
        self.feedback_event_capacity = max(
            10, int(self.get_parameter("feedback_event_capacity").value)
        )
        self.stream_jpeg_quality = min(
            100,
            max(70, int(self.get_parameter("stream_jpeg_quality").value)),
        )
        (
            self.dialog_model_endpoint,
            self.dialog_model_name,
            self.dialog_api_key,
        ) = _resolve_dialog_runtime_config(
            endpoint=self.get_parameter("dialog_model_endpoint").value,
            model_name=self.get_parameter("dialog_model_name").value,
            api_key=self.get_parameter("dialog_api_key").value,
        )
        self.dialog_request_timeout_sec = max(
            1.0, float(self.get_parameter("dialog_request_timeout_sec").value)
        )
        self.dialog_temperature = max(
            0.0, float(self.get_parameter("dialog_temperature").value)
        )
        self.dialog_max_tokens = max(128, int(self.get_parameter("dialog_max_tokens").value))
        self.dialog_history_turns = max(
            2, int(self.get_parameter("dialog_history_turns").value)
        )
        self.dialog_auto_execute_confidence_threshold = max(
            0.0,
            min(1.0, float(self.get_parameter("dialog_auto_execute_confidence_threshold").value)),
        )
        self.dialog_use_visual_context = bool(
            self.get_parameter("dialog_use_visual_context").value
        )
        self.dialog_visual_input_mode = str(
            self.get_parameter("dialog_visual_input_mode").value or "raw"
        ).strip().lower()
        if self.dialog_visual_input_mode not in {"off", "raw", "annotated", "both"}:
            self.dialog_visual_input_mode = "raw"
        self.dialog_visual_max_side_px = max(
            256, int(self.get_parameter("dialog_visual_max_side_px").value)
        )
        self.dialog_visual_jpeg_quality = max(
            50,
            min(100, int(self.get_parameter("dialog_visual_jpeg_quality").value)),
        )
        self.dialog_visual_focus_timeout_sec = max(
            1.0, float(self.get_parameter("dialog_visual_focus_timeout_sec").value)
        )
        self.dialog_default_reply_language = _normalize_dialog_reply_language(
            self.get_parameter("dialog_default_reply_language").value
        )
        self.default_task = str(self.get_parameter("default_task").value).strip() or "pick"
        self.default_target_hint = (
            str(self.get_parameter("default_target_hint").value).strip() or "target object"
        )
        self.default_constraints = [
            str(item).strip()
            for item in list(self.get_parameter("default_constraints").value)
            if str(item).strip()
        ]
        if not self.default_constraints:
            self.default_constraints = ["parallel_gripper"]

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
        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.prompt_pub = self.create_publisher(String, self.prompt_topic, qos_reliable)
        self.semantic_override_pub = self.create_publisher(
            SemanticTask, self.semantic_task_topic, qos_latched
        )
        self.execute_task_client = ActionClient(self, ExecuteTask, self.execute_task_action)
        self.execute_pick_client = self.create_client(Trigger, self.execute_pick_service)
        self.reset_pick_client = self.create_client(Trigger, self.reset_pick_session_service)
        self.return_home_client = self.create_client(Trigger, self.return_home_service)
        self.start_search_sweep_client = self.create_client(
            Trigger, self.start_search_sweep_service
        )
        self.stop_search_sweep_client = self.create_client(
            Trigger, self.stop_search_sweep_service
        )
        self.control_arm_enable_client = self.create_client(
            SetBool, self.control_arm_enable_service
        )
        self.control_arm_move_joints_client = self.create_client(
            MoveArmJoints, self.control_arm_move_joints_service
        )
        self.scene_reset_client = self.create_client(
            ControlWorld, self.scene_reset_service
        )
        self.scene_set_pose_client = self.create_client(
            SetEntityPose, self.scene_set_pose_service
        )

        self.create_subscription(String, self.semantic_result_topic, self._on_semantic_result, qos_reliable)
        self.create_subscription(SemanticTask, self.semantic_task_topic, self._on_semantic_task, qos_reliable)
        self.create_subscription(TaskGoal, self.task_goal_topic, self._on_task_goal, qos_reliable)
        self.create_subscription(
            TaskExecutionStatus,
            self.task_execution_status_topic,
            self._on_task_execution_status,
            qos_reliable,
        )
        self.create_subscription(Image, self.color_topic, self._on_color_image, qos_sensor)
        self.create_subscription(
            DetectionResult, self.detection_result_topic, self._on_detection_result, qos_reliable
        )
        self.create_subscription(String, self.detection_debug_topic, self._on_detection_debug, qos_reliable)
        self.create_subscription(
            Image, self.detection_debug_overlay_topic, self._on_detection_overlay, qos_sensor
        )
        self.create_subscription(Bool, self.target_locked_topic, self._on_target_locked, qos_reliable)
        self.create_subscription(Bool, self.pick_active_topic, self._on_pick_active, qos_reliable)
        self.create_subscription(String, self.pick_status_topic, self._on_pick_status, qos_reliable)
        self.create_subscription(
            GraspProposalArray,
            self.candidate_grasp_proposal_array_topic,
            self._on_grasp_proposals,
            qos_reliable,
        )
        self.create_subscription(String, self.grasp_backend_debug_topic, self._on_backend_debug, qos_reliable)
        self.create_subscription(Image, self.grasp_overlay_topic, self._on_grasp_overlay, qos_sensor)
        self.create_subscription(TactileRaw, self.tactile_topic, self._on_tactile, qos_sensor)
        self.create_subscription(SystemHealth, self.health_topic, self._on_health, qos_reliable)
        self.create_subscription(ArmState, self.arm_state_topic, self._on_arm_state, qos_reliable)
        self.create_subscription(TFMessage, self.scene_pose_info_topic, self._on_scene_pose_info, qos_reliable)

        self._lock = threading.Lock()
        self._state_version = 0
        self._event_id = 0
        self._events: deque[dict[str, Any]] = deque(maxlen=self.event_log_capacity)
        self._feedback_events: deque[dict[str, Any]] = deque(maxlen=self.feedback_event_capacity)
        self._last_prompt_text = ""
        self._intervention = {"active": False, "source": "", "label": ""}
        self._pending_execute = {"active": False, "source": "", "message": "", "updated_at": 0.0}
        self._pending_execute_dispatching = False
        self._execute_task_goal_handle = None
        self._active_execute_task_goal_id = ""
        self._ignored_task_goal_ids: set[str] = set()
        self._semantic_result: dict[str, Any] = {"raw": "", "updated_at": 0.0}
        self._semantic_task: dict[str, Any] = {"updated_at": 0.0}
        self._task_goal: dict[str, Any] = {"updated_at": 0.0}
        self._task_execution_status: dict[str, Any] = {
            "phase": "idle",
            "message": "waiting for task goal",
            "updated_at": 0.0,
        }
        self._detection_result: dict[str, Any] = {"updated_at": 0.0}
        self._detection_debug: dict[str, Any] = {"updated_at": 0.0}
        self._grasp_backend_debug: dict[str, Any] = {"updated_at": 0.0}
        self._pick_status: dict[str, Any] = {"phase": "idle", "message": "idle", "updated_at": 0.0}
        self._grasp_proposals: dict[str, Any] = {
            "updated_at": 0.0,
            "proposals": [],
            "selected_index": 0,
            "count": 0,
        }
        self._target_locked = False
        self._pick_active = False
        self._tactile: dict[str, Any] = {"updated_at": 0.0}
        self._arm_state: dict[str, Any] = {"updated_at": 0.0}
        self._health_by_node: dict[str, dict[str, Any]] = {}
        self._scene_target_pose: dict[str, Any] = {
            "model_name": self.scene_target_model_name,
            "found": False,
            "reset_pose": copy.deepcopy(self.scene_target_reset_pose),
            "updated_at": 0.0,
        }
        self._dialog_endpoint_health = {"ready": False, "checked_at": 0.0}
        self._ws_root = Path(__file__).resolve().parents[3]
        self._runtime_log_dir = self._ws_root / ".runtime" / "programme-ui"
        self._runtime_log_dir.mkdir(parents=True, exist_ok=True)
        self._debug_gazebo_log = self._runtime_log_dir / "debug-gazebo-gui.log"
        self._debug_rviz_log = self._runtime_log_dir / "debug-rviz.log"
        self._dialog_message_id = 0
        self._dialog = {
            "session_id": uuid4().hex,
            "mode": "auto",
            "reply_language": self.dialog_default_reply_language,
            "status": "idle",
            "pending_auto_execute": False,
            "last_error": "",
            "visual_focus": None,
            "messages": [],
            "updated_at": 0.0,
        }
        self._dialog_http = requests.Session()

        self.rgb_stream = MjpegFrameBuffer()
        self.detection_overlay_stream = MjpegFrameBuffer()
        self.grasp_overlay_stream = MjpegFrameBuffer()
        self.rgb_encoder = AsyncImageEncoder(
            stream_name="rgb",
            logger=self.get_logger(),
            jpeg_quality=self.stream_jpeg_quality,
            buffer=self.rgb_stream,
        )
        self.detection_overlay_encoder = AsyncImageEncoder(
            stream_name="detection_overlay",
            logger=self.get_logger(),
            jpeg_quality=self.stream_jpeg_quality,
            buffer=self.detection_overlay_stream,
        )
        self.grasp_overlay_encoder = AsyncImageEncoder(
            stream_name="grasp_overlay",
            logger=self.get_logger(),
            jpeg_quality=self.stream_jpeg_quality,
            buffer=self.grasp_overlay_stream,
        )

        self._last_pick_status_phase = "idle"
        self._last_task_execution_phase = "idle"
        self._last_pick_progress_key = ""
        self._last_pick_active = False
        self._last_target_locked = False

        self._record_event(
            "system",
            "info",
            f"web gateway ready on http://{self.host}:{self.port}",
            feedback=False,
        )
        self.get_logger().info(
            f"tactile_web_gateway ready: host={self.host} port={self.port} "
            f"prompt={self.prompt_topic} semantic_task={self.semantic_task_topic}"
        )
        if self.startup_prepare_home:
            threading.Thread(target=self._startup_prepare_home_worker, daemon=True).start()

    def destroy_node(self) -> bool:
        self.rgb_encoder.close()
        self.detection_overlay_encoder.close()
        self.grasp_overlay_encoder.close()
        self._dialog_http.close()
        return super().destroy_node()

    def _record_event(
        self,
        category: str,
        level: str,
        message: str,
        *,
        data: Optional[dict[str, Any]] = None,
        feedback: bool = False,
    ) -> dict[str, Any]:
        event = {
            "id": self._event_id + 1,
            "created_at": _now_sec(),
            "category": category,
            "level": level,
            "message": message,
            "data": data or {},
        }
        with self._lock:
            self._event_id = int(event["id"])
            self._events.append(event)
            if feedback:
                self._feedback_events.append(event)
            self._state_version += 1
        return event

    def _set_intervention(self, active: bool, source: str = "", label: str = "") -> None:
        with self._lock:
            self._intervention = {
                "active": bool(active),
                "source": str(source or ""),
                "label": str(label or ""),
            }
            self._state_version += 1

    def _set_pending_execute(
        self,
        active: bool,
        *,
        source: str = "",
        message: str = "",
    ) -> None:
        with self._lock:
            self._pending_execute = {
                "active": bool(active),
                "source": str(source or "") if active else "",
                "message": str(message or "") if active else "",
                "updated_at": _now_sec() if active else 0.0,
            }
            if not active:
                self._pending_execute_dispatching = False
            self._state_version += 1

    def _on_semantic_result(self, msg: String) -> None:
        parsed = _safe_json_loads(str(msg.data or ""))
        parsed["raw"] = str(msg.data or "")
        parsed["updated_at"] = _now_sec()
        with self._lock:
            self._semantic_result = parsed
            self._state_version += 1

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        semantic = _semantic_task_to_dict(msg)
        prompt_text = str(semantic.get("prompt_text", "") or "").strip()
        if prompt_text:
            self._last_prompt_text = prompt_text
        with self._lock:
            self._semantic_task = semantic
            self._state_version += 1

    def _on_task_goal(self, msg: TaskGoal) -> None:
        task_goal = _task_goal_to_dict(msg)
        with self._lock:
            self._task_goal = task_goal
            self._state_version += 1

    def _on_task_execution_status(self, msg: TaskExecutionStatus) -> None:
        parsed = _task_execution_status_to_dict(msg)
        goal_id = str(parsed.get("goal_id", "") or "").strip()
        ignored = False
        if goal_id:
            with self._lock:
                ignored = goal_id in self._ignored_task_goal_ids
                if ignored and str(parsed.get("phase", "") or "") in {"completed", "error", "cancelled"}:
                    self._ignored_task_goal_ids.discard(goal_id)
        if ignored:
            return
        phase = str(parsed.get("phase", "idle") or "idle")
        phase_changed = phase != self._last_task_execution_phase
        if phase_changed:
            self._last_task_execution_phase = phase
            self._record_event(
                "task",
                "error" if phase in {"error", "cancelled"} else "info",
                str(parsed.get("message") or phase),
                data={
                    "phase": phase,
                    "current_skill": str(parsed.get("current_skill", "") or ""),
                    "goal_id": str(parsed.get("goal_id", "") or ""),
                },
                feedback=phase in {"completed", "error", "cancelled"},
            )
        with self._lock:
            self._task_execution_status = parsed
            if bool(parsed.get("active", False)):
                self._pending_execute = {
                    "active": False,
                    "source": "",
                    "message": "",
                    "updated_at": 0.0,
                }
                self._pending_execute_dispatching = False
            self._state_version += 1
        if phase_changed and phase == "completed":
            self._append_dialog_message("system", "Task execution completed.")
            self._set_dialog_status(status="ready", pending_auto_execute=False, last_error="")
        elif phase_changed and phase in {"error", "cancelled"}:
            self._append_dialog_message(
                "system",
                f"Task execution {phase}: {str(parsed.get('message') or phase)}",
            )
            self._set_dialog_status(
                status="error",
                pending_auto_execute=False,
                last_error=str(parsed.get("message") or phase),
            )

    def _on_color_image(self, msg: Image) -> None:
        self.rgb_encoder.submit(msg)

    def _on_detection_result(self, msg: DetectionResult) -> None:
        detection = _detection_result_to_dict(msg)
        with self._lock:
            self._detection_result = detection
            self._state_version += 1

    def _on_detection_debug(self, msg: String) -> None:
        parsed = _safe_json_loads(str(msg.data or ""))
        parsed["updated_at"] = _now_sec()
        with self._lock:
            self._detection_debug = parsed
            self._state_version += 1

    def _on_detection_overlay(self, msg: Image) -> None:
        self.detection_overlay_encoder.submit(msg)

    def _on_target_locked(self, msg: Bool) -> None:
        locked = bool(msg.data)
        if locked != self._last_target_locked:
            self._last_target_locked = locked
            self._record_event(
                "execution",
                "info",
                "target locked" if locked else "target lock released",
                feedback=False,
            )
        with self._lock:
            self._target_locked = locked
            self._state_version += 1
        self._maybe_run_pending_auto_execute(locked)
        self._maybe_run_pending_manual_execute(locked)

    def _on_pick_active(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active != self._last_pick_active:
            self._last_pick_active = active
            self._record_event(
                "execution",
                "info",
                "pick_active became true" if active else "pick_active returned false",
                feedback=False,
            )
        with self._lock:
            self._pick_active = active
            if active:
                self._pending_execute = {
                    "active": False,
                    "source": "",
                    "message": "",
                    "updated_at": 0.0,
                }
                self._pending_execute_dispatching = False
            self._state_version += 1

    def _on_pick_status(self, msg: String) -> None:
        parsed = _safe_json_loads(str(msg.data or ""))
        parsed["raw"] = str(msg.data or "")
        parsed["updated_at"] = _now_sec()
        phase = str(parsed.get("phase", "idle") or "idle")
        phase_changed = phase != self._last_pick_status_phase
        progress_total = int(parsed.get("progress_total", 0) or 0)
        progress_current = int(parsed.get("progress_current", 0) or 0)
        progress_percent = int(parsed.get("progress_percent", 0) or 0)
        progress_stage = str(parsed.get("progress_stage", "") or "").strip()
        progress_key = (
            f"{phase}|{progress_stage}|{progress_current}|{progress_total}|{progress_percent}"
            if phase == "planning" and progress_total > 0
            else ""
        )
        if phase_changed:
            self._last_pick_status_phase = phase
            self._record_event(
                "execution",
                "error" if phase == "error" else "info",
                str(parsed.get("message") or phase),
                data={"phase": phase},
                feedback=phase in {"completed", "error"},
            )
            self._last_pick_progress_key = ""
        elif progress_key and progress_key != self._last_pick_progress_key:
            self._record_event(
                "planning",
                "info",
                str(parsed.get("message") or "planning progress"),
                data={
                    "phase": phase,
                    "progress_stage": progress_stage,
                    "progress_current": progress_current,
                    "progress_total": progress_total,
                    "progress_percent": progress_percent,
                },
                feedback=False,
            )
        if progress_key:
            self._last_pick_progress_key = progress_key
        with self._lock:
            self._pick_status = parsed
            task_phase = str(self._task_execution_status.get("phase", "") or "").strip()
            self._state_version += 1
        high_level_owned = task_phase in {
            "accepted",
            "grounding",
            "awaiting_lock",
            "planning",
            "executing",
            "recovering",
            "completed",
            "error",
            "cancelled",
        }
        if phase_changed and phase == "completed" and not high_level_owned:
            self._append_dialog_message("system", "Execution completed.")
            self._set_dialog_status(status="ready", pending_auto_execute=False, last_error="")
        elif phase_changed and phase == "error" and not high_level_owned:
            self._append_dialog_message(
                "system",
                f"Execution error: {str(parsed.get('message') or 'unknown error')}",
            )
            self._set_dialog_status(
                status="error",
                pending_auto_execute=False,
                last_error=str(parsed.get("message") or "execution error"),
            )

    def _on_grasp_proposals(self, msg: GraspProposalArray) -> None:
        proposals = [_proposal_to_dict(item) for item in list(msg.proposals)[:8]]
        with self._lock:
            self._grasp_proposals = {
                "proposals": proposals,
                "selected_index": int(msg.selected_index),
                "count": len(msg.proposals),
                "updated_at": _now_sec(),
                "stamp_sec": _msg_stamp_sec(msg),
            }
            self._state_version += 1

    def _on_backend_debug(self, msg: String) -> None:
        parsed = _safe_json_loads(str(msg.data or ""))
        parsed["raw"] = str(msg.data or "")
        parsed["updated_at"] = _now_sec()
        with self._lock:
            self._grasp_backend_debug = parsed
            self._state_version += 1

    def _on_grasp_overlay(self, msg: Image) -> None:
        self.grasp_overlay_encoder.submit(msg)

    def _on_tactile(self, msg: TactileRaw) -> None:
        tactile = {
            "rows": int(msg.rows),
            "cols": int(msg.cols),
            "sequence_id": int(msg.sequence_id),
            "frame_id": str(msg.frame_id or ""),
            "forces": [float(item) for item in msg.forces],
            "forces_fx": [float(item) for item in msg.forces_fx],
            "forces_fy": [float(item) for item in msg.forces_fy],
            "forces_fz": [float(item) for item in msg.forces_fz],
            "updated_at": _now_sec(),
            "stamp_sec": _msg_stamp_sec(msg),
        }
        with self._lock:
            self._tactile = tactile
            self._state_version += 1

    def _on_health(self, msg: SystemHealth) -> None:
        item = {
            "node_name": str(msg.node_name or ""),
            "healthy": bool(msg.healthy),
            "level": int(msg.level),
            "message": str(msg.message or ""),
            "cpu_percent": float(msg.cpu_percent),
            "memory_percent": float(msg.memory_percent),
            "updated_at": _now_sec(),
            "stamp_sec": _msg_stamp_sec(msg),
        }
        with self._lock:
            self._health_by_node[item["node_name"] or f"node-{len(self._health_by_node) + 1}"] = item
            self._state_version += 1
        if not item["healthy"] or item["level"] >= 2:
            self._record_event(
                "health",
                "error" if item["level"] >= 2 else "warn",
                f"{item['node_name']}: {item['message']}",
                data={"level": item["level"]},
                feedback=item["level"] >= 2,
            )

    def _on_arm_state(self, msg: ArmState) -> None:
        arm_state = _arm_state_to_dict(msg)
        with self._lock:
            self._arm_state = arm_state
            self._state_version += 1

    def _on_scene_pose_info(self, msg: TFMessage) -> None:
        target_name = self.scene_target_model_name
        pose_info: Optional[dict[str, Any]] = None
        for transform in list(msg.transforms):
            child_frame_id = str(getattr(transform, "child_frame_id", "") or "")
            if target_name not in child_frame_id:
                continue
            header = getattr(transform, "header", None)
            translation = getattr(transform, "transform", None)
            pose_info = {
                "model_name": target_name,
                "child_frame_id": child_frame_id,
                "frame_id": str(getattr(header, "frame_id", "") or ""),
                "found": True,
                "position": {
                    "x": float(getattr(getattr(translation, "translation", None), "x", 0.0)),
                    "y": float(getattr(getattr(translation, "translation", None), "y", 0.0)),
                    "z": float(getattr(getattr(translation, "translation", None), "z", 0.0)),
                },
                "orientation": {
                    "x": float(getattr(getattr(translation, "rotation", None), "x", 0.0)),
                    "y": float(getattr(getattr(translation, "rotation", None), "y", 0.0)),
                    "z": float(getattr(getattr(translation, "rotation", None), "z", 0.0)),
                    "w": float(getattr(getattr(translation, "rotation", None), "w", 1.0)),
                },
                "reset_pose": copy.deepcopy(self.scene_target_reset_pose),
                "updated_at": _now_sec(),
            }
            break
        if pose_info is None:
            return
        with self._lock:
            self._scene_target_pose = pose_info
            self._state_version += 1

    def _append_dialog_message(
        self,
        role: str,
        text: str,
        *,
        meta: str = "",
        semantic_task: Optional[dict[str, Any]] = None,
        requested_action: str = "",
    ) -> dict[str, Any]:
        message = {
            "id": 0,
            "role": str(role or "system"),
            "text": str(text or "").strip(),
            "meta": str(meta or ""),
            "semantic_task": copy.deepcopy(semantic_task) if semantic_task else None,
            "requested_action": str(requested_action or ""),
            "created_at": _now_sec(),
        }
        with self._lock:
            self._dialog_message_id += 1
            message["id"] = int(self._dialog_message_id)
            self._dialog["messages"].append(message)
            self._dialog["messages"] = list(self._dialog["messages"][-40:])
            self._dialog["updated_at"] = float(message["created_at"])
            self._state_version += 1
        return copy.deepcopy(message)

    def _set_dialog_status(
        self,
        *,
        status: Optional[str] = None,
        mode: Optional[str] = None,
        reply_language: Optional[str] = None,
        pending_auto_execute: Optional[bool] = None,
        last_error: Optional[str] = None,
    ) -> None:
        with self._lock:
            if mode is not None:
                self._dialog["mode"] = "auto" if str(mode).lower() == "auto" else "review"
            if reply_language is not None:
                self._dialog["reply_language"] = _normalize_dialog_reply_language(reply_language)
            if status is not None:
                self._dialog["status"] = str(status or "idle")
            if pending_auto_execute is not None:
                self._dialog["pending_auto_execute"] = bool(pending_auto_execute)
            if last_error is not None:
                self._dialog["last_error"] = str(last_error or "")
            self._dialog["updated_at"] = _now_sec()
            self._state_version += 1

    def _set_dialog_visual_focus(self, focus: Optional[dict[str, Any]]) -> None:
        with self._lock:
            self._dialog["visual_focus"] = copy.deepcopy(focus) if focus else None
            self._dialog["updated_at"] = _now_sec()
            self._state_version += 1

    def _dialog_visual_focus_is_fresh(self, focus: Any) -> bool:
        if not isinstance(focus, dict) or not focus:
            return False
        try:
            updated_at = float(focus.get("updated_at", 0.0) or 0.0)
        except (TypeError, ValueError):
            return False
        return updated_at > 0.0 and (_now_sec() - updated_at) <= self.dialog_visual_focus_timeout_sec

    def _annotate_dialog_bgr_image(
        self,
        image_bgr: np.ndarray,
        *,
        snapshot: dict[str, Any],
    ) -> np.ndarray:
        annotated = image_bgr.copy()
        selected_candidate = snapshot.get("selected_candidate")
        selected_bbox = []
        if isinstance(selected_candidate, dict):
            selected_bbox = list(selected_candidate.get("bbox_xyxy", []) or [])

        for candidate in list(snapshot.get("top_candidates", []) or [])[:5]:
            if not isinstance(candidate, dict):
                continue
            bbox = _candidate_bbox_xyxy(candidate)
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            confidence = float(candidate.get("confidence", 0.0) or 0.0)
            label = _dialog_candidate_primary_label(candidate) or "candidate"
            status = str(candidate.get("status", "") or "")
            is_selected = list(candidate.get("bbox_xyxy", []) or []) == selected_bbox
            if is_selected:
                color = (80, 220, 180)
                thickness = 4
            elif status == "selectable":
                color = (90, 175, 255)
                thickness = 3
            else:
                color = (88, 136, 230)
                thickness = 2
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
            caption = f"#{int(candidate.get('index', 0))} {label} {confidence:.2f}"
            if status and status != "selectable":
                caption = f"{caption} {status}"
            (text_width, text_height), baseline = cv2.getTextSize(
                caption,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                2,
            )
            top = max(0, y1 - text_height - baseline - 10)
            bottom = top + text_height + baseline + 10
            right = min(annotated.shape[1], x1 + text_width + 16)
            cv2.rectangle(annotated, (x1, top), (right, bottom), color, -1)
            cv2.putText(
                annotated,
                caption,
                (x1 + 8, bottom - baseline - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (12, 18, 24),
                2,
                cv2.LINE_AA,
            )
        return annotated

    def _build_dialog_visual_content(
        self,
        *,
        snapshot: dict[str, Any],
        user_text: str,
    ) -> list[dict[str, Any]]:
        if not self.dialog_use_visual_context or self.dialog_visual_input_mode == "off":
            return []
        rgb_frame = self.rgb_stream.latest()
        if rgb_frame.jpeg is None or not rgb_frame.updated_at:
            return []
        try:
            base_bgr = _jpeg_to_bgr_image(rgb_frame.jpeg)
            visual_content: list[dict[str, Any]] = []
            if self.dialog_visual_input_mode in {"raw", "both"}:
                raw_image = _resize_bgr_image(base_bgr, self.dialog_visual_max_side_px)
                raw_jpeg = _bgr_image_to_jpeg(raw_image, self.dialog_visual_jpeg_quality)
                visual_content.extend(
                    [
                        {
                            "type": "text",
                            "text": "Visual context image: current RGB camera frame.",
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": _jpeg_bytes_to_data_url(raw_jpeg)},
                        },
                    ]
                )
            if self.dialog_visual_input_mode in {"annotated", "both"}:
                annotated_bgr = self._annotate_dialog_bgr_image(base_bgr, snapshot=snapshot)
                annotated_bgr = _resize_bgr_image(annotated_bgr, self.dialog_visual_max_side_px)
                annotated_jpeg = _bgr_image_to_jpeg(
                    annotated_bgr, self.dialog_visual_jpeg_quality
                )
                description = (
                    "Visual context image: current RGB frame annotated with detector boxes, labels, "
                    "and confidence values."
                )
                if _dialog_mentions_visual_query(user_text) or _dialog_mentions_deictic_reference(
                    user_text
                ):
                    description += " Use it to answer visual questions and resolve this/that references."
                visual_content.extend(
                    [
                        {"type": "text", "text": description},
                        {
                            "type": "image_url",
                            "image_url": {"url": _jpeg_bytes_to_data_url(annotated_jpeg)},
                        },
                    ]
                )
            return visual_content
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"failed to build dialog visual context: {exc}")
            return []

    def _match_dialog_candidate(
        self,
        candidates: list[dict[str, Any]],
        *,
        target_label: str,
        target_hint: str,
        user_text: str = "",
    ) -> Optional[dict[str, Any]]:
        normalized_label = _normalize_dialog_label_text(target_label).lower()
        normalized_hint = _normalize_dialog_label_text(target_hint).lower()
        direction_hints = _dialog_extract_direction_hints(
            f"{_normalize_dialog_label_text(target_hint)} {_normalize_dialog_label_text(user_text)}"
        )
        best_match: Optional[dict[str, Any]] = None
        best_score = float("-inf")
        for candidate in candidates:
            if not isinstance(candidate, dict):
                continue
            candidate_texts = _dialog_candidate_texts(candidate)
            candidate_tokens: set[str] = set()
            for text in candidate_texts:
                candidate_tokens.update(_dialog_text_tokens(text))

            score = float(candidate.get("confidence", 0.0) or 0.0) * 0.08

            if normalized_label:
                label_related = False
                for text in candidate_texts:
                    lowered = text.lower()
                    if lowered == normalized_label:
                        score += 1.0
                        label_related = True
                        break
                    if normalized_label in lowered or lowered in normalized_label:
                        score += 0.65
                        label_related = True
                if not label_related:
                    score -= 0.55

            if normalized_hint:
                hint_tokens = _dialog_text_tokens(normalized_hint)
                overlap = len(candidate_tokens.intersection(hint_tokens))
                score += min(0.45, 0.12 * float(overlap))
                if any(
                    normalized_hint in text.lower() or text.lower() in normalized_hint
                    for text in candidate_texts
                ):
                    score += 0.35

            candidate_side = _dialog_candidate_side(candidate)
            if direction_hints:
                if candidate_side and candidate_side in direction_hints:
                    score += 0.35
                elif candidate_side:
                    score -= 0.18

            if best_match is None or score > best_score:
                best_match = candidate
                best_score = score

        minimum_score = 0.2 if (normalized_label or normalized_hint or direction_hints) else 0.0
        if best_match is None or best_score < minimum_score:
            return None
        return copy.deepcopy(best_match)

    def _resolve_dialog_visual_focus(
        self,
        *,
        snapshot: dict[str, Any],
        normalized: dict[str, Any],
        user_text: str,
    ) -> Optional[dict[str, Any]]:
        dialog = snapshot.get("dialog", {}) or {}
        existing_focus = dialog.get("visual_focus")
        if not self._dialog_visual_focus_is_fresh(existing_focus):
            existing_focus = None
        top_candidates = list(snapshot.get("top_candidates", []) or [])
        selected_candidate = snapshot.get("selected_candidate")
        detection = snapshot.get("detection", {}) or {}
        requested_action = str(normalized.get("requested_action", "") or "")
        target_label = str(normalized.get("target_label", "") or "").strip()
        target_hint = str(normalized.get("target_hint", "") or "").strip()
        focus_seed_label = target_label
        focus_seed_hint = target_hint
        if _dialog_mentions_deictic_reference(user_text) and isinstance(existing_focus, dict):
            focus_seed_label = focus_seed_label or _normalize_dialog_label_text(
                existing_focus.get("canonical_label") or existing_focus.get("label")
            )
            focus_seed_hint = focus_seed_hint or _dialog_focus_target_hint(existing_focus)
        user_grounded_candidate = self._match_dialog_candidate(
            top_candidates,
            target_label="",
            target_hint=user_text,
            user_text=user_text,
        )
        matched_candidate = self._match_dialog_candidate(
            top_candidates,
            target_label=focus_seed_label,
            target_hint=focus_seed_hint or user_text,
            user_text=user_text,
        )
        if (
            user_grounded_candidate is not None
            and (
                matched_candidate is None
                or (
                    _dialog_mentions_execute_intent(user_text)
                    and not _dialog_labels_related(
                        focus_seed_label or target_label,
                        _dialog_candidate_primary_label(user_grounded_candidate),
                    )
                )
            )
        ):
            matched_candidate = user_grounded_candidate
            if not target_label or not _dialog_labels_related(
                target_label,
                _dialog_candidate_primary_label(user_grounded_candidate),
            ):
                target_label = _normalize_dialog_label_text(
                    user_grounded_candidate.get("canonical_label")
                    or user_grounded_candidate.get("label")
                )
            if not target_hint or not _dialog_labels_related(
                target_hint,
                _dialog_focus_target_hint(user_grounded_candidate),
            ):
                target_hint = _dialog_focus_target_hint(user_grounded_candidate)
        if matched_candidate is None and _dialog_mentions_deictic_reference(user_text):
            matched_candidate = self._match_dialog_candidate(
                top_candidates,
                target_label=str(
                    (existing_focus or {}).get("canonical_label", "")
                    or (existing_focus or {}).get("label", "")
                    or ""
                ),
                target_hint=_dialog_focus_target_hint(existing_focus) or user_text,
                user_text=user_text,
            )
        if (
            requested_action == "answer"
            and not target_label
            and not target_hint
            and isinstance(selected_candidate, dict)
        ):
            matched_candidate = copy.deepcopy(selected_candidate)
            target_label = str(selected_candidate.get("label", "") or "").strip()
            target_hint = target_label

        if requested_action == "cancel":
            return copy.deepcopy(existing_focus) if isinstance(existing_focus, dict) else None

        focus_canonical = _normalize_dialog_label_text(
            (matched_candidate or {}).get("canonical_label")
            or focus_seed_label
            or (existing_focus or {}).get("canonical_label")
            or (existing_focus or {}).get("label")
            or detection.get("target_label")
        )
        focus_display = _normalize_dialog_label_text(
            (matched_candidate or {}).get("display_label")
            or (matched_candidate or {}).get("label_zh")
            or focus_canonical
            or (existing_focus or {}).get("display_label")
            or (existing_focus or {}).get("label")
        )
        focus_label = (
            focus_canonical
            or str(detection.get("target_label", "") or "")
        ).strip()
        if not focus_label:
            if requested_action == "answer" and _dialog_mentions_visual_query(user_text):
                return copy.deepcopy(existing_focus) if isinstance(existing_focus, dict) else None
            return None

        bbox_xyxy = list((matched_candidate or {}).get("bbox_xyxy", []) or [])
        if not bbox_xyxy and isinstance(existing_focus, dict):
            bbox_xyxy = list(existing_focus.get("bbox_xyxy", []) or [])
        if not bbox_xyxy and isinstance(detection.get("bbox"), dict):
            bbox_xyxy = list(detection["bbox"].get("xyxy", []) or [])
        point_px = _coerce_int_list((matched_candidate or {}).get("point_px", []), expected_len=2)
        if not point_px and isinstance(existing_focus, dict):
            point_px = _coerce_int_list(existing_focus.get("point_px", []), expected_len=2)
        if not point_px and bbox_xyxy:
            point_px = _bbox_center_point_xyxy(bbox_xyxy)
        focus_track_id = 0
        try:
            focus_track_id = int((matched_candidate or {}).get("track_id", 0) or 0)
        except (TypeError, ValueError):
            focus_track_id = 0
        if focus_track_id <= 0 and isinstance(existing_focus, dict):
            try:
                focus_track_id = int(existing_focus.get("track_id", 0) or 0)
            except (TypeError, ValueError):
                focus_track_id = 0

        focus_confidence = float(normalized.get("confidence", 0.0) or 0.0)
        if matched_candidate is not None:
            focus_confidence = max(
                focus_confidence,
                float(matched_candidate.get("confidence", 0.0) or 0.0),
            )
        elif isinstance(existing_focus, dict):
            focus_confidence = max(
                focus_confidence,
                float(existing_focus.get("confidence", 0.0) or 0.0),
            )
        elif bool(detection.get("accepted")):
            focus_confidence = max(
                focus_confidence,
                float(detection.get("confidence", 0.0) or 0.0),
            )

        focus_side = (
            _dialog_candidate_side(matched_candidate or {})
            or _normalize_dialog_label_text((existing_focus or {}).get("side", "")).lower()
        )
        candidate_target_hint = _dialog_focus_target_hint(matched_candidate)
        target_hint_is_direction_only = _normalize_dialog_label_text(target_hint).lower() in {
            "left",
            "right",
            "center",
        }
        resolved_target_hint = (
            (candidate_target_hint if target_hint_is_direction_only and candidate_target_hint else target_hint)
            or candidate_target_hint
            or _dialog_focus_target_hint(existing_focus)
            or focus_label
        ).strip()
        return {
            "index": int((matched_candidate or {}).get("index", -1)),
            "track_id": focus_track_id,
            "label": focus_label,
            "canonical_label": focus_canonical or focus_label,
            "display_label": focus_display or focus_label,
            "label_zh": _normalize_dialog_label_text((matched_candidate or {}).get("label_zh", "")),
            "target_hint": resolved_target_hint,
            "confidence": focus_confidence,
            "bbox_xyxy": bbox_xyxy,
            "point_px": point_px,
            "side": focus_side,
            "requested_action": requested_action,
            "source": "dialog_visual_grounding",
            "updated_at": _now_sec(),
        }

    def _reset_dialog_session(self) -> None:
        with self._lock:
            current_mode = str(self._dialog.get("mode", "auto") or "auto")
            current_reply_language = _normalize_dialog_reply_language(
                self._dialog.get("reply_language", self.dialog_default_reply_language)
            )
            self._dialog = {
                "session_id": uuid4().hex,
                "mode": current_mode,
                "reply_language": current_reply_language,
                "status": "idle",
                "pending_auto_execute": False,
                "last_error": "",
                "visual_focus": None,
                "messages": [],
                "updated_at": _now_sec(),
            }
            self._state_version += 1

    def _dialog_context_snapshot(self) -> dict[str, Any]:
        with self._lock:
            dialog = copy.deepcopy(self._dialog)
            semantic = copy.deepcopy(self._semantic_task)
            task_status = copy.deepcopy(self._task_execution_status)
            detection = copy.deepcopy(self._detection_result)
            detection_debug = copy.deepcopy(self._detection_debug)
            execution = {
                "phase": self._derive_phase(
                    semantic=self._semantic_task,
                    task_status=self._task_execution_status,
                    detection=self._detection_result,
                    detection_debug=self._detection_debug,
                    target_locked=bool(self._target_locked),
                    pick_active=bool(self._pick_active),
                    pick_status=self._pick_status,
                ),
                "target_locked": bool(self._target_locked),
                "pick_active": bool(self._pick_active),
                "task_status": task_status,
                "pick_status": copy.deepcopy(self._pick_status),
            }
        image_width = int(detection.get("image_width", 0) or detection_debug.get("image_width", 0) or 0)
        image_height = int(
            detection.get("image_height", 0) or detection_debug.get("image_height", 0) or 0
        )
        top_candidates = []
        for item in _debug_candidates_from_payload(detection_debug)[:5]:
            if not isinstance(item, dict):
                continue
            candidate = copy.deepcopy(item)
            candidate["image_width"] = image_width
            candidate["image_height"] = image_height
            candidate["side"] = _dialog_candidate_side(candidate)
            top_candidates.append(candidate)
        selected_candidate = copy.deepcopy(detection_debug.get("selected_candidate"))
        if isinstance(selected_candidate, dict):
            selected_candidate["image_width"] = image_width
            selected_candidate["image_height"] = image_height
            selected_candidate["side"] = _dialog_candidate_side(selected_candidate)
        return {
            "dialog": dialog,
            "semantic": semantic,
            "detection": detection,
            "top_candidates": top_candidates,
            "selected_candidate": selected_candidate,
            "visual_focus": copy.deepcopy(dialog.get("visual_focus")),
            "execution": execution,
        }

    def _build_dialog_context_text(self, snapshot: dict[str, Any], mode: str) -> str:
        semantic = _semantic_payload_defaults(snapshot.get("semantic", {}))
        detection = snapshot.get("detection", {}) or {}
        execution = snapshot.get("execution", {}) or {}
        top_candidates = snapshot.get("top_candidates", []) or []
        visual_focus = snapshot.get("visual_focus")
        if not self._dialog_visual_focus_is_fresh(visual_focus):
            visual_focus = None
        context = {
            "mode": "auto" if str(mode).lower() == "auto" else "review",
            "current_task": semantic,
            "vision": {
                "accepted": bool(detection.get("accepted", False)),
                "target_label": str(detection.get("target_label", "") or ""),
                "confidence": float(detection.get("confidence", 0.0) or 0.0),
                "image_width": int(detection.get("image_width", 0) or 0),
                "image_height": int(detection.get("image_height", 0) or 0),
                "top_candidates": top_candidates,
            },
            "execution": {
                "phase": str(execution.get("phase", "idle") or "idle"),
                "target_locked": bool(execution.get("target_locked", False)),
                "pick_active": bool(execution.get("pick_active", False)),
                "pick_status_message": str(
                    (execution.get("pick_status", {}) or {}).get("message", "") or ""
                ),
            },
            "visual_focus": visual_focus,
        }
        return (
            "Robot UI context. Use this to resolve references and keep continuity between turns. "
            + _compact_json(context)
        )

    def _dialog_history_messages(self, dialog_messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
        history: list[dict[str, Any]] = []
        conversational_messages = [
            item
            for item in dialog_messages
            if str(item.get("role", "")) in {"user", "assistant"}
        ]
        for item in conversational_messages[-self.dialog_history_turns * 2 :]:
            role = str(item.get("role", "user") or "user")
            text = str(item.get("text", "") or "").strip()
            semantic_task = item.get("semantic_task")
            if role == "assistant" and isinstance(semantic_task, dict):
                text = (
                    f"{text}\nStructured task snapshot: "
                    f"{_compact_json(_semantic_payload_defaults(semantic_task))}"
                ).strip()
            if not text:
                continue
            history.append({"role": role, "content": [{"type": "text", "text": text}]})
        return history

    def _coerce_dialog_model_response(
        self,
        *,
        raw_text: str,
        snapshot: dict[str, Any],
        user_text: str,
        reply_language: str,
    ) -> dict[str, Any]:
        semantic_defaults = _semantic_payload_defaults(snapshot.get("semantic", {}))
        default_target_instance = _target_instance_from_payload(semantic_defaults)
        visual_focus = snapshot.get("visual_focus")
        if not self._dialog_visual_focus_is_fresh(visual_focus):
            visual_focus = None
        detection = snapshot.get("detection", {}) or {}
        requested_action = _infer_dialog_requested_action(user_text)
        assistant_text = _clean_dialog_assistant_text(raw_text)
        if not assistant_text:
            if requested_action == "answer":
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我理解了当前视觉场景。",
                    "I interpreted the current visual scene.",
                )
            elif requested_action == "execute":
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我把这句话理解为执行请求。",
                    "I interpreted this as an execution request.",
                )
            elif requested_action == "cancel":
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我把这句话理解为取消请求。",
                    "I interpreted this as a cancellation request.",
                )
            else:
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我理解了这条最新指令。",
                    "I interpreted the latest instruction.",
                )

        target_label = ""
        target_hint = ""
        confidence = 0.0
        if isinstance(visual_focus, dict) and _dialog_mentions_deictic_reference(user_text):
            target_label = str(visual_focus.get("label", "") or "").strip()
            target_hint = str(
                visual_focus.get("target_hint", "") or target_label
            ).strip()
            confidence = max(
                confidence,
                float(visual_focus.get("confidence", 0.0) or 0.0),
            )
        elif requested_action == "answer":
            confidence = max(
                confidence,
                float(detection.get("confidence", 0.0) or 0.0),
            )

        self.get_logger().warn(
            "dialog model returned non-JSON content; coercing raw assistant text into a fallback response"
        )
        return {
            "assistant_text": assistant_text,
            "requested_action": requested_action,
            "task": str(semantic_defaults.get("task", self.default_task) or self.default_task),
            "target_label": target_label,
            "target_hint": target_hint,
            "constraints": list(semantic_defaults.get("constraints", []) or self.default_constraints),
            "excluded_labels": list(semantic_defaults.get("excluded_labels", [])),
            "confidence": confidence,
            "need_human_confirm": requested_action not in {"answer", "cancel"},
            "reason": "fallback applied because the dialog model response was not strict JSON",
        }

    def _rewrite_dialog_assistant_text(
        self,
        *,
        assistant_text: str,
        reply_language: str,
    ) -> str:
        normalized_language = _normalize_dialog_reply_language(reply_language)
        if not assistant_text.strip() or _dialog_text_matches_reply_language(
            assistant_text, normalized_language
        ):
            return assistant_text

        target_language_name = (
            "Simplified Chinese" if normalized_language == "zh" else "English"
        )
        headers = {"Content-Type": "application/json"}
        if self.dialog_api_key:
            headers["Authorization"] = f"Bearer {self.dialog_api_key}"
        payload = {
            "model": self.dialog_model_name,
            "messages": [
                {
                    "role": "system",
                    "content": [
                        {
                            "type": "text",
                            "text": (
                                "Rewrite the assistant reply into the requested language only. "
                                "Do not change intent, task meaning, names, or confidence claims. "
                                "Output plain text only with no JSON, no markdown, and no extra explanation."
                            ),
                        }
                    ],
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": (
                                f"Target language: {target_language_name}.\n"
                                f"Reply: {assistant_text}"
                            ),
                        }
                    ],
                },
            ],
            "temperature": 0.0,
            "max_tokens": min(192, self.dialog_max_tokens),
        }
        response = self._dialog_http.post(
            self.dialog_model_endpoint,
            headers=headers,
            json=payload,
            timeout=self.dialog_request_timeout_sec,
        )
        response.raise_for_status()
        rewritten = _clean_dialog_assistant_text(_extract_model_message_text(response.json()))
        return rewritten or assistant_text

    def _query_dialog_model(
        self,
        *,
        user_text: str,
        mode: str,
        reply_language: str,
        snapshot: dict[str, Any],
        history_messages: list[dict[str, Any]],
    ) -> tuple[str, dict[str, Any]]:
        normalized_language = _normalize_dialog_reply_language(reply_language)
        language_instruction = (
            "assistant_text must be written only in Simplified Chinese."
            if normalized_language == "zh"
            else "assistant_text must be written only in English."
        )
        system_prompt = (
            "You are the multi-turn dialog controller for a robot manipulation UI with visual context. "
            "Return exactly one JSON object with no markdown. "
            "Required keys: assistant_text, requested_action, task, target_label, target_hint, "
            "constraints, excluded_labels, confidence, need_human_confirm, reason, detection_spec. "
            "requested_action must be one of update_task, execute, clarify, cancel, answer. "
            "constraints and excluded_labels must always be JSON arrays. Use [] when empty. "
            "detection_spec must always be a JSON object. "
            "When requested_action is update_task or execute, detection_spec must include: "
            "primary_label, prompt_classes, negative_labels, attributes. "
            "prompt_classes must be 1 to 5 short English detector phrases optimized for open-vocabulary detection. "
            "negative_labels and attributes must be JSON arrays. Use [] when empty. "
            "When requested_action is answer or cancel, detection_spec may be {}. "
            "confidence must always be a numeric value from 0.0 to 1.0, never a word. "
            "Keep continuity with the current task unless the user explicitly changes it. "
            "Use the provided image(s), top_candidates, and visual_focus to answer visual questions. "
            "If the user asks a visual or explanatory question without changing the robot task, "
            "set requested_action to answer. "
            "If requested_action is answer, keep the current task unchanged unless the user clearly asks "
            "to modify it. "
            "If the user says this, that, or it, resolve the reference using visual_focus and the "
            "current visual context. "
            "If the user only changes one field, keep the others consistent. "
            "If the request is ambiguous, ask one concise clarification question and set "
            "requested_action to clarify. "
            f"{language_instruction} "
            "The final response must begin with { and end with }. "
            "In review mode, do not promise automatic execution. "
            "In auto mode, you may say execution will start only when the command is clear and safe."
        )
        headers = {"Content-Type": "application/json"}
        if self.dialog_api_key:
            headers["Authorization"] = f"Bearer {self.dialog_api_key}"
        user_content: list[dict[str, Any]] = [
            {"type": "text", "text": str(user_text or "").strip()}
        ]
        user_content.extend(
            self._build_dialog_visual_content(snapshot=snapshot, user_text=user_text)
        )
        payload = {
            "model": self.dialog_model_name,
            "messages": [
                {"role": "system", "content": [{"type": "text", "text": system_prompt}]},
                {
                    "role": "system",
                    "content": [
                        {
                            "type": "text",
                            "text": self._build_dialog_context_text(snapshot, mode),
                        }
                    ],
                },
                *history_messages,
                {"role": "user", "content": user_content},
            ],
            "temperature": self.dialog_temperature,
            "max_tokens": self.dialog_max_tokens,
            "response_format": {"type": "json_object"},
        }
        response = self._dialog_http.post(
            self.dialog_model_endpoint,
            headers=headers,
            json=payload,
            timeout=self.dialog_request_timeout_sec,
        )
        if response.status_code in {400, 404, 422}:
            fallback_payload = dict(payload)
            fallback_payload.pop("response_format", None)
            response = self._dialog_http.post(
                self.dialog_model_endpoint,
                headers=headers,
                json=fallback_payload,
                timeout=self.dialog_request_timeout_sec,
            )
        response.raise_for_status()
        raw_text = _extract_model_message_text(response.json())
        try:
            parsed = _extract_first_json_object(raw_text)
        except ValueError:
            parsed = self._coerce_dialog_model_response(
                raw_text=raw_text,
                snapshot=snapshot,
                user_text=user_text,
                reply_language=reply_language,
            )
        return raw_text, parsed

    def _normalize_dialog_response(
        self,
        response: dict[str, Any],
        *,
        snapshot: dict[str, Any],
        user_text: str,
        reply_language: str,
    ) -> dict[str, Any]:
        semantic_defaults = _semantic_payload_defaults(snapshot.get("semantic", {}))
        default_target_instance = _target_instance_from_payload(semantic_defaults)
        visual_focus = snapshot.get("visual_focus")
        if not self._dialog_visual_focus_is_fresh(visual_focus):
            visual_focus = None
        constraints = response.get("constraints", semantic_defaults["constraints"])
        if not isinstance(constraints, list):
            constraints = semantic_defaults["constraints"]
        normalized_constraints = [
            str(item).strip() for item in constraints if str(item).strip()
        ]
        gripper = str(response.get("gripper", "") or "").strip()
        if gripper:
            normalized_constraints = [
                gripper,
                *[item for item in normalized_constraints if item != gripper],
            ]
        excluded_labels = _dialog_coerce_text_list(
            response.get("excluded_labels", semantic_defaults["excluded_labels"])
        ) or _dialog_coerce_text_list(semantic_defaults["excluded_labels"])
        requested_action = str(
            response.get("requested_action")
            or response.get("action")
            or response.get("intent")
            or "update_task"
        ).strip().lower()
        action_aliases = {
            "review": "update_task",
            "update": "update_task",
            "execute_task": "execute",
            "run": "execute",
            "ask": "clarify",
            "question": "clarify",
            "stop": "cancel",
            "answer": "answer",
            "observe": "answer",
            "explain": "answer",
        }
        requested_action = action_aliases.get(requested_action, requested_action)
        if requested_action not in {"update_task", "execute", "clarify", "cancel", "answer"}:
            requested_action = _infer_dialog_requested_action(user_text)
        if (
            requested_action in {"answer", "update_task"}
            and _dialog_mentions_execute_intent(user_text)
            and not _dialog_mentions_answer_query(user_text)
        ):
            requested_action = "execute"
        top_candidates = [
            item for item in list(snapshot.get("top_candidates", []) or []) if isinstance(item, dict)
        ]
        top_candidate = top_candidates[0] if top_candidates else None
        selected_candidate = (
            copy.deepcopy(snapshot.get("selected_candidate"))
            if isinstance(snapshot.get("selected_candidate"), dict)
            else None
        )
        if (
            requested_action == "clarify"
            and _dialog_mentions_answer_query(user_text)
            and isinstance(top_candidate, dict)
            and float(top_candidate.get("confidence", 0.0) or 0.0) >= 0.1
        ):
            requested_action = "answer"
        if (
            requested_action == "clarify"
            and _dialog_mentions_execute_intent(user_text)
            and _dialog_mentions_deictic_reference(user_text)
            and isinstance(visual_focus, dict)
        ):
            requested_action = "execute"

        deictic_reference = _dialog_mentions_deictic_reference(user_text)
        response_target_label = _dialog_coerce_text_value(
            response.get("target_label")
            or response.get("target")
            or ""
        ).strip()
        response_target_hint = _dialog_coerce_text_value(response.get("target_hint") or "").strip()
        focus_canonical_label = ""
        if isinstance(visual_focus, dict):
            focus_canonical_label = str(
                visual_focus.get("canonical_label", "") or visual_focus.get("label", "") or ""
            ).strip()
        if (
            deictic_reference
            and focus_canonical_label
            and (
                not response_target_label
                or not _dialog_labels_related(response_target_label, focus_canonical_label)
            )
        ):
            target_label = focus_canonical_label
        elif response_target_label:
            target_label = response_target_label
        elif deictic_reference and isinstance(visual_focus, dict):
            target_label = str(
                visual_focus.get("canonical_label", "") or visual_focus.get("label", "") or ""
            ).strip()
        elif requested_action in {"update_task", "execute"}:
            target_label = str(semantic_defaults["target_label"] or "").strip()
        else:
            target_label = ""

        focus_target_hint = (
            _dialog_focus_target_hint(visual_focus) if isinstance(visual_focus, dict) else ""
        )
        if (
            deictic_reference
            and focus_target_hint
            and (
                not response_target_hint
                or _dialog_target_hint_is_low_signal(response_target_hint)
                or not _dialog_labels_related(response_target_hint, focus_target_hint)
            )
        ):
            target_hint = focus_target_hint
        elif response_target_hint and not _dialog_target_hint_is_low_signal(response_target_hint):
            target_hint = response_target_hint
        elif target_label:
            target_hint = target_label
        elif deictic_reference and isinstance(visual_focus, dict):
            target_hint = _dialog_focus_target_hint(visual_focus) or str(
                visual_focus.get("target_hint", "") or target_label
            ).strip()
        elif requested_action in {"update_task", "execute"}:
            target_hint = str(semantic_defaults["target_hint"] or self.default_target_hint).strip()
        else:
            target_hint = ""

        execution_candidate = None
        if requested_action in {"update_task", "execute"}:
            execution_candidate = self._match_dialog_candidate(
                top_candidates,
                target_label=target_label,
                target_hint=target_hint or user_text,
                user_text=user_text,
            )
            if execution_candidate is None and isinstance(selected_candidate, dict):
                selected_label = _dialog_candidate_primary_label(selected_candidate)
                if not target_label or _dialog_labels_related(target_label, selected_label):
                    execution_candidate = copy.deepcopy(selected_candidate)
        target_instance = None
        if requested_action in {"update_task", "execute"}:
            target_instance = _target_instance_from_candidate(
                execution_candidate,
                source="dialog_execution_candidate",
            )
            if target_instance is None:
                target_instance = _target_instance_from_candidate(
                    visual_focus,
                    source="dialog_visual_focus",
                )
            if target_instance is None:
                preserve_existing_target = (
                    (not target_label and not target_hint)
                    or _dialog_labels_related(
                        target_label,
                        str(semantic_defaults.get("target_label", "") or ""),
                    )
                    or _dialog_labels_related(
                        target_hint,
                        str(semantic_defaults.get("target_hint", "") or ""),
                    )
                )
                if preserve_existing_target:
                    target_instance = default_target_instance

        confidence_raw = response.get("confidence", semantic_defaults["confidence"])
        try:
            confidence = float(confidence_raw)
        except (TypeError, ValueError):
            confidence = float(semantic_defaults["confidence"])
        if deictic_reference and isinstance(visual_focus, dict):
            confidence = max(confidence, float(visual_focus.get("confidence", 0.0) or 0.0))
        if requested_action == "execute":
            if isinstance(execution_candidate, dict):
                confidence = max(confidence, float(execution_candidate.get("confidence", 0.0) or 0.0))
            elif isinstance(top_candidate, dict) and target_label and _dialog_labels_related(
                target_label,
                _dialog_candidate_primary_label(top_candidate),
            ):
                confidence = max(confidence, float(top_candidate.get("confidence", 0.0) or 0.0))
        confidence = max(0.0, min(1.0, confidence))
        assistant_text = str(
            response.get("assistant_text")
            or response.get("reply")
            or response.get("message")
            or response.get("assistant")
            or ""
        ).strip()
        if target_hint and (
            target_hint == assistant_text or _dialog_target_hint_is_low_signal(target_hint)
        ):
            if target_label:
                target_hint = target_label
            elif deictic_reference and isinstance(visual_focus, dict):
                target_hint = _dialog_focus_target_hint(visual_focus) or str(
                    visual_focus.get("target_hint", "") or visual_focus.get("label", "")
                ).strip()
        if requested_action == "answer" and (
            _dialog_assistant_text_is_low_signal(assistant_text)
            or "confirm" in assistant_text.lower()
        ):
            if isinstance(visual_focus, dict):
                focus_label = (
                    _dialog_focus_target_hint(visual_focus)
                    or _dialog_candidate_primary_label(visual_focus)
                    or "object"
                )
                assistant_text = _dialog_reply_text(
                    reply_language,
                    f"我当前能在画面中看到你指的这个{focus_label}。",
                    f"I can currently see the referenced {focus_label} in the scene.",
                )
            elif isinstance(top_candidate, dict):
                top_label = (
                    _dialog_focus_target_hint(top_candidate)
                    or _dialog_candidate_primary_label(top_candidate)
                    or "object"
                )
                top_confidence = float(top_candidate.get("confidence", 0.0) or 0.0)
                assistant_text = _dialog_reply_text(
                    reply_language,
                    f"我当前能在画面中看到一个{top_label}，检测置信度约为 {top_confidence:.2f}。",
                    (
                        f"I can currently see a {top_label} in the scene with detector confidence "
                        f"{top_confidence:.2f}."
                    ),
                )
        elif requested_action == "execute" and (
            _dialog_assistant_text_is_low_signal(assistant_text)
            or "confirm" in assistant_text.lower()
            or not _dialog_assistant_text_mentions_execution(assistant_text)
        ):
            execute_target = target_hint or target_label or str(
                (visual_focus or {}).get("target_hint", "") or (visual_focus or {}).get("label", "")
            ).strip()
            if execute_target:
                assistant_text = _dialog_reply_text(
                    reply_language,
                    f"我已经把任务更新为抓取{execute_target}。",
                    f"I updated the task to pick the {execute_target}.",
                )
        default_need_human_confirm = requested_action == "clarify" or (
            requested_action not in {"answer", "cancel"} and confidence < 0.5
        )
        need_human_confirm = bool(
            response.get(
                "need_human_confirm",
                default_need_human_confirm,
            )
        )
        strong_execute_signal = (
            requested_action == "execute"
            and bool(target_label or target_hint)
            and confidence >= self.dialog_auto_execute_confidence_threshold
            and (
                isinstance(execution_candidate, dict)
                or isinstance(visual_focus, dict)
                or (
                    isinstance(selected_candidate, dict)
                    and (
                        not target_label
                        or _dialog_labels_related(
                            target_label,
                            _dialog_candidate_primary_label(selected_candidate),
                        )
                    )
                )
            )
        )
        if strong_execute_signal:
            need_human_confirm = False
        reason = str(response.get("reason", "") or "").strip()
        asks_confirmation = "confirm" in assistant_text.lower() or "确认" in assistant_text
        if requested_action == "execute" and not need_human_confirm and (
            _dialog_assistant_text_is_low_signal(assistant_text)
            or asks_confirmation
            or not _dialog_assistant_text_mentions_execution(assistant_text)
        ):
            execute_target = target_hint or target_label or str(
                (visual_focus or {}).get("target_hint", "") or (visual_focus or {}).get("label", "")
            ).strip()
            if execute_target:
                assistant_text = _dialog_reply_text(
                    reply_language,
                    f"我已经把任务更新为抓取{execute_target}，并会按当前锁定目标继续执行。",
                    f"I updated the task to pick the {execute_target} and will continue with the current locked target.",
                )
        if not assistant_text:
            if requested_action == "clarify":
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "在更新任务前，我还需要你补充一个关键信息。",
                    "I need one more detail before updating the task.",
                )
            elif requested_action == "execute":
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我已经更新任务并准备执行。",
                    "I updated the task and prepared execution.",
                )
            elif requested_action == "answer":
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我已经回答了这个视觉问题。",
                    "I answered the visual question.",
                )
            elif requested_action == "cancel":
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我保持当前任务不变。",
                    "I left the current task unchanged.",
                )
            else:
                assistant_text = _dialog_reply_text(
                    reply_language,
                    "我已经更新结构化任务。",
                    "I updated the structured task.",
                )
        if not _dialog_text_matches_reply_language(assistant_text, reply_language):
            try:
                assistant_text = self._rewrite_dialog_assistant_text(
                    assistant_text=assistant_text,
                    reply_language=reply_language,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"dialog assistant text rewrite failed: {exc}")
        sanitized_excluded_labels: list[str] = []
        excluded_seen: set[str] = set()
        for item in excluded_labels:
            label_text = _dialog_coerce_text_value(item).strip()
            if not label_text:
                continue
            if (
                (target_label and _dialog_labels_related(label_text, target_label))
                or (target_hint and _dialog_labels_related(label_text, target_hint))
            ):
                continue
            key = label_text.lower()
            if key in excluded_seen:
                continue
            excluded_seen.add(key)
            sanitized_excluded_labels.append(label_text)
        return {
            "assistant_text": assistant_text,
            "requested_action": requested_action,
            "task": str(response.get("task") or semantic_defaults["task"] or self.default_task),
            "target_label": target_label,
            "target_hint": target_hint,
            "target_instance": target_instance,
            "constraints": normalized_constraints or list(self.default_constraints),
            "excluded_labels": sanitized_excluded_labels,
            "detection_spec": response.get("detection_spec", {}),
            "confidence": confidence,
            "need_human_confirm": need_human_confirm,
            "reason": reason or f"dialog turn interpreted from: {str(user_text or '').strip()}",
        }

    def _publish_semantic_payload(
        self,
        payload: dict[str, Any],
        *,
        prompt_text: str,
        raw_json: str,
        intervention_active: bool,
        intervention_source: str = "",
        intervention_label: str = "",
    ) -> dict[str, Any]:
        constraints = [
            str(item).strip() for item in payload.get("constraints", []) if str(item).strip()
        ]
        gripper = str(payload.get("gripper", "") or "").strip()
        if gripper and gripper not in constraints:
            constraints = [gripper, *constraints]
        target_instance = _target_instance_from_payload(payload)
        sanitized_target_label = _dialog_coerce_text_value(payload.get("target_label", "")).strip()
        sanitized_target_hint = _dialog_coerce_text_value(
            payload.get("target_hint", sanitized_target_label) or sanitized_target_label
        ).strip()
        sanitized_excluded_labels: list[str] = []
        excluded_seen: set[str] = set()
        for item in _dialog_coerce_text_list(payload.get("excluded_labels", [])):
            if (
                (sanitized_target_label and _dialog_labels_related(item, sanitized_target_label))
                or (sanitized_target_hint and _dialog_labels_related(item, sanitized_target_hint))
            ):
                continue
            key = item.lower()
            if key in excluded_seen:
                continue
            excluded_seen.add(key)
            sanitized_excluded_labels.append(item)
        task_msg = SemanticTask()
        task_msg.header.stamp = self.get_clock().now().to_msg()
        task_msg.task = str(payload.get("task", "pick") or "pick")
        task_msg.target_label = sanitized_target_label
        task_msg.target_hint = sanitized_target_hint or task_msg.target_label
        task_msg.target_instance_track_id = int(
            (target_instance or {}).get("track_id", 0) or 0
        )
        task_msg.target_instance_bbox_xyxy = list(
            (target_instance or {}).get("bbox_xyxy", []) or []
        )
        task_msg.target_instance_point_px = list(
            (target_instance or {}).get("point_px", []) or []
        )
        task_msg.target_instance_source = str(
            (target_instance or {}).get("source", "") or ""
        )
        task_msg.constraints = constraints
        task_msg.excluded_labels = sanitized_excluded_labels
        task_msg.confidence = float(payload.get("confidence", 1.0) or 1.0)
        task_msg.need_human_confirm = bool(payload.get("need_human_confirm", False))
        task_msg.reason = str(payload.get("reason", "") or "")
        task_msg.prompt_text = str(prompt_text or "")
        task_msg.raw_json = str(raw_json or "")
        self.semantic_override_pub.publish(task_msg)

        semantic = _semantic_task_to_dict(task_msg)
        with self._lock:
            self._semantic_task = semantic
            self._semantic_result = {
                **copy.deepcopy(payload),
                "prompt_text": task_msg.prompt_text,
                "raw_json": task_msg.raw_json,
                "updated_at": _now_sec(),
            }
            self._state_version += 1
        self._last_prompt_text = task_msg.prompt_text
        if intervention_active:
            label = intervention_label or task_msg.target_label or task_msg.target_hint
            self._set_intervention(True, intervention_source, label)
        else:
            self._set_intervention(False)
        return semantic

    def _clear_semantic_context(
        self,
        *,
        reason: str,
        clear_dialog_focus: bool = True,
        clear_prompt_text: bool = True,
        publish_empty_task: bool = True,
    ) -> None:
        if publish_empty_task:
            clear_msg = SemanticTask()
            clear_msg.header.stamp = self.get_clock().now().to_msg()
            clear_msg.task = self.default_task
            clear_msg.reason = str(reason or "semantic context cleared")
            self.semantic_override_pub.publish(clear_msg)

        updated_at = _now_sec()
        with self._lock:
            self._semantic_task = {"updated_at": updated_at}
            self._semantic_result = {"raw": "", "updated_at": updated_at}
            self._task_goal = {"updated_at": 0.0}
            if clear_dialog_focus:
                self._dialog["visual_focus"] = None
                self._dialog["pending_auto_execute"] = False
            if clear_prompt_text:
                self._last_prompt_text = ""
            self._state_version += 1

    def set_dialog_mode(self, mode: str) -> dict[str, Any]:
        normalized = "auto" if str(mode).lower() == "auto" else "review"
        self._set_dialog_status(mode=normalized, status="idle", pending_auto_execute=False, last_error="")
        self._append_dialog_message("system", f"Dialog mode switched to {normalized.title()}.")
        return self.build_state()

    def set_dialog_reply_language(self, reply_language: str) -> dict[str, Any]:
        normalized = _normalize_dialog_reply_language(reply_language)
        self._set_dialog_status(
            reply_language=normalized,
            status="idle",
            pending_auto_execute=False,
            last_error="",
        )
        return self.build_state()

    def reset_dialog_session(self) -> dict[str, Any]:
        self._reset_dialog_session()
        self._set_pending_execute(False)
        return self.build_state()

    def dialog_send_message(self, payload: dict[str, Any]) -> dict[str, Any]:
        user_text = str(payload.get("message") or payload.get("prompt") or "").strip()
        if not user_text:
            raise ValueError("message is required")
        requested_mode = str(payload.get("mode", "") or "").strip().lower()
        requested_reply_language = _normalize_dialog_reply_language(
            payload.get("reply_language", "")
        )
        if requested_mode not in {"review", "auto"}:
            with self._lock:
                requested_mode = str(self._dialog.get("mode", "auto") or "auto")
        with self._lock:
            current_reply_language = _normalize_dialog_reply_language(
                self._dialog.get("reply_language", self.dialog_default_reply_language)
            )
        if not str(payload.get("reply_language", "") or "").strip():
            requested_reply_language = current_reply_language

        snapshot = self._dialog_context_snapshot()
        history_messages = self._dialog_history_messages(snapshot.get("dialog", {}).get("messages", []))
        with self._lock:
            self._dialog["mode"] = requested_mode
            self._dialog["reply_language"] = requested_reply_language
            self._dialog["status"] = "thinking"
            self._dialog["pending_auto_execute"] = False
            self._dialog["last_error"] = ""
            self._dialog["updated_at"] = _now_sec()
            self._dialog_message_id += 1
            self._dialog["messages"].append(
                {
                    "id": int(self._dialog_message_id),
                    "role": "user",
                    "text": user_text,
                    "meta": "",
                    "semantic_task": None,
                    "requested_action": "",
                    "created_at": _now_sec(),
                }
            )
            self._dialog["messages"] = list(self._dialog["messages"][-40:])
            self._state_version += 1

        try:
            raw_text, parsed = self._query_dialog_model(
                user_text=user_text,
                mode=requested_mode,
                reply_language=requested_reply_language,
                snapshot=snapshot,
                history_messages=history_messages,
            )
            normalized = self._normalize_dialog_response(
                parsed,
                snapshot=snapshot,
                user_text=user_text,
                reply_language=requested_reply_language,
            )
        except Exception as exc:  # noqa: BLE001
            error_message = f"dialog inference failed: {exc}"
            self._set_dialog_status(status="error", pending_auto_execute=False, last_error=error_message)
            self._append_dialog_message("system", error_message)
            raise

        requested_action = str(normalized["requested_action"])
        explicit_execute_intent = _dialog_mentions_execute_intent(user_text) and not _dialog_mentions_answer_query(user_text)
        visual_focus = self._resolve_dialog_visual_focus(
            snapshot=snapshot,
            normalized=normalized,
            user_text=user_text,
        )
        if (
            requested_action == "clarify"
            and _dialog_mentions_execute_intent(user_text)
            and not _dialog_mentions_answer_query(user_text)
            and isinstance(visual_focus, dict)
        ):
            requested_action = "execute"
            normalized["requested_action"] = requested_action
            normalized["target_label"] = str(
                visual_focus.get("canonical_label", "")
                or visual_focus.get("label", "")
                or normalized.get("target_label", "")
            ).strip()
            normalized["target_hint"] = _dialog_focus_target_hint(visual_focus) or str(
                normalized.get("target_hint", "") or normalized.get("target_label", "")
            ).strip()
            normalized["need_human_confirm"] = False
            normalized["assistant_text"] = _dialog_reply_text(
                requested_reply_language,
                f"我已经把任务更新为抓取{normalized['target_hint']}。",
                f"I updated the task to pick the {normalized['target_hint']}.",
            )
        if (
            requested_action == "answer"
            and _dialog_mentions_execute_intent(user_text)
            and not _dialog_mentions_answer_query(user_text)
            and isinstance(visual_focus, dict)
        ):
            requested_action = "execute"
            normalized["requested_action"] = requested_action
            normalized["target_label"] = str(
                visual_focus.get("canonical_label", "")
                or visual_focus.get("label", "")
                or normalized.get("target_label", "")
            ).strip()
            normalized["target_hint"] = _dialog_focus_target_hint(visual_focus) or str(
                normalized.get("target_hint", "") or normalized.get("target_label", "")
            ).strip()
            normalized["need_human_confirm"] = False
            if not _dialog_assistant_text_mentions_execution(
                str(normalized.get("assistant_text", "") or "")
            ):
                normalized["assistant_text"] = _dialog_reply_text(
                    requested_reply_language,
                    f"我已经把任务更新为抓取{normalized['target_hint']}。",
                    f"I updated the task to pick the {normalized['target_hint']}.",
                )
        if requested_mode == "auto" and explicit_execute_intent and requested_action != "cancel":
            requested_action = "execute"
            normalized["requested_action"] = requested_action
            normalized["need_human_confirm"] = False
        elif requested_mode == "auto" and requested_action == "update_task":
            requested_action = "execute"
            normalized["requested_action"] = requested_action
        if requested_action == "execute" and (
            _dialog_assistant_text_is_low_signal(str(normalized.get("assistant_text", "") or ""))
            or "confirm" in str(normalized.get("assistant_text", "") or "").lower()
            or "确认" in str(normalized.get("assistant_text", "") or "")
            or not _dialog_assistant_text_mentions_execution(
                str(normalized.get("assistant_text", "") or "")
            )
        ):
            execute_target = (
                str(normalized.get("target_hint", "") or "").strip()
                or str(normalized.get("target_label", "") or "").strip()
                or str((visual_focus or {}).get("target_hint", "") or "").strip()
                or str((visual_focus or {}).get("label", "") or "").strip()
            )
            if execute_target:
                normalized["assistant_text"] = _dialog_reply_text(
                    requested_reply_language,
                    f"我已将任务更新为抓取{execute_target}，并开始按当前视觉目标继续执行。",
                    f"I updated the task to pick the {execute_target} and started execution with the current visual target.",
                )
        if visual_focus is not None:
            self._set_dialog_visual_focus(visual_focus)
        self.get_logger().info(
            "dialog normalized: "
            f"action={requested_action} "
            f"target={str(normalized.get('target_hint') or normalized.get('target_label') or '').strip()} "
            f"confidence={float(normalized.get('confidence', 0.0) or 0.0):.3f} "
            f"need_human_confirm={bool(normalized.get('need_human_confirm', False))} "
            f"visual_focus={'yes' if isinstance(visual_focus, dict) else 'no'}"
        )
        semantic_payload: Optional[dict[str, Any]] = None
        if requested_action not in {"clarify", "cancel", "answer"}:
            semantic_payload = {
                key: normalized[key]
                for key in (
                    "task",
                    "target_label",
                    "target_hint",
                    "target_instance",
                    "constraints",
                    "excluded_labels",
                    "detection_spec",
                    "confidence",
                    "need_human_confirm",
                    "reason",
                )
            }
            semantic_payload = self._publish_semantic_payload(
                semantic_payload,
                prompt_text=user_text,
                raw_json=raw_text,
                intervention_active=False,
            )
            self._record_event(
                "dialog",
                "info",
                f"dialog updated task: {semantic_payload.get('target_label') or semantic_payload.get('target_hint') or '<unset>'}",
                data={"mode": requested_mode, "requested_action": requested_action},
                feedback=False,
            )
        elif requested_action == "answer":
            focus_label = (
                str((visual_focus or {}).get("label", "") or "").strip()
                or str((visual_focus or {}).get("target_hint", "") or "").strip()
                or "<visual answer>"
            )
            self._record_event(
                "dialog",
                "info",
                f"dialog answered visual question: {focus_label}",
                data={"mode": requested_mode, "requested_action": requested_action},
                feedback=False,
            )

        self._append_dialog_message(
            "assistant",
            str(normalized["assistant_text"]),
            meta=_dialog_message_summary(semantic_payload, requested_action),
            semantic_task=semantic_payload,
            requested_action=requested_action,
        )

        if requested_action == "cancel":
            self._set_dialog_status(status="ready", pending_auto_execute=False, last_error="")
            self._append_dialog_message("system", "Current structured task was left unchanged.")
            return self.build_state()

        if requested_mode == "auto" and requested_action == "execute":
            auto_note = self._configure_auto_execute(
                semantic_payload=semantic_payload,
                user_text=user_text,
                confidence=float(normalized["confidence"]),
                need_human_confirm=bool(normalized["need_human_confirm"]),
            )
            self._append_dialog_message("system", auto_note)
            return self.build_state()

        self._set_dialog_status(status="ready", pending_auto_execute=False, last_error="")
        return self.build_state()

    def _configure_auto_execute(
        self,
        *,
        semantic_payload: Optional[dict[str, Any]],
        user_text: str,
        confidence: float,
        need_human_confirm: bool,
    ) -> str:
        if semantic_payload is None:
            self._set_dialog_status(status="error", pending_auto_execute=False, last_error="no semantic task to execute")
            return "Auto mode could not start because no structured task was produced."
        target_name = str(
            semantic_payload.get("target_label")
            or semantic_payload.get("target_hint")
            or ""
        ).strip()
        if not target_name:
            self._set_dialog_status(status="ready", pending_auto_execute=False, last_error="")
            return "Auto mode held execution because the target is still unspecified."
        explicit_execute_intent = _dialog_mentions_execute_intent(user_text) and not _dialog_mentions_answer_query(user_text)
        if explicit_execute_intent:
            need_human_confirm = False
        if (
            need_human_confirm
            and confidence >= self.dialog_auto_execute_confidence_threshold
            and _dialog_mentions_execute_intent(user_text)
        ):
            need_human_confirm = False
        if need_human_confirm or (
            confidence < self.dialog_auto_execute_confidence_threshold and not explicit_execute_intent
        ):
            self._set_dialog_status(status="ready", pending_auto_execute=False, last_error="")
            self.get_logger().info(
                "dialog auto execute held: "
                f"target={target_name} "
                f"confidence={float(confidence):.3f} "
                f"threshold={float(self.dialog_auto_execute_confidence_threshold):.3f} "
                f"need_human_confirm={bool(need_human_confirm)}"
            )
            return (
                "Auto mode updated the task but held execution "
                f"(confidence={float(confidence):.2f}, "
                f"threshold={float(self.dialog_auto_execute_confidence_threshold):.2f}, "
                f"need_human_confirm={'true' if need_human_confirm else 'false'})."
            )
        self.get_logger().info(
            "dialog auto execute allowed: "
            f"target={target_name} "
            f"confidence={float(confidence):.3f} "
            f"threshold={float(self.dialog_auto_execute_confidence_threshold):.3f}"
        )
        ok, message, _ = self.execute_pick(
            start_search_sweep=True,
            allow_queue=False,
            request_source="auto_execute",
        )
        if ok:
            self._set_dialog_status(status="auto_executing", pending_auto_execute=False, last_error="")
            return "Auto mode accepted the task and handed execution to the task executive."
        self._set_dialog_status(status="error", pending_auto_execute=False, last_error=message)
        return f"Auto mode could not start execution: {message}"

    def _run_auto_execute(self, reason: str) -> None:
        ok, message, _ = self.execute_pick(
            start_search_sweep=True,
            allow_queue=False,
            request_source="auto_execute",
        )
        if ok:
            self._append_dialog_message("system", f"Auto execute requested: {reason}.")
            self._set_dialog_status(status="auto_executing", pending_auto_execute=False, last_error="")
            return
        self._append_dialog_message("system", f"Auto execute failed: {message}")
        self._set_dialog_status(status="error", pending_auto_execute=False, last_error=message)

    def _maybe_run_pending_auto_execute(self, locked: bool) -> None:
        if not locked:
            return
        with self._lock:
            should_execute = bool(self._dialog.get("pending_auto_execute", False)) and not bool(
                self._pick_active
            )
            if should_execute:
                self._dialog["pending_auto_execute"] = False
                self._dialog["status"] = "auto_executing"
                self._dialog["updated_at"] = _now_sec()
                self._state_version += 1
        if should_execute:
            threading.Thread(
                target=self._run_auto_execute,
                args=("target lock acquired",),
                daemon=True,
            ).start()

    def _run_pending_manual_execute(self, reason: str) -> None:
        ok, message, _ = self.execute_pick(
            start_search_sweep=False,
            allow_queue=False,
            request_source="manual_queue",
        )
        if ok:
            self._set_pending_execute(False)
            self._record_event(
                "execution",
                "info",
                f"queued execute started: {reason}",
                feedback=False,
            )
            return
        lowered = str(message or "").strip().lower()
        with self._lock:
            self._pending_execute_dispatching = False
            self._state_version += 1
        if "target is not locked" in lowered:
            return
        self._set_pending_execute(False)

    def _maybe_run_pending_manual_execute(self, locked: bool) -> None:
        if not locked:
            return
        with self._lock:
            should_execute = bool(self._pending_execute.get("active", False)) and str(
                self._pending_execute.get("source", "") or ""
            ) == "manual" and not bool(self._pick_active) and not bool(
                self._pending_execute_dispatching
            )
            if should_execute:
                self._pending_execute_dispatching = True
                self._state_version += 1
        if should_execute:
            threading.Thread(
                target=self._run_pending_manual_execute,
                args=("target lock acquired",),
                daemon=True,
            ).start()

    def _call_service_request(
        self, client: Any, request: Any, timeout_sec: float = 4.0
    ) -> tuple[bool, str, Any]:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, "service is unavailable", None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_sec):
            return False, "service call timed out", None
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc), None
        return True, "", result

    def _call_trigger(self, client: Any, timeout_sec: float = 4.0) -> tuple[bool, str]:
        ok, error, result = self._call_service_request(client, Trigger.Request(), timeout_sec=timeout_sec)
        if not ok or result is None:
            return False, error or "service call failed"
        return bool(result.success), str(result.message or "")

    def _current_task_goal_payload(self) -> dict[str, Any]:
        with self._lock:
            semantic = copy.deepcopy(self._semantic_task)
            semantic_result = copy.deepcopy(self._semantic_result)
        merged = {**semantic, **semantic_result}
        transport_constraints = [
            str(item).strip()
            for item in list(
                semantic_result.get("transport_constraints", semantic.get("constraints", [])) or []
            )
            if str(item).strip()
        ]
        execution_constraints = [
            str(item).strip()
            for item in list(semantic_result.get("execution_constraints", []) or [])
            if str(item).strip()
        ]
        return {
            "goal_type": str(semantic_result.get("goal_type", "") or ""),
            "task": str(merged.get("task", self.default_task) or self.default_task),
            "target_label": str(merged.get("target_label", "") or ""),
            "target_hint": str(merged.get("target_hint", "") or ""),
            "target_instance": _target_instance_from_payload(merged),
            "grasp_region": str(semantic_result.get("grasp_region", "") or ""),
            "placement_label": str(semantic_result.get("placement_label", "") or ""),
            "transport_constraints": transport_constraints,
            "execution_constraints": execution_constraints,
            "excluded_labels": [
                str(item).strip()
                for item in list(merged.get("excluded_labels", []) or [])
                if str(item).strip()
            ],
            "confidence": float(merged.get("confidence", 0.0) or 0.0),
            "need_human_confirm": bool(merged.get("need_human_confirm", False)),
            "allow_instance_grounding": bool(
                semantic_result.get("allow_instance_grounding", True)
            ),
            "require_target_lock": bool(semantic_result.get("require_target_lock", False)),
            "allow_replan": bool(semantic_result.get("allow_replan", True)),
            "allow_rescan": bool(semantic_result.get("allow_rescan", True)),
            "max_retries": max(0, int(semantic_result.get("max_retries", 1) or 1)),
            "planning_timeout_sec": float(semantic_result.get("planning_timeout_sec", 0.0) or 0.0),
            "execution_timeout_sec": float(
                semantic_result.get("execution_timeout_sec", 0.0) or 0.0
            ),
            "success_criteria": str(semantic_result.get("success_criteria", "") or ""),
            "failure_policy": str(semantic_result.get("failure_policy", "") or ""),
            "reason": str(merged.get("reason", "") or ""),
            "prompt_text": str(
                merged.get("prompt_text", self._last_prompt_text) or self._last_prompt_text
            ),
            "raw_json": str(merged.get("raw_json", "") or ""),
        }

    def _build_task_goal_msg(
        self,
        *,
        start_search_sweep: bool,
    ) -> Optional[TaskGoal]:
        payload = self._current_task_goal_payload()
        target_name = str(payload.get("target_label") or payload.get("target_hint") or "").strip()
        if not target_name:
            return None
        target_instance = _target_instance_from_payload(payload)
        goal_msg = TaskGoal()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.schema_version = str(payload.get("schema_version", "") or "").strip() or "task_goal.v2alpha1"
        goal_msg.goal_id = uuid4().hex[:12]
        goal_msg.parent_goal_id = str(payload.get("parent_goal_id", "") or "").strip()
        goal_msg.step_id = str(payload.get("step_id", "") or "").strip() or "pick"
        goal_msg.step_index = int(payload.get("step_index", 0) or 0)
        goal_msg.goal_type = str(payload.get("goal_type", "") or "").strip() or "pick"
        goal_msg.task = str(payload.get("task", self.default_task) or self.default_task).strip()
        goal_msg.target_label = str(payload.get("target_label", "") or "").strip()
        goal_msg.target_hint = str(payload.get("target_hint", goal_msg.target_label) or goal_msg.target_label).strip()
        goal_msg.target_attributes = [
            str(item).strip()
            for item in list(payload.get("target_attributes", []) or [])
            if str(item).strip()
        ]
        goal_msg.target_relations = [
            str(item).strip()
            for item in list(payload.get("target_relations", []) or [])
            if str(item).strip()
        ]
        goal_msg.target_instance_track_id = int((target_instance or {}).get("track_id", 0) or 0)
        goal_msg.target_instance_bbox_xyxy = list((target_instance or {}).get("bbox_xyxy", []) or [])
        goal_msg.target_instance_point_px = list((target_instance or {}).get("point_px", []) or [])
        goal_msg.target_instance_source = str((target_instance or {}).get("source", "") or "")
        goal_msg.target_part_query = str(
            payload.get("target_part_query", payload.get("grasp_region", "")) or ""
        ).strip()
        goal_msg.preferred_part_tags = [
            str(item).strip()
            for item in list(payload.get("preferred_part_tags", []) or [])
            if str(item).strip()
        ]
        if (not list(goal_msg.preferred_part_tags)) and str(goal_msg.target_part_query or "").strip():
            goal_msg.preferred_part_tags = [str(goal_msg.target_part_query).strip()]
        goal_msg.forbidden_part_tags = [
            str(item).strip()
            for item in list(payload.get("forbidden_part_tags", []) or [])
            if str(item).strip()
        ]
        goal_msg.grasp_region = str(payload.get("grasp_region", "") or "").strip()
        goal_msg.preferred_grasp_family = str(
            payload.get("preferred_grasp_family", "") or ""
        ).strip()
        goal_msg.placement_label = str(payload.get("placement_label", "") or "").strip()
        goal_msg.preferred_approach_dirs = [
            str(item).strip()
            for item in list(payload.get("preferred_approach_dirs", []) or [])
            if str(item).strip()
        ]
        goal_msg.forbidden_approach_dirs = [
            str(item).strip()
            for item in list(payload.get("forbidden_approach_dirs", []) or [])
            if str(item).strip()
        ]
        goal_msg.transport_constraints = [
            str(item).strip()
            for item in list(payload.get("transport_constraints", []) or [])
            if str(item).strip()
        ]
        goal_msg.execution_constraints = [
            str(item).strip()
            for item in list(payload.get("execution_constraints", []) or [])
            if str(item).strip()
        ]
        goal_msg.excluded_labels = [
            str(item).strip()
            for item in list(payload.get("excluded_labels", []) or [])
            if str(item).strip()
        ]
        goal_msg.reserved_next_skills = [
            str(item).strip()
            for item in list(payload.get("reserved_next_skills", []) or [])
            if str(item).strip()
        ]
        goal_msg.confidence = float(payload.get("confidence", 0.0) or 0.0)
        goal_msg.min_affordance_score = float(payload.get("min_affordance_score", 0.0) or 0.0)
        goal_msg.min_grasp_quality = float(payload.get("min_grasp_quality", 0.0) or 0.0)
        goal_msg.max_forbidden_overlap = float(payload.get("max_forbidden_overlap", 0.0) or 0.0)
        goal_msg.max_target_staleness_sec = float(
            payload.get("max_target_staleness_sec", 0.0) or 0.0
        )
        goal_msg.need_human_confirm = bool(payload.get("need_human_confirm", False))
        goal_msg.start_search_sweep = bool(start_search_sweep)
        goal_msg.allow_instance_grounding = bool(payload.get("allow_instance_grounding", True))
        goal_msg.require_target_lock = bool(payload.get("require_target_lock", False))
        goal_msg.allow_replan = bool(payload.get("allow_replan", True))
        goal_msg.allow_rescan = bool(payload.get("allow_rescan", True))
        goal_msg.strict_part_match = bool(payload.get("strict_part_match", False))
        goal_msg.strict_approach_match = bool(payload.get("strict_approach_match", False))
        goal_msg.max_retries = max(0, int(payload.get("max_retries", 1) or 1))
        goal_msg.planning_timeout_sec = float(payload.get("planning_timeout_sec", 0.0) or 0.0)
        goal_msg.execution_timeout_sec = float(payload.get("execution_timeout_sec", 0.0) or 0.0)
        goal_msg.success_criteria = str(payload.get("success_criteria", "") or "").strip()
        goal_msg.failure_policy = str(payload.get("failure_policy", "") or "").strip()
        goal_msg.verify_policy = str(payload.get("verify_policy", "") or "").strip()
        goal_msg.recovery_policy = str(payload.get("recovery_policy", "") or "").strip()
        goal_msg.handoff_context_json = str(
            payload.get("handoff_context_json", "") or ""
        ).strip()
        goal_msg.reason = str(payload.get("reason", "") or "").strip()
        goal_msg.prompt_text = str(payload.get("prompt_text", self._last_prompt_text) or self._last_prompt_text).strip()
        goal_msg.raw_json = str(payload.get("raw_json", "") or "").strip()
        return goal_msg

    def _on_execute_task_result(self, goal_id: str, future: Any) -> None:
        try:
            wrapped_result = future.result()
            result = getattr(wrapped_result, "result", None)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"execute_task result callback failed: {exc}")
            return
        with self._lock:
            if str(self._active_execute_task_goal_id or "") == str(goal_id or ""):
                self._execute_task_goal_handle = None
                self._active_execute_task_goal_id = ""
            ignored = str(goal_id or "") in self._ignored_task_goal_ids
            if ignored:
                self._ignored_task_goal_ids.discard(str(goal_id or ""))
        if result is None:
            return
        if ignored:
            return
        status = _task_execution_status_to_dict(result.status)
        with self._lock:
            self._task_execution_status = status
            self._state_version += 1

    def _send_execute_task_goal(
        self,
        goal_msg: TaskGoal,
        *,
        timeout_sec: float = 12.0,
    ) -> tuple[bool, str]:
        if not self.execute_task_client.wait_for_server(timeout_sec=timeout_sec):
            return False, "task executive action unavailable"
        request = ExecuteTask.Goal()
        request.goal = goal_msg
        future = self.execute_task_client.send_goal_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_sec):
            return False, "task goal request timed out"
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        if goal_handle is None or not bool(goal_handle.accepted):
            return False, "task goal rejected"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, goal_id=str(goal_msg.goal_id or ""): self._on_execute_task_result(goal_id, future)
        )
        with self._lock:
            self._execute_task_goal_handle = goal_handle
            self._active_execute_task_goal_id = str(goal_msg.goal_id or "")
            self._ignored_task_goal_ids.discard(self._active_execute_task_goal_id)
            self._task_goal = _task_goal_to_dict(goal_msg)
            self._state_version += 1
        return True, f"task goal accepted ({goal_msg.goal_id})"

    def _interrupt_active_execution(self, *, reason: str, timeout_sec: float = 2.0) -> tuple[bool, str]:
        self._set_pending_execute(False)
        self._set_intervention(False)
        self._set_dialog_status(status="idle", pending_auto_execute=False, last_error="")
        with self._lock:
            goal_handle = self._execute_task_goal_handle
            goal_id = str(self._active_execute_task_goal_id or "")
            if goal_id:
                self._ignored_task_goal_ids.add(goal_id)
            self._execute_task_goal_handle = None
            self._active_execute_task_goal_id = ""
            self._task_goal = {"updated_at": 0.0}
            self._task_execution_status = {
                "phase": "idle",
                "message": str(reason or "execution interrupted"),
                "updated_at": _now_sec(),
            }
            self._pending_execute_dispatching = False
            self._state_version += 1
        if goal_handle is None:
            return True, "no active task goal"
        future = goal_handle.cancel_goal_async()
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(max(0.1, float(timeout_sec))):
            return False, "task cancel request timed out"
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        goals_canceling = list(getattr(response, "goals_canceling", []) or [])
        if goals_canceling:
            return True, "task cancel requested"
        return True, "task already terminal"

    def _call_set_bool(self, client: Any, value: bool, timeout_sec: float = 4.0) -> tuple[bool, str]:
        request = SetBool.Request()
        request.data = bool(value)
        ok, error, result = self._call_service_request(client, request, timeout_sec=timeout_sec)
        if not ok or result is None:
            return False, error or "service call failed"
        return bool(result.success), str(result.message or "")

    def _execution_backend_ready(self) -> bool:
        try:
            action_ready = bool(self.execute_task_client.server_is_ready())
        except Exception:
            action_ready = False
        try:
            reset_ready = bool(self.reset_pick_client.service_is_ready())
        except Exception:
            reset_ready = False
        try:
            return_home_ready = bool(self.return_home_client.service_is_ready())
        except Exception:
            return_home_ready = False
        try:
            search_ready = bool(self.start_search_sweep_client.service_is_ready())
        except Exception:
            search_ready = False
        return bool(
            action_ready
            and reset_ready
            and return_home_ready
            and search_ready
            and self._dialog_model_ready()
            and self._perception_pipeline_ready()
        )

    def _dialog_model_health_url(self) -> str:
        endpoint = str(self.dialog_model_endpoint or "").strip()
        if not endpoint:
            return ""
        parts = urlsplit(endpoint)
        path = str(parts.path or "").strip()
        if path.endswith("/chat/completions"):
            path = path[: -len("/chat/completions")] + "/models"
        elif path.endswith("/v1"):
            path = path + "/models"
        else:
            path = "/v1/models"
        return urlunsplit((parts.scheme, parts.netloc, path, "", ""))

    def _dialog_model_ready(self) -> bool:
        now_sec = _now_sec()
        cached_checked_at = float(self._dialog_endpoint_health.get("checked_at", 0.0) or 0.0)
        if (now_sec - cached_checked_at) <= 2.0:
            return bool(self._dialog_endpoint_health.get("ready", False))

        ready = False
        health_url = self._dialog_model_health_url()
        if health_url:
            try:
                headers = {}
                if self.dialog_api_key:
                    headers["Authorization"] = f"Bearer {self.dialog_api_key}"
                response = self._dialog_http.get(
                    health_url,
                    headers=headers,
                    timeout=max(0.25, min(1.0, float(self.dialog_request_timeout_sec))),
                )
                ready = bool(response.ok)
            except Exception:
                ready = False

        self._dialog_endpoint_health["ready"] = bool(ready)
        self._dialog_endpoint_health["checked_at"] = now_sec
        return bool(ready)

    def _perception_pipeline_ready(self) -> bool:
        latest_detection_at = max(
            float(self._detection_result.get("updated_at", 0.0) or 0.0),
            float(self._detection_debug.get("updated_at", 0.0) or 0.0),
            float(self.rgb_stream.latest().updated_at or 0.0),
        )
        if latest_detection_at <= 0.0:
            return False
        return (_now_sec() - latest_detection_at) <= 3.0

    def _call_world_reset(self, *, model_only: bool = True, timeout_sec: float = 4.0) -> tuple[bool, str]:
        request = ControlWorld.Request()
        request.world_control.reset.model_only = bool(model_only)
        request.world_control.reset.all = not bool(model_only)
        request.world_control.reset.time_only = False
        ok, error, result = self._call_service_request(
            self.scene_reset_client,
            request,
            timeout_sec=timeout_sec,
        )
        if not ok or result is None:
            return False, error or "service call failed"
        return bool(result.success), "scene reset accepted" if bool(result.success) else "scene reset failed"

    def _call_scene_set_pose(self, timeout_sec: float = 4.0) -> tuple[bool, str]:
        request = SetEntityPose.Request()
        request.entity = Entity(
            name=self.scene_target_model_name,
            type=Entity.MODEL,
        )
        request.pose.position.x = float(self.scene_target_reset_pose["position"]["x"])
        request.pose.position.y = float(self.scene_target_reset_pose["position"]["y"])
        request.pose.position.z = float(self.scene_target_reset_pose["position"]["z"])
        request.pose.orientation.x = float(self.scene_target_reset_pose["orientation"]["x"])
        request.pose.orientation.y = float(self.scene_target_reset_pose["orientation"]["y"])
        request.pose.orientation.z = float(self.scene_target_reset_pose["orientation"]["z"])
        request.pose.orientation.w = float(self.scene_target_reset_pose["orientation"]["w"])
        ok, error, result = self._call_service_request(
            self.scene_set_pose_client,
            request,
            timeout_sec=timeout_sec,
        )
        if not ok or result is None:
            return False, error or "service call failed"
        if bool(result.success):
            with self._lock:
                self._scene_target_pose = {
                    "model_name": self.scene_target_model_name,
                    "found": True,
                    "frame_id": "world",
                    "child_frame_id": self.scene_target_model_name,
                    "position": copy.deepcopy(self.scene_target_reset_pose["position"]),
                    "orientation": copy.deepcopy(self.scene_target_reset_pose["orientation"]),
                    "reset_pose": copy.deepcopy(self.scene_target_reset_pose),
                    "updated_at": _now_sec(),
                }
                self._state_version += 1
        return bool(result.success), (
            "target pose restored" if bool(result.success) else "target pose restore failed"
        )

    def _debug_open_views_script_path(self) -> str:
        if self.debug_open_views_script:
            return self.debug_open_views_script
        distro = str(os.environ.get("WSL_DISTRO_NAME", "Ubuntu-24.04") or "Ubuntu-24.04")
        script_path = str(self._ws_root / "scripts" / "open_programme_debug_views.ps1")
        script_path = script_path.replace("/", "\\")
        return f"\\\\wsl.localhost\\{distro}{script_path}"

    def _windows_powershell_exe(self) -> str:
        candidates = [
            "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe",
            "/mnt/c/Windows/Sysnative/WindowsPowerShell/v1.0/powershell.exe",
            "powershell.exe",
        ]
        for candidate in candidates:
            if candidate == "powershell.exe" or os.path.exists(candidate):
                return candidate
        return "powershell.exe"

    def open_debug_views(self) -> tuple[bool, str, dict[str, Any]]:
        distro = str(os.environ.get("WSL_DISTRO_NAME", "Ubuntu-24.04") or "Ubuntu-24.04")
        script_path = self._debug_open_views_script_path()
        args = [
            self._windows_powershell_exe(),
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            script_path,
            "-Distro",
            distro,
        ]
        if self.debug_open_gazebo_gui:
            args.append("-OpenGazeboGui")
        if self.debug_open_rviz:
            args.append("-OpenRviz")
        ok = False
        message = "debug views disabled"
        try:
            result = subprocess.run(
                args,
                cwd=str(self._ws_root),
                capture_output=True,
                text=True,
                timeout=20.0,
                check=False,
            )
            stdout = str(result.stdout or "").strip()
            stderr = str(result.stderr or "").strip()
            message = stdout or stderr or "debug launch requested"
            ok = result.returncode == 0
        except Exception as exc:  # noqa: BLE001
            message = str(exc)
        self._record_event(
            "execution",
            "info" if ok else "error",
            "debug views requested" if ok else f"debug views failed: {message}",
            data={
                "gazebo_gui_enabled": self.debug_open_gazebo_gui,
                "rviz_enabled": self.debug_open_rviz,
                "gazebo_log": str(self._debug_gazebo_log),
                "rviz_log": str(self._debug_rviz_log),
            },
            feedback=True,
        )
        return ok, message, self.build_state()

    def _call_move_joints(
        self,
        joint_ids: list[int],
        angles_deg: list[float],
        *,
        duration_ms: int,
        wait: bool,
        timeout_sec: float = 4.0,
    ) -> tuple[bool, str]:
        request = MoveArmJoints.Request()
        request.joint_ids = [int(item) for item in joint_ids]
        request.angles_deg = [float(item) for item in angles_deg]
        request.duration_ms = int(duration_ms)
        request.wait = bool(wait)
        ok, error, result = self._call_service_request(
            self.control_arm_move_joints_client,
            request,
            timeout_sec=timeout_sec,
        )
        if not ok or result is None:
            return False, error or "service call failed"
        return bool(result.success), str(result.message or "")

    def _request_return_home_motion(
        self,
        *,
        reason: str,
        retries: int,
        retry_delay_sec: float,
    ) -> tuple[bool, str]:
        if len(self.return_home_joint_ids) != len(self.return_home_joint_angles_deg):
            return False, "home joint configuration is invalid"
        last_error = "unknown error"
        for attempt in range(max(1, retries)):
            enable_ok, enable_message = self._call_set_bool(
                self.control_arm_enable_client,
                True,
                timeout_sec=3.0,
            )
            if not enable_ok:
                last_error = f"enable failed: {enable_message}"
            else:
                move_timeout_sec = max(
                    4.0,
                    float(self.return_home_duration_ms) / 1000.0
                    + (3.0 if self.return_home_wait_for_completion else 1.5),
                )
                move_ok, move_message = self._call_move_joints(
                    self.return_home_joint_ids,
                    self.return_home_joint_angles_deg,
                    duration_ms=self.return_home_duration_ms,
                    wait=self.return_home_wait_for_completion,
                    timeout_sec=move_timeout_sec,
                )
                if move_ok:
                    return True, move_message or "trajectory goal accepted"
                last_error = f"move failed: {move_message}"
            if attempt + 1 < max(1, retries):
                time.sleep(max(0.1, retry_delay_sec))
        self.get_logger().warn(f"{reason}: {last_error}")
        return False, last_error

    def _request_stop_search_sweep(
        self,
        *,
        reason: str,
        timeout_sec: float = 1.5,
    ) -> tuple[bool, str]:
        ok, message = self._call_trigger(
            self.stop_search_sweep_client,
            timeout_sec=timeout_sec,
        )
        if not ok:
            self.get_logger().warn(f"{reason}: stop search sweep failed: {message}")
        return ok, message

    def _startup_prepare_home_worker(self) -> None:
        time.sleep(self.startup_prepare_home_delay_sec)
        ok, message = self._request_return_home_motion(
            reason="startup home/open prepare",
            retries=6,
            retry_delay_sec=1.0,
        )
        self._record_event(
            "execution",
            "info" if ok else "warn",
            "startup home/open prepared"
            if ok
            else f"startup home/open prepare failed: {message}",
            feedback=False,
        )

    def publish_prompt(self, prompt: str) -> dict[str, Any]:
        message = String()
        message.data = str(prompt or "").strip()
        self.prompt_pub.publish(message)
        self._last_prompt_text = message.data
        self._set_intervention(False)
        self._set_pending_execute(False)
        sweep_ok, sweep_message = self._call_trigger(self.start_search_sweep_client, timeout_sec=1.0)
        self._record_event(
            "prompt",
            "info",
            f"prompt submitted: {message.data or '<empty>'}",
            data={
                "search_sweep_started": sweep_ok,
                "search_sweep_message": sweep_message,
            },
            feedback=True,
        )
        if not sweep_ok:
            self._record_event(
                "execution",
                "warn",
                f"search sweep did not start after prompt: {sweep_message}",
                feedback=False,
            )
        return self.build_state()

    def publish_override(self, payload: dict[str, Any]) -> dict[str, Any]:
        target_label = str(payload.get("target_label", "") or "").strip()
        target_hint = str(payload.get("target_hint", "") or target_label).strip()
        self._set_pending_execute(False)
        semantic = self._publish_semantic_payload(
            {
                **payload,
                "reason": str(payload.get("reason", "manual override from web ui") or ""),
            },
            prompt_text=str(payload.get("prompt_text", self._last_prompt_text) or ""),
            raw_json=str(payload.get("raw_json", "") or ""),
            intervention_active=True,
            intervention_source="override_api",
            intervention_label=target_label or target_hint,
        )
        self._record_event(
            "override",
            "info",
            f"override applied: {target_label or target_hint or '<unset>'}",
            data={"label": target_label or target_hint, "task": semantic.get("task", "pick")},
            feedback=True,
        )
        return self.build_state()

    def replan(self) -> tuple[bool, str, dict[str, Any]]:
        self._set_pending_execute(False)
        ok, message = self._call_trigger(self.reset_pick_client)
        if not ok:
            self._record_event("execution", "error", f"replan failed: {message}", feedback=True)
            return False, message, self.build_state()

        replay_prompt = self._last_prompt_text or str(
            self._semantic_task.get("prompt_text", "") or ""
        )
        self._set_intervention(False)
        if replay_prompt:
            prompt_msg = String()
            prompt_msg.data = replay_prompt
            self.prompt_pub.publish(prompt_msg)
            self._record_event(
                "replan",
                "info",
                f"re-plan requested; replayed prompt: {replay_prompt}",
                feedback=True,
            )
            return True, "pick session reset and prompt replayed", self.build_state()

        self._record_event(
            "replan",
            "info",
            "pick session reset without prompt replay",
            feedback=True,
        )
        return True, "pick session reset", self.build_state()

    def execute_pick(
        self,
        *,
        start_search_sweep: bool = True,
        allow_queue: bool = True,
        request_source: str = "manual_api",
    ) -> tuple[bool, str, dict[str, Any]]:
        del allow_queue
        goal_msg = self._build_task_goal_msg(start_search_sweep=start_search_sweep)
        if goal_msg is None:
            message = "no structured task is ready for execution"
            self._record_event("execution", "error", message, feedback=True)
            return False, message, self.build_state()

        ok, message = self._send_execute_task_goal(goal_msg)
        if ok:
            self._set_pending_execute(False)
            self._record_event(
                "execution",
                "info",
                "task execute request accepted",
                data={
                    "request_source": request_source,
                    "goal_id": goal_msg.goal_id,
                    "goal_type": goal_msg.goal_type,
                    "task": goal_msg.task,
                    "target_label": goal_msg.target_label or goal_msg.target_hint,
                    "start_search_sweep": bool(start_search_sweep),
                },
                feedback=True,
            )
            return True, message, self.build_state()

        self._record_event(
            "execution",
            "error",
            f"execute request failed: {message}",
            data={
                "request_source": request_source,
                "target_label": goal_msg.target_label or goal_msg.target_hint,
                "goal_type": goal_msg.goal_type,
            },
            feedback=True,
        )
        return False, message, self.build_state()

    def return_home(self) -> tuple[bool, str, dict[str, Any]]:
        cancel_ok, cancel_message = self._interrupt_active_execution(reason="return home")
        stop_ok, stop_message = self._request_stop_search_sweep(
            reason="return home request",
            timeout_sec=1.5,
        )
        trigger_timeout_sec = max(
            6.0,
            float(self.return_home_duration_ms) / 1000.0
            + (3.0 if self.return_home_wait_for_completion else 1.5),
        )
        home_ok, home_message = self._call_trigger(
            self.return_home_client, timeout_sec=trigger_timeout_sec
        )
        if not home_ok:
            home_ok, home_message = self._request_return_home_motion(
                reason="return home request",
                retries=2,
                retry_delay_sec=0.5,
            )
        detail_parts = [
            f"task interrupt: {cancel_message}",
            f"stop sweep: {stop_message}",
            f"return home: {home_message}",
        ]
        self._record_event(
            "execution",
            "info" if home_ok else "error",
            "return home requested"
            if home_ok
            else f"return home failed: {' | '.join(detail_parts)}",
            data={
                "task_interrupt_ok": bool(cancel_ok),
                "stop_search_sweep_ok": bool(stop_ok),
                "return_home_ok": bool(home_ok),
            },
            feedback=True,
        )
        return home_ok, " | ".join(detail_parts), self.build_state()

    def reset_scene(self) -> tuple[bool, str, dict[str, Any]]:
        cancel_ok, cancel_message = self._interrupt_active_execution(reason="scene reset")
        stop_ok, stop_message = self._request_stop_search_sweep(
            reason="scene reset request",
            timeout_sec=1.5,
        )
        home_ok = True
        home_message = "skipped"
        session_ok = True
        session_message = "skipped"
        if self.scene_reset_return_home_after_reset:
            trigger_timeout_sec = max(
                6.0,
                float(self.return_home_duration_ms) / 1000.0
                + (3.0 if self.return_home_wait_for_completion else 1.5),
            )
            home_ok, home_message = self._call_trigger(
                self.return_home_client, timeout_sec=trigger_timeout_sec
            )
            if not home_ok:
                home_ok, home_message = self._request_return_home_motion(
                    reason="scene reset home prepare",
                    retries=3,
                    retry_delay_sec=0.5,
                )
            session_ok, session_message = self._call_trigger(self.reset_pick_client, timeout_sec=1.5)
        else:
            session_ok, session_message = self._call_trigger(
                self.reset_pick_client, timeout_sec=1.5
            )
        world_ok = True
        world_message = "skipped"
        if self.scene_reset_use_world_reset:
            world_ok, world_message = self._call_world_reset(model_only=True, timeout_sec=3.0)
        pose_ok, pose_message = self._call_scene_set_pose(timeout_sec=3.0)
        if (world_ok or pose_ok) and self.scene_reset_delay_sec > 0.0:
            time.sleep(self.scene_reset_delay_sec)

        scene_ok = bool(world_ok or pose_ok)
        ok = bool(scene_ok and session_ok and home_ok)
        with self._lock:
            restore_waiting_execute = bool(self._target_locked)
            self._grasp_proposals = {
                "updated_at": 0.0,
                "proposals": [],
                "selected_index": 0,
                "count": 0,
            }
            self._grasp_backend_debug = {"updated_at": 0.0}
            if restore_waiting_execute:
                self._pick_status = {
                    "phase": "waiting_execute",
                    "message": "target locked; waiting for explicit execute command",
                    "updated_at": _now_sec(),
                }
                self._task_execution_status = {
                    "phase": "waiting_execute",
                    "message": "target locked; waiting for explicit execute command",
                    "updated_at": _now_sec(),
                }
            else:
                self._pick_status = {
                    "phase": "idle",
                    "message": "scene reset",
                    "updated_at": _now_sec(),
                }
                self._task_execution_status = {
                    "phase": "idle",
                    "message": "scene reset",
                    "updated_at": _now_sec(),
                }
            self._pick_active = False
            self._state_version += 1
        detail_parts = [f"task interrupt: {cancel_message}", f"stop sweep: {stop_message}"]
        detail_parts.append(
            f"{'return home' if self.scene_reset_return_home_after_reset else 'pick reset'}: "
            f"{session_message}"
        )
        detail_parts.append(f"world reset: {world_message}")
        detail_parts.append(f"target pose: {pose_message}")
        message = " | ".join(part for part in detail_parts if part)
        self._record_event(
            "execution",
            "info" if ok else "error",
            "scene reset requested" if ok else f"scene reset failed: {message}",
            data={
                "task_interrupt_ok": bool(cancel_ok),
                "stop_search_sweep_ok": bool(stop_ok),
                "session_reset_ok": bool(session_ok),
                "world_reset_ok": world_ok,
                "target_pose_ok": pose_ok,
                "return_home_ok": home_ok,
                "scene_target_pose": copy.deepcopy(self._scene_target_pose),
            },
            feedback=True,
        )
        return ok, message, self.build_state()

    def build_state(self) -> dict[str, Any]:
        with self._lock:
            semantic = copy.deepcopy(self._semantic_task)
            semantic_result = copy.deepcopy(self._semantic_result)
            task_goal = copy.deepcopy(self._task_goal)
            task_status = copy.deepcopy(self._task_execution_status)
            detection = copy.deepcopy(self._detection_result)
            detection_debug = copy.deepcopy(self._detection_debug)
            pick_status = copy.deepcopy(self._pick_status)
            proposals = copy.deepcopy(self._grasp_proposals)
            backend_debug = copy.deepcopy(self._grasp_backend_debug)
            tactile = copy.deepcopy(self._tactile)
            arm_state = copy.deepcopy(self._arm_state)
            intervention = copy.deepcopy(self._intervention)
            pending_execute = copy.deepcopy(self._pending_execute)
            health_by_node = copy.deepcopy(self._health_by_node)
            dialog = copy.deepcopy(self._dialog)
            scene_target_pose = copy.deepcopy(self._scene_target_pose)
            events = list(self._events)
            feedback_events = list(self._feedback_events)
            target_locked = bool(self._target_locked)
            pick_active = bool(self._pick_active)
            state_version = int(self._state_version)

        health_items = sorted(
            health_by_node.values(),
            key=lambda item: (
                int(item.get("level", 0)) * -1,
                float(item.get("updated_at", 0.0)) * -1,
            ),
        )
        health_issues = [
            item
            for item in health_items
            if (not item.get("healthy", True)) or int(item.get("level", 0)) > 0
        ]

        vision_updated_at = max(
            float(detection.get("updated_at", 0.0)),
            float(detection_debug.get("updated_at", 0.0)),
            float(self.rgb_stream.latest().updated_at),
            float(self.detection_overlay_stream.latest().updated_at),
            float(self.grasp_overlay_stream.latest().updated_at),
        )
        debug_candidates = _debug_candidates_from_payload(detection_debug)
        candidate_summary = str(detection_debug.get("candidate_summary", "") or "").strip()
        if not candidate_summary and debug_candidates:
            candidate_summary = "; ".join(
                f"#{int(item.get('index', 0))}:{str(item.get('label', '') or '<unknown>')} "
                f"({float(item.get('confidence', 0.0) or 0.0):.3f}, {str(item.get('status', '') or 'unknown')})"
                for item in debug_candidates[:5]
            )

        execution_updated_at = max(
            float(task_goal.get("updated_at", 0.0)),
            float(task_status.get("updated_at", 0.0)),
            float(pick_status.get("updated_at", 0.0)),
            float(detection.get("updated_at", 0.0)),
            float(proposals.get("updated_at", 0.0)),
        )

        phase = self._derive_phase(
            semantic=semantic,
            task_status=task_status,
            detection=detection,
            detection_debug=detection_debug,
            target_locked=target_locked,
            pick_active=pick_active,
            pick_status=pick_status,
        )
        backend_ready = self._execution_backend_ready()
        return _json_compatible(
            {
                "connection": {
                    "backend_ready": bool(backend_ready),
                    "backend_time": _now_sec(),
                    "state_version": state_version,
                    "host": self.host,
                    "port": self.port,
                },
                "semantic": {
                    **semantic,
                    "result": semantic_result,
                    "updated_at": float(semantic.get("updated_at", 0.0)),
                },
                "vision": {
                    "detection": detection,
                    "debug": detection_debug,
                    "debug_candidates": debug_candidates,
                    "selected_candidate": detection_debug.get("selected_candidate"),
                    "candidate_summary": candidate_summary,
                    "image_width": int(
                        detection.get("image_width", 0)
                        or detection_debug.get("image_width", 0)
                        or 0
                    ),
                    "image_height": int(
                        detection.get("image_height", 0)
                        or detection_debug.get("image_height", 0)
                        or 0
                    ),
                    "updated_at": vision_updated_at,
                },
                "execution": {
                    "phase": phase,
                    "target_locked": target_locked,
                    "pick_active": pick_active,
                    "pending_execute": pending_execute,
                    "task_goal": task_goal,
                    "task_status": task_status,
                    "pick_status": pick_status,
                    "intervention": intervention,
                    "grasp_proposals": proposals,
                    "backend_debug": backend_debug,
                    "updated_at": execution_updated_at,
                },
                "tactile": tactile,
                "health": {
                    "healthy": not health_issues,
                    "issues": health_issues,
                    "latest": health_items[:12],
                    "arm_state": arm_state,
                    "updated_at": max(
                        float(arm_state.get("updated_at", 0.0)),
                        max(
                            (float(item.get("updated_at", 0.0)) for item in health_items),
                            default=0.0,
                        ),
                    ),
                },
                "logs": {
                    "stepper_phase": phase,
                    "intervention_badge": bool(intervention.get("active", False)),
                    "events": events,
                    "updated_at": max(
                        (float(item.get("created_at", 0.0)) for item in events), default=0.0
                    ),
                },
                "ui_feedback": {
                    "events": feedback_events,
                    "last_event_id": int(feedback_events[-1]["id"]) if feedback_events else 0,
                },
                "dialog": {
                    **dialog,
                    "status_label": _dialog_status_label(str(dialog.get("status", "idle"))),
                },
                "scene": {
                    "target": scene_target_pose,
                },
            }
        )

    def _derive_phase(
        self,
        *,
        semantic: dict[str, Any],
        task_status: dict[str, Any],
        detection: dict[str, Any],
        detection_debug: dict[str, Any],
        target_locked: bool,
        pick_active: bool,
        pick_status: dict[str, Any],
    ) -> str:
        task_phase = str(task_status.get("phase", "") or "").strip()
        if task_phase and task_phase != "idle":
            return task_phase
        pick_phase = str(pick_status.get("phase", "") or "").strip()
        if pick_phase in {"waiting_execute", "planning", "executing", "completed", "error"}:
            return pick_phase
        if pick_active:
            return "executing"
        if target_locked:
            return "target_locked"
        if bool(detection.get("accepted")) or bool(_debug_candidates_from_payload(detection_debug)):
            return "vision_ready"
        if bool(semantic.get("task")) or bool(semantic.get("target_label")) or bool(
            semantic.get("target_hint")
        ):
            return "semantic_ready"
        return "idle"


def _frontend_root_candidates(bridge: TactileWebGateway) -> list[Path]:
    candidates: list[Path] = []
    seen: set[str] = set()

    def add_candidate(path: Path) -> None:
        resolved = path.expanduser().resolve()
        key = resolved.as_posix()
        if key in seen:
            return
        seen.add(key)
        candidates.append(resolved)

    if bridge.frontend_dist_dir:
        add_candidate(Path(bridge.frontend_dist_dir))

    if get_package_share_directory is not None:
        try:
            share_dir = Path(get_package_share_directory("tactile_web_bridge"))
            add_candidate(share_dir / "frontend" / "dist")
        except PackageNotFoundError:
            pass
        except Exception:  # noqa: BLE001
            pass

    add_candidate(Path(__file__).resolve().parents[1] / "frontend" / "dist")
    return candidates


def _frontend_root_dir(bridge: TactileWebGateway) -> Path:
    candidates = _frontend_root_candidates(bridge)
    for candidate in candidates:
        if (candidate / "index.html").exists():
            return candidate
    return candidates[0]


def create_app(bridge: TactileWebGateway) -> Any:
    app = FastAPI(title="Programme UI Gateway")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://127.0.0.1:5173",
            "http://localhost:5173",
            "http://127.0.0.1:8765",
            "http://localhost:8765",
        ],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    dist_dir = _frontend_root_dir(bridge)
    assets_dir = dist_dir / "assets"
    if assets_dir.exists():
        app.mount(
            "/assets",
            StaticFiles(directory=str(assets_dir), follow_symlink=True),
            name="assets",
        )

    @app.get("/api/bootstrap")
    async def bootstrap() -> JSONResponse:
        return JSONResponse(
            {
                "state": bridge.build_state(),
                "streams": {
                    "rgb": "/api/streams/rgb.mjpeg",
                    "detection_overlay": "/api/streams/detection_overlay.mjpeg",
                    "grasp_overlay": "/api/streams/grasp_overlay.mjpeg",
                },
                "frontend_ready": (dist_dir / "index.html").exists(),
            }
        )

    @app.post("/api/prompt")
    async def post_prompt(payload: dict[str, Any]) -> JSONResponse:
        prompt = str(payload.get("prompt", "") or "").strip()
        if not prompt:
            raise HTTPException(status_code=400, detail="prompt is required")
        return JSONResponse({"ok": True, "state": bridge.publish_prompt(prompt)})

    @app.post("/api/dialog/message")
    async def post_dialog_message(payload: dict[str, Any]) -> JSONResponse:
        try:
            state = bridge.dialog_send_message(payload)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except requests.HTTPError as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        return JSONResponse({"ok": True, "state": state})

    @app.post("/api/dialog/mode")
    async def post_dialog_mode(payload: dict[str, Any]) -> JSONResponse:
        mode = str(payload.get("mode", "") or "").strip().lower()
        if mode not in {"review", "auto"}:
            raise HTTPException(status_code=400, detail="mode must be review or auto")
        return JSONResponse({"ok": True, "state": bridge.set_dialog_mode(mode)})

    @app.post("/api/dialog/reply-language")
    async def post_dialog_reply_language(payload: dict[str, Any]) -> JSONResponse:
        reply_language = _normalize_dialog_reply_language(payload.get("reply_language", ""))
        return JSONResponse(
            {"ok": True, "state": bridge.set_dialog_reply_language(reply_language)}
        )

    @app.post("/api/dialog/reset")
    async def post_dialog_reset() -> JSONResponse:
        return JSONResponse({"ok": True, "state": bridge.reset_dialog_session()})

    @app.post("/api/task/override")
    async def post_override(payload: dict[str, Any]) -> JSONResponse:
        return JSONResponse({"ok": True, "state": bridge.publish_override(payload)})

    @app.post("/api/task/replan")
    async def post_replan() -> JSONResponse:
        ok, message, state = bridge.replan()
        status_code = 200 if ok else 503
        return JSONResponse(
            {"ok": ok, "message": message, "state": state},
            status_code=status_code,
        )

    @app.post("/api/execution/execute")
    async def post_execute() -> JSONResponse:
        ok, message, state = bridge.execute_pick()
        status_code = 200 if ok else 409
        return JSONResponse(
            {"ok": ok, "message": message, "state": state},
            status_code=status_code,
        )

    @app.post("/api/execution/return-home")
    async def post_return_home() -> JSONResponse:
        ok, message, state = bridge.return_home()
        status_code = 200 if ok else 409
        return JSONResponse(
            {"ok": ok, "message": message, "state": state},
            status_code=status_code,
        )

    @app.post("/api/execution/reset-scene")
    async def post_reset_scene() -> JSONResponse:
        ok, message, state = bridge.reset_scene()
        status_code = 200 if ok else 409
        return JSONResponse(
            {"ok": ok, "message": message, "state": state},
            status_code=status_code,
        )

    @app.post("/api/debug/open-views")
    async def post_open_debug_views() -> JSONResponse:
        ok, message, state = bridge.open_debug_views()
        status_code = 200 if ok else 409
        return JSONResponse(
            {"ok": ok, "message": message, "state": state},
            status_code=status_code,
        )

    def _stream_generator(buffer: MjpegFrameBuffer):
        last_update = 0.0
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        while True:
            frame = buffer.wait_next(last_update, timeout_sec=1.0)
            if frame.jpeg is None or frame.updated_at <= last_update:
                continue
            last_update = frame.updated_at
            yield boundary + frame.jpeg + b"\r\n"

    @app.get("/api/streams/rgb.mjpeg")
    async def stream_rgb() -> StreamingResponse:
        return StreamingResponse(
            _stream_generator(bridge.rgb_stream),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/api/streams/detection_overlay.mjpeg")
    async def stream_detection_overlay() -> StreamingResponse:
        return StreamingResponse(
            _stream_generator(bridge.detection_overlay_stream),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/api/streams/grasp_overlay.mjpeg")
    async def stream_grasp_overlay() -> StreamingResponse:
        return StreamingResponse(
            _stream_generator(bridge.grasp_overlay_stream),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/api/diagnostics/streams")
    async def get_stream_diagnostics() -> JSONResponse:
        return JSONResponse(
            {
                "rgb": bridge.rgb_encoder.stats(),
                "detection_overlay": bridge.detection_overlay_encoder.stats(),
                "grasp_overlay": bridge.grasp_overlay_encoder.stats(),
                "jpeg_quality": int(bridge.stream_jpeg_quality),
            }
        )

    @app.websocket("/ws/live")
    async def ws_state(websocket: WebSocket) -> None:
        await websocket.accept()
        last_state_version = -1
        last_sent_at = 0.0
        poll_interval_sec = max(0.05, float(getattr(bridge, "live_state_period_sec", 0.15)))
        try:
            while True:
                state = bridge.build_state()
                state_version = int(state["connection"]["state_version"])
                now_sec = _now_sec()
                if state_version != last_state_version or (now_sec - last_sent_at) >= 1.0:
                    await websocket.send_json(state)
                    last_state_version = state_version
                    last_sent_at = now_sec
                await asyncio.sleep(poll_interval_sec)
        except (WebSocketDisconnect, RuntimeError):
            return

    def _fallback_html() -> str:
        return """
        <!doctype html>
        <html lang="en">
          <head>
            <meta charset="utf-8" />
            <title>Programme UI Gateway</title>
            <style>
              body { background: #0d1524; color: #e8edf7; font-family: 'IBM Plex Sans', 'Segoe UI', sans-serif; margin: 0; padding: 48px; }
              .panel { max-width: 820px; margin: 0 auto; padding: 32px; border-radius: 22px; background: linear-gradient(160deg, rgba(28,45,74,0.95), rgba(10,17,30,0.96)); box-shadow: 0 24px 80px rgba(0,0,0,0.35); }
              h1 { margin: 0 0 12px; font-size: 34px; }
              p { line-height: 1.6; color: #bfd0ea; }
              code { background: rgba(255,255,255,0.08); padding: 2px 6px; border-radius: 6px; }
            </style>
          </head>
          <body>
            <div class="panel">
              <h1>Programme UI Gateway</h1>
              <p>The backend is running, but the built frontend bundle was not found.</p>
              <p>Run the Vite app under <code>src/tactile_web_bridge/frontend</code> for development, or build it so the bundle appears under <code>frontend/dist</code>.</p>
            </div>
          </body>
        </html>
        """

    @app.get("/")
    async def root() -> HTMLResponse:
        index_path = dist_dir / "index.html"
        if index_path.exists():
            return HTMLResponse(index_path.read_text(encoding="utf-8"))
        return HTMLResponse(_fallback_html())

    @app.get("/{full_path:path}")
    async def spa_fallback(full_path: str) -> HTMLResponse:
        if full_path.startswith("api/"):
            raise HTTPException(status_code=404, detail="not found")
        index_path = dist_dir / "index.html"
        if index_path.exists():
            return HTMLResponse(index_path.read_text(encoding="utf-8"))
        return HTMLResponse(_fallback_html(), status_code=200)

    return app


def main(args: Optional[list[str]] = None) -> None:
    try:
        import uvicorn
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "tactile_web_gateway requires fastapi and uvicorn. "
            "Install the tactile_web_bridge Python dependencies first."
        ) from exc

    rclpy.init(args=args)
    node = TactileWebGateway()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(node)
    try:
        uvicorn.run(app, host=node.host, port=node.port, log_level="info", ws="wsproto")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
