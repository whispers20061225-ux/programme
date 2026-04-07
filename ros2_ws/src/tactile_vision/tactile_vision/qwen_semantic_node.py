from __future__ import annotations

import os
import threading
import time
from typing import Any, Optional

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tactile_interfaces.msg import SemanticTask

from tactile_vision.modular_common import (
    compact_json,
    decode_color_image,
    encode_image_to_base64_jpeg,
    extract_first_json_object,
    extract_message_text,
)


DEFAULT_SYSTEM_PROMPT = (
    "You are a robot task-understanding assistant. "
    "Return exactly one JSON object and no markdown or extra commentary. "
    "Required keys: task, target_label, target_hint, constraints, excluded_labels, "
    "confidence, need_human_confirm, reason, detection_spec. "
    "constraints and excluded_labels must be arrays of short strings. "
    "detection_spec must be a JSON object. "
    "When a task target exists, detection_spec must include primary_label, prompt_classes, "
    "negative_labels, and attributes. prompt_classes must be short English detector phrases. "
    "confidence must be a number from 0.0 to 1.0."
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


def normalize_semantic_result(
    data: dict[str, Any],
    *,
    default_task: str,
    default_target_hint: str,
    default_constraints: list[str],
    prompt_text: str,
    raw_json: str,
) -> dict[str, Any]:
    task = str(data.get("task") or default_task).strip() or default_task
    target_label = str(
        data.get("target_label") or data.get("target") or data.get("label") or ""
    ).strip()
    target_hint = str(data.get("target_hint") or target_label or default_target_hint).strip()

    constraints_raw = data.get("constraints", default_constraints)
    if not isinstance(constraints_raw, list):
        constraints_raw = default_constraints
    constraints = [str(item).strip() for item in constraints_raw if str(item).strip()]
    normalized_constraints = {item.lower() for item in constraints}

    excluded_raw = data.get("excluded_labels", data.get("exclude", []))
    if not isinstance(excluded_raw, list):
        excluded_raw = []
    excluded_labels = [str(item).strip() for item in excluded_raw if str(item).strip()]

    if not target_label and target_hint.lower() in normalized_constraints:
        target_hint = default_target_hint

    confidence_raw = data.get("confidence", data.get("score", 0.0))
    try:
        confidence = float(confidence_raw)
    except (TypeError, ValueError):
        confidence = 0.0
    confidence = max(0.0, min(1.0, confidence))

    need_human_confirm = bool(
        data.get("need_human_confirm", not target_hint or confidence < 0.5)
    )
    reason = str(data.get("reason") or "").strip()

    return {
        "task": task,
        "target_label": target_label,
        "target_hint": target_hint,
        "constraints": constraints,
        "excluded_labels": excluded_labels,
        "confidence": confidence,
        "need_human_confirm": need_human_confirm,
        "reason": reason,
        "prompt_text": prompt_text,
        "raw_json": raw_json,
    }


class QwenSemanticNode(Node):
    def __init__(self) -> None:
        super().__init__("qwen_semantic_node")

        self.declare_parameter("prompt_topic", "/qwen/user_prompt")
        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("semantic_result_topic", "/qwen/semantic_result")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("enabled", True)
        self.declare_parameter("use_visual_context", False)
        self.declare_parameter("auto_infer_on_start", True)
        self.declare_parameter("model_endpoint", "http://127.0.0.1:8000/v1/chat/completions")
        self.declare_parameter("model_name", "Qwen/Qwen2.5-VL-3B-Instruct-AWQ")
        self.declare_parameter("api_key", "EMPTY")
        self.declare_parameter("request_timeout_sec", 20.0)
        self.declare_parameter("max_inference_rate_hz", 0.5)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("max_tokens", 256)
        self.declare_parameter("resize_max_side_px", 1024)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("default_task", "pick")
        self.declare_parameter("default_target_hint", "blue cylinder")
        self.declare_parameter("default_constraints", ["parallel_gripper"])
        self.declare_parameter(
            "default_prompt",
            "Understand the user's task intent for robotic picking and return structured semantics.",
        )
        self.declare_parameter("log_interval_sec", 10.0)

        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.semantic_result_topic = str(self.get_parameter("semantic_result_topic").value)
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.use_visual_context = bool(self.get_parameter("use_visual_context").value)
        self.auto_infer_on_start = bool(self.get_parameter("auto_infer_on_start").value)
        self.model_endpoint, self.model_name, self.api_key = _resolve_runtime_model_config(
            endpoint=self.get_parameter("model_endpoint").value,
            model_name=self.get_parameter("model_name").value,
            api_key=self.get_parameter("api_key").value,
        )
        self.request_timeout_sec = max(
            1.0, float(self.get_parameter("request_timeout_sec").value)
        )
        self.max_inference_rate_hz = max(
            0.1, float(self.get_parameter("max_inference_rate_hz").value)
        )
        self.temperature = max(0.0, float(self.get_parameter("temperature").value))
        self.max_tokens = max(64, int(self.get_parameter("max_tokens").value))
        self.resize_max_side_px = max(256, int(self.get_parameter("resize_max_side_px").value))
        self.jpeg_quality = max(50, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.default_task = str(self.get_parameter("default_task").value).strip() or "pick"
        self.default_target_hint = (
            str(self.get_parameter("default_target_hint").value).strip() or "target object"
        )
        self.default_constraints = [
            str(item).strip()
            for item in list(self.get_parameter("default_constraints").value)
            if str(item).strip()
        ]
        self.default_prompt = str(self.get_parameter("default_prompt").value).strip()
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._session = requests.Session()
        self._prompt_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._latest_color_lock = threading.Lock()
        self._prompt_override = ""
        self._pending_request = bool(self.auto_infer_on_start)
        self._request_in_flight = False
        self._latest_color_msg: Optional[Image] = None
        self._last_terminal_summary = ""

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

        if self.prompt_topic:
            self.create_subscription(String, self.prompt_topic, self._on_prompt, qos_reliable)
        if self.use_visual_context and self.color_topic:
            self.create_subscription(Image, self.color_topic, self._on_color_image, qos_sensor)

        self.semantic_result_pub = self.create_publisher(
            String, self.semantic_result_topic, qos_reliable
        )
        self.semantic_task_pub = self.create_publisher(
            SemanticTask, self.semantic_task_topic, qos_reliable
        )
        self.create_timer(1.0 / self.max_inference_rate_hz, self._maybe_run_inference)

        self.get_logger().info(
            "qwen_semantic_node started: "
            f"prompt={self.prompt_topic or '<disabled>'}, semantic_task={self.semantic_task_topic}, "
            f"use_visual_context={self.use_visual_context}, endpoint={self.model_endpoint}"
        )

    def _on_prompt(self, msg: String) -> None:
        with self._prompt_lock:
            self._prompt_override = str(msg.data or "").strip()
            self._pending_request = True

    def _on_color_image(self, msg: Image) -> None:
        with self._latest_color_lock:
            self._latest_color_msg = msg

    def _maybe_run_inference(self) -> None:
        if not self.enabled:
            return

        with self._request_lock:
            if self._request_in_flight or not self._pending_request:
                return
            self._request_in_flight = True
            self._pending_request = False

        with self._prompt_lock:
            prompt_override = self._prompt_override
        with self._latest_color_lock:
            color_msg = self._latest_color_msg

        worker = threading.Thread(
            target=self._run_inference,
            args=(prompt_override, color_msg),
            daemon=True,
        )
        worker.start()

    def _run_inference(self, prompt_override: str, color_msg: Optional[Image]) -> None:
        prompt_text = self._build_prompt_text(prompt_override)
        try:
            if self._should_use_config_defaults(prompt_override, color_msg):
                result = self._build_default_result(
                    prompt_text=prompt_text,
                    reason="using configured default semantic task",
                )
            else:
                raw_text = self._query_model(prompt_text, color_msg)
                parsed = extract_first_json_object(raw_text)
                result = normalize_semantic_result(
                    parsed,
                    default_task=self.default_task,
                    default_target_hint=self.default_target_hint,
                    default_constraints=self.default_constraints,
                    prompt_text=prompt_text,
                    raw_json=raw_text,
                )
            self._publish_result(result)
            self._maybe_log_result(result)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"qwen semantic inference failed: {exc}")
            fallback = self._build_default_result(
                prompt_text=prompt_text,
                reason=f"fallback to configured defaults after inference failure: {exc}",
            )
            self._publish_result(fallback)
            self._maybe_log_result(fallback)
        finally:
            with self._request_lock:
                self._request_in_flight = False

    def _should_use_config_defaults(
        self,
        prompt_override: str,
        color_msg: Optional[Image],
    ) -> bool:
        if prompt_override.strip():
            return False
        if self.use_visual_context and color_msg is not None:
            return False
        return True

    def _build_default_result(self, *, prompt_text: str, reason: str) -> dict[str, Any]:
        return {
            "task": self.default_task,
            "target_label": "",
            "target_hint": self.default_target_hint,
            "constraints": list(self.default_constraints),
            "excluded_labels": [],
            "confidence": 1.0,
            "need_human_confirm": False,
            "reason": reason,
            "prompt_text": prompt_text,
            "raw_json": "",
        }

    def _build_prompt_text(self, prompt_override: str) -> str:
        instructions = []
        if self.default_prompt:
            instructions.append(self.default_prompt)
        instructions.append(f"Default task: {self.default_task}.")
        if self.default_target_hint:
            instructions.append(f"Default target hint: {self.default_target_hint}.")
        if self.default_constraints:
            instructions.append(
                "Default constraints: " + ", ".join(self.default_constraints) + "."
            )
        if prompt_override:
            instructions.append(f"Operator instruction: {prompt_override}")
        instructions.append("Return strict JSON only with the required keys.")
        return " ".join(instructions).strip()

    def _query_model(self, prompt_text: str, color_msg: Optional[Image]) -> str:
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"

        user_content: list[dict[str, Any]] = [{"type": "text", "text": prompt_text}]
        if self.use_visual_context and color_msg is not None:
            image_rgb = decode_color_image(color_msg)
            if image_rgb is None:
                raise ValueError(
                    f"unsupported or invalid color image encoding: {color_msg.encoding}"
                )
            max_side = max(image_rgb.shape[:2])
            if max_side > self.resize_max_side_px:
                scale = float(self.resize_max_side_px) / float(max_side)
                new_width = max(1, int(round(image_rgb.shape[1] * scale)))
                new_height = max(1, int(round(image_rgb.shape[0] * scale)))
                import cv2

                image_rgb = cv2.resize(
                    image_rgb,
                    (new_width, new_height),
                    interpolation=cv2.INTER_AREA,
                )
            image_b64 = encode_image_to_base64_jpeg(image_rgb, self.jpeg_quality)
            user_content.append(
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                }
            )

        payload = {
            "model": self.model_name,
            "messages": [
                {
                    "role": "system",
                    "content": [{"type": "text", "text": DEFAULT_SYSTEM_PROMPT}],
                },
                {
                    "role": "user",
                    "content": user_content,
                },
            ],
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
        }

        response = self._session.post(
            self.model_endpoint,
            headers=headers,
            json=payload,
            timeout=self.request_timeout_sec,
        )
        response.raise_for_status()
        return extract_message_text(response.json())

    def _publish_result(self, payload: dict[str, Any]) -> None:
        raw_msg = String()
        raw_msg.data = compact_json(payload)
        self.semantic_result_pub.publish(raw_msg)

        task_msg = SemanticTask()
        task_msg.header.stamp = self.get_clock().now().to_msg()
        task_msg.task = str(payload.get("task", ""))
        task_msg.target_label = str(payload.get("target_label", ""))
        task_msg.target_hint = str(payload.get("target_hint", ""))
        task_msg.constraints = [str(item) for item in payload.get("constraints", [])]
        task_msg.excluded_labels = [
            str(item) for item in payload.get("excluded_labels", [])
        ]
        task_msg.confidence = float(payload.get("confidence", 0.0))
        task_msg.need_human_confirm = bool(payload.get("need_human_confirm", False))
        task_msg.reason = str(payload.get("reason", ""))
        task_msg.prompt_text = str(payload.get("prompt_text", ""))
        task_msg.raw_json = str(payload.get("raw_json", ""))
        self.semantic_task_pub.publish(task_msg)

    def _maybe_log_result(self, payload: dict[str, Any]) -> None:
        summary = (
            "semantic decision: "
            f"task={payload.get('task')} "
            f"target_hint={payload.get('target_hint')} "
            f"need_confirm={payload.get('need_human_confirm')}"
        )
        if summary == self._last_terminal_summary:
            return
        self._last_terminal_summary = summary
        self.get_logger().info(summary)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QwenSemanticNode()
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
