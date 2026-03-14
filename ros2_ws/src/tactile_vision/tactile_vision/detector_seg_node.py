from __future__ import annotations

import json
import threading
import time
from typing import Any, Optional

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tactile_interfaces.msg import SemanticTask

from tactile_vision.modular_common import (
    coerce_bbox,
    coerce_point,
    compact_json,
    decode_color_image,
    encode_image_to_base64_jpeg,
)


def normalize_detector_result(
    data: dict[str, Any],
    *,
    frame_stamp_sec: float,
    image_width: int,
    image_height: int,
    target_hint: str,
    backend: str,
) -> dict[str, Any]:
    target_label = str(
        data.get("target_label") or data.get("target") or data.get("label") or target_hint
    ).strip()
    confidence_raw = data.get("confidence", data.get("score", 0.0))
    try:
        confidence = float(confidence_raw)
    except (TypeError, ValueError):
        confidence = 0.0
    confidence = max(0.0, min(1.0, confidence))

    bbox_xyxy = coerce_bbox(data.get("bbox_xyxy", data.get("bbox")), image_width, image_height)
    point_px = coerce_point(data.get("point_px", data.get("point")), image_width, image_height)
    if point_px is None and bbox_xyxy is not None:
        point_px = [
            int(round((bbox_xyxy[0] + bbox_xyxy[2]) * 0.5)),
            int(round((bbox_xyxy[1] + bbox_xyxy[3]) * 0.5)),
        ]

    accepted = bool(target_label and bbox_xyxy is not None and confidence > 0.0)
    return {
        "status": "ok" if accepted else "no_detection",
        "backend": backend,
        "accepted": accepted,
        "candidate_visible": bool(accepted),
        "task": str(data.get("task") or "pick"),
        "target_label": target_label,
        "point_px": point_px,
        "bbox_xyxy": bbox_xyxy,
        "confidence": round(confidence, 4),
        "need_human_confirm": bool(data.get("need_human_confirm", not accepted)),
        "reason": str(data.get("reason") or ""),
        "frame_stamp_sec": round(float(frame_stamp_sec), 6),
        "image_size": [int(image_width), int(image_height)],
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
        self.declare_parameter("backend", "disabled")
        self.declare_parameter("backend_url", "")
        self.declare_parameter("request_timeout_sec", 10.0)
        self.declare_parameter("max_inference_rate_hz", 2.0)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("resize_max_side_px", 1024)
        self.declare_parameter("enabled", True)
        self.declare_parameter("log_interval_sec", 10.0)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.candidate_visible_topic = str(self.get_parameter("candidate_visible_topic").value)
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.backend_url = str(self.get_parameter("backend_url").value).strip()
        self.request_timeout_sec = max(
            0.5, float(self.get_parameter("request_timeout_sec").value)
        )
        self.max_inference_rate_hz = max(
            0.2, float(self.get_parameter("max_inference_rate_hz").value)
        )
        self.jpeg_quality = max(50, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.resize_max_side_px = max(256, int(self.get_parameter("resize_max_side_px").value))
        self.enabled = bool(self.get_parameter("enabled").value)
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._session = requests.Session()
        self._request_lock = threading.Lock()
        self._semantic_lock = threading.Lock()
        self._color_lock = threading.Lock()
        self._request_in_flight = False
        self._pending_request = False
        self._latest_color_msg: Optional[Image] = None
        self._semantic_task: Optional[SemanticTask] = None
        self._last_log_ts = 0.0

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
            String, self.detection_result_topic, qos_reliable
        )
        self.candidate_visible_pub = self.create_publisher(
            Bool, self.candidate_visible_topic, qos_reliable
        )
        if self.backend == "http_json":
            self.create_timer(1.0 / self.max_inference_rate_hz, self._maybe_run_inference)

        self.get_logger().info(
            "detector_seg_node started: "
            f"backend={self.backend}, result={self.detection_result_topic}, "
            f"candidate_visible={self.candidate_visible_topic}"
        )

    def _on_color_image(self, msg: Image) -> None:
        with self._color_lock:
            self._latest_color_msg = msg
        if self.backend == "http_json":
            self._pending_request = True

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        with self._semantic_lock:
            self._semantic_task = msg
        if self.backend == "disabled":
            self._publish_detection(
                {
                    "status": "backend_not_configured",
                    "backend": "disabled",
                    "accepted": False,
                    "candidate_visible": False,
                    "task": msg.task,
                    "target_label": msg.target_label,
                    "reason": "detector backend disabled; configure YOLOv8-seg service or legacy bridge",
                }
            )
        elif self.backend == "http_json":
            self._pending_request = True

    def _maybe_run_inference(self) -> None:
        if not self.enabled or self.backend != "http_json" or not self._pending_request:
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

        worker = threading.Thread(
            target=self._run_http_inference,
            args=(color_msg, semantic_task),
            daemon=True,
        )
        worker.start()

    def _run_http_inference(
        self,
        color_msg: Optional[Image],
        semantic_task: Optional[SemanticTask],
    ) -> None:
        try:
            if color_msg is None:
                raise ValueError("missing color image for detector inference")
            if not self.backend_url:
                raise ValueError("detector backend_url is empty")

            image_rgb = decode_color_image(color_msg)
            if image_rgb is None:
                raise ValueError(f"unsupported color image encoding: {color_msg.encoding}")

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
            semantic_payload = {
                "task": semantic_task.task if semantic_task is not None else "pick",
                "target_label": semantic_task.target_label if semantic_task is not None else "",
                "target_hint": semantic_task.target_hint if semantic_task is not None else "",
                "constraints": list(semantic_task.constraints) if semantic_task is not None else [],
                "excluded_labels": list(semantic_task.excluded_labels)
                if semantic_task is not None
                else [],
            }

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

            payload = normalize_detector_result(
                parsed,
                frame_stamp_sec=time.time(),
                image_width=int(color_msg.width),
                image_height=int(color_msg.height),
                target_hint=semantic_payload["target_hint"],
                backend="http_json",
            )
            self._publish_detection(payload)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"detector inference failed: {exc}")
            self._publish_detection(
                {
                    "status": "error",
                    "backend": self.backend,
                    "accepted": False,
                    "candidate_visible": False,
                    "reason": str(exc),
                }
            )
        finally:
            with self._request_lock:
                self._request_in_flight = False

    def _publish_detection(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = compact_json(payload)
        self.detection_pub.publish(msg)
        self.candidate_visible_pub.publish(Bool(data=bool(payload.get("candidate_visible", False))))

        now = time.time()
        if now - self._last_log_ts >= self.log_interval_sec:
            self._last_log_ts = now
            self.get_logger().info(
                "detector result: "
                f"status={payload.get('status')} "
                f"label={payload.get('target_label')} "
                f"accepted={payload.get('accepted')} "
                f"backend={payload.get('backend')}"
            )


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
