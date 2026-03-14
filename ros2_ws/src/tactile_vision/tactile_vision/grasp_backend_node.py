from __future__ import annotations

import threading
import time
from typing import Any, Optional

import numpy as np
import requests
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tactile_interfaces.msg import GraspProposal, GraspProposalArray, SemanticTask

from tactile_vision.modular_common import compact_json


def point_cloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    if int(msg.point_step) < 12 or int(msg.width) <= 0:
        return np.empty((0, 3), dtype=np.float32)
    point_step_floats = max(3, int(msg.point_step) // 4)
    points = np.frombuffer(msg.data, dtype=np.float32)
    if points.size < int(msg.width) * point_step_floats:
        return np.empty((0, 3), dtype=np.float32)
    points = points.reshape((-1, point_step_floats))
    return points[:, :3].astype(np.float32)


def to_point(values: Any) -> Point:
    point = Point()
    if isinstance(values, (list, tuple)) and len(values) >= 3:
        point.x = float(values[0])
        point.y = float(values[1])
        point.z = float(values[2])
    return point


def to_vector3(values: Any) -> Vector3:
    vector = Vector3()
    if isinstance(values, (list, tuple)) and len(values) >= 3:
        vector.x = float(values[0])
        vector.y = float(values[1])
        vector.z = float(values[2])
    return vector


class GraspBackendNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_backend_node")

        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("target_cloud_topic", "/perception/target_cloud")
        self.declare_parameter(
            "candidate_grasp_proposal_array_topic", "/grasp/candidate_grasp_proposals"
        )
        self.declare_parameter("backend", "disabled")
        self.declare_parameter("backend_url", "")
        self.declare_parameter("request_timeout_sec", 10.0)
        self.declare_parameter("max_points", 2048)
        self.declare_parameter("min_points_required", 64)
        self.declare_parameter("publish_empty_on_failure", False)
        self.declare_parameter("default_task_constraint_tag", "pick")
        self.declare_parameter("log_interval_sec", 8.0)

        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.target_cloud_topic = str(self.get_parameter("target_cloud_topic").value)
        self.candidate_grasp_proposal_array_topic = str(
            self.get_parameter("candidate_grasp_proposal_array_topic").value
        )
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.backend_url = str(self.get_parameter("backend_url").value).strip()
        self.request_timeout_sec = max(
            0.5, float(self.get_parameter("request_timeout_sec").value)
        )
        self.max_points = max(64, int(self.get_parameter("max_points").value))
        self.min_points_required = max(
            8, int(self.get_parameter("min_points_required").value)
        )
        self.publish_empty_on_failure = bool(
            self.get_parameter("publish_empty_on_failure").value
        )
        self.default_task_constraint_tag = str(
            self.get_parameter("default_task_constraint_tag").value
        ).strip()
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._session = requests.Session()
        self._semantic_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._semantic_task: Optional[SemanticTask] = None
        self._request_in_flight = False
        self._last_log_ts = 0.0

        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            SemanticTask, self.semantic_task_topic, self._on_semantic_task, qos_reliable
        )
        self.create_subscription(
            PointCloud2, self.target_cloud_topic, self._on_target_cloud, qos_reliable
        )
        self.proposal_pub = self.create_publisher(
            GraspProposalArray, self.candidate_grasp_proposal_array_topic, qos_reliable
        )
        self.debug_pub = self.create_publisher(String, "/grasp/backend_debug", qos_reliable)

        self.get_logger().info(
            "grasp_backend_node started: "
            f"backend={self.backend}, target_cloud={self.target_cloud_topic}, "
            f"proposal_topic={self.candidate_grasp_proposal_array_topic}"
        )

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        with self._semantic_lock:
            self._semantic_task = msg

    def _on_target_cloud(self, msg: PointCloud2) -> None:
        if self.backend == "disabled":
            self._publish_debug(
                {
                    "status": "backend_disabled",
                    "reason": "configure Contact-GraspNet/GPD service endpoint",
                }
            )
            if self.publish_empty_on_failure:
                empty = GraspProposalArray()
                empty.header = msg.header
                self.proposal_pub.publish(empty)
            return

        if self.backend != "http_json":
            self._publish_debug({"status": "unsupported_backend", "backend": self.backend})
            return

        with self._request_lock:
            if self._request_in_flight:
                return
            self._request_in_flight = True

        worker = threading.Thread(target=self._run_backend, args=(msg,), daemon=True)
        worker.start()

    def _run_backend(self, cloud_msg: PointCloud2) -> None:
        try:
            if not self.backend_url:
                raise ValueError("grasp backend_url is empty")

            points = point_cloud2_to_xyz_array(cloud_msg)
            if points.shape[0] < self.min_points_required:
                raise ValueError(
                    f"target cloud has too few points: {points.shape[0]} < {self.min_points_required}"
                )
            if points.shape[0] > self.max_points:
                step = max(1, points.shape[0] // self.max_points)
                points = points[::step][: self.max_points]

            with self._semantic_lock:
                semantic_task = self._semantic_task

            payload = {
                "frame_id": cloud_msg.header.frame_id,
                "points_xyz": points.astype(float).tolist(),
                "task": semantic_task.task if semantic_task is not None else "pick",
                "target_label": semantic_task.target_label if semantic_task is not None else "",
                "target_hint": semantic_task.target_hint if semantic_task is not None else "",
                "constraints": list(semantic_task.constraints) if semantic_task is not None else [],
            }

            response = self._session.post(
                self.backend_url,
                json=payload,
                timeout=self.request_timeout_sec,
            )
            response.raise_for_status()
            parsed = response.json()
            if not isinstance(parsed, dict):
                raise ValueError("grasp backend response is not a JSON object")

            proposals_raw = parsed.get("proposals", [])
            if not isinstance(proposals_raw, list):
                raise ValueError("grasp backend response missing proposals array")

            proposal_array = GraspProposalArray()
            proposal_array.header = cloud_msg.header
            for item in proposals_raw:
                if not isinstance(item, dict):
                    continue
                proposal = GraspProposal()
                proposal.contact_point_1 = to_point(item.get("contact_point_1"))
                proposal.contact_point_2 = to_point(item.get("contact_point_2"))
                proposal.grasp_center = to_point(item.get("grasp_center"))
                proposal.closing_direction = to_vector3(item.get("closing_direction"))
                proposal.approach_direction = to_vector3(item.get("approach_direction"))
                proposal.grasp_width_m = float(item.get("grasp_width_m", 0.0))
                proposal.confidence_score = float(item.get("confidence_score", 0.0))
                proposal.semantic_score = float(item.get("semantic_score", 0.0))
                proposal.task_constraint_tag = str(
                    item.get("task_constraint_tag") or self.default_task_constraint_tag
                )
                proposal_array.proposals.append(proposal)

            self.proposal_pub.publish(proposal_array)
            self._publish_debug(
                {
                    "status": "ok",
                    "backend": self.backend,
                    "proposal_count": len(proposal_array.proposals),
                }
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"grasp backend inference failed: {exc}")
            self._publish_debug({"status": "error", "backend": self.backend, "reason": str(exc)})
            if self.publish_empty_on_failure:
                empty = GraspProposalArray()
                empty.header = cloud_msg.header
                self.proposal_pub.publish(empty)
        finally:
            with self._request_lock:
                self._request_in_flight = False

    def _publish_debug(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = compact_json(payload)
        self.debug_pub.publish(msg)
        now = time.time()
        if now - self._last_log_ts >= self.log_interval_sec:
            self._last_log_ts = now
            self.get_logger().info(f"grasp backend: {msg.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspBackendNode()
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
