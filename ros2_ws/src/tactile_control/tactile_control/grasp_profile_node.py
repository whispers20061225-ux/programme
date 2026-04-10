from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from tactile_interfaces.msg import SystemHealth, TaskGoal


def _safe_json_loads(raw_text: str) -> dict[str, Any]:
    try:
        parsed = json.loads(raw_text)
    except Exception:
        return {}
    return parsed if isinstance(parsed, dict) else {}


def _clamp01(value: Any, default: float) -> float:
    try:
        numeric = float(value)
    except Exception:
        return float(default)
    return max(0.0, min(1.0, numeric))


def _positive(value: Any, default: float) -> float:
    try:
        numeric = float(value)
    except Exception:
        return float(default)
    return float(default) if numeric <= 0.0 else numeric


class GraspProfileNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_profile_node")

        self.declare_parameter("task_goal_topic", "/task/goal")
        self.declare_parameter("profile_topic", "/control/gripper/profile_json")
        self.declare_parameter("health_topic", "/system/health")

        self.task_goal_topic = str(self.get_parameter("task_goal_topic").value)
        self.profile_topic = str(self.get_parameter("profile_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)

        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.profile_pub = self.create_publisher(String, self.profile_topic, qos_reliable)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, qos_reliable)
        self.create_subscription(TaskGoal, self.task_goal_topic, self._on_task_goal, qos_reliable)
        self.create_timer(1.0, self._publish_health)

        self._last_profile: dict[str, Any] = self._build_profile(TaskGoal())
        self._publish_profile()
        self.get_logger().info(
            f"grasp_profile_node started: task_goal_topic={self.task_goal_topic} profile_topic={self.profile_topic}"
        )

    def _on_task_goal(self, msg: TaskGoal) -> None:
        self._last_profile = self._build_profile(msg)
        self._publish_profile()

    def _build_profile(self, goal: TaskGoal) -> dict[str, Any]:
        raw_context = str(goal.handoff_context_json or "").strip()
        context = _safe_json_loads(raw_context)
        object_type = str(
            context.get("object_type")
            or context.get("object")
            or goal.target_label
            or goal.target_hint
            or ""
        ).strip()

        fragility = _clamp01(context.get("fragility"), 0.45)
        surface_friction = _clamp01(
            context.get("surface_friction", context.get("friction")), 0.50
        )
        compliance = _clamp01(context.get("compliance"), 0.45)
        slip_risk = _clamp01(context.get("slip_risk"), max(0.0, 0.8 - surface_friction))
        confidence = _clamp01(context.get("confidence", goal.confidence), 0.50)
        mass_g = _positive(context.get("mass_g"), 80.0)

        preferred_contact_force = _positive(
            context.get("preferred_contact_force_n"),
            (0.8 + min(1.2, mass_g / 250.0)) * (1.15 - 0.45 * fragility) * (1.0 + 0.25 * slip_risk),
        )

        if fragility >= 0.70:
            profile_id = "fragile_soft"
        elif surface_friction <= 0.35:
            profile_id = "slippery_secure"
        elif compliance >= 0.65:
            profile_id = "compliant_gentle"
        else:
            profile_id = "default_pick"

        kp = 0.60 + (1.0 - fragility) * 0.80 + slip_risk * 0.30
        kd = 0.08 + slip_risk * 0.18 + (1.0 - surface_friction) * 0.08
        target_force = preferred_contact_force
        contact_threshold = max(1.0, min(6.0, target_force * 0.35 + compliance * 0.60))
        safety_max = max(target_force + 0.60, target_force * (1.70 - 0.40 * fragility + 0.20 * slip_risk))

        return {
            "profile_id": profile_id,
            "target_label": str(goal.target_label or goal.target_hint or object_type or "").strip(),
            "object_type": object_type,
            "source": "handoff_context_json" if raw_context else "default_rules",
            "kp": round(kp, 3),
            "kd": round(kd, 3),
            "target_force": round(target_force, 2),
            "contact_threshold": round(contact_threshold, 2),
            "safety_max": round(safety_max, 2),
            "confidence": round(confidence, 2),
            "updated_at": time.time(),
            "raw": context,
        }

    def _publish_profile(self) -> None:
        msg = String()
        msg.data = json.dumps(self._last_profile, ensure_ascii=False, separators=(",", ":"))
        self.profile_pub.publish(msg)

    def _publish_health(self) -> None:
        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.healthy = True
        msg.level = 0
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0
        msg.message = f"profile={self._last_profile.get('profile_id', 'unset')}"
        self.health_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspProfileNode()
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
