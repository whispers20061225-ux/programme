from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from tactile_interfaces.msg import SystemHealth


@dataclass
class StreamStats:
    stamp: float = 0.0
    count: int = 0


class RealsenseMonitorNode(Node):
    """Monitors RealSense topics and reports camera health to /system/health."""

    def __init__(self) -> None:
        super().__init__("realsense_monitor_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("health_topic", "/system/health")
        self.declare_parameter("status_rate_hz", 1.0)
        self.declare_parameter("stale_timeout_sec", 1.5)
        self.declare_parameter("require_depth_stream", False)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.status_rate_hz = max(0.2, float(self.get_parameter("status_rate_hz").value))
        self.stale_timeout_sec = max(0.2, float(self.get_parameter("stale_timeout_sec").value))
        self.require_depth_stream = bool(self.get_parameter("require_depth_stream").value)

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_health = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._stats: Dict[str, StreamStats] = {
            "color": StreamStats(),
            "depth": StreamStats(),
            "camera_info": StreamStats(),
        }

        self.create_subscription(Image, self.color_topic, self._on_color, qos_sensor)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_sensor)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_info, qos_sensor)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, qos_health)

        period = 1.0 / self.status_rate_hz
        self.create_timer(period, self._publish_health)

        self.get_logger().info(
            "realsense_monitor_node started: "
            f"color={self.color_topic}, depth={self.depth_topic}, camera_info={self.camera_info_topic}, "
            f"require_depth={self.require_depth_stream}"
        )

    def _touch_stream(self, stream_name: str) -> None:
        stat = self._stats[stream_name]
        stat.stamp = time.time()
        stat.count += 1

    def _on_color(self, _: Image) -> None:
        self._touch_stream("color")

    def _on_depth(self, _: Image) -> None:
        self._touch_stream("depth")

    def _on_info(self, _: CameraInfo) -> None:
        self._touch_stream("camera_info")

    def _publish_health(self) -> None:
        now = time.time()
        color_ok = now - self._stats["color"].stamp <= self.stale_timeout_sec
        info_ok = now - self._stats["camera_info"].stamp <= self.stale_timeout_sec
        depth_ok = now - self._stats["depth"].stamp <= self.stale_timeout_sec

        required_ok = color_ok and info_ok
        if self.require_depth_stream:
            required_ok = required_ok and depth_ok

        details = [
            f"color={'ok' if color_ok else 'stale'}",
            f"camera_info={'ok' if info_ok else 'stale'}",
            f"depth={'ok' if depth_ok else 'stale'}",
            f"counts=color:{self._stats['color'].count},depth:{self._stats['depth'].count},info:{self._stats['camera_info'].count}",
        ]

        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.healthy = bool(required_ok)
        msg.level = 0 if required_ok else 1
        msg.message = "; ".join(details)
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0
        self.health_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealsenseMonitorNode()
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

