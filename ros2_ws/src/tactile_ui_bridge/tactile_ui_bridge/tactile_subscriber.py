import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tactile_interfaces.msg import SystemHealth, TactileRaw


@dataclass
class TactileSnapshot:
    stamp_sec: float
    rows: int
    cols: int
    sequence_id: int
    forces: List[float]
    forces_fx: List[float]
    forces_fy: List[float]
    forces_fz: List[float]


class TactileUiSubscriber(Node):
    """Phase 1 UI bridge subscriber.

    This node only reads ROS2 topics and keeps latest tactile data in memory.
    It is intentionally read-only to keep migration risk low in phase 1.
    """

    def __init__(self) -> None:
        super().__init__("tactile_ui_subscriber")

        self.declare_parameter("tactile_topic", "/tactile/raw")
        self.declare_parameter("health_topic", "/system/health")
        self.declare_parameter("report_interval_sec", 2.0)

        tactile_topic = str(self.get_parameter("tactile_topic").value)
        health_topic = str(self.get_parameter("health_topic").value)
        self.report_interval_sec = float(self.get_parameter("report_interval_sec").value)

        self._lock = threading.Lock()
        self._latest: Optional[TactileSnapshot] = None
        self._message_count = 0
        self._last_report = time.time()

        tactile_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(TactileRaw, tactile_topic, self._on_tactile, tactile_qos)
        self.create_subscription(SystemHealth, health_topic, self._on_health, health_qos)

        self.get_logger().info(
            f"UI bridge subscribed: tactile_topic={tactile_topic}, health_topic={health_topic}"
        )

    def _on_tactile(self, msg: TactileRaw) -> None:
        snapshot = TactileSnapshot(
            stamp_sec=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9,
            rows=int(msg.rows),
            cols=int(msg.cols),
            sequence_id=int(msg.sequence_id),
            forces=list(msg.forces),
            forces_fx=list(msg.forces_fx),
            forces_fy=list(msg.forces_fy),
            forces_fz=list(msg.forces_fz),
        )

        with self._lock:
            self._latest = snapshot
            self._message_count += 1

        now = time.time()
        if now - self._last_report >= self.report_interval_sec:
            self._last_report = now
            mean_force = 0.0
            if snapshot.forces:
                mean_force = sum(snapshot.forces) / len(snapshot.forces)
            self.get_logger().info(
                f"cached tactile frame seq={snapshot.sequence_id}, "
                f"grid={snapshot.rows}x{snapshot.cols}, mean_force={mean_force:.3f}"
            )

    def _on_health(self, msg: SystemHealth) -> None:
        if not msg.healthy or msg.level >= 2:
            self.get_logger().warn(
                f"health issue from {msg.node_name}: level={msg.level}, message={msg.message}"
            )

    def get_latest_snapshot(self) -> Optional[TactileSnapshot]:
        with self._lock:
            return self._latest


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileUiSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
