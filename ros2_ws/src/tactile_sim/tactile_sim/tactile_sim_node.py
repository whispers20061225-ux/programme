from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tactile_interfaces.msg import SystemHealth, TactileRaw


class TactileSimNode(Node):
    """Phase 6.2 tactile simulator baseline.

    Publishes synthetic tactile frames and system health so phase-5 runtime can
    run on a full ROS2-only simulation chain before Gazebo contact integration.
    """

    def __init__(self) -> None:
        super().__init__("tactile_sim_node")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("rows", 3)
        self.declare_parameter("cols", 3)
        self.declare_parameter("tactile_topic", "/tactile/raw")
        self.declare_parameter("health_topic", "/system/health")
        self.declare_parameter("frame_id", "tactile_sensor_sim")
        self.declare_parameter("base_force", 0.8)
        self.declare_parameter("pulse_amplitude", 2.2)
        self.declare_parameter("noise_std", 0.03)
        self.declare_parameter("lateral_force_scale", 0.08)
        self.declare_parameter("phase_speed", 0.12)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.rows = max(1, int(self.get_parameter("rows").value))
        self.cols = max(1, int(self.get_parameter("cols").value))
        self.tactile_topic = str(self.get_parameter("tactile_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_force = float(self.get_parameter("base_force").value)
        self.pulse_amplitude = float(self.get_parameter("pulse_amplitude").value)
        self.noise_std = max(0.0, float(self.get_parameter("noise_std").value))
        self.lateral_force_scale = float(self.get_parameter("lateral_force_scale").value)
        self.phase_speed = float(self.get_parameter("phase_speed").value)

        tactile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        health_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tactile_pub = self.create_publisher(TactileRaw, self.tactile_topic, tactile_qos)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, health_qos)

        self._sequence_id = 0
        self._phase = 0.0

        publish_period = max(0.001, 1.0 / max(1.0, self.publish_rate_hz))
        self.create_timer(publish_period, self._publish_tactile)
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            "tactile_sim_node started: "
            f"topic={self.tactile_topic}, grid={self.rows}x{self.cols}, rate={self.publish_rate_hz}Hz"
        )

    def _publish_tactile(self) -> None:
        count = self.rows * self.cols
        forces, fx, fy, fz, tx, ty, tz = self._generate_simulated_forces(count)

        msg = TactileRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.rows = self.rows
        msg.cols = self.cols
        msg.sequence_id = self._sequence_id
        msg.frame_id = "taxel_grid"
        msg.forces = forces
        msg.forces_fx = fx
        msg.forces_fy = fy
        msg.forces_fz = fz
        msg.torques_tx = tx
        msg.torques_ty = ty
        msg.torques_tz = tz

        self._sequence_id += 1
        self._phase += self.phase_speed

        self.tactile_pub.publish(msg)

    def _generate_simulated_forces(self, count: int):
        forces = []
        fx = []
        fy = []
        fz = []
        tx = []
        ty = []
        tz = []

        sigma = max(0.8, min(self.rows, self.cols) * 0.55)
        center_x = (self.cols - 1) * 0.5 + (self.cols - 1) * 0.35 * math.sin(self._phase)
        center_y = (self.rows - 1) * 0.5 + (self.rows - 1) * 0.35 * math.cos(self._phase * 0.8)

        for idx in range(count):
            row = idx // self.cols
            col = idx % self.cols

            dx = float(col) - center_x
            dy = float(row) - center_y
            distance2 = dx * dx + dy * dy

            gaussian_peak = math.exp(-distance2 / (2.0 * sigma * sigma))
            normal_force = (
                self.base_force
                + self.pulse_amplitude * gaussian_peak
                + random.gauss(0.0, self.noise_std)
            )
            normal_force = max(0.0, normal_force)

            tangential_x = (
                -self.lateral_force_scale * dy * gaussian_peak
                + random.gauss(0.0, self.noise_std * 0.5)
            )
            tangential_y = (
                self.lateral_force_scale * dx * gaussian_peak
                + random.gauss(0.0, self.noise_std * 0.5)
            )
            torque_x = 0.06 * tangential_y + random.gauss(0.0, self.noise_std * 0.08)
            torque_y = -0.06 * tangential_x + random.gauss(0.0, self.noise_std * 0.08)
            torque_z = 0.025 * (dx - dy) * gaussian_peak + random.gauss(0.0, self.noise_std * 0.08)

            fz.append(float(normal_force))
            fx.append(float(tangential_x))
            fy.append(float(tangential_y))
            tx.append(float(torque_x))
            ty.append(float(torque_y))
            tz.append(float(torque_z))
            forces.append(float(normal_force))

        return forces, fx, fy, fz, tx, ty, tz

    def _publish_health(self) -> None:
        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.healthy = True
        msg.level = 0
        msg.message = "tactile simulation stream active"
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0
        self.health_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileSimNode()
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
