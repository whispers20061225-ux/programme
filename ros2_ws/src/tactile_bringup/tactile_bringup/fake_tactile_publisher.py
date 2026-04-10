import math
import random
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from tactile_interfaces.msg import SystemHealth, TactileRaw


class FakeTactilePublisher(Node):
    def __init__(self) -> None:
        super().__init__("fake_tactile_publisher")

        self.declare_parameter("rows", 3)
        self.declare_parameter("cols", 3)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("noise_sigma", 0.08)
        self.declare_parameter("pattern", "sine")

        self.rows = int(self.get_parameter("rows").value)
        self.cols = int(self.get_parameter("cols").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.noise_sigma = float(self.get_parameter("noise_sigma").value)
        self.pattern = str(self.get_parameter("pattern").value)

        self._sequence = 0
        self._phase = 0.0

        self.tactile_pub = self.create_publisher(TactileRaw, "/tactile/raw", 10)
        self.health_pub = self.create_publisher(SystemHealth, "/system/health", 10)

        tactile_period = max(0.001, 1.0 / max(1.0, self.publish_rate_hz))
        self.create_timer(tactile_period, self._publish_tactile)
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            "Fake tactile publisher started: "
            f"{self.rows}x{self.cols}, {self.publish_rate_hz:.1f}Hz, pattern={self.pattern}"
        )

    def _publish_tactile(self) -> None:
        forces, fx, fy, fz, tx, ty, tz = self._generate_frame()

        msg = TactileRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tactile_sensor"
        msg.rows = self.rows
        msg.cols = self.cols
        msg.sequence_id = self._sequence
        msg.frame_id = "taxel_grid"
        msg.forces = forces
        msg.forces_fx = fx
        msg.forces_fy = fy
        msg.forces_fz = fz
        msg.torques_tx = tx
        msg.torques_ty = ty
        msg.torques_tz = tz
        self.tactile_pub.publish(msg)

        self._sequence += 1
        self._phase += 0.08

    def _publish_health(self) -> None:
        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.healthy = True
        msg.level = 0
        msg.message = "phase1 fake tactile stream is active"
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0
        self.health_pub.publish(msg)

    def _generate_frame(
        self,
    ) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float], List[float]]:
        count = self.rows * self.cols
        forces: List[float] = []
        fx: List[float] = []
        fy: List[float] = []
        fz: List[float] = []
        tx: List[float] = []
        ty: List[float] = []
        tz: List[float] = []

        for idx in range(count):
            row = idx // self.cols
            col = idx % self.cols
            x = (col - (self.cols - 1) / 2.0) / max(1.0, self.cols - 1)
            y = (row - (self.rows - 1) / 2.0) / max(1.0, self.rows - 1)

            if self.pattern == "random":
                base = 2.0 + random.uniform(-0.8, 0.8)
            elif self.pattern == "ramp":
                base = 1.0 + (self._sequence % 100) / 20.0
            else:
                # Default sine wave pattern centered at middle taxels.
                radial = math.sqrt(x * x + y * y)
                base = 2.8 + 1.5 * math.sin(self._phase - 2.2 * radial)

            n = random.gauss(0.0, self.noise_sigma)
            force_z = max(0.0, base + n)
            force_x = 0.15 * math.sin(self._phase + x * 3.0) + random.gauss(0.0, self.noise_sigma / 3.0)
            force_y = 0.15 * math.cos(self._phase + y * 3.0) + random.gauss(0.0, self.noise_sigma / 3.0)
            torque_x = 0.05 * force_y + random.gauss(0.0, self.noise_sigma / 5.0)
            torque_y = -0.05 * force_x + random.gauss(0.0, self.noise_sigma / 5.0)
            torque_z = 0.02 * (x - y) * force_z + random.gauss(0.0, self.noise_sigma / 5.0)

            fz.append(float(force_z))
            fx.append(float(force_x))
            fy.append(float(force_y))
            tx.append(float(torque_x))
            ty.append(float(torque_y))
            tz.append(float(torque_z))
            forces.append(float(force_z))

        return forces, fx, fy, fz, tx, ty, tz


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeTactilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
