from __future__ import annotations

import math
import random
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tactile_interfaces.msg import SystemHealth, TactileRaw

from .legacy_runtime import ensure_legacy_import_paths


class TactileSensorNode(Node):
    def __init__(self) -> None:
        super().__init__("tactile_sensor_node")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("rows", 3)
        self.declare_parameter("cols", 3)
        self.declare_parameter("use_simulation", True)
        self.declare_parameter("sensor_port", "SIM_SENSOR")
        self.declare_parameter("sensor_baudrate", 115200)
        self.declare_parameter("sensor_timeout", 0.1)
        self.declare_parameter("tactile_topic", "/tactile/raw")
        self.declare_parameter("health_topic", "/system/health")
        self.declare_parameter("reconnect_interval_sec", 2.0)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.rows = int(self.get_parameter("rows").value)
        self.cols = int(self.get_parameter("cols").value)
        self.use_simulation = bool(self.get_parameter("use_simulation").value)
        self.sensor_port = str(self.get_parameter("sensor_port").value)
        self.sensor_baudrate = int(self.get_parameter("sensor_baudrate").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.tactile_topic = str(self.get_parameter("tactile_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.reconnect_interval_sec = float(self.get_parameter("reconnect_interval_sec").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        health_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.tactile_pub = self.create_publisher(TactileRaw, self.tactile_topic, sensor_qos)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, health_qos)

        self._project_root = ensure_legacy_import_paths()
        self._hardware = None
        self._connect_error = ""
        self._last_connect_try = 0.0
        self._last_no_data_warn_ts = 0.0

        self._sequence_id = 0
        self._phase = 0.0

        self._try_connect_legacy_sensor(initial=True)

        publish_period = max(0.001, 1.0 / max(1.0, self.publish_rate_hz))
        self.create_timer(publish_period, self._publish_tactile)
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            "tactile_sensor_node started: "
            f"topic={self.tactile_topic}, mode={'simulation' if self.use_simulation else 'hardware'}"
        )

    def _try_connect_legacy_sensor(self, initial: bool = False) -> bool:
        if self._hardware is not None:
            return True

        if self.use_simulation and self.sensor_port.strip():
            effective_port = "SIM_SENSOR"
        else:
            effective_port = self.sensor_port.strip()

        if self._project_root is None:
            self._connect_error = (
                "legacy project root not found; set PROGRAMME_PROJECT_ROOT to enable legacy import"
            )
            if initial:
                self.get_logger().warn(self._connect_error)
            return False

        try:
            from config.demo_config import DemoConfig
            from src.core.hardware_interface import HardwareInterface

            cfg = DemoConfig()
            cfg.hardware.sensor.port = effective_port
            cfg.hardware.sensor.baudrate = self.sensor_baudrate
            cfg.hardware.sensor.timeout = self.sensor_timeout
            cfg.hardware.sensor.rows = self.rows
            cfg.hardware.sensor.cols = self.cols
            cfg.hardware.sensor.num_taxels = max(1, self.rows * self.cols)

            hardware = HardwareInterface(cfg.hardware)
            if not hardware.connect_sensor():
                self._connect_error = f"connect_sensor failed on port={effective_port}"
                self.get_logger().warn(self._connect_error)
                return False

            self._hardware = hardware
            self._connect_error = ""
            self.get_logger().info(f"sensor connected via legacy HardwareInterface (port={effective_port})")
            return True
        except Exception as exc:
            self._connect_error = f"legacy sensor init failed: {exc}"
            self.get_logger().warn(self._connect_error)
            self._hardware = None
            return False

    def _build_from_legacy(self):
        if self._hardware is None:
            return None
        try:
            return self._hardware.read_sensor()
        except Exception as exc:
            self._connect_error = f"read_sensor failed: {exc}"
            self.get_logger().warn(self._connect_error)
            self._disconnect_legacy_sensor()
            return None

    def _disconnect_legacy_sensor(self) -> None:
        if self._hardware is None:
            return
        try:
            self._hardware.disconnect_sensor()
        except Exception:
            pass
        self._hardware = None

    def _publish_tactile(self) -> None:
        reading = self._build_from_legacy()
        if reading is not None:
            msg = self._to_tactile_msg_from_reading(reading)
            self.tactile_pub.publish(msg)
            return

        # Reconnect attempts for hardware mode.
        if not self.use_simulation:
            now = time.time()
            if now - self._last_connect_try >= max(0.5, self.reconnect_interval_sec):
                self._last_connect_try = now
                self._try_connect_legacy_sensor()
            if now - self._last_no_data_warn_ts >= 2.0:
                self._last_no_data_warn_ts = now
                self.get_logger().warn(
                    "sensor hardware unavailable; skip tactile publish (simulation disabled)"
                )
            return

        msg = self._to_tactile_msg_from_sim()
        self.tactile_pub.publish(msg)

    def _to_tactile_msg_from_reading(self, reading) -> TactileRaw:
        forces = [float(v) for v in list(getattr(reading, "force_data", []) or [])]
        vectors = list(getattr(reading, "force_vectors", []) or [])
        rows, cols = self._infer_grid_shape(len(forces))

        fx: List[float] = []
        fy: List[float] = []
        fz: List[float] = []

        if vectors and len(vectors) == len(forces):
            for vec in vectors:
                if isinstance(vec, (list, tuple)) and len(vec) >= 3:
                    fx.append(float(vec[0]))
                    fy.append(float(vec[1]))
                    fz.append(float(vec[2]))
                else:
                    fx.append(0.0)
                    fy.append(0.0)
                    fz.append(0.0)
        else:
            fx = [0.0] * len(forces)
            fy = [0.0] * len(forces)
            fz = list(forces)

        msg = TactileRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tactile_sensor"
        msg.rows = rows
        msg.cols = cols
        msg.sequence_id = self._sequence_id
        msg.frame_id = "taxel_grid"
        msg.forces = forces
        msg.forces_fx = fx
        msg.forces_fy = fy
        msg.forces_fz = fz
        msg.torques_tx = [0.0] * len(forces)
        msg.torques_ty = [0.0] * len(forces)
        msg.torques_tz = [0.0] * len(forces)

        self._sequence_id += 1
        return msg

    def _to_tactile_msg_from_sim(self) -> TactileRaw:
        count = max(1, self.rows * self.cols)
        forces, fx, fy, fz = self._generate_simulated_forces(count)

        msg = TactileRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tactile_sensor"
        msg.rows = max(1, self.rows)
        msg.cols = max(1, self.cols)
        msg.sequence_id = self._sequence_id
        msg.frame_id = "taxel_grid"
        msg.forces = forces
        msg.forces_fx = fx
        msg.forces_fy = fy
        msg.forces_fz = fz
        msg.torques_tx = [0.0] * len(forces)
        msg.torques_ty = [0.0] * len(forces)
        msg.torques_tz = [0.0] * len(forces)

        self._sequence_id += 1
        self._phase += 0.08
        return msg

    def _generate_simulated_forces(
        self, count: int
    ) -> Tuple[List[float], List[float], List[float], List[float]]:
        forces: List[float] = []
        fx: List[float] = []
        fy: List[float] = []
        fz: List[float] = []

        cols = max(1, self.cols)
        rows = max(1, self.rows)
        for idx in range(count):
            row = idx // cols
            col = idx % cols
            x = (col - (cols - 1) / 2.0) / max(1.0, cols - 1)
            y = (row - (rows - 1) / 2.0) / max(1.0, rows - 1)

            radial = math.sqrt(x * x + y * y)
            base = 2.5 + 1.6 * math.sin(self._phase - 2.1 * radial)
            noise = random.gauss(0.0, 0.06)
            force_z = max(0.0, base + noise)
            force_x = 0.12 * math.sin(self._phase + x * 3.0) + random.gauss(0.0, 0.02)
            force_y = 0.12 * math.cos(self._phase + y * 3.0) + random.gauss(0.0, 0.02)

            fz.append(float(force_z))
            fx.append(float(force_x))
            fy.append(float(force_y))
            forces.append(float(force_z))

        return forces, fx, fy, fz

    def _infer_grid_shape(self, value_count: int) -> Tuple[int, int]:
        if self.rows > 0 and self.cols > 0 and self.rows * self.cols == value_count:
            return self.rows, self.cols
        side = int(math.sqrt(max(1, value_count)))
        if side * side == value_count:
            return side, side
        return 1, max(1, value_count)

    def _publish_health(self) -> None:
        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0

        if self._hardware is not None:
            msg.healthy = True
            msg.level = 0
            msg.message = "sensor stream active from legacy hardware interface"
        elif self.use_simulation:
            msg.healthy = True
            msg.level = 0
            msg.message = "sensor stream active in simulation fallback mode"
        else:
            msg.healthy = False
            msg.level = 2
            msg.message = self._connect_error or "sensor unavailable"

        self.health_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._disconnect_legacy_sensor()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Launch shutdown may already stop the context on SIGINT.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
