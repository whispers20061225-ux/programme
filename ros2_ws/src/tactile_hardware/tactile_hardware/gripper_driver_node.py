from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tactile_interfaces.msg import GripperState, SystemHealth
from tactile_interfaces.srv import SetGripperForce

from .legacy_runtime import ensure_legacy_import_paths


_STATUS_CODE = {
    "idle": 0,
    "connecting": 1,
    "connected": 2,
    "calibrating": 3,
    "moving": 4,
    "holding": 5,
    "error": 6,
    "emergency_stop": 7,
}


class GripperDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_driver_node")

        self.declare_parameter("state_rate_hz", 20.0)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("use_simulation", True)
        self.declare_parameter("servo_port", "SIM_GRIPPER")
        self.declare_parameter("servo_baudrate", 115200)
        self.declare_parameter("servo_timeout", 1.0)
        self.declare_parameter("servo_min_angle", 0)
        self.declare_parameter("servo_max_angle", 180)
        self.declare_parameter("servo_home_position", 90)
        self.declare_parameter("servo_speed", 50)
        self.declare_parameter("servo_max_force", 10.0)
        self.declare_parameter("state_topic", "/gripper/state")
        self.declare_parameter("health_topic", "/system/health")

        self.state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.use_simulation = bool(self.get_parameter("use_simulation").value)
        self.servo_port = str(self.get_parameter("servo_port").value)
        self.servo_baudrate = int(self.get_parameter("servo_baudrate").value)
        self.servo_timeout = float(self.get_parameter("servo_timeout").value)
        self.servo_min_angle = int(self.get_parameter("servo_min_angle").value)
        self.servo_max_angle = int(self.get_parameter("servo_max_angle").value)
        self.servo_home_position = int(self.get_parameter("servo_home_position").value)
        self.servo_speed = int(self.get_parameter("servo_speed").value)
        self.servo_max_force = float(self.get_parameter("servo_max_force").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        health_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.state_pub = self.create_publisher(GripperState, self.state_topic, state_qos)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, health_qos)
        self.set_force_srv = self.create_service(
            SetGripperForce, "/gripper/set_force", self._on_set_force
        )

        self._project_root = ensure_legacy_import_paths()
        self._hardware = None
        self._gripper = None
        self._last_error = ""

        if self.auto_enable:
            ok, message = self._enable_gripper()
            if not ok:
                self.get_logger().warn(f"auto enable failed: {message}")

        period = max(0.01, 1.0 / max(1.0, self.state_rate_hz))
        self.create_timer(period, self._publish_state)
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            "gripper_driver_node started: "
            f"state_topic={self.state_topic}, auto_enable={self.auto_enable}"
        )

    @property
    def connected(self) -> bool:
        return self._gripper is not None and self._gripper.is_connected()

    def _enable_gripper(self) -> tuple[bool, str]:
        if self.connected:
            return True, "gripper already enabled"

        if self._project_root is None:
            self._last_error = (
                "legacy project root not found; set PROGRAMME_PROJECT_ROOT to enable gripper driver"
            )
            return False, self._last_error

        try:
            from config.demo_config import DemoConfig
            from core.hardware_interface import HardwareInterface
            from servo_control.gripper_controller import GripperController

            cfg = DemoConfig()
            cfg.hardware.servo.port = "SIM_GRIPPER" if self.use_simulation else self.servo_port
            cfg.hardware.servo.baudrate = self.servo_baudrate
            cfg.hardware.servo.timeout = self.servo_timeout
            cfg.hardware.servo.min_angle = self.servo_min_angle
            cfg.hardware.servo.max_angle = self.servo_max_angle
            cfg.hardware.servo.home_position = self.servo_home_position
            cfg.hardware.servo.speed = self.servo_speed
            cfg.hardware.servo.max_force = self.servo_max_force

            hardware = HardwareInterface(cfg.hardware)
            gripper = GripperController(cfg.hardware.servo, hardware)
            if not gripper.connect():
                self._last_error = f"gripper connect failed on port={cfg.hardware.servo.port}"
                return False, self._last_error

            self._hardware = hardware
            self._gripper = gripper
            self._last_error = ""
            return True, "gripper enabled"
        except Exception as exc:
            self._hardware = None
            self._gripper = None
            self._last_error = str(exc)
            return False, f"gripper initialization failed: {exc}"

    def _disable_gripper(self) -> tuple[bool, str]:
        try:
            if self._gripper is not None:
                self._gripper.disconnect()
            if self._hardware is not None:
                self._hardware.disconnect_servo()
        except Exception as exc:
            self._last_error = str(exc)
            return False, f"gripper disable failed: {exc}"
        finally:
            self._gripper = None
            self._hardware = None

        return True, "gripper disabled"

    def _on_set_force(
        self, request: SetGripperForce.Request, response: SetGripperForce.Response
    ) -> SetGripperForce.Response:
        if not self.connected:
            ok, message = self._enable_gripper()
            if not ok:
                response.success = False
                response.message = f"gripper is not available: {message}"
                response.applied_force = 0.0
                return response

        timeout_sec = float(request.timeout_sec)
        if timeout_sec <= 0.0:
            timeout_sec = 5.0

        try:
            success = bool(
                self._gripper.set_force(
                    force=float(request.force),
                    block=bool(request.block),
                    wait_timeout=timeout_sec,
                )
            )
            state = self._gripper.get_state()
            response.success = success
            response.message = "set_force succeeded" if success else "set_force failed"
            response.applied_force = float(state.target_force if success else state.force)
            return response
        except Exception as exc:
            self._last_error = str(exc)
            response.success = False
            response.message = f"set_force exception: {exc}"
            response.applied_force = 0.0
            return response

    def _publish_state(self) -> None:
        msg = GripperState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gripper_base"

        if not self.connected:
            msg.connected = False
            msg.moving = False
            msg.position = 0.0
            msg.velocity = 0.0
            msg.force = 0.0
            msg.current = 0.0
            msg.temperature = 0.0
            msg.voltage = 0.0
            msg.load = 0.0
            msg.status_code = _STATUS_CODE["error"] if self._last_error else _STATUS_CODE["idle"]
            msg.status_text = self._last_error or "disconnected"
            msg.target_force = 0.0
            self.state_pub.publish(msg)
            return

        try:
            state = self._gripper.update_state()
            if state is None:
                state = self._gripper.get_state()

            status_text = ""
            if state.status is not None:
                status_text = str(getattr(state.status, "value", state.status))
            status_code = _STATUS_CODE.get(status_text, 0)

            msg.connected = True
            msg.moving = bool(state.moving)
            msg.position = float(state.position)
            msg.velocity = float(state.velocity)
            msg.force = float(state.force)
            msg.current = float(state.current)
            msg.temperature = float(state.temperature)
            msg.voltage = float(state.voltage)
            msg.load = float(state.load)
            msg.status_code = int(status_code)
            msg.status_text = status_text or "unknown"
            msg.target_force = float(state.target_force)
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)
            msg.connected = False
            msg.moving = False
            msg.position = 0.0
            msg.velocity = 0.0
            msg.force = 0.0
            msg.current = 0.0
            msg.temperature = 0.0
            msg.voltage = 0.0
            msg.load = 0.0
            msg.status_code = _STATUS_CODE["error"]
            msg.status_text = f"state update failed: {exc}"
            msg.target_force = 0.0

        self.state_pub.publish(msg)

    def _publish_health(self) -> None:
        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0

        if self.connected:
            msg.healthy = True
            msg.level = 0
            msg.message = "gripper driver connected"
        else:
            msg.healthy = False
            msg.level = 1
            msg.message = self._last_error or "gripper driver disconnected"

        self.health_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._disable_gripper()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Give internal control thread a brief chance to stop cleanly.
        time.sleep(0.05)
        rclpy.shutdown()


if __name__ == "__main__":
    main()

