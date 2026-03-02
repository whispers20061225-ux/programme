from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger

from tactile_interfaces.msg import ArmState, SystemHealth
from tactile_interfaces.srv import MoveArmJoint, MoveArmJoints

from .legacy_runtime import ensure_legacy_import_paths


class ArmDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_driver_node")

        self.declare_parameter("state_rate_hz", 20.0)
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("arm_serial_port", "COM5")
        self.declare_parameter("arm_baudrate", 115200)
        self.declare_parameter("arm_timeout", 1.0)
        self.declare_parameter("arm_startup_delay", 1.0)
        self.declare_parameter("home_duration_ms", 1500)
        self.declare_parameter("home_joint_angles", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("arm_num_joints", 6)
        self.declare_parameter("command_default_duration_ms", 1000)
        self.declare_parameter("enforce_angle_limits", True)
        self.declare_parameter("min_angle_deg", -180.0)
        self.declare_parameter("max_angle_deg", 270.0)
        self.declare_parameter("state_topic", "/arm/state")
        self.declare_parameter("health_topic", "/system/health")

        self.state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.arm_serial_port = str(self.get_parameter("arm_serial_port").value)
        self.arm_baudrate = int(self.get_parameter("arm_baudrate").value)
        self.arm_timeout = float(self.get_parameter("arm_timeout").value)
        self.arm_startup_delay = float(self.get_parameter("arm_startup_delay").value)
        self.home_duration_ms = int(self.get_parameter("home_duration_ms").value)
        self.arm_num_joints = int(self.get_parameter("arm_num_joints").value)
        self.command_default_duration_ms = int(
            self.get_parameter("command_default_duration_ms").value
        )
        self.enforce_angle_limits = bool(self.get_parameter("enforce_angle_limits").value)
        self.min_angle_deg = float(self.get_parameter("min_angle_deg").value)
        self.max_angle_deg = float(self.get_parameter("max_angle_deg").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)

        if self.arm_num_joints <= 0:
            self.arm_num_joints = 6
        if self.min_angle_deg > self.max_angle_deg:
            self.get_logger().warn(
                "min_angle_deg > max_angle_deg, swapping values for safety"
            )
            self.min_angle_deg, self.max_angle_deg = self.max_angle_deg, self.min_angle_deg

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

        self.state_pub = self.create_publisher(ArmState, self.state_topic, state_qos)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, health_qos)

        self.enable_srv = self.create_service(SetBool, "/arm/enable", self._on_enable)
        self.home_srv = self.create_service(Trigger, "/arm/home", self._on_home)
        self.move_joint_srv = self.create_service(
            MoveArmJoint, "/arm/move_joint", self._on_move_joint
        )
        self.move_joints_srv = self.create_service(
            MoveArmJoints, "/arm/move_joints", self._on_move_joints
        )

        self._project_root = ensure_legacy_import_paths()
        self._arm = None
        self._last_error = ""

        if self.auto_enable:
            ok, message = self._enable_arm()
            if not ok:
                self.get_logger().warn(f"auto enable failed: {message}")

        period = max(0.01, 1.0 / max(1.0, self.state_rate_hz))
        self.create_timer(period, self._publish_state)
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            "arm_driver_node started: "
            f"state_topic={self.state_topic}, auto_enable={self.auto_enable}, "
            f"num_joints={self.arm_num_joints}"
        )

    @property
    def connected(self) -> bool:
        return bool(self._arm and self._arm.is_connected())

    def _enable_arm(self) -> tuple[bool, str]:
        if self.connected:
            return True, "arm already enabled"
        if self._project_root is None:
            self._last_error = (
                "legacy project root not found; set PROGRAMME_PROJECT_ROOT to enable arm driver"
            )
            return False, self._last_error

        try:
            from arm_control.learm_interface import LearmInterface
            from config.demo_config import DemoConfig

            cfg = DemoConfig()
            cfg.learm_arm = {
                "CONNECTION": {
                    "serial_port": self.arm_serial_port,
                    "baud_rate": self.arm_baudrate,
                    "timeout": self.arm_timeout,
                    "startup_delay": self.arm_startup_delay,
                }
            }
            arm = LearmInterface(cfg)
            if not arm.connect():
                self._last_error = f"arm connect failed on port={self.arm_serial_port}"
                self._arm = None
                return False, self._last_error

            self._arm = arm
            self._last_error = ""
            return True, "arm enabled"
        except Exception as exc:
            self._arm = None
            self._last_error = str(exc)
            return False, f"arm initialization failed: {exc}"

    def _disable_arm(self) -> tuple[bool, str]:
        if self._arm is None:
            return True, "arm already disabled"
        try:
            self._arm.disconnect()
            self._arm = None
            return True, "arm disabled"
        except Exception as exc:
            self._last_error = str(exc)
            return False, f"arm disable failed: {exc}"

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            ok, message = self._enable_arm()
        else:
            ok, message = self._disable_arm()
        response.success = ok
        response.message = message
        return response

    def _on_home(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self.connected:
            response.success = False
            response.message = "arm is not enabled"
            return response

        try:
            home_joint_angles = list(self.get_parameter("home_joint_angles").value)
            if not home_joint_angles:
                response.success = False
                response.message = "home_joint_angles is empty"
                return response

            positions: Dict[int, float] = {
                idx + 1: float(angle) for idx, angle in enumerate(home_joint_angles)
            }
            ok = bool(
                self._arm.move_joints(
                    positions=positions,
                    duration=max(0, self.home_duration_ms),
                    wait=True,
                )
            )
            response.success = ok
            response.message = "arm homed" if ok else "arm home command failed"
            return response
        except Exception as exc:
            self._last_error = str(exc)
            response.success = False
            response.message = f"arm home failed: {exc}"
            return response

    def _validate_joint_id(self, joint_id: int) -> Tuple[bool, str]:
        if joint_id < 1 or joint_id > self.arm_num_joints:
            return (
                False,
                f"joint_id out of range: {joint_id} (valid 1..{self.arm_num_joints})",
            )
        return True, ""

    def _validate_angle(self, angle_deg: float) -> Tuple[bool, str]:
        if not math.isfinite(angle_deg):
            return False, "angle_deg is not finite"
        if self.enforce_angle_limits and (
            angle_deg < self.min_angle_deg or angle_deg > self.max_angle_deg
        ):
            return (
                False,
                f"angle_deg out of range: {angle_deg} "
                f"(valid {self.min_angle_deg}..{self.max_angle_deg})",
            )
        return True, ""

    def _resolve_duration(self, duration_ms: int) -> Optional[int]:
        if duration_ms > 0:
            return int(duration_ms)
        if self.command_default_duration_ms > 0:
            return int(self.command_default_duration_ms)
        return None

    def _on_move_joint(
        self, request: MoveArmJoint.Request, response: MoveArmJoint.Response
    ) -> MoveArmJoint.Response:
        response.joint_id = int(request.joint_id)
        response.commanded_angle_deg = float(request.angle_deg)

        if not self.connected:
            response.success = False
            response.message = "arm is not enabled"
            return response

        joint_id = int(request.joint_id)
        ok, message = self._validate_joint_id(joint_id)
        if not ok:
            response.success = False
            response.message = message
            return response

        angle_deg = float(request.angle_deg)
        ok, message = self._validate_angle(angle_deg)
        if not ok:
            response.success = False
            response.message = message
            return response

        duration = self._resolve_duration(int(request.duration_ms))
        wait = bool(request.wait)

        try:
            success = bool(
                self._arm.move_joint(
                    joint_id,
                    angle_deg,
                    duration=duration,
                    wait=wait,
                )
            )
            response.success = success
            response.message = (
                f"joint {joint_id} moved to {angle_deg:.2f} deg"
                if success
                else f"move_joint failed for joint {joint_id}"
            )
            if success:
                self._last_error = ""
            return response
        except Exception as exc:
            self._last_error = str(exc)
            response.success = False
            response.message = f"move_joint exception: {exc}"
            return response

    def _on_move_joints(
        self, request: MoveArmJoints.Request, response: MoveArmJoints.Response
    ) -> MoveArmJoints.Response:
        joint_ids: List[int] = [int(v) for v in list(request.joint_ids)]
        angles_deg: List[float] = [float(v) for v in list(request.angles_deg)]

        response.joint_ids = joint_ids
        response.commanded_angles_deg = angles_deg

        if not self.connected:
            response.success = False
            response.message = "arm is not enabled"
            return response

        if not joint_ids:
            response.success = False
            response.message = "joint_ids is empty"
            return response
        if len(joint_ids) != len(angles_deg):
            response.success = False
            response.message = "joint_ids and angles_deg size mismatch"
            return response
        if len(set(joint_ids)) != len(joint_ids):
            response.success = False
            response.message = "joint_ids contains duplicates"
            return response

        positions: Dict[int, float] = {}
        for joint_id, angle_deg in zip(joint_ids, angles_deg):
            ok, message = self._validate_joint_id(joint_id)
            if not ok:
                response.success = False
                response.message = message
                return response
            ok, message = self._validate_angle(angle_deg)
            if not ok:
                response.success = False
                response.message = message
                return response
            positions[joint_id] = angle_deg

        duration = self._resolve_duration(int(request.duration_ms))
        wait = bool(request.wait)

        try:
            success = bool(self._arm.move_joints(positions=positions, duration=duration, wait=wait))
            response.success = success
            response.message = (
                f"moved {len(positions)} joints"
                if success
                else "move_joints failed"
            )
            if success:
                self._last_error = ""
            return response
        except Exception as exc:
            self._last_error = str(exc)
            response.success = False
            response.message = f"move_joints exception: {exc}"
            return response

    def _publish_state(self) -> None:
        msg = ArmState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "arm_base"

        if not self.connected:
            msg.connected = False
            msg.moving = False
            msg.error = bool(self._last_error)
            msg.error_message = self._last_error
            msg.battery_voltage = 0.0
            msg.joint_positions = []
            msg.joint_angles = []
            self.state_pub.publish(msg)
            return

        try:
            status = self._arm.update_status()
            msg.connected = bool(status.connected)
            msg.moving = bool(status.moving)
            msg.error = bool(status.error)
            msg.error_message = str(status.error_msg or "")
            msg.battery_voltage = float(status.battery_voltage or 0.0)
            msg.joint_positions = [float(v) for v in list(status.joint_positions or [])]
            msg.joint_angles = [float(v) for v in list(status.joint_angles or [])]
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)
            msg.connected = False
            msg.moving = False
            msg.error = True
            msg.error_message = f"update_status failed: {exc}"
            msg.battery_voltage = 0.0
            msg.joint_positions = []
            msg.joint_angles = []

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
            msg.message = "arm driver connected"
        else:
            msg.healthy = False
            msg.level = 1
            msg.message = self._last_error or "arm driver disconnected"

        self.health_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._disable_arm()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmDriverNode()
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
