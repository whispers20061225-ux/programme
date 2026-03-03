import json
import logging
import math
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from PyQt5.QtCore import QObject, QThread, pyqtSignal

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import SetBool, Trigger
    from tactile_interfaces.action import ExecuteDemo
    from tactile_interfaces.msg import ArmState, SystemHealth
    from tactile_interfaces.srv import MoveArmJoint, MoveArmJoints

    ROS2_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - optional at runtime
    rclpy = None
    ActionClient = None
    ExternalShutdownException = Exception
    SingleThreadedExecutor = None
    Node = object
    QoSProfile = None
    HistoryPolicy = None
    ReliabilityPolicy = None
    SetBool = None
    Trigger = None
    ExecuteDemo = None
    ArmState = None
    SystemHealth = None
    MoveArmJoint = None
    MoveArmJoints = None
    ROS2_IMPORT_ERROR = exc


class Ros2ControlThreadStub(QThread):
    """Fallback control stub for read-only ROS2 mode."""

    status_updated = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, dict)

    def __init__(self) -> None:
        super().__init__()
        self._running = False
        self.logger = logging.getLogger(__name__)

    def run(self) -> None:
        self._running = True
        self.status_updated.emit(
            "ros2_mode_ready",
            {"message": "ROS2 monitor mode is active. Control path is disabled."},
        )
        while self._running:
            time.sleep(0.2)

    def stop_control(self) -> None:
        self._running = False

    def send_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> bool:
        self.status_updated.emit(
            "ros2_command_ignored",
            {
                "command": command,
                "params": params or {},
                "message": "Command ignored in ROS2 monitor mode.",
            },
        )
        return True

    def get_status(self) -> Dict[str, Any]:
        return {"arm": {"connected": False, "enabled": False, "homed": False}}


class _Ros2ControlNode(Node):
    """Internal ROS2 node used by Ros2ControlThread."""

    def __init__(
        self,
        owner: "Ros2ControlThread",
        arm_state_topic: str,
        health_topic: str,
        control_enable_service: str,
        control_home_service: str,
        control_move_joint_service: str,
        control_move_joints_service: str,
        control_reset_emergency_service: str,
        execute_demo_action: str,
        pause_demo_service: str,
        resume_demo_service: str,
        stop_demo_service: str,
    ) -> None:
        super().__init__("ros2_control_bridge")
        self._owner = owner

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.create_subscription(ArmState, arm_state_topic, self._on_arm_state, state_qos)
        self.create_subscription(SystemHealth, health_topic, self._on_health, health_qos)

        self.enable_client = self.create_client(SetBool, control_enable_service)
        self.home_client = self.create_client(Trigger, control_home_service)
        self.move_joint_client = self.create_client(MoveArmJoint, control_move_joint_service)
        self.move_joints_client = self.create_client(MoveArmJoints, control_move_joints_service)
        self.reset_emergency_client = self.create_client(Trigger, control_reset_emergency_service)
        self.pause_demo_client = self.create_client(Trigger, pause_demo_service)
        self.resume_demo_client = self.create_client(Trigger, resume_demo_service)
        self.stop_demo_client = self.create_client(Trigger, stop_demo_service)
        self.execute_demo_client = ActionClient(self, ExecuteDemo, execute_demo_action)

    def _on_arm_state(self, msg: ArmState) -> None:
        self._owner.on_arm_state(msg)

    def _on_health(self, msg: SystemHealth) -> None:
        self._owner.on_health(msg)


class Ros2ControlThread(QThread):
    """Phase 5 ROS2 control thread.

    It keeps Qt/UI wiring compatible while forwarding arm commands to
    /control/arm/* services.
    """

    status_updated = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, dict)

    def __init__(
        self,
        arm_state_topic: str = "/arm/state",
        health_topic: str = "/system/health",
        control_enable_service: str = "/control/arm/enable",
        control_home_service: str = "/control/arm/home",
        control_move_joint_service: str = "/control/arm/move_joint",
        control_move_joints_service: str = "/control/arm/move_joints",
        control_reset_emergency_service: str = "/system/reset_emergency",
        execute_demo_action: str = "/task/execute_demo",
        pause_demo_service: str = "/task/pause_demo",
        resume_demo_service: str = "/task/resume_demo",
        stop_demo_service: str = "/task/stop_demo",
        command_timeout_sec: float = 5.0,
        gripper_joint_id: int = 6,
        gripper_position_min: float = 0.0,
        gripper_position_max: float = 100.0,
        gripper_angle_min_deg: float = 0.0,
        gripper_angle_max_deg: float = 100.0,
        gripper_default_speed: float = 50.0,
        gripper_min_duration_ms: int = 180,
        gripper_max_duration_ms: int = 1200,
    ) -> None:
        super().__init__()
        self.logger = logging.getLogger(__name__)

        self.arm_state_topic = arm_state_topic
        self.health_topic = health_topic
        self.control_enable_service = control_enable_service
        self.control_home_service = control_home_service
        self.control_move_joint_service = control_move_joint_service
        self.control_move_joints_service = control_move_joints_service
        self.control_reset_emergency_service = control_reset_emergency_service
        self.execute_demo_action = execute_demo_action
        self.pause_demo_service = pause_demo_service
        self.resume_demo_service = resume_demo_service
        self.stop_demo_service = stop_demo_service
        self.command_timeout_sec = max(0.2, float(command_timeout_sec))
        self.gripper_joint_id = max(1, int(gripper_joint_id))
        self.gripper_position_min = float(gripper_position_min)
        self.gripper_position_max = float(gripper_position_max)
        self.gripper_angle_min_deg = float(gripper_angle_min_deg)
        self.gripper_angle_max_deg = float(gripper_angle_max_deg)
        self.gripper_default_speed = float(gripper_default_speed)
        self.gripper_min_duration_ms = max(50, int(gripper_min_duration_ms))
        self.gripper_max_duration_ms = max(self.gripper_min_duration_ms, int(gripper_max_duration_ms))
        self._gripper_speed = float(gripper_default_speed)
        self._gripper_force = 0.0

        self._running = False
        self._command_queue: "queue.Queue[Tuple[str, Dict[str, Any]]]" = queue.Queue(maxsize=200)
        self._lock = threading.Lock()

        self._arm_enabled = False
        self._arm_homed = False
        self._stm32_connected = False
        self._tactile_connected = False
        self._servo_simulation = False
        self._sensor_simulation = False
        self._arm_snapshot: Dict[str, Any] = {
            "connected": False,
            "enabled": False,
            "homed": False,
            "joint_angles": [],
            "joint_positions": [],
            "joint_targets": [],
            "battery_voltage": 0.0,
            "safety": "unknown",
            "control_mode": "joint",
            "connection_type": "ros2_control",
            "error": False,
            "error_message": "",
        }
        self._last_health: Dict[str, Any] = {}

        self._ros_owned_context = False
        self._node: Optional[_Ros2ControlNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._demo_goal_handle = None
        self._demo_result_future = None
        self._demo_name = ""

    def run(self) -> None:
        if ROS2_IMPORT_ERROR is not None:
            self.error_occurred.emit(
                "ros2_import_error",
                {"error": str(ROS2_IMPORT_ERROR), "message": "rclpy or tactile_interfaces unavailable"},
            )
            self.status_updated.emit("error", {"message": "ROS2 control runtime unavailable"})
            return

        self._running = True
        self.status_updated.emit("starting", {"message": "ROS2 control thread starting"})
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._ros_owned_context = True

            self._node = _Ros2ControlNode(
                owner=self,
                arm_state_topic=self.arm_state_topic,
                health_topic=self.health_topic,
                control_enable_service=self.control_enable_service,
                control_home_service=self.control_home_service,
                control_move_joint_service=self.control_move_joint_service,
                control_move_joints_service=self.control_move_joints_service,
                control_reset_emergency_service=self.control_reset_emergency_service,
                execute_demo_action=self.execute_demo_action,
                pause_demo_service=self.pause_demo_service,
                resume_demo_service=self.resume_demo_service,
                stop_demo_service=self.stop_demo_service,
            )
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)

            self.status_updated.emit(
                "ros2_mode_ready",
                {
                    "message": "ROS2 phase5 control mode is active.",
                    "arm_state_topic": self.arm_state_topic,
                    "health_topic": self.health_topic,
                    "control_enable_service": self.control_enable_service,
                    "execute_demo_action": self.execute_demo_action,
                },
            )

            while self._running:
                self._executor.spin_once(timeout_sec=0.05)
                self._poll_demo_result()
                self._drain_command_queue(max_items=4)
        except ExternalShutdownException:
            # Expected during Ctrl-C / external ROS shutdown.
            pass
        except Exception as exc:  # noqa: BLE001
            self.logger.exception("ROS2 control thread crashed")
            self.error_occurred.emit("thread_error", {"error": str(exc)})
            self.status_updated.emit("error", {"message": f"ROS2 control thread crashed: {exc}"})
        finally:
            self._cleanup_ros()
            self._running = False
            self.status_updated.emit("stopped", {"message": "ROS2 control thread stopped"})

    def _cleanup_ros(self) -> None:
        try:
            if self._executor is not None and self._node is not None:
                self._executor.remove_node(self._node)
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._executor = None
        self._node = None
        self._demo_goal_handle = None
        self._demo_result_future = None
        self._demo_name = ""
        if self._ros_owned_context and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        self._ros_owned_context = False

    def stop_control(self) -> None:
        self._running = False

    def send_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> bool:
        try:
            self._command_queue.put((str(command), dict(params or {})), timeout=0.1)
            return True
        except queue.Full:
            self.error_occurred.emit("command_queue_full", {"command": command})
            self.status_updated.emit("error", {"message": "Control command queue is full"})
            return False

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "arm": dict(self._arm_snapshot),
                "health": dict(self._last_health),
                "servo": {
                    "connected": bool(self._stm32_connected),
                    "simulation": bool(self._servo_simulation),
                },
                "sensor": {
                    "connected": bool(self._tactile_connected),
                    "simulation": bool(self._sensor_simulation),
                },
                "demo": {
                    "running": bool(self._demo_goal_handle is not None),
                    "name": str(self._demo_name),
                },
            }

    def on_arm_state(self, msg: ArmState) -> None:
        snapshot = {
            "connected": bool(msg.connected),
            "enabled": bool(self._arm_enabled),
            "homed": bool(self._arm_homed),
            "joint_angles": [float(v) for v in list(msg.joint_angles)],
            "joint_positions": [float(v) for v in list(msg.joint_positions)],
            "joint_targets": [float(v) for v in list(msg.joint_angles)],
            "battery_voltage": float(msg.battery_voltage),
            "safety": "error" if bool(msg.error) else "normal",
            "control_mode": "joint",
            "connection_type": "ros2_control",
            "error": bool(msg.error),
            "error_message": str(msg.error_message),
            "moving": bool(msg.moving),
        }
        if not snapshot["connected"]:
            snapshot["enabled"] = False

        with self._lock:
            self._arm_snapshot = snapshot
            self._stm32_connected = bool(snapshot["connected"])
            if not self._stm32_connected:
                self._arm_enabled = False

        self.status_updated.emit("arm_state", snapshot)

    def on_health(self, msg: SystemHealth) -> None:
        health = {
            "node_name": str(msg.node_name),
            "healthy": bool(msg.healthy),
            "level": int(msg.level),
            "message": str(msg.message),
        }
        with self._lock:
            self._last_health = health

        node_name = str(health["node_name"])
        message = str(health["message"]).lower()
        healthy = bool(health["healthy"])
        level = int(health["level"])

        if node_name == "tactile_sensor_node":
            sensor_connected = bool(healthy and level <= 1)
            sensor_simulation = "simulation" in message
            with self._lock:
                self._tactile_connected = sensor_connected
                self._sensor_simulation = sensor_simulation

        if node_name == "arm_driver_node":
            arm_connected = bool(healthy and "disconnected" not in message)
            with self._lock:
                self._stm32_connected = arm_connected
                # Arm driver node is always hardware-facing in current phase.
                self._servo_simulation = False

        if (not health["healthy"]) and health["level"] >= 2:
            self.error_occurred.emit("health_error", health)
            self.status_updated.emit(
                "error",
                {"message": f"{health['node_name']} unhealthy: {health['message']}"},
            )
        elif (not health["healthy"]) and health["node_name"] == "arm_control_node":
            self.status_updated.emit(
                "health_warning",
                {"message": f"{health['node_name']}: {health['message']}", **health},
            )

    def _drain_command_queue(self, max_items: int = 4) -> None:
        handled = 0
        while handled < max_items:
            try:
                command, params = self._command_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._execute_command(command, params)
            except Exception as exc:  # noqa: BLE001
                self.error_occurred.emit(
                    "command_error",
                    {"command": command, "params": params, "error": str(exc)},
                )
                self.status_updated.emit("error", {"message": f"{command} failed: {exc}"})
            finally:
                handled += 1
                self._command_queue.task_done()

    def _execute_command(self, command: str, params: Dict[str, Any]) -> None:
        if self._node is None:
            self.status_updated.emit("error", {"message": "ROS2 control node not ready"})
            return

        if command == "connect_hardware":
            with self._lock:
                self._stm32_connected = True
                self._tactile_connected = True
            self._emit_stm32_status(connected=True, success=True, message="STM32 bridge ready (ROS2 mode)")
            self._emit_tactile_status(connected=True, success=True, message="Tactile stream ready (ROS2 mode)")
            self.status_updated.emit(
                "connected",
                {"success": True, "message": "ROS2 hardware chain acknowledged"},
            )
            return

        if command == "disconnect_hardware":
            success, message = self._set_arm_enabled(False)
            with self._lock:
                self._stm32_connected = False
                self._tactile_connected = False
            self._emit_stm32_status(
                connected=False,
                success=True,
                message="STM32 bridge disconnected (ROS2 mode)",
                is_disconnect=True,
            )
            self._emit_tactile_status(
                connected=False,
                success=True,
                message="Tactile stream disconnected (ROS2 mode)",
                is_disconnect=True,
            )
            self.status_updated.emit(
                "disconnected" if success else "error",
                {"success": success, "message": message if message else "Hardware chain disconnected"},
            )
            return

        if command in ("connect_arm", "arm_enable"):
            success, message = self._set_arm_enabled(True)
            if command == "connect_arm":
                self.status_updated.emit(
                    "arm_connect_result",
                    {"success": success, "message": message},
                )
            if command in ("arm_enable", "connect_arm") and not success:
                self.status_updated.emit("error", {"message": message})
            return

        if command in ("disconnect_arm", "arm_disable"):
            success, message = self._set_arm_enabled(False)
            if command == "disconnect_arm":
                self.status_updated.emit(
                    "arm_connect_result",
                    {"success": success, "message": message},
                )
            if (not success) and command in ("arm_disable", "disconnect_arm"):
                self.status_updated.emit("error", {"message": message})
            return

        if command == "connect_stm32":
            with self._lock:
                self._stm32_connected = True
            self._emit_stm32_status(
                connected=True,
                success=True,
                message="STM32 bridge ready (ROS2 mode)",
            )
            return

        if command == "disconnect_stm32":
            with self._lock:
                self._stm32_connected = False
            self._emit_stm32_status(
                connected=False,
                success=True,
                message="STM32 bridge disconnected (ROS2 mode)",
                is_disconnect=True,
            )
            return

        if command == "connect_tactile":
            with self._lock:
                self._tactile_connected = True
            self._emit_tactile_status(
                connected=True,
                success=True,
                message="Tactile stream ready (ROS2 mode)",
            )
            return

        if command == "disconnect_tactile":
            with self._lock:
                self._tactile_connected = False
            self._emit_tactile_status(
                connected=False,
                success=True,
                message="Tactile stream disconnected (ROS2 mode)",
                is_disconnect=True,
            )
            return

        if command == "arm_home":
            result, error = self._call_service(self._node.home_client, Trigger.Request())
            if result is None:
                self.status_updated.emit("error", {"message": f"arm_home failed: {error}"})
                return
            success = bool(result.success)
            message = str(result.message)
            if success:
                with self._lock:
                    self._arm_homed = True
            else:
                self.status_updated.emit("error", {"message": f"arm_home failed: {message}"})
            self.status_updated.emit(
                "arm_home_result",
                {"success": success, "message": message},
            )
            return

        if command == "move_arm_joint":
            req = MoveArmJoint.Request()
            req.joint_id = int(self._resolve_joint_id(params))
            req.angle_deg = float(params.get("angle", params.get("angle_deg", 0.0)))
            req.duration_ms = int(params.get("duration_ms", params.get("duration", 1000)))
            req.wait = bool(params.get("wait", False))
            result, error = self._call_service(self._node.move_joint_client, req)
            if result is None:
                self.status_updated.emit("error", {"message": f"move_arm_joint failed: {error}"})
                return
            if not bool(result.success):
                self.status_updated.emit("error", {"message": str(result.message)})
            self.status_updated.emit(
                "arm_move_joint_result",
                {
                    "success": bool(result.success),
                    "message": str(result.message),
                    "joint_id": int(result.joint_id),
                    "commanded_angle_deg": float(result.commanded_angle_deg),
                },
            )
            return

        if command == "move_arm_joints":
            joint_ids, angles = self._resolve_joint_batch(params)
            if not joint_ids or not angles:
                self.status_updated.emit("error", {"message": "move_arm_joints invalid parameters"})
                return
            if len(joint_ids) != len(angles):
                self.status_updated.emit("error", {"message": "move_arm_joints size mismatch"})
                return

            req = MoveArmJoints.Request()
            req.joint_ids = [int(v) for v in joint_ids]
            req.angles_deg = [float(v) for v in angles]
            req.duration_ms = int(params.get("duration_ms", params.get("duration", 1200)))
            req.wait = bool(params.get("wait", False))
            result, error = self._call_service(self._node.move_joints_client, req)
            if result is None:
                self.status_updated.emit("error", {"message": f"move_arm_joints failed: {error}"})
                return
            if not bool(result.success):
                self.status_updated.emit("error", {"message": str(result.message)})
            self.status_updated.emit(
                "arm_move_joints_result",
                {
                    "success": bool(result.success),
                    "message": str(result.message),
                    "joint_ids": [int(v) for v in list(result.joint_ids)],
                    "commanded_angles_deg": [float(v) for v in list(result.commanded_angles_deg)],
                },
            )
            return

        if command in ("move_gripper", "set_servo_position"):
            self._execute_gripper_move(params)
            return

        if command == "set_servo_speed":
            speed = float(params.get("speed", self._gripper_speed))
            if not math.isfinite(speed):
                self.status_updated.emit("error", {"message": "set_servo_speed invalid speed"})
                return
            self._gripper_speed = max(1.0, min(100.0, speed))
            self.status_updated.emit(
                "speed_set",
                {
                    "success": True,
                    "speed": self._gripper_speed,
                    "message": f"servo speed set to {self._gripper_speed:.1f}",
                },
            )
            return

        if command == "set_servo_force":
            force = float(params.get("force", self._gripper_force))
            if not math.isfinite(force):
                self.status_updated.emit("error", {"message": "set_servo_force invalid force"})
                return
            self._gripper_force = max(0.0, force)
            self.status_updated.emit(
                "force_limit_set",
                {
                    "success": True,
                    "force_limit": self._gripper_force,
                    "message": "force limit cached in ROS2 bridge (no dedicated force service configured)",
                },
            )
            return

        if command in ("calibrate_hardware", "calibrate_3d"):
            self._execute_calibration(command, params)
            return

        if command == "auto_grasp":
            self._execute_auto_grasp_compat(params)
            return

        if command == "start_demo":
            self._execute_start_demo(params)
            return

        if command == "pause_demo":
            self._execute_demo_trigger(
                self._node.pause_demo_client,
                status="demo_running",
                fallback_message="demo paused",
            )
            return

        if command == "resume_demo":
            self._execute_demo_trigger(
                self._node.resume_demo_client,
                status="demo_running",
                fallback_message="demo resumed",
            )
            return

        if command == "stop_demo":
            self._execute_demo_trigger(
                self._node.stop_demo_client,
                status="demo_running",
                fallback_message="demo stop requested",
            )
            return

        if command == "emergency_stop":
            success, message = self._set_arm_enabled(False)
            self.status_updated.emit(
                "emergency_stop",
                {"success": success, "message": message if message else "Emergency stop sent"},
            )
            return

        if command == "reset_system":
            result, error = self._call_service(self._node.reset_emergency_client, Trigger.Request())
            if result is None:
                self.status_updated.emit("error", {"message": f"reset_system failed: {error}"})
                return
            self.status_updated.emit(
                "system_reset_result",
                {"success": bool(result.success), "message": str(result.message)},
            )
            return

        # Not migrated in phase 5 control bridge.
        self.status_updated.emit(
            "command_ignored",
            {
                "command": command,
                "params": params,
                "message": "Command is not migrated to ROS2 phase5 control path.",
            },
        )

    def _execute_start_demo(self, params: Dict[str, Any]) -> None:
        if self._node is None:
            self.status_updated.emit("demo_failed", {"error": "ROS2 control node not ready"})
            return

        demo_name = str(params.get("demo_name", "ros2_demo")).strip() or "ros2_demo"
        if self._demo_goal_handle is not None:
            self.status_updated.emit(
                "demo_failed",
                {"demo_name": demo_name, "error": "another demo is already running"},
            )
            return

        payload = {}
        nested = params.get("params")
        if isinstance(nested, dict):
            payload.update(nested)
        for key, value in params.items():
            if key in ("demo_name", "params", "duration_sec"):
                continue
            payload[key] = value
        if "duration_sec" in params and "duration" not in payload:
            payload["duration"] = params.get("duration_sec")

        goal = ExecuteDemo.Goal()
        goal.demo_name = demo_name
        goal.duration_sec = float(payload.get("duration", params.get("duration_sec", 0.0)) or 0.0)
        try:
            goal.params_json = json.dumps(payload, ensure_ascii=False)
        except Exception:
            goal.params_json = "{}"

        if not self._node.execute_demo_client.wait_for_server(timeout_sec=self.command_timeout_sec):
            self.status_updated.emit(
                "demo_failed",
                {"demo_name": demo_name, "error": "task action server unavailable"},
            )
            return

        self.status_updated.emit(
            "demo_starting",
            {"demo_name": demo_name, "message": "submitting action goal"},
        )

        future = self._node.execute_demo_client.send_goal_async(
            goal, feedback_callback=self._on_demo_feedback
        )
        goal_handle, error = self._wait_future(future, timeout_sec=self.command_timeout_sec)
        if goal_handle is None:
            self.status_updated.emit(
                "demo_failed",
                {"demo_name": demo_name, "error": f"send_goal failed: {error}"},
            )
            return

        if not bool(getattr(goal_handle, "accepted", False)):
            self.status_updated.emit(
                "demo_failed",
                {"demo_name": demo_name, "error": "goal rejected by task node"},
            )
            return

        self._demo_goal_handle = goal_handle
        self._demo_result_future = goal_handle.get_result_async()
        self._demo_name = demo_name
        self.status_updated.emit(
            "demo_started",
            {"demo_name": demo_name, "message": "task action started"},
        )

    def _execute_demo_trigger(self, client, status: str, fallback_message: str) -> None:
        if self._node is None:
            self.status_updated.emit("demo_failed", {"error": "ROS2 control node not ready"})
            return

        result, error = self._call_service(client, Trigger.Request())
        if result is None:
            self.status_updated.emit(
                "demo_failed",
                {"demo_name": self._demo_name, "error": f"{fallback_message} failed: {error}"},
            )
            return

        if not bool(result.success):
            self.status_updated.emit(
                "demo_failed",
                {"demo_name": self._demo_name, "error": str(result.message)},
            )
            return

        self.status_updated.emit(
            status,
            {
                "demo_name": self._demo_name,
                "message": str(result.message) if str(result.message) else fallback_message,
            },
        )

    def _on_demo_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        info = {
            "demo_name": self._demo_name,
            "progress": float(feedback.progress),
            "state": str(feedback.current_state),
            "message": str(feedback.message),
        }
        self.status_updated.emit("demo_progress", info)
        if info["state"] in ("starting", "running", "paused"):
            self.status_updated.emit("demo_running", info)

    def _poll_demo_result(self) -> None:
        if self._demo_result_future is None:
            return
        if not self._demo_result_future.done():
            return

        future = self._demo_result_future
        demo_name = self._demo_name
        self._demo_result_future = None
        self._demo_goal_handle = None
        self._demo_name = ""

        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(result.success)
            message = str(result.message)
            final_state = str(result.final_state or "")
        except Exception as exc:  # noqa: BLE001
            self.status_updated.emit(
                "demo_failed",
                {"demo_name": demo_name, "error": f"result retrieval failed: {exc}"},
            )
            return

        info = {
            "demo_name": demo_name,
            "success": success,
            "message": message,
            "final_state": final_state,
        }
        if success and final_state == "completed":
            self.status_updated.emit("demo_completed", info)
            return
        if final_state in ("stopped", "canceled"):
            self.status_updated.emit("demo_stopped", info)
            return

        self.status_updated.emit(
            "demo_failed",
            {
                "demo_name": demo_name,
                "error": message if message else "demo failed",
                "final_state": final_state,
            },
        )

    def _wait_future(self, future, timeout_sec: float):
        if self._executor is None:
            return None, "executor not initialized"

        deadline = time.time() + max(0.1, float(timeout_sec))
        while self._running and time.time() < deadline:
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except ExternalShutdownException:
                return None, "ros context shutdown"
            if future.done():
                break

        if not future.done():
            return None, "future timeout"
        try:
            return future.result(), ""
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

    def _execute_gripper_move(self, params: Dict[str, Any]) -> bool:
        if self._node is None:
            self.status_updated.emit("error", {"message": "ROS2 control node not ready"})
            return False

        position = float(params.get("position", 50.0))
        speed = float(params.get("speed", self._gripper_speed))
        wait = bool(params.get("wait", False))
        duration_ms = params.get("duration_ms")
        if duration_ms is None:
            duration_ms = self._speed_to_duration_ms(speed)
        else:
            duration_ms = max(50, int(duration_ms))

        angle_deg = self._position_to_angle(position)
        req = MoveArmJoint.Request()
        req.joint_id = int(self.gripper_joint_id)
        req.angle_deg = float(angle_deg)
        req.duration_ms = int(duration_ms)
        req.wait = wait

        result, error = self._call_service(self._node.move_joint_client, req)
        if result is None:
            self.status_updated.emit("move_error", {"error": f"move_gripper failed: {error}"})
            self.status_updated.emit("error", {"message": f"move_gripper failed: {error}"})
            return False

        success = bool(result.success)
        message = str(result.message)
        if not success:
            self.status_updated.emit("move_error", {"error": message})
            self.status_updated.emit("error", {"message": message})
            return False

        self.status_updated.emit(
            "moved",
            {
                "success": True,
                "position": float(position),
                "joint_id": int(result.joint_id),
                "commanded_angle_deg": float(result.commanded_angle_deg),
                "message": message,
            },
        )
        return True

    def _execute_calibration(self, command: str, params: Dict[str, Any]) -> None:
        self.status_updated.emit(
            "calibrating",
            {
                "command": command,
                "message": "ROS2 calibration started",
                "calibration_type": str(params.get("type", "all")),
            },
        )

        if self._node is None:
            self.status_updated.emit("calibration_error", {"error": "ROS2 control node not ready"})
            self.status_updated.emit("error", {"message": "calibration failed: ROS2 control node not ready"})
            return

        result, error = self._call_service(self._node.home_client, Trigger.Request())
        if result is None:
            self.status_updated.emit("calibration_error", {"error": error})
            self.status_updated.emit("error", {"message": f"calibration failed: {error}"})
            return

        if not bool(result.success):
            message = str(result.message)
            self.status_updated.emit("calibration_error", {"error": message})
            self.status_updated.emit("error", {"message": f"calibration failed: {message}"})
            return

        self.status_updated.emit(
            "calibrated",
            {
                "success": True,
                "message": "ROS2 calibration completed (arm homed)",
                "calibration_type": str(params.get("type", "all")),
            },
        )

    def _execute_auto_grasp_compat(self, params: Dict[str, Any]) -> None:
        # Phase4 compatibility path: keep UI command functional by executing
        # the gripper open/close sequence on the mapped arm joint.
        open_pos = float(params.get("gripper_open_position", 100.0))
        close_pos = float(params.get("gripper_close_position", 40.0))
        speed = float(params.get("speed", self._gripper_speed))
        close_gripper = bool(params.get("close_gripper", True))

        open_ok = self._execute_gripper_move({"position": open_pos, "speed": speed, "wait": True})
        close_ok = True
        if close_gripper:
            close_ok = self._execute_gripper_move({"position": close_pos, "speed": speed, "wait": True})

        success = bool(open_ok and close_ok)
        if not success:
            self.status_updated.emit(
                "error",
                {"message": "auto_grasp compatibility sequence failed"},
            )

        self.status_updated.emit(
            "auto_grasp_result",
            {
                "success": success,
                "mode": "gripper_only",
                "message": "auto_grasp executed in phase5 compatibility mode (gripper sequence only)"
                if success
                else "auto_grasp compatibility sequence failed",
                "object_cam_mm": params.get("object_cam_mm"),
            },
        )

    def _position_to_angle(self, position: float) -> float:
        pmin = self.gripper_position_min
        pmax = self.gripper_position_max
        amin = self.gripper_angle_min_deg
        amax = self.gripper_angle_max_deg

        if not math.isfinite(position):
            position = pmin
        if pmax <= pmin:
            return float(amin)

        clamped = max(pmin, min(pmax, position))
        ratio = (clamped - pmin) / (pmax - pmin)
        return float(amin + ratio * (amax - amin))

    def _speed_to_duration_ms(self, speed: float) -> int:
        if not math.isfinite(speed):
            speed = self.gripper_default_speed
        clamped = max(1.0, min(100.0, float(speed)))
        ratio = clamped / 100.0
        duration = self.gripper_max_duration_ms - ratio * (
            self.gripper_max_duration_ms - self.gripper_min_duration_ms
        )
        return int(round(duration))

    def _set_arm_enabled(self, enabled: bool) -> Tuple[bool, str]:
        if self._node is None:
            return False, "ROS2 control node not ready"
        req = SetBool.Request()
        req.data = bool(enabled)
        result, error = self._call_service(self._node.enable_client, req)
        if result is None:
            return False, f"enable call failed: {error}"
        success = bool(result.success)
        message = str(result.message)
        if success:
            with self._lock:
                self._arm_enabled = bool(enabled)
                if not enabled:
                    self._arm_homed = False
        return success, message

    def _emit_stm32_status(
        self, connected: bool, success: bool, message: str, is_disconnect: bool = False
    ) -> None:
        status = "stm32_disconnect_result" if is_disconnect else "stm32_connect_result"
        self.status_updated.emit(
            status,
            {
                "success": bool(success),
                "connected": bool(connected),
                "simulation": bool(self._servo_simulation),
                "message": str(message),
            },
        )

    def _emit_tactile_status(
        self, connected: bool, success: bool, message: str, is_disconnect: bool = False
    ) -> None:
        status = "tactile_disconnect_result" if is_disconnect else "tactile_connect_result"
        self.status_updated.emit(
            status,
            {
                "success": bool(success),
                "connected": bool(connected),
                "simulation": bool(self._sensor_simulation),
                "message": str(message),
            },
        )

    def _call_service(self, client, request):
        if self._executor is None:
            return None, "executor not initialized"
        if not client.wait_for_service(timeout_sec=self.command_timeout_sec):
            return None, "service unavailable"

        future = client.call_async(request)
        deadline = time.time() + self.command_timeout_sec
        while self._running and time.time() < deadline:
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except ExternalShutdownException:
                return None, "ros context shutdown"
            if future.done():
                break
        if not future.done():
            return None, "service timeout"
        try:
            return future.result(), ""
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

    @staticmethod
    def _resolve_joint_id(params: Dict[str, Any]) -> int:
        if "joint_id" in params:
            return int(params["joint_id"])
        if "joint_index" in params:
            return int(params["joint_index"]) + 1
        return 1

    @staticmethod
    def _resolve_joint_batch(params: Dict[str, Any]) -> Tuple[List[int], List[float]]:
        if "joint_ids" in params:
            ids = [int(v) for v in list(params.get("joint_ids") or [])]
        elif "joint_indices" in params:
            ids = [int(v) + 1 for v in list(params.get("joint_indices") or [])]
        else:
            ids = []

        if "angles_deg" in params:
            angles = [float(v) for v in list(params.get("angles_deg") or [])]
        else:
            angles = [float(v) for v in list(params.get("angles") or [])]

        if not ids and angles:
            ids = list(range(1, len(angles) + 1))
        if len(ids) != len(angles):
            return [], []
        if any((not math.isfinite(v)) for v in angles):
            return [], []
        return ids, angles


class Ros2DemoManagerStub(QObject):
    """Minimal demo manager replacement for phased ROS2 migration."""

    status_changed = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, dict)
    demo_started = pyqtSignal(str, dict)
    demo_stopped = pyqtSignal(str, dict)
    demo_progress = pyqtSignal(float, dict)
    vector_data_updated = pyqtSignal(object)
    contact_map_updated = pyqtSignal(object)
    tactile_mapping_ready = pyqtSignal(object)

    _FORWARDED_CONTROL_COMMANDS = {
        "connect_hardware",
        "disconnect_hardware",
        "connect_stm32",
        "disconnect_stm32",
        "connect_tactile",
        "disconnect_tactile",
        "calibrate_hardware",
        "calibrate_3d",
        "move_gripper",
        "set_servo_position",
        "set_servo_speed",
        "set_servo_force",
        "emergency_stop",
        "reset_system",
        "connect_arm",
        "disconnect_arm",
        "arm_enable",
        "arm_disable",
        "arm_home",
        "move_arm_joint",
        "move_arm_joints",
        "auto_grasp",
        "start_demo",
        "stop_demo",
        "pause_demo",
        "resume_demo",
    }

    def __init__(self, config: Any, data_acquisition: Any, control_thread: Any):
        super().__init__()
        self.config = config
        self.data_acquisition = data_acquisition
        self.control_thread = control_thread
        self.hardware_interface = None
        self.logger = logging.getLogger(__name__)
        if self.control_thread is not None and hasattr(self.control_thread, "status_updated"):
            self.control_thread.status_updated.connect(self._on_control_status)

    def handle_control_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> None:
        params = params or {}
        if self.control_thread is None or not hasattr(self.control_thread, "send_command"):
            self.status_changed.emit(
                "error",
                {
                    "mode": "ros2_phase5",
                    "command": command,
                    "message": "ROS2 control thread is unavailable",
                },
            )
            return

        if command in self._FORWARDED_CONTROL_COMMANDS:
            ok = bool(self.control_thread.send_command(command, params))
            if not ok:
                self.status_changed.emit(
                    "error",
                    {
                        "mode": "ros2_phase5",
                        "command": command,
                        "message": "Failed to queue ROS2 control command",
                    },
                )
            return

        self.status_changed.emit(
            "command_ignored",
            {
                "mode": "ros2_phase5",
                "command": command,
                "message": "Command is not supported in current ROS2 stage.",
            },
        )

    def _on_control_status(self, status: str, info: Dict[str, Any]) -> None:
        info = dict(info or {})
        demo_statuses = {
            "demo_starting",
            "demo_started",
            "demo_running",
            "demo_progress",
            "demo_stopped",
            "demo_completed",
            "demo_failed",
        }
        if status not in demo_statuses:
            return

        self.status_changed.emit(status, info)
        demo_name = str(info.get("demo_name", "ros2_demo"))

        if status == "demo_started":
            self.demo_started.emit(demo_name, info)
        elif status in ("demo_stopped", "demo_completed", "demo_failed"):
            self.demo_stopped.emit(demo_name, info)
        elif status == "demo_progress":
            progress = float(info.get("progress", 0.0))
            self.demo_progress.emit(progress, info)

    def start_demo(self, demo_name: str, params: Optional[Dict[str, Any]] = None) -> bool:
        self.handle_control_command("start_demo", {"demo_name": demo_name, **(params or {})})
        return True

    def stop_demo(self) -> bool:
        self.handle_control_command("stop_demo", {})
        return True

    def get_status(self) -> Dict[str, Any]:
        latest = None
        if self.data_acquisition is not None and hasattr(self.data_acquisition, "get_latest_data"):
            latest = self.data_acquisition.get_latest_data()

        arm_status: Dict[str, Any] = {"connected": False, "enabled": False, "homed": False}
        servo_status: Dict[str, Any] = {"connected": False, "simulation": False}
        sensor_status: Dict[str, Any] = {"connected": False, "simulation": False}
        demo_status: Dict[str, Any] = {"running": False, "name": ""}
        if self.control_thread is not None and hasattr(self.control_thread, "get_status"):
            maybe = self.control_thread.get_status()
            if isinstance(maybe, dict):
                arm_status = maybe.get("arm", arm_status) or arm_status
                servo_status = maybe.get("servo", servo_status) or servo_status
                sensor_status = maybe.get("sensor", sensor_status) or sensor_status
                demo_status = maybe.get("demo", demo_status) or demo_status

        tactile_connected = bool(latest is not None) or bool(sensor_status.get("connected", False))

        return {
            "mode": "ros2_phase5",
            "servo": {
                "connected": bool(servo_status.get("connected", False)),
                "simulation": bool(servo_status.get("simulation", False)),
            },
            "sensor": {
                "connected": tactile_connected,
                "simulation": bool(sensor_status.get("simulation", False)),
                "latest_timestamp": getattr(latest, "timestamp", None),
            },
            "arm": arm_status,
            "demo": demo_status,
        }

    def export_data(self, file_path: str) -> bool:
        try:
            latest = None
            if self.data_acquisition is not None and hasattr(self.data_acquisition, "get_latest_data"):
                latest = self.data_acquisition.get_latest_data()
            payload = {
                "mode": "ros2_phase5",
                "export_time": time.time(),
                "latest_data": latest.to_dict() if latest is not None and hasattr(latest, "to_dict") else None,
            }
            with open(file_path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2)
            return True
        except Exception as exc:  # noqa: BLE001
            self.error_occurred.emit("export_error", {"error": str(exc)})
            return False
