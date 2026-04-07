from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger

from tactile_interfaces.msg import ArmState, SystemHealth
from tactile_interfaces.srv import MoveArmJoint, MoveArmJoints

try:
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint

    ROS2_CONTROL_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - runtime dependency in ROS2 env
    FollowJointTrajectory = None
    ActionClient = None
    JointState = None
    JointTrajectoryPoint = None
    ROS2_CONTROL_IMPORT_ERROR = exc


class ArmSimDriverNode(Node):
    """Phase 6.2 simulation arm backend.

    This node mirrors the phase-2 arm driver service API so existing control/task
    nodes can run unchanged while Gazebo/ros2_control integration is staged in.

    Backends:
    - memory: in-process interpolation (phase6_sim_base default)
    - ros2_control: proxy /arm/* service calls to FollowJointTrajectory action
    """

    def __init__(self) -> None:
        super().__init__("arm_sim_driver_node")

        self.declare_parameter("backend", "memory")
        self.declare_parameter("state_rate_hz", 20.0)
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("arm_num_joints", 6)
        self.declare_parameter("command_default_duration_ms", 1000)
        self.declare_parameter("home_duration_ms", 1500)
        self.declare_parameter("home_joint_angles", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("initial_joint_angles", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_zero_offsets_deg", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_names", Parameter.Type.STRING_ARRAY)

        self.declare_parameter("enforce_angle_limits", True)
        self.declare_parameter("min_angle_deg", -180.0)
        self.declare_parameter("max_angle_deg", 270.0)

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "trajectory_action_name", "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.declare_parameter("action_server_timeout_sec", 3.0)
        self.declare_parameter("trajectory_result_timeout_sec", 10.0)
        self.declare_parameter("joint_state_stale_timeout_sec", 1.5)

        self.declare_parameter("state_topic", "/arm/state")
        self.declare_parameter("health_topic", "/system/health")
        self.declare_parameter("simulation_mode_label", "phase6_sim_base")

        backend = str(self.get_parameter("backend").value).strip().lower()
        self.backend = backend if backend in {"memory", "ros2_control"} else "memory"
        if self.backend != backend:
            self.get_logger().warn(
                f"unknown backend '{backend}', fallback to memory"
            )

        self.state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.arm_num_joints = max(1, int(self.get_parameter("arm_num_joints").value))
        self.command_default_duration_ms = max(
            50, int(self.get_parameter("command_default_duration_ms").value)
        )
        self.home_duration_ms = max(50, int(self.get_parameter("home_duration_ms").value))

        self.enforce_angle_limits = bool(self.get_parameter("enforce_angle_limits").value)
        self.min_angle_deg = float(self.get_parameter("min_angle_deg").value)
        self.max_angle_deg = float(self.get_parameter("max_angle_deg").value)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.trajectory_action_name = str(self.get_parameter("trajectory_action_name").value)
        self.action_server_timeout_sec = max(
            0.1, float(self.get_parameter("action_server_timeout_sec").value)
        )
        self.trajectory_result_timeout_sec = max(
            0.1, float(self.get_parameter("trajectory_result_timeout_sec").value)
        )
        self.joint_state_stale_timeout_sec = max(
            0.1, float(self.get_parameter("joint_state_stale_timeout_sec").value)
        )

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.simulation_mode_label = str(self.get_parameter("simulation_mode_label").value)

        if self.min_angle_deg > self.max_angle_deg:
            self.min_angle_deg, self.max_angle_deg = self.max_angle_deg, self.min_angle_deg

        default_joint_names = [f"joint{i + 1}" for i in range(self.arm_num_joints)]
        joint_names = self._parse_string_list_param("joint_names")
        if not joint_names:
            joint_names = list(default_joint_names)
        if len(joint_names) < self.arm_num_joints:
            joint_names.extend(default_joint_names[len(joint_names):])
        if len(joint_names) > self.arm_num_joints:
            joint_names = joint_names[: self.arm_num_joints]
        self.joint_names = joint_names

        home_joint_angles = self._normalize_angles(
            self._parse_float_list_param("home_joint_angles"), default_value=0.0
        )
        initial_joint_angles = self._parse_float_list_param("initial_joint_angles")
        if not initial_joint_angles:
            initial_joint_angles = list(home_joint_angles)
        initial_joint_angles = self._normalize_angles(initial_joint_angles, default_value=0.0)
        self.joint_zero_offsets_deg = self._normalize_angles(
            self._parse_float_list_param("joint_zero_offsets_deg"), default_value=0.0
        )

        self._state_lock = threading.Lock()
        self._service_group = ReentrantCallbackGroup()
        self._backend_group = ReentrantCallbackGroup()
        self._timer_group = ReentrantCallbackGroup()
        self._connected = bool(self.auto_enable)
        self._moving = False
        self._error = False
        self._error_message = ""

        self._joint_angles = initial_joint_angles
        self._start_angles = list(initial_joint_angles)
        self._target_angles = list(initial_joint_angles)
        self._command_start_ts = 0.0
        self._command_end_ts = 0.0

        self._home_joint_angles = home_joint_angles
        self._joint_state_received_ts = 0.0

        self._joint_state_sub = None
        self._trajectory_client = None

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(ArmState, self.state_topic, state_qos)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, state_qos)

        if self.backend == "ros2_control":
            self._init_ros2_control_backend()

        self.enable_srv = self.create_service(
            SetBool, "/arm/enable", self._on_enable, callback_group=self._service_group
        )
        self.home_srv = self.create_service(
            Trigger, "/arm/home", self._on_home, callback_group=self._service_group
        )
        self.move_joint_srv = self.create_service(
            MoveArmJoint, "/arm/move_joint", self._on_move_joint, callback_group=self._service_group
        )
        self.move_joints_srv = self.create_service(
            MoveArmJoints, "/arm/move_joints", self._on_move_joints, callback_group=self._service_group
        )

        state_period = max(0.01, 1.0 / max(1.0, self.state_rate_hz))
        self.create_timer(state_period, self._on_state_timer, callback_group=self._timer_group)
        self.create_timer(1.0, self._publish_health, callback_group=self._timer_group)

        self.get_logger().info(
            "arm_sim_driver_node started: "
            f"backend={self.backend}, state_topic={self.state_topic}, "
            f"num_joints={self.arm_num_joints}, auto_enable={self.auto_enable}, "
            f"mode={self.simulation_mode_label}"
        )
        if any(abs(offset) > 1e-6 for offset in self.joint_zero_offsets_deg):
            self.get_logger().info(
                f"arm joint zero offsets enabled: {self.joint_zero_offsets_deg}"
            )

    def _init_ros2_control_backend(self) -> None:
        if ROS2_CONTROL_IMPORT_ERROR is not None:
            with self._state_lock:
                self._error = True
                self._error_message = (
                    "ros2_control backend import failed: "
                    f"{ROS2_CONTROL_IMPORT_ERROR}"
                )
            self.get_logger().error(self._error_message)
            return

        joint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            joint_qos,
            callback_group=self._backend_group,
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action_name,
            callback_group=self._backend_group,
        )
        self.get_logger().info(
            "ros2_control backend initialized: "
            f"joint_state_topic={self.joint_state_topic}, action={self.trajectory_action_name}"
        )

    def _parse_float_list_param(self, name: str) -> List[float]:
        try:
            raw = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        if raw is None or not isinstance(raw, (list, tuple)):
            return []

        values: List[float] = []
        for item in raw:
            try:
                values.append(float(item))
            except (TypeError, ValueError):
                continue
        return values

    def _parse_string_list_param(self, name: str) -> List[str]:
        try:
            raw = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        if raw is None or not isinstance(raw, (list, tuple)):
            return []
        values: List[str] = []
        for item in raw:
            if item is None:
                continue
            text = str(item).strip()
            if text:
                values.append(text)
        return values

    def _normalize_angles(self, angles: List[float], default_value: float) -> List[float]:
        normalized = [float(v) for v in angles]
        if len(normalized) < self.arm_num_joints:
            normalized.extend([default_value] * (self.arm_num_joints - len(normalized)))
        if len(normalized) > self.arm_num_joints:
            normalized = normalized[: self.arm_num_joints]
        return normalized

    def _resolve_duration_ms(self, requested_duration_ms: int) -> int:
        if requested_duration_ms > 0:
            return int(requested_duration_ms)
        return int(self.command_default_duration_ms)

    def _user_to_model_angles(self, angles_deg: List[float]) -> List[float]:
        return [
            float(angle) + self.joint_zero_offsets_deg[idx]
            for idx, angle in enumerate(angles_deg)
        ]

    def _model_to_user_angles(self, angles_deg: List[float]) -> List[float]:
        return [
            float(angle) - self.joint_zero_offsets_deg[idx]
            for idx, angle in enumerate(angles_deg)
        ]

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

    def _start_motion(self, target_angles: List[float], duration_ms: int) -> None:
        now = time.time()
        duration_sec = max(0.02, float(duration_ms) / 1000.0)

        with self._state_lock:
            self._advance_motion_locked(now)
            self._start_angles = list(self._joint_angles)
            self._target_angles = list(target_angles)
            self._command_start_ts = now
            self._command_end_ts = now + duration_sec
            self._moving = True
            self._error = False
            self._error_message = ""

    def _advance_motion_locked(self, now_ts: float) -> None:
        if self.backend != "memory":
            return
        if not self._moving:
            return

        duration = max(1e-6, self._command_end_ts - self._command_start_ts)
        if now_ts >= self._command_end_ts:
            self._joint_angles = list(self._target_angles)
            self._moving = False
            return

        alpha = max(0.0, min(1.0, (now_ts - self._command_start_ts) / duration))
        self._joint_angles = [
            start + (target - start) * alpha
            for start, target in zip(self._start_angles, self._target_angles)
        ]

    def _wait_motion_complete(self, timeout_sec: float) -> bool:
        deadline = time.time() + max(0.1, timeout_sec)
        while time.time() < deadline:
            with self._state_lock:
                self._advance_motion_locked(time.time())
                if not self._moving:
                    return True
            time.sleep(0.01)
        return False

    def _wait_ros2_target_reached(self, timeout_sec: float, tolerance_deg: float = 1.0) -> bool:
        deadline = time.time() + max(0.1, timeout_sec)
        while time.time() < deadline:
            with self._state_lock:
                target = list(self._target_angles)
                current = list(self._joint_angles)
                moving = bool(self._moving)
                joint_state_ts = float(self._joint_state_received_ts)

            if joint_state_ts > 0.0 and all(
                abs(current_angle - target_angle) <= tolerance_deg
                for current_angle, target_angle in zip(current, target)
            ):
                with self._state_lock:
                    self._moving = False
                return True

            if not moving and joint_state_ts > 0.0:
                return all(
                    abs(current_angle - target_angle) <= tolerance_deg
                    for current_angle, target_angle in zip(current, target)
                )

            time.sleep(0.02)
        return False

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        now_ts = time.time()

        with self._state_lock:
            updated = False
            next_angles = list(self._joint_angles)
            for i, joint_name in enumerate(self.joint_names):
                idx = name_to_idx.get(joint_name)
                if idx is None or idx >= len(msg.position):
                    continue
                next_angles[i] = (
                    math.degrees(float(msg.position[idx])) - self.joint_zero_offsets_deg[i]
                )
                updated = True

            if updated:
                self._joint_angles = next_angles
                self._joint_state_received_ts = now_ts
                if self._moving:
                    if all(
                        abs(current - target) <= 1.0
                        for current, target in zip(self._joint_angles, self._target_angles)
                    ):
                        self._moving = False

    def _ros2_action_available(self) -> Tuple[bool, str]:
        if self.backend != "ros2_control":
            return True, ""
        if self._trajectory_client is None:
            return False, "trajectory action client not initialized"
        if not self._trajectory_client.wait_for_server(timeout_sec=self.action_server_timeout_sec):
            return False, "trajectory action server unavailable"
        return True, ""

    def _is_joint_state_fresh(self) -> bool:
        with self._state_lock:
            ts = self._joint_state_received_ts
        if ts <= 0.0:
            return False
        return (time.time() - ts) <= self.joint_state_stale_timeout_sec

    def _wait_for_future(self, future, timeout_sec: float):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=max(0.1, float(timeout_sec))):
            return None, "future timeout"
        try:
            return future.result(), ""
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

    def _execute_ros2_trajectory(
        self,
        target_angles_deg: List[float],
        duration_ms: int,
        wait: bool,
    ) -> Tuple[bool, str]:
        ok, message = self._ros2_action_available()
        if not ok:
            return False, message

        duration_ms = max(50, int(duration_ms))
        sec = duration_ms // 1000
        nanosec = int((duration_ms % 1000) * 1_000_000)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        target_model_angles_deg = self._user_to_model_angles(target_angles_deg)
        point.positions = [math.radians(float(v)) for v in target_model_angles_deg]
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int(nanosec)
        goal.trajectory.header.stamp = (self.get_clock().now() + Duration(seconds=0.2)).to_msg()
        goal.trajectory.points = [point]

        with self._state_lock:
            self._target_angles = list(target_angles_deg)
            self._moving = True
            self._error = False
            self._error_message = ""

        send_future = self._trajectory_client.send_goal_async(goal)
        goal_handle, error = self._wait_for_future(send_future, self.action_server_timeout_sec)
        if goal_handle is None:
            with self._state_lock:
                self._error = True
                self._error_message = f"trajectory goal send failed: {error}"
                self._moving = False
            return False, self._error_message
        if goal_handle is None or not goal_handle.accepted:
            with self._state_lock:
                self._error = True
                self._error_message = "trajectory goal rejected"
                self._moving = False
            return False, self._error_message

        if not wait:
            return True, "trajectory goal accepted"

        execution_timeout_sec = max(self.trajectory_result_timeout_sec, duration_ms / 1000.0 + 1.0)
        reached = self._wait_ros2_target_reached(execution_timeout_sec)
        if reached:
            with self._state_lock:
                self._error = False
                self._error_message = ""
                self._moving = False
            return True, "trajectory completed"

        result_future = goal_handle.get_result_async()
        wrapped, error = self._wait_for_future(result_future, timeout_sec=1.0)
        detail = ""
        if wrapped is not None:
            result = wrapped.result
            if result.error_code != 0:
                detail = result.error_string or f"error_code={result.error_code}"
            else:
                detail = "target not reached within tolerance"
        else:
            detail = error or "target not reached within tolerance"

        with self._state_lock:
            self._error = True
            self._error_message = f"trajectory execution failed: {detail}"
            self._moving = False
        return False, self._error_message

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not bool(request.data):
            with self._state_lock:
                self._connected = False
                self._moving = False
                self._target_angles = list(self._joint_angles)
            response.success = True
            response.message = "sim arm disabled"
            return response

        if self.backend == "ros2_control":
            ok, message = self._ros2_action_available()
            if not ok:
                response.success = False
                response.message = message
                return response
            if not self._is_joint_state_fresh():
                self.get_logger().warn(
                    "enabling ros2_control backend without fresh /joint_states yet"
                )

        with self._state_lock:
            self._connected = True
            self._error = False
            self._error_message = ""

        response.success = True
        response.message = "sim arm enabled"
        return response

    def _on_home(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            connected = self._connected

        if not connected:
            response.success = False
            response.message = "sim arm is not enabled"
            return response

        if self.backend == "memory":
            self._start_motion(list(self._home_joint_angles), self.home_duration_ms)
            response.success = True
            response.message = "sim arm homing started"
            return response

        ok, message = self._execute_ros2_trajectory(
            target_angles_deg=list(self._home_joint_angles),
            duration_ms=self.home_duration_ms,
            wait=True,
        )
        response.success = ok
        response.message = message
        return response

    def _on_move_joint(
        self, request: MoveArmJoint.Request, response: MoveArmJoint.Response
    ) -> MoveArmJoint.Response:
        response.joint_id = int(request.joint_id)
        response.commanded_angle_deg = float(request.angle_deg)

        with self._state_lock:
            connected = self._connected
            current = list(self._joint_angles)

        if not connected:
            response.success = False
            response.message = "sim arm is not enabled"
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

        target = list(current)
        target[joint_id - 1] = angle_deg
        duration_ms = self._resolve_duration_ms(int(request.duration_ms))

        if self.backend == "memory":
            self._start_motion(target, duration_ms)
            if bool(request.wait):
                done = self._wait_motion_complete(duration_ms / 1000.0 + 1.0)
                if not done:
                    response.success = False
                    response.message = "sim arm move_joint timeout"
                    return response
            response.success = True
            response.message = "sim arm move_joint accepted"
            return response

        ok, message = self._execute_ros2_trajectory(
            target_angles_deg=target,
            duration_ms=duration_ms,
            wait=bool(request.wait),
        )
        response.success = ok
        response.message = message
        return response

    def _on_move_joints(
        self, request: MoveArmJoints.Request, response: MoveArmJoints.Response
    ) -> MoveArmJoints.Response:
        response.joint_ids = [int(v) for v in list(request.joint_ids)]
        response.commanded_angles_deg = [float(v) for v in list(request.angles_deg)]

        with self._state_lock:
            connected = self._connected
            current = list(self._joint_angles)

        if not connected:
            response.success = False
            response.message = "sim arm is not enabled"
            return response

        joint_ids = [int(v) for v in list(request.joint_ids)]
        angles_deg = [float(v) for v in list(request.angles_deg)]

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

        target = list(current)
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

            target[joint_id - 1] = angle_deg

        duration_ms = self._resolve_duration_ms(int(request.duration_ms))

        if self.backend == "memory":
            self._start_motion(target, duration_ms)
            if bool(request.wait):
                done = self._wait_motion_complete(duration_ms / 1000.0 + 1.0)
                if not done:
                    response.success = False
                    response.message = "sim arm move_joints timeout"
                    return response
            response.success = True
            response.message = "sim arm move_joints accepted"
            response.commanded_angles_deg = [float(v) for v in target]
            return response

        ok, message = self._execute_ros2_trajectory(
            target_angles_deg=target,
            duration_ms=duration_ms,
            wait=bool(request.wait),
        )
        response.success = ok
        response.message = message
        response.commanded_angles_deg = [float(v) for v in target]
        return response

    def _on_state_timer(self) -> None:
        now = time.time()
        with self._state_lock:
            self._advance_motion_locked(now)
            connected = self._connected
            moving = self._moving
            error = self._error
            error_message = self._error_message
            joint_angles = list(self._joint_angles)
            joint_state_ts = self._joint_state_received_ts

        if (
            self.backend == "ros2_control"
            and connected
            and joint_state_ts > 0.0
            and (now - joint_state_ts) > self.joint_state_stale_timeout_sec
        ):
            error = True
            if not error_message:
                error_message = "joint_states stream stale"

        msg = ArmState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sim_arm_ros2_control" if self.backend == "ros2_control" else "sim_arm"
        msg.connected = connected
        msg.moving = moving
        msg.error = error
        msg.error_message = error_message
        msg.battery_voltage = 12.0 if connected else 0.0
        msg.joint_positions = [float(v) for v in joint_angles]
        msg.joint_angles = [float(v) for v in joint_angles]
        self.state_pub.publish(msg)

    def _publish_health(self) -> None:
        with self._state_lock:
            connected = self._connected
            moving = self._moving
            error = self._error
            error_message = self._error_message
            joint_state_ts = self._joint_state_received_ts

        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0

        if error:
            msg.healthy = False
            msg.level = 2
            msg.message = error_message or "sim arm error"
            self.health_pub.publish(msg)
            return

        if not connected:
            msg.healthy = True
            msg.level = 1
            msg.message = "sim arm disabled; call /arm/enable to activate"
            self.health_pub.publish(msg)
            return

        if self.backend == "ros2_control":
            fresh = (
                joint_state_ts > 0.0
                and (time.time() - joint_state_ts) <= self.joint_state_stale_timeout_sec
            )
            if not fresh:
                msg.healthy = False
                msg.level = 2
                msg.message = "ros2_control enabled but /joint_states not fresh"
            else:
                msg.healthy = True
                msg.level = 0
                msg.message = (
                    f"sim arm active ({self.simulation_mode_label}, ros2_control), "
                    f"state={'moving' if moving else 'idle'}"
                )
            self.health_pub.publish(msg)
            return

        msg.healthy = True
        msg.level = 0
        msg.message = (
            f"sim arm active ({self.simulation_mode_label}, memory), "
            f"state={'moving' if moving else 'idle'}"
        )
        self.health_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmSimDriverNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

