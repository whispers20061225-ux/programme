from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger

from tactile_interfaces.action import MoveArmJoints as MoveArmJointsAction
from tactile_interfaces.msg import ArmState, SystemHealth
from tactile_interfaces.srv import MoveArmJoint
from tactile_interfaces.srv import MoveArmJoints as MoveArmJointsSrv


class ArmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control_node")

        self.declare_parameter("arm_state_topic", "/arm/state")
        self.declare_parameter("health_topic", "/system/health")

        self.declare_parameter("hardware_enable_service", "/arm/enable")
        self.declare_parameter("hardware_home_service", "/arm/home")
        self.declare_parameter("hardware_move_joint_service", "/arm/move_joint")
        self.declare_parameter("hardware_move_joints_service", "/arm/move_joints")

        self.declare_parameter("control_enable_service", "/control/arm/enable")
        self.declare_parameter("control_home_service", "/control/arm/home")
        self.declare_parameter("control_move_joint_service", "/control/arm/move_joint")
        self.declare_parameter("control_move_joints_service", "/control/arm/move_joints")
        self.declare_parameter("control_move_joints_action", "/control/arm/move_joints_action")
        self.declare_parameter("control_reset_emergency_service", "/system/reset_emergency")

        self.declare_parameter("arm_num_joints", 6)
        self.declare_parameter("arm_state_timeout_sec", 2.0)
        self.declare_parameter("command_timeout_sec", 5.0)
        self.declare_parameter("client_retry_count", 1)
        self.declare_parameter("emergency_latch_level", 2)
        self.declare_parameter("reject_on_arm_error", True)
        self.declare_parameter("enforce_angle_limits", True)
        self.declare_parameter("min_angle_deg", -180.0)
        self.declare_parameter("max_angle_deg", 270.0)

        self.arm_state_topic = str(self.get_parameter("arm_state_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)

        self.hardware_enable_service = str(self.get_parameter("hardware_enable_service").value)
        self.hardware_home_service = str(self.get_parameter("hardware_home_service").value)
        self.hardware_move_joint_service = str(
            self.get_parameter("hardware_move_joint_service").value
        )
        self.hardware_move_joints_service = str(
            self.get_parameter("hardware_move_joints_service").value
        )

        self.control_enable_service = str(self.get_parameter("control_enable_service").value)
        self.control_home_service = str(self.get_parameter("control_home_service").value)
        self.control_move_joint_service = str(
            self.get_parameter("control_move_joint_service").value
        )
        self.control_move_joints_service = str(
            self.get_parameter("control_move_joints_service").value
        )
        self.control_move_joints_action = str(
            self.get_parameter("control_move_joints_action").value
        )
        self.control_reset_emergency_service = str(
            self.get_parameter("control_reset_emergency_service").value
        )

        self.arm_num_joints = max(1, int(self.get_parameter("arm_num_joints").value))
        self.arm_state_timeout_sec = max(0.1, float(self.get_parameter("arm_state_timeout_sec").value))
        self.command_timeout_sec = max(0.2, float(self.get_parameter("command_timeout_sec").value))
        self.client_retry_count = max(0, int(self.get_parameter("client_retry_count").value))
        self.emergency_latch_level = int(self.get_parameter("emergency_latch_level").value)
        self.reject_on_arm_error = bool(self.get_parameter("reject_on_arm_error").value)
        self.enforce_angle_limits = bool(self.get_parameter("enforce_angle_limits").value)
        self.min_angle_deg = float(self.get_parameter("min_angle_deg").value)
        self.max_angle_deg = float(self.get_parameter("max_angle_deg").value)
        if self.min_angle_deg > self.max_angle_deg:
            self.min_angle_deg, self.max_angle_deg = self.max_angle_deg, self.min_angle_deg

        self._service_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            ArmState, self.arm_state_topic, self._on_arm_state, state_qos, callback_group=self._service_group
        )
        self.create_subscription(
            SystemHealth, self.health_topic, self._on_health, state_qos, callback_group=self._service_group
        )

        self._enable_client = self.create_client(
            SetBool, self.hardware_enable_service, callback_group=self._client_group
        )
        self._home_client = self.create_client(
            Trigger, self.hardware_home_service, callback_group=self._client_group
        )
        self._move_joint_client = self.create_client(
            MoveArmJoint, self.hardware_move_joint_service, callback_group=self._client_group
        )
        self._move_joints_client = self.create_client(
            MoveArmJointsSrv, self.hardware_move_joints_service, callback_group=self._client_group
        )

        self.create_service(
            SetBool, self.control_enable_service, self._on_control_enable, callback_group=self._service_group
        )
        self.create_service(
            Trigger, self.control_home_service, self._on_control_home, callback_group=self._service_group
        )
        self.create_service(
            MoveArmJoint,
            self.control_move_joint_service,
            self._on_control_move_joint,
            callback_group=self._service_group,
        )
        self.create_service(
            MoveArmJointsSrv,
            self.control_move_joints_service,
            self._on_control_move_joints,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            self.control_reset_emergency_service,
            self._on_reset_emergency,
            callback_group=self._service_group,
        )

        self._action_server = ActionServer(
            self,
            MoveArmJointsAction,
            self.control_move_joints_action,
            execute_callback=self._execute_move_joints_action,
            goal_callback=self._on_action_goal,
            cancel_callback=self._on_action_cancel,
            callback_group=self._action_group,
        )

        self._latest_arm_state: Optional[ArmState] = None
        self._latest_arm_state_ts = 0.0
        self._state_lock = threading.Lock()

        self._emergency_latched = False
        self._emergency_reason = ""
        self._emergency_lock = threading.Lock()

        self.get_logger().info(
            "arm_control_node started: "
            f"proxying to [{self.hardware_enable_service}, {self.hardware_home_service}, "
            f"{self.hardware_move_joint_service}, {self.hardware_move_joints_service}]"
        )

    def _on_arm_state(self, msg: ArmState) -> None:
        with self._state_lock:
            self._latest_arm_state = msg
            self._latest_arm_state_ts = time.time()

    def _on_health(self, msg: SystemHealth) -> None:
        if msg.node_name not in ("arm_driver_node", "arm_sim_driver_node"):
            return
        if msg.level < self.emergency_latch_level:
            return
        if msg.healthy:
            return
        with self._emergency_lock:
            self._emergency_latched = True
            self._emergency_reason = f"{msg.node_name}: level={msg.level}, msg={msg.message}"

    def _is_emergency_latched(self) -> Tuple[bool, str]:
        with self._emergency_lock:
            return self._emergency_latched, self._emergency_reason

    def _check_arm_ready(self) -> Tuple[bool, str]:
        with self._state_lock:
            state = self._latest_arm_state
            ts = self._latest_arm_state_ts

        if state is None:
            return False, "arm state not received (check arm_driver_node/arm_sim_driver_node and /arm/state)"
        if time.time() - ts > self.arm_state_timeout_sec:
            return False, "arm state is stale"
        if not state.connected:
            return False, "arm is not connected"
        if self.reject_on_arm_error and state.error:
            return False, f"arm reports error: {state.error_message}"
        return True, ""

    def _wait_arm_ready(self, timeout_sec: float) -> Tuple[bool, str]:
        deadline = time.time() + max(0.2, float(timeout_sec))
        last_message = (
            "arm state not received (check arm_driver_node/arm_sim_driver_node and /arm/state)"
        )
        while time.time() < deadline:
            ok, message = self._check_arm_ready()
            if ok:
                return True, ""
            if message:
                last_message = message
            time.sleep(0.05)
        return False, last_message

    def _ensure_arm_ready(self) -> Tuple[bool, str]:
        ok, message = self._check_arm_ready()
        if ok:
            return True, ""

        req = SetBool.Request()
        req.data = True
        result, error = self._call_service_with_retries(self._enable_client, req)
        if result is None:
            return False, f"enable call failed: {error}"
        if not bool(result.success):
            return False, str(result.message or "enable failed")

        with self._emergency_lock:
            self._emergency_latched = False
            self._emergency_reason = ""

        ready_ok, ready_message = self._wait_arm_ready(
            max(self.command_timeout_sec, self.arm_state_timeout_sec + 2.0)
        )
        if not ready_ok:
            return False, ready_message
        return True, ""

    def _validate_joint_id(self, joint_id: int) -> Tuple[bool, str]:
        if joint_id < 1 or joint_id > self.arm_num_joints:
            return False, f"joint_id {joint_id} out of range [1..{self.arm_num_joints}]"
        return True, ""

    def _validate_angle(self, angle_deg: float) -> Tuple[bool, str]:
        if not math.isfinite(angle_deg):
            return False, "angle_deg is not finite"
        if self.enforce_angle_limits and (
            angle_deg < self.min_angle_deg or angle_deg > self.max_angle_deg
        ):
            return (
                False,
                f"angle_deg {angle_deg} out of range [{self.min_angle_deg}..{self.max_angle_deg}]",
            )
        return True, ""

    def _call_service_once(self, client, request):
        if not client.wait_for_service(timeout_sec=self.command_timeout_sec):
            return None, "target service unavailable"

        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.command_timeout_sec):
            return None, "target service timeout"

        try:
            return future.result(), ""
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

    def _call_service_with_retries(self, client, request):
        last_error = ""
        for _ in range(self.client_retry_count + 1):
            result, error = self._call_service_once(client, request)
            if result is not None:
                return result, ""
            last_error = error
        return None, last_error or "service call failed"

    def _proxy_move_joint(
        self, joint_id: int, angle_deg: float, duration_ms: int, wait: bool
    ) -> Tuple[bool, str, int, float]:
        latched, reason = self._is_emergency_latched()
        if latched:
            return False, f"emergency latched: {reason}", joint_id, angle_deg

        ok, message = self._ensure_arm_ready()
        if not ok:
            return False, message, joint_id, angle_deg

        ok, message = self._validate_joint_id(joint_id)
        if not ok:
            return False, message, joint_id, angle_deg

        ok, message = self._validate_angle(angle_deg)
        if not ok:
            return False, message, joint_id, angle_deg

        req = MoveArmJoint.Request()
        req.joint_id = joint_id
        req.angle_deg = float(angle_deg)
        req.duration_ms = int(duration_ms)
        req.wait = bool(wait)

        result, error = self._call_service_with_retries(self._move_joint_client, req)
        if result is None:
            return False, f"move_joint call failed: {error}", joint_id, angle_deg
        return bool(result.success), str(result.message), int(result.joint_id), float(
            result.commanded_angle_deg
        )

    def _proxy_move_joints(
        self, joint_ids: List[int], angles_deg: List[float], duration_ms: int, wait: bool
    ) -> Tuple[bool, str, List[int], List[float]]:
        latched, reason = self._is_emergency_latched()
        if latched:
            return False, f"emergency latched: {reason}", joint_ids, angles_deg

        ok, message = self._ensure_arm_ready()
        if not ok:
            return False, message, joint_ids, angles_deg

        if not joint_ids:
            return False, "joint_ids is empty", joint_ids, angles_deg
        if len(joint_ids) != len(angles_deg):
            return False, "joint_ids and angles_deg size mismatch", joint_ids, angles_deg
        if len(set(joint_ids)) != len(joint_ids):
            return False, "joint_ids contains duplicates", joint_ids, angles_deg

        for joint_id, angle_deg in zip(joint_ids, angles_deg):
            ok, message = self._validate_joint_id(joint_id)
            if not ok:
                return False, message, joint_ids, angles_deg
            ok, message = self._validate_angle(angle_deg)
            if not ok:
                return False, message, joint_ids, angles_deg

        req = MoveArmJointsSrv.Request()
        req.joint_ids = [int(v) for v in joint_ids]
        req.angles_deg = [float(v) for v in angles_deg]
        req.duration_ms = int(duration_ms)
        req.wait = bool(wait)

        result, error = self._call_service_with_retries(self._move_joints_client, req)
        if result is None:
            return False, f"move_joints call failed: {error}", joint_ids, angles_deg
        return (
            bool(result.success),
            str(result.message),
            [int(v) for v in list(result.joint_ids)],
            [float(v) for v in list(result.commanded_angles_deg)],
        )

    def _on_control_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        req = SetBool.Request()
        req.data = bool(request.data)
        result, error = self._call_service_with_retries(self._enable_client, req)
        if result is None:
            response.success = False
            response.message = f"enable call failed: {error}"
            return response

        response.success = bool(result.success)
        response.message = str(result.message)
        if response.success and request.data:
            with self._emergency_lock:
                self._emergency_latched = False
                self._emergency_reason = ""
            ready_ok, ready_message = self._wait_arm_ready(
                max(self.command_timeout_sec, self.arm_state_timeout_sec + 2.0)
            )
            if not ready_ok:
                response.success = False
                response.message = f"enable succeeded but arm not ready: {ready_message}"
        return response

    def _on_control_home(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._ensure_arm_ready()
        if not ok:
            response.success = False
            response.message = message
            return response
        result, error = self._call_service_with_retries(self._home_client, Trigger.Request())
        if result is None:
            response.success = False
            response.message = f"home call failed: {error}"
            return response
        response.success = bool(result.success)
        response.message = str(result.message)
        return response

    def _on_control_move_joint(
        self, request: MoveArmJoint.Request, response: MoveArmJoint.Response
    ) -> MoveArmJoint.Response:
        success, message, joint_id, commanded_angle = self._proxy_move_joint(
            joint_id=int(request.joint_id),
            angle_deg=float(request.angle_deg),
            duration_ms=int(request.duration_ms),
            wait=bool(request.wait),
        )
        response.success = bool(success)
        response.message = str(message)
        response.joint_id = int(joint_id)
        response.commanded_angle_deg = float(commanded_angle)
        return response

    def _on_control_move_joints(
        self, request: MoveArmJointsSrv.Request, response: MoveArmJointsSrv.Response
    ) -> MoveArmJointsSrv.Response:
        success, message, joint_ids, angles_deg = self._proxy_move_joints(
            joint_ids=[int(v) for v in list(request.joint_ids)],
            angles_deg=[float(v) for v in list(request.angles_deg)],
            duration_ms=int(request.duration_ms),
            wait=bool(request.wait),
        )
        response.success = bool(success)
        response.message = str(message)
        response.joint_ids = [int(v) for v in joint_ids]
        response.commanded_angles_deg = [float(v) for v in angles_deg]
        return response

    def _on_reset_emergency(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._emergency_lock:
            was_latched = self._emergency_latched
            self._emergency_latched = False
            self._emergency_reason = ""
        response.success = True
        response.message = "emergency latch cleared" if was_latched else "emergency latch already clear"
        return response

    def _on_action_goal(self, goal_request: MoveArmJointsAction.Goal) -> GoalResponse:
        latched, _ = self._is_emergency_latched()
        if latched:
            return GoalResponse.REJECT
        ids = [int(v) for v in list(goal_request.joint_ids)]
        angles = [float(v) for v in list(goal_request.angles_deg)]
        if not ids or len(ids) != len(angles):
            return GoalResponse.REJECT
        if len(set(ids)) != len(ids):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_action_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_move_joints_action(self, goal_handle):
        goal = goal_handle.request

        feedback = MoveArmJointsAction.Feedback()
        feedback.progress = 0.1
        feedback.status = "validating request"
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = MoveArmJointsAction.Result()
            result.success = False
            result.message = "goal canceled before execution"
            result.executed_joint_ids = []
            result.executed_angles_deg = []
            return result

        success, message, joint_ids, angles_deg = self._proxy_move_joints(
            joint_ids=[int(v) for v in list(goal.joint_ids)],
            angles_deg=[float(v) for v in list(goal.angles_deg)],
            duration_ms=int(goal.duration_ms),
            wait=bool(goal.wait),
        )

        feedback.progress = 1.0 if success else 0.9
        feedback.status = "completed" if success else "failed"
        goal_handle.publish_feedback(feedback)

        result = MoveArmJointsAction.Result()
        result.success = bool(success)
        result.message = str(message)
        result.executed_joint_ids = [int(v) for v in joint_ids]
        result.executed_angles_deg = [float(v) for v in angles_deg]

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def destroy_node(self) -> bool:
        try:
            self._action_server.destroy()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmControlNode()
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
