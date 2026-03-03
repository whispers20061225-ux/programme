from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from tactile_interfaces.action import ExecuteDemo
from tactile_interfaces.srv import MoveArmJoints


class DemoTaskNode(Node):
    """Phase 5 task orchestrator.

    Exposes:
    - Action: /task/execute_demo
    - Service: /task/pause_demo
    - Service: /task/resume_demo
    - Service: /task/stop_demo
    """

    _ARM_SEQUENCE_DEFAULT_DEMOS = {
        "grasping",
        "auto_grasp",
        "grasp_demo",
    }

    def __init__(self) -> None:
        super().__init__("demo_task_node")

        self.declare_parameter("execute_demo_action", "/task/execute_demo")
        self.declare_parameter("pause_demo_service", "/task/pause_demo")
        self.declare_parameter("resume_demo_service", "/task/resume_demo")
        self.declare_parameter("stop_demo_service", "/task/stop_demo")

        self.declare_parameter("control_enable_service", "/control/arm/enable")
        self.declare_parameter("control_home_service", "/control/arm/home")
        self.declare_parameter("control_move_joints_service", "/control/arm/move_joints")

        self.declare_parameter("default_duration_sec", 15.0)
        self.declare_parameter("max_duration_sec", 120.0)
        self.declare_parameter("feedback_rate_hz", 10.0)
        self.declare_parameter("command_timeout_sec", 5.0)
        self.declare_parameter("auto_prepare_arm", True)
        self.declare_parameter("default_gripper_joint_id", 6)
        self.declare_parameter("default_open_angle_deg", 85.0)
        self.declare_parameter("default_close_angle_deg", 35.0)
        self.declare_parameter("default_motion_duration_ms", 800)

        self.execute_demo_action = str(self.get_parameter("execute_demo_action").value)
        self.pause_demo_service = str(self.get_parameter("pause_demo_service").value)
        self.resume_demo_service = str(self.get_parameter("resume_demo_service").value)
        self.stop_demo_service = str(self.get_parameter("stop_demo_service").value)

        self.control_enable_service = str(self.get_parameter("control_enable_service").value)
        self.control_home_service = str(self.get_parameter("control_home_service").value)
        self.control_move_joints_service = str(
            self.get_parameter("control_move_joints_service").value
        )

        self.default_duration_sec = max(1.0, float(self.get_parameter("default_duration_sec").value))
        self.max_duration_sec = max(self.default_duration_sec, float(self.get_parameter("max_duration_sec").value))
        self.feedback_rate_hz = max(1.0, float(self.get_parameter("feedback_rate_hz").value))
        self.command_timeout_sec = max(0.2, float(self.get_parameter("command_timeout_sec").value))
        self.auto_prepare_arm = bool(self.get_parameter("auto_prepare_arm").value)
        self.default_gripper_joint_id = max(1, int(self.get_parameter("default_gripper_joint_id").value))
        self.default_open_angle_deg = float(self.get_parameter("default_open_angle_deg").value)
        self.default_close_angle_deg = float(self.get_parameter("default_close_angle_deg").value)
        self.default_motion_duration_ms = max(50, int(self.get_parameter("default_motion_duration_ms").value))

        self._service_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        self._enable_client = self.create_client(
            SetBool, self.control_enable_service, callback_group=self._client_group
        )
        self._home_client = self.create_client(
            Trigger, self.control_home_service, callback_group=self._client_group
        )
        self._move_joints_client = self.create_client(
            MoveArmJoints, self.control_move_joints_service, callback_group=self._client_group
        )

        self.create_service(
            Trigger,
            self.pause_demo_service,
            self._on_pause_demo,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            self.resume_demo_service,
            self._on_resume_demo,
            callback_group=self._service_group,
        )
        self.create_service(
            Trigger,
            self.stop_demo_service,
            self._on_stop_demo,
            callback_group=self._service_group,
        )

        self._action_server = ActionServer(
            self,
            ExecuteDemo,
            self.execute_demo_action,
            execute_callback=self._execute_demo,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._action_group,
        )

        self._state_lock = threading.Lock()
        self._goal_reserved = False
        self._active_goal = None
        self._active_demo_name = ""
        self._paused = False
        self._stop_requested = False

        self.get_logger().info(
            "demo_task_node started: action=%s, services=[%s, %s, %s]",
            self.execute_demo_action,
            self.pause_demo_service,
            self.resume_demo_service,
            self.stop_demo_service,
        )

    def _on_goal(self, goal_request: ExecuteDemo.Goal) -> GoalResponse:
        with self._state_lock:
            if self._goal_reserved or self._active_goal is not None:
                return GoalResponse.REJECT
            self._goal_reserved = True
        demo_name = str(goal_request.demo_name or "").strip()
        if not demo_name:
            with self._state_lock:
                self._goal_reserved = False
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        with self._state_lock:
            self._stop_requested = True
        return CancelResponse.ACCEPT

    def _on_pause_demo(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            if self._active_goal is None:
                response.success = False
                response.message = "no active demo"
                return response
            self._paused = True
        response.success = True
        response.message = "demo paused"
        return response

    def _on_resume_demo(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            if self._active_goal is None:
                response.success = False
                response.message = "no active demo"
                return response
            self._paused = False
        response.success = True
        response.message = "demo resumed"
        return response

    def _on_stop_demo(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            if self._active_goal is None:
                response.success = False
                response.message = "no active demo"
                return response
            self._stop_requested = True
        response.success = True
        response.message = "stop requested"
        return response

    def _execute_demo(self, goal_handle):
        goal = goal_handle.request
        demo_name = str(goal.demo_name or "").strip()
        params = self._parse_params(goal.params_json)
        duration_sec = self._resolve_duration(goal.duration_sec, params)

        with self._state_lock:
            self._goal_reserved = False
            self._active_goal = goal_handle
            self._active_demo_name = demo_name
            self._paused = False
            self._stop_requested = False

        self.get_logger().info("execute demo: name=%s duration=%.2fs", demo_name, duration_sec)

        try:
            feedback = ExecuteDemo.Feedback()
            feedback.progress = 0.0
            feedback.current_state = "starting"
            feedback.message = "demo accepted"
            goal_handle.publish_feedback(feedback)

            prepare_arm = bool(params.get("prepare_arm", self.auto_prepare_arm))
            if prepare_arm:
                ok, message = self._prepare_arm(params)
                if not ok:
                    return self._finish_goal(
                        goal_handle,
                        success=False,
                        message=f"prepare arm failed: {message}",
                        final_state="failed",
                    )

            success, message, final_state = self._run_demo_loop(
                goal_handle=goal_handle,
                demo_name=demo_name,
                params=params,
                duration_sec=duration_sec,
            )
            return self._finish_goal(
                goal_handle,
                success=success,
                message=message,
                final_state=final_state,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().exception("execute demo crashed")
            return self._finish_goal(
                goal_handle,
                success=False,
                message=f"task node exception: {exc}",
                final_state="failed",
            )
        finally:
            with self._state_lock:
                self._goal_reserved = False
                self._active_goal = None
                self._active_demo_name = ""
                self._paused = False
                self._stop_requested = False

    def _prepare_arm(self, params: Dict[str, Any]) -> Tuple[bool, str]:
        req_enable = SetBool.Request()
        req_enable.data = True
        result, error = self._call_service(self._enable_client, req_enable, self.command_timeout_sec)
        if result is None:
            return False, error
        if not bool(result.success):
            return False, str(result.message)

        if not bool(params.get("home_before_start", True)):
            return True, "arm enabled"

        result, error = self._call_service(self._home_client, Trigger.Request(), self.command_timeout_sec)
        if result is None:
            return False, error
        if not bool(result.success):
            return False, str(result.message)
        return True, str(result.message or "arm enabled and homed")

    def _run_demo_loop(
        self,
        goal_handle,
        demo_name: str,
        params: Dict[str, Any],
        duration_sec: float,
    ) -> Tuple[bool, str, str]:
        use_arm_sequence = bool(
            params.get("use_arm_sequence", demo_name in self._ARM_SEQUENCE_DEFAULT_DEMOS)
        )
        tick_sec = 1.0 / self.feedback_rate_hz
        start_ts = time.time()
        end_ts = start_ts + duration_sec

        open_sent = False
        close_sent = False
        paused_reported = False

        while True:
            now = time.time()
            progress = 1.0 if duration_sec <= 0.0 else min(1.0, max(0.0, (now - start_ts) / duration_sec))

            if goal_handle.is_cancel_requested:
                return False, "goal canceled by client", "canceled"

            paused, stop_requested = self._get_runtime_flags()
            if stop_requested:
                return False, "demo stopped by service request", "stopped"

            if paused:
                if not paused_reported:
                    self._publish_feedback(goal_handle, progress, "paused", "demo paused")
                    paused_reported = True
                time.sleep(min(0.1, tick_sec))
                continue

            if paused_reported:
                paused_reported = False
                self._publish_feedback(goal_handle, progress, "running", "demo resumed")

            if use_arm_sequence:
                if (not open_sent) and progress >= 0.20:
                    ok, message = self._send_arm_pose("open", params)
                    if not ok:
                        return False, f"open pose failed: {message}", "failed"
                    open_sent = True
                if (not close_sent) and progress >= 0.70:
                    ok, message = self._send_arm_pose("close", params)
                    if not ok:
                        return False, f"close pose failed: {message}", "failed"
                    close_sent = True

            self._publish_feedback(
                goal_handle,
                progress,
                "running" if progress < 1.0 else "finishing",
                f"{demo_name} running",
            )

            if now >= end_ts:
                break
            time.sleep(tick_sec)

        self._publish_feedback(goal_handle, 1.0, "completed", f"{demo_name} completed")
        return True, "demo completed", "completed"

    def _send_arm_pose(self, pose: str, params: Dict[str, Any]) -> Tuple[bool, str]:
        joint_ids_raw = params.get(f"{pose}_joint_ids")
        angles_raw = params.get(f"{pose}_angles_deg")

        if isinstance(joint_ids_raw, list) and isinstance(angles_raw, list):
            joint_ids = [int(v) for v in joint_ids_raw]
            angles = [float(v) for v in angles_raw]
        else:
            angle_key = f"{pose}_angle_deg"
            default_angle = self.default_open_angle_deg if pose == "open" else self.default_close_angle_deg
            joint_ids = [self.default_gripper_joint_id]
            angles = [float(params.get(angle_key, default_angle))]

        if len(joint_ids) != len(angles) or not joint_ids:
            return False, "invalid pose parameters"

        duration_ms = int(params.get(f"{pose}_duration_ms", params.get("motion_duration_ms", self.default_motion_duration_ms)))
        wait = bool(params.get("wait_for_motion", True))

        req = MoveArmJoints.Request()
        req.joint_ids = [int(v) for v in joint_ids]
        req.angles_deg = [float(v) for v in angles]
        req.duration_ms = max(50, int(duration_ms))
        req.wait = wait

        result, error = self._call_service(self._move_joints_client, req, self.command_timeout_sec)
        if result is None:
            return False, error
        return bool(result.success), str(result.message)

    def _finish_goal(self, goal_handle, success: bool, message: str, final_state: str) -> ExecuteDemo.Result:
        result = ExecuteDemo.Result()
        result.success = bool(success)
        result.message = str(message)
        result.final_state = str(final_state)

        if final_state == "canceled":
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _publish_feedback(
        self,
        goal_handle,
        progress: float,
        current_state: str,
        message: str,
    ) -> None:
        feedback = ExecuteDemo.Feedback()
        feedback.progress = float(max(0.0, min(1.0, progress)))
        feedback.current_state = str(current_state)
        feedback.message = str(message)
        goal_handle.publish_feedback(feedback)

    def _parse_params(self, params_json: str) -> Dict[str, Any]:
        if not params_json:
            return {}
        try:
            parsed = json.loads(params_json)
            if isinstance(parsed, dict):
                return parsed
            return {}
        except Exception:
            return {}

    def _resolve_duration(self, goal_duration_sec: float, params: Dict[str, Any]) -> float:
        duration = float(goal_duration_sec)
        if duration <= 0.0:
            duration = float(params.get("duration", self.default_duration_sec))
        return max(1.0, min(self.max_duration_sec, duration))

    def _get_runtime_flags(self) -> Tuple[bool, bool]:
        with self._state_lock:
            return bool(self._paused), bool(self._stop_requested)

    @staticmethod
    def _call_service(client, request, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None, "service unavailable"

        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_sec):
            return None, "service timeout"
        try:
            return future.result(), ""
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

    def destroy_node(self) -> bool:
        try:
            self._action_server.destroy()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoTaskNode()
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
