from __future__ import annotations

import json
import threading
import time
from typing import Any
from uuid import uuid4

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from tactile_interfaces.action import ExecuteTask, SearchTarget
from tactile_interfaces.msg import SemanticTask, TaskExecutionStatus, TaskGoal


def _safe_json_loads(raw_text: str) -> dict[str, Any]:
    try:
        parsed = json.loads(raw_text)
    except Exception:
        return {"raw": raw_text}
    if isinstance(parsed, dict):
        return parsed
    return {"value": parsed}


class TaskExecutiveNode(Node):
    """Single owner for pick-task execution orchestration."""

    def __init__(self) -> None:
        super().__init__("task_executive_node")

        self.declare_parameter("execute_task_action", "/task/execute_task")
        self.declare_parameter("task_goal_topic", "/task/goal")
        self.declare_parameter("task_execution_status_topic", "/task/execution_status")
        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("target_pose_topic", "/sim/perception/target_pose")
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter(
            "target_candidate_visible_topic", "/sim/perception/target_candidate_visible"
        )
        self.declare_parameter("pick_active_topic", "/sim/task/pick_active")
        self.declare_parameter("pick_status_topic", "/sim/task/pick_status")
        self.declare_parameter("execute_pick_service", "/task/execute_pick")
        self.declare_parameter("reset_pick_session_service", "/task/reset_pick_session")
        self.declare_parameter("return_home_service", "/task/return_home")
        self.declare_parameter("start_search_sweep_service", "/task/start_search_sweep")
        self.declare_parameter("search_target_action", "/task/search_target")
        self.declare_parameter("prefer_search_target_action", False)
        self.declare_parameter("search_sweep_start_delay_sec", 1.0)
        self.declare_parameter("visible_target_lock_wait_sec", 2.0)
        self.declare_parameter("loop_sleep_sec", 0.25)
        self.declare_parameter("target_lock_timeout_sec", 18.0)
        self.declare_parameter("execution_timeout_sec", 45.0)
        self.declare_parameter("service_timeout_sec", 2.0)
        self.declare_parameter("target_pose_fresh_timeout_sec", 1.5)
        self.declare_parameter("allow_latched_target_pose_when_locked", True)
        self.declare_parameter("allow_latched_target_pose_when_candidate_visible", True)
        self.declare_parameter("recovery_return_home_on_failure", False)

        self.execute_task_action = str(self.get_parameter("execute_task_action").value)
        self.task_goal_topic = str(self.get_parameter("task_goal_topic").value)
        self.task_execution_status_topic = str(
            self.get_parameter("task_execution_status_topic").value
        )
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.target_candidate_visible_topic = str(
            self.get_parameter("target_candidate_visible_topic").value
        )
        self.pick_active_topic = str(self.get_parameter("pick_active_topic").value)
        self.pick_status_topic = str(self.get_parameter("pick_status_topic").value)
        self.execute_pick_service = str(self.get_parameter("execute_pick_service").value)
        self.reset_pick_session_service = str(
            self.get_parameter("reset_pick_session_service").value
        )
        self.return_home_service = str(self.get_parameter("return_home_service").value)
        self.start_search_sweep_service = str(
            self.get_parameter("start_search_sweep_service").value
        )
        self.search_target_action = str(self.get_parameter("search_target_action").value)
        self.prefer_search_target_action = bool(
            self.get_parameter("prefer_search_target_action").value
        )
        self.loop_sleep_sec = max(0.05, float(self.get_parameter("loop_sleep_sec").value))
        self.search_sweep_start_delay_sec = max(
            0.0, float(self.get_parameter("search_sweep_start_delay_sec").value)
        )
        self.visible_target_lock_wait_sec = max(
            self.loop_sleep_sec, float(self.get_parameter("visible_target_lock_wait_sec").value)
        )
        self.target_lock_timeout_sec = max(
            self.loop_sleep_sec, float(self.get_parameter("target_lock_timeout_sec").value)
        )
        self.execution_timeout_sec = max(
            self.target_lock_timeout_sec,
            float(self.get_parameter("execution_timeout_sec").value),
        )
        self.service_timeout_sec = max(
            self.loop_sleep_sec, float(self.get_parameter("service_timeout_sec").value)
        )
        self.target_pose_fresh_timeout_sec = max(
            self.loop_sleep_sec, float(self.get_parameter("target_pose_fresh_timeout_sec").value)
        )
        self.allow_latched_target_pose_when_locked = bool(
            self.get_parameter("allow_latched_target_pose_when_locked").value
        )
        self.allow_latched_target_pose_when_candidate_visible = bool(
            self.get_parameter("allow_latched_target_pose_when_candidate_visible").value
        )
        self.recovery_return_home_on_failure = bool(
            self.get_parameter("recovery_return_home_on_failure").value
        )

        self._service_group = ReentrantCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.task_goal_pub = self.create_publisher(TaskGoal, self.task_goal_topic, qos_latched)
        self.task_status_pub = self.create_publisher(
            TaskExecutionStatus, self.task_execution_status_topic, qos_latched
        )
        self.semantic_task_pub = self.create_publisher(
            SemanticTask, self.semantic_task_topic, qos_reliable
        )

        self.execute_pick_client = self.create_client(
            Trigger,
            self.execute_pick_service,
            callback_group=self._service_group,
        )
        self.reset_pick_client = self.create_client(
            Trigger,
            self.reset_pick_session_service,
            callback_group=self._service_group,
        )
        self.return_home_client = self.create_client(
            Trigger,
            self.return_home_service,
            callback_group=self._service_group,
        )
        self.start_search_sweep_client = self.create_client(
            Trigger,
            self.start_search_sweep_service,
            callback_group=self._service_group,
        )

        self.create_subscription(Bool, self.target_locked_topic, self._on_target_locked, qos_reliable)
        self.create_subscription(
            Bool,
            self.target_candidate_visible_topic,
            self._on_target_candidate_visible,
            qos_reliable,
        )
        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._on_target_pose,
            qos_latched,
        )
        self.create_subscription(Bool, self.pick_active_topic, self._on_pick_active, qos_reliable)
        self.create_subscription(String, self.pick_status_topic, self._on_pick_status, qos_reliable)

        self._action_server = ActionServer(
            self,
            ExecuteTask,
            self.execute_task_action,
            execute_callback=self._execute_task,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._action_group,
        )
        self.search_target_client = ActionClient(
            self,
            SearchTarget,
            self.search_target_action,
            callback_group=self._action_group,
        )

        self._state_lock = threading.Lock()
        self._goal_reserved = False
        self._active_goal = None
        self._has_target_pose = False
        self._latest_target_pose_update = 0.0
        self._target_locked = False
        self._target_candidate_visible = False
        self._pick_active = False
        self._pick_status: dict[str, Any] = {
            "phase": "idle",
            "message": "waiting for task goal",
            "updated_at": 0.0,
        }

        self._publish_status(
            self._make_status(
                TaskGoal(),
                phase="idle",
                current_skill="idle",
                message="waiting for task goal",
                active=False,
                success=False,
                progress=0.0,
            )
        )
        self.get_logger().info(
            "task_executive_node started: "
            f"action={self.execute_task_action} "
            f"goal_topic={self.task_goal_topic} "
            f"status_topic={self.task_execution_status_topic}"
        )

    def _on_goal(self, goal_request: ExecuteTask.Goal) -> GoalResponse:
        with self._state_lock:
            if self._goal_reserved or self._active_goal is not None:
                return GoalResponse.REJECT
            self._goal_reserved = True
        goal = goal_request.goal
        target_name = str(goal.target_label or goal.target_hint or "").strip()
        if not target_name:
            with self._state_lock:
                self._goal_reserved = False
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _on_target_locked(self, msg: Bool) -> None:
        with self._state_lock:
            self._target_locked = bool(msg.data)

    def _on_target_candidate_visible(self, msg: Bool) -> None:
        with self._state_lock:
            self._target_candidate_visible = bool(msg.data)

    def _on_target_pose(self, _msg: PoseStamped) -> None:
        with self._state_lock:
            self._has_target_pose = True
            self._latest_target_pose_update = time.time()

    def _on_pick_active(self, msg: Bool) -> None:
        with self._state_lock:
            self._pick_active = bool(msg.data)

    def _on_pick_status(self, msg: String) -> None:
        parsed = _safe_json_loads(str(msg.data or ""))
        parsed["raw"] = str(msg.data or "")
        parsed["updated_at"] = time.time()
        with self._state_lock:
            self._pick_status = parsed

    def _snapshot_runtime(self) -> tuple[bool, bool, bool, bool, dict[str, Any]]:
        with self._state_lock:
            pose_age_sec = (
                time.time() - self._latest_target_pose_update
                if self._latest_target_pose_update > 0.0
                else float("inf")
            )
            has_latched_pose = bool(self._has_target_pose)
            pose_fresh = bool(has_latched_pose and pose_age_sec <= self.target_pose_fresh_timeout_sec)
            pose_ready = pose_fresh
            if not pose_ready and has_latched_pose:
                if self.allow_latched_target_pose_when_locked and self._target_locked:
                    pose_ready = True
                elif (
                    self.allow_latched_target_pose_when_candidate_visible
                    and self._target_candidate_visible
                ):
                    pose_ready = True
            return (
                bool(pose_ready),
                bool(self._target_locked),
                bool(self._target_candidate_visible),
                bool(self._pick_active),
                dict(self._pick_status),
            )

    def _call_trigger(self, client: Any, timeout_sec: float | None = None) -> tuple[bool, str]:
        resolved_timeout = self.service_timeout_sec if timeout_sec is None else max(
            self.loop_sleep_sec, float(timeout_sec)
        )
        if not client.wait_for_service(timeout_sec=resolved_timeout):
            return False, "service unavailable"
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(resolved_timeout):
            return False, "service timed out"
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        return bool(result.success), str(result.message or "")

    def _normalize_goal(self, goal: TaskGoal) -> TaskGoal:
        goal_msg = TaskGoal()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.schema_version = str(goal.schema_version or "").strip() or "task_goal.v2alpha1"
        goal_msg.goal_id = str(goal.goal_id or uuid4().hex[:12]).strip()
        goal_msg.parent_goal_id = str(goal.parent_goal_id or "").strip()
        goal_msg.step_id = str(goal.step_id or "").strip() or "pick"
        goal_msg.step_index = int(goal.step_index or 0)
        goal_msg.goal_type = str(goal.goal_type or "").strip() or (
            "pick_and_place" if str(goal.placement_label or "").strip() else "pick"
        )
        goal_msg.task = str(goal.task or "pick").strip() or "pick"
        goal_msg.target_label = str(goal.target_label or "").strip()
        goal_msg.target_hint = str(goal.target_hint or goal_msg.target_label).strip()
        goal_msg.target_attributes = [
            str(item).strip() for item in list(goal.target_attributes) if str(item).strip()
        ]
        goal_msg.target_relations = [
            str(item).strip() for item in list(goal.target_relations) if str(item).strip()
        ]
        goal_msg.target_instance_track_id = int(goal.target_instance_track_id or 0)
        goal_msg.target_instance_bbox_xyxy = [int(item) for item in list(goal.target_instance_bbox_xyxy)]
        goal_msg.target_instance_point_px = [int(item) for item in list(goal.target_instance_point_px)]
        goal_msg.target_instance_source = str(goal.target_instance_source or "").strip()
        goal_msg.target_part_query = str(goal.target_part_query or "").strip()
        goal_msg.preferred_part_tags = [
            str(item).strip() for item in list(goal.preferred_part_tags) if str(item).strip()
        ]
        goal_msg.forbidden_part_tags = [
            str(item).strip() for item in list(goal.forbidden_part_tags) if str(item).strip()
        ]
        goal_msg.grasp_region = str(goal.grasp_region or "").strip()
        goal_msg.preferred_grasp_family = str(goal.preferred_grasp_family or "").strip()
        goal_msg.placement_label = str(goal.placement_label or "").strip()
        goal_msg.preferred_approach_dirs = [
            str(item).strip() for item in list(goal.preferred_approach_dirs) if str(item).strip()
        ]
        goal_msg.forbidden_approach_dirs = [
            str(item).strip() for item in list(goal.forbidden_approach_dirs) if str(item).strip()
        ]
        goal_msg.transport_constraints = [
            str(item).strip() for item in list(goal.transport_constraints) if str(item).strip()
        ]
        goal_msg.execution_constraints = [
            str(item).strip() for item in list(goal.execution_constraints) if str(item).strip()
        ]
        goal_msg.excluded_labels = [
            str(item).strip() for item in list(goal.excluded_labels) if str(item).strip()
        ]
        goal_msg.reserved_next_skills = [
            str(item).strip() for item in list(goal.reserved_next_skills) if str(item).strip()
        ]
        goal_msg.confidence = float(goal.confidence or 0.0)
        goal_msg.min_affordance_score = max(0.0, float(goal.min_affordance_score or 0.0))
        goal_msg.min_grasp_quality = max(0.0, float(goal.min_grasp_quality or 0.0))
        goal_msg.max_forbidden_overlap = max(0.0, float(goal.max_forbidden_overlap or 0.0))
        goal_msg.max_target_staleness_sec = max(
            0.0, float(goal.max_target_staleness_sec or 0.0)
        )
        goal_msg.need_human_confirm = bool(goal.need_human_confirm)
        goal_msg.start_search_sweep = bool(goal.start_search_sweep)
        goal_msg.allow_instance_grounding = bool(goal.allow_instance_grounding)
        goal_msg.require_target_lock = bool(goal.require_target_lock)
        goal_msg.allow_replan = bool(goal.allow_replan)
        goal_msg.allow_rescan = bool(goal.allow_rescan)
        goal_msg.strict_part_match = bool(goal.strict_part_match)
        goal_msg.strict_approach_match = bool(goal.strict_approach_match)
        goal_msg.max_retries = max(0, int(goal.max_retries or 0))
        goal_msg.planning_timeout_sec = max(
            self.loop_sleep_sec,
            float(goal.planning_timeout_sec or self.target_lock_timeout_sec),
        )
        goal_msg.execution_timeout_sec = max(
            goal_msg.planning_timeout_sec,
            float(goal.execution_timeout_sec or self.execution_timeout_sec),
        )
        goal_msg.success_criteria = str(goal.success_criteria or "").strip()
        goal_msg.failure_policy = str(goal.failure_policy or "").strip()
        goal_msg.verify_policy = str(goal.verify_policy or "").strip()
        goal_msg.recovery_policy = str(goal.recovery_policy or "").strip()
        goal_msg.handoff_context_json = str(goal.handoff_context_json or "").strip()
        goal_msg.reason = str(goal.reason or "").strip()
        goal_msg.prompt_text = str(goal.prompt_text or "").strip()
        goal_msg.raw_json = str(goal.raw_json or "").strip()
        return goal_msg

    def _goal_to_semantic_task(self, goal: TaskGoal) -> SemanticTask:
        semantic = SemanticTask()
        semantic.header.stamp = self.get_clock().now().to_msg()
        semantic.task = str(goal.task or "pick").strip() or "pick"
        semantic.target_label = str(goal.target_label or "").strip()
        semantic.target_hint = str(goal.target_hint or semantic.target_label).strip()
        semantic.target_instance_track_id = int(goal.target_instance_track_id or 0)
        semantic.target_instance_bbox_xyxy = [int(item) for item in list(goal.target_instance_bbox_xyxy)]
        semantic.target_instance_point_px = [int(item) for item in list(goal.target_instance_point_px)]
        semantic.target_instance_source = str(goal.target_instance_source or "").strip()
        semantic.constraints = [
            *[str(item).strip() for item in list(goal.transport_constraints) if str(item).strip()],
            *[str(item).strip() for item in list(goal.execution_constraints) if str(item).strip()],
        ]
        semantic.excluded_labels = [
            str(item).strip() for item in list(goal.excluded_labels) if str(item).strip()
        ]
        semantic.confidence = float(goal.confidence or 0.0)
        semantic.need_human_confirm = False
        semantic.reason = str(goal.reason or "").strip()
        semantic.prompt_text = str(goal.prompt_text or "").strip()
        semantic.raw_json = str(goal.raw_json or "").strip()
        return semantic

    def _make_status(
        self,
        goal: TaskGoal,
        *,
        phase: str,
        current_skill: str,
        message: str,
        active: bool,
        success: bool,
        progress: float,
        retry_count: int = 0,
        error_code: str = "",
        skill_status_code: str = "",
        selected_candidate_id: int = 0,
        grounding_confidence: float = 0.0,
        best_grasp_quality: float = 0.0,
        best_affordance_score: float = 0.0,
        verification_score: float = 0.0,
        recommended_recovery: str = "",
    ) -> TaskExecutionStatus:
        _, target_locked, candidate_visible, pick_active, _ = self._snapshot_runtime()
        status = TaskExecutionStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.goal_id = str(goal.goal_id or "").strip()
        status.parent_goal_id = str(goal.parent_goal_id or "").strip()
        status.step_id = str(goal.step_id or "").strip()
        status.phase = str(phase or "idle")
        status.current_skill = str(current_skill or "")
        status.skill_status_code = str(skill_status_code or "")
        status.message = str(message or "")
        status.error_code = str(error_code or "")
        status.target_label = str(goal.target_label or "").strip()
        status.target_hint = str(goal.target_hint or "").strip()
        status.target_part_query = str(goal.target_part_query or "").strip()
        status.grasp_region = str(goal.grasp_region or "").strip()
        status.active = bool(active)
        status.success = bool(success)
        status.requires_confirmation = bool(goal.need_human_confirm)
        status.target_locked = bool(target_locked)
        status.target_candidate_visible = bool(candidate_visible)
        status.pick_active = bool(pick_active)
        status.retry_count = max(0, int(retry_count))
        status.max_retries = max(0, int(goal.max_retries or 0))
        status.grounded_track_id = int(goal.target_instance_track_id or 0)
        status.selected_candidate_id = int(selected_candidate_id or 0)
        status.progress = max(0.0, min(1.0, float(progress)))
        status.grounding_confidence = max(0.0, float(grounding_confidence or 0.0))
        status.best_grasp_quality = max(0.0, float(best_grasp_quality or 0.0))
        status.best_affordance_score = max(0.0, float(best_affordance_score or 0.0))
        status.verification_score = max(0.0, float(verification_score or 0.0))
        status.recommended_recovery = str(recommended_recovery or "")
        return status

    def _publish_status(self, status: TaskExecutionStatus, goal_handle: Any | None = None) -> None:
        self.task_status_pub.publish(status)
        if goal_handle is not None:
            feedback = ExecuteTask.Feedback()
            feedback.status = status
            goal_handle.publish_feedback(feedback)

    def _handle_cancel(self, goal_handle: Any, goal: TaskGoal, message: str) -> ExecuteTask.Result:
        home_ok, _home_message = self._call_trigger(self.return_home_client, timeout_sec=3.0)
        if not home_ok:
            self._call_trigger(self.reset_pick_client, timeout_sec=1.5)
        status = self._make_status(
            goal,
            phase="cancelled",
            current_skill="cancel",
            message=message,
            active=False,
            success=False,
            progress=0.0,
            error_code="cancelled",
        )
        self._publish_status(status, goal_handle)
        goal_handle.canceled()
        result = ExecuteTask.Result()
        result.success = False
        result.final_phase = "cancelled"
        result.message = message
        result.error_code = "cancelled"
        result.status = status
        return result

    def _finish_result(
        self,
        goal_handle: Any,
        *,
        goal: TaskGoal,
        success: bool,
        final_phase: str,
        current_skill: str,
        message: str,
        progress: float,
        retry_count: int,
        error_code: str = "",
    ) -> ExecuteTask.Result:
        status = self._make_status(
            goal,
            phase=final_phase,
            current_skill=current_skill,
            message=message,
            active=False,
            success=success,
            progress=progress,
            retry_count=retry_count,
            error_code=error_code,
        )
        self._publish_status(status, goal_handle)
        result = ExecuteTask.Result()
        result.success = bool(success)
        result.final_phase = str(final_phase or "")
        result.message = str(message or "")
        result.error_code = str(error_code or "")
        result.status = status
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _publish_goal_context(self, goal: TaskGoal, goal_handle: Any) -> None:
        self.task_goal_pub.publish(goal)
        self.semantic_task_pub.publish(self._goal_to_semantic_task(goal))
        self._publish_status(
            self._make_status(
                goal,
                phase="accepted",
                current_skill="publish_goal",
                message="task goal accepted and semantic task published",
                active=True,
                success=False,
                progress=0.08,
            ),
            goal_handle,
        )

    def _start_search(self, goal: TaskGoal, goal_handle: Any, retry_count: int) -> bool:
        if not bool(goal.start_search_sweep):
            return False
        ok, message = self._call_trigger(self.start_search_sweep_client, timeout_sec=1.5)
        state_message = "search sweep started"
        if not ok and message:
            state_message = f"search sweep warning: {message}"
        self._publish_status(
            self._make_status(
                goal,
                phase="grounding",
                current_skill="search_sweep",
                message=state_message,
                active=True,
                success=False,
                progress=0.15,
                retry_count=retry_count,
                error_code="" if ok else "search_sweep_warning",
            ),
            goal_handle,
        )
        return bool(ok)

    def _search_target_feedback_callback(
        self,
        *,
        goal_handle: Any,
        goal: TaskGoal,
        retry_count: int,
    ):
        def _callback(feedback_msg: Any) -> None:
            feedback = getattr(feedback_msg, "feedback", None)
            if feedback is None:
                return
            phase = str(getattr(feedback, "phase", "") or "SCANNING").strip().lower()
            message = {
                "scanning": "search_target scanning for a candidate",
                "centering": "search_target found a candidate and is waiting for lock",
                "waiting_stable_lock": "search_target waiting for a stable lock",
                "locked": "search_target acquired a stable lock",
            }.get(phase, f"search_target phase={phase or 'unknown'}")
            progress = 0.14
            if phase == "centering":
                progress = 0.2
            elif phase == "waiting_stable_lock":
                progress = 0.26
            elif phase == "locked":
                progress = 0.32
            self._publish_status(
                self._make_status(
                    goal,
                    phase="grounding",
                    current_skill="search_target",
                    message=message,
                    active=True,
                    success=False,
                    progress=progress,
                    retry_count=retry_count,
                    skill_status_code=phase.upper(),
                    grounding_confidence=float(
                        getattr(feedback, "grounding_confidence", 0.0) or 0.0
                    ),
                ),
                goal_handle,
            )

        return _callback

    def _run_search_target_action(
        self,
        *,
        goal_handle: Any,
        goal: TaskGoal,
        retry_count: int,
    ) -> tuple[bool, str, str]:
        if not self.prefer_search_target_action:
            self._publish_status(
                self._make_status(
                    goal,
                    phase="grounding",
                    current_skill="ground_target",
                    message="waiting for current target visibility before legacy sweep",
                    active=True,
                    success=False,
                    progress=0.14,
                    retry_count=retry_count,
                ),
                goal_handle,
            )
            return True, "legacy grounding wait started", ""
        if not self.search_target_client.wait_for_server(timeout_sec=self.service_timeout_sec):
            return True, "search_target action unavailable; using legacy wait", ""

        request = SearchTarget.Goal()
        request.goal = goal
        request.strategy = "FIXED_SWEEP"
        request.time_budget_sec = max(
            self.loop_sleep_sec, float(goal.planning_timeout_sec or self.target_lock_timeout_sec)
        )
        request.min_lock_frames = 2 if bool(goal.require_target_lock or goal.start_search_sweep) else 1
        request.min_grounding_confidence = 0.05
        request.stop_on_candidate_visible = not bool(
            goal.require_target_lock or goal.start_search_sweep
        )
        request.require_affordance_ready = False

        send_future = self.search_target_client.send_goal_async(
            request,
            feedback_callback=self._search_target_feedback_callback(
                goal_handle=goal_handle,
                goal=goal,
                retry_count=retry_count,
            ),
        )
        accepted = threading.Event()
        send_future.add_done_callback(lambda _: accepted.set())
        if not accepted.wait(self.service_timeout_sec):
            return True, "search_target goal request timed out; using legacy wait", ""
        try:
            search_goal_handle = send_future.result()
        except Exception as exc:  # noqa: BLE001
            return True, f"search_target action error: {exc}; using legacy wait", ""
        if search_goal_handle is None or not bool(search_goal_handle.accepted):
            return True, "search_target goal rejected; using legacy wait", ""

        result_future = search_goal_handle.get_result_async()
        done = threading.Event()
        result_future.add_done_callback(lambda _: done.set())
        wait_deadline = time.time() + max(self.loop_sleep_sec, float(request.time_budget_sec) + 2.0)
        while time.time() < wait_deadline:
            if goal_handle.is_cancel_requested:
                try:
                    search_goal_handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                return False, "cancel requested", "cancelled"
            if done.wait(self.loop_sleep_sec):
                break
        if not done.is_set():
            try:
                search_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            return True, "search_target action timed out; falling back to legacy wait", ""
        try:
            wrapped_result = result_future.result()
            result = getattr(wrapped_result, "result", None)
        except Exception as exc:  # noqa: BLE001
            return True, f"search_target result failed: {exc}; falling back to legacy wait", ""
        if result is None:
            return True, "search_target returned no result; falling back to legacy wait", ""
        if bool(result.success):
            self._publish_status(
                self._make_status(
                    goal,
                    phase="grounded",
                    current_skill="search_target",
                    message=str(result.message or "target search completed"),
                    active=True,
                    success=False,
                    progress=0.34,
                    retry_count=retry_count,
                    skill_status_code=str(result.status_code or ""),
                    grounding_confidence=float(result.grounding_confidence or 0.0),
                    ),
                goal_handle,
            )
            return True, str(result.message or "target search completed"), ""
        fallback_message = str(result.message or "search_target failed")
        self._publish_status(
            self._make_status(
                goal,
                phase="grounding",
                current_skill="ground_target",
                message=f"{fallback_message}; falling back to legacy wait",
                active=True,
                success=False,
                progress=0.16,
                retry_count=retry_count,
                error_code=str(result.status_code or "grounding_error").lower(),
                skill_status_code="FALLBACK_LEGACY_WAIT",
                grounding_confidence=float(result.grounding_confidence or 0.0),
            ),
            goal_handle,
        )
        return True, f"{fallback_message}; falling back to legacy wait", ""

    def _request_execution(
        self,
        *,
        goal_handle: Any,
        goal: TaskGoal,
        retry_count: int,
    ) -> tuple[bool, str, str]:
        deadline = time.time() + max(self.loop_sleep_sec, float(goal.planning_timeout_sec))
        search_started = False
        candidate_visible_since = 0.0
        candidate_missing_since = 0.0
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "cancel requested", "cancelled"
            pose_ready, target_locked, candidate_visible, _, _ = self._snapshot_runtime()
            if not pose_ready:
                now = time.time()
                if candidate_visible:
                    if candidate_visible_since <= 0.0:
                        candidate_visible_since = now
                    candidate_missing_since = 0.0
                else:
                    if candidate_missing_since <= 0.0:
                        candidate_missing_since = now
                    candidate_visible_since = 0.0

                if bool(goal.start_search_sweep) and not search_started:
                    should_start_search = False
                    if not candidate_visible:
                        should_start_search = (
                            now - candidate_missing_since
                        ) >= self.search_sweep_start_delay_sec
                    elif (now - candidate_visible_since) >= self.visible_target_lock_wait_sec:
                        should_start_search = True
                    if should_start_search:
                        search_started = self._start_search(goal, goal_handle, retry_count)

                phase = "grounding"
                if target_locked:
                    phase = "planning"
                elif candidate_visible:
                    phase = "awaiting_lock"
                wait_message = "waiting for target pose from perception"
                if bool(goal.start_search_sweep) and not search_started:
                    if candidate_visible:
                        wait_message = (
                            "candidate visible but target pose not ready; waiting briefly before sweep"
                        )
                    else:
                        wait_message = "candidate missing and target pose not ready; waiting briefly before sweep"
                self._publish_status(
                    self._make_status(
                        goal,
                        phase=phase,
                        current_skill="ground_target",
                        message=wait_message,
                        active=True,
                        success=False,
                        progress=0.24 if candidate_visible else 0.18,
                        retry_count=retry_count,
                    ),
                    goal_handle,
                )
                time.sleep(self.loop_sleep_sec)
                continue

            ok, message = self._call_trigger(self.execute_pick_client, timeout_sec=1.5)
            lowered = str(message or "").strip().lower()
            if ok or "already executing" in lowered or "already armed" in lowered:
                accepted_message = str(message or "pick execution accepted")
                self._publish_status(
                    self._make_status(
                        goal,
                        phase="planning",
                        current_skill="execute_pick",
                        message=accepted_message,
                        active=True,
                        success=False,
                        progress=0.35,
                        retry_count=retry_count,
                    ),
                    goal_handle,
                )
                return True, accepted_message, ""

            _, target_locked, candidate_visible, _, _ = self._snapshot_runtime()
            if "target is not locked" in lowered or "target pose is unavailable" in lowered:
                now = time.time()
                if candidate_visible:
                    if candidate_visible_since <= 0.0:
                        candidate_visible_since = now
                    candidate_missing_since = 0.0
                else:
                    if candidate_missing_since <= 0.0:
                        candidate_missing_since = now
                    candidate_visible_since = 0.0

                if bool(goal.start_search_sweep) and not search_started:
                    should_start_search = False
                    if not candidate_visible:
                        should_start_search = (
                            now - candidate_missing_since
                        ) >= self.search_sweep_start_delay_sec
                    elif (now - candidate_visible_since) >= self.visible_target_lock_wait_sec:
                        should_start_search = True
                    if should_start_search:
                        search_started = self._start_search(goal, goal_handle, retry_count)

                phase = "grounding"
                if target_locked:
                    phase = "planning"
                elif candidate_visible:
                    phase = "awaiting_lock"
                wait_message = str(message or "waiting for a grounded target")
                if bool(goal.start_search_sweep) and not search_started:
                    if candidate_visible:
                        wait_message = (
                            "candidate already visible; waiting briefly for lock before sweep"
                        )
                    else:
                        wait_message = "candidate missing; waiting briefly before sweep"
                self._publish_status(
                    self._make_status(
                        goal,
                        phase=phase,
                        current_skill="ground_target",
                        message=wait_message,
                        active=True,
                        success=False,
                        progress=0.22 if candidate_visible else 0.18,
                        retry_count=retry_count,
                    ),
                    goal_handle,
                )
                time.sleep(self.loop_sleep_sec)
                continue

            return False, str(message or "execute_pick rejected"), "execute_rejected"

        return False, "timed out while waiting for execution readiness", "grounding_timeout"

    def _monitor_execution(
        self,
        *,
        goal_handle: Any,
        goal: TaskGoal,
        retry_count: int,
    ) -> tuple[bool, str, str]:
        idle_timeout_sec = max(self.loop_sleep_sec, float(goal.execution_timeout_sec))
        last_activity_at = time.time()
        last_signature: tuple[Any, ...] | None = None
        last_meaningful_message = ""
        saw_pick_start = False

        while True:
            if goal_handle.is_cancel_requested:
                return False, "cancel requested", "cancelled"
            now = time.time()
            _, _, _, pick_active, pick_status = self._snapshot_runtime()
            low_level_phase = str(pick_status.get("phase", "") or "").strip()
            low_level_message = str(pick_status.get("message", "") or "").strip()
            progress_percent = int(pick_status.get("progress_percent", 0) or 0)
            status_updated_at = float(pick_status.get("updated_at", 0.0) or 0.0)

            signature = (
                low_level_phase,
                low_level_message,
                progress_percent,
                bool(pick_active),
                round(status_updated_at, 3),
            )
            if signature != last_signature:
                last_signature = signature
                last_activity_at = now
            if low_level_message:
                last_meaningful_message = low_level_message
            if pick_active or low_level_phase in {"planning", "executing"}:
                saw_pick_start = True

            progress = 0.55
            if progress_percent > 0:
                progress = min(
                    0.95,
                    0.55 + 0.4 * (float(progress_percent) / 100.0),
                )
            if low_level_phase == "completed":
                return True, low_level_message or "pick completed", ""
            if low_level_phase == "error":
                return False, low_level_message or "pick execution failed", "pick_error"
            if saw_pick_start and not pick_active and low_level_phase not in {"completed", "error"}:
                message = last_meaningful_message or low_level_message or "pick became inactive"
                return False, message, "pick_inactive"

            mapped_phase = "executing" if pick_active else "planning"
            mapped_skill = "monitor_pick"
            if low_level_phase in {"planning", "executing"}:
                mapped_phase = low_level_phase
            self._publish_status(
                self._make_status(
                    goal,
                    phase=mapped_phase,
                    current_skill=mapped_skill,
                    message=low_level_message or "waiting for pick execution to finish",
                    active=True,
                    success=False,
                    progress=progress,
                    retry_count=retry_count,
                ),
                goal_handle,
            )
            if now - last_activity_at >= idle_timeout_sec:
                message = (
                    last_meaningful_message
                    or low_level_message
                    or "timed out while waiting for pick progress"
                )
                return False, message, "execution_timeout"
            time.sleep(self.loop_sleep_sec)

    def _recover_for_retry(
        self,
        *,
        goal_handle: Any,
        goal: TaskGoal,
        retry_count: int,
        last_error: str,
    ) -> None:
        self._publish_status(
            self._make_status(
                goal,
                phase="recovering",
                current_skill="reset_pick_session",
                message=f"recovering after failure: {last_error}",
                active=True,
                success=False,
                progress=0.12,
                retry_count=retry_count,
                error_code="recovering",
            ),
            goal_handle,
        )
        self._call_trigger(self.reset_pick_client, timeout_sec=1.5)
        if bool(goal.allow_rescan):
            self._call_trigger(self.start_search_sweep_client, timeout_sec=1.5)

    def _execute_task(self, goal_handle: Any) -> ExecuteTask.Result:
        raw_goal = goal_handle.request.goal
        goal = self._normalize_goal(raw_goal)
        with self._state_lock:
            self._goal_reserved = False
            self._active_goal = goal_handle

        try:
            self._publish_goal_context(goal, goal_handle)
            self._call_trigger(self.reset_pick_client, timeout_sec=1.5)

            max_attempts = max(1, int(goal.max_retries or 0) + 1)
            for attempt_index in range(max_attempts):
                retry_count = int(attempt_index)
                if goal_handle.is_cancel_requested:
                    return self._handle_cancel(goal_handle, goal, "task cancelled before execution")

                self._publish_goal_context(goal, goal_handle)
                grounded, grounding_message, grounding_error = self._run_search_target_action(
                    goal_handle=goal_handle,
                    goal=goal,
                    retry_count=retry_count,
                )
                if grounding_error == "cancelled":
                    return self._handle_cancel(goal_handle, goal, grounding_message)
                if not grounded:
                    can_retry = retry_count + 1 < max_attempts and (
                        bool(goal.allow_replan) or bool(goal.allow_rescan)
                    )
                    if can_retry:
                        self._recover_for_retry(
                            goal_handle=goal_handle,
                            goal=goal,
                            retry_count=retry_count + 1,
                            last_error=grounding_message,
                        )
                        continue
                    if self.recovery_return_home_on_failure:
                        self._call_trigger(self.return_home_client, timeout_sec=1.5)
                    return self._finish_result(
                        goal_handle,
                        goal=goal,
                        success=False,
                        final_phase="error",
                        current_skill="search_target",
                        message=grounding_message,
                        progress=0.0,
                        retry_count=retry_count,
                        error_code=grounding_error or "grounding_error",
                    )
                accepted, message, error_code = self._request_execution(
                    goal_handle=goal_handle,
                    goal=goal,
                    retry_count=retry_count,
                )
                if error_code == "cancelled":
                    return self._handle_cancel(goal_handle, goal, message)
                if not accepted:
                    can_retry = retry_count + 1 < max_attempts and (
                        bool(goal.allow_replan) or bool(goal.allow_rescan)
                    )
                    if can_retry:
                        self._recover_for_retry(
                            goal_handle=goal_handle,
                            goal=goal,
                            retry_count=retry_count + 1,
                            last_error=message,
                        )
                        continue
                    if self.recovery_return_home_on_failure:
                        self._call_trigger(self.return_home_client, timeout_sec=1.5)
                    return self._finish_result(
                        goal_handle,
                        goal=goal,
                        success=False,
                        final_phase="error",
                        current_skill="execute_pick",
                        message=message,
                        progress=0.0,
                        retry_count=retry_count,
                        error_code=error_code or "execute_rejected",
                    )

                success, monitor_message, monitor_error = self._monitor_execution(
                    goal_handle=goal_handle,
                    goal=goal,
                    retry_count=retry_count,
                )
                if monitor_error == "cancelled":
                    return self._handle_cancel(goal_handle, goal, monitor_message)
                if success:
                    return self._finish_result(
                        goal_handle,
                        goal=goal,
                        success=True,
                        final_phase="completed",
                        current_skill="verify_result",
                        message=monitor_message,
                        progress=1.0,
                        retry_count=retry_count,
                    )

                can_retry = retry_count + 1 < max_attempts and (
                    bool(goal.allow_replan) or bool(goal.allow_rescan)
                )
                if can_retry:
                    self._recover_for_retry(
                        goal_handle=goal_handle,
                        goal=goal,
                        retry_count=retry_count + 1,
                        last_error=monitor_message,
                    )
                    continue

                if self.recovery_return_home_on_failure:
                    self._call_trigger(self.return_home_client, timeout_sec=1.5)
                return self._finish_result(
                    goal_handle,
                    goal=goal,
                    success=False,
                    final_phase="error",
                    current_skill="monitor_pick",
                    message=monitor_message,
                    progress=0.0,
                    retry_count=retry_count,
                    error_code=monitor_error or "pick_error",
                )

            return self._finish_result(
                goal_handle,
                goal=goal,
                success=False,
                final_phase="error",
                current_skill="task_executive",
                message="task exhausted all retries",
                progress=0.0,
                retry_count=max_attempts - 1,
                error_code="retry_exhausted",
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().exception("task execution crashed")
            return self._finish_result(
                goal_handle,
                goal=goal,
                success=False,
                final_phase="error",
                current_skill="task_executive",
                message=f"task executive exception: {exc}",
                progress=0.0,
                retry_count=0,
                error_code="exception",
            )
        finally:
            with self._state_lock:
                self._goal_reserved = False
                self._active_goal = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TaskExecutiveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
