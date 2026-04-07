from __future__ import annotations

import json
import threading
import time
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from tactile_interfaces.action import SearchTarget
from tactile_interfaces.msg import AffordanceGroundingResult, DetectionResult, MaskChannel


class SearchTargetSkillNode(Node):
    """Action wrapper around the existing sweep-and-lock pipeline."""

    def __init__(self) -> None:
        super().__init__("search_target_skill_node")

        self.declare_parameter("search_target_action", "/task/search_target")
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter(
            "target_candidate_visible_topic", "/sim/perception/target_candidate_visible"
        )
        self.declare_parameter("detection_result_topic", "/perception/detection_result")
        self.declare_parameter("start_search_sweep_service", "/task/start_search_sweep")
        self.declare_parameter("search_sweep_start_delay_sec", 1.0)
        self.declare_parameter("visible_target_lock_wait_sec", 2.0)
        self.declare_parameter("loop_sleep_sec", 0.2)
        self.declare_parameter("service_timeout_sec", 1.5)
        self.declare_parameter("default_time_budget_sec", 18.0)
        self.declare_parameter("default_min_lock_frames", 2)

        self.search_target_action = str(self.get_parameter("search_target_action").value)
        self.target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.target_candidate_visible_topic = str(
            self.get_parameter("target_candidate_visible_topic").value
        )
        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.start_search_sweep_service = str(
            self.get_parameter("start_search_sweep_service").value
        )
        self.loop_sleep_sec = max(0.05, float(self.get_parameter("loop_sleep_sec").value))
        self.search_sweep_start_delay_sec = max(
            0.0, float(self.get_parameter("search_sweep_start_delay_sec").value)
        )
        self.visible_target_lock_wait_sec = max(
            self.loop_sleep_sec, float(self.get_parameter("visible_target_lock_wait_sec").value)
        )
        self.service_timeout_sec = max(
            self.loop_sleep_sec, float(self.get_parameter("service_timeout_sec").value)
        )
        self.default_time_budget_sec = max(
            self.loop_sleep_sec, float(self.get_parameter("default_time_budget_sec").value)
        )
        self.default_min_lock_frames = max(
            1, int(self.get_parameter("default_min_lock_frames").value)
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._target_locked = False
        self._target_candidate_visible = False
        self._last_detection = DetectionResult()
        self._last_detection_ts = 0.0

        self.create_subscription(Bool, self.target_locked_topic, self._on_target_locked, qos)
        self.create_subscription(
            Bool, self.target_candidate_visible_topic, self._on_target_candidate_visible, qos
        )
        self.create_subscription(
            DetectionResult, self.detection_result_topic, self._on_detection_result, qos
        )
        self.start_search_sweep_client = self.create_client(
            Trigger,
            self.start_search_sweep_service,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            SearchTarget,
            self.search_target_action,
            execute_callback=self._execute_search,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "search_target_skill_node started: "
            f"action={self.search_target_action} "
            f"detection={self.detection_result_topic}"
        )

    def _on_goal(self, goal_request: SearchTarget.Goal) -> GoalResponse:
        goal = goal_request.goal
        target_name = str(goal.target_label or goal.target_hint or "").strip()
        if not target_name:
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

    def _on_detection_result(self, msg: DetectionResult) -> None:
        with self._state_lock:
            self._last_detection = msg
            self._last_detection_ts = time.time()

    def _snapshot(self) -> tuple[bool, bool, DetectionResult, float]:
        with self._state_lock:
            return (
                bool(self._target_locked),
                bool(self._target_candidate_visible),
                self._last_detection,
                float(self._last_detection_ts),
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

    def _feedback_message(
        self,
        *,
        candidate_visible: bool,
        target_locked: bool,
        stable_lock_frames: int,
        min_lock_frames: int,
    ) -> tuple[str, str]:
        if target_locked and stable_lock_frames >= min_lock_frames:
            return "LOCKED", "target lock is stable"
        if target_locked:
            return "WAITING_STABLE_LOCK", "target lock detected; waiting for stability"
        if candidate_visible:
            return "CENTERING", "candidate visible; waiting for lock"
        return "SCANNING", "searching for target candidate"

    def _build_grounding(
        self,
        *,
        goal: SearchTarget.Goal,
        detection: DetectionResult,
        detection_ts: float,
        strategy: str,
        stable_lock_frames: int,
        min_lock_frames: int,
    ) -> AffordanceGroundingResult:
        grounding = AffordanceGroundingResult()
        grounding.header.stamp = self.get_clock().now().to_msg()
        grounding.goal_id = str(goal.goal.goal_id or "").strip()
        grounding.step_id = str(goal.goal.step_id or "").strip() or "search_target"
        grounding.track_id = int(goal.goal.target_instance_track_id or 0)
        grounding.target_label = str(detection.target_label or goal.goal.target_label or "").strip()
        grounding.target_hint = str(goal.goal.target_hint or grounding.target_label).strip()
        grounding.target_part = str(goal.goal.target_part_query or goal.goal.grasp_region or "").strip()
        grounding.grounding_confidence = float(detection.confidence or 0.0)
        grounding.part_confidence = 0.0
        grounding.temporal_stability = min(
            1.0, float(stable_lock_frames) / float(max(1, min_lock_frames))
        )
        if bool(detection.has_bbox):
            grounding.roi_xyxy = [
                int(detection.bbox.x_offset),
                int(detection.bbox.y_offset),
                int(detection.bbox.x_offset + detection.bbox.width),
                int(detection.bbox.y_offset + detection.bbox.height),
            ]
        grounding.image_width = int(detection.image_width)
        grounding.image_height = int(detection.image_height)
        grounding.target_pose.header.stamp = grounding.header.stamp
        grounding.target_pose.header.frame_id = str(detection.header.frame_id or "")
        if bool(detection.has_mask):
            object_mask = MaskChannel()
            object_mask.name = "object"
            object_mask.mask = detection.mask
            object_mask.confidence = grounding.grounding_confidence
            object_mask.source = "detection_result"
            object_mask.tags = [item for item in [grounding.target_label] if item]
            grounding.masks = [object_mask]
        grounding.affordance_tags = [
            str(item).strip()
            for item in list(goal.goal.preferred_part_tags)
            if str(item).strip()
        ]
        grounding.forbidden_tags = [
            str(item).strip()
            for item in list(goal.goal.forbidden_part_tags)
            if str(item).strip()
        ]
        grounding.allowed_approach_dirs = [
            str(item).strip()
            for item in list(goal.goal.preferred_approach_dirs)
            if str(item).strip()
        ]
        grounding.transport_constraints = [
            str(item).strip()
            for item in list(goal.goal.transport_constraints)
            if str(item).strip()
        ]
        grounding.selected_strategy = str(strategy or "FIXED_SWEEP")
        grounding.mask_bundle_id = (
            f"{grounding.goal_id}:{int(detection_ts * 1000.0)}" if detection_ts > 0.0 else ""
        )
        grounding.debug_json = json.dumps(
            {
                "candidate_visible": bool(detection.candidate_visible),
                "candidate_complete": bool(detection.candidate_complete),
                "has_bbox": bool(detection.has_bbox),
                "has_mask": bool(detection.has_mask),
                "stable_lock_frames": int(stable_lock_frames),
            },
            separators=(",", ":"),
        )
        return grounding

    def _result_from_snapshot(
        self,
        *,
        goal: SearchTarget.Goal,
        success: bool,
        status_code: str,
        message: str,
        detection: DetectionResult,
        detection_ts: float,
        strategy: str,
        stable_lock_frames: int,
        min_lock_frames: int,
    ) -> SearchTarget.Result:
        result = SearchTarget.Result()
        result.success = bool(success)
        result.status_code = str(status_code or "")
        result.track_id = int(goal.goal.target_instance_track_id or 0)
        if bool(detection.has_bbox):
            result.bbox_xyxy = [
                int(detection.bbox.x_offset),
                int(detection.bbox.y_offset),
                int(detection.bbox.x_offset + detection.bbox.width),
                int(detection.bbox.y_offset + detection.bbox.height),
            ]
        if bool(detection.has_point):
            result.point_px = [int(detection.point_px[0]), int(detection.point_px[1])]
        result.grounding_confidence = float(detection.confidence or 0.0)
        result.grounding = self._build_grounding(
            goal=goal,
            detection=detection,
            detection_ts=detection_ts,
            strategy=strategy,
            stable_lock_frames=stable_lock_frames,
            min_lock_frames=min_lock_frames,
        )
        result.message = str(message or "")
        return result

    def _execute_search(self, goal_handle: Any) -> SearchTarget.Result:
        goal = goal_handle.request
        strategy = str(goal.strategy or "").strip() or "FIXED_SWEEP"
        time_budget_sec = max(
            self.loop_sleep_sec,
            float(goal.time_budget_sec or self.default_time_budget_sec),
        )
        min_lock_frames = max(
            1,
            int(goal.min_lock_frames or self.default_min_lock_frames),
        )
        min_grounding_confidence = max(0.0, float(goal.min_grounding_confidence or 0.0))
        require_affordance_ready = bool(goal.require_affordance_ready)
        stop_on_candidate_visible = bool(goal.stop_on_candidate_visible)

        deadline = time.time() + time_budget_sec
        stable_lock_frames = 0
        last_candidate_visible = False
        search_started = False
        candidate_visible_since = 0.0
        candidate_missing_since = 0.0
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                _, _, detection, detection_ts = self._snapshot()
                return self._result_from_snapshot(
                    goal=goal,
                    success=False,
                    status_code="CANCELLED",
                    message="search target cancelled",
                    detection=detection,
                    detection_ts=detection_ts,
                    strategy=strategy,
                    stable_lock_frames=stable_lock_frames,
                    min_lock_frames=min_lock_frames,
                )

            target_locked, candidate_visible, detection, detection_ts = self._snapshot()
            now = time.time()
            if candidate_visible:
                if candidate_visible_since <= 0.0:
                    candidate_visible_since = now
                candidate_missing_since = 0.0
            else:
                if candidate_missing_since <= 0.0:
                    candidate_missing_since = now
                candidate_visible_since = 0.0

            if bool(goal.goal.start_search_sweep) and not search_started and not target_locked:
                should_start_search = False
                if not candidate_visible:
                    should_start_search = (
                        now - candidate_missing_since
                    ) >= self.search_sweep_start_delay_sec
                elif (now - candidate_visible_since) >= self.visible_target_lock_wait_sec:
                    should_start_search = True
                if should_start_search:
                    self._call_trigger(self.start_search_sweep_client, timeout_sec=1.5)
                    search_started = True

            has_grounding = bool(detection.accepted) and float(detection.confidence or 0.0) >= min_grounding_confidence
            affordance_ready = bool(detection.has_mask or detection.has_bbox)
            if require_affordance_ready and not affordance_ready:
                has_grounding = False

            if target_locked:
                stable_lock_frames += 1
            else:
                stable_lock_frames = 0

            phase, message = self._feedback_message(
                candidate_visible=candidate_visible,
                target_locked=target_locked,
                stable_lock_frames=stable_lock_frames,
                min_lock_frames=min_lock_frames,
            )
            feedback = SearchTarget.Feedback()
            feedback.phase = phase
            feedback.candidate_visible = bool(candidate_visible)
            feedback.target_locked = bool(target_locked)
            feedback.track_id = int(goal.goal.target_instance_track_id or 0)
            feedback.stable_lock_frames = int(stable_lock_frames)
            if bool(detection.has_bbox):
                feedback.bbox_xyxy = [
                    int(detection.bbox.x_offset),
                    int(detection.bbox.y_offset),
                    int(detection.bbox.x_offset + detection.bbox.width),
                    int(detection.bbox.y_offset + detection.bbox.height),
                ]
            if bool(detection.has_point):
                feedback.point_px = [int(detection.point_px[0]), int(detection.point_px[1])]
            feedback.grounding_confidence = float(detection.confidence or 0.0)
            feedback.elapsed_sec = float(time_budget_sec - max(0.0, deadline - time.time()))
            goal_handle.publish_feedback(feedback)

            if target_locked and has_grounding and stable_lock_frames >= min_lock_frames:
                goal_handle.succeed()
                return self._result_from_snapshot(
                    goal=goal,
                    success=True,
                    status_code="OK",
                    message="target search succeeded with stable lock",
                    detection=detection,
                    detection_ts=detection_ts,
                    strategy=strategy,
                    stable_lock_frames=stable_lock_frames,
                    min_lock_frames=min_lock_frames,
                )

            if stop_on_candidate_visible and candidate_visible and has_grounding:
                goal_handle.succeed()
                return self._result_from_snapshot(
                    goal=goal,
                    success=True,
                    status_code="VISIBLE_NOT_LOCKED",
                    message="candidate became visible before target lock",
                    detection=detection,
                    detection_ts=detection_ts,
                    strategy=strategy,
                    stable_lock_frames=stable_lock_frames,
                    min_lock_frames=min_lock_frames,
                )

            last_candidate_visible = bool(candidate_visible)
            time.sleep(self.loop_sleep_sec)

        goal_handle.abort()
        _, _, detection, detection_ts = self._snapshot()
        status_code = "SEARCH_TIMEOUT"
        if last_candidate_visible:
            status_code = "LOCK_UNSTABLE"
        return self._result_from_snapshot(
            goal=goal,
            success=False,
            status_code=status_code,
            message="search target timed out",
            detection=detection,
            detection_ts=detection_ts,
            strategy=strategy,
            stable_lock_frames=stable_lock_frames,
            min_lock_frames=min_lock_frames,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SearchTargetSkillNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
