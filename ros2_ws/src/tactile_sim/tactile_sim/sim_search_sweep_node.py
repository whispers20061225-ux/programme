from __future__ import annotations

import json
import math
from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tactile_interfaces.msg import DetectionResult
from tactile_interfaces.srv import MoveArmJoints
from trajectory_msgs.msg import JointTrajectoryPoint


class SimSearchSweepNode(Node):
    """Drive the arm through coarse search, then center visible targets in RGB."""

    JOINT_NAMES = [
        "arm_joint1",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
    ]

    def __init__(self) -> None:
        super().__init__("sim_search_sweep_node")

        self.declare_parameter(
            "trajectory_action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "target_locked_topic",
            "/sim/perception/target_locked",
        )
        self.declare_parameter("pick_active_topic", "")
        self.declare_parameter("pick_status_topic", "")
        self.declare_parameter("candidate_visible_topic", "")
        self.declare_parameter("detection_result_topic", "")
        self.declare_parameter("vision_result_topic", "/qwen/vision_result")
        self.declare_parameter("auto_start_enabled", True)
        self.declare_parameter("enable_service", "/task/start_search_sweep")
        self.declare_parameter("disable_service", "/task/stop_search_sweep")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "search_pose_deg",
            [-87.0, -72.0, 90.0, 90.0, -90.0],
        )
        self.declare_parameter(
            "scan_joint1_angles_deg",
            [-90.0, -84.0, -78.0, -72.0],
        )
        self.declare_parameter("startup_delay_sec", 3.0)
        self.declare_parameter("startup_prepare_home", False)
        self.declare_parameter("startup_prepare_home_delay_sec", 0.0)
        self.declare_parameter("motion_duration_sec", 1.0)
        self.declare_parameter("settle_duration_sec", 0.5)
        self.declare_parameter("joint1_min_deg", -180.0)
        self.declare_parameter("joint1_max_deg", 180.0)
        self.declare_parameter("stop_after_first_lock", True)
        self.declare_parameter("enable_centering", True)
        self.declare_parameter("vision_result_stale_sec", 3.0)
        self.declare_parameter("horizontal_center_tolerance_px", 40.0)
        self.declare_parameter("vertical_center_tolerance_px", 32.0)
        self.declare_parameter("bbox_edge_margin_px", 36.0)
        self.declare_parameter("centering_hold_sec", 0.6)
        self.declare_parameter("centering_horizontal_gain_deg", 14.0)
        self.declare_parameter("centering_vertical_gain_deg", 10.0)
        self.declare_parameter("centering_max_joint1_step_deg", 10.0)
        self.declare_parameter("centering_max_vertical_step_deg", 6.0)
        self.declare_parameter("centering_min_step_deg", 1.0)
        self.declare_parameter("centering_horizontal_sign", 1.0)
        self.declare_parameter("centering_vertical_sign", -1.0)
        self.declare_parameter("prefer_bbox_center", True)
        self.declare_parameter("adaptive_centering_signs", False)
        self.declare_parameter("vertical_joint_index", 3)
        self.declare_parameter("vertical_joint_min_deg", 70.0)
        self.declare_parameter("vertical_joint_max_deg", 110.0)
        self.declare_parameter("target_session_lock_enabled", True)
        self.declare_parameter("target_session_match_distance_px", 120.0)
        self.declare_parameter("target_session_bbox_iou_threshold", 0.15)
        self.declare_parameter("target_session_timeout_sec", 4.0)
        self.declare_parameter("hold_on_pick_error", True)
        self.declare_parameter("failure_return_pose_deg", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("failure_open_gripper_on_hold", True)
        self.declare_parameter("failure_open_gripper_joint_id", 6)
        self.declare_parameter("failure_open_gripper_angle_deg", -68.754936)
        self.declare_parameter("failure_open_gripper_duration_ms", 1500)
        self.declare_parameter("failure_open_gripper_wait", False)
        self.declare_parameter("control_arm_move_joints_service", "/control/arm/move_joints")

        self.trajectory_action_name = str(self.get_parameter("trajectory_action_name").value)
        target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.pick_active_topic = str(self.get_parameter("pick_active_topic").value).strip()
        self.pick_status_topic = str(self.get_parameter("pick_status_topic").value).strip()
        self.candidate_visible_topic = str(
            self.get_parameter("candidate_visible_topic").value
        ).strip()
        detection_result_topic = str(
            self.get_parameter("detection_result_topic").value
        ).strip()
        legacy_vision_result_topic = str(
            self.get_parameter("vision_result_topic").value
        ).strip()
        self.auto_start_enabled = bool(self.get_parameter("auto_start_enabled").value)
        self.enable_service = str(self.get_parameter("enable_service").value).strip()
        self.disable_service = str(self.get_parameter("disable_service").value).strip()
        self.vision_result_topic = detection_result_topic or legacy_vision_result_topic
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value).strip()
        self.search_pose_deg = [
            float(v) for v in list(self.get_parameter("search_pose_deg").value)
        ]
        joint1_min_deg = float(self.get_parameter("joint1_min_deg").value)
        joint1_max_deg = float(self.get_parameter("joint1_max_deg").value)
        if joint1_min_deg > joint1_max_deg:
            joint1_min_deg, joint1_max_deg = joint1_max_deg, joint1_min_deg
        self.joint1_min_deg = joint1_min_deg
        self.joint1_max_deg = joint1_max_deg

        raw_scan_angles = [
            float(v) for v in list(self.get_parameter("scan_joint1_angles_deg").value)
        ]
        clamped_scan_angles: List[float] = []
        for angle in raw_scan_angles:
            bounded = max(joint1_min_deg, min(joint1_max_deg, angle))
            if not clamped_scan_angles or abs(clamped_scan_angles[-1] - bounded) > 1e-6:
                clamped_scan_angles.append(bounded)
        self.scan_joint1_angles_deg = clamped_scan_angles or [self.search_pose_deg[0]]

        self.startup_delay_sec = max(0.0, float(self.get_parameter("startup_delay_sec").value))
        self.startup_prepare_home = bool(self.get_parameter("startup_prepare_home").value)
        self.startup_prepare_home_delay_sec = max(
            0.0, float(self.get_parameter("startup_prepare_home_delay_sec").value)
        )
        self.motion_duration_sec = max(0.2, float(self.get_parameter("motion_duration_sec").value))
        self.settle_duration_sec = max(0.0, float(self.get_parameter("settle_duration_sec").value))
        self.stop_after_first_lock = bool(self.get_parameter("stop_after_first_lock").value)
        self.enable_centering = bool(self.get_parameter("enable_centering").value)
        self.vision_result_stale_sec = max(
            0.2, float(self.get_parameter("vision_result_stale_sec").value)
        )
        self.horizontal_center_tolerance_px = max(
            1.0, float(self.get_parameter("horizontal_center_tolerance_px").value)
        )
        self.vertical_center_tolerance_px = max(
            1.0, float(self.get_parameter("vertical_center_tolerance_px").value)
        )
        self.bbox_edge_margin_px = max(0.0, float(self.get_parameter("bbox_edge_margin_px").value))
        self.centering_hold_sec = max(0.0, float(self.get_parameter("centering_hold_sec").value))
        self.centering_horizontal_gain_deg = max(
            0.1, float(self.get_parameter("centering_horizontal_gain_deg").value)
        )
        self.centering_vertical_gain_deg = max(
            0.1, float(self.get_parameter("centering_vertical_gain_deg").value)
        )
        self.centering_max_joint1_step_deg = max(
            0.1, float(self.get_parameter("centering_max_joint1_step_deg").value)
        )
        self.centering_max_vertical_step_deg = max(
            0.1, float(self.get_parameter("centering_max_vertical_step_deg").value)
        )
        self.centering_min_step_deg = max(
            0.0, float(self.get_parameter("centering_min_step_deg").value)
        )
        self.horizontal_response_sign = (
            1.0 if float(self.get_parameter("centering_horizontal_sign").value) >= 0.0 else -1.0
        )
        self.vertical_response_sign = (
            1.0 if float(self.get_parameter("centering_vertical_sign").value) >= 0.0 else -1.0
        )
        self.prefer_bbox_center = bool(self.get_parameter("prefer_bbox_center").value)
        self.adaptive_centering_signs = bool(
            self.get_parameter("adaptive_centering_signs").value
        )
        self.vertical_joint_index = int(self.get_parameter("vertical_joint_index").value)
        self.vertical_joint_index = max(1, min(len(self.JOINT_NAMES) - 1, self.vertical_joint_index))
        vertical_joint_min_deg = float(self.get_parameter("vertical_joint_min_deg").value)
        vertical_joint_max_deg = float(self.get_parameter("vertical_joint_max_deg").value)
        if vertical_joint_min_deg > vertical_joint_max_deg:
            vertical_joint_min_deg, vertical_joint_max_deg = (
                vertical_joint_max_deg,
                vertical_joint_min_deg,
            )
        self.vertical_joint_min_deg = vertical_joint_min_deg
        self.vertical_joint_max_deg = vertical_joint_max_deg
        self.target_session_lock_enabled = bool(
            self.get_parameter("target_session_lock_enabled").value
        )
        self.target_session_match_distance_px = max(
            8.0, float(self.get_parameter("target_session_match_distance_px").value)
        )
        self.target_session_bbox_iou_threshold = max(
            0.0, min(1.0, float(self.get_parameter("target_session_bbox_iou_threshold").value))
        )
        self.target_session_timeout_sec = max(
            0.2, float(self.get_parameter("target_session_timeout_sec").value)
        )
        self.hold_on_pick_error = bool(self.get_parameter("hold_on_pick_error").value)
        failure_return_pose_deg = [
            float(v) for v in list(self.get_parameter("failure_return_pose_deg").value)
        ]
        if len(failure_return_pose_deg) != len(self.JOINT_NAMES):
            failure_return_pose_deg = [0.0] * len(self.JOINT_NAMES)
        self.failure_return_pose_deg = failure_return_pose_deg
        self.failure_open_gripper_on_hold = bool(
            self.get_parameter("failure_open_gripper_on_hold").value
        )
        self.failure_open_gripper_joint_id = int(
            self.get_parameter("failure_open_gripper_joint_id").value
        )
        self.failure_open_gripper_angle_deg = float(
            self.get_parameter("failure_open_gripper_angle_deg").value
        )
        self.failure_open_gripper_duration_ms = max(
            100, int(self.get_parameter("failure_open_gripper_duration_ms").value)
        )
        self.failure_open_gripper_wait = bool(
            self.get_parameter("failure_open_gripper_wait").value
        )
        self.control_arm_move_joints_service = str(
            self.get_parameter("control_arm_move_joints_service").value
        ).strip()

        self._locked = False
        self._locked_once = False
        self._enabled = self.auto_start_enabled
        self._pick_active = False
        self._candidate_visible = False
        self._goal_inflight = False
        self._initialized = False
        self._startup_prepare_home_pending = self.startup_prepare_home
        self._scan_index = 0
        initial_delay_sec = (
            self.startup_prepare_home_delay_sec
            if self._startup_prepare_home_pending
            else self.startup_delay_sec
        )
        self._next_motion_time = self.get_clock().now().nanoseconds / 1e9 + initial_delay_sec
        self._active_goal_handle = None

        self._current_pose_deg: Optional[List[float]] = None
        self._last_command_pose_deg: List[float] = list(self.search_pose_deg)
        self._last_vision_result_ts = 0.0
        self._vision_point_px: Optional[List[float]] = None
        self._vision_bbox_xyxy: Optional[List[float]] = None
        self._vision_image_size: Optional[List[int]] = None
        self._vision_label = ""
        self._vision_pose_valid = False
        self._vision_pose_stable = False
        self._vision_candidate_complete = False
        self._vision_target_locked = False
        self._pending_center_feedback: Optional[dict[str, float]] = None
        self._holding_for_pose_lock = False
        self._session_active = False
        self._session_label = ""
        self._session_point_px: Optional[List[float]] = None
        self._session_bbox_xyxy: Optional[List[float]] = None
        self._session_last_seen_ts = 0.0
        self._last_centering_log_ts = 0.0
        self._failure_hold = False
        self._failure_return_pending = False
        self._pick_status_phase = ""
        self._pick_status_message = ""

        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action_name,
        )
        self._move_joints_client = None
        if self.control_arm_move_joints_service:
            self._move_joints_client = self.create_client(
                MoveArmJoints, self.control_arm_move_joints_service
            )

        self.create_subscription(Bool, target_locked_topic, self._on_target_locked, 10)
        self.create_subscription(JointState, self.joint_state_topic, self._on_joint_state, 10)
        if self.enable_service:
            self.create_service(Trigger, self.enable_service, self._on_enable_search_sweep)
        if self.disable_service:
            self.create_service(Trigger, self.disable_service, self._on_disable_search_sweep)
        if self.pick_active_topic:
            self.create_subscription(Bool, self.pick_active_topic, self._on_pick_active, 10)
        if self.pick_status_topic:
            self.create_subscription(
                String,
                self.pick_status_topic,
                self._on_pick_status,
                10,
            )
        if self.candidate_visible_topic:
            self.create_subscription(
                Bool, self.candidate_visible_topic, self._on_candidate_visible, 10
            )
        if self.vision_result_topic:
            self.create_subscription(
                DetectionResult, self.vision_result_topic, self._on_vision_result, 10
            )
        self.create_timer(0.2, self._on_timer)

        self.get_logger().info(
            "sim_search_sweep_node started: "
            f"action={self.trajectory_action_name}, joint1_scan={self.scan_joint1_angles_deg}, "
            f"vision_result_topic={self.vision_result_topic or '<disabled>'}, "
            f"vertical_joint={self.JOINT_NAMES[self.vertical_joint_index]}, "
            f"auto_start_enabled={self.auto_start_enabled}, "
            f"disable_service={self.disable_service or '<disabled>'}, "
            f"pick_active_topic={self.pick_active_topic or '<disabled>'}, "
            f"pick_status_topic={self.pick_status_topic or '<disabled>'}, "
            f"candidate_visible_topic={self.candidate_visible_topic or '<disabled>'}"
        )

    def _on_enable_search_sweep(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self._enabled and not self._failure_hold:
            response.success = True
            response.message = "search sweep already enabled"
            return response
        self._enabled = True
        self._locked = False
        self._locked_once = False
        self._goal_inflight = False
        self._failure_hold = False
        self._failure_return_pending = False
        self._startup_prepare_home_pending = self.startup_prepare_home
        self._scan_index = 0
        self._clear_runtime_tracking(clear_session=True)
        initial_delay_sec = (
            self.startup_prepare_home_delay_sec
            if self._startup_prepare_home_pending
            else self.startup_delay_sec
        )
        self._next_motion_time = self.get_clock().now().nanoseconds / 1e9 + initial_delay_sec
        self.get_logger().info("search sweep enabled by external request")
        response.success = True
        response.message = "search sweep enabled"
        return response

    def _on_disable_search_sweep(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._enabled = False
        self._locked = False
        self._locked_once = False
        self._failure_hold = False
        self._failure_return_pending = False
        self._clear_runtime_tracking(clear_session=True)
        self._next_motion_time = self.get_clock().now().nanoseconds / 1e9
        self._goal_inflight = False

        goal_handle = self._active_goal_handle
        self._active_goal_handle = None
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
                self.get_logger().info("search sweep disabled; active trajectory cancel requested")
                response.message = "search sweep disabled; active trajectory cancel requested"
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(
                    f"search sweep disable could not cancel active trajectory: {exc}"
                )
                response.message = f"search sweep disabled; cancel failed: {exc}"
            response.success = True
            return response

        self.get_logger().info("search sweep disabled")
        response.success = True
        response.message = "search sweep disabled"
        return response

    def _on_target_locked(self, msg: Bool) -> None:
        if self._failure_hold or not self._enabled:
            return
        was_locked = self._locked
        self._locked = bool(msg.data)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._locked and not was_locked:
            if self.stop_after_first_lock:
                self._locked_once = True
            self.get_logger().info("search sweep locked on target; holding position")
        elif (
            was_locked
            and not self._locked
            and not self.stop_after_first_lock
            and not self._pick_active
        ):
            if self._has_recent_visual_target(now_sec):
                self.get_logger().info(
                    "search sweep kept centering mode after hard lock dropped because the visual target is still tracked"
                )
                return
            self._next_motion_time = (
                self.get_clock().now().nanoseconds / 1e9 + self.settle_duration_sec
            )
            self.get_logger().info("search sweep lost target lock; resuming scan")

    def _on_pick_active(self, msg: Bool) -> None:
        if self._failure_hold or not self._enabled:
            return
        was_pick_active = self._pick_active
        self._pick_active = bool(msg.data)
        if self._pick_active and not was_pick_active:
            self.get_logger().info("search sweep paused: pick sequence active")
        elif was_pick_active and not self._pick_active:
            self._next_motion_time = (
                self.get_clock().now().nanoseconds / 1e9 + self.settle_duration_sec
            )
            self.get_logger().info("search sweep resumed: pick sequence inactive")

    def _on_pick_status(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data or "").strip())
        except json.JSONDecodeError:
            payload = {}
        phase = str(payload.get("phase", "")).strip().lower()
        message = str(payload.get("message", "")).strip()
        self._pick_status_phase = phase
        self._pick_status_message = message
        if self.hold_on_pick_error and phase == "error":
            self._enter_failure_hold(message or "pick sequence failed")

    def _on_candidate_visible(self, msg: Bool) -> None:
        if self._failure_hold or not self._enabled:
            return
        was_candidate_visible = self._candidate_visible
        requested_visible = bool(msg.data)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if not requested_visible and self._has_recent_visual_target(now_sec):
            self.get_logger().info(
                "search sweep ignored candidate-visible drop because the visual target session is still active"
            )
            return
        self._candidate_visible = requested_visible
        if self._candidate_visible and not was_candidate_visible:
            self.get_logger().info("search sweep candidate visible; switching to centering mode")
        elif (
            was_candidate_visible
            and not self._candidate_visible
            and not self._pick_active
            and not self._locked
        ):
            self._next_motion_time = (
                self.get_clock().now().nanoseconds / 1e9 + self.settle_duration_sec
            )
            self.get_logger().info("search sweep resumed: candidate no longer visible")

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        pose_deg: List[float] = []
        for joint_name in self.JOINT_NAMES:
            idx = name_to_index.get(joint_name)
            if idx is None or idx >= len(msg.position):
                return
            pose_deg.append(math.degrees(float(msg.position[idx])))
        self._current_pose_deg = pose_deg

    def _on_vision_result(self, msg: DetectionResult) -> None:
        if self._failure_hold or not self._enabled:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        point_px_value = (
            [float(msg.point_px[0]), float(msg.point_px[1])]
            if bool(msg.has_point)
            else None
        )
        bbox_xyxy_value = (
            [
                float(msg.bbox.x_offset),
                float(msg.bbox.y_offset),
                float(msg.bbox.x_offset + msg.bbox.width),
                float(msg.bbox.y_offset + msg.bbox.height),
            ]
            if bool(msg.has_bbox)
            else None
        )
        image_size_value = (
            [int(msg.image_width), int(msg.image_height)]
            if int(msg.image_width) > 0 and int(msg.image_height) > 0
            else None
        )
        target_label = str(msg.target_label or "").strip()
        accepted = bool(msg.accepted)

        session_accepts = self._update_target_session(
            now_sec=now_sec,
            accepted=accepted,
            label=target_label,
            point_px=point_px_value,
            bbox_xyxy=bbox_xyxy_value,
        )

        if session_accepts:
            self._vision_label = target_label
            self._vision_point_px = point_px_value
            self._vision_bbox_xyxy = bbox_xyxy_value
            self._vision_image_size = image_size_value
            self._vision_pose_valid = False
            self._vision_pose_stable = False
            self._vision_candidate_complete = bool(msg.candidate_complete)
            self._vision_target_locked = False
            self._candidate_visible = bool(msg.candidate_visible)
            self._last_vision_result_ts = now_sec
        elif self._session_active and (now_sec - self._session_last_seen_ts) <= self.target_session_timeout_sec:
            self._vision_pose_valid = False
            self._vision_pose_stable = False
            self._vision_candidate_complete = bool(msg.candidate_complete)
            self._vision_target_locked = False
            self._candidate_visible = bool(msg.candidate_visible)
        else:
            self._vision_label = target_label
            self._vision_point_px = point_px_value
            self._vision_bbox_xyxy = bbox_xyxy_value
            self._vision_image_size = image_size_value
            self._vision_pose_valid = False
            self._vision_pose_stable = False
            self._vision_candidate_complete = bool(msg.candidate_complete)
            self._vision_target_locked = False
            self._candidate_visible = bool(msg.candidate_visible)

        errors = self._compute_centering_errors()
        if errors is not None:
            self._maybe_update_response_signs(errors[0], errors[1])

    def _update_target_session(
        self,
        *,
        now_sec: float,
        accepted: bool,
        label: str,
        point_px: Optional[List[float]],
        bbox_xyxy: Optional[List[float]],
    ) -> bool:
        if not self.target_session_lock_enabled:
            return bool(accepted and point_px is not None)

        if (
            self._session_active
            and (now_sec - self._session_last_seen_ts) > self.target_session_timeout_sec
        ):
            self._clear_target_session("search sweep target session expired")

        if not accepted or point_px is None:
            return False

        if not self._session_active:
            self._session_active = True
            self._session_label = label
            self._session_point_px = list(point_px)
            self._session_bbox_xyxy = list(bbox_xyxy) if bbox_xyxy is not None else None
            self._session_last_seen_ts = now_sec
            self.get_logger().info(
                f"search sweep target session started: label={label or '<unknown>'}"
            )
            return True

        if self._session_matches(label=label, point_px=point_px, bbox_xyxy=bbox_xyxy):
            self._session_label = label or self._session_label
            self._session_point_px = list(point_px)
            self._session_bbox_xyxy = list(bbox_xyxy) if bbox_xyxy is not None else None
            self._session_last_seen_ts = now_sec
            return True

        return False

    def _session_matches(
        self,
        *,
        label: str,
        point_px: Optional[List[float]],
        bbox_xyxy: Optional[List[float]],
    ) -> bool:
        if point_px is None:
            return False

        if self._session_label and label and self._session_label != label:
            return False

        if self._session_point_px is not None:
            dx = float(point_px[0]) - float(self._session_point_px[0])
            dy = float(point_px[1]) - float(self._session_point_px[1])
            if math.hypot(dx, dy) <= self.target_session_match_distance_px:
                return True

        if self._session_bbox_xyxy is not None and bbox_xyxy is not None:
            if self._bbox_iou(self._session_bbox_xyxy, bbox_xyxy) >= self.target_session_bbox_iou_threshold:
                return True

        return False

    def _bbox_iou(self, bbox_a: List[float], bbox_b: List[float]) -> float:
        ax1, ay1, ax2, ay2 = bbox_a
        bx1, by1, bx2, by2 = bbox_b
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area <= 0.0:
            return 0.0
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        denom = area_a + area_b - inter_area
        if denom <= 1e-6:
            return 0.0
        return inter_area / denom

    def _clear_target_session(self, reason: str) -> None:
        self._session_active = False
        self._session_label = ""
        self._session_point_px = None
        self._session_bbox_xyxy = None
        self._session_last_seen_ts = 0.0
        self.get_logger().info(reason)

    def _clear_runtime_tracking(self, *, clear_session: bool) -> None:
        self._locked = False
        self._pick_active = False
        self._candidate_visible = False
        self._last_vision_result_ts = 0.0
        self._vision_point_px = None
        self._vision_bbox_xyxy = None
        self._vision_image_size = None
        self._vision_label = ""
        self._vision_pose_valid = False
        self._vision_pose_stable = False
        self._vision_candidate_complete = False
        self._vision_target_locked = False
        self._pending_center_feedback = None
        self._holding_for_pose_lock = False
        if clear_session and self._session_active:
            self._clear_target_session("search sweep cleared target session")
        elif clear_session:
            self._session_active = False
            self._session_label = ""
            self._session_point_px = None
            self._session_bbox_xyxy = None
            self._session_last_seen_ts = 0.0

    def _enter_failure_hold(self, reason: str) -> None:
        if self._failure_hold:
            return
        self._failure_hold = True
        self._failure_return_pending = True
        self._locked_once = False
        self._goal_inflight = False
        # Preserve the target session so the detector can stay target-locked
        # after failure and re-arm to the same visual target instead of falling
        # back to a scene-level prompt.
        self._clear_runtime_tracking(clear_session=False)
        self._next_motion_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().warn(
            "search sweep entering failure hold: "
            f"{reason or 'pick sequence error'}; returning arm to configured home pose"
        )

    def _on_timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if not self._enabled and not self._startup_prepare_home_pending:
            return
        if self._failure_hold:
            if self._failure_return_pending and not self._goal_inflight:
                self._failure_return_pending = False
                self._request_failure_open_gripper()
                self._send_goal(
                    list(self.failure_return_pose_deg),
                    "return to configured home pose after pick failure",
                )
            return
        if (
            self._locked
            or (self.stop_after_first_lock and self._locked_once)
            or self._pick_active
            or self._goal_inflight
            or now_sec < self._next_motion_time
        ):
            return

        if not self._trajectory_client.wait_for_server(timeout_sec=0.1):
            return

        if not self._initialized:
            self._initialized = True
            if self._startup_prepare_home_pending:
                self._next_motion_time = now_sec + self.startup_prepare_home_delay_sec
                self.get_logger().info(
                    "search sweep waiting "
                    f"{self.startup_prepare_home_delay_sec:.1f}s for startup home preparation"
                )
            else:
                self._next_motion_time = now_sec + self.startup_delay_sec
                self.get_logger().info(
                    f"search sweep waiting {self.startup_delay_sec:.1f}s for initial pose assessment"
                )
            return

        if self._startup_prepare_home_pending:
            self._startup_prepare_home_pending = False
            self._send_goal(
                list(self.failure_return_pose_deg),
                "startup prepare configured home pose",
            )
            return

        if self._should_center_target(now_sec):
            handled = self._run_centering(now_sec)
            if handled:
                return

        target_pose = list(self.search_pose_deg)
        target_pose[0] = self.scan_joint1_angles_deg[self._scan_index]
        self._scan_index = (self._scan_index + 1) % len(self.scan_joint1_angles_deg)
        self._send_goal(target_pose, f"scan joint1 -> {target_pose[0]:.1f}deg")

    def _should_center_target(self, now_sec: float) -> bool:
        return bool(self.enable_centering and self._has_recent_visual_target(now_sec))

    def _has_recent_visual_target(self, now_sec: float) -> bool:
        recent_detection = bool(
            self._last_vision_result_ts > 0.0
            and (now_sec - self._last_vision_result_ts) <= self.vision_result_stale_sec
        )
        recent_session = bool(
            self._session_active
            and self._session_last_seen_ts > 0.0
            and (now_sec - self._session_last_seen_ts) <= self.target_session_timeout_sec
            and (self._session_point_px is not None or self._session_bbox_xyxy is not None)
        )
        return bool((self._candidate_visible or recent_session) and recent_detection)

    def _run_centering(self, now_sec: float) -> bool:
        errors = self._compute_centering_errors()
        if errors is None:
            return False

        error_x_px, error_y_px = errors
        centered_x = abs(error_x_px) <= self.horizontal_center_tolerance_px
        centered_y = abs(error_y_px) <= self.vertical_center_tolerance_px
        centered_enough = bool(centered_x and centered_y and self._vision_candidate_complete)

        if centered_enough:
            self._next_motion_time = now_sec + self.centering_hold_sec
            if self._vision_pose_valid:
                self.get_logger().info(
                    "search sweep target centered; holding for stable lock"
                )
            return True

        current_pose = list(self._current_pose_deg) if self._current_pose_deg is not None else list(
            self._last_command_pose_deg
        )
        if len(current_pose) != len(self.JOINT_NAMES):
            current_pose = list(self.search_pose_deg)

        target_pose = list(current_pose)
        image_w = max(1.0, float(self._vision_image_size[0]))
        image_h = max(1.0, float(self._vision_image_size[1]))

        normalized_x = max(-1.0, min(1.0, error_x_px / max(1.0, image_w * 0.5)))
        normalized_y = max(-1.0, min(1.0, error_y_px / max(1.0, image_h * 0.5)))
        active_x = not centered_x
        active_y = not centered_y

        if not centered_x and not centered_y:
            if abs(normalized_x) >= abs(normalized_y):
                normalized_y = 0.0
                active_y = False
            else:
                normalized_x = 0.0
                active_x = False

        if active_x:
            joint1_step = (
                self.horizontal_response_sign
                * normalized_x
                * self.centering_horizontal_gain_deg
            )
            joint1_step = max(
                -self.centering_max_joint1_step_deg,
                min(self.centering_max_joint1_step_deg, joint1_step),
            )
            if abs(joint1_step) < self.centering_min_step_deg:
                joint1_step = math.copysign(
                    self.centering_min_step_deg,
                    joint1_step or normalized_x or 1.0,
                )
        else:
            joint1_step = 0.0

        if active_y:
            vertical_step = (
                self.vertical_response_sign
                * normalized_y
                * self.centering_vertical_gain_deg
            )
            vertical_step = max(
                -self.centering_max_vertical_step_deg,
                min(self.centering_max_vertical_step_deg, vertical_step),
            )
            if abs(vertical_step) < self.centering_min_step_deg:
                vertical_step = math.copysign(
                    self.centering_min_step_deg,
                    vertical_step or normalized_y or 1.0,
                )
        else:
            vertical_step = 0.0

        target_pose[0] = max(
            self.joint1_min_deg,
            min(self.joint1_max_deg, current_pose[0] + joint1_step),
        )
        vertical_index = self.vertical_joint_index
        target_pose[vertical_index] = max(
            self.vertical_joint_min_deg,
            min(self.vertical_joint_max_deg, current_pose[vertical_index] + vertical_step),
        )

        if (
            abs(target_pose[0] - current_pose[0]) < 0.25
            and abs(target_pose[vertical_index] - current_pose[vertical_index]) < 0.25
        ):
            self._next_motion_time = now_sec + self.centering_hold_sec
            return True

        self._pending_center_feedback = {
            "error_x_px": float(error_x_px),
            "error_y_px": float(error_y_px),
            "joint1_delta_deg": float(target_pose[0] - current_pose[0]),
            "vertical_delta_deg": float(target_pose[vertical_index] - current_pose[vertical_index]),
        }

        self._send_goal(
            target_pose,
            (
                "center target "
                f"dx={error_x_px:.1f}px dy={error_y_px:.1f}px "
                f"j1={target_pose[0]:.1f}deg {self.JOINT_NAMES[vertical_index]}="
                f"{target_pose[vertical_index]:.1f}deg"
            ),
        )
        return True

    def _compute_centering_errors(self) -> Optional[tuple[float, float]]:
        if self._vision_point_px is None or self._vision_image_size is None:
            return None

        image_w = max(1, int(self._vision_image_size[0]))
        image_h = max(1, int(self._vision_image_size[1]))
        center_x = (image_w - 1) * 0.5
        center_y = (image_h - 1) * 0.5
        bbox = self._vision_bbox_xyxy
        if self.prefer_bbox_center and bbox is not None:
            ref_x = float(bbox[0] + bbox[2]) * 0.5
            ref_y = float(bbox[1] + bbox[3]) * 0.5
        else:
            ref_x = float(self._vision_point_px[0])
            ref_y = float(self._vision_point_px[1])

        error_x = ref_x - center_x
        error_y = ref_y - center_y
        margin = self.bbox_edge_margin_px
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            if x1 <= margin:
                error_x = min(error_x, x1 - margin)
            elif x2 >= (image_w - margin):
                error_x = max(error_x, x2 - (image_w - margin))

            if y1 <= margin:
                error_y = min(error_y, y1 - margin)
            elif y2 >= (image_h - margin):
                error_y = max(error_y, y2 - (image_h - margin))

        return error_x, error_y

    def _maybe_update_response_signs(self, current_error_x_px: float, current_error_y_px: float) -> None:
        if not self.adaptive_centering_signs:
            self._pending_center_feedback = None
            return

        feedback = self._pending_center_feedback
        if feedback is None:
            return

        joint1_delta = float(feedback.get("joint1_delta_deg", 0.0))
        if abs(joint1_delta) >= 0.5:
            delta_error_x = current_error_x_px - float(feedback.get("error_x_px", current_error_x_px))
            if abs(delta_error_x) >= 2.0:
                inferred_sign = -1.0 if (delta_error_x / joint1_delta) > 0.0 else 1.0
                if inferred_sign != self.horizontal_response_sign:
                    self.get_logger().info(
                        f"search sweep adapted joint1 centering sign to {inferred_sign:.0f}"
                    )
                self.horizontal_response_sign = inferred_sign

        vertical_delta = float(feedback.get("vertical_delta_deg", 0.0))
        if abs(vertical_delta) >= 0.5:
            delta_error_y = current_error_y_px - float(feedback.get("error_y_px", current_error_y_px))
            if abs(delta_error_y) >= 2.0:
                inferred_sign = -1.0 if (delta_error_y / vertical_delta) > 0.0 else 1.0
                if inferred_sign != self.vertical_response_sign:
                    self.get_logger().info(
                        "search sweep adapted "
                        f"{self.JOINT_NAMES[self.vertical_joint_index]} centering sign "
                        f"to {inferred_sign:.0f}"
                    )
                self.vertical_response_sign = inferred_sign

        self._pending_center_feedback = None

    def _send_goal(self, joint_angles_deg: List[float], label: str) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [math.radians(float(v)) for v in joint_angles_deg]
        point.time_from_start.sec = int(self.motion_duration_sec)
        point.time_from_start.nanosec = int(
            (self.motion_duration_sec - int(self.motion_duration_sec)) * 1_000_000_000
        )
        goal.trajectory.points = [point]

        if label.startswith("center target"):
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if (now_sec - self._last_centering_log_ts) >= 2.0:
                self._last_centering_log_ts = now_sec
                self.get_logger().info(label)
        self._goal_inflight = True
        self._last_command_pose_deg = list(joint_angles_deg)
        send_future = self._trajectory_client.send_goal_async(goal)
        send_future.add_done_callback(lambda future: self._on_goal_response(future, label))

    def _request_failure_open_gripper(self) -> None:
        if not self.failure_open_gripper_on_hold or self._move_joints_client is None:
            return
        try:
            if not self._move_joints_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn(
                    "search sweep failure return could not open gripper: "
                    "move_joints service unavailable"
                )
                return
            request = MoveArmJoints.Request()
            request.joint_ids = [int(self.failure_open_gripper_joint_id)]
            request.angles_deg = [float(self.failure_open_gripper_angle_deg)]
            request.duration_ms = int(self.failure_open_gripper_duration_ms)
            request.wait = bool(self.failure_open_gripper_wait)
            future = self._move_joints_client.call_async(request)
            future.add_done_callback(self._on_failure_open_gripper_response)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(
                f"search sweep failure return failed to request gripper open: {exc}"
            )

    def _on_failure_open_gripper_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(
                f"search sweep failure return gripper-open call failed: {exc}"
            )
            return
        if response is None:
            self.get_logger().warn(
                "search sweep failure return gripper-open call returned no response"
            )
            return
        if not bool(response.success):
            self.get_logger().warn(
                "search sweep failure return gripper-open rejected: "
                f"{response.message}"
            )

    def _on_goal_response(self, future, label: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{label}: failed to send trajectory goal: {exc}")
            self._goal_inflight = False
            self._next_motion_time = self.get_clock().now().nanoseconds / 1e9 + 1.0
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"{label}: trajectory goal rejected")
            self._goal_inflight = False
            self._next_motion_time = self.get_clock().now().nanoseconds / 1e9 + 1.0
            return

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._on_goal_result(result, label))

    def _on_goal_result(self, future, label: str) -> None:
        self._goal_inflight = False
        self._active_goal_handle = None
        self._next_motion_time = (
            self.get_clock().now().nanoseconds / 1e9 + self.settle_duration_sec
        )
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{label}: failed waiting for trajectory result: {exc}")
            return

        result = wrapped.result
        if result.error_code != 0:
            detail = result.error_string or f"error_code={result.error_code}"
            self.get_logger().warn(f"{label}: trajectory failed: {detail}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSearchSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
