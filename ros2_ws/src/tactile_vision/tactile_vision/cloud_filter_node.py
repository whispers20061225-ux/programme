from __future__ import annotations

import copy
import json
import threading
import time
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, String
from tactile_interfaces.msg import DetectionResult, SemanticTask
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

from tactile_vision.modular_common import (
    compact_json,
    decode_depth_image,
    decode_mono8_image,
    make_mono8_image,
    make_xyz_cloud,
    open3d_available,
    quaternion_to_rotation_matrix,
    remove_dominant_plane,
    select_anchor_cluster,
    statistical_outlier_filter,
    voxel_downsample,
)


class CloudFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("cloud_filter_node")

        self.declare_parameter("detection_result_topic", "/perception/detection_result")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info"
        )
        self.declare_parameter("target_cloud_topic", "/perception/target_cloud")
        self.declare_parameter("target_cloud_markers_topic", "/perception/target_cloud_markers")
        self.declare_parameter("target_pose_topic", "/sim/perception/target_pose")
        self.declare_parameter("target_pose_camera_topic", "/perception/target_pose_camera")
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter("pick_status_topic", "/sim/task/pick_status")
        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter(
            "candidate_visible_topic", "/sim/perception/target_candidate_visible"
        )
        self.declare_parameter("debug_topic", "/perception/target_cloud_debug")
        self.declare_parameter("scene_pose_info_topic", "/sim/world/pose/info")
        self.declare_parameter("scene_target_model_name", "target_cylinder")
        self.declare_parameter("use_scene_target_pose_fallback", True)
        self.declare_parameter("scene_target_expected_position", [0.5, 0.08, 0.44])
        self.declare_parameter("scene_target_pose_match_tol_m", 0.12)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("enabled", True)
        self.declare_parameter("publish_rate_hz", 4.0)
        self.declare_parameter("detection_stale_sec", 2.0)
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 2.00)
        self.declare_parameter("bbox_margin_px", 12)
        self.declare_parameter("min_target_points", 40)
        self.declare_parameter("depth_percentile_low", 30.0)
        self.declare_parameter("depth_band_m", 0.05)
        self.declare_parameter("mask_context_hold_sec", 2.5)
        self.declare_parameter("mask_reuse_min_bbox_iou", 0.15)
        self.declare_parameter("mask_projection_enabled", True)
        self.declare_parameter("mask_depth_mad_scale", 2.5)
        self.declare_parameter("voxel_size_m", 0.004)
        self.declare_parameter("open3d_remove_outliers", True)
        self.declare_parameter("open3d_outlier_nb_neighbors", 16)
        self.declare_parameter("open3d_outlier_std_ratio", 1.5)
        self.declare_parameter("publish_min_target_points", 64)
        self.declare_parameter("plane_removal_enabled", True)
        self.declare_parameter("plane_distance_threshold_m", 0.006)
        self.declare_parameter("plane_min_inlier_ratio", 0.18)
        self.declare_parameter("plane_min_points", 48)
        self.declare_parameter("plane_max_iterations", 64)
        self.declare_parameter("plane_preferred_axis_alignment_min", 0.85)
        self.declare_parameter("plane_min_keep_ratio", 0.35)
        self.declare_parameter("cluster_selection_enabled", True)
        self.declare_parameter("cluster_tolerance_m", 0.015)
        self.declare_parameter("cluster_min_points", 18)
        self.declare_parameter("cluster_min_keep_ratio", 0.45)
        self.declare_parameter("stable_cloud_reuse_sec", 2.5)
        self.declare_parameter("stable_cloud_reuse_min_ratio", 0.60)
        self.declare_parameter("stable_cloud_reuse_max_centroid_shift_m", 0.04)
        self.declare_parameter("visual_republish_rate_hz", 8.0)
        self.declare_parameter("soft_lock_frames_required", 2)
        self.declare_parameter("soft_lock_match_distance_px", 48.0)
        self.declare_parameter("soft_lock_bbox_iou_threshold", 0.15)
        self.declare_parameter("stable_frames_required", 2)
        self.declare_parameter("lock_position_tol_m", 0.03)
        self.declare_parameter("unlock_grace_sec", 1.2)
        self.declare_parameter("unlock_miss_count_threshold", 3)
        self.declare_parameter("execution_unlock_grace_sec", 8.0)
        self.declare_parameter("execution_unlock_miss_count_threshold", 50)
        self.declare_parameter("require_semantic_task_to_lock", False)
        self.declare_parameter("log_interval_sec", 8.0)

        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.target_cloud_topic = str(self.get_parameter("target_cloud_topic").value)
        self.target_cloud_markers_topic = str(
            self.get_parameter("target_cloud_markers_topic").value
        )
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_pose_camera_topic = str(
            self.get_parameter("target_pose_camera_topic").value
        )
        self.target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.pick_status_topic = str(self.get_parameter("pick_status_topic").value)
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.candidate_visible_topic = str(self.get_parameter("candidate_visible_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.scene_pose_info_topic = str(self.get_parameter("scene_pose_info_topic").value)
        self.scene_target_model_name = str(
            self.get_parameter("scene_target_model_name").value
        ).strip() or "target_cylinder"
        self.use_scene_target_pose_fallback = bool(
            self.get_parameter("use_scene_target_pose_fallback").value
        )
        expected_position = list(self.get_parameter("scene_target_expected_position").value or [])
        if len(expected_position) != 3:
            expected_position = [0.5, 0.08, 0.44]
        self.scene_target_expected_position = np.asarray(
            [float(expected_position[0]), float(expected_position[1]), float(expected_position[2])],
            dtype=np.float32,
        )
        self.scene_target_pose_match_tol_m = max(
            0.01, float(self.get_parameter("scene_target_pose_match_tol_m").value)
        )
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.publish_rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.detection_stale_sec = max(
            0.1, float(self.get_parameter("detection_stale_sec").value)
        )
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.bbox_margin_px = max(0, int(self.get_parameter("bbox_margin_px").value))
        self.min_target_points = max(8, int(self.get_parameter("min_target_points").value))
        self.depth_percentile_low = max(
            1.0, min(99.0, float(self.get_parameter("depth_percentile_low").value))
        )
        self.depth_band_m = max(0.002, float(self.get_parameter("depth_band_m").value))
        self.mask_context_hold_sec = max(
            0.0, float(self.get_parameter("mask_context_hold_sec").value)
        )
        self.mask_reuse_min_bbox_iou = max(
            0.0, min(1.0, float(self.get_parameter("mask_reuse_min_bbox_iou").value))
        )
        self.mask_projection_enabled = bool(
            self.get_parameter("mask_projection_enabled").value
        )
        self.mask_depth_mad_scale = max(
            0.5, float(self.get_parameter("mask_depth_mad_scale").value)
        )
        self.voxel_size_m = max(0.0, float(self.get_parameter("voxel_size_m").value))
        self.open3d_remove_outliers = bool(
            self.get_parameter("open3d_remove_outliers").value
        )
        self.open3d_outlier_nb_neighbors = max(
            4, int(self.get_parameter("open3d_outlier_nb_neighbors").value)
        )
        self.open3d_outlier_std_ratio = max(
            0.1, float(self.get_parameter("open3d_outlier_std_ratio").value)
        )
        self.publish_min_target_points = max(
            self.min_target_points,
            int(self.get_parameter("publish_min_target_points").value),
        )
        self.plane_removal_enabled = bool(
            self.get_parameter("plane_removal_enabled").value
        )
        self.plane_distance_threshold_m = max(
            1e-4, float(self.get_parameter("plane_distance_threshold_m").value)
        )
        self.plane_min_inlier_ratio = max(
            0.0, min(1.0, float(self.get_parameter("plane_min_inlier_ratio").value))
        )
        self.plane_min_points = max(8, int(self.get_parameter("plane_min_points").value))
        self.plane_max_iterations = max(
            8, int(self.get_parameter("plane_max_iterations").value)
        )
        self.plane_preferred_axis_alignment_min = max(
            0.0, min(1.0, float(self.get_parameter("plane_preferred_axis_alignment_min").value))
        )
        self.plane_min_keep_ratio = max(
            0.0, min(1.0, float(self.get_parameter("plane_min_keep_ratio").value))
        )
        self.cluster_selection_enabled = bool(
            self.get_parameter("cluster_selection_enabled").value
        )
        self.cluster_tolerance_m = max(
            1e-4, float(self.get_parameter("cluster_tolerance_m").value)
        )
        self.cluster_min_points = max(
            2, int(self.get_parameter("cluster_min_points").value)
        )
        self.cluster_min_keep_ratio = max(
            0.0, min(1.0, float(self.get_parameter("cluster_min_keep_ratio").value))
        )
        self.stable_cloud_reuse_sec = max(
            0.0, float(self.get_parameter("stable_cloud_reuse_sec").value)
        )
        self.stable_cloud_reuse_min_ratio = max(
            0.0, min(1.0, float(self.get_parameter("stable_cloud_reuse_min_ratio").value))
        )
        self.stable_cloud_reuse_max_centroid_shift_m = max(
            0.0, float(self.get_parameter("stable_cloud_reuse_max_centroid_shift_m").value)
        )
        self.visual_republish_rate_hz = max(
            0.2, float(self.get_parameter("visual_republish_rate_hz").value)
        )
        self.soft_lock_frames_required = max(
            1, int(self.get_parameter("soft_lock_frames_required").value)
        )
        self.soft_lock_match_distance_px = max(
            1.0, float(self.get_parameter("soft_lock_match_distance_px").value)
        )
        self.soft_lock_bbox_iou_threshold = max(
            0.0, min(1.0, float(self.get_parameter("soft_lock_bbox_iou_threshold").value))
        )
        self.stable_frames_required = max(
            1, int(self.get_parameter("stable_frames_required").value)
        )
        self.lock_position_tol_m = max(
            0.001, float(self.get_parameter("lock_position_tol_m").value)
        )
        self.unlock_grace_sec = max(0.0, float(self.get_parameter("unlock_grace_sec").value))
        self.unlock_miss_count_threshold = max(
            1, int(self.get_parameter("unlock_miss_count_threshold").value)
        )
        self.execution_unlock_grace_sec = max(
            0.0, float(self.get_parameter("execution_unlock_grace_sec").value)
        )
        self.execution_unlock_miss_count_threshold = max(
            1, int(self.get_parameter("execution_unlock_miss_count_threshold").value)
        )
        self.require_semantic_task_to_lock = bool(
            self.get_parameter("require_semantic_task_to_lock").value
        )
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._detection_lock = threading.Lock()
        self._sensor_lock = threading.Lock()
        self._latest_detection: Optional[DetectionResult] = None
        self._latest_detection_ts = 0.0
        self._latest_masked_detection: Optional[DetectionResult] = None
        self._latest_masked_detection_ts = 0.0
        self._latest_depth_msg: Optional[Image] = None
        self._latest_camera_info: Optional[CameraInfo] = None
        self._soft_last_point_px: Optional[np.ndarray] = None
        self._soft_last_bbox_xyxy: Optional[np.ndarray] = None
        self._soft_last_label = ""
        self._soft_stable_count = 0
        self._soft_locked = False
        self._last_world_position: Optional[np.ndarray] = None
        self._stable_count = 0
        self._last_lock_state = False
        self._last_reset_reason = ""
        self._last_summary = ""
        self._unlock_pending_since = 0.0
        self._unlock_miss_count = 0
        self._last_target_cloud_points: Optional[np.ndarray] = None
        self._last_target_pose_position: Optional[np.ndarray] = None
        self._last_target_pose_camera_position: Optional[np.ndarray] = None
        self._last_target_pose_camera_frame = ""
        self._last_stable_target_cloud_points: Optional[np.ndarray] = None
        self._last_stable_target_pose_position: Optional[np.ndarray] = None
        self._last_stable_target_pose_camera_position: Optional[np.ndarray] = None
        self._last_stable_target_pose_camera_frame = ""
        self._last_stable_target_cloud_ts = 0.0
        self._scene_target_pose_position: Optional[np.ndarray] = None
        self._semantic_task_active = not self.require_semantic_task_to_lock
        self._execution_active = False
        self._pick_phase = "idle"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
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

        self.create_subscription(
            DetectionResult, self.detection_result_topic, self._on_detection_result, qos_reliable
        )
        self.create_subscription(
            SemanticTask, self.semantic_task_topic, self._on_semantic_task, qos_reliable
        )
        self.create_subscription(String, self.pick_status_topic, self._on_pick_status, qos_reliable)
        self.create_subscription(Image, self.depth_topic, self._on_depth_image, qos_sensor)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, qos_sensor
        )
        self.create_subscription(TFMessage, self.scene_pose_info_topic, self._on_scene_pose_info, qos_reliable)

        self.target_cloud_pub = self.create_publisher(
            PointCloud2, self.target_cloud_topic, qos_latched
        )
        self.target_cloud_marker_pub = self.create_publisher(
            Marker, self.target_cloud_markers_topic, qos_latched
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped, self.target_pose_topic, qos_latched
        )
        self.target_pose_camera_pub = self.create_publisher(
            PoseStamped, self.target_pose_camera_topic, qos_latched
        )
        self.target_locked_pub = self.create_publisher(
            Bool, self.target_locked_topic, qos_reliable
        )
        self.candidate_visible_pub = self.create_publisher(
            Bool, self.candidate_visible_topic, qos_reliable
        )
        self.debug_pub = self.create_publisher(String, self.debug_topic, qos_reliable)
        self.create_timer(1.0 / self.publish_rate_hz, self._process_latest)
        self.create_timer(1.0 / self.visual_republish_rate_hz, self._republish_last_visuals)

        self.get_logger().info(
            "cloud_filter_node started: "
            f"detection={self.detection_result_topic}, target_cloud={self.target_cloud_topic}, "
            f"target_pose={self.target_pose_topic}, target_pose_camera={self.target_pose_camera_topic}, "
            f"target_frame={self.target_frame}, "
            f"open3d_available={open3d_available()}"
        )

    def _on_detection_result(self, msg: DetectionResult) -> None:
        with self._detection_lock:
            self._latest_detection = msg
            self._latest_detection_ts = time.time()
            if (
                bool(msg.accepted)
                and bool(msg.has_bbox)
                and bool(msg.has_mask)
                and self._is_stable_mask_detection(msg)
            ):
                self._latest_masked_detection = copy.deepcopy(msg)
                self._latest_masked_detection_ts = self._latest_detection_ts

    def _detection_inference_scope(self, detection: Optional[DetectionResult]) -> str:
        backend = str(getattr(detection, "backend", "") or "").strip().lower()
        if "+" not in backend:
            return ""
        return backend.rsplit("+", 1)[-1]

    def _is_stable_mask_detection(self, detection: Optional[DetectionResult]) -> bool:
        scope = self._detection_inference_scope(detection)
        if not scope:
            return True
        return scope == "full_frame"

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        self._semantic_task_active = bool(
            str(msg.prompt_text or "").strip()
            or str(msg.target_label or "").strip()
            or str(msg.target_hint or "").strip()
            or str(msg.task or "").strip()
        )

    def _on_pick_status(self, msg: String) -> None:
        phase = ""
        executing = False
        try:
            payload = json.loads(str(msg.data or "{}"))
            phase = str(payload.get("phase") or "").strip().lower()
            executing = bool(payload.get("executing")) or phase == "executing"
        except Exception:
            phase = str(msg.data or "").strip().lower()
            executing = phase == "executing"
        self._pick_phase = phase or self._pick_phase
        self._execution_active = executing

    def _on_depth_image(self, msg: Image) -> None:
        with self._sensor_lock:
            self._latest_depth_msg = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._sensor_lock:
            self._latest_camera_info = msg

    def _on_scene_pose_info(self, msg: TFMessage) -> None:
        if not self.use_scene_target_pose_fallback:
            return
        best_position: Optional[np.ndarray] = None
        best_distance = float("inf")
        for transform in list(msg.transforms or []):
            child_frame_id = str(transform.child_frame_id or "").strip()
            position = np.asarray(
                [
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    float(transform.transform.translation.z),
                ],
                dtype=np.float32,
            )
            if child_frame_id != self.scene_target_model_name:
                if child_frame_id:
                    continue
                distance = float(np.linalg.norm(position - self.scene_target_expected_position))
                if distance > self.scene_target_pose_match_tol_m or distance >= best_distance:
                    continue
                best_distance = distance
                best_position = position
                continue
            self._scene_target_pose_position = position
            return
        if best_position is not None:
            self._scene_target_pose_position = best_position

    def _process_latest(self) -> None:
        if not self.enabled:
            return

        now_sec = time.time()
        with self._detection_lock:
            detection = self._latest_detection
            detection_ts = self._latest_detection_ts
        with self._sensor_lock:
            depth_msg = self._latest_depth_msg
            camera_info = self._latest_camera_info

        if detection is None or depth_msg is None or camera_info is None:
            return
        had_soft_tracking = bool(self._soft_locked or self._soft_stable_count > 0)
        if now_sec - detection_ts > self.detection_stale_sec:
            if self._hold_hard_lock_grace(
                "stale_detection",
                now_sec,
                candidate_visible=bool(self._soft_locked or self._last_lock_state),
            ):
                return
            if had_soft_tracking:
                self._clear_hard_state()
                self._publish_soft_state(
                    "stale_detection",
                    str(detection.target_label or self._soft_last_label or ""),
                )
                return
            self._reset_state("stale_detection")
            return

        soft_visible = self._update_soft_lock_from_detection(detection)
        if not soft_visible:
            if self._hold_hard_lock_grace("soft_lock_lost", now_sec, candidate_visible=False):
                return
            if had_soft_tracking:
                self._clear_hard_state()
                self._publish_soft_state(
                    str(detection.reason or "soft_lock_lost"),
                    str(detection.target_label or self._soft_last_label or ""),
                )
                return
            self._reset_state(str(detection.reason or "soft_lock_lost"))
            return

        depth_image = decode_depth_image(depth_msg)
        if depth_image is None:
            if self._hold_hard_lock_grace("invalid_depth_image", now_sec, candidate_visible=True):
                return
            if self._publish_stable_cloud_reuse(
                detection=detection,
                reason="invalid_depth_image",
                now_sec=now_sec,
                current_points=0,
            ):
                return
            self._clear_hard_state()
            self._publish_soft_state("invalid_depth_image", str(detection.target_label or ""))
            return

        detection_context, detection_source, mask_source = self._select_detection_context(
            detection,
            depth_image.shape[:2],
        )
        extraction = self._extract_target_cloud(
            depth_image,
            camera_info,
            depth_msg,
            detection_context,
        )
        if extraction is None:
            if self._publish_stable_cloud_reuse(
                detection=detection,
                reason="extraction_failed",
                now_sec=now_sec,
                current_points=0,
            ):
                return
            if self._publish_soft_scene_pose(detection=detection, reason="scene_target_pose"):
                self._clear_hard_state()
                return
            anchor_pose = self._extract_target_anchor_pose(
                depth_image, camera_info, depth_msg, detection_context
            )
            if anchor_pose is not None:
                self._clear_hard_state()
                self._publish_soft_anchor_pose(
                    detection=detection,
                    anchor_pose=anchor_pose,
                    reason="insufficient_target_points",
                )
                return
            if self._hold_hard_lock_grace(
                "insufficient_target_points", now_sec, candidate_visible=True
            ):
                return
            self._clear_hard_state()
            self._publish_soft_state("insufficient_target_points", str(detection.target_label or ""))
            return

        points_world = np.asarray(extraction["points_world"], dtype=np.float32)
        source_frame = str(extraction["source_frame"])
        anchor_depth = float(extraction["anchor_depth"])
        candidate_mask_pixels = int(extraction["candidate_mask_pixels"])
        target_mask_pixels = int(extraction["target_mask_pixels"])
        rotation = np.asarray(extraction["rotation"], dtype=np.float64)
        translation = np.asarray(extraction["translation"], dtype=np.float64)
        anchor_point_world = np.asarray(extraction["anchor_point_world"], dtype=np.float32)
        target_mask_mode = str(extraction["target_mask_mode"])
        if points_world.shape[0] < self.min_target_points:
            if self._publish_stable_cloud_reuse(
                detection=detection,
                reason="insufficient_target_points",
                now_sec=now_sec,
                current_points=int(points_world.shape[0]),
            ):
                return
            if self._publish_soft_scene_pose(detection=detection, reason="scene_target_pose"):
                self._clear_hard_state()
                return
            anchor_pose = self._extract_target_anchor_pose(
                depth_image, camera_info, depth_msg, detection_context
            )
            if anchor_pose is not None:
                self._clear_hard_state()
                self._publish_soft_anchor_pose(
                    detection=detection,
                    anchor_pose=anchor_pose,
                    reason="insufficient_target_points",
                )
                return
            if self._hold_hard_lock_grace(
                "insufficient_target_points", now_sec, candidate_visible=True
            ):
                return
            self._clear_hard_state()
            self._publish_soft_state("insufficient_target_points", str(detection.target_label or ""))
            return

        extraction_points = int(points_world.shape[0])
        points_world = voxel_downsample(points_world, self.voxel_size_m)
        voxel_points = int(points_world.shape[0])
        if points_world.shape[0] < self.min_target_points:
            if self._publish_stable_cloud_reuse(
                detection=detection,
                reason="insufficient_target_points_after_voxel",
                now_sec=now_sec,
                current_points=int(points_world.shape[0]),
            ):
                return
            if self._publish_soft_scene_pose(detection=detection, reason="scene_target_pose"):
                self._clear_hard_state()
                return
            anchor_pose = self._extract_target_anchor_pose(
                depth_image, camera_info, depth_msg, detection_context
            )
            if anchor_pose is not None:
                self._clear_hard_state()
                self._publish_soft_anchor_pose(
                    detection=detection,
                    anchor_pose=anchor_pose,
                    reason="insufficient_target_points_after_voxel",
                )
                return
            if self._hold_hard_lock_grace(
                "insufficient_target_points_after_voxel", now_sec, candidate_visible=True
            ):
                return
            self._clear_hard_state()
            self._publish_soft_state(
                "insufficient_target_points_after_voxel",
                str(detection.target_label or ""),
            )
            return

        has_instance_mask = bool(detection_context.has_mask)
        filter_pipeline_used = (
            "mask_instance_passthrough"
            if has_instance_mask
            else "bbox_fallback_optional_filters"
        )
        plane_removed_points = 0
        plane_filter_applied = False
        plane_filter_rejected = False
        plane_filter_skipped = has_instance_mask
        plane_keep_ratio = 1.0
        plane_points = voxel_points
        outlier_filtered_points = voxel_points
        cluster_count = 0
        cluster_filter_applied = False
        cluster_filter_rejected = False
        cluster_filter_skipped = has_instance_mask
        cluster_keep_ratio = 1.0
        analysis_points = np.asarray(points_world, dtype=np.float32).copy()
        if self.plane_removal_enabled and not has_instance_mask:
            plane_input_points = max(1, int(analysis_points.shape[0]))
            preferred_axis = (
                np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
                if str(self.target_frame or "").strip().lower() == "world"
                else None
            )
            plane_filtered_points, plane_removed_points = remove_dominant_plane(
                analysis_points,
                distance_threshold_m=self.plane_distance_threshold_m,
                min_inlier_ratio=self.plane_min_inlier_ratio,
                min_points=max(self.min_target_points, self.plane_min_points),
                max_iterations=self.plane_max_iterations,
                preferred_axis=preferred_axis,
                preferred_axis_alignment_min=self.plane_preferred_axis_alignment_min,
            )
            plane_keep_ratio = float(plane_filtered_points.shape[0]) / float(plane_input_points)
            if (
                plane_filtered_points.shape[0] >= self.min_target_points
                and plane_keep_ratio >= self.plane_min_keep_ratio
            ):
                analysis_points = plane_filtered_points
                plane_filter_applied = True
            elif plane_filtered_points.shape[0] >= self.min_target_points:
                plane_filter_rejected = True
        plane_points = int(analysis_points.shape[0])
        if self.open3d_remove_outliers and not has_instance_mask:
            filtered_points = statistical_outlier_filter(
                analysis_points,
                self.open3d_outlier_nb_neighbors,
                self.open3d_outlier_std_ratio,
            )
            if filtered_points.shape[0] >= self.min_target_points:
                analysis_points = filtered_points
        outlier_filtered_points = int(analysis_points.shape[0])
        if self.cluster_selection_enabled and not has_instance_mask:
            cluster_input_points = max(1, int(analysis_points.shape[0]))
            clustered_points, cluster_count = select_anchor_cluster(
                analysis_points,
                tolerance_m=self.cluster_tolerance_m,
                min_points=max(self.cluster_min_points, self.min_target_points),
                anchor_point_xyz=anchor_point_world,
            )
            cluster_keep_ratio = float(clustered_points.shape[0]) / float(cluster_input_points)
            if (
                clustered_points.shape[0] >= self.min_target_points
                and cluster_keep_ratio >= self.cluster_min_keep_ratio
            ):
                analysis_points = clustered_points
                cluster_filter_applied = True
            elif clustered_points.shape[0] >= self.min_target_points:
                cluster_filter_rejected = True
        clustered_points = int(analysis_points.shape[0])
        current_centroid = np.mean(points_world, axis=0)
        if (
            self._last_stable_target_cloud_points is not None
            and self._last_stable_target_pose_position is not None
            and self.stable_cloud_reuse_sec > 0.0
            and (now_sec - float(self._last_stable_target_cloud_ts or 0.0)) <= self.stable_cloud_reuse_sec
        ):
            stable_point_count = int(self._last_stable_target_cloud_points.shape[0])
            stable_centroid = np.asarray(
                self._last_stable_target_pose_position, dtype=np.float32
            )
            shrink_threshold = max(
                self.publish_min_target_points,
                int(np.ceil(stable_point_count * self.stable_cloud_reuse_min_ratio)),
            )
            centroid_shift = float(np.linalg.norm(current_centroid - stable_centroid))
            if (
                int(points_world.shape[0]) < shrink_threshold
                and centroid_shift <= self.stable_cloud_reuse_max_centroid_shift_m
            ):
                if self._publish_stable_cloud_reuse(
                    detection=detection,
                    reason="transient_cloud_shrink",
                    now_sec=now_sec,
                    current_points=int(points_world.shape[0]),
                ):
                    return

        if int(points_world.shape[0]) < self.publish_min_target_points:
            if self._publish_stable_cloud_reuse(
                detection=detection,
                reason="below_publish_min_target_points",
                now_sec=now_sec,
                current_points=int(points_world.shape[0]),
            ):
                return

        centroid = current_centroid
        if self._last_world_position is None:
            self._stable_count = 1
        elif np.linalg.norm(centroid - self._last_world_position) <= self.lock_position_tol_m:
            self._stable_count += 1
        else:
            self._stable_count = 1
        self._last_world_position = centroid
        locked = bool(
            self._soft_locked and self._stable_count >= self.stable_frames_required
        )
        semantic_gate_allows_lock, semantic_gate_reason = self._semantic_gate_state(
            detection
        )
        if not semantic_gate_allows_lock:
            locked = False
        self._unlock_pending_since = 0.0
        self._unlock_miss_count = 0

        cloud_msg = make_xyz_cloud(points_world, self.target_frame, depth_msg.header.stamp)
        self.target_cloud_pub.publish(cloud_msg)
        self.target_cloud_marker_pub.publish(
            self._make_target_cloud_marker(points_world, depth_msg.header.stamp)
        )
        pose_msg = PoseStamped()
        pose_msg.header.stamp = depth_msg.header.stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = float(centroid[0])
        pose_msg.pose.position.y = float(centroid[1])
        pose_msg.pose.position.z = float(centroid[2])
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)
        centroid_camera = (centroid.astype(np.float64) - translation) @ rotation
        pose_camera_msg = PoseStamped()
        pose_camera_msg.header.stamp = depth_msg.header.stamp
        pose_camera_msg.header.frame_id = source_frame
        pose_camera_msg.pose.position.x = float(centroid_camera[0])
        pose_camera_msg.pose.position.y = float(centroid_camera[1])
        pose_camera_msg.pose.position.z = float(centroid_camera[2])
        pose_camera_msg.pose.orientation.w = 1.0
        self.target_pose_camera_pub.publish(pose_camera_msg)
        self._last_target_cloud_points = points_world.astype(np.float32, copy=True)
        self._last_target_pose_position = centroid.astype(np.float32, copy=True)
        self._last_target_pose_camera_position = np.asarray(
            centroid_camera, dtype=np.float32
        ).copy()
        self._last_target_pose_camera_frame = str(source_frame)
        self._update_stable_cloud_cache(
            points_world=points_world,
            centroid=centroid,
            centroid_camera=centroid_camera,
            source_frame=source_frame,
            now_sec=now_sec,
        )
        self.target_locked_pub.publish(Bool(data=locked))
        self.candidate_visible_pub.publish(Bool(data=True))

        point_px = None
        if bool(detection.has_point) and len(detection.point_px) >= 2:
            point_px = [int(detection.point_px[0]), int(detection.point_px[1])]
        bbox_xyxy = None
        if bool(detection.has_bbox):
            bbox_xyxy = [
                int(detection.bbox.x_offset),
                int(detection.bbox.y_offset),
                int(detection.bbox.x_offset + detection.bbox.width),
                int(detection.bbox.y_offset + detection.bbox.height),
            ]

        debug_msg = String()
        debug_msg.data = compact_json(
            {
                "status": "ok",
                "target_label": str(detection.target_label or ""),
                "target_points": int(points_world.shape[0]),
                "extraction_points": extraction_points,
                "voxel_points": voxel_points,
                "plane_points": plane_points,
                "outlier_filtered_points": outlier_filtered_points,
                "clustered_points": clustered_points,
                "point_px": point_px,
                "bbox_xyxy": bbox_xyxy,
                "source_frame": str(source_frame),
                "anchor_depth_m": round(float(anchor_depth), 4),
                "candidate_mask_pixels": int(candidate_mask_pixels),
                "target_mask_pixels": int(target_mask_pixels),
                "detection_source": detection_source,
                "mask_source": mask_source,
                "target_mask_mode": target_mask_mode,
                "plane_removed_points": int(plane_removed_points),
                "cluster_count": int(cluster_count),
                "publish_min_target_points": int(self.publish_min_target_points),
                "has_instance_mask": bool(has_instance_mask),
                "filter_pipeline_used": filter_pipeline_used,
                "plane_filter_applied": bool(plane_filter_applied),
                "plane_filter_rejected": bool(plane_filter_rejected),
                "plane_filter_skipped": bool(plane_filter_skipped),
                "plane_keep_ratio": round(float(plane_keep_ratio), 4),
                "cluster_filter_applied": bool(cluster_filter_applied),
                "cluster_filter_rejected": bool(cluster_filter_rejected),
                "cluster_filter_skipped": bool(cluster_filter_skipped),
                "cluster_keep_ratio": round(float(cluster_keep_ratio), 4),
                "centroid_xyz_camera": [
                    round(float(centroid_camera[0]), 4),
                    round(float(centroid_camera[1]), 4),
                    round(float(centroid_camera[2]), 4),
                ],
                "centroid_xyz": [
                    round(float(centroid[0]), 4),
                    round(float(centroid[1]), 4),
                    round(float(centroid[2]), 4),
                ],
                "soft_locked": bool(self._soft_locked),
                "soft_stable_count": int(self._soft_stable_count),
                "locked": locked,
                "stable_count": int(self._stable_count),
                "semantic_task_active": bool(self._semantic_task_active),
                "semantic_gate_allows_lock": bool(semantic_gate_allows_lock),
                "semantic_gate_reason": semantic_gate_reason,
                "pick_phase": self._pick_phase,
                "mask_used": bool(detection_context.has_mask),
                "open3d_filtered": bool(self.open3d_remove_outliers and open3d_available()),
            }
        )
        self.debug_pub.publish(debug_msg)
        summary_key = (
            str(detection.target_label or "<none>"),
            bool(self._soft_locked),
            bool(locked),
        )
        if locked != self._last_lock_state or str(summary_key) != self._last_summary:
            self._last_summary = str(summary_key)
            self._last_lock_state = locked
            self._last_reset_reason = ""
            self.get_logger().info(
                "target tracking decision: "
                f"label={str(detection.target_label or '<none>')} "
                f"soft_locked={self._soft_locked} "
                f"locked={locked} "
                f"points={points_world.shape[0]} "
                f"soft_frames={self._soft_stable_count} "
                f"stable_frames={self._stable_count} "
                f"semantic_active={self._semantic_task_active} "
                f"semantic_gate={semantic_gate_reason} "
                f"phase={self._pick_phase}"
            )

    def _update_soft_lock_from_detection(self, detection: DetectionResult) -> bool:
        if not bool(detection.accepted) or not bool(detection.candidate_visible):
            self._clear_soft_state()
            return False

        point_px: Optional[np.ndarray] = None
        if bool(detection.has_point):
            point_px = np.asarray(
                [float(detection.point_px[0]), float(detection.point_px[1])],
                dtype=np.float32,
            )

        bbox_xyxy: Optional[np.ndarray] = None
        if bool(detection.has_bbox):
            bbox_xyxy = np.asarray(
                [
                    float(detection.bbox.x_offset),
                    float(detection.bbox.y_offset),
                    float(detection.bbox.x_offset + detection.bbox.width),
                    float(detection.bbox.y_offset + detection.bbox.height),
                ],
                dtype=np.float32,
            )
            if point_px is None:
                point_px = np.asarray(
                    [
                        float(bbox_xyxy[0] + bbox_xyxy[2]) * 0.5,
                        float(bbox_xyxy[1] + bbox_xyxy[3]) * 0.5,
                    ],
                    dtype=np.float32,
                )

        if point_px is None:
            self._clear_soft_state()
            return False

        label = str(detection.target_label or "").strip()
        matched = False
        if self._soft_last_point_px is not None:
            if self._soft_last_label and label and self._soft_last_label != label:
                matched = False
            else:
                distance_px = float(np.linalg.norm(point_px - self._soft_last_point_px))
                matched = distance_px <= self.soft_lock_match_distance_px
                if (
                    not matched
                    and self._soft_last_bbox_xyxy is not None
                    and bbox_xyxy is not None
                    and self._bbox_iou(self._soft_last_bbox_xyxy, bbox_xyxy)
                    >= self.soft_lock_bbox_iou_threshold
                ):
                    matched = True

        if matched:
            self._soft_stable_count += 1
        else:
            self._soft_stable_count = 1
        self._soft_last_point_px = point_px.astype(np.float32, copy=True)
        self._soft_last_bbox_xyxy = (
            bbox_xyxy.astype(np.float32, copy=True) if bbox_xyxy is not None else None
        )
        self._soft_last_label = label
        self._soft_locked = bool(self._soft_stable_count >= self.soft_lock_frames_required)
        return True

    def _bbox_iou(self, bbox_a: np.ndarray, bbox_b: np.ndarray) -> float:
        ax1, ay1, ax2, ay2 = [float(v) for v in bbox_a]
        bx1, by1, bx2, by2 = [float(v) for v in bbox_b]
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

    def _detection_bbox_xyxy(self, detection: Optional[DetectionResult]) -> Optional[np.ndarray]:
        if detection is None or not bool(detection.has_bbox):
            return None
        return np.asarray(
            [
                float(detection.bbox.x_offset),
                float(detection.bbox.y_offset),
                float(detection.bbox.x_offset + detection.bbox.width),
                float(detection.bbox.y_offset + detection.bbox.height),
            ],
            dtype=np.float32,
        )

    def _normalized_detection_label(self, detection: Optional[DetectionResult]) -> str:
        return str(getattr(detection, "target_label", "") or "").strip().lower()

    def _can_reuse_masked_detection(
        self,
        primary_detection: Optional[DetectionResult],
        cached_masked_detection: Optional[DetectionResult],
        cached_masked_detection_ts: float,
    ) -> bool:
        if cached_masked_detection is None:
            return False
        if not bool(cached_masked_detection.accepted) or not bool(cached_masked_detection.has_mask):
            return False
        cached_bbox = self._detection_bbox_xyxy(cached_masked_detection)
        if cached_bbox is None:
            return False
        if (time.time() - float(cached_masked_detection_ts or 0.0)) > self.mask_context_hold_sec:
            return False
        if primary_detection is None:
            return True
        primary_bbox = self._detection_bbox_xyxy(primary_detection)
        if primary_bbox is None:
            return True
        if (
            str(primary_detection.header.frame_id or "").strip()
            and str(cached_masked_detection.header.frame_id or "").strip()
            and str(primary_detection.header.frame_id or "").strip()
            != str(cached_masked_detection.header.frame_id or "").strip()
        ):
            return False
        primary_label = self._normalized_detection_label(primary_detection)
        cached_label = self._normalized_detection_label(cached_masked_detection)
        if primary_label and cached_label and primary_label != cached_label:
            return False
        return self._bbox_iou(primary_bbox, cached_bbox) >= self.mask_reuse_min_bbox_iou

    def _project_cached_mask_to_bbox(
        self,
        *,
        cached_detection: DetectionResult,
        target_detection: DetectionResult,
        image_shape: tuple[int, int],
    ) -> Optional[np.ndarray]:
        cached_mask = decode_mono8_image(cached_detection.mask)
        if cached_mask is None or cached_mask.shape[:2] != tuple(image_shape):
            return None
        src_bbox = self._detection_bbox_xyxy(cached_detection)
        dst_bbox = self._detection_bbox_xyxy(target_detection)
        if src_bbox is None or dst_bbox is None:
            return None

        image_h, image_w = int(image_shape[0]), int(image_shape[1])
        src_x1 = max(0, min(image_w - 1, int(round(float(src_bbox[0])))))
        src_y1 = max(0, min(image_h - 1, int(round(float(src_bbox[1])))))
        src_x2 = max(src_x1 + 1, min(image_w, int(round(float(src_bbox[2])))))
        src_y2 = max(src_y1 + 1, min(image_h, int(round(float(src_bbox[3])))))
        dst_x1 = max(0, min(image_w - 1, int(round(float(dst_bbox[0])))))
        dst_y1 = max(0, min(image_h - 1, int(round(float(dst_bbox[1])))))
        dst_x2 = max(dst_x1 + 1, min(image_w, int(round(float(dst_bbox[2])))))
        dst_y2 = max(dst_y1 + 1, min(image_h, int(round(float(dst_bbox[3])))))

        src_crop = np.asarray(cached_mask[src_y1:src_y2, src_x1:src_x2], dtype=np.uint8)
        if src_crop.size == 0 or int(np.count_nonzero(src_crop)) <= 0:
            return None
        resized_mask = cv2.resize(
            src_crop,
            (max(1, dst_x2 - dst_x1), max(1, dst_y2 - dst_y1)),
            interpolation=cv2.INTER_NEAREST,
        )
        projected = np.zeros((image_h, image_w), dtype=np.uint8)
        projected[dst_y1:dst_y2, dst_x1:dst_x2] = resized_mask
        if int(np.count_nonzero(projected)) <= 0:
            return None
        return (projected > 0).astype(np.uint8) * 255

    def _select_detection_context(
        self,
        detection: DetectionResult,
        image_shape: tuple[int, int],
    ) -> tuple[DetectionResult, str, str]:
        if bool(detection.has_mask) and self._is_stable_mask_detection(detection):
            return detection, "latest_detection", "live_mask"

        with self._detection_lock:
            cached_masked_detection = copy.deepcopy(self._latest_masked_detection)
            cached_masked_detection_ts = self._latest_masked_detection_ts

        if not self._can_reuse_masked_detection(
            detection,
            cached_masked_detection,
            cached_masked_detection_ts,
        ):
            return detection, "latest_detection", "no_mask"

        if cached_masked_detection is not None:
            return cached_masked_detection, "cached_masked_detection", "cached_mask"

        if bool(detection.has_mask):
            return detection, "latest_detection", "unstable_live_mask"

        if (
            self.mask_projection_enabled
            and cached_masked_detection is not None
            and self._detection_bbox_xyxy(detection) is not None
        ):
            projected_mask = self._project_cached_mask_to_bbox(
                cached_detection=cached_masked_detection,
                target_detection=detection,
                image_shape=image_shape,
            )
            if projected_mask is not None:
                selected_detection = copy.deepcopy(detection)
                selected_detection.has_mask = True
                selected_detection.mask = make_mono8_image(
                    projected_mask,
                    frame_id=str(selected_detection.header.frame_id or ""),
                    stamp=selected_detection.header.stamp,
                )
                return selected_detection, "latest_detection", "projected_cached_mask"

        return detection, "latest_detection", "no_mask"

    def _select_anchor_pixel(
        self,
        *,
        detection: DetectionResult,
        candidate_mask: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> Optional[tuple[int, int, int, int]]:
        point_u = None
        point_v = None
        if bool(detection.has_point) and len(detection.point_px) >= 2:
            point_u = int(detection.point_px[0])
            point_v = int(detection.point_px[1])
        elif bool(detection.has_bbox):
            point_u = int(detection.bbox.x_offset + detection.bbox.width * 0.5)
            point_v = int(detection.bbox.y_offset + detection.bbox.height * 0.5)

        if point_u is None or point_v is None:
            pixels = np.argwhere(candidate_mask)
            if pixels.size == 0:
                return None
            median_idx = int(len(pixels) * 0.5)
            point_v = int(pixels[median_idx, 0] + y1)
            point_u = int(pixels[median_idx, 1] + x1)

        roi_u = int(np.clip(point_u - x1, 0, max(0, x2 - x1 - 1)))
        roi_v = int(np.clip(point_v - y1, 0, max(0, y2 - y1 - 1)))
        if not bool(candidate_mask[roi_v, roi_u]):
            pixels = np.argwhere(candidate_mask)
            if pixels.size == 0:
                return None
            distances = (pixels[:, 1] - roi_u) ** 2 + (pixels[:, 0] - roi_v) ** 2
            nearest = pixels[int(np.argmin(distances))]
            roi_v = int(nearest[0])
            roi_u = int(nearest[1])
            point_v = int(roi_v + y1)
            point_u = int(roi_u + x1)

        return point_u, point_v, roi_u, roi_v

    def _extract_seeded_component(
        self,
        *,
        binary_mask: np.ndarray,
        seed_u: int,
        seed_v: int,
    ) -> np.ndarray:
        mask_u8 = np.asarray(binary_mask, dtype=np.uint8)
        if int(np.count_nonzero(mask_u8)) <= 0:
            return np.asarray(binary_mask, dtype=bool)
        num_labels, labels = cv2.connectedComponents(mask_u8, connectivity=8)
        if int(num_labels) <= 1:
            return np.asarray(binary_mask, dtype=bool)

        selected_label = int(labels[seed_v, seed_u]) if 0 <= seed_v < labels.shape[0] and 0 <= seed_u < labels.shape[1] else 0
        if selected_label <= 0:
            pixels = np.argwhere(labels > 0)
            if pixels.size == 0:
                return np.asarray(binary_mask, dtype=bool)
            distances = (pixels[:, 1] - seed_u) ** 2 + (pixels[:, 0] - seed_v) ** 2
            nearest = pixels[int(np.argmin(distances))]
            selected_label = int(labels[int(nearest[0]), int(nearest[1])])
        component_mask = labels == selected_label
        return np.asarray(component_mask, dtype=bool)

    def _semantic_gate_state(self, detection: DetectionResult) -> tuple[bool, str]:
        if not self.require_semantic_task_to_lock:
            return True, "disabled"
        if self._semantic_task_active:
            return True, "semantic_task_active"
        label_present = bool(str(detection.target_label or self._soft_last_label or "").strip())
        if self._pick_phase in {"planning", "executing"} and label_present:
            return True, "pick_phase_active"
        if (
            bool(detection.accepted)
            and bool(detection.candidate_visible)
            and bool(self._soft_locked)
            and label_present
        ):
            return True, "stable_visible_target"
        return False, "semantic_task_required"

    def _clear_soft_state(self) -> None:
        self._soft_last_point_px = None
        self._soft_last_bbox_xyxy = None
        self._soft_last_label = ""
        self._soft_stable_count = 0
        self._soft_locked = False

    def _clear_hard_state(self) -> None:
        self._stable_count = 0
        self._last_world_position = None

    def _hold_hard_lock_grace(self, reason: str, now_sec: float, candidate_visible: bool) -> bool:
        grace_sec = self.unlock_grace_sec
        miss_threshold = self.unlock_miss_count_threshold
        if self._execution_active:
            grace_sec = max(grace_sec, self.execution_unlock_grace_sec)
            miss_threshold = max(
                miss_threshold, self.execution_unlock_miss_count_threshold
            )
        if not self._last_lock_state or grace_sec <= 0.0:
            return False

        if self._unlock_pending_since <= 0.0:
            self._unlock_pending_since = now_sec
        self._unlock_miss_count += 1
        within_time = (now_sec - self._unlock_pending_since) <= grace_sec
        within_count = self._unlock_miss_count < miss_threshold
        if not (within_time and within_count):
            return False

        self.target_locked_pub.publish(Bool(data=True))
        self.candidate_visible_pub.publish(Bool(data=bool(candidate_visible)))
        msg = String()
        msg.data = compact_json(
            {
                "status": "grace",
                "reason": reason,
                "soft_locked": bool(self._soft_locked),
                "soft_stable_count": int(self._soft_stable_count),
                "locked": True,
                "stable_count": int(self._stable_count),
                "miss_count": int(self._unlock_miss_count),
                "grace_sec": float(grace_sec),
                "execution_active": bool(self._execution_active),
                "phase": self._pick_phase,
            }
        )
        self.debug_pub.publish(msg)
        self._last_reset_reason = reason
        return True

    def _publish_soft_state(self, reason: str, target_label: str) -> None:
        self._republish_last_visuals()
        self.target_locked_pub.publish(Bool(data=False))
        self.candidate_visible_pub.publish(Bool(data=True))
        msg = String()
        msg.data = compact_json(
            {
                "status": "soft_lock",
                "reason": reason,
                "target_label": target_label,
                "soft_locked": bool(self._soft_locked),
                "soft_stable_count": int(self._soft_stable_count),
                "locked": False,
                "stable_count": int(self._stable_count),
            }
        )
        self.debug_pub.publish(msg)
        summary_key = (target_label or "<none>", bool(self._soft_locked), False)
        if self._last_lock_state or str(summary_key) != self._last_summary:
            self.get_logger().info(
                "target tracking decision: "
                f"label={target_label or '<none>'} "
                f"soft_locked={self._soft_locked} "
                f"locked=False "
                f"reason={reason}"
            )
        self._last_summary = str(summary_key)
        self._last_lock_state = False
        self._last_reset_reason = reason

    def _publish_stable_cloud_reuse(
        self,
        *,
        detection: DetectionResult,
        reason: str,
        now_sec: float,
        current_points: int,
    ) -> bool:
        if (
            self._last_stable_target_cloud_points is None
            or self._last_stable_target_pose_position is None
            or self.stable_cloud_reuse_sec <= 0.0
        ):
            return False
        if (now_sec - float(self._last_stable_target_cloud_ts or 0.0)) > self.stable_cloud_reuse_sec:
            return False

        stable_points = np.asarray(self._last_stable_target_cloud_points, dtype=np.float32)
        stable_centroid = np.asarray(
            self._last_stable_target_pose_position, dtype=np.float32
        )
        stamp = self.get_clock().now().to_msg()

        self.target_cloud_pub.publish(make_xyz_cloud(stable_points, self.target_frame, stamp))
        self.target_cloud_marker_pub.publish(
            self._make_target_cloud_marker(stable_points, stamp)
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = float(stable_centroid[0])
        pose_msg.pose.position.y = float(stable_centroid[1])
        pose_msg.pose.position.z = float(stable_centroid[2])
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)

        if (
            self._last_stable_target_pose_camera_position is not None
            and self._last_stable_target_pose_camera_frame
        ):
            stable_centroid_camera = np.asarray(
                self._last_stable_target_pose_camera_position, dtype=np.float32
            )
            pose_camera_msg = PoseStamped()
            pose_camera_msg.header.stamp = stamp
            pose_camera_msg.header.frame_id = self._last_stable_target_pose_camera_frame
            pose_camera_msg.pose.position.x = float(stable_centroid_camera[0])
            pose_camera_msg.pose.position.y = float(stable_centroid_camera[1])
            pose_camera_msg.pose.position.z = float(stable_centroid_camera[2])
            pose_camera_msg.pose.orientation.w = 1.0
            self.target_pose_camera_pub.publish(pose_camera_msg)

        self._last_target_cloud_points = stable_points.copy()
        self._last_target_pose_position = stable_centroid.copy()
        if self._last_stable_target_pose_camera_position is not None:
            self._last_target_pose_camera_position = np.asarray(
                self._last_stable_target_pose_camera_position, dtype=np.float32
            ).copy()
        self._last_target_pose_camera_frame = self._last_stable_target_pose_camera_frame

        locked = bool(self._soft_locked and self._last_lock_state)
        self.target_locked_pub.publish(Bool(data=locked))
        self.candidate_visible_pub.publish(Bool(data=True))

        debug_msg = String()
        debug_msg.data = compact_json(
            {
                "status": "stable_cloud_reuse",
                "reason": reason,
                "target_label": str(detection.target_label or ""),
                "target_points": int(stable_points.shape[0]),
                "current_points": int(current_points),
                "locked": locked,
                "soft_locked": bool(self._soft_locked),
            }
        )
        self.debug_pub.publish(debug_msg)
        return True

    def _update_stable_cloud_cache(
        self,
        *,
        points_world: np.ndarray,
        centroid: np.ndarray,
        centroid_camera: np.ndarray,
        source_frame: str,
        now_sec: float,
    ) -> None:
        stable_points = np.asarray(points_world, dtype=np.float32)
        stable_centroid = np.asarray(centroid, dtype=np.float32)
        stable_centroid_camera = np.asarray(centroid_camera, dtype=np.float32)
        self._last_stable_target_cloud_points = stable_points.copy()
        self._last_stable_target_pose_position = stable_centroid.copy()
        self._last_stable_target_pose_camera_position = stable_centroid_camera.copy()
        self._last_stable_target_pose_camera_frame = str(source_frame)
        self._last_stable_target_cloud_ts = float(now_sec)

    def _extract_target_cloud(
        self,
        depth_image: np.ndarray,
        camera_info: CameraInfo,
        depth_msg: Image,
        detection: DetectionResult,
    ) -> Optional[dict[str, Any]]:
        image_h, image_w = depth_image.shape[:2]
        x1 = 0
        y1 = 0
        x2 = image_w
        y2 = image_h

        if detection.has_bbox:
            x1 = max(0, int(detection.bbox.x_offset) - self.bbox_margin_px)
            y1 = max(0, int(detection.bbox.y_offset) - self.bbox_margin_px)
            x2 = min(image_w, int(detection.bbox.x_offset + detection.bbox.width) + self.bbox_margin_px)
            y2 = min(image_h, int(detection.bbox.y_offset + detection.bbox.height) + self.bbox_margin_px)
            if x2 <= x1 or y2 <= y1:
                return None

        roi = depth_image[y1:y2, x1:x2]
        valid_depth_mask = np.isfinite(roi) & (roi >= self.min_depth_m) & (roi <= self.max_depth_m)

        candidate_mask = valid_depth_mask
        if detection.has_mask:
            mask_image = decode_mono8_image(detection.mask)
            if mask_image is not None and mask_image.shape[:2] == depth_image.shape[:2]:
                roi_mask = mask_image[y1:y2, x1:x2] > 0
                candidate_mask = valid_depth_mask & roi_mask
                if int(np.count_nonzero(candidate_mask)) < self.min_target_points:
                    candidate_mask = valid_depth_mask

        if int(np.count_nonzero(candidate_mask)) < self.min_target_points:
            return None

        anchor_selection = self._select_anchor_pixel(
            detection=detection,
            candidate_mask=candidate_mask,
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
        )
        if anchor_selection is None:
            return None
        point_u, point_v, roi_u, roi_v = anchor_selection

        valid_depths = roi[candidate_mask].astype(np.float64)
        if valid_depths.size == 0:
            return None

        anchor_depth = float(roi[roi_v, roi_u])
        if not np.isfinite(anchor_depth) or anchor_depth < self.min_depth_m or anchor_depth > self.max_depth_m:
            anchor_depth = float(np.percentile(valid_depths, self.depth_percentile_low))

        target_mask_mode = "candidate_mask"
        if bool(detection.has_mask):
            median_depth = float(np.median(valid_depths))
            mad = float(np.median(np.abs(valid_depths - median_depth))) if valid_depths.size > 0 else 0.0
            depth_window = max(self.depth_band_m, self.mask_depth_mad_scale * 1.4826 * mad)
            depth_window_mask = candidate_mask & (np.abs(roi - median_depth) <= depth_window)
            if int(np.count_nonzero(depth_window_mask)) >= self.min_target_points:
                target_mask = depth_window_mask
                target_mask_mode = "mask_median_band"
            else:
                target_mask = candidate_mask
                target_mask_mode = "mask_full"
            if not np.isfinite(anchor_depth):
                anchor_depth = median_depth
        else:
            depth_band_mask = candidate_mask & (np.abs(roi - anchor_depth) <= self.depth_band_m)
            seeded_component = self._extract_seeded_component(
                binary_mask=depth_band_mask,
                seed_u=roi_u,
                seed_v=roi_v,
            )
            if int(np.count_nonzero(seeded_component)) >= self.min_target_points:
                target_mask = seeded_component
                target_mask_mode = "seeded_depth_component"
            elif int(np.count_nonzero(depth_band_mask)) >= self.min_target_points:
                target_mask = depth_band_mask
                target_mask_mode = "seeded_depth_band"
            else:
                target_mask = candidate_mask
                target_mask_mode = "bbox_candidate_mask"

        pixels = np.argwhere(target_mask)
        if pixels.size == 0:
            return None

        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return None

        v = pixels[:, 0].astype(np.float64) + float(y1)
        u = pixels[:, 1].astype(np.float64) + float(x1)
        z = roi[target_mask].astype(np.float64)
        points_camera = np.column_stack(
            (
                (u - cx) * z / fx,
                (v - cy) * z / fy,
                z,
            )
        )

        source_frame = camera_info.header.frame_id or depth_msg.header.frame_id
        transform_data = self._lookup_depth_transform(source_frame, depth_msg.header.stamp)
        if transform_data is None:
            return None
        rotation, translation = transform_data
        anchor_point_camera = np.asarray(
            [
                (float(point_u) - cx) * anchor_depth / fx,
                (float(point_v) - cy) * anchor_depth / fy,
                anchor_depth,
            ],
            dtype=np.float64,
        )
        return {
            "points_world": (points_camera @ rotation.T + translation).astype(np.float32),
            "source_frame": str(source_frame),
            "anchor_depth": float(anchor_depth),
            "candidate_mask_pixels": int(np.count_nonzero(candidate_mask)),
            "target_mask_pixels": int(np.count_nonzero(target_mask)),
            "rotation": rotation.astype(np.float64, copy=True),
            "translation": translation.astype(np.float64, copy=True),
            "anchor_point_world": (anchor_point_camera @ rotation.T + translation).astype(np.float32),
            "target_mask_mode": target_mask_mode,
        }

    def _lookup_depth_transform(
        self,
        source_frame: str,
        stamp,
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(stamp),
            )
        except TransformException:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time(),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"cloud filter transform failed from {source_frame} to {self.target_frame}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return None

        quat = np.array(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],
            dtype=np.float64,
        )
        rotation = quaternion_to_rotation_matrix(quat)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        return rotation, translation

    def _extract_target_anchor_pose(
        self,
        depth_image: np.ndarray,
        camera_info: CameraInfo,
        depth_msg: Image,
        detection: DetectionResult,
    ) -> Optional[tuple[np.ndarray, np.ndarray, str, float, int, int]]:
        image_h, image_w = depth_image.shape[:2]
        x1 = 0
        y1 = 0
        x2 = image_w
        y2 = image_h

        if detection.has_bbox:
            x1 = max(0, int(detection.bbox.x_offset) - self.bbox_margin_px)
            y1 = max(0, int(detection.bbox.y_offset) - self.bbox_margin_px)
            x2 = min(image_w, int(detection.bbox.x_offset + detection.bbox.width) + self.bbox_margin_px)
            y2 = min(image_h, int(detection.bbox.y_offset + detection.bbox.height) + self.bbox_margin_px)
            if x2 <= x1 or y2 <= y1:
                return None

        roi = depth_image[y1:y2, x1:x2]
        valid_depth_mask = np.isfinite(roi) & (roi >= self.min_depth_m) & (roi <= self.max_depth_m)
        candidate_mask = valid_depth_mask
        if detection.has_mask:
            mask_image = decode_mono8_image(detection.mask)
            if mask_image is not None and mask_image.shape[:2] == depth_image.shape[:2]:
                roi_mask = mask_image[y1:y2, x1:x2] > 0
                masked_candidate = valid_depth_mask & roi_mask
                if int(np.count_nonzero(masked_candidate)) > 0:
                    candidate_mask = masked_candidate

        candidate_mask_pixels = int(np.count_nonzero(candidate_mask))
        if candidate_mask_pixels <= 0:
            return None

        anchor_selection = self._select_anchor_pixel(
            detection=detection,
            candidate_mask=candidate_mask,
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
        )
        if anchor_selection is None:
            return None
        point_u, point_v, roi_u, roi_v = anchor_selection

        anchor_depth = float(roi[roi_v, roi_u])
        if not np.isfinite(anchor_depth) or anchor_depth < self.min_depth_m or anchor_depth > self.max_depth_m:
            valid_depths = roi[candidate_mask]
            if valid_depths.size == 0:
                return None
            anchor_depth = float(np.percentile(valid_depths, self.depth_percentile_low))

        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return None

        source_frame = camera_info.header.frame_id or depth_msg.header.frame_id
        transform_data = self._lookup_depth_transform(source_frame, depth_msg.header.stamp)
        if transform_data is None:
            return None
        rotation, translation = transform_data

        point_camera = np.asarray(
            [
                (float(point_u) - cx) * anchor_depth / fx,
                (float(point_v) - cy) * anchor_depth / fy,
                anchor_depth,
            ],
            dtype=np.float64,
        )
        point_world = (point_camera @ rotation.T + translation).astype(np.float32)
        return (
            point_world,
            point_camera.astype(np.float32),
            str(source_frame),
            anchor_depth,
            candidate_mask_pixels,
            1,
        )

    def _publish_soft_anchor_pose(
        self,
        *,
        detection: DetectionResult,
        anchor_pose: tuple[np.ndarray, np.ndarray, str, float, int, int],
        reason: str,
    ) -> None:
        (
            point_world,
            point_camera,
            source_frame,
            anchor_depth,
            candidate_mask_pixels,
            target_mask_pixels,
        ) = anchor_pose
        stamp = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = float(point_world[0])
        pose_msg.pose.position.y = float(point_world[1])
        pose_msg.pose.position.z = float(point_world[2])
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)

        pose_camera_msg = PoseStamped()
        pose_camera_msg.header.stamp = stamp
        pose_camera_msg.header.frame_id = source_frame
        pose_camera_msg.pose.position.x = float(point_camera[0])
        pose_camera_msg.pose.position.y = float(point_camera[1])
        pose_camera_msg.pose.position.z = float(point_camera[2])
        pose_camera_msg.pose.orientation.w = 1.0
        self.target_pose_camera_pub.publish(pose_camera_msg)

        self._last_target_pose_position = np.asarray(point_world, dtype=np.float32).copy()
        self._last_target_pose_camera_position = np.asarray(point_camera, dtype=np.float32).copy()
        self._last_target_pose_camera_frame = str(source_frame)

        self.target_locked_pub.publish(Bool(data=False))
        self.candidate_visible_pub.publish(Bool(data=True))
        point_px = None
        if bool(detection.has_point) and len(detection.point_px) >= 2:
            point_px = [int(detection.point_px[0]), int(detection.point_px[1])]
        bbox_xyxy = None
        if bool(detection.has_bbox):
            bbox_xyxy = [
                int(detection.bbox.x_offset),
                int(detection.bbox.y_offset),
                int(detection.bbox.x_offset + detection.bbox.width),
                int(detection.bbox.y_offset + detection.bbox.height),
            ]
        msg = String()
        msg.data = compact_json(
            {
                "status": "soft_pose",
                "reason": reason,
                "target_label": str(detection.target_label or ""),
                "target_points": 1,
                "point_px": point_px,
                "bbox_xyxy": bbox_xyxy,
                "source_frame": str(source_frame),
                "anchor_depth_m": round(float(anchor_depth), 4),
                "candidate_mask_pixels": int(candidate_mask_pixels),
                "target_mask_pixels": int(target_mask_pixels),
                "centroid_xyz_camera": [
                    round(float(point_camera[0]), 4),
                    round(float(point_camera[1]), 4),
                    round(float(point_camera[2]), 4),
                ],
                "centroid_xyz": [
                    round(float(point_world[0]), 4),
                    round(float(point_world[1]), 4),
                    round(float(point_world[2]), 4),
                ],
                "soft_locked": bool(self._soft_locked),
                "soft_stable_count": int(self._soft_stable_count),
                "locked": False,
                "stable_count": int(self._stable_count),
                "semantic_task_active": bool(self._semantic_task_active),
                "semantic_gate_allows_lock": False,
                "semantic_gate_reason": reason,
                "pick_phase": self._pick_phase,
                "mask_used": bool(detection.has_mask),
                "open3d_filtered": False,
            }
        )
        self.debug_pub.publish(msg)
        summary_key = (str(detection.target_label or "<none>"), bool(self._soft_locked), False, "soft_pose")
        if str(summary_key) != self._last_summary:
            self.get_logger().info(
                "target tracking decision: "
                f"label={str(detection.target_label or '<none>')} "
                f"soft_locked={self._soft_locked} "
                f"locked=False "
                f"reason={reason} "
                f"anchor_pose=({float(point_world[0]):.3f}, {float(point_world[1]):.3f}, {float(point_world[2]):.3f})"
            )
        self._last_summary = str(summary_key)
        self._last_lock_state = False
        self._last_reset_reason = reason

    def _publish_soft_scene_pose(
        self,
        *,
        detection: DetectionResult,
        reason: str,
    ) -> bool:
        if not self.use_scene_target_pose_fallback or self._scene_target_pose_position is None:
            return False
        point_world = np.asarray(self._scene_target_pose_position, dtype=np.float32)
        stamp = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = float(point_world[0])
        pose_msg.pose.position.y = float(point_world[1])
        pose_msg.pose.position.z = float(point_world[2])
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)
        self._last_target_pose_position = point_world.copy()
        self.target_locked_pub.publish(Bool(data=False))
        self.candidate_visible_pub.publish(Bool(data=True))
        msg = String()
        msg.data = compact_json(
            {
                "status": "soft_pose_scene",
                "reason": reason,
                "target_label": str(detection.target_label or ""),
                "centroid_xyz": [
                    round(float(point_world[0]), 4),
                    round(float(point_world[1]), 4),
                    round(float(point_world[2]), 4),
                ],
                "soft_locked": bool(self._soft_locked),
                "locked": False,
                "stable_count": int(self._stable_count),
            }
        )
        self.debug_pub.publish(msg)
        summary_key = (
            str(detection.target_label or "<none>"),
            bool(self._soft_locked),
            False,
            "soft_pose_scene",
        )
        if str(summary_key) != self._last_summary:
            self.get_logger().info(
                "target tracking decision: "
                f"label={str(detection.target_label or '<none>')} "
                f"soft_locked={self._soft_locked} "
                f"locked=False "
                f"reason={reason} "
                f"scene_pose=({float(point_world[0]):.3f}, {float(point_world[1]):.3f}, {float(point_world[2]):.3f})"
            )
        self._last_summary = str(summary_key)
        self._last_lock_state = False
        self._last_reset_reason = reason
        return True

    def _reset_state(self, reason: str) -> None:
        self._clear_soft_state()
        self._clear_hard_state()
        self._unlock_pending_since = 0.0
        self._unlock_miss_count = 0
        self.target_locked_pub.publish(Bool(data=False))
        self.candidate_visible_pub.publish(Bool(data=False))
        msg = String()
        msg.data = compact_json({"status": "reset", "reason": reason})
        self.debug_pub.publish(msg)
        if reason != self._last_reset_reason or self._last_lock_state:
            self.get_logger().info(f"target tracking decision: reset reason={reason}")
        self._last_lock_state = False
        self._last_reset_reason = reason

    def _republish_last_visuals(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if self._last_target_cloud_points is not None:
            self.target_cloud_pub.publish(
                make_xyz_cloud(self._last_target_cloud_points, self.target_frame, stamp)
            )
            self.target_cloud_marker_pub.publish(
                self._make_target_cloud_marker(self._last_target_cloud_points, stamp)
            )
        if self._last_target_pose_position is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self.target_frame
            pose_msg.pose.position.x = float(self._last_target_pose_position[0])
            pose_msg.pose.position.y = float(self._last_target_pose_position[1])
            pose_msg.pose.position.z = float(self._last_target_pose_position[2])
            pose_msg.pose.orientation.w = 1.0
            self.target_pose_pub.publish(pose_msg)
        if (
            self._last_target_pose_camera_position is not None
            and self._last_target_pose_camera_frame
        ):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self._last_target_pose_camera_frame
            pose_msg.pose.position.x = float(self._last_target_pose_camera_position[0])
            pose_msg.pose.position.y = float(self._last_target_pose_camera_position[1])
            pose_msg.pose.position.z = float(self._last_target_pose_camera_position[2])
            pose_msg.pose.orientation.w = 1.0
            self.target_pose_camera_pub.publish(pose_msg)

    def _make_target_cloud_marker(self, points_world: np.ndarray, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = stamp
        marker.ns = "target_cloud"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.006
        marker.scale.y = 0.006
        marker.scale.z = 0.006
        marker.color.r = 0.16
        marker.color.g = 0.82
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.points = []
        for point in np.asarray(points_world, dtype=np.float32).reshape((-1, 3)):
            ros_point = Point()
            ros_point.x = float(point[0])
            ros_point.y = float(point[1])
            ros_point.z = float(point[2])
            marker.points.append(ros_point)
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudFilterNode()
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
