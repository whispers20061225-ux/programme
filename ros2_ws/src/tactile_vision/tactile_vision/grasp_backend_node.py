from __future__ import annotations

import copy
import importlib
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import requests
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, ColorRGBA, String
from tactile_interfaces.msg import DetectionResult, GraspProposal, GraspProposalArray, SemanticTask
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from tactile_vision.modular_common import (
    compact_json,
    decode_depth_image,
    decode_mono8_image,
    encode_depth_m_to_base64_png,
    encode_png_base64,
    make_rgb8_image,
    make_xyz_cloud,
    quaternion_to_rotation_matrix,
)


def point_cloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    if int(msg.point_step) < 12 or int(msg.width) <= 0:
        return np.empty((0, 3), dtype=np.float32)
    point_step_floats = max(3, int(msg.point_step) // 4)
    points = np.frombuffer(msg.data, dtype=np.float32)
    if points.size < int(msg.width) * point_step_floats:
        return np.empty((0, 3), dtype=np.float32)
    points = points.reshape((-1, point_step_floats))
    return points[:, :3].astype(np.float32)


def to_point(values: Any) -> Point:
    point = Point()
    if isinstance(values, (list, tuple)) and len(values) >= 3:
        point.x = float(values[0])
        point.y = float(values[1])
        point.z = float(values[2])
    return point


def to_vector3(values: Any) -> Vector3:
    vector = Vector3()
    if isinstance(values, (list, tuple)) and len(values) >= 3:
        vector.x = float(values[0])
        vector.y = float(values[1])
        vector.z = float(values[2])
    return vector


def point_to_numpy(point: Point) -> np.ndarray:
    return np.array([float(point.x), float(point.y), float(point.z)], dtype=np.float32)


def vector3_to_numpy(vector: Vector3) -> np.ndarray:
    return np.array([float(vector.x), float(vector.y), float(vector.z)], dtype=np.float32)


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float32).reshape((3,))
    length = float(np.linalg.norm(values))
    if length <= 1e-6:
        return np.zeros((3,), dtype=np.float32)
    return values / length


def make_color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    color = ColorRGBA()
    color.r = float(r)
    color.g = float(g)
    color.b = float(b)
    color.a = float(a)
    return color


def make_point_xyz(values: np.ndarray) -> Point:
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2])
    return point


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    quaternion /= norm
    return quaternion.astype(float).tolist()


class GraspBackendNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_backend_node")

        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("target_cloud_topic", "/perception/target_cloud")
        self.declare_parameter("detection_result_topic", "/perception/detection_result")
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info"
        )
        self.declare_parameter(
            "candidate_grasp_proposal_array_topic", "/grasp/candidate_grasp_proposals"
        )
        self.declare_parameter("candidate_markers_topic", "/grasp/candidate_markers")
        self.declare_parameter(
            "selected_contact_point_1_topic", "/grasp/selected_contact_point_1_cloud"
        )
        self.declare_parameter(
            "selected_contact_point_2_topic", "/grasp/selected_contact_point_2_cloud"
        )
        self.declare_parameter(
            "selected_grasp_center_topic", "/grasp/selected_grasp_center_cloud"
        )
        self.declare_parameter("ggcnn_depth_roi_topic", "/grasp/ggcnn/depth_roi")
        self.declare_parameter("ggcnn_q_heatmap_topic", "/grasp/ggcnn/q_heatmap")
        self.declare_parameter("ggcnn_angle_map_topic", "/grasp/ggcnn/angle_map")
        self.declare_parameter("ggcnn_width_map_topic", "/grasp/ggcnn/width_map")
        self.declare_parameter("ggcnn_grasp_overlay_topic", "/grasp/ggcnn/grasp_overlay")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("backend", "disabled")
        self.declare_parameter("backend_url", "")
        self.declare_parameter("request_timeout_sec", 300.0)
        self.declare_parameter("max_inference_rate_hz", 1.0)
        self.declare_parameter("sensor_stale_sec", 1.5)
        self.declare_parameter("max_points", 2048)
        self.declare_parameter("min_points_required", 64)
        self.declare_parameter("publish_empty_on_failure", False)
        self.declare_parameter("default_task_constraint_tag", "pick")
        self.declare_parameter("contact_graspnet_segmap_id", 1)
        self.declare_parameter("contact_graspnet_local_regions", True)
        self.declare_parameter("contact_graspnet_filter_grasps", True)
        self.declare_parameter("contact_graspnet_skip_border_objects", False)
        self.declare_parameter("contact_graspnet_forward_passes", 1)
        self.declare_parameter("contact_graspnet_z_range", [0.2, 1.8])
        self.declare_parameter("contact_graspnet_depth_window_m", 0.12)
        self.declare_parameter("contact_graspnet_roi_margin_px", 24)
        self.declare_parameter("contact_graspnet_pregrasp_offset_m", 0.06)
        self.declare_parameter("contact_graspnet_max_proposals", 0)
        self.declare_parameter("contact_graspnet_min_mask_pixels", 96)
        self.declare_parameter("contact_graspnet_visualize", False)
        self.declare_parameter("request_signature_center_bin_px", 24)
        self.declare_parameter("request_signature_size_bin_px", 24)
        self.declare_parameter("request_signature_depth_bin_m", 0.02)
        self.declare_parameter("ggcnn_repo_root", "/home/whispers/ggcnn")
        self.declare_parameter(
            "ggcnn_checkpoint_path",
            "/home/whispers/ggcnn_weights/ggcnn2_weights_cornell/epoch_50_cornell_statedict.pt",
        )
        self.declare_parameter("ggcnn_model", "ggcnn2")
        self.declare_parameter("ggcnn_device", "")
        self.declare_parameter("ggcnn_input_size", 300)
        self.declare_parameter("ggcnn_top_k", 10)
        self.declare_parameter("ggcnn_quality_threshold", 0.20)
        self.declare_parameter("ggcnn_nms_radius_px", 20)
        self.declare_parameter("ggcnn_bbox_margin_px", 20)
        self.declare_parameter("ggcnn_min_mask_pixels", 96)
        self.declare_parameter("ggcnn_pregrasp_offset_m", 0.06)
        self.declare_parameter("visual_republish_rate_hz", 8.0)
        self.declare_parameter("log_interval_sec", 8.0)

        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.target_cloud_topic = str(self.get_parameter("target_cloud_topic").value)
        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.candidate_grasp_proposal_array_topic = str(
            self.get_parameter("candidate_grasp_proposal_array_topic").value
        )
        self.candidate_markers_topic = str(self.get_parameter("candidate_markers_topic").value)
        self.selected_contact_point_1_topic = str(
            self.get_parameter("selected_contact_point_1_topic").value
        )
        self.selected_contact_point_2_topic = str(
            self.get_parameter("selected_contact_point_2_topic").value
        )
        self.selected_grasp_center_topic = str(
            self.get_parameter("selected_grasp_center_topic").value
        )
        self.ggcnn_depth_roi_topic = str(self.get_parameter("ggcnn_depth_roi_topic").value)
        self.ggcnn_q_heatmap_topic = str(self.get_parameter("ggcnn_q_heatmap_topic").value)
        self.ggcnn_angle_map_topic = str(self.get_parameter("ggcnn_angle_map_topic").value)
        self.ggcnn_width_map_topic = str(self.get_parameter("ggcnn_width_map_topic").value)
        self.ggcnn_grasp_overlay_topic = str(
            self.get_parameter("ggcnn_grasp_overlay_topic").value
        )
        self.target_frame = str(self.get_parameter("target_frame").value).strip() or "world"
        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.backend_url = str(self.get_parameter("backend_url").value).strip()
        self.request_timeout_sec = max(
            0.5, float(self.get_parameter("request_timeout_sec").value)
        )
        self.max_inference_rate_hz = max(
            0.2, float(self.get_parameter("max_inference_rate_hz").value)
        )
        self.sensor_stale_sec = max(0.0, float(self.get_parameter("sensor_stale_sec").value))
        self.max_points = max(64, int(self.get_parameter("max_points").value))
        self.min_points_required = max(
            8, int(self.get_parameter("min_points_required").value)
        )
        self.publish_empty_on_failure = bool(
            self.get_parameter("publish_empty_on_failure").value
        )
        self.default_task_constraint_tag = str(
            self.get_parameter("default_task_constraint_tag").value
        ).strip()
        self.contact_graspnet_segmap_id = max(
            1, int(self.get_parameter("contact_graspnet_segmap_id").value)
        )
        self.contact_graspnet_local_regions = bool(
            self.get_parameter("contact_graspnet_local_regions").value
        )
        self.contact_graspnet_filter_grasps = bool(
            self.get_parameter("contact_graspnet_filter_grasps").value
        )
        self.contact_graspnet_skip_border_objects = bool(
            self.get_parameter("contact_graspnet_skip_border_objects").value
        )
        self.contact_graspnet_forward_passes = max(
            1, int(self.get_parameter("contact_graspnet_forward_passes").value)
        )
        z_range_values = list(self.get_parameter("contact_graspnet_z_range").value)
        if len(z_range_values) >= 2:
            z0 = float(z_range_values[0])
            z1 = float(z_range_values[1])
        else:
            z0, z1 = 0.2, 1.8
        self.contact_graspnet_z_range = [min(z0, z1), max(z0, z1)]
        self.contact_graspnet_depth_window_m = max(
            0.02, float(self.get_parameter("contact_graspnet_depth_window_m").value)
        )
        self.contact_graspnet_roi_margin_px = max(
            0, int(self.get_parameter("contact_graspnet_roi_margin_px").value)
        )
        self.contact_graspnet_pregrasp_offset_m = max(
            0.01, float(self.get_parameter("contact_graspnet_pregrasp_offset_m").value)
        )
        self.contact_graspnet_max_proposals = max(
            0, int(self.get_parameter("contact_graspnet_max_proposals").value)
        )
        self.contact_graspnet_min_mask_pixels = max(
            16, int(self.get_parameter("contact_graspnet_min_mask_pixels").value)
        )
        self.contact_graspnet_visualize = bool(
            self.get_parameter("contact_graspnet_visualize").value
        )
        self.request_signature_center_bin_px = max(
            1, int(self.get_parameter("request_signature_center_bin_px").value)
        )
        self.request_signature_size_bin_px = max(
            1, int(self.get_parameter("request_signature_size_bin_px").value)
        )
        self.request_signature_depth_bin_m = max(
            0.001, float(self.get_parameter("request_signature_depth_bin_m").value)
        )
        self.ggcnn_repo_root = Path(
            str(self.get_parameter("ggcnn_repo_root").value).strip()
        ).expanduser()
        self.ggcnn_checkpoint_path = Path(
            str(self.get_parameter("ggcnn_checkpoint_path").value).strip()
        ).expanduser()
        self.ggcnn_model_name = str(self.get_parameter("ggcnn_model").value).strip().lower()
        self.ggcnn_device = str(self.get_parameter("ggcnn_device").value).strip()
        self.ggcnn_input_size = max(64, int(self.get_parameter("ggcnn_input_size").value))
        self.ggcnn_top_k = max(1, int(self.get_parameter("ggcnn_top_k").value))
        self.ggcnn_quality_threshold = max(
            0.0, min(1.0, float(self.get_parameter("ggcnn_quality_threshold").value))
        )
        self.ggcnn_nms_radius_px = max(1, int(self.get_parameter("ggcnn_nms_radius_px").value))
        self.ggcnn_bbox_margin_px = max(0, int(self.get_parameter("ggcnn_bbox_margin_px").value))
        self.ggcnn_min_mask_pixels = max(
            16, int(self.get_parameter("ggcnn_min_mask_pixels").value)
        )
        self.ggcnn_pregrasp_offset_m = max(
            0.01, float(self.get_parameter("ggcnn_pregrasp_offset_m").value)
        )
        self.visual_republish_rate_hz = max(
            0.2, float(self.get_parameter("visual_republish_rate_hz").value)
        )
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._session = requests.Session()
        self._semantic_lock = threading.Lock()
        self._sensor_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._semantic_task: Optional[SemanticTask] = None
        self._latest_target_cloud: Optional[PointCloud2] = None
        self._latest_target_cloud_ts = 0.0
        self._latest_detection: Optional[DetectionResult] = None
        self._latest_detection_ts = 0.0
        self._latest_depth_msg: Optional[Image] = None
        self._latest_depth_ts = 0.0
        self._latest_camera_info: Optional[CameraInfo] = None
        self._latest_camera_info_ts = 0.0
        self._target_locked = False
        self._pending_request = False
        self._request_in_flight = False
        self._last_dispatched_target_signature = ""
        self._last_log_ts = 0.0
        self._last_terminal_summary = ""
        self._last_precheck_summary = ""
        self._last_precheck_log_ts = 0.0
        self._last_visual_marker_ids: set[int] = set()
        self._last_visual_frame_id = "world"
        self._last_visual_proposal_array: Optional[GraspProposalArray] = None
        self._last_ggcnn_depth_roi_msg: Optional[Image] = None
        self._last_ggcnn_q_heatmap_msg: Optional[Image] = None
        self._last_ggcnn_angle_map_msg: Optional[Image] = None
        self._last_ggcnn_width_map_msg: Optional[Image] = None
        self._last_ggcnn_grasp_overlay_msg: Optional[Image] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._ggcnn_lock = threading.Lock()
        self._ggcnn_model: Any = None
        self._ggcnn_post_process_output = None
        self._ggcnn_device_resolved = ""

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
            SemanticTask, self.semantic_task_topic, self._on_semantic_task, qos_reliable
        )
        self.create_subscription(
            PointCloud2, self.target_cloud_topic, self._on_target_cloud, qos_reliable
        )
        self.create_subscription(
            DetectionResult, self.detection_result_topic, self._on_detection_result, qos_reliable
        )
        self.create_subscription(Bool, self.target_locked_topic, self._on_target_locked, qos_reliable)
        self.create_subscription(Image, self.depth_topic, self._on_depth_image, qos_sensor)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, qos_sensor
        )
        self.proposal_pub = self.create_publisher(
            GraspProposalArray, self.candidate_grasp_proposal_array_topic, qos_latched
        )
        self.candidate_marker_pub = self.create_publisher(
            MarkerArray, self.candidate_markers_topic, qos_latched
        )
        self.selected_contact_point_1_pub = self.create_publisher(
            PointCloud2, self.selected_contact_point_1_topic, qos_latched
        )
        self.selected_contact_point_2_pub = self.create_publisher(
            PointCloud2, self.selected_contact_point_2_topic, qos_latched
        )
        self.selected_grasp_center_pub = self.create_publisher(
            PointCloud2, self.selected_grasp_center_topic, qos_latched
        )
        self.ggcnn_depth_roi_pub = self.create_publisher(Image, self.ggcnn_depth_roi_topic, qos_latched)
        self.ggcnn_q_heatmap_pub = self.create_publisher(Image, self.ggcnn_q_heatmap_topic, qos_latched)
        self.ggcnn_angle_map_pub = self.create_publisher(Image, self.ggcnn_angle_map_topic, qos_latched)
        self.ggcnn_width_map_pub = self.create_publisher(Image, self.ggcnn_width_map_topic, qos_latched)
        self.ggcnn_grasp_overlay_pub = self.create_publisher(
            Image, self.ggcnn_grasp_overlay_topic, qos_latched
        )
        self.debug_pub = self.create_publisher(String, "/grasp/backend_debug", qos_reliable)
        self.create_timer(1.0 / self.max_inference_rate_hz, self._maybe_run_inference)
        self.create_timer(1.0 / self.visual_republish_rate_hz, self._republish_last_visuals)

        self.get_logger().info(
            "grasp_backend_node started: "
            f"backend={self.backend}, target_cloud={self.target_cloud_topic}, "
            f"detection={self.detection_result_topic}, proposal_topic={self.candidate_grasp_proposal_array_topic}, "
            f"markers={self.candidate_markers_topic}, target_frame={self.target_frame}"
        )

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        with self._semantic_lock:
            self._semantic_task = msg
        if self._target_locked:
            self._pending_request = True

    def _on_target_cloud(self, msg: PointCloud2) -> None:
        with self._sensor_lock:
            self._latest_target_cloud = msg
            self._latest_target_cloud_ts = time.time()
        if self._target_locked:
            self._pending_request = True

    def _on_detection_result(self, msg: DetectionResult) -> None:
        with self._sensor_lock:
            self._latest_detection = msg
            self._latest_detection_ts = time.time()
        if self._target_locked:
            self._pending_request = True

    def _on_depth_image(self, msg: Image) -> None:
        with self._sensor_lock:
            self._latest_depth_msg = msg
            self._latest_depth_ts = time.time()
        if self._target_locked:
            self._pending_request = True

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._sensor_lock:
            self._latest_camera_info = msg
            self._latest_camera_info_ts = time.time()
        if self._target_locked:
            self._pending_request = True

    def _on_target_locked(self, msg: Bool) -> None:
        self._target_locked = bool(msg.data)
        self._pending_request = self._target_locked

    def _maybe_run_inference(self) -> None:
        if not self._pending_request:
            return

        if self.backend == "disabled":
            self._pending_request = False
            self._publish_debug(
                {
                    "status": "backend_disabled",
                    "reason": "configure Contact-GraspNet service endpoint",
                }
            )
            return

        if self.backend in ("contact_graspnet_http", "ggcnn_local"):
            if not self._target_locked:
                self._pending_request = False
                return
            if not self._detection_depth_camera_inputs_ready():
                self._log_precheck_state(
                    f"{self.backend} waiting for detection/depth/camera info"
                )
                return
            try:
                if self.backend == "contact_graspnet_http":
                    signature = self._get_contact_graspnet_target_signature()
                else:
                    signature = self._get_ggcnn_target_signature()
            except Exception as exc:  # noqa: BLE001
                self._log_precheck_state(f"{self.backend} precheck failed: {str(exc).strip()}")
                return
            if signature == self._last_dispatched_target_signature:
                self._pending_request = False
                return

        with self._request_lock:
            if self._request_in_flight:
                return
            self._request_in_flight = True
            self._pending_request = False
            if self.backend in ("contact_graspnet_http", "ggcnn_local"):
                self._last_dispatched_target_signature = signature

        worker = threading.Thread(target=self._run_backend, daemon=True)
        worker.start()

    def _run_backend(self) -> None:
        try:
            with self._semantic_lock:
                semantic_task = self._semantic_task
            with self._sensor_lock:
                target_cloud = self._latest_target_cloud
                target_cloud_ts = self._latest_target_cloud_ts
                detection = self._latest_detection
                detection_ts = self._latest_detection_ts
                depth_msg = self._latest_depth_msg
                depth_ts = self._latest_depth_ts
                camera_info = self._latest_camera_info
                camera_info_ts = self._latest_camera_info_ts

            if self.backend == "http_json":
                parsed = self._run_point_cloud_http_backend(
                    semantic_task=semantic_task,
                    cloud_msg=target_cloud,
                    cloud_ts=target_cloud_ts,
                )
            elif self.backend == "contact_graspnet_http":
                parsed = self._run_contact_graspnet_http_backend(
                    semantic_task=semantic_task,
                    detection=detection,
                    detection_ts=detection_ts,
                    depth_msg=depth_msg,
                    depth_ts=depth_ts,
                    camera_info=camera_info,
                    camera_info_ts=camera_info_ts,
                )
            elif self.backend == "ggcnn_local":
                parsed = self._run_ggcnn_local_backend(
                    semantic_task=semantic_task,
                    detection=detection,
                    detection_ts=detection_ts,
                    depth_msg=depth_msg,
                    depth_ts=depth_ts,
                    camera_info=camera_info,
                    camera_info_ts=camera_info_ts,
                )
            else:
                raise ValueError(f"unsupported grasp backend: {self.backend}")

            proposal_array = self._parse_proposal_array(parsed)
            self.proposal_pub.publish(proposal_array)
            self._publish_candidate_visuals(proposal_array)
            self._publish_debug(
                {
                    "status": "ok",
                    "backend": self.backend,
                    "proposal_count": len(proposal_array.proposals),
                    "frame_id": proposal_array.header.frame_id,
                    "selected_index": int(proposal_array.selected_index),
                    "target_label": (
                        semantic_task.target_label if semantic_task is not None else ""
                    ),
                }
            )
        except Exception as exc:  # noqa: BLE001
            message = str(exc).strip()
            self.get_logger().warn(f"grasp backend inference failed: {message}")
            self._publish_debug({"status": "error", "backend": self.backend, "reason": str(exc)})
            if self.publish_empty_on_failure:
                empty = GraspProposalArray()
                self.proposal_pub.publish(empty)
        finally:
            with self._request_lock:
                self._request_in_flight = False

    def _detection_depth_camera_inputs_ready(self) -> bool:
        with self._sensor_lock:
            return (
                self._latest_detection is not None
                and self._latest_depth_msg is not None
                and self._latest_camera_info is not None
            )

    def _log_precheck_state(self, summary: str) -> None:
        now = time.time()
        if (
            summary == self._last_precheck_summary and
            (now - self._last_precheck_log_ts) < self.log_interval_sec
        ):
            return
        self._last_precheck_summary = summary
        self._last_precheck_log_ts = now
        self.get_logger().info(summary)

    def _get_contact_graspnet_target_signature(self) -> str:
        with self._sensor_lock:
            detection = self._latest_detection
            detection_ts = self._latest_detection_ts
            depth_msg = self._latest_depth_msg
            depth_ts = self._latest_depth_ts
            camera_info = self._latest_camera_info
            camera_info_ts = self._latest_camera_info_ts
        context = self._extract_contact_graspnet_context(
            detection=detection,
            detection_ts=detection_ts,
            depth_msg=depth_msg,
            depth_ts=depth_ts,
            camera_info=camera_info,
            camera_info_ts=camera_info_ts,
        )
        return str(context["target_signature"])

    def _get_ggcnn_target_signature(self) -> str:
        with self._sensor_lock:
            detection = self._latest_detection
            detection_ts = self._latest_detection_ts
            depth_msg = self._latest_depth_msg
            depth_ts = self._latest_depth_ts
            camera_info = self._latest_camera_info
            camera_info_ts = self._latest_camera_info_ts
        context = self._extract_ggcnn_context(
            detection=detection,
            detection_ts=detection_ts,
            depth_msg=depth_msg,
            depth_ts=depth_ts,
            camera_info=camera_info,
            camera_info_ts=camera_info_ts,
        )
        return str(context["target_signature"])

    def _run_ggcnn_local_backend(
        self,
        *,
        semantic_task: Optional[SemanticTask],
        detection: Optional[DetectionResult],
        detection_ts: float,
        depth_msg: Optional[Image],
        depth_ts: float,
        camera_info: Optional[CameraInfo],
        camera_info_ts: float,
    ) -> dict[str, Any]:
        context = self._extract_ggcnn_context(
            detection=detection,
            detection_ts=detection_ts,
            depth_msg=depth_msg,
            depth_ts=depth_ts,
            camera_info=camera_info,
            camera_info_ts=camera_info_ts,
        )
        model, post_process_output, device, torch_mod = self._ensure_ggcnn_model_loaded()
        input_tensor = (
            torch_mod.from_numpy(context["network_input"])
            .unsqueeze(0)
            .unsqueeze(0)
            .to(device=device, dtype=torch_mod.float32)
        )

        start_time = time.perf_counter()
        with torch_mod.no_grad():
            pos, cos, sin, width = model(input_tensor)
            if device.startswith("cuda"):
                torch_mod.cuda.synchronize()
        inference_sec = time.perf_counter() - start_time
        q_img, ang_img, width_img = post_process_output(pos, cos, sin, width)

        candidates = self._extract_ggcnn_candidates(
            q_img=q_img,
            ang_img=ang_img,
            width_img=width_img,
            valid_mask=context["resized_mask"],
        )
        if not candidates:
            raise ValueError("GG-CNN produced no grasp candidates above threshold")

        proposals: list[dict[str, Any]] = []
        for rank, candidate in enumerate(candidates, start=1):
            proposal = self._build_ggcnn_proposal_item(
                context=context,
                candidate=candidate,
                candidate_rank=rank,
            )
            if proposal is not None:
                proposals.append(proposal)

        if not proposals:
            raise ValueError("GG-CNN candidates could not be converted into 3D proposals")

        selected_index = int(
            max(
                range(len(proposals)),
                key=lambda idx: float(proposals[idx].get("confidence_score", 0.0)),
            )
        )
        self._publish_ggcnn_shadow_images(
            context=context,
            q_img=q_img,
            ang_img=ang_img,
            width_img=width_img,
            candidates=candidates,
            selected_index=selected_index,
        )
        selected_score = float(proposals[selected_index].get("confidence_score", 0.0))
        target_label = (
            str(detection.target_label or "")
            if detection is not None
            else (semantic_task.target_label if semantic_task is not None else "")
        ).strip() or "unknown"
        self.get_logger().info(
            "grasp backend decision: "
            f"backend=ggcnn_local target={target_label} "
            f"roi={int(context['roi_xyxy'][2] - context['roi_xyxy'][0])}x{int(context['roi_xyxy'][3] - context['roi_xyxy'][1])} "
            f"mask_pixels={int(context['roi_mask_pixels'])} "
            f"valid_depth={int(context['roi_target_valid_depth_pixels'])} "
            f"proposals={len(proposals)} "
            f"top_score={selected_score:.3f} "
            f"inference_sec={inference_sec:.4f}"
        )
        return {
            "frame_id": self.target_frame,
            "selected_index": selected_index,
            "proposals": proposals,
            "debug": {
                "backend": "ggcnn_local",
                "inference_sec": inference_sec,
                "proposal_count": len(proposals),
                "roi_xyxy": list(context["roi_xyxy"]),
                "target_signature": str(context["target_signature"]),
            },
        }

    def _ensure_ggcnn_model_loaded(self):
        with self._ggcnn_lock:
            if (
                self._ggcnn_model is not None and
                self._ggcnn_post_process_output is not None and
                self._ggcnn_device_resolved
            ):
                torch_mod = importlib.import_module("torch")
                return (
                    self._ggcnn_model,
                    self._ggcnn_post_process_output,
                    self._ggcnn_device_resolved,
                    torch_mod,
                )

            repo_root = self.ggcnn_repo_root.resolve()
            checkpoint_path = self.ggcnn_checkpoint_path.resolve()
            if not repo_root.is_dir():
                raise ValueError(f"GG-CNN repo root not found: {repo_root}")
            if not checkpoint_path.is_file():
                raise ValueError(f"GG-CNN checkpoint not found: {checkpoint_path}")

            repo_root_str = str(repo_root)
            if repo_root_str not in sys.path:
                sys.path.insert(0, repo_root_str)

            torch_mod = importlib.import_module("torch")
            if self.ggcnn_model_name == "ggcnn2":
                model_module = importlib.import_module("models.ggcnn2")
                model_type = getattr(model_module, "GGCNN2")
            else:
                model_module = importlib.import_module("models.ggcnn")
                model_type = getattr(model_module, "GGCNN")

            requested_device = self.ggcnn_device or (
                "cuda:0" if bool(torch_mod.cuda.is_available()) else "cpu"
            )
            if requested_device.startswith("cuda") and not bool(torch_mod.cuda.is_available()):
                requested_device = "cpu"

            model = model_type()
            state = torch_mod.load(checkpoint_path, map_location=requested_device)
            if isinstance(state, dict):
                try:
                    model.load_state_dict(state)
                except Exception:
                    state_dict = state.get("model_state_dict")
                    if state_dict is None:
                        raise
                    model.load_state_dict(state_dict)
            else:
                model = state

            model = model.to(requested_device)
            model.eval()

            self._ggcnn_model = model
            self._ggcnn_post_process_output = self._post_process_ggcnn_output
            self._ggcnn_device_resolved = requested_device
            self.get_logger().info(
                "grasp backend GG-CNN ready: "
                f"model={self.ggcnn_model_name} "
                f"device={requested_device} "
                f"checkpoint={checkpoint_path}"
            )
            return model, self._ggcnn_post_process_output, requested_device, torch_mod

    def _extract_ggcnn_context(
        self,
        *,
        detection: Optional[DetectionResult],
        detection_ts: float,
        depth_msg: Optional[Image],
        depth_ts: float,
        camera_info: Optional[CameraInfo],
        camera_info_ts: float,
    ) -> dict[str, Any]:
        if detection is None or depth_msg is None or camera_info is None:
            raise ValueError("missing detection/depth/camera info for GG-CNN backend")
        now = time.time()
        if self.sensor_stale_sec > 0.0:
            if now - detection_ts > self.sensor_stale_sec:
                raise ValueError("detection result is stale")
            if now - depth_ts > self.sensor_stale_sec:
                raise ValueError("depth image is stale")
        if not bool(detection.accepted):
            raise ValueError(str(detection.reason or "detection_not_accepted"))

        depth_image = decode_depth_image(depth_msg)
        if depth_image is None:
            raise ValueError(f"unsupported depth image encoding: {depth_msg.encoding}")
        if len(camera_info.k) < 9:
            raise ValueError("camera info K matrix is incomplete")

        mask_image = self._build_detection_mask(
            detection=detection,
            image_shape=depth_image.shape,
        )
        if int(np.count_nonzero(mask_image)) < self.ggcnn_min_mask_pixels:
            raise ValueError(
                f"mask too sparse for GG-CNN: {int(np.count_nonzero(mask_image))} < {self.ggcnn_min_mask_pixels}"
            )

        roi_x0, roi_y0, roi_x1, roi_y1 = self._compute_detection_roi(
            detection=detection,
            mask_image=mask_image,
            image_shape=depth_image.shape,
            margin_px=self.ggcnn_bbox_margin_px,
        )
        roi_depth = depth_image[roi_y0:roi_y1, roi_x0:roi_x1].astype(np.float32, copy=True)
        roi_mask = mask_image[roi_y0:roi_y1, roi_x0:roi_x1] > 0
        roi_mask_pixels = int(np.count_nonzero(roi_mask))
        if roi_mask_pixels < self.ggcnn_min_mask_pixels:
            raise ValueError(
                f"GG-CNN ROI mask too sparse: {roi_mask_pixels} < {self.ggcnn_min_mask_pixels}"
            )

        roi_valid_depth_mask = np.isfinite(roi_depth) & (roi_depth > 0.0)
        roi_target_valid_depth_mask = roi_valid_depth_mask & roi_mask
        roi_target_valid_depth_pixels = int(np.count_nonzero(roi_target_valid_depth_mask))
        if roi_target_valid_depth_pixels <= 0:
            raise ValueError("no valid masked depth pixels for GG-CNN")

        target_depth_m = float(np.median(roi_depth[roi_target_valid_depth_mask]))
        network_input, resized_mask = self._preprocess_ggcnn_depth_roi(
            depth_roi=roi_depth,
            roi_mask=roi_mask,
        )
        source_frame = (
            str(camera_info.header.frame_id or "").strip() or
            str(depth_msg.header.frame_id or "").strip() or
            str(detection.header.frame_id or "").strip()
        )
        rotation, translation = self._lookup_target_transform(
            source_frame=source_frame,
            stamp=depth_msg.header.stamp,
        )

        bbox_center_x = 0.5 * float(roi_x0 + roi_x1)
        bbox_center_y = 0.5 * float(roi_y0 + roi_y1)
        bbox_width = float(roi_x1 - roi_x0)
        bbox_height = float(roi_y1 - roi_y0)
        target_signature = (
            int(round(bbox_center_x / float(self.request_signature_center_bin_px))),
            int(round(bbox_center_y / float(self.request_signature_center_bin_px))),
            int(round(bbox_width / float(self.request_signature_size_bin_px))),
            int(round(bbox_height / float(self.request_signature_size_bin_px))),
            int(round(target_depth_m / self.request_signature_depth_bin_m)),
        )

        return {
            "frame_id": self.target_frame,
            "source_frame": source_frame,
            "stamp": depth_msg.header.stamp,
            "target_signature": target_signature,
            "depth_image": depth_image,
            "mask_image": mask_image,
            "roi_depth": roi_depth,
            "roi_mask": roi_mask,
            "roi_xyxy": (roi_x0, roi_y0, roi_x1, roi_y1),
            "roi_mask_pixels": roi_mask_pixels,
            "roi_target_valid_depth_pixels": roi_target_valid_depth_pixels,
            "network_input": network_input,
            "resized_mask": resized_mask,
            "input_size": self.ggcnn_input_size,
            "fx": float(camera_info.k[0]),
            "fy": float(camera_info.k[4]),
            "cx": float(camera_info.k[2]),
            "cy": float(camera_info.k[5]),
            "target_depth_m": target_depth_m,
            "rotation": rotation,
            "translation": translation,
        }

    def _build_detection_mask(
        self,
        *,
        detection: DetectionResult,
        image_shape: tuple[int, int],
    ) -> np.ndarray:
        image_h, image_w = int(image_shape[0]), int(image_shape[1])
        mask_image: Optional[np.ndarray] = None
        if bool(detection.has_mask):
            mask_image = decode_mono8_image(detection.mask)
            if mask_image is not None and mask_image.shape[:2] != (image_h, image_w):
                mask_image = cv2.resize(
                    mask_image,
                    (image_w, image_h),
                    interpolation=cv2.INTER_NEAREST,
                )
        if mask_image is None:
            mask_image = np.zeros((image_h, image_w), dtype=np.uint8)
            if bool(detection.has_bbox) and int(detection.bbox.width) > 0 and int(detection.bbox.height) > 0:
                x0 = max(0, int(detection.bbox.x_offset))
                y0 = max(0, int(detection.bbox.y_offset))
                x1 = min(image_w, x0 + int(detection.bbox.width))
                y1 = min(image_h, y0 + int(detection.bbox.height))
                if x1 > x0 and y1 > y0:
                    mask_image[y0:y1, x0:x1] = 255
        return (mask_image > 0).astype(np.uint8) * 255

    def _compute_detection_roi(
        self,
        *,
        detection: DetectionResult,
        mask_image: np.ndarray,
        image_shape: tuple[int, int],
        margin_px: int,
    ) -> tuple[int, int, int, int]:
        image_h, image_w = int(image_shape[0]), int(image_shape[1])
        if bool(detection.has_bbox) and int(detection.bbox.width) > 0 and int(detection.bbox.height) > 0:
            x0 = int(detection.bbox.x_offset)
            y0 = int(detection.bbox.y_offset)
            x1 = x0 + int(detection.bbox.width)
            y1 = y0 + int(detection.bbox.height)
        else:
            ys, xs = np.nonzero(mask_image > 0)
            if xs.size <= 0 or ys.size <= 0:
                raise ValueError("unable to compute ROI from empty detection mask")
            x0 = int(xs.min())
            y0 = int(ys.min())
            x1 = int(xs.max()) + 1
            y1 = int(ys.max()) + 1

        x0 = max(0, x0 - int(margin_px))
        y0 = max(0, y0 - int(margin_px))
        x1 = min(image_w, x1 + int(margin_px))
        y1 = min(image_h, y1 + int(margin_px))
        if x1 - x0 < 2 or y1 - y0 < 2:
            raise ValueError("computed ROI is too small")
        return x0, y0, x1, y1

    def _preprocess_ggcnn_depth_roi(
        self,
        *,
        depth_roi: np.ndarray,
        roi_mask: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        depth = np.nan_to_num(np.asarray(depth_roi, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        mask = np.asarray(roi_mask, dtype=bool)
        depth[~mask] = 0.0

        bordered_depth = cv2.copyMakeBorder(depth, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        missing_mask = (bordered_depth <= 0.0).astype(np.uint8)
        scale = float(np.max(np.abs(bordered_depth)))
        if scale <= 1e-6:
            raise ValueError("GG-CNN ROI depth has no valid scale")
        normalized_depth = bordered_depth.astype(np.float32) / scale
        inpainted = cv2.inpaint(normalized_depth, missing_mask, 1, cv2.INPAINT_NS)
        inpainted = inpainted[1:-1, 1:-1] * scale

        resized_depth = cv2.resize(
            inpainted,
            (self.ggcnn_input_size, self.ggcnn_input_size),
            interpolation=cv2.INTER_LINEAR,
        ).astype(np.float32)
        resized_mask = cv2.resize(
            mask.astype(np.uint8),
            (self.ggcnn_input_size, self.ggcnn_input_size),
            interpolation=cv2.INTER_NEAREST,
        ) > 0

        network_input = resized_depth.astype(np.float32) / 255.0
        network_input -= float(network_input.mean())
        return network_input.astype(np.float32), resized_mask.astype(bool)

    def _lookup_target_transform(self, *, source_frame: str, stamp) -> tuple[np.ndarray, np.ndarray]:
        if not source_frame:
            raise ValueError("source frame is empty for grasp backend transform")
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
                raise ValueError(
                    f"grasp backend transform failed from {source_frame} to {self.target_frame}: {exc}"
                ) from exc

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

    def _extract_ggcnn_candidates(
        self,
        *,
        q_img: np.ndarray,
        ang_img: np.ndarray,
        width_img: np.ndarray,
        valid_mask: np.ndarray,
    ) -> list[dict[str, float]]:
        q_values = np.asarray(q_img, dtype=np.float32)
        ang_values = np.asarray(ang_img, dtype=np.float32)
        width_values = np.asarray(width_img, dtype=np.float32)
        candidate_mask = np.asarray(valid_mask, dtype=bool) & np.isfinite(q_values)
        candidate_mask &= q_values >= float(self.ggcnn_quality_threshold)
        coords = np.argwhere(candidate_mask)
        if coords.size == 0:
            return []

        score_values = q_values[candidate_mask]
        sort_order = np.argsort(score_values)[::-1]
        sorted_coords = coords[sort_order]
        sorted_scores = score_values[sort_order]
        accepted: list[dict[str, float]] = []
        radius_sq = float(self.ggcnn_nms_radius_px * self.ggcnn_nms_radius_px)
        for coord, score in zip(sorted_coords, sorted_scores):
            row = int(coord[0])
            col = int(coord[1])
            too_close = False
            for existing in accepted:
                dy = row - int(existing["row"])
                dx = col - int(existing["col"])
                if float(dy * dy + dx * dx) < radius_sq:
                    too_close = True
                    break
            if too_close:
                continue
            accepted.append(
                {
                    "row": row,
                    "col": col,
                    "score": float(score),
                    "angle_rad": float(ang_values[row, col]),
                    "length_px": float(max(4.0, width_values[row, col])),
                }
            )
            if len(accepted) >= self.ggcnn_top_k:
                break
        return accepted

    def _post_process_ggcnn_output(self, pos, cos, sin, width):
        q_img = pos.detach().cpu().numpy().squeeze().astype(np.float32)
        ang_img = (
            np.arctan2(
                sin.detach().cpu().numpy().squeeze(),
                cos.detach().cpu().numpy().squeeze(),
            ) / 2.0
        ).astype(np.float32)
        width_img = (width.detach().cpu().numpy().squeeze().astype(np.float32) * 150.0)

        q_img = cv2.GaussianBlur(q_img, (0, 0), 2.0)
        ang_img = cv2.GaussianBlur(ang_img, (0, 0), 2.0)
        width_img = cv2.GaussianBlur(width_img, (0, 0), 1.0)
        return q_img, ang_img, width_img

    def _publish_ggcnn_shadow_images(
        self,
        *,
        context: dict[str, Any],
        q_img: np.ndarray,
        ang_img: np.ndarray,
        width_img: np.ndarray,
        candidates: list[dict[str, float]],
        selected_index: int,
    ) -> None:
        stamp = context["stamp"]
        frame_id = str(context["source_frame"] or context["frame_id"])
        depth_roi = np.asarray(context["roi_depth"], dtype=np.float32)
        roi_mask = np.asarray(context["roi_mask"], dtype=bool)
        valid_depth = np.isfinite(depth_roi) & (depth_roi > 0.0)

        depth_vis = self._colorize_scalar_map(
            scalar=depth_roi,
            valid_mask=valid_depth,
            colormap=cv2.COLORMAP_BONE,
        )
        q_vis = self._colorize_scalar_map(
            scalar=q_img,
            valid_mask=np.asarray(context["resized_mask"], dtype=bool),
            colormap=cv2.COLORMAP_TURBO,
            vmin=0.0,
            vmax=max(0.4, float(np.max(q_img)) if q_img.size else 1.0),
        )
        angle_vis = self._colorize_angle_map(
            angle_img=ang_img,
            valid_mask=np.asarray(context["resized_mask"], dtype=bool),
        )
        width_vis = self._colorize_scalar_map(
            scalar=width_img,
            valid_mask=np.asarray(context["resized_mask"], dtype=bool),
            colormap=cv2.COLORMAP_VIRIDIS,
        )
        overlay_vis = depth_vis.copy()
        for index, candidate in enumerate(candidates):
            p1, p2, center = self._candidate_endpoints_on_roi(
                context=context,
                candidate=candidate,
            )
            color = (80, 255, 120) if index == selected_index else (255, 210, 120)
            thickness = 3 if index == selected_index else 2
            cv2.line(overlay_vis, p1, p2, color, thickness, cv2.LINE_AA)
            cv2.circle(overlay_vis, center, 3 if index == selected_index else 2, (255, 64, 64), -1)
            cv2.putText(
                overlay_vis,
                f"#{index + 1}:{float(candidate['score']):.2f}",
                (max(0, center[0] + 4), max(14, center[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv2.LINE_AA,
            )

        depth_msg = make_rgb8_image(depth_vis, frame_id, stamp)
        q_msg = make_rgb8_image(q_vis, frame_id, stamp)
        angle_msg = make_rgb8_image(angle_vis, frame_id, stamp)
        width_msg = make_rgb8_image(width_vis, frame_id, stamp)
        overlay_msg = make_rgb8_image(overlay_vis, frame_id, stamp)
        self.ggcnn_depth_roi_pub.publish(depth_msg)
        self.ggcnn_q_heatmap_pub.publish(q_msg)
        self.ggcnn_angle_map_pub.publish(angle_msg)
        self.ggcnn_width_map_pub.publish(width_msg)
        self.ggcnn_grasp_overlay_pub.publish(overlay_msg)
        self._last_ggcnn_depth_roi_msg = copy.deepcopy(depth_msg)
        self._last_ggcnn_q_heatmap_msg = copy.deepcopy(q_msg)
        self._last_ggcnn_angle_map_msg = copy.deepcopy(angle_msg)
        self._last_ggcnn_width_map_msg = copy.deepcopy(width_msg)
        self._last_ggcnn_grasp_overlay_msg = copy.deepcopy(overlay_msg)

    def _colorize_scalar_map(
        self,
        *,
        scalar: np.ndarray,
        valid_mask: np.ndarray,
        colormap: int,
        vmin: Optional[float] = None,
        vmax: Optional[float] = None,
    ) -> np.ndarray:
        values = np.asarray(scalar, dtype=np.float32)
        mask = np.asarray(valid_mask, dtype=bool)
        if values.ndim != 2:
            raise ValueError("scalar map must be HxW")
        normalized = np.zeros(values.shape, dtype=np.uint8)
        if np.any(mask):
            valid_values = values[mask]
            low = float(np.min(valid_values) if vmin is None else vmin)
            high = float(np.max(valid_values) if vmax is None else vmax)
            if high <= low + 1e-6:
                high = low + 1.0
            clipped = np.clip((values - low) / (high - low), 0.0, 1.0)
            normalized = np.rint(clipped * 255.0).astype(np.uint8)
        color_bgr = cv2.applyColorMap(normalized, colormap)
        color_bgr[~mask] = 0
        return cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

    def _colorize_angle_map(self, *, angle_img: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
        values = np.asarray(angle_img, dtype=np.float32)
        mask = np.asarray(valid_mask, dtype=bool)
        hue = np.zeros(values.shape, dtype=np.uint8)
        hue[mask] = np.rint(((values[mask] + (math.pi / 2.0)) / math.pi) * 179.0).astype(np.uint8)
        saturation = np.zeros(values.shape, dtype=np.uint8)
        saturation[mask] = 255
        value = np.zeros(values.shape, dtype=np.uint8)
        value[mask] = 255
        hsv = np.stack([hue, saturation, value], axis=2)
        rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        rgb[~mask] = 0
        return rgb

    def _candidate_endpoints_on_roi(
        self,
        *,
        context: dict[str, Any],
        candidate: dict[str, float],
    ) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int]]:
        roi_depth = np.asarray(context["roi_depth"], dtype=np.float32)
        roi_h, roi_w = roi_depth.shape[:2]
        scale_x = float(roi_w) / float(context["input_size"])
        scale_y = float(roi_h) / float(context["input_size"])
        center_row = float(candidate["row"]) * scale_y
        center_col = float(candidate["col"]) * scale_x
        angle_rad = float(candidate["angle_rad"])
        half_length = 0.5 * float(candidate["length_px"])
        half_dx = math.cos(angle_rad) * half_length * scale_x
        half_dy = -math.sin(angle_rad) * half_length * scale_y

        def clamp_point(x_value: float, y_value: float) -> tuple[int, int]:
            x_int = int(round(max(0.0, min(float(roi_w - 1), x_value))))
            y_int = int(round(max(0.0, min(float(roi_h - 1), y_value))))
            return (x_int, y_int)

        start = clamp_point(center_col - half_dx, center_row - half_dy)
        end = clamp_point(center_col + half_dx, center_row + half_dy)
        center = clamp_point(center_col, center_row)
        return start, end, center

    def _build_ggcnn_proposal_item(
        self,
        *,
        context: dict[str, Any],
        candidate: dict[str, float],
        candidate_rank: int,
    ) -> Optional[dict[str, Any]]:
        roi_x0, roi_y0, roi_x1, roi_y1 = context["roi_xyxy"]
        roi_width = float(roi_x1 - roi_x0)
        roi_height = float(roi_y1 - roi_y0)
        input_size = float(context["input_size"])
        center_row = float(candidate["row"])
        center_col = float(candidate["col"])
        angle_rad = float(candidate["angle_rad"])
        half_length = 0.5 * float(candidate["length_px"])
        du = math.cos(angle_rad)
        dv = -math.sin(angle_rad)

        contact_1_row = center_row - dv * half_length
        contact_1_col = center_col - du * half_length
        contact_2_row = center_row + dv * half_length
        contact_2_col = center_col + du * half_length

        center_u = roi_x0 + (center_col + 0.5) * roi_width / input_size
        center_v = roi_y0 + (center_row + 0.5) * roi_height / input_size
        contact_1_u = roi_x0 + (contact_1_col + 0.5) * roi_width / input_size
        contact_1_v = roi_y0 + (contact_1_row + 0.5) * roi_height / input_size
        contact_2_u = roi_x0 + (contact_2_col + 0.5) * roi_width / input_size
        contact_2_v = roi_y0 + (contact_2_row + 0.5) * roi_height / input_size

        center_depth = self._sample_depth_at(
            depth_image=context["depth_image"],
            mask_image=context["mask_image"],
            u=center_u,
            v=center_v,
            fallback_depth_m=float(context["target_depth_m"]),
        )
        contact_1_depth = self._sample_depth_at(
            depth_image=context["depth_image"],
            mask_image=context["mask_image"],
            u=contact_1_u,
            v=contact_1_v,
            fallback_depth_m=center_depth,
        )
        contact_2_depth = self._sample_depth_at(
            depth_image=context["depth_image"],
            mask_image=context["mask_image"],
            u=contact_2_u,
            v=contact_2_v,
            fallback_depth_m=center_depth,
        )

        center_camera = self._backproject_pixel_to_camera(
            u=center_u,
            v=center_v,
            depth_m=center_depth,
            fx=float(context["fx"]),
            fy=float(context["fy"]),
            cx=float(context["cx"]),
            cy=float(context["cy"]),
        )
        contact_1_camera = self._backproject_pixel_to_camera(
            u=contact_1_u,
            v=contact_1_v,
            depth_m=contact_1_depth,
            fx=float(context["fx"]),
            fy=float(context["fy"]),
            cx=float(context["cx"]),
            cy=float(context["cy"]),
        )
        contact_2_camera = self._backproject_pixel_to_camera(
            u=contact_2_u,
            v=contact_2_v,
            depth_m=contact_2_depth,
            fx=float(context["fx"]),
            fy=float(context["fy"]),
            cx=float(context["cx"]),
            cy=float(context["cy"]),
        )

        rotation = np.asarray(context["rotation"], dtype=np.float64)
        translation = np.asarray(context["translation"], dtype=np.float64)
        center_world = center_camera @ rotation.T + translation
        contact_1_world = contact_1_camera @ rotation.T + translation
        contact_2_world = contact_2_camera @ rotation.T + translation

        closing_direction = normalize_vector(contact_2_world - contact_1_world)
        if not np.any(closing_direction):
            return None
        approach_direction = normalize_vector(
            rotation @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
        )
        if not np.any(approach_direction):
            approach_direction = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        y_axis = normalize_vector(np.cross(approach_direction, closing_direction))
        if not np.any(y_axis):
            fallback_axis = np.array([0.0, 1.0, 0.0], dtype=np.float32)
            if abs(float(np.dot(fallback_axis, approach_direction))) > 0.95:
                fallback_axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            y_axis = normalize_vector(np.cross(approach_direction, fallback_axis))
        closing_direction = normalize_vector(np.cross(y_axis, approach_direction))
        if not np.any(closing_direction):
            return None

        grasp_width_m = float(np.linalg.norm(contact_2_world - contact_1_world))
        grasp_center = 0.5 * (contact_1_world + contact_2_world)
        pregrasp_position = grasp_center - approach_direction * float(self.ggcnn_pregrasp_offset_m)
        rotation_matrix = np.column_stack(
            [
                closing_direction.astype(np.float64),
                y_axis.astype(np.float64),
                approach_direction.astype(np.float64),
            ]
        )
        quaternion_xyzw = rotation_matrix_to_quaternion_xyzw(rotation_matrix)
        confidence_score = max(0.0, min(1.0, float(candidate["score"])))
        return {
            "frame_id": self.target_frame,
            "contact_point_1": contact_1_world.astype(float).tolist(),
            "contact_point_2": contact_2_world.astype(float).tolist(),
            "grasp_center": grasp_center.astype(float).tolist(),
            "closing_direction": closing_direction.astype(float).tolist(),
            "approach_direction": approach_direction.astype(float).tolist(),
            "grasp_pose_position": grasp_center.astype(float).tolist(),
            "grasp_pose_orientation": quaternion_xyzw,
            "pregrasp_pose_position": pregrasp_position.astype(float).tolist(),
            "pregrasp_pose_orientation": quaternion_xyzw,
            "grasp_width_m": grasp_width_m,
            "confidence_score": confidence_score,
            "semantic_score": 1.0,
            "candidate_rank": int(candidate_rank),
            "task_constraint_tag": self.default_task_constraint_tag,
        }

    def _sample_depth_at(
        self,
        *,
        depth_image: np.ndarray,
        mask_image: np.ndarray,
        u: float,
        v: float,
        fallback_depth_m: float,
        window_radius_px: int = 3,
    ) -> float:
        image_h, image_w = depth_image.shape[:2]
        x = int(round(u))
        y = int(round(v))
        x = max(0, min(image_w - 1, x))
        y = max(0, min(image_h - 1, y))
        x0 = max(0, x - window_radius_px)
        y0 = max(0, y - window_radius_px)
        x1 = min(image_w, x + window_radius_px + 1)
        y1 = min(image_h, y + window_radius_px + 1)

        depth_patch = depth_image[y0:y1, x0:x1]
        mask_patch = mask_image[y0:y1, x0:x1] > 0
        valid_mask = np.isfinite(depth_patch) & (depth_patch > 0.0) & mask_patch
        if not np.any(valid_mask):
            valid_mask = np.isfinite(depth_patch) & (depth_patch > 0.0)
        if np.any(valid_mask):
            return float(np.median(depth_patch[valid_mask]))
        return float(fallback_depth_m)

    def _backproject_pixel_to_camera(
        self,
        *,
        u: float,
        v: float,
        depth_m: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> np.ndarray:
        return np.array(
            [
                (float(u) - cx) * float(depth_m) / fx,
                (float(v) - cy) * float(depth_m) / fy,
                float(depth_m),
            ],
            dtype=np.float64,
        )

    def _run_point_cloud_http_backend(
        self,
        *,
        semantic_task: Optional[SemanticTask],
        cloud_msg: Optional[PointCloud2],
        cloud_ts: float,
    ) -> dict[str, Any]:
        if not self.backend_url:
            raise ValueError("grasp backend_url is empty")
        if cloud_msg is None:
            raise ValueError("missing target cloud for http_json grasp backend")
        if time.time() - cloud_ts > self.sensor_stale_sec:
            raise ValueError("target cloud is stale")

        points = point_cloud2_to_xyz_array(cloud_msg)
        if points.shape[0] < self.min_points_required:
            raise ValueError(
                f"target cloud has too few points: {points.shape[0]} < {self.min_points_required}"
            )
        if points.shape[0] > self.max_points:
            step = max(1, points.shape[0] // self.max_points)
            points = points[::step][: self.max_points]

        payload = {
            "frame_id": cloud_msg.header.frame_id,
            "points_xyz": points.astype(float).tolist(),
            "task": semantic_task.task if semantic_task is not None else "pick",
            "target_label": semantic_task.target_label if semantic_task is not None else "",
            "target_hint": semantic_task.target_hint if semantic_task is not None else "",
            "constraints": list(semantic_task.constraints) if semantic_task is not None else [],
        }
        response = self._session.post(
            self.backend_url,
            json=payload,
            timeout=self.request_timeout_sec,
        )
        if not response.ok:
            detail = response.text.strip()
            detail = detail[:600] if detail else response.reason
            raise RuntimeError(
                f"Contact-GraspNet HTTP {response.status_code}: {detail or 'request failed'}"
            )
        parsed = response.json()
        if not isinstance(parsed, dict):
            raise ValueError("grasp backend response is not a JSON object")
        return parsed

    def _run_contact_graspnet_http_backend(
        self,
        *,
        semantic_task: Optional[SemanticTask],
        detection: Optional[DetectionResult],
        detection_ts: float,
        depth_msg: Optional[Image],
        depth_ts: float,
        camera_info: Optional[CameraInfo],
        camera_info_ts: float,
    ) -> dict[str, Any]:
        if not self.backend_url:
            raise ValueError("grasp backend_url is empty")
        context = self._extract_contact_graspnet_context(
            detection=detection,
            detection_ts=detection_ts,
            depth_msg=depth_msg,
            depth_ts=depth_ts,
            camera_info=camera_info,
            camera_info_ts=camera_info_ts,
        )
        payload: dict[str, Any] = {
            "frame_id": context["frame_id"],
            "task": semantic_task.task if semantic_task is not None else "pick",
            "target_label": semantic_task.target_label if semantic_task is not None else "",
            "target_hint": semantic_task.target_hint if semantic_task is not None else "",
            "constraints": list(semantic_task.constraints) if semantic_task is not None else [],
            "excluded_labels": list(semantic_task.excluded_labels) if semantic_task is not None else [],
            "depth_png_b64": encode_depth_m_to_base64_png(context["depth_roi"]),
            "depth_scale_m_per_unit": 0.001,
            "segmap_png_b64": encode_png_base64(context["segmap_roi"]),
            "segmap_id": self.contact_graspnet_segmap_id,
            "image_width": int(context["depth_roi"].shape[1]),
            "image_height": int(context["depth_roi"].shape[0]),
            "K": context["k_matrix"],
            "local_regions": self.contact_graspnet_local_regions,
            "filter_grasps": self.contact_graspnet_filter_grasps,
            "skip_border_objects": self.contact_graspnet_skip_border_objects,
            "forward_passes": self.contact_graspnet_forward_passes,
            "z_range": list(context["z_range"]),
            "pregrasp_offset_m": self.contact_graspnet_pregrasp_offset_m,
            "max_proposals": self.contact_graspnet_max_proposals,
            "task_constraint_tag": self.default_task_constraint_tag,
            "visualize": self.contact_graspnet_visualize,
        }
        payload["bbox_xyxy"] = [0, 0, int(context["depth_roi"].shape[1]), int(context["depth_roi"].shape[0])]
        self.get_logger().info(
            "grasp backend decision: "
            f"label={str(detection.target_label or semantic_task.target_label if semantic_task is not None else '') or 'unknown'} "
            f"signature={context['target_signature']} "
            f"roi={int(context['depth_roi'].shape[1])}x{int(context['depth_roi'].shape[0])} "
            f"full_mask_pixels={int(context['full_mask_pixels'])} "
            f"full_target_valid_depth_pixels={int(context['full_target_valid_depth_pixels'])} "
            f"roi_mask_pixels={int(context['roi_mask_pixels'])} "
            f"roi_target_valid_depth_pixels={int(context['roi_target_valid_depth_pixels'])} "
            f"roi_target_z_valid_depth_pixels={int(context['roi_target_z_valid_depth_pixels'])} "
            f"target_depth_m={float(context['target_depth_m']):.3f} "
            f"z_range=[{float(context['z_range'][0]):.3f},{float(context['z_range'][1]):.3f}]"
        )

        response = self._session.post(
            self.backend_url,
            json=payload,
            timeout=self.request_timeout_sec,
        )
        if not response.ok:
            detail = response.text.strip()
            detail = detail[:600] if detail else response.reason
            raise RuntimeError(
                f"Contact-GraspNet HTTP {response.status_code}: {detail or 'request failed'}"
            )
        parsed = response.json()
        if not isinstance(parsed, dict):
            raise ValueError("Contact-GraspNet response is not a JSON object")
        parsed_debug = dict(parsed.get("debug") or {})
        parsed_debug.update(
            {
                "mask_pixels": int(context["roi_mask_pixels"]),
                "segmap_id": self.contact_graspnet_segmap_id,
                "visualize": self.contact_graspnet_visualize,
                "roi_xyxy": list(context["roi_xyxy"]),
                "target_depth_m": float(context["target_depth_m"]),
                "z_range": list(context["z_range"]),
                "target_signature": str(context["target_signature"]),
                "full_mask_pixels": int(context["full_mask_pixels"]),
                "full_valid_depth_pixels": int(context["full_valid_depth_pixels"]),
                "full_target_valid_depth_pixels": int(context["full_target_valid_depth_pixels"]),
                "roi_valid_depth_pixels": int(context["roi_valid_depth_pixels"]),
                "roi_mask_pixels": int(context["roi_mask_pixels"]),
                "roi_target_valid_depth_pixels": int(context["roi_target_valid_depth_pixels"]),
                "roi_target_z_valid_depth_pixels": int(context["roi_target_z_valid_depth_pixels"]),
            }
        )
        parsed["debug"] = parsed_debug
        return parsed

    def _extract_contact_graspnet_context(
        self,
        *,
        detection: Optional[DetectionResult],
        detection_ts: float,
        depth_msg: Optional[Image],
        depth_ts: float,
        camera_info: Optional[CameraInfo],
        camera_info_ts: float,
    ) -> dict[str, Any]:
        if detection is None or depth_msg is None or camera_info is None:
            raise ValueError("missing detection/depth/camera info for Contact-GraspNet backend")
        now = time.time()
        if self.sensor_stale_sec > 0.0:
            if now - detection_ts > self.sensor_stale_sec:
                raise ValueError("detection result is stale")
            if now - depth_ts > self.sensor_stale_sec:
                raise ValueError("depth image is stale")
        if not bool(detection.accepted):
            raise ValueError(str(detection.reason or "detection_not_accepted"))
        if not bool(detection.has_mask):
            raise ValueError("Contact-GraspNet backend requires an instance mask")

        depth_image = decode_depth_image(depth_msg)
        if depth_image is None:
            raise ValueError(f"unsupported depth image encoding: {depth_msg.encoding}")

        mask_image = decode_mono8_image(detection.mask)
        if mask_image is None:
            raise ValueError("failed to decode detection mask")
        if mask_image.shape[:2] != depth_image.shape[:2]:
            mask_image = cv2.resize(
                mask_image,
                (depth_image.shape[1], depth_image.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        segmap_full = np.where(mask_image > 0, self.contact_graspnet_segmap_id, 0).astype(np.uint8)
        full_valid_depth_mask = np.isfinite(depth_image) & (depth_image > 0.0)
        full_mask_pixels = int(np.count_nonzero(segmap_full))
        full_valid_depth_pixels = int(np.count_nonzero(full_valid_depth_mask))
        full_target_valid_depth_pixels = int(np.count_nonzero(full_valid_depth_mask & (segmap_full > 0)))
        roi_x0, roi_y0, roi_x1, roi_y1 = self._compute_contact_graspnet_roi(
            detection=detection,
            segmap_full=segmap_full,
            image_shape=depth_image.shape,
        )
        depth_roi = depth_image[roi_y0:roi_y1, roi_x0:roi_x1]
        segmap_roi = segmap_full[roi_y0:roi_y1, roi_x0:roi_x1]
        roi_mask_pixels = int(np.count_nonzero(segmap_roi))
        if roi_mask_pixels < self.contact_graspnet_min_mask_pixels:
            raise ValueError(
                f"mask too sparse for Contact-GraspNet: {roi_mask_pixels} < {self.contact_graspnet_min_mask_pixels}"
            )

        roi_valid_depth_mask = np.isfinite(depth_roi) & (depth_roi > 0.0)
        valid_depth_mask = roi_valid_depth_mask & (segmap_roi > 0)
        roi_valid_depth_pixels = int(np.count_nonzero(roi_valid_depth_mask))
        roi_target_valid_depth_pixels = int(np.count_nonzero(valid_depth_mask))
        depth_values = depth_roi[valid_depth_mask]
        if depth_values.size <= 0:
            raise ValueError(
                "no valid target depth pixels inside Contact-GraspNet ROI "
                f"(full_target_valid_depth_pixels={full_target_valid_depth_pixels} "
                f"roi_mask_pixels={roi_mask_pixels} roi_valid_depth_pixels={roi_valid_depth_pixels})"
            )
        target_depth_m = float(np.median(depth_values))

        z_min = max(self.contact_graspnet_z_range[0], target_depth_m - self.contact_graspnet_depth_window_m)
        z_max = min(self.contact_graspnet_z_range[1], target_depth_m + self.contact_graspnet_depth_window_m)
        if z_max <= z_min:
            epsilon = max(0.02, 0.5 * self.contact_graspnet_depth_window_m)
            z_min = max(self.contact_graspnet_z_range[0], target_depth_m - epsilon)
            z_max = min(self.contact_graspnet_z_range[1], target_depth_m + epsilon)
        roi_target_z_valid_depth_pixels = int(
            np.count_nonzero(valid_depth_mask & (depth_roi >= z_min) & (depth_roi <= z_max))
        )

        if len(camera_info.k) < 9:
            raise ValueError("camera info K matrix is incomplete")
        k_matrix = [
            float(camera_info.k[0]),
            float(camera_info.k[1]),
            float(camera_info.k[2] - roi_x0),
            float(camera_info.k[3]),
            float(camera_info.k[4]),
            float(camera_info.k[5] - roi_y0),
            float(camera_info.k[6]),
            float(camera_info.k[7]),
            float(camera_info.k[8]),
        ]

        bbox_center_x = 0.5 * float(roi_x0 + roi_x1)
        bbox_center_y = 0.5 * float(roi_y0 + roi_y1)
        bbox_width = float(roi_x1 - roi_x0)
        bbox_height = float(roi_y1 - roi_y0)
        target_signature = (
            int(round(bbox_center_x / float(self.request_signature_center_bin_px))),
            int(round(bbox_center_y / float(self.request_signature_center_bin_px))),
            int(round(bbox_width / float(self.request_signature_size_bin_px))),
            int(round(bbox_height / float(self.request_signature_size_bin_px))),
            int(round(target_depth_m / self.request_signature_depth_bin_m)),
        )

        return {
            "depth_roi": depth_roi,
            "segmap_roi": segmap_roi,
            "k_matrix": k_matrix,
            "mask_pixels": roi_mask_pixels,
            "full_mask_pixels": full_mask_pixels,
            "full_valid_depth_pixels": full_valid_depth_pixels,
            "full_target_valid_depth_pixels": full_target_valid_depth_pixels,
            "roi_mask_pixels": roi_mask_pixels,
            "roi_valid_depth_pixels": roi_valid_depth_pixels,
            "roi_target_valid_depth_pixels": roi_target_valid_depth_pixels,
            "roi_target_z_valid_depth_pixels": roi_target_z_valid_depth_pixels,
            "roi_xyxy": (roi_x0, roi_y0, roi_x1, roi_y1),
            "target_depth_m": target_depth_m,
            "z_range": (z_min, z_max),
            "frame_id": camera_info.header.frame_id or depth_msg.header.frame_id or detection.header.frame_id,
            "target_signature": target_signature,
        }

    def _compute_contact_graspnet_roi(
        self,
        *,
        detection: DetectionResult,
        segmap_full: np.ndarray,
        image_shape: tuple[int, int],
    ) -> tuple[int, int, int, int]:
        image_h, image_w = int(image_shape[0]), int(image_shape[1])
        if bool(detection.has_bbox) and int(detection.bbox.width) > 0 and int(detection.bbox.height) > 0:
            x0 = int(detection.bbox.x_offset)
            y0 = int(detection.bbox.y_offset)
            x1 = x0 + int(detection.bbox.width)
            y1 = y0 + int(detection.bbox.height)
        else:
            ys, xs = np.nonzero(segmap_full > 0)
            if xs.size <= 0 or ys.size <= 0:
                raise ValueError("unable to compute Contact-GraspNet ROI from empty mask")
            x0 = int(xs.min())
            y0 = int(ys.min())
            x1 = int(xs.max()) + 1
            y1 = int(ys.max()) + 1

        margin = int(self.contact_graspnet_roi_margin_px)
        x0 = max(0, x0 - margin)
        y0 = max(0, y0 - margin)
        x1 = min(image_w, x1 + margin)
        y1 = min(image_h, y1 + margin)
        if x1 - x0 < 2 or y1 - y0 < 2:
            raise ValueError("computed Contact-GraspNet ROI is too small")
        return x0, y0, x1, y1

    def _parse_proposal_array(self, parsed: dict[str, Any]) -> GraspProposalArray:
        proposals_raw = parsed.get("proposals", [])
        if not isinstance(proposals_raw, list):
            raise ValueError("grasp backend response missing proposals array")

        proposal_array = GraspProposalArray()
        proposal_array.header.frame_id = str(parsed.get("frame_id") or "")
        proposal_array.selected_index = max(0, int(parsed.get("selected_index", 0)))
        for item in proposals_raw:
            if not isinstance(item, dict):
                continue
            proposal = GraspProposal()
            proposal.header.frame_id = str(item.get("frame_id") or proposal_array.header.frame_id)
            proposal.contact_point_1 = to_point(item.get("contact_point_1"))
            proposal.contact_point_2 = to_point(item.get("contact_point_2"))
            proposal.grasp_center = to_point(item.get("grasp_center"))
            proposal.closing_direction = to_vector3(item.get("closing_direction"))
            proposal.approach_direction = to_vector3(item.get("approach_direction"))
            grasp_position = to_point(item.get("grasp_pose_position"))
            proposal.grasp_pose.position.x = grasp_position.x
            proposal.grasp_pose.position.y = grasp_position.y
            proposal.grasp_pose.position.z = grasp_position.z
            pregrasp_position = to_point(item.get("pregrasp_pose_position"))
            proposal.pregrasp_pose.position.x = pregrasp_position.x
            proposal.pregrasp_pose.position.y = pregrasp_position.y
            proposal.pregrasp_pose.position.z = pregrasp_position.z

            grasp_orientation = item.get("grasp_pose_orientation")
            if isinstance(grasp_orientation, (list, tuple)) and len(grasp_orientation) >= 4:
                proposal.grasp_pose.orientation.x = float(grasp_orientation[0])
                proposal.grasp_pose.orientation.y = float(grasp_orientation[1])
                proposal.grasp_pose.orientation.z = float(grasp_orientation[2])
                proposal.grasp_pose.orientation.w = float(grasp_orientation[3])
            else:
                proposal.grasp_pose.orientation.w = 1.0

            pregrasp_orientation = item.get("pregrasp_pose_orientation")
            if isinstance(pregrasp_orientation, (list, tuple)) and len(pregrasp_orientation) >= 4:
                proposal.pregrasp_pose.orientation.x = float(pregrasp_orientation[0])
                proposal.pregrasp_pose.orientation.y = float(pregrasp_orientation[1])
                proposal.pregrasp_pose.orientation.z = float(pregrasp_orientation[2])
                proposal.pregrasp_pose.orientation.w = float(pregrasp_orientation[3])
            else:
                proposal.pregrasp_pose.orientation.x = proposal.grasp_pose.orientation.x
                proposal.pregrasp_pose.orientation.y = proposal.grasp_pose.orientation.y
                proposal.pregrasp_pose.orientation.z = proposal.grasp_pose.orientation.z
                proposal.pregrasp_pose.orientation.w = proposal.grasp_pose.orientation.w

            proposal.grasp_width_m = float(item.get("grasp_width_m", 0.0))
            proposal.confidence_score = float(item.get("confidence_score", 0.0))
            proposal.semantic_score = float(item.get("semantic_score", 0.0))
            proposal.candidate_rank = int(item.get("candidate_rank", 0))
            proposal.task_constraint_tag = str(
                item.get("task_constraint_tag") or self.default_task_constraint_tag
            )
            proposal_array.proposals.append(proposal)

        if proposal_array.proposals and proposal_array.selected_index >= len(proposal_array.proposals):
            proposal_array.selected_index = 0
        return proposal_array

    def _publish_candidate_visuals(
        self,
        proposal_array: GraspProposalArray,
        *,
        cache_visuals: bool = True,
    ) -> None:
        if cache_visuals:
            self._last_visual_proposal_array = copy.deepcopy(proposal_array)
        stamp = self.get_clock().now().to_msg()
        frame_id = str(proposal_array.header.frame_id or "").strip()
        if not frame_id:
            for proposal in proposal_array.proposals:
                if proposal.header.frame_id:
                    frame_id = proposal.header.frame_id
                    break
        if not frame_id:
            frame_id = self._last_visual_frame_id
        else:
            self._last_visual_frame_id = frame_id

        selected_index = int(proposal_array.selected_index)
        if selected_index < 0 or selected_index >= len(proposal_array.proposals):
            selected_index = 0

        self._publish_selected_debug_clouds(
            frame_id=frame_id,
            stamp=stamp,
            proposal=proposal_array.proposals[selected_index] if proposal_array.proposals else None,
        )

        marker_array = MarkerArray()
        active_marker_ids: set[int] = set()
        for index, proposal in enumerate(proposal_array.proposals):
            marker_base = index * 10
            markers = self._build_candidate_markers(
                proposal=proposal,
                frame_id=frame_id,
                stamp=stamp,
                marker_base=marker_base,
                is_selected=(index == selected_index),
                list_index=index,
            )
            marker_array.markers.extend(markers)
            active_marker_ids.update(marker.id for marker in markers)

        for marker_id in sorted(self._last_visual_marker_ids - active_marker_ids):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "grasp_backend"
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self._last_visual_marker_ids = active_marker_ids
        self.candidate_marker_pub.publish(marker_array)

    def _republish_last_visuals(self) -> None:
        if self._last_visual_proposal_array is not None:
            self._publish_candidate_visuals(self._last_visual_proposal_array, cache_visuals=False)
        if self._last_ggcnn_depth_roi_msg is not None:
            self.ggcnn_depth_roi_pub.publish(self._last_ggcnn_depth_roi_msg)
        if self._last_ggcnn_q_heatmap_msg is not None:
            self.ggcnn_q_heatmap_pub.publish(self._last_ggcnn_q_heatmap_msg)
        if self._last_ggcnn_angle_map_msg is not None:
            self.ggcnn_angle_map_pub.publish(self._last_ggcnn_angle_map_msg)
        if self._last_ggcnn_width_map_msg is not None:
            self.ggcnn_width_map_pub.publish(self._last_ggcnn_width_map_msg)
        if self._last_ggcnn_grasp_overlay_msg is not None:
            self.ggcnn_grasp_overlay_pub.publish(self._last_ggcnn_grasp_overlay_msg)

    def _publish_selected_debug_clouds(
        self,
        *,
        frame_id: str,
        stamp,
        proposal: Optional[GraspProposal],
    ) -> None:
        empty_cloud = make_xyz_cloud(np.empty((0, 3), dtype=np.float32), frame_id, stamp)
        if proposal is None:
            self.selected_contact_point_1_pub.publish(empty_cloud)
            self.selected_contact_point_2_pub.publish(empty_cloud)
            self.selected_grasp_center_pub.publish(empty_cloud)
            return

        self.selected_contact_point_1_pub.publish(
            make_xyz_cloud(point_to_numpy(proposal.contact_point_1).reshape((1, 3)), frame_id, stamp)
        )
        self.selected_contact_point_2_pub.publish(
            make_xyz_cloud(point_to_numpy(proposal.contact_point_2).reshape((1, 3)), frame_id, stamp)
        )
        self.selected_grasp_center_pub.publish(
            make_xyz_cloud(point_to_numpy(proposal.grasp_center).reshape((1, 3)), frame_id, stamp)
        )

    def _build_candidate_markers(
        self,
        *,
        proposal: GraspProposal,
        frame_id: str,
        stamp,
        marker_base: int,
        is_selected: bool,
        list_index: int,
    ) -> list[Marker]:
        contact_1 = point_to_numpy(proposal.contact_point_1)
        contact_2 = point_to_numpy(proposal.contact_point_2)
        grasp_center = point_to_numpy(proposal.grasp_center)
        pregrasp = np.array(
            [
                float(proposal.pregrasp_pose.position.x),
                float(proposal.pregrasp_pose.position.y),
                float(proposal.pregrasp_pose.position.z),
            ],
            dtype=np.float32,
        )
        closing_direction = normalize_vector(vector3_to_numpy(proposal.closing_direction))
        approach_direction = normalize_vector(vector3_to_numpy(proposal.approach_direction))
        if not np.any(approach_direction):
            approach_direction = normalize_vector(grasp_center - pregrasp)

        alpha = 0.98 if is_selected else max(0.22, 0.55 - 0.08 * float(list_index))
        point_scale = 0.014 if is_selected else 0.010
        center_scale = 0.012 if is_selected else 0.008
        line_scale = 0.0045 if is_selected else 0.0030
        text_scale = 0.03 if is_selected else 0.022
        approach_length = 0.060 if is_selected else 0.045
        closing_length = max(0.025, min(0.060, float(proposal.grasp_width_m) * 0.5))

        markers = [
            self._make_sphere_marker(
                marker_id=marker_base,
                frame_id=frame_id,
                stamp=stamp,
                position=contact_1,
                scale=point_scale,
                color=make_color(1.00, 0.72, 0.72, alpha),
            ),
            self._make_sphere_marker(
                marker_id=marker_base + 1,
                frame_id=frame_id,
                stamp=stamp,
                position=contact_2,
                scale=point_scale,
                color=make_color(0.72, 0.83, 1.00, alpha),
            ),
            self._make_sphere_marker(
                marker_id=marker_base + 2,
                frame_id=frame_id,
                stamp=stamp,
                position=grasp_center,
                scale=center_scale,
                color=make_color(1.00, 0.92, 0.72, alpha),
            ),
            self._make_line_marker(
                marker_id=marker_base + 3,
                frame_id=frame_id,
                stamp=stamp,
                points=[contact_1, contact_2],
                scale=line_scale,
                color=make_color(0.94, 0.96, 1.00, alpha),
            ),
            self._make_arrow_marker(
                marker_id=marker_base + 4,
                frame_id=frame_id,
                stamp=stamp,
                start=pregrasp,
                end=grasp_center,
                shaft_diameter=0.0034 if is_selected else 0.0026,
                head_diameter=0.0062 if is_selected else 0.0048,
                head_length=0.010 if is_selected else 0.008,
                color=make_color(0.72, 1.00, 0.80, alpha),
            ),
            self._make_arrow_marker(
                marker_id=marker_base + 5,
                frame_id=frame_id,
                stamp=stamp,
                start=grasp_center - (closing_direction * closing_length),
                end=grasp_center + (closing_direction * closing_length),
                shaft_diameter=0.0026 if is_selected else 0.0022,
                head_diameter=0.0048 if is_selected else 0.0040,
                head_length=0.006 if is_selected else 0.005,
                color=make_color(0.98, 0.80, 0.60, alpha),
            ),
            self._make_text_marker(
                marker_id=marker_base + 6,
                frame_id=frame_id,
                stamp=stamp,
                position=grasp_center + np.array([0.0, 0.0, 0.05 + 0.012 * list_index], dtype=np.float32),
                scale=text_scale,
                text=(
                    f"#{int(proposal.candidate_rank or (list_index + 1))} "
                    f"score={proposal.confidence_score:.2f} "
                    f"width={proposal.grasp_width_m:.3f}m"
                ),
                color=make_color(0.95, 0.97, 1.00, 0.92 if is_selected else alpha),
            ),
        ]

        if np.any(approach_direction):
            markers.append(
                self._make_arrow_marker(
                    marker_id=marker_base + 7,
                    frame_id=frame_id,
                    stamp=stamp,
                    start=grasp_center,
                    end=grasp_center + (approach_direction * approach_length),
                    shaft_diameter=0.0024 if is_selected else 0.0020,
                    head_diameter=0.0048 if is_selected else 0.0040,
                    head_length=0.007 if is_selected else 0.006,
                    color=make_color(0.82, 0.98, 1.00, alpha),
                )
            )
        return markers

    def _make_sphere_marker(
        self,
        *,
        marker_id: int,
        frame_id: str,
        stamp,
        position: np.ndarray,
        scale: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._make_marker(marker_id, frame_id, stamp, Marker.SPHERE)
        marker.pose.position = make_point_xyz(position)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        return marker

    def _make_line_marker(
        self,
        *,
        marker_id: int,
        frame_id: str,
        stamp,
        points: list[np.ndarray],
        scale: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._make_marker(marker_id, frame_id, stamp, Marker.LINE_STRIP)
        marker.scale.x = scale
        marker.color = color
        marker.points = [make_point_xyz(point) for point in points]
        return marker

    def _make_arrow_marker(
        self,
        *,
        marker_id: int,
        frame_id: str,
        stamp,
        start: np.ndarray,
        end: np.ndarray,
        shaft_diameter: float,
        head_diameter: float,
        head_length: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._make_marker(marker_id, frame_id, stamp, Marker.ARROW)
        marker.scale.x = shaft_diameter
        marker.scale.y = head_diameter
        marker.scale.z = head_length
        marker.color = color
        marker.points = [make_point_xyz(start), make_point_xyz(end)]
        return marker

    def _make_text_marker(
        self,
        *,
        marker_id: int,
        frame_id: str,
        stamp,
        position: np.ndarray,
        scale: float,
        text: str,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._make_marker(marker_id, frame_id, stamp, Marker.TEXT_VIEW_FACING)
        marker.pose.position = make_point_xyz(position)
        marker.scale.z = scale
        marker.color = color
        marker.text = text
        return marker

    def _make_marker(self, marker_id: int, frame_id: str, stamp, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "grasp_backend"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish_debug(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = compact_json(payload)
        self.debug_pub.publish(msg)
        summary = self._format_terminal_summary(payload)
        if summary and summary != self._last_terminal_summary:
            self._last_terminal_summary = summary
            if payload.get("status") == "ok":
                self.get_logger().info(summary)
            else:
                self.get_logger().warn(summary)

    def _format_terminal_summary(self, payload: dict[str, Any]) -> str:
        status = str(payload.get("status") or "")
        if status == "ok":
            target_label = str(payload.get("target_label") or "").strip() or "<unset>"
            return (
                "grasp backend decision: "
                f"target={target_label} proposals={int(payload.get('proposal_count', 0))} "
                f"selected_index={int(payload.get('selected_index', 0))} "
                f"frame={str(payload.get('frame_id') or '<unknown>')}"
            )
        reason = str(payload.get("reason") or status or "unknown")
        return f"grasp backend decision: no grasp proposals ({reason})"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspBackendNode()
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
