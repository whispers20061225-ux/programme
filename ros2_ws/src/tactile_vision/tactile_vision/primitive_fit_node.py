from __future__ import annotations

import copy
import json
import math
import time
from collections import deque
from itertools import permutations
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tactile_interfaces.msg import DetectionResult, SemanticTask
from visualization_msgs.msg import Marker

from tactile_vision.modular_common import compact_json, make_xyz_cloud, voxel_downsample


def _normalize_vector(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    candidate = np.asarray(vector, dtype=np.float64).reshape((3,))
    norm = float(np.linalg.norm(candidate))
    if norm <= 1e-8:
        candidate = np.asarray(fallback, dtype=np.float64).reshape((3,))
        norm = float(np.linalg.norm(candidate))
        if norm <= 1e-8:
            return np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    return candidate / norm


def _rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        root = math.sqrt(trace + 1.0)
        w = 0.5 * root
        root = 0.5 / root
        x = (matrix[2, 1] - matrix[1, 2]) * root
        y = (matrix[0, 2] - matrix[2, 0]) * root
        z = (matrix[1, 0] - matrix[0, 1]) * root
        return np.asarray([x, y, z, w], dtype=np.float64)

    diagonal = np.diag(matrix)
    index = int(np.argmax(diagonal))
    if index == 0:
        root = math.sqrt(max(1e-12, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]))
        x = 0.5 * root
        root = 0.5 / root
        y = (matrix[0, 1] + matrix[1, 0]) * root
        z = (matrix[0, 2] + matrix[2, 0]) * root
        w = (matrix[2, 1] - matrix[1, 2]) * root
    elif index == 1:
        root = math.sqrt(max(1e-12, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]))
        y = 0.5 * root
        root = 0.5 / root
        x = (matrix[0, 1] + matrix[1, 0]) * root
        z = (matrix[1, 2] + matrix[2, 1]) * root
        w = (matrix[0, 2] - matrix[2, 0]) * root
    else:
        root = math.sqrt(max(1e-12, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]))
        z = 0.5 * root
        root = 0.5 / root
        x = (matrix[0, 2] + matrix[2, 0]) * root
        y = (matrix[1, 2] + matrix[2, 1]) * root
        w = (matrix[1, 0] - matrix[0, 1]) * root
    return np.asarray([x, y, z, w], dtype=np.float64)


class PrimitiveFitNode(Node):
    def __init__(self) -> None:
        super().__init__("primitive_fit_node")

        self.declare_parameter("target_cloud_topic", "/perception/target_cloud")
        self.declare_parameter("target_cloud_debug_topic", "/perception/target_cloud_debug")
        self.declare_parameter("detection_result_topic", "/perception/detection_result")
        self.declare_parameter("semantic_task_topic", "/qwen/semantic_task")
        self.declare_parameter("fitted_cloud_topic", "/perception/target_cloud_fitted")
        self.declare_parameter("fitted_marker_topic", "/perception/target_primitive_marker")
        self.declare_parameter("debug_topic", "/perception/target_fit_debug")
        self.declare_parameter("enabled", True)
        self.declare_parameter("publish_rate_hz", 4.0)
        self.declare_parameter("cloud_stale_sec", 4.0)
        self.declare_parameter("detection_stale_sec", 4.0)
        self.declare_parameter("history_hold_sec", 2.5)
        self.declare_parameter("history_max_frames", 8)
        self.declare_parameter("history_merge_distance_m", 0.05)
        self.declare_parameter("fit_min_points", 96)
        self.declare_parameter("output_voxel_size_m", 0.003)
        self.declare_parameter("cylinder_labels", ["cylinder", "can"])
        self.declare_parameter("sphere_labels", ["sphere", "ball"])
        self.declare_parameter("hemisphere_labels", ["hemisphere", "half sphere", "dome"])
        self.declare_parameter("frustum_labels", ["frustum", "truncated cone", "cone"])
        self.declare_parameter(
            "axis_profile_labels",
            ["bottle", "cup", "mug", "glass", "vase", "wine glass"],
        )
        self.declare_parameter("box_labels", ["cube", "box", "block", "container"])
        self.declare_parameter("cylinder_angle_samples", 56)
        self.declare_parameter("cylinder_height_samples", 24)
        self.declare_parameter("cylinder_height_hold_sec", 1.2)
        self.declare_parameter("cylinder_height_match_center_m", 0.025)
        self.declare_parameter("cylinder_height_match_radius_m", 0.008)
        self.declare_parameter("cylinder_height_match_axis_deg", 12.0)
        self.declare_parameter("cylinder_height_shrink_tolerance_m", 0.004)
        self.declare_parameter("profile_height_bins", 20)
        self.declare_parameter("profile_angle_samples", 56)
        self.declare_parameter("box_face_grid", 10)
        self.declare_parameter("box_extent_hold_sec", 1.0)
        self.declare_parameter("box_match_center_m", 0.03)
        self.declare_parameter("box_match_axis_deg", 12.0)
        self.declare_parameter("box_shrink_tolerance_m", 0.004)
        self.declare_parameter("marker_alpha", 0.35)
        self.declare_parameter("log_interval_sec", 8.0)

        self.target_cloud_topic = str(self.get_parameter("target_cloud_topic").value)
        self.target_cloud_debug_topic = str(self.get_parameter("target_cloud_debug_topic").value)
        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.semantic_task_topic = str(self.get_parameter("semantic_task_topic").value)
        self.fitted_cloud_topic = str(self.get_parameter("fitted_cloud_topic").value)
        self.fitted_marker_topic = str(self.get_parameter("fitted_marker_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.publish_rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.cloud_stale_sec = max(0.1, float(self.get_parameter("cloud_stale_sec").value))
        self.detection_stale_sec = max(
            0.1, float(self.get_parameter("detection_stale_sec").value)
        )
        self.history_hold_sec = max(0.1, float(self.get_parameter("history_hold_sec").value))
        self.history_max_frames = max(1, int(self.get_parameter("history_max_frames").value))
        self.history_merge_distance_m = max(
            0.0, float(self.get_parameter("history_merge_distance_m").value)
        )
        self.fit_min_points = max(16, int(self.get_parameter("fit_min_points").value))
        self.output_voxel_size_m = max(
            0.0, float(self.get_parameter("output_voxel_size_m").value)
        )
        self.cylinder_labels = self._normalize_label_list(
            self.get_parameter("cylinder_labels").value
        )
        self.sphere_labels = self._normalize_label_list(self.get_parameter("sphere_labels").value)
        self.hemisphere_labels = self._normalize_label_list(
            self.get_parameter("hemisphere_labels").value
        )
        self.frustum_labels = self._normalize_label_list(
            self.get_parameter("frustum_labels").value
        )
        self.axis_profile_labels = self._normalize_label_list(
            self.get_parameter("axis_profile_labels").value
        )
        self.box_labels = self._normalize_label_list(self.get_parameter("box_labels").value)
        self.cylinder_angle_samples = max(
            12, int(self.get_parameter("cylinder_angle_samples").value)
        )
        self.cylinder_height_samples = max(
            6, int(self.get_parameter("cylinder_height_samples").value)
        )
        self.cylinder_height_hold_sec = max(
            0.0, float(self.get_parameter("cylinder_height_hold_sec").value)
        )
        self.cylinder_height_match_center_m = max(
            0.0, float(self.get_parameter("cylinder_height_match_center_m").value)
        )
        self.cylinder_height_match_radius_m = max(
            0.0, float(self.get_parameter("cylinder_height_match_radius_m").value)
        )
        self.cylinder_height_match_axis_deg = max(
            1.0, float(self.get_parameter("cylinder_height_match_axis_deg").value)
        )
        self.cylinder_height_shrink_tolerance_m = max(
            0.0, float(self.get_parameter("cylinder_height_shrink_tolerance_m").value)
        )
        self.profile_height_bins = max(
            8, int(self.get_parameter("profile_height_bins").value)
        )
        self.profile_angle_samples = max(
            12, int(self.get_parameter("profile_angle_samples").value)
        )
        self.box_face_grid = max(3, int(self.get_parameter("box_face_grid").value))
        self.box_extent_hold_sec = max(
            0.0, float(self.get_parameter("box_extent_hold_sec").value)
        )
        self.box_match_center_m = max(
            0.0, float(self.get_parameter("box_match_center_m").value)
        )
        self.box_match_axis_deg = max(
            1.0, float(self.get_parameter("box_match_axis_deg").value)
        )
        self.box_shrink_tolerance_m = max(
            0.0, float(self.get_parameter("box_shrink_tolerance_m").value)
        )
        self.marker_alpha = max(0.05, min(1.0, float(self.get_parameter("marker_alpha").value)))
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._latest_detection: Optional[DetectionResult] = None
        self._latest_detection_ts = 0.0
        self._latest_semantic_target_label = ""
        self._latest_cloud_target_label = ""
        self._latest_cloud_msg: Optional[PointCloud2] = None
        self._latest_cloud_ts = 0.0
        self._history: deque[dict[str, Any]] = deque(maxlen=self.history_max_frames)
        self._last_history_key: Optional[tuple[int, int, int, str]] = None
        self._last_summary = ""
        self._last_log_time = 0.0
        self._last_cylinder_fit_state: dict[str, Any] = {}
        self._last_box_fit_state: dict[str, Any] = {}

        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            PointCloud2, self.target_cloud_topic, self._on_target_cloud, qos_reliable
        )
        self.create_subscription(String, self.target_cloud_debug_topic, self._on_target_cloud_debug, qos_reliable)
        self.create_subscription(
            DetectionResult, self.detection_result_topic, self._on_detection_result, qos_reliable
        )
        self.create_subscription(
            SemanticTask, self.semantic_task_topic, self._on_semantic_task, qos_reliable
        )
        self.fitted_cloud_pub = self.create_publisher(
            PointCloud2, self.fitted_cloud_topic, qos_latched
        )
        self.fitted_marker_pub = self.create_publisher(Marker, self.fitted_marker_topic, qos_latched)
        self.debug_pub = self.create_publisher(String, self.debug_topic, qos_reliable)
        self.create_timer(1.0 / self.publish_rate_hz, self._process_latest)

        self.get_logger().info(
            "primitive_fit_node started: "
            f"target_cloud={self.target_cloud_topic}, fitted_cloud={self.fitted_cloud_topic}, "
            f"fitted_marker={self.fitted_marker_topic}"
        )

    def _normalize_label_list(self, values: Any) -> list[str]:
        return [token for token in (self._normalize_label(value) for value in list(values or [])) if token]

    def _normalize_label(self, value: Any) -> str:
        return str(value or "").strip().lower()

    def _semantic_tokens(self, label: str) -> set[str]:
        text = self._normalize_label(label)
        if not text:
            return set()
        tokens = {
            token
            for token in "".join(ch if ch.isalnum() else " " for ch in text).split()
            if token
        }
        if text:
            tokens.add(text)
        return tokens

    def _label_matches(self, label: str, keywords: list[str]) -> bool:
        if not label or not keywords:
            return False
        tokens = self._semantic_tokens(label)
        for keyword in keywords:
            if keyword in tokens:
                return True
            keyword_tokens = self._semantic_tokens(keyword)
            if keyword_tokens and keyword_tokens.issubset(tokens):
                return True
        return False

    def _strategy_for_label(self, label: str) -> str:
        if self._label_matches(label, self.cylinder_labels):
            return "cylinder"
        if self._label_matches(label, self.sphere_labels):
            return "sphere"
        if self._label_matches(label, self.hemisphere_labels):
            return "hemisphere"
        if self._label_matches(label, self.frustum_labels):
            return "frustum"
        if self._label_matches(label, self.axis_profile_labels):
            return "axis_profile"
        if self._label_matches(label, self.box_labels):
            return "box"
        return "temporal_raw"

    def _on_detection_result(self, msg: DetectionResult) -> None:
        self._latest_detection = copy.deepcopy(msg)
        self._latest_detection_ts = time.time()

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        target_label = str(msg.target_label or msg.target_hint or "").strip()
        self._latest_semantic_target_label = self._normalize_label(target_label)

    def _on_target_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud_msg = msg
        self._latest_cloud_ts = time.time()

    def _on_target_cloud_debug(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data or "{}"))
        except Exception:  # noqa: BLE001
            return
        label = self._normalize_label(payload.get("target_label", ""))
        if label:
            self._latest_cloud_target_label = label

    def _read_xyz_points(self, cloud_msg: PointCloud2) -> np.ndarray:
        structured = point_cloud2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        points = np.asarray(list(structured))
        if points.size <= 0:
            return np.zeros((0, 3), dtype=np.float32)
        if getattr(points.dtype, "names", None):
            return np.column_stack([points["x"], points["y"], points["z"]]).astype(np.float32)
        return np.asarray(points, dtype=np.float32).reshape((-1, 3))

    def _append_history(
        self,
        *,
        points: np.ndarray,
        label: str,
        stamp_sec: float,
        cloud_key: tuple[int, int, int, str],
    ) -> None:
        if self._last_history_key == cloud_key:
            return
        centroid = np.mean(points, axis=0, dtype=np.float64).astype(np.float32)
        entry = {
            "timestamp": float(stamp_sec),
            "label": label,
            "points": np.asarray(points, dtype=np.float32).copy(),
            "centroid": centroid,
        }
        self._history.append(entry)
        self._last_history_key = cloud_key

    def _select_history_points(self, label: str, current_centroid: np.ndarray, now_sec: float) -> np.ndarray:
        if not self._history:
            return np.zeros((0, 3), dtype=np.float32)
        selected: list[np.ndarray] = []
        for entry in self._history:
            if (now_sec - float(entry["timestamp"])) > self.history_hold_sec:
                continue
            if label and self._normalize_label(entry.get("label")) != label:
                continue
            centroid = np.asarray(entry["centroid"], dtype=np.float32).reshape((3,))
            if (
                self.history_merge_distance_m > 0.0
                and float(np.linalg.norm(centroid - current_centroid)) > self.history_merge_distance_m
            ):
                continue
            selected.append(np.asarray(entry["points"], dtype=np.float32).reshape((-1, 3)))
        if not selected:
            return np.zeros((0, 3), dtype=np.float32)
        merged = np.concatenate(selected, axis=0)
        return voxel_downsample(merged, self.output_voxel_size_m)

    def _principal_axis_frame(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        center = np.mean(pts, axis=0)
        centered = pts - center
        covariance = centered.T @ centered / float(max(1, pts.shape[0] - 1))
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        axis = _normalize_vector(eigenvectors[:, order[0]], np.asarray([0.0, 0.0, 1.0]))
        if axis[2] < 0.0:
            axis = -axis
        reference = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(reference, axis))) > 0.92:
            reference = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        lateral_u = _normalize_vector(np.cross(axis, reference), np.asarray([1.0, 0.0, 0.0]))
        lateral_v = _normalize_vector(np.cross(axis, lateral_u), np.asarray([0.0, 1.0, 0.0]))
        basis = np.stack([lateral_u, lateral_v, axis], axis=1)
        local = centered @ basis
        return center.astype(np.float64), basis.astype(np.float64), local.astype(np.float64)

    def _basis_from_axis(self, axis: np.ndarray) -> np.ndarray:
        direction = _normalize_vector(axis, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
        reference = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(direction, reference))) > 0.92:
            reference = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        lateral_u = _normalize_vector(
            np.cross(direction, reference), np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        )
        lateral_v = _normalize_vector(
            np.cross(direction, lateral_u), np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        )
        return np.stack([lateral_u, lateral_v, direction], axis=1).astype(np.float64)

    def _local_surface_geometry(self, points: np.ndarray) -> dict[str, np.ndarray]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        count = int(pts.shape[0])
        normals = np.zeros((count, 3), dtype=np.float64)
        planarity = np.zeros((count,), dtype=np.float64)
        linearity = np.zeros((count,), dtype=np.float64)
        curvature = np.zeros((count,), dtype=np.float64)
        neighbor_mean_dist = np.zeros((count,), dtype=np.float64)
        if count <= 4:
            normals[:, 2] = 1.0
            density_quality = np.ones((count,), dtype=np.float64)
            return {
                "normals": normals,
                "planarity": planarity,
                "linearity": linearity,
                "curvature": curvature,
                "neighbor_mean_dist": neighbor_mean_dist,
                "density_quality": density_quality,
            }

        global_center = np.mean(pts, axis=0)
        diffs = pts[:, None, :] - pts[None, :, :]
        distances_sq = np.sum(diffs * diffs, axis=2)
        neighbor_count = min(max(10, int(round(math.sqrt(count))) + 4), max(3, count - 1))

        for idx in range(count):
            nearest = np.argpartition(distances_sq[idx], neighbor_count + 1)[: neighbor_count + 1]
            nearest = nearest[nearest != idx][:neighbor_count]
            if nearest.size < 3:
                normals[idx] = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
                continue
            neighbor_points = pts[nearest]
            neighbor_mean_dist[idx] = float(
                np.mean(np.sqrt(np.maximum(0.0, distances_sq[idx, nearest])))
            )
            centered = neighbor_points - np.mean(neighbor_points, axis=0, keepdims=True)
            covariance = centered.T @ centered / float(max(1, centered.shape[0] - 1))
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            order = np.argsort(eigenvalues)
            eigenvalues = np.maximum(eigenvalues[order], 1e-12)
            eigenvectors = eigenvectors[:, order]
            normal = eigenvectors[:, 0]
            outward = pts[idx] - global_center
            if float(np.dot(normal, outward)) < 0.0:
                normal = -normal
            normals[idx] = _normalize_vector(normal, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
            lam0, lam1, lam2 = eigenvalues.tolist()
            denom = max(lam0 + lam1 + lam2, 1e-12)
            curvature[idx] = lam0 / denom
            planarity[idx] = max(0.0, min(1.0, (lam1 - lam0) / max(lam2, 1e-12)))
            linearity[idx] = max(0.0, min(1.0, (lam2 - lam1) / max(lam2, 1e-12)))

        dist_med = float(np.median(neighbor_mean_dist))
        dist_mad = float(np.median(np.abs(neighbor_mean_dist - dist_med))) + 1e-6
        curvature_med = float(np.median(curvature))
        curvature_mad = float(np.median(np.abs(curvature - curvature_med))) + 1e-6

        density_penalty = np.maximum(0.0, (neighbor_mean_dist - dist_med) / (2.5 * dist_mad))
        density_quality = 1.0 / (1.0 + density_penalty * density_penalty)
        curvature_penalty = np.maximum(0.0, (curvature - curvature_med) / (3.0 * curvature_mad))
        planarity = np.clip(planarity * (1.0 / (1.0 + curvature_penalty)), 0.0, 1.0)

        return {
            "normals": normals,
            "planarity": planarity,
            "linearity": linearity,
            "curvature": curvature,
            "neighbor_mean_dist": neighbor_mean_dist,
            "density_quality": density_quality,
        }

    def _weighted_circle_fit_2d(
        self,
        points_2d: np.ndarray,
        weights: np.ndarray,
        fallback_center: np.ndarray,
        fallback_radius: float,
    ) -> tuple[np.ndarray, float]:
        pts = np.asarray(points_2d, dtype=np.float64).reshape((-1, 2))
        w = np.asarray(weights, dtype=np.float64).reshape((-1,))
        valid = np.isfinite(pts).all(axis=1) & np.isfinite(w) & (w > 1e-8)
        pts = pts[valid]
        w = w[valid]
        if pts.shape[0] < 6:
            return np.asarray(fallback_center, dtype=np.float64).reshape((2,)), float(
                max(1e-3, fallback_radius)
            )
        system = np.column_stack([pts[:, 0], pts[:, 1], np.ones((pts.shape[0],), dtype=np.float64)])
        rhs = -(pts[:, 0] * pts[:, 0] + pts[:, 1] * pts[:, 1])
        sqrt_w = np.sqrt(np.maximum(w, 1e-8))
        system_w = system * sqrt_w.reshape((-1, 1))
        rhs_w = rhs * sqrt_w
        try:
            solution, _, _, _ = np.linalg.lstsq(system_w, rhs_w, rcond=None)
        except np.linalg.LinAlgError:
            return np.asarray(fallback_center, dtype=np.float64).reshape((2,)), float(
                max(1e-3, fallback_radius)
            )
        center = -0.5 * solution[:2]
        radius_sq = float(np.dot(center, center) - solution[2])
        if not np.isfinite(radius_sq) or radius_sq <= 1e-8:
            return np.asarray(fallback_center, dtype=np.float64).reshape((2,)), float(
                max(1e-3, fallback_radius)
            )
        radius = math.sqrt(radius_sq)
        return center.astype(np.float64), float(max(1e-3, radius))

    def _angular_outer_samples(
        self,
        points_2d: np.ndarray,
        center_2d: np.ndarray,
        support_weight: np.ndarray,
        *,
        angle_bin_count: int,
        min_relative_weight: float = 0.2,
        min_absolute_weight: float = 0.08,
        radial_keep_percentile: float = 75.0,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pts = np.asarray(points_2d, dtype=np.float64).reshape((-1, 2))
        center = np.asarray(center_2d, dtype=np.float64).reshape((2,))
        weights = np.asarray(support_weight, dtype=np.float64).reshape((-1,))
        if pts.shape[0] < 8:
            return (
                np.zeros((0, 2), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.int32),
            )

        offsets = pts - center.reshape((1, 2))
        radial = np.linalg.norm(offsets, axis=1)
        max_weight = float(np.max(weights)) if weights.size else 0.0
        if max_weight <= 1e-8:
            return (
                np.zeros((0, 2), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.int32),
            )
        support_mask = weights >= max(min_absolute_weight, min_relative_weight * max_weight)
        if int(np.count_nonzero(support_mask)) < 8:
            support_mask = weights >= max(min_absolute_weight * 0.5, 0.1 * max_weight)
        pts_sel = pts[support_mask]
        offsets_sel = offsets[support_mask]
        radial_sel = radial[support_mask]
        weights_sel = weights[support_mask]
        if pts_sel.shape[0] < 8:
            return (
                np.zeros((0, 2), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.int32),
            )

        angles = np.arctan2(offsets_sel[:, 1], offsets_sel[:, 0])
        normalized = (angles + math.pi) / (2.0 * math.pi)
        bins = np.clip((normalized * angle_bin_count).astype(np.int32), 0, angle_bin_count - 1)

        boundary_points: list[np.ndarray] = []
        boundary_weights: list[float] = []
        boundary_radial: list[float] = []
        boundary_bins: list[int] = []
        for bin_index in range(angle_bin_count):
            mask = bins == bin_index
            if not np.any(mask):
                continue
            radial_bin = radial_sel[mask]
            weights_bin = weights_sel[mask]
            pts_bin = pts_sel[mask]
            keep_threshold = float(np.percentile(radial_bin, radial_keep_percentile))
            outer_mask = radial_bin >= keep_threshold
            if not np.any(outer_mask):
                outer_mask = np.ones_like(radial_bin, dtype=bool)
            pts_outer = pts_bin[outer_mask]
            weights_outer = weights_bin[outer_mask]
            radial_outer = radial_bin[outer_mask]
            best_index = int(np.argmax(radial_outer + 0.2 * radial_outer * weights_outer))
            boundary_points.append(pts_outer[best_index])
            boundary_weights.append(float(max(1e-4, weights_outer[best_index])))
            boundary_radial.append(float(radial_outer[best_index]))
            boundary_bins.append(int(bin_index))

        if not boundary_points:
            return (
                np.zeros((0, 2), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.float64),
                np.zeros((0,), dtype=np.int32),
            )
        return (
            np.asarray(boundary_points, dtype=np.float64).reshape((-1, 2)),
            np.asarray(boundary_weights, dtype=np.float64).reshape((-1,)),
            np.asarray(boundary_radial, dtype=np.float64).reshape((-1,)),
            np.asarray(boundary_bins, dtype=np.int32).reshape((-1,)),
        )

    def _fixed_center_radius_from_samples(
        self,
        radial_samples: np.ndarray,
        *,
        radius_prior: float,
        lower_quantile: float,
        upper_quantile: float,
    ) -> float:
        radial = np.asarray(radial_samples, dtype=np.float64).reshape((-1,))
        radial = radial[np.isfinite(radial) & (radial > 1e-6)]
        if radial.size <= 0:
            return float(max(1e-3, radius_prior))
        lower_value = float(np.percentile(radial, lower_quantile))
        upper_value = float(np.percentile(radial, upper_quantile))
        radius = max(float(radius_prior), lower_value)
        radius = min(radius, max(radius, upper_value + self.output_voxel_size_m * 2.0))
        return float(max(1e-3, radius))

    def _estimate_cap_fixed_center(
        self,
        points_2d: np.ndarray,
        axial: np.ndarray,
        cap_weight: np.ndarray,
        center_2d: np.ndarray,
        *,
        radius_prior: float,
        h_reference: float,
        upper: bool,
        angle_bin_count: int,
    ) -> dict[str, Any]:
        pts = np.asarray(points_2d, dtype=np.float64).reshape((-1, 2))
        axial_values = np.asarray(axial, dtype=np.float64).reshape((-1,))
        cap_values = np.asarray(cap_weight, dtype=np.float64).reshape((-1,))
        center = np.asarray(center_2d, dtype=np.float64).reshape((2,))
        if pts.shape[0] < 8 or cap_values.size != pts.shape[0]:
            return {
                "visible": False,
                "radius": float(max(1e-3, radius_prior)),
                "plane": float(h_reference),
                "sample_count": 0,
                "mass": 0.0,
            }

        radial = np.linalg.norm(pts - center.reshape((1, 2)), axis=1)
        slab_half = max(self.output_voxel_size_m * 3.0, 0.18 * float(radius_prior), 0.004)
        axial_delta = np.abs(axial_values - float(h_reference))
        slab_gate = np.exp(-0.5 * np.square(axial_delta / slab_half))
        rim_gate = np.clip(
            (radial - 0.45 * float(radius_prior)) / max(0.18 * float(radius_prior), self.output_voxel_size_m),
            0.0,
            1.0,
        )
        support = np.clip(cap_values * slab_gate * (0.2 + 0.8 * rim_gate), 0.0, 1.0)
        boundary_points, _, boundary_radial, _ = self._angular_outer_samples(
            pts,
            center,
            support,
            angle_bin_count=angle_bin_count,
            min_relative_weight=0.15,
            min_absolute_weight=0.05,
            radial_keep_percentile=68.0,
        )
        if boundary_points.shape[0] < 6:
            return {
                "visible": False,
                "radius": float(max(1e-3, radius_prior)),
                "plane": float(h_reference),
                "sample_count": int(boundary_points.shape[0]),
                "mass": float(np.sum(support)),
            }

        boundary_axial = np.zeros((boundary_points.shape[0],), dtype=np.float64)
        for index, point_2d in enumerate(boundary_points):
            delta = pts - point_2d.reshape((1, 2))
            match_index = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
            boundary_axial[index] = axial_values[match_index]

        plane = (
            float(np.percentile(boundary_axial, 85.0))
            if upper
            else float(np.percentile(boundary_axial, 15.0))
        )
        radius = self._fixed_center_radius_from_samples(
            boundary_radial,
            radius_prior=float(radius_prior),
            lower_quantile=65.0,
            upper_quantile=90.0,
        )
        return {
            "visible": True,
            "radius": float(radius),
            "plane": float(plane),
            "sample_count": int(boundary_points.shape[0]),
            "mass": float(np.sum(support)),
        }

    def _extend_cylinder_height_tails(
        self,
        axial: np.ndarray,
        theta: np.ndarray,
        side_weight: np.ndarray,
        radial_residual: np.ndarray,
        *,
        h_min: float,
        h_max: float,
    ) -> tuple[float, float, dict[str, Any]]:
        axial_values = np.asarray(axial, dtype=np.float64).reshape((-1,))
        theta_values = np.asarray(theta, dtype=np.float64).reshape((-1,))
        side_values = np.asarray(side_weight, dtype=np.float64).reshape((-1,))
        radial_values = np.asarray(radial_residual, dtype=np.float64).reshape((-1,))
        if axial_values.size <= 0:
            return float(h_min), float(h_max), {"tail_lower_extended_m": 0.0, "tail_upper_extended_m": 0.0}

        side_peak = float(np.max(side_values)) if side_values.size else 0.0
        core_mask = side_values >= max(0.2, 0.35 * side_peak)
        if np.count_nonzero(core_mask) >= 2:
            core_sorted = np.sort(axial_values[core_mask])
            core_diffs = np.diff(core_sorted)
            core_diffs = core_diffs[core_diffs > 1e-6]
            typical_gap = (
                float(np.median(core_diffs))
                if core_diffs.size > 0
                else max(self.output_voxel_size_m * 2.0, 0.002)
            )
        else:
            typical_gap = max(self.output_voxel_size_m * 2.0, 0.002)
        gap_limit = max(self.output_voxel_size_m * 4.0, 3.0 * typical_gap)
        if np.count_nonzero(core_mask) > 0:
            residual_limit = max(
                self.output_voxel_size_m * 3.0,
                2.0 * float(np.percentile(radial_values[core_mask], 75.0)),
            )
        else:
            residual_limit = max(
                self.output_voxel_size_m * 3.0,
                2.0 * float(np.percentile(radial_values, 75.0)),
            )
        tail_support_threshold = max(0.05, 0.12 * side_peak)

        sector_count = int(max(12, min(28, self.cylinder_angle_samples // 2)))
        theta_normalized = (theta_values + math.pi) / (2.0 * math.pi)
        theta_bins = np.clip((theta_normalized * sector_count).astype(np.int32), 0, sector_count - 1)

        def extend_in_sector(values: np.ndarray, start_value: float, ascending: bool) -> tuple[float, int]:
            ordered = np.sort(values) if ascending else np.sort(values)[::-1]
            current = float(start_value)
            extended = float(start_value)
            count = 0
            for value in ordered:
                gap = (value - current) if ascending else (current - value)
                if gap <= gap_limit:
                    extended = float(value)
                    current = float(value)
                    count += 1
                else:
                    break
            return extended, count

        def pick_sector_extension(base_mask: np.ndarray, start_value: float, ascending: bool) -> tuple[float, int, int]:
            sector_extensions: list[dict[str, Any]] = []
            for sector_index in range(sector_count):
                sector_mask = base_mask & (theta_bins == sector_index)
                if not np.any(sector_mask):
                    continue
                extension_value, count = extend_in_sector(
                    axial_values[sector_mask], start_value, ascending=ascending
                )
                if count <= 0:
                    continue
                sector_extensions.append(
                    {
                        "sector": int(sector_index),
                        "value": float(extension_value),
                        "count": int(count),
                    }
                )
            if not sector_extensions:
                return float(start_value), 0, 0

            sector_by_index = {item["sector"]: item for item in sector_extensions}
            visited: set[int] = set()
            best_value = float(start_value)
            best_sector_span = 0
            best_total_points = 0
            for item in sector_extensions:
                sector_index = int(item["sector"])
                if sector_index in visited:
                    continue
                run_indices = [sector_index]
                visited.add(sector_index)
                next_index = (sector_index + 1) % sector_count
                while next_index in sector_by_index and next_index not in visited:
                    run_indices.append(next_index)
                    visited.add(next_index)
                    next_index = (next_index + 1) % sector_count
                prev_index = (sector_index - 1 + sector_count) % sector_count
                while prev_index in sector_by_index and prev_index not in visited:
                    run_indices.insert(0, prev_index)
                    visited.add(prev_index)
                    prev_index = (prev_index - 1 + sector_count) % sector_count

                run_items = [sector_by_index[index] for index in run_indices]
                run_span = len(run_items)
                run_points = int(sum(int(entry["count"]) for entry in run_items))
                if run_span < 2 and run_points < 4:
                    continue
                candidate_value = (
                    float(min(float(entry["value"]) for entry in run_items))
                    if not ascending
                    else float(max(float(entry["value"]) for entry in run_items))
                )
                if (run_span > best_sector_span) or (
                    run_span == best_sector_span and run_points > best_total_points
                ):
                    best_sector_span = run_span
                    best_total_points = run_points
                    best_value = candidate_value
            return best_value, best_sector_span, best_total_points

        lower_mask = (
            (axial_values < h_min)
            & (side_values >= tail_support_threshold)
            & (radial_values <= residual_limit)
        )
        upper_mask = (
            (axial_values > h_max)
            & (side_values >= tail_support_threshold)
            & (radial_values <= residual_limit)
        )
        new_h_min, lower_sector_span, lower_tail_points = (
            pick_sector_extension(lower_mask, h_min, ascending=False)
            if np.any(lower_mask)
            else (float(h_min), 0, 0)
        )
        new_h_max, upper_sector_span, upper_tail_points = (
            pick_sector_extension(upper_mask, h_max, ascending=True)
            if np.any(upper_mask)
            else (float(h_max), 0, 0)
        )
        return new_h_min, new_h_max, {
            "tail_lower_extended_m": round(float(h_min - new_h_min), 4),
            "tail_upper_extended_m": round(float(new_h_max - h_max), 4),
            "tail_gap_limit_m": round(float(gap_limit), 4),
            "tail_residual_limit_m": round(float(residual_limit), 4),
            "tail_lower_sector_span": int(lower_sector_span),
            "tail_upper_sector_span": int(upper_sector_span),
            "tail_lower_points": int(lower_tail_points),
            "tail_upper_points": int(upper_tail_points),
        }

    def _candidate_plane_axis(
        self,
        normals: np.ndarray,
        planarity: np.ndarray,
        curvature: np.ndarray,
    ) -> Optional[np.ndarray]:
        if normals.shape[0] < 12:
            return None
        planar_quality = np.clip(planarity, 0.0, 1.0) * np.clip(
            1.0 - (curvature / max(float(np.percentile(curvature, 90.0)), 1e-6)), 0.0, 1.0
        )
        candidate_indices = np.argsort(planar_quality)[-min(24, planar_quality.shape[0]) :]
        candidate_indices = candidate_indices[planar_quality[candidate_indices] > 0.18]
        if candidate_indices.size < 6:
            return None
        support_threshold = math.cos(math.radians(20.0))
        best_support = 0.0
        best_axis: Optional[np.ndarray] = None
        normals_subset = normals[candidate_indices]
        quality_subset = planar_quality[candidate_indices]
        for seed in normals_subset:
            support = float(np.sum(quality_subset[np.abs(normals_subset @ seed) >= support_threshold]))
            if support > best_support:
                best_support = support
                best_axis = np.asarray(seed, dtype=np.float64).reshape((3,))
        if best_axis is None or best_support < 1.0:
            return None
        return _normalize_vector(best_axis, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))

    def _fit_cylinder_model_from_axis(
        self,
        points: np.ndarray,
        geometry: dict[str, np.ndarray],
        axis_init: np.ndarray,
        *,
        source: str,
    ) -> dict[str, Any]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        normals = np.asarray(geometry["normals"], dtype=np.float64).reshape((-1, 3))
        planarity = np.asarray(geometry["planarity"], dtype=np.float64).reshape((-1,))
        density_quality = np.asarray(geometry["density_quality"], dtype=np.float64).reshape((-1,))
        base_quality = np.clip(0.25 + 0.75 * planarity, 0.05, 1.0) * np.clip(
            density_quality, 0.05, 1.0
        )

        def evaluate_axis(axis_value: np.ndarray) -> dict[str, Any]:
            axis_current = _normalize_vector(
                axis_value, np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
            )
            basis = self._basis_from_axis(axis_current)
            projected = pts @ basis[:, :2]
            center_guess = np.mean(pts, axis=0)
            projected_center_guess = center_guess @ basis[:, :2]
            abs_normal_axis = np.abs(normals @ axis_current)

            side_seed = np.clip(
                base_quality * np.exp(-0.5 * np.square(abs_normal_axis / 0.28)),
                0.0,
                1.0,
            )
            seed_mask = side_seed >= max(0.08, 0.2 * float(np.max(side_seed)))
            if not np.any(seed_mask):
                seed_mask = np.ones((pts.shape[0],), dtype=bool)
            projected_radius_guess = float(
                max(
                    1e-3,
                    np.percentile(
                        np.linalg.norm(
                            projected[seed_mask] - projected_center_guess.reshape((1, 2)), axis=1
                        ),
                        70.0,
                    ),
                )
            )
            circle_center_2d, _ = self._weighted_circle_fit_2d(
                projected,
                side_seed,
                projected_center_guess,
                projected_radius_guess,
            )
            side_boundary_points, side_boundary_weights, _, _ = self._angular_outer_samples(
                projected,
                circle_center_2d,
                side_seed,
                angle_bin_count=max(32, min(72, self.cylinder_angle_samples)),
                min_relative_weight=0.18,
                min_absolute_weight=0.06,
                radial_keep_percentile=72.0,
            )
            if side_boundary_points.shape[0] >= 8:
                circle_center_2d, _ = self._weighted_circle_fit_2d(
                    side_boundary_points,
                    side_boundary_weights,
                    circle_center_2d,
                    projected_radius_guess,
                )
            side_boundary_points, side_boundary_weights, side_boundary_radial, _ = self._angular_outer_samples(
                projected,
                circle_center_2d,
                side_seed,
                angle_bin_count=max(32, min(72, self.cylinder_angle_samples)),
                min_relative_weight=0.16,
                min_absolute_weight=0.05,
                radial_keep_percentile=68.0,
            )
            side_radius = self._fixed_center_radius_from_samples(
                side_boundary_radial,
                radius_prior=projected_radius_guess,
                lower_quantile=58.0,
                upper_quantile=88.0,
            )

            centerline_anchor = basis[:, :2] @ circle_center_2d
            axial = (pts - centerline_anchor.reshape((1, 3))) @ axis_current
            radial_vectors = pts - centerline_anchor.reshape((1, 3)) - np.outer(axial, axis_current)
            radial = np.linalg.norm(radial_vectors, axis=1)
            theta = np.arctan2(projected[:, 1] - circle_center_2d[1], projected[:, 0] - circle_center_2d[0])
            radial_residual = np.abs(radial - side_radius)

            radial_scale = max(
                self.output_voxel_size_m * 2.0,
                float(np.percentile(radial_residual, 55.0)) * 1.6,
                float(side_radius) * 0.12,
                1e-3,
            )
            side_geom = np.exp(-0.5 * np.square(radial_residual / radial_scale))
            side_normal = np.exp(-0.5 * np.square(abs_normal_axis / 0.35))
            side_weight = np.clip(base_quality * side_geom * side_normal, 0.0, 1.0)

            cap_margin = max(self.output_voxel_size_m * 2.0, float(side_radius) * 0.18, 1e-3)
            cap_geom = np.exp(-0.5 * np.square(np.maximum(0.0, radial - side_radius) / cap_margin))
            cap_normal = np.exp(-0.5 * np.square((1.0 - abs_normal_axis) / 0.22))
            cap_weight = np.clip(base_quality * cap_geom * cap_normal, 0.0, 1.0)

            support_weight = np.maximum(side_weight, 0.85 * cap_weight)
            if not np.any(support_weight > 1e-5):
                support_weight = np.clip(base_quality, 0.0, 1.0)

            bin_count = int(max(16, min(40, round(math.sqrt(float(pts.shape[0]))) * 2)))
            axial_min = float(np.min(axial))
            axial_max = float(np.max(axial))
            if (axial_max - axial_min) <= 1e-5:
                axial_min -= 0.001
                axial_max += 0.001
            bin_edges = np.linspace(axial_min, axial_max, num=bin_count + 1, endpoint=True)
            hist = np.zeros((bin_count,), dtype=np.float64)
            bin_index = np.clip(np.digitize(axial, bin_edges) - 1, 0, bin_count - 1)
            np.add.at(hist, bin_index, support_weight)
            if hist.size >= 3:
                hist = np.convolve(hist, np.asarray([0.25, 0.5, 0.25]), mode="same")
            nonzero_hist = hist[hist > 1e-8]
            if nonzero_hist.size > 0:
                threshold = max(
                    0.05 * float(np.max(hist)),
                    0.35 * float(np.percentile(nonzero_hist, 20.0)),
                )
                active = hist >= threshold
                if active.size >= 3:
                    active = active | (np.roll(active, 1) & np.roll(active, -1))
                if np.any(active):
                    first_active = int(np.argmax(active))
                    last_active = int(active.size - 1 - np.argmax(active[::-1]))
                    h_min = float(bin_edges[first_active])
                    h_max = float(bin_edges[last_active + 1])
                else:
                    mask = support_weight >= max(0.12, 0.2 * float(np.max(support_weight)))
                    h_min = float(np.min(axial[mask])) if np.any(mask) else axial_min
                    h_max = float(np.max(axial[mask])) if np.any(mask) else axial_max
            else:
                h_min = axial_min
                h_max = axial_max

            upper_cap = self._estimate_cap_fixed_center(
                projected,
                axial,
                cap_weight,
                circle_center_2d,
                radius_prior=side_radius,
                h_reference=h_max,
                upper=True,
                angle_bin_count=max(20, min(48, self.cylinder_angle_samples)),
            )
            lower_cap = self._estimate_cap_fixed_center(
                projected,
                axial,
                cap_weight,
                circle_center_2d,
                radius_prior=side_radius,
                h_reference=h_min,
                upper=False,
                angle_bin_count=max(20, min(48, self.cylinder_angle_samples)),
            )
            radius_candidates = [float(side_radius)]
            if bool(upper_cap["visible"]):
                radius_candidates.append(float(upper_cap["radius"]))
                h_max = max(h_max, float(upper_cap["plane"]))
            if bool(lower_cap["visible"]):
                radius_candidates.append(float(lower_cap["radius"]))
                h_min = min(h_min, float(lower_cap["plane"]))
            final_radius = float(max(radius_candidates))

            radial_residual = np.abs(radial - final_radius)
            radial_scale = max(
                self.output_voxel_size_m * 2.0,
                float(np.percentile(radial_residual, 55.0)) * 1.6,
                float(final_radius) * 0.12,
                1e-3,
            )
            side_geom = np.exp(-0.5 * np.square(radial_residual / radial_scale))
            side_normal = np.exp(-0.5 * np.square(abs_normal_axis / 0.35))
            side_weight = np.clip(base_quality * side_geom * side_normal, 0.0, 1.0)
            cap_margin = max(self.output_voxel_size_m * 2.0, float(final_radius) * 0.18, 1e-3)
            cap_geom = np.exp(-0.5 * np.square(np.maximum(0.0, radial - final_radius) / cap_margin))
            cap_normal = np.exp(-0.5 * np.square((1.0 - abs_normal_axis) / 0.22))
            cap_weight = np.clip(base_quality * cap_geom * cap_normal, 0.0, 1.0)
            support_weight = np.maximum(side_weight, 0.85 * cap_weight)
            if not np.any(support_weight > 1e-5):
                support_weight = np.clip(base_quality, 0.0, 1.0)

            h_min, h_max, tail_debug = self._extend_cylinder_height_tails(
                axial,
                theta,
                side_weight,
                radial_residual,
                h_min=h_min,
                h_max=h_max,
            )
            if h_max < h_min:
                h_min, h_max = h_max, h_min

            axial_mid = 0.5 * (h_min + h_max)
            center_world = centerline_anchor + axis_current * axial_mid
            height = max(self.output_voxel_size_m, h_max - h_min)

            side_mask = side_weight >= max(0.2, 0.35 * float(np.max(side_weight)))
            cap_mask = cap_weight >= max(0.2, 0.35 * float(np.max(cap_weight)))
            radial_error = float(
                np.median(radial_residual[side_mask]) if np.any(side_mask) else np.median(radial_residual)
            )
            side_normal_error = float(
                np.mean(abs_normal_axis[side_mask]) if np.any(side_mask) else np.mean(abs_normal_axis)
            )
            cap_normal_error = float(
                np.mean(1.0 - abs_normal_axis[cap_mask])
                if np.any(cap_mask)
                else np.mean(1.0 - abs_normal_axis)
            )
            coverage = float(np.count_nonzero(hist > 1e-8)) / float(max(1, hist.size))
            support_mean = float(np.mean(support_weight))
            score = (
                2.0 * float(np.mean(side_weight))
                + 0.55 * float(np.mean(cap_weight))
                + 0.25 * coverage
                + 0.2 * support_mean
                - 2.2 * (radial_error / max(float(final_radius), self.output_voxel_size_m * 1.5))
                - 0.7 * side_normal_error
                - 0.2 * cap_normal_error
            )

            lower_cap_mass = float(np.sum(cap_weight[axial <= axial_mid]))
            upper_cap_mass = float(np.sum(cap_weight[axial >= axial_mid]))
            model = {
                "axis": axis_current.copy(),
                "basis": self._basis_from_axis(axis_current),
                "centerline_anchor": centerline_anchor.copy(),
                "center_world": center_world.copy(),
                "radius": float(max(self.output_voxel_size_m, final_radius)),
                "side_radius": float(max(self.output_voxel_size_m, side_radius)),
                "upper_cap_radius": float(upper_cap["radius"]),
                "lower_cap_radius": float(lower_cap["radius"]),
                "upper_cap_plane": float(upper_cap["plane"]),
                "lower_cap_plane": float(lower_cap["plane"]),
                "h_min": float(h_min),
                "h_max": float(h_max),
                "height": float(height),
                "side_weight": side_weight,
                "cap_weight": cap_weight,
                "support_weight": support_weight,
                "radial_residual": radial_residual,
                "abs_normal_axis": abs_normal_axis,
                "theta": theta,
                "score": float(score),
                "side_support_mean": float(np.mean(side_weight)),
                "cap_support_mean": float(np.mean(cap_weight)),
                "support_coverage": float(coverage),
                "radial_error": float(radial_error),
                "side_normal_error": float(side_normal_error),
                "cap_normal_error": float(cap_normal_error),
                "lower_cap_mass": float(lower_cap_mass),
                "upper_cap_mass": float(upper_cap_mass),
                "boundary_sample_count": int(side_boundary_points.shape[0]),
                "upper_cap_sample_count": int(upper_cap["sample_count"]),
                "lower_cap_sample_count": int(lower_cap["sample_count"]),
                "source": source,
            }
            model.update(tail_debug)
            return model

        axis = _normalize_vector(axis_init, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
        for _ in range(3):
            provisional_model = evaluate_axis(axis)
            side_weight = np.asarray(provisional_model["side_weight"], dtype=np.float64).reshape((-1,))
            cap_weight = np.asarray(provisional_model["cap_weight"], dtype=np.float64).reshape((-1,))
            side_mass = float(np.sum(side_weight)) + 1e-8
            cap_mass = float(np.sum(cap_weight)) + 1e-8
            refined_axis = np.asarray(provisional_model["axis"], dtype=np.float64).reshape((3,))
            if side_mass > 1e-4:
                side_matrix = (normals * side_weight.reshape((-1, 1))).T @ normals
                _, side_evecs = np.linalg.eigh(side_matrix)
                refined_axis = side_evecs[:, 0]
                if float(np.dot(refined_axis, axis)) < 0.0:
                    refined_axis = -refined_axis
            if cap_mass > 1e-4:
                cap_matrix = (normals * cap_weight.reshape((-1, 1))).T @ normals
                _, cap_evecs = np.linalg.eigh(cap_matrix)
                cap_axis = cap_evecs[:, 2]
                if float(np.dot(cap_axis, refined_axis)) < 0.0:
                    cap_axis = -cap_axis
                blend_side = side_mass / (side_mass + cap_mass)
                blend_cap = 1.0 - blend_side
                refined_axis = _normalize_vector(
                    (blend_side * refined_axis) + (blend_cap * cap_axis), refined_axis
                )
            if abs(float(np.dot(refined_axis, axis))) >= 0.999:
                axis = refined_axis
                break
            axis = refined_axis

        return evaluate_axis(axis)

    def _estimate_cylinder_v1(self, points: np.ndarray) -> dict[str, Any]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        geometry = self._local_surface_geometry(pts)
        center, _, _ = self._principal_axis_frame(pts)
        covariance = (pts - center).T @ (pts - center) / float(max(1, pts.shape[0] - 1))
        evals, evecs = np.linalg.eigh(covariance)
        pca_axis = evecs[:, int(np.argmax(evals))]
        if pca_axis[2] < 0.0:
            pca_axis = -pca_axis

        normals = np.asarray(geometry["normals"], dtype=np.float64)
        planarity = np.asarray(geometry["planarity"], dtype=np.float64)
        density_quality = np.asarray(geometry["density_quality"], dtype=np.float64)
        normal_weights = np.clip(planarity, 0.0, 1.0) * np.clip(density_quality, 0.05, 1.0)
        normal_matrix = (normals * normal_weights.reshape((-1, 1))).T @ normals
        normal_evals, normal_evecs = np.linalg.eigh(normal_matrix)
        normal_axis = normal_evecs[:, int(np.argmin(normal_evals))]
        if float(np.dot(normal_axis, pca_axis)) < 0.0:
            normal_axis = -normal_axis

        candidate_axes: list[tuple[str, np.ndarray]] = []
        for source, candidate in (
            ("pca_axis", pca_axis),
            ("normal_axis", normal_axis),
            ("world_z", np.asarray([0.0, 0.0, 1.0], dtype=np.float64)),
        ):
            axis = _normalize_vector(candidate, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
            duplicate = False
            for _, existing in candidate_axes:
                if abs(float(np.dot(existing, axis))) >= 0.985:
                    duplicate = True
                    break
            if not duplicate:
                candidate_axes.append((source, axis))

        plane_axis = self._candidate_plane_axis(normals, planarity, np.asarray(geometry["curvature"]))
        if plane_axis is not None:
            plane_axis = _normalize_vector(plane_axis, pca_axis)
            duplicate = any(abs(float(np.dot(axis, plane_axis))) >= 0.985 for _, axis in candidate_axes)
            if not duplicate:
                candidate_axes.append(("planar_cap_axis", plane_axis))

        best_model: Optional[dict[str, Any]] = None
        for source, axis in candidate_axes:
            model = self._fit_cylinder_model_from_axis(pts, geometry, axis, source=source)
            if best_model is None or float(model["score"]) > float(best_model["score"]):
                best_model = model

        assert best_model is not None
        return best_model

    def _stabilize_cylinder_height(
        self, model: dict[str, Any], *, label: str, now_sec: float
    ) -> dict[str, Any]:
        stabilized = dict(model)
        debug_hold = {
            "height_hold_applied": False,
            "height_hold_lower_applied": False,
            "height_hold_upper_applied": False,
            "height_hold_age_sec": 0.0,
            "height_hold_prev_height_m": 0.0,
        }
        axis = np.asarray(model["axis"], dtype=np.float64).reshape((3,))
        center_world = np.asarray(model["center_world"], dtype=np.float64).reshape((3,))
        centerline_anchor = np.asarray(model["centerline_anchor"], dtype=np.float64).reshape((3,))
        radius = float(model["radius"])
        h_min = float(model["h_min"])
        h_max = float(model["h_max"])
        height = float(model["height"])

        previous = self._last_cylinder_fit_state.get(label)
        if previous and self.cylinder_height_hold_sec > 0.0:
            age_sec = max(0.0, float(now_sec - float(previous.get("timestamp", 0.0))))
            debug_hold["height_hold_age_sec"] = round(age_sec, 4)
            debug_hold["height_hold_prev_height_m"] = round(
                float(previous.get("height", 0.0)), 4
            )
            if age_sec <= self.cylinder_height_hold_sec:
                prev_axis = np.asarray(previous.get("axis", axis), dtype=np.float64).reshape((3,))
                prev_center = np.asarray(previous.get("center_world", center_world), dtype=np.float64).reshape((3,))
                prev_radius = float(previous.get("radius", radius))
                prev_h_min = float(previous.get("h_min", h_min))
                prev_h_max = float(previous.get("h_max", h_max))
                axis_cos = abs(float(np.dot(prev_axis, axis)))
                axis_match = axis_cos >= math.cos(math.radians(self.cylinder_height_match_axis_deg))
                center_match = (
                    float(np.linalg.norm(prev_center - center_world))
                    <= self.cylinder_height_match_center_m
                )
                radius_match = abs(prev_radius - radius) <= self.cylinder_height_match_radius_m
                if axis_match and center_match and radius_match:
                    lower_shrunk = h_min > (prev_h_min + self.cylinder_height_shrink_tolerance_m)
                    upper_shrunk = h_max < (prev_h_max - self.cylinder_height_shrink_tolerance_m)
                    if lower_shrunk:
                        h_min = prev_h_min
                        debug_hold["height_hold_lower_applied"] = True
                    if upper_shrunk:
                        h_max = prev_h_max
                        debug_hold["height_hold_upper_applied"] = True
                    if debug_hold["height_hold_lower_applied"] or debug_hold["height_hold_upper_applied"]:
                        height = max(self.output_voxel_size_m, h_max - h_min)
                        axial_mid = 0.5 * (h_min + h_max)
                        center_world = centerline_anchor + axis * axial_mid
                        debug_hold["height_hold_applied"] = True

        stabilized["h_min"] = float(h_min)
        stabilized["h_max"] = float(h_max)
        stabilized["height"] = float(height)
        stabilized["center_world"] = center_world.copy()
        stabilized["height_hold_applied"] = bool(debug_hold["height_hold_applied"])
        stabilized["height_hold_lower_applied"] = bool(debug_hold["height_hold_lower_applied"])
        stabilized["height_hold_upper_applied"] = bool(debug_hold["height_hold_upper_applied"])
        stabilized["height_hold_age_sec"] = float(debug_hold["height_hold_age_sec"])
        stabilized["height_hold_prev_height_m"] = float(debug_hold["height_hold_prev_height_m"])

        self._last_cylinder_fit_state[label] = {
            "timestamp": float(now_sec),
            "axis": axis.copy(),
            "center_world": center_world.copy(),
            "centerline_anchor": centerline_anchor.copy(),
            "radius": float(radius),
            "h_min": float(h_min),
            "h_max": float(h_max),
            "height": float(height),
        }
        return stabilized

    def _generate_cylinder_surface_points(self, model: dict[str, Any]) -> np.ndarray:
        basis = np.asarray(model["basis"], dtype=np.float64).reshape((3, 3))
        radius = float(model["radius"])
        h_min = float(model["h_min"])
        h_max = float(model["h_max"])
        centerline_anchor = np.asarray(model["centerline_anchor"], dtype=np.float64).reshape((3,))
        theta = np.linspace(0.0, 2.0 * math.pi, num=self.cylinder_angle_samples, endpoint=False)
        axial_values = np.linspace(h_min, h_max, num=self.cylinder_height_samples, endpoint=True)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        lateral_u = basis[:, 0]
        lateral_v = basis[:, 1]
        axis = basis[:, 2]

        rings: list[np.ndarray] = []
        for axial in axial_values:
            ring = (
                centerline_anchor.reshape((1, 3))
                + axial * axis.reshape((1, 3))
                + radius * cos_theta.reshape((-1, 1)) * lateral_u.reshape((1, 3))
                + radius * sin_theta.reshape((-1, 1)) * lateral_v.reshape((1, 3))
            )
            rings.append(ring)

        cap_radii = np.linspace(0.0, radius, num=max(4, self.cylinder_height_samples // 3), endpoint=True)
        lower_cap_visible = float(model["lower_cap_mass"]) >= max(
            1.0, 0.15 * float(np.sum(model["side_weight"]))
        )
        upper_cap_visible = float(model["upper_cap_mass"]) >= max(
            1.0, 0.15 * float(np.sum(model["side_weight"]))
        )
        if lower_cap_visible or upper_cap_visible:
            axial_cap_values = []
            if lower_cap_visible:
                axial_cap_values.append(h_min)
            if upper_cap_visible:
                axial_cap_values.append(h_max)
            for axial in axial_cap_values:
                for radius_value in cap_radii:
                    disk = (
                        centerline_anchor.reshape((1, 3))
                        + axial * axis.reshape((1, 3))
                        + radius_value * cos_theta.reshape((-1, 1)) * lateral_u.reshape((1, 3))
                        + radius_value * sin_theta.reshape((-1, 1)) * lateral_v.reshape((1, 3))
                    )
                    rings.append(disk)

        return np.asarray(np.concatenate(rings, axis=0), dtype=np.float32)

    def _fit_cylinder(self, points: np.ndarray) -> tuple[np.ndarray, Marker, dict[str, Any]]:
        label = self._normalize_label(self._latest_cloud_target_label or self._latest_semantic_target_label)
        model = self._estimate_cylinder_v1(points)
        model = self._stabilize_cylinder_height(model, label=label or "cylinder", now_sec=time.time())
        fitted_points = self._generate_cylinder_surface_points(model)
        marker = self._make_cylinder_marker(
            stamp=self._latest_cloud_msg.header.stamp,
            frame_id=str(self._latest_cloud_msg.header.frame_id or ""),
            basis=np.asarray(model["basis"], dtype=np.float64),
            center_world=np.asarray(model["center_world"], dtype=np.float64),
            radius=float(model["radius"]),
            height=float(model["height"]),
        )
        debug = {
            "fit_radius_m": round(float(model["radius"]), 4),
            "fit_side_radius_m": round(float(model.get("side_radius", model["radius"])), 4),
            "fit_upper_cap_radius_m": round(float(model.get("upper_cap_radius", model["radius"])), 4),
            "fit_lower_cap_radius_m": round(float(model.get("lower_cap_radius", model["radius"])), 4),
            "fit_height_m": round(float(model["height"]), 4),
            "fit_axis_source": str(model["source"]),
            "fit_score": round(float(model["score"]), 4),
            "fit_side_support": round(float(model["side_support_mean"]), 4),
            "fit_cap_support": round(float(model["cap_support_mean"]), 4),
            "fit_support_coverage": round(float(model["support_coverage"]), 4),
            "fit_radial_error_m": round(float(model["radial_error"]), 4),
            "fit_side_normal_error": round(float(model["side_normal_error"]), 4),
            "fit_cap_normal_error": round(float(model["cap_normal_error"]), 4),
            "fit_boundary_samples": int(model.get("boundary_sample_count", 0)),
            "fit_upper_cap_samples": int(model.get("upper_cap_sample_count", 0)),
            "fit_lower_cap_samples": int(model.get("lower_cap_sample_count", 0)),
            "fit_lower_cap_visible": bool(
                float(model["lower_cap_mass"]) >= max(1.0, 0.15 * float(np.sum(model["side_weight"])))
            ),
            "fit_upper_cap_visible": bool(
                float(model["upper_cap_mass"]) >= max(1.0, 0.15 * float(np.sum(model["side_weight"])))
            ),
            "fit_tail_lower_extended_m": round(float(model.get("tail_lower_extended_m", 0.0)), 4),
            "fit_tail_upper_extended_m": round(float(model.get("tail_upper_extended_m", 0.0)), 4),
            "fit_tail_gap_limit_m": round(float(model.get("tail_gap_limit_m", 0.0)), 4),
            "fit_tail_residual_limit_m": round(float(model.get("tail_residual_limit_m", 0.0)), 4),
            "fit_tail_lower_sector_span": int(model.get("tail_lower_sector_span", 0)),
            "fit_tail_upper_sector_span": int(model.get("tail_upper_sector_span", 0)),
            "fit_tail_lower_points": int(model.get("tail_lower_points", 0)),
            "fit_tail_upper_points": int(model.get("tail_upper_points", 0)),
            "fit_height_hold_applied": bool(model.get("height_hold_applied", False)),
            "fit_height_hold_lower_applied": bool(model.get("height_hold_lower_applied", False)),
            "fit_height_hold_upper_applied": bool(model.get("height_hold_upper_applied", False)),
            "fit_height_hold_age_sec": round(float(model.get("height_hold_age_sec", 0.0)), 4),
            "fit_height_hold_prev_height_m": round(float(model.get("height_hold_prev_height_m", 0.0)), 4),
        }
        return np.asarray(fitted_points, dtype=np.float32), marker, debug

    def _fit_axis_profile(self, points: np.ndarray) -> tuple[np.ndarray, Marker, dict[str, Any]]:
        center, basis, local = self._principal_axis_frame(points)
        radial = np.linalg.norm(local[:, :2], axis=1)
        heights = local[:, 2]
        z_min = float(np.percentile(heights, 2.0))
        z_max = float(np.percentile(heights, 98.0))
        if (z_max - z_min) <= 1e-4:
            return self._fit_cylinder(points)
        bin_edges = np.linspace(z_min, z_max, num=self.profile_height_bins + 1, endpoint=True)
        bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])
        profile_radii = np.zeros((self.profile_height_bins,), dtype=np.float64)
        valid_bins = np.zeros((self.profile_height_bins,), dtype=bool)
        for idx in range(self.profile_height_bins):
            mask = (heights >= bin_edges[idx]) & (heights <= bin_edges[idx + 1])
            if int(np.count_nonzero(mask)) < 4:
                continue
            profile_radii[idx] = max(0.001, float(np.percentile(radial[mask], 60.0)))
            valid_bins[idx] = True
        if not np.any(valid_bins):
            return self._fit_cylinder(points)
        if np.count_nonzero(valid_bins) == 1:
            profile_radii[:] = float(profile_radii[valid_bins][0])
        else:
            profile_radii = np.interp(
                np.arange(self.profile_height_bins, dtype=np.float64),
                np.flatnonzero(valid_bins).astype(np.float64),
                profile_radii[valid_bins],
            )
        if self.profile_height_bins >= 3:
            profile_radii = np.convolve(profile_radii, np.asarray([0.25, 0.5, 0.25]), mode="same")
            profile_radii = np.maximum(profile_radii, 0.001)
        theta = np.linspace(0.0, 2.0 * math.pi, num=self.profile_angle_samples, endpoint=False)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        surface_rings = []
        for z_value, radius in zip(bin_centers, profile_radii.tolist()):
            ring = np.column_stack(
                [
                    float(radius) * cos_theta,
                    float(radius) * sin_theta,
                    np.full_like(cos_theta, z_value),
                ]
            )
            surface_rings.append(ring)
        local_surface = np.concatenate(surface_rings, axis=0)
        fitted_points = (local_surface @ basis.T) + center.reshape((1, 3))
        axis_line_center = center + basis[:, 2] * float(0.5 * (z_min + z_max))
        marker = self._make_axis_marker(
            stamp=self._latest_cloud_msg.header.stamp,
            frame_id=str(self._latest_cloud_msg.header.frame_id or ""),
            start=(axis_line_center - basis[:, 2] * (0.5 * (z_max - z_min))),
            end=(axis_line_center + basis[:, 2] * (0.5 * (z_max - z_min))),
            ns="target_profile_axis",
            marker_id=1,
            color_rgb=(0.3, 0.8, 1.0),
            scale_x=0.004,
        )
        debug = {
            "fit_height_m": round(float(z_max - z_min), 4),
            "profile_radius_min_m": round(float(np.min(profile_radii)), 4),
            "profile_radius_max_m": round(float(np.max(profile_radii)), 4),
        }
        return np.asarray(fitted_points, dtype=np.float32), marker, debug

    def _weighted_sphere_fit_3d(
        self,
        points: np.ndarray,
        weights: np.ndarray,
        fallback_center: np.ndarray,
        fallback_radius: float,
    ) -> tuple[np.ndarray, float]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        w = np.asarray(weights, dtype=np.float64).reshape((-1,))
        valid = np.isfinite(pts).all(axis=1) & np.isfinite(w) & (w > 1e-8)
        pts = pts[valid]
        w = w[valid]
        if pts.shape[0] < 8:
            return np.asarray(fallback_center, dtype=np.float64).reshape((3,)), float(
                max(1e-3, fallback_radius)
            )
        system = np.column_stack(
            [pts[:, 0], pts[:, 1], pts[:, 2], np.ones((pts.shape[0],), dtype=np.float64)]
        )
        rhs = -(pts[:, 0] * pts[:, 0] + pts[:, 1] * pts[:, 1] + pts[:, 2] * pts[:, 2])
        sqrt_w = np.sqrt(np.maximum(w, 1e-8))
        system_w = system * sqrt_w.reshape((-1, 1))
        rhs_w = rhs * sqrt_w
        try:
            solution, _, _, _ = np.linalg.lstsq(system_w, rhs_w, rcond=None)
        except np.linalg.LinAlgError:
            return np.asarray(fallback_center, dtype=np.float64).reshape((3,)), float(
                max(1e-3, fallback_radius)
            )
        center = -0.5 * solution[:3]
        radius_sq = float(np.dot(center, center) - solution[3])
        if not np.isfinite(radius_sq) or radius_sq <= 1e-8:
            return np.asarray(fallback_center, dtype=np.float64).reshape((3,)), float(
                max(1e-3, fallback_radius)
            )
        radius = math.sqrt(radius_sq)
        return center.astype(np.float64), float(max(1e-3, radius))

    def _fit_sphere_model(self, points: np.ndarray) -> dict[str, Any]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        geometry = self._local_surface_geometry(pts)
        normals = np.asarray(geometry["normals"], dtype=np.float64).reshape((-1, 3))
        planarity = np.asarray(geometry["planarity"], dtype=np.float64).reshape((-1,))
        density_quality = np.asarray(geometry["density_quality"], dtype=np.float64).reshape((-1,))
        curvature = np.asarray(geometry["curvature"], dtype=np.float64).reshape((-1,))
        curvature_scale = max(float(np.percentile(curvature, 80.0)), 1e-6)
        base_quality = np.clip(
            density_quality * (0.35 + 0.65 * np.clip(curvature / curvature_scale, 0.0, 1.0)),
            0.05,
            1.0,
        )

        center_guess = np.mean(pts, axis=0)
        radius_guess = float(
            max(
                self.output_voxel_size_m,
                np.percentile(np.linalg.norm(pts - center_guess.reshape((1, 3)), axis=1), 65.0),
            )
        )
        center, radius = self._weighted_sphere_fit_3d(
            pts, base_quality, center_guess, radius_guess
        )

        best_model: Optional[dict[str, Any]] = None
        for _ in range(3):
            radial_vectors = pts - center.reshape((1, 3))
            radial_norm = np.linalg.norm(radial_vectors, axis=1)
            radial_dir = radial_vectors / np.maximum(radial_norm.reshape((-1, 1)), 1e-6)
            radial_residual = np.abs(radial_norm - radius)
            residual_scale = max(
                self.output_voxel_size_m * 2.0,
                float(np.percentile(radial_residual, 60.0)) * 1.8,
                float(radius) * 0.10,
            )
            residual_weight = np.exp(-0.5 * np.square(radial_residual / residual_scale))
            normal_alignment = np.abs(np.einsum("ij,ij->i", radial_dir, normals))
            normal_weight = np.exp(-0.5 * np.square((1.0 - normal_alignment) / 0.22))
            fit_weight = np.clip(base_quality * residual_weight * normal_weight, 0.0, 1.0)
            center, radius = self._weighted_sphere_fit_3d(
                pts, fit_weight, center, radius
            )
            support_mask = fit_weight >= max(0.15, 0.3 * float(np.max(fit_weight)))
            radial_error = float(
                np.median(radial_residual[support_mask])
                if np.any(support_mask)
                else np.median(radial_residual)
            )
            normal_error = float(
                np.mean(1.0 - normal_alignment[support_mask])
                if np.any(support_mask)
                else np.mean(1.0 - normal_alignment)
            )
            score = (
                1.8 * float(np.mean(fit_weight))
                - 3.2 * (radial_error / max(radius, self.output_voxel_size_m * 1.5))
                - 0.5 * normal_error
            )
            best_model = {
                "center_world": center.copy(),
                "radius": float(radius),
                "fit_weight": fit_weight.copy(),
                "radial_error": float(radial_error),
                "normal_error": float(normal_error),
                "score": float(score),
                "support_mean": float(np.mean(fit_weight)),
            }
        assert best_model is not None
        return best_model

    def _generate_sphere_surface_points(self, model: dict[str, Any]) -> np.ndarray:
        center = np.asarray(model["center_world"], dtype=np.float64).reshape((3,))
        radius = float(model["radius"])
        theta = np.linspace(0.0, 2.0 * math.pi, num=self.profile_angle_samples, endpoint=False)
        phi = np.linspace(0.0, math.pi, num=max(10, self.profile_height_bins), endpoint=True)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        rings: list[np.ndarray] = []
        for sin_value, cos_value in zip(sin_phi.tolist(), cos_phi.tolist()):
            ring = np.column_stack(
                [
                    center[0] + radius * sin_value * cos_theta,
                    center[1] + radius * sin_value * sin_theta,
                    center[2] + radius * cos_value * np.ones_like(theta),
                ]
            )
            rings.append(ring)
        return np.asarray(np.concatenate(rings, axis=0), dtype=np.float32)

    def _fit_sphere(self, points: np.ndarray) -> tuple[np.ndarray, Marker, dict[str, Any]]:
        model = self._fit_sphere_model(points)
        fitted_points = self._generate_sphere_surface_points(model)
        marker = self._make_sphere_marker(
            stamp=self._latest_cloud_msg.header.stamp,
            frame_id=str(self._latest_cloud_msg.header.frame_id or ""),
            center_world=np.asarray(model["center_world"], dtype=np.float64),
            radius=float(model["radius"]),
        )
        debug = {
            "fit_radius_m": round(float(model["radius"]), 4),
            "fit_score": round(float(model["score"]), 4),
            "fit_support_mean": round(float(model["support_mean"]), 4),
            "fit_radial_error_m": round(float(model["radial_error"]), 4),
            "fit_normal_error": round(float(model["normal_error"]), 4),
        }
        return np.asarray(fitted_points, dtype=np.float32), marker, debug

    def _fit_hemisphere_model(self, points: np.ndarray) -> Optional[dict[str, Any]]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        if pts.shape[0] < self.fit_min_points:
            return None
        sphere = self._fit_sphere_model(pts)
        center = np.asarray(sphere["center_world"], dtype=np.float64).reshape((3,))
        radius = float(sphere["radius"])
        geometry = self._local_surface_geometry(pts)
        normals = np.asarray(geometry["normals"], dtype=np.float64).reshape((-1, 3))
        planarity = np.asarray(geometry["planarity"], dtype=np.float64).reshape((-1,))
        curvature = np.asarray(geometry["curvature"], dtype=np.float64).reshape((-1,))
        density_quality = np.asarray(geometry["density_quality"], dtype=np.float64).reshape((-1,))
        rel = pts - center.reshape((1, 3))
        mean_visible = np.mean(rel, axis=0)
        fallback_axis = _normalize_vector(mean_visible, np.asarray([0.0, 0.0, 1.0], dtype=np.float64))
        plane_axis = self._candidate_plane_axis(normals, planarity, curvature)
        if plane_axis is not None and float(np.dot(plane_axis, fallback_axis)) < 0.0:
            plane_axis = -plane_axis
        axis = _normalize_vector(plane_axis if plane_axis is not None else fallback_axis, fallback_axis)

        signed_axial = rel @ axis
        radial_dir = rel / np.maximum(np.linalg.norm(rel, axis=1, keepdims=True), 1e-6)
        normal_alignment = np.abs(np.einsum("ij,j->i", normals, axis))
        cap_support = (
            np.clip(planarity, 0.0, 1.0)
            * np.clip(density_quality, 0.05, 1.0)
            * np.exp(-0.5 * np.square(signed_axial / max(radius * 0.18, self.output_voxel_size_m * 2.0)))
            * np.exp(-0.5 * np.square((1.0 - normal_alignment) / 0.22))
        )
        dome_alignment = np.clip(np.einsum("ij,j->i", radial_dir, axis), -1.0, 1.0)
        dome_support = (
            np.clip(density_quality, 0.05, 1.0)
            * np.exp(-0.5 * np.square((1.0 - dome_alignment) / 0.55))
        )
        visible_fraction = float(np.mean(signed_axial >= -0.08 * radius))
        cap_visible = float(np.sum(cap_support)) >= max(1.0, 0.08 * float(pts.shape[0]))
        if visible_fraction < 0.62 and not cap_visible:
            return None

        basis = self._basis_from_axis(axis)
        score = (
            float(sphere["score"])
            + 0.45 * visible_fraction
            + 0.25 * float(np.mean(cap_support))
            + 0.15 * float(np.mean(dome_support))
        )
        return {
            "center_world": center.copy(),
            "radius": float(radius),
            "axis": axis.copy(),
            "basis": basis.copy(),
            "cap_visible": bool(cap_visible),
            "visible_fraction": float(visible_fraction),
            "cap_support_mean": float(np.mean(cap_support)),
            "score": float(score),
            "sphere_score": float(sphere["score"]),
            "radial_error": float(sphere["radial_error"]),
            "normal_error": float(sphere["normal_error"]),
        }

    def _generate_hemisphere_surface_points(self, model: dict[str, Any]) -> np.ndarray:
        center = np.asarray(model["center_world"], dtype=np.float64).reshape((3,))
        basis = np.asarray(model["basis"], dtype=np.float64).reshape((3, 3))
        radius = float(model["radius"])
        theta = np.linspace(0.0, 2.0 * math.pi, num=self.profile_angle_samples, endpoint=False)
        phi = np.linspace(0.0, 0.5 * math.pi, num=max(8, self.profile_height_bins), endpoint=True)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rings: list[np.ndarray] = []
        for phi_value in phi.tolist():
            ring_radius = radius * math.sin(phi_value)
            axial = radius * math.cos(phi_value)
            local_ring = np.column_stack(
                [
                    ring_radius * cos_theta,
                    ring_radius * sin_theta,
                    np.full_like(theta, axial),
                ]
            )
            rings.append((local_ring @ basis.T) + center.reshape((1, 3)))
        cap_radii = np.linspace(0.0, radius, num=max(4, self.profile_height_bins // 3), endpoint=True)
        for radius_value in cap_radii.tolist():
            local_disk = np.column_stack(
                [
                    radius_value * cos_theta,
                    radius_value * sin_theta,
                    np.zeros_like(theta),
                ]
            )
            rings.append((local_disk @ basis.T) + center.reshape((1, 3)))
        return np.asarray(np.concatenate(rings, axis=0), dtype=np.float32)

    def _fit_hemisphere(self, points: np.ndarray) -> tuple[np.ndarray, Marker, dict[str, Any]]:
        model = self._fit_hemisphere_model(points)
        if model is None:
            fitted_points, marker, debug = self._fit_sphere(points)
            debug["fit_fallback"] = "sphere"
            return fitted_points, marker, debug
        fitted_points = self._generate_hemisphere_surface_points(model)
        marker = self._make_hemisphere_marker(
            stamp=self._latest_cloud_msg.header.stamp,
            frame_id=str(self._latest_cloud_msg.header.frame_id or ""),
            center_world=np.asarray(model["center_world"], dtype=np.float64),
            basis=np.asarray(model["basis"], dtype=np.float64),
            radius=float(model["radius"]),
        )
        debug = {
            "fit_radius_m": round(float(model["radius"]), 4),
            "fit_score": round(float(model["score"]), 4),
            "fit_visible_fraction": round(float(model["visible_fraction"]), 4),
            "fit_cap_visible": bool(model["cap_visible"]),
            "fit_cap_support": round(float(model["cap_support_mean"]), 4),
            "fit_radial_error_m": round(float(model["radial_error"]), 4),
            "fit_normal_error": round(float(model["normal_error"]), 4),
        }
        return np.asarray(fitted_points, dtype=np.float32), marker, debug

    def _fit_frustum_model(self, points: np.ndarray) -> Optional[dict[str, Any]]:
        cylinder = self._estimate_cylinder_v1(points)
        basis = np.asarray(cylinder["basis"], dtype=np.float64).reshape((3, 3))
        axis = np.asarray(cylinder["axis"], dtype=np.float64).reshape((3,))
        centerline_anchor = np.asarray(cylinder["centerline_anchor"], dtype=np.float64).reshape((3,))
        h_min = float(cylinder["h_min"])
        h_max = float(cylinder["h_max"])
        height = max(self.output_voxel_size_m, h_max - h_min)
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        rel = pts - centerline_anchor.reshape((1, 3))
        axial = rel @ axis
        radial_vectors = rel - np.outer(axial, axis)
        radial = np.linalg.norm(radial_vectors, axis=1)
        side_weight = np.asarray(cylinder["side_weight"], dtype=np.float64).reshape((-1,))
        fit_mask = (
            (axial >= h_min - 0.08 * height)
            & (axial <= h_max + 0.08 * height)
            & (side_weight >= max(0.08, 0.25 * float(np.max(side_weight))))
        )
        if int(np.count_nonzero(fit_mask)) < 16:
            return None

        fit_axial = axial[fit_mask]
        fit_radial = radial[fit_mask]
        fit_weight = side_weight[fit_mask]
        t = (fit_axial - h_min) / max(height, self.output_voxel_size_m)
        a_matrix = np.column_stack([np.ones_like(t), t])
        sqrt_w = np.sqrt(np.maximum(fit_weight, 1e-8))
        try:
            coeffs, _, _, _ = np.linalg.lstsq(
                a_matrix * sqrt_w.reshape((-1, 1)),
                fit_radial * sqrt_w,
                rcond=None,
            )
        except np.linalg.LinAlgError:
            return None
        lower_radius = float(max(self.output_voxel_size_m, coeffs[0]))
        upper_radius = float(max(self.output_voxel_size_m, coeffs[0] + coeffs[1]))
        slope_ratio = abs(upper_radius - lower_radius) / max(max(lower_radius, upper_radius), 1e-6)
        if slope_ratio < 0.06:
            return None

        pred_radial = coeffs[0] + coeffs[1] * t
        residual = np.abs(fit_radial - pred_radial)
        radial_error = float(np.median(residual))
        support = float(np.mean(np.exp(-0.5 * np.square(residual / max(0.003, 0.15 * np.mean(pred_radial))))))
        center_world = centerline_anchor + axis * (0.5 * (h_min + h_max))
        score = (
            float(cylinder["score"])
            + 0.35 * slope_ratio
            + 0.25 * support
            - 1.4 * (radial_error / max(np.mean(pred_radial), self.output_voxel_size_m))
        )
        return {
            "basis": basis.copy(),
            "axis": axis.copy(),
            "centerline_anchor": centerline_anchor.copy(),
            "center_world": center_world.copy(),
            "h_min": float(h_min),
            "h_max": float(h_max),
            "height": float(height),
            "lower_radius": float(lower_radius),
            "upper_radius": float(upper_radius),
            "score": float(score),
            "radial_error": float(radial_error),
            "support_mean": float(support),
            "cylinder_score": float(cylinder["score"]),
            "slope_ratio": float(slope_ratio),
        }

    def _generate_frustum_surface_points(self, model: dict[str, Any]) -> np.ndarray:
        basis = np.asarray(model["basis"], dtype=np.float64).reshape((3, 3))
        centerline_anchor = np.asarray(model["centerline_anchor"], dtype=np.float64).reshape((3,))
        axis = basis[:, 2]
        lateral_u = basis[:, 0]
        lateral_v = basis[:, 1]
        theta = np.linspace(0.0, 2.0 * math.pi, num=self.cylinder_angle_samples, endpoint=False)
        axial_values = np.linspace(
            float(model["h_min"]),
            float(model["h_max"]),
            num=self.cylinder_height_samples,
            endpoint=True,
        )
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        rings: list[np.ndarray] = []
        for axial in axial_values.tolist():
            blend = (axial - float(model["h_min"])) / max(float(model["height"]), self.output_voxel_size_m)
            radius = (1.0 - blend) * float(model["lower_radius"]) + blend * float(model["upper_radius"])
            ring = (
                centerline_anchor.reshape((1, 3))
                + axial * axis.reshape((1, 3))
                + radius * cos_theta.reshape((-1, 1)) * lateral_u.reshape((1, 3))
                + radius * sin_theta.reshape((-1, 1)) * lateral_v.reshape((1, 3))
            )
            rings.append(ring)
        for axial, radius in (
            (float(model["h_min"]), float(model["lower_radius"])),
            (float(model["h_max"]), float(model["upper_radius"])),
        ):
            disk_radii = np.linspace(0.0, radius, num=max(4, self.cylinder_height_samples // 3), endpoint=True)
            for radius_value in disk_radii.tolist():
                disk = (
                    centerline_anchor.reshape((1, 3))
                    + axial * axis.reshape((1, 3))
                    + radius_value * cos_theta.reshape((-1, 1)) * lateral_u.reshape((1, 3))
                    + radius_value * sin_theta.reshape((-1, 1)) * lateral_v.reshape((1, 3))
                )
                rings.append(disk)
        return np.asarray(np.concatenate(rings, axis=0), dtype=np.float32)

    def _fit_frustum(self, points: np.ndarray) -> tuple[np.ndarray, Marker, dict[str, Any]]:
        model = self._fit_frustum_model(points)
        if model is None:
            fitted_points, marker, debug = self._fit_axis_profile(points)
            debug["fit_fallback"] = "axis_profile"
            return fitted_points, marker, debug
        fitted_points = self._generate_frustum_surface_points(model)
        marker = self._make_frustum_marker(
            stamp=self._latest_cloud_msg.header.stamp,
            frame_id=str(self._latest_cloud_msg.header.frame_id or ""),
            basis=np.asarray(model["basis"], dtype=np.float64),
            centerline_anchor=np.asarray(model["centerline_anchor"], dtype=np.float64),
            h_min=float(model["h_min"]),
            h_max=float(model["h_max"]),
            lower_radius=float(model["lower_radius"]),
            upper_radius=float(model["upper_radius"]),
        )
        debug = {
            "fit_height_m": round(float(model["height"]), 4),
            "fit_lower_radius_m": round(float(model["lower_radius"]), 4),
            "fit_upper_radius_m": round(float(model["upper_radius"]), 4),
            "fit_score": round(float(model["score"]), 4),
            "fit_support_mean": round(float(model["support_mean"]), 4),
            "fit_radial_error_m": round(float(model["radial_error"]), 4),
            "fit_slope_ratio": round(float(model["slope_ratio"]), 4),
        }
        return np.asarray(fitted_points, dtype=np.float32), marker, debug

    def _weighted_percentile(
        self,
        values: np.ndarray,
        weights: np.ndarray,
        percentile: float,
        fallback: float,
    ) -> float:
        vals = np.asarray(values, dtype=np.float64).reshape((-1,))
        w = np.asarray(weights, dtype=np.float64).reshape((-1,))
        valid = np.isfinite(vals) & np.isfinite(w) & (w > 1e-8)
        vals = vals[valid]
        w = w[valid]
        if vals.size <= 0:
            return float(fallback)
        order = np.argsort(vals)
        vals = vals[order]
        w = w[order]
        cumulative = np.cumsum(w)
        if float(cumulative[-1]) <= 1e-8:
            return float(fallback)
        target = 0.01 * float(percentile) * float(cumulative[-1])
        index = int(np.searchsorted(cumulative, target, side="left"))
        index = max(0, min(index, vals.size - 1))
        return float(vals[index])

    def _estimate_patch_rectangle_bounds(
        self,
        patch_local_2d: np.ndarray,
        weights: np.ndarray,
    ) -> tuple[float, float, float, float, int]:
        coords = np.asarray(patch_local_2d, dtype=np.float64).reshape((-1, 2))
        w = np.asarray(weights, dtype=np.float64).reshape((-1,))
        valid = np.isfinite(coords).all(axis=1) & np.isfinite(w) & (w > 1e-8)
        coords = coords[valid]
        w = w[valid]
        if coords.shape[0] < 8:
            if coords.shape[0] <= 0:
                return -0.01, 0.01, -0.01, 0.01, 0
            return (
                float(np.min(coords[:, 0])),
                float(np.max(coords[:, 0])),
                float(np.min(coords[:, 1])),
                float(np.max(coords[:, 1])),
                int(coords.shape[0]),
            )

        u = coords[:, 0]
        v = coords[:, 1]
        u_lo = self._weighted_percentile(u, w, 2.0, float(np.min(u)))
        u_hi = self._weighted_percentile(u, w, 98.0, float(np.max(u)))
        v_lo = self._weighted_percentile(v, w, 2.0, float(np.min(v)))
        v_hi = self._weighted_percentile(v, w, 98.0, float(np.max(v)))
        span_u = max(u_hi - u_lo, self.output_voxel_size_m)
        span_v = max(v_hi - v_lo, self.output_voxel_size_m)
        center_u = 0.5 * (u_lo + u_hi)
        center_v = 0.5 * (v_lo + v_hi)
        edge_metric_u = np.abs(u - center_u) / max(0.5 * span_u, self.output_voxel_size_m)
        edge_metric_v = np.abs(v - center_v) / max(0.5 * span_v, self.output_voxel_size_m)
        edge_metric = np.maximum(edge_metric_u, edge_metric_v)
        boundary_weight = w * np.clip((edge_metric - 0.45) / 0.55, 0.0, 1.0)
        if int(np.count_nonzero(boundary_weight > 1e-6)) < 10:
            boundary_weight = w.copy()

        bound_u_lo = self._weighted_percentile(u, boundary_weight, 1.0, u_lo)
        bound_u_hi = self._weighted_percentile(u, boundary_weight, 99.0, u_hi)
        bound_v_lo = self._weighted_percentile(v, boundary_weight, 1.0, v_lo)
        bound_v_hi = self._weighted_percentile(v, boundary_weight, 99.0, v_hi)
        if bound_u_hi <= bound_u_lo:
            bound_u_lo, bound_u_hi = u_lo, u_hi
        if bound_v_hi <= bound_v_lo:
            bound_v_lo, bound_v_hi = v_lo, v_hi
        return (
            float(bound_u_lo),
            float(bound_u_hi),
            float(bound_v_lo),
            float(bound_v_hi),
            int(np.count_nonzero(boundary_weight > 1e-6)),
        )

    def _orthonormalize_box_basis(
        self, basis: np.ndarray, reference: Optional[np.ndarray] = None
    ) -> np.ndarray:
        matrix = np.asarray(basis, dtype=np.float64).reshape((3, 3)).copy()
        ref = (
            np.asarray(reference, dtype=np.float64).reshape((3, 3))
            if reference is not None
            else np.eye(3, dtype=np.float64)
        )
        u0 = _normalize_vector(matrix[:, 0], ref[:, 0])
        v1 = matrix[:, 1] - float(np.dot(matrix[:, 1], u0)) * u0
        u1 = _normalize_vector(v1, ref[:, 1])
        v2 = matrix[:, 2] - float(np.dot(matrix[:, 2], u0)) * u0 - float(np.dot(matrix[:, 2], u1)) * u1
        u2 = _normalize_vector(v2, np.cross(u0, u1))
        if float(np.linalg.norm(np.cross(u0, u1))) <= 1e-6:
            u2 = _normalize_vector(np.cross(u0, ref[:, 1]), ref[:, 2])
            u1 = _normalize_vector(np.cross(u2, u0), ref[:, 1])
        else:
            u2 = _normalize_vector(np.cross(u0, u1), ref[:, 2])
            u1 = _normalize_vector(np.cross(u2, u0), ref[:, 1])
        ortho = np.stack([u0, u1, u2], axis=1)
        if np.linalg.det(ortho) < 0.0:
            ortho[:, 2] *= -1.0
        for axis_index in range(3):
            if float(np.dot(ortho[:, axis_index], ref[:, axis_index])) < 0.0:
                ortho[:, axis_index] *= -1.0
        if np.linalg.det(ortho) < 0.0:
            ortho[:, 2] *= -1.0
        return ortho.astype(np.float64)

    def _reorder_basis_like(self, candidate: np.ndarray, reference: np.ndarray) -> np.ndarray:
        cand = np.asarray(candidate, dtype=np.float64).reshape((3, 3))
        ref = np.asarray(reference, dtype=np.float64).reshape((3, 3))
        best_score = -1.0
        best_basis = ref.copy()
        for order in permutations((0, 1, 2)):
            permuted = cand[:, order].copy()
            for axis_index in range(3):
                if float(np.dot(permuted[:, axis_index], ref[:, axis_index])) < 0.0:
                    permuted[:, axis_index] *= -1.0
            score = float(
                sum(abs(float(np.dot(permuted[:, axis_index], ref[:, axis_index]))) for axis_index in range(3))
            )
            if score > best_score:
                best_score = score
                best_basis = permuted
        return self._orthonormalize_box_basis(best_basis, ref)

    def _candidate_box_basis_from_normals(
        self,
        normals: np.ndarray,
        planarity: np.ndarray,
        density_quality: np.ndarray,
        reference_basis: np.ndarray,
    ) -> np.ndarray:
        normal_weights = np.clip(planarity, 0.0, 1.0) * np.clip(density_quality, 0.05, 1.0)
        covariance = (normals * normal_weights.reshape((-1, 1))).T @ normals
        _, eigenvectors = np.linalg.eigh(covariance)
        basis = eigenvectors[:, ::-1]
        basis = self._reorder_basis_like(basis, reference_basis)
        return basis.astype(np.float64)

    def _largest_connected_component_indices(
        self, points: np.ndarray, *, neighbor_radius: float
    ) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        count = int(pts.shape[0])
        if count <= 1:
            return np.arange(count, dtype=np.int32)
        radius_sq = float(max(neighbor_radius, self.output_voxel_size_m * 2.0) ** 2)
        diffs = pts[:, None, :] - pts[None, :, :]
        dist_sq = np.sum(diffs * diffs, axis=2)
        adjacency = dist_sq <= radius_sq
        visited = np.zeros((count,), dtype=bool)
        best_component = np.zeros((0,), dtype=np.int32)
        for seed in range(count):
            if visited[seed]:
                continue
            stack = [seed]
            visited[seed] = True
            component: list[int] = []
            while stack:
                current = stack.pop()
                component.append(int(current))
                neighbors = np.flatnonzero(adjacency[current] & (~visited))
                for neighbor in neighbors.tolist():
                    visited[neighbor] = True
                    stack.append(int(neighbor))
            if len(component) > int(best_component.size):
                best_component = np.asarray(component, dtype=np.int32)
        return best_component

    def _extract_plane_patches(
        self, points: np.ndarray, geometry: dict[str, np.ndarray]
    ) -> list[dict[str, Any]]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        normals = np.asarray(geometry["normals"], dtype=np.float64).reshape((-1, 3))
        planarity = np.asarray(geometry["planarity"], dtype=np.float64).reshape((-1,))
        density_quality = np.asarray(geometry["density_quality"], dtype=np.float64).reshape((-1,))
        base_quality = np.clip(planarity, 0.0, 1.0) * np.clip(density_quality, 0.05, 1.0)
        available = base_quality >= max(0.12, 0.35 * float(np.max(base_quality)))
        if int(np.count_nonzero(available)) < 12:
            return []

        seed_indices = np.argsort(base_quality)[::-1][: min(80, pts.shape[0])]
        plane_dist_thresh = max(self.output_voxel_size_m * 2.5, 0.004)
        neighbor_radius = max(self.output_voxel_size_m * 4.0, 0.010)
        align_threshold = math.cos(math.radians(16.0))
        patches: list[dict[str, Any]] = []

        for seed in seed_indices.tolist():
            if not available[seed]:
                continue
            seed_normal = normals[seed]
            seed_point = pts[seed]
            orientation = normals @ seed_normal
            distance = np.abs((pts - seed_point.reshape((1, 3))) @ seed_normal.reshape((3, 1))).reshape((-1,))
            inlier_mask = available & (orientation >= align_threshold) & (distance <= plane_dist_thresh)
            inlier_indices = np.flatnonzero(inlier_mask)
            if inlier_indices.size < 12:
                continue

            component_local = self._largest_connected_component_indices(
                pts[inlier_indices], neighbor_radius=neighbor_radius
            )
            if component_local.size < 12:
                continue
            indices = inlier_indices[component_local]
            patch_points = pts[indices]
            patch_weights = base_quality[indices]
            patch_normal = np.sum(
                normals[indices] * patch_weights.reshape((-1, 1)), axis=0, dtype=np.float64
            )
            patch_normal = _normalize_vector(patch_normal, seed_normal)
            plane_coord = float(
                self._weighted_percentile(
                    patch_points @ patch_normal,
                    patch_weights,
                    50.0,
                    float(np.dot(np.mean(patch_points, axis=0), patch_normal)),
                )
            )
            tangent_u = _normalize_vector(
                np.cross(
                    patch_normal,
                    np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
                    if abs(float(patch_normal[2])) < 0.92
                    else np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
                ),
                np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
            )
            tangent_v = _normalize_vector(np.cross(patch_normal, tangent_u), np.asarray([0.0, 1.0, 0.0], dtype=np.float64))
            patch_center = np.mean(patch_points, axis=0)
            patch_local = np.column_stack(
                [
                    (patch_points - patch_center.reshape((1, 3))) @ tangent_u,
                    (patch_points - patch_center.reshape((1, 3))) @ tangent_v,
                ]
            )
            bound_u_lo, bound_u_hi, bound_v_lo, bound_v_hi, boundary_count = self._estimate_patch_rectangle_bounds(
                patch_local,
                patch_weights,
            )
            span_u = float(bound_u_hi - bound_u_lo)
            span_v = float(bound_v_hi - bound_v_lo)
            area = max(0.0, span_u) * max(0.0, span_v)
            patch_score = float(np.sum(patch_weights)) + 120.0 * float(area)
            patches.append(
                {
                    "normal": patch_normal.copy(),
                    "indices": indices.copy(),
                    "plane_coord": float(plane_coord),
                    "center": patch_center.copy(),
                    "area": float(area),
                    "score": float(patch_score),
                    "span_u": float(span_u),
                    "span_v": float(span_v),
                    "bound_u_lo": float(bound_u_lo),
                    "bound_u_hi": float(bound_u_hi),
                    "bound_v_lo": float(bound_v_lo),
                    "bound_v_hi": float(bound_v_hi),
                    "boundary_count": int(boundary_count),
                    "weight_sum": float(np.sum(patch_weights)),
                }
            )
            available[indices] = False

        patches.sort(key=lambda item: float(item["score"]), reverse=True)
        return patches

    def _select_box_face_patches(
        self, patches: list[dict[str, Any]], reference_basis: np.ndarray
    ) -> list[dict[str, Any]]:
        if not patches:
            return []
        selected: list[dict[str, Any]] = []
        orth_threshold = math.cos(math.radians(62.0))
        parallel_threshold = math.cos(math.radians(20.0))
        for patch in patches:
            normal = np.asarray(patch["normal"], dtype=np.float64).reshape((3,))
            if not selected:
                selected.append(patch)
                continue
            is_parallel = any(
                abs(float(np.dot(np.asarray(existing["normal"], dtype=np.float64), normal))) >= parallel_threshold
                for existing in selected
            )
            if is_parallel:
                continue
            orthogonal_to_all = all(
                abs(float(np.dot(np.asarray(existing["normal"], dtype=np.float64), normal))) <= orth_threshold
                for existing in selected
            )
            if orthogonal_to_all:
                selected.append(patch)
            if len(selected) >= 3:
                break
        if not selected:
            return []
        if len(selected) == 1:
            return selected
        selected.sort(
            key=lambda patch: -float(
                sum(
                    abs(
                        float(
                            np.dot(
                                np.asarray(patch["normal"], dtype=np.float64),
                                np.asarray(reference_basis[:, axis_index], dtype=np.float64),
                            )
                        )
                    )
                    for axis_index in range(3)
                )
            )
        )
        return selected

    def _basis_from_face_patches(
        self, patches: list[dict[str, Any]], reference_basis: np.ndarray
    ) -> np.ndarray:
        ref = np.asarray(reference_basis, dtype=np.float64).reshape((3, 3))
        if not patches:
            return ref.copy()
        normals = [np.asarray(patch["normal"], dtype=np.float64).reshape((3,)) for patch in patches]
        if len(normals) >= 3:
            basis = np.stack(normals[:3], axis=1)
            return self._reorder_basis_like(basis, ref)
        if len(normals) == 2:
            axis0 = normals[0]
            axis1 = normals[1]
            axis2 = _normalize_vector(np.cross(axis0, axis1), ref[:, 2])
            basis = np.stack([axis0, axis1, axis2], axis=1)
            return self._reorder_basis_like(basis, ref)
        axis0 = normals[0]
        tangent = ref[:, int(np.argmax([abs(float(np.dot(axis0, ref[:, i]))) for i in range(3)]))]
        tangent = tangent - float(np.dot(tangent, axis0)) * axis0
        axis1 = _normalize_vector(tangent, ref[:, 1])
        axis2 = _normalize_vector(np.cross(axis0, axis1), ref[:, 2])
        basis = np.stack([axis0, axis1, axis2], axis=1)
        return self._reorder_basis_like(basis, ref)

    def _build_box_model_from_face_patches(
        self,
        points: np.ndarray,
        patches: list[dict[str, Any]],
        basis: np.ndarray,
        *,
        source: str,
    ) -> Optional[dict[str, Any]]:
        if not patches:
            return None
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        basis_matrix = self._orthonormalize_box_basis(basis, basis)
        origin_center = np.mean(pts, axis=0)
        local = (pts - origin_center.reshape((1, 3))) @ basis_matrix

        mins = np.full((3,), np.inf, dtype=np.float64)
        maxs = np.full((3,), -np.inf, dtype=np.float64)
        face_support_pos = np.zeros((3,), dtype=np.float64)
        face_support_neg = np.zeros((3,), dtype=np.float64)
        face_visible_pos = np.zeros((3,), dtype=bool)
        face_visible_neg = np.zeros((3,), dtype=bool)
        plane_residuals: list[float] = []
        patch_point_indices: list[np.ndarray] = []
        patch_boundary_total = 0

        for patch in patches:
            indices = np.asarray(patch["indices"], dtype=np.int32).reshape((-1,))
            if indices.size < 8:
                continue
            patch_point_indices.append(indices)
            patch_points = pts[indices]
            patch_local = (patch_points - origin_center.reshape((1, 3))) @ basis_matrix
            patch_normal_local = np.asarray(patch["normal"], dtype=np.float64).reshape((3,)) @ basis_matrix
            axis_index = int(np.argmax(np.abs(patch_normal_local)))
            sign = 1.0 if float(patch_normal_local[axis_index]) >= 0.0 else -1.0
            coord = patch_local[:, axis_index]
            plane_coord = float(np.percentile(coord, 60.0 if sign > 0.0 else 40.0))
            tangent_axes = [axis for axis in range(3) if axis != axis_index]
            face_local_2d = patch_local[:, tangent_axes]
            patch_weights = np.full((patch_points.shape[0],), float(patch.get("weight_sum", patch_points.shape[0])) / max(1, patch_points.shape[0]), dtype=np.float64)
            tangent_min_0, tangent_max_0, tangent_min_1, tangent_max_1, boundary_count = self._estimate_patch_rectangle_bounds(
                face_local_2d,
                patch_weights,
            )
            patch_boundary_total += int(boundary_count)

            mins[tangent_axes[0]] = min(mins[tangent_axes[0]], tangent_min_0)
            maxs[tangent_axes[0]] = max(maxs[tangent_axes[0]], tangent_max_0)
            mins[tangent_axes[1]] = min(mins[tangent_axes[1]], tangent_min_1)
            maxs[tangent_axes[1]] = max(maxs[tangent_axes[1]], tangent_max_1)
            patch_mass = float(patch.get("weight_sum", float(indices.size)))
            if sign > 0.0:
                maxs[axis_index] = max(maxs[axis_index], plane_coord)
                face_support_pos[axis_index] += patch_mass
                face_visible_pos[axis_index] = True
            else:
                mins[axis_index] = min(mins[axis_index], plane_coord)
                face_support_neg[axis_index] += patch_mass
                face_visible_neg[axis_index] = True
            plane_residuals.append(float(np.median(np.abs(coord - plane_coord))))

        if not patch_point_indices:
            return None
        support_indices = np.unique(np.concatenate(patch_point_indices, axis=0))
        support_local = local[support_indices]
        for axis_index in range(3):
            if not np.isfinite(mins[axis_index]):
                mins[axis_index] = float(np.percentile(support_local[:, axis_index], 5.0))
            if not np.isfinite(maxs[axis_index]):
                maxs[axis_index] = float(np.percentile(support_local[:, axis_index], 95.0))

        extents = np.maximum(maxs - mins, self.output_voxel_size_m)
        local_center = 0.5 * (mins + maxs)
        center_world = origin_center + (local_center @ basis_matrix.T)
        visible_face_count = int(np.count_nonzero(face_visible_pos) + np.count_nonzero(face_visible_neg))
        plane_residual = float(np.median(plane_residuals)) if plane_residuals else 0.0
        patch_score = float(sum(float(patch["score"]) for patch in patches))
        score = patch_score + 50.0 * visible_face_count - 800.0 * plane_residual
        return {
            "source": source,
            "origin_center": origin_center.copy(),
            "basis": basis_matrix.copy(),
            "mins": mins.copy(),
            "maxs": maxs.copy(),
            "local_center": local_center.copy(),
            "center_world": center_world.copy(),
            "extents": extents.copy(),
            "score": float(score),
            "plane_residual_m": float(plane_residual),
            "visible_face_count": int(visible_face_count),
            "face_support_pos": face_support_pos.copy(),
            "face_support_neg": face_support_neg.copy(),
            "face_visible_pos": face_visible_pos.copy(),
            "face_visible_neg": face_visible_neg.copy(),
            "face_weight_mean": float(np.mean(np.concatenate([face_support_pos, face_support_neg]))),
            "support_mean": float(np.mean(np.concatenate([face_support_pos, face_support_neg]))),
            "patch_count": int(len(patches)),
            "patch_area_sum_m2": float(sum(float(patch["area"]) for patch in patches)),
            "patch_boundary_count": int(patch_boundary_total),
        }

    def _fit_box_model_from_basis(
        self,
        points: np.ndarray,
        geometry: dict[str, np.ndarray],
        basis_init: np.ndarray,
        *,
        source: str,
    ) -> dict[str, Any]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        normals = np.asarray(geometry["normals"], dtype=np.float64).reshape((-1, 3))
        planarity = np.asarray(geometry["planarity"], dtype=np.float64).reshape((-1,))
        density_quality = np.asarray(geometry["density_quality"], dtype=np.float64).reshape((-1,))
        base_quality = np.clip(0.25 + 0.75 * planarity, 0.05, 1.0) * np.clip(
            density_quality, 0.05, 1.0
        )
        origin_center = np.mean(pts, axis=0)
        basis = self._orthonormalize_box_basis(basis_init, basis_init)
        best_model: Optional[dict[str, Any]] = None

        for _ in range(2):
            local = (pts - origin_center.reshape((1, 3))) @ basis
            local_normals = normals @ basis
            abs_normals = np.abs(local_normals)
            face_axis_score = np.clip(
                base_quality.reshape((-1, 1))
                * np.exp(-0.5 * np.square((1.0 - abs_normals) / 0.22)),
                0.0,
                1.0,
            )
            face_axis = np.argmax(face_axis_score, axis=1)
            axis_best = np.max(face_axis_score, axis=1)
            axis_second = np.partition(face_axis_score, -2, axis=1)[:, -2]
            axis_margin = np.clip(axis_best - axis_second, 0.0, 1.0)
            face_weight = np.clip(axis_best * (0.65 + 0.35 * axis_margin), 0.0, 1.0)

            mins = np.zeros((3,), dtype=np.float64)
            maxs = np.zeros((3,), dtype=np.float64)
            face_support_pos = np.zeros((3,), dtype=np.float64)
            face_support_neg = np.zeros((3,), dtype=np.float64)
            face_visible_pos = np.zeros((3,), dtype=bool)
            face_visible_neg = np.zeros((3,), dtype=bool)
            plane_residuals: list[float] = []

            for axis_index in range(3):
                coord = local[:, axis_index]
                global_lo = float(np.percentile(coord, 2.0))
                global_hi = float(np.percentile(coord, 98.0))
                global_span = max(global_hi - global_lo, self.output_voxel_size_m)
                axis_mask = face_axis == axis_index
                dominant_threshold = max(0.12, 0.25 * float(np.max(face_axis_score[:, axis_index])))
                pos_support = np.where(
                    axis_mask,
                    face_axis_score[:, axis_index]
                    * face_weight
                    * np.clip(local_normals[:, axis_index], 0.0, 1.0),
                    0.0,
                )
                neg_support = np.where(
                    axis_mask,
                    face_axis_score[:, axis_index]
                    * face_weight
                    * np.clip(-local_normals[:, axis_index], 0.0, 1.0),
                    0.0,
                )
                if float(np.max(pos_support)) < dominant_threshold:
                    pos_support = face_axis_score[:, axis_index] * np.clip(local_normals[:, axis_index], 0.0, 1.0)
                if float(np.max(neg_support)) < dominant_threshold:
                    neg_support = face_axis_score[:, axis_index] * np.clip(-local_normals[:, axis_index], 0.0, 1.0)

                pos_mass = float(np.sum(pos_support))
                neg_mass = float(np.sum(neg_support))
                face_support_pos[axis_index] = pos_mass
                face_support_neg[axis_index] = neg_mass
                pos_visible = pos_mass >= max(0.35, 0.05 * float(np.sum(base_quality)))
                neg_visible = neg_mass >= max(0.35, 0.05 * float(np.sum(base_quality)))
                face_visible_pos[axis_index] = pos_visible
                face_visible_neg[axis_index] = neg_visible

                pos_plane = self._weighted_percentile(coord, pos_support, 92.0, global_hi)
                neg_plane = self._weighted_percentile(coord, neg_support, 8.0, global_lo)

                if pos_visible and neg_visible:
                    mins[axis_index] = neg_plane
                    maxs[axis_index] = pos_plane
                elif pos_visible:
                    maxs[axis_index] = pos_plane
                    mins[axis_index] = min(global_lo, pos_plane - global_span)
                elif neg_visible:
                    mins[axis_index] = neg_plane
                    maxs[axis_index] = max(global_hi, neg_plane + global_span)
                else:
                    mins[axis_index] = global_lo
                    maxs[axis_index] = global_hi

                pos_mask = pos_support > 1e-4
                neg_mask = neg_support > 1e-4
                if np.any(pos_mask):
                    plane_residuals.append(
                        float(np.median(np.abs(coord[pos_mask] - maxs[axis_index])))
                    )
                if np.any(neg_mask):
                    plane_residuals.append(
                        float(np.median(np.abs(coord[neg_mask] - mins[axis_index])))
                    )

            extents = np.maximum(maxs - mins, self.output_voxel_size_m)
            local_center = 0.5 * (mins + maxs)
            center_world = origin_center + (local_center @ basis.T)

            visible_face_count = int(np.count_nonzero(face_visible_pos) + np.count_nonzero(face_visible_neg))
            plane_residual = float(np.median(plane_residuals)) if plane_residuals else 0.0
            support_mean = float(np.mean(0.5 * (face_support_pos + face_support_neg)))
            score = (
                1.4 * float(np.mean(face_weight))
                + 0.25 * (visible_face_count / 6.0)
                + 0.2 * support_mean
                - 6.0 * plane_residual
            )

            refined_axes = np.zeros((3, 3), dtype=np.float64)
            for axis_index in range(3):
                axis_vector = np.zeros((3,), dtype=np.float64)
                pos_mask = face_axis_score[:, axis_index] >= max(
                    0.1, 0.25 * float(np.max(face_axis_score[:, axis_index]))
                )
                pos_signed = pos_mask & (local_normals[:, axis_index] > 0.1)
                neg_signed = pos_mask & (local_normals[:, axis_index] < -0.1)
                if np.any(pos_signed):
                    axis_vector += np.sum(
                        normals[pos_signed] * pos_support[pos_signed].reshape((-1, 1)), axis=0
                    )
                if np.any(neg_signed):
                    axis_vector -= np.sum(
                        normals[neg_signed] * neg_support[neg_signed].reshape((-1, 1)), axis=0
                    )
                refined_axes[:, axis_index] = _normalize_vector(axis_vector, basis[:, axis_index])
            basis = self._orthonormalize_box_basis(refined_axes, basis)

            best_model = {
                "source": source,
                "origin_center": origin_center.copy(),
                "basis": basis.copy(),
                "mins": mins.copy(),
                "maxs": maxs.copy(),
                "local_center": local_center.copy(),
                "center_world": center_world.copy(),
                "extents": extents.copy(),
                "score": float(score),
                "plane_residual_m": float(plane_residual),
                "visible_face_count": int(visible_face_count),
                "face_support_pos": face_support_pos.copy(),
                "face_support_neg": face_support_neg.copy(),
                "face_visible_pos": face_visible_pos.copy(),
                "face_visible_neg": face_visible_neg.copy(),
                "face_weight_mean": float(np.mean(face_weight)),
                "support_mean": float(support_mean),
            }

        assert best_model is not None
        return best_model

    def _estimate_box_v1(self, points: np.ndarray) -> dict[str, Any]:
        pts = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        _, pca_basis, _ = self._principal_axis_frame(pts)
        geometry = self._local_surface_geometry(pts)
        patches = self._extract_plane_patches(pts, geometry)
        selected_patches = self._select_box_face_patches(patches, pca_basis)
        if selected_patches:
            patch_basis = self._basis_from_face_patches(selected_patches, pca_basis)
            patch_model = self._build_box_model_from_face_patches(
                pts,
                selected_patches,
                patch_basis,
                source=f"plane_patches_{len(selected_patches)}",
            )
            if patch_model is not None and int(patch_model.get("visible_face_count", 0)) >= 1:
                patch_model["candidate_patch_count"] = int(len(patches))
                return patch_model

        normals = np.asarray(geometry["normals"], dtype=np.float64)
        planarity = np.asarray(geometry["planarity"], dtype=np.float64)
        density_quality = np.asarray(geometry["density_quality"], dtype=np.float64)
        normal_basis = self._candidate_box_basis_from_normals(
            normals, planarity, density_quality, pca_basis
        )

        candidates = [
            ("pca_basis", pca_basis),
            ("normal_basis", normal_basis),
        ]
        best_model: Optional[dict[str, Any]] = None
        for source, basis in candidates:
            model = self._fit_box_model_from_basis(pts, geometry, basis, source=source)
            if best_model is None or float(model["score"]) > float(best_model["score"]):
                best_model = model
        assert best_model is not None
        best_model["candidate_patch_count"] = int(len(patches))
        return best_model

    def _stabilize_box_extents(
        self, model: dict[str, Any], *, label: str, now_sec: float
    ) -> dict[str, Any]:
        stabilized = dict(model)
        mins = np.asarray(model["mins"], dtype=np.float64).reshape((3,))
        maxs = np.asarray(model["maxs"], dtype=np.float64).reshape((3,))
        basis = np.asarray(model["basis"], dtype=np.float64).reshape((3, 3))
        origin_center = np.asarray(model["origin_center"], dtype=np.float64).reshape((3,))
        hold_applied_axes: list[int] = []
        previous = self._last_box_fit_state.get(label)
        age_sec = 0.0
        prev_volume = 0.0

        if previous and self.box_extent_hold_sec > 0.0:
            age_sec = max(0.0, float(now_sec - float(previous.get("timestamp", 0.0))))
            prev_extents = np.asarray(previous.get("extents", np.zeros((3,))), dtype=np.float64).reshape((3,))
            prev_volume = float(np.prod(np.maximum(prev_extents, self.output_voxel_size_m)))
            if age_sec <= self.box_extent_hold_sec:
                prev_center = np.asarray(previous.get("center_world", model["center_world"]), dtype=np.float64).reshape((3,))
                prev_basis = np.asarray(previous.get("basis", basis), dtype=np.float64).reshape((3, 3))
                center_match = (
                    float(np.linalg.norm(prev_center - np.asarray(model["center_world"], dtype=np.float64).reshape((3,))))
                    <= self.box_match_center_m
                )
                axis_match = all(
                    abs(float(np.dot(prev_basis[:, axis_index], basis[:, axis_index])))
                    >= math.cos(math.radians(self.box_match_axis_deg))
                    for axis_index in range(3)
                )
                if center_match and axis_match:
                    prev_mins = np.asarray(previous.get("mins", mins), dtype=np.float64).reshape((3,))
                    prev_maxs = np.asarray(previous.get("maxs", maxs), dtype=np.float64).reshape((3,))
                    for axis_index in range(3):
                        if mins[axis_index] > (prev_mins[axis_index] + self.box_shrink_tolerance_m):
                            mins[axis_index] = prev_mins[axis_index]
                            hold_applied_axes.append(axis_index)
                        if maxs[axis_index] < (prev_maxs[axis_index] - self.box_shrink_tolerance_m):
                            maxs[axis_index] = prev_maxs[axis_index]
                            hold_applied_axes.append(axis_index)

        extents = np.maximum(maxs - mins, self.output_voxel_size_m)
        local_center = 0.5 * (mins + maxs)
        center_world = origin_center + (local_center @ basis.T)
        stabilized["mins"] = mins.copy()
        stabilized["maxs"] = maxs.copy()
        stabilized["extents"] = extents.copy()
        stabilized["local_center"] = local_center.copy()
        stabilized["center_world"] = center_world.copy()
        stabilized["box_hold_applied"] = bool(hold_applied_axes)
        stabilized["box_hold_axes"] = sorted(set(int(axis) for axis in hold_applied_axes))
        stabilized["box_hold_age_sec"] = float(age_sec)
        stabilized["box_hold_prev_volume_m3"] = float(prev_volume)

        self._last_box_fit_state[label] = {
            "timestamp": float(now_sec),
            "center_world": center_world.copy(),
            "origin_center": origin_center.copy(),
            "basis": basis.copy(),
            "mins": mins.copy(),
            "maxs": maxs.copy(),
            "extents": extents.copy(),
        }
        return stabilized

    def _generate_box_surface_points(self, model: dict[str, Any]) -> np.ndarray:
        basis = np.asarray(model["basis"], dtype=np.float64).reshape((3, 3))
        origin_center = np.asarray(model["origin_center"], dtype=np.float64).reshape((3,))
        mins = np.asarray(model["mins"], dtype=np.float64).reshape((3,))
        maxs = np.asarray(model["maxs"], dtype=np.float64).reshape((3,))
        grid = np.linspace(-0.5, 0.5, num=self.box_face_grid, endpoint=True)
        local_center = 0.5 * (mins + maxs)
        extents = np.maximum(maxs - mins, self.output_voxel_size_m)
        face_points: list[np.ndarray] = []
        for fixed_axis in range(3):
            other_axes = [axis for axis in range(3) if axis != fixed_axis]
            for sign in (-0.5, 0.5):
                uv = np.stack(np.meshgrid(grid, grid, indexing="xy"), axis=-1).reshape((-1, 2))
                face = np.zeros((uv.shape[0], 3), dtype=np.float64)
                face[:, fixed_axis] = sign
                face[:, other_axes[0]] = uv[:, 0]
                face[:, other_axes[1]] = uv[:, 1]
                face = face * extents.reshape((1, 3)) + local_center.reshape((1, 3))
                face_points.append(face)
        local_surface = np.concatenate(face_points, axis=0)
        fitted_points = (local_surface @ basis.T) + origin_center.reshape((1, 3))
        return np.asarray(fitted_points, dtype=np.float32)

    def _fit_box(self, points: np.ndarray) -> tuple[np.ndarray, Marker, dict[str, Any]]:
        label = self._normalize_label(self._latest_cloud_target_label or self._latest_semantic_target_label)
        model = self._estimate_box_v1(points)
        model = self._stabilize_box_extents(model, label=label or "box", now_sec=time.time())
        fitted_points = self._generate_box_surface_points(model)
        marker = self._make_box_marker(
            stamp=self._latest_cloud_msg.header.stamp,
            frame_id=str(self._latest_cloud_msg.header.frame_id or ""),
            center_world=np.asarray(model["center_world"], dtype=np.float64),
            basis=np.asarray(model["basis"], dtype=np.float64),
            extents=np.asarray(model["extents"], dtype=np.float64),
        )
        debug = {
            "fit_basis_source": str(model["source"]),
            "fit_extent_x_m": round(float(model["extents"][0]), 4),
            "fit_extent_y_m": round(float(model["extents"][1]), 4),
            "fit_extent_z_m": round(float(model["extents"][2]), 4),
            "fit_plane_residual_m": round(float(model.get("plane_residual_m", 0.0)), 4),
            "fit_visible_face_count": int(model.get("visible_face_count", 0)),
            "fit_patch_count": int(model.get("patch_count", 0)),
            "fit_candidate_patch_count": int(model.get("candidate_patch_count", 0)),
            "fit_patch_area_sum_m2": round(float(model.get("patch_area_sum_m2", 0.0)), 4),
            "fit_patch_boundary_count": int(model.get("patch_boundary_count", 0)),
            "fit_face_weight_mean": round(float(model.get("face_weight_mean", 0.0)), 4),
            "fit_face_support_x_pos": round(float(model["face_support_pos"][0]), 4),
            "fit_face_support_x_neg": round(float(model["face_support_neg"][0]), 4),
            "fit_face_support_y_pos": round(float(model["face_support_pos"][1]), 4),
            "fit_face_support_y_neg": round(float(model["face_support_neg"][1]), 4),
            "fit_face_support_z_pos": round(float(model["face_support_pos"][2]), 4),
            "fit_face_support_z_neg": round(float(model["face_support_neg"][2]), 4),
            "fit_box_hold_applied": bool(model.get("box_hold_applied", False)),
            "fit_box_hold_axes": list(model.get("box_hold_axes", [])),
            "fit_box_hold_age_sec": round(float(model.get("box_hold_age_sec", 0.0)), 4),
        }
        return np.asarray(fitted_points, dtype=np.float32), marker, debug

    def _make_delete_marker(self, *, stamp, frame_id: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "target_primitive"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def _make_axis_marker(
        self,
        *,
        stamp,
        frame_id: str,
        start: np.ndarray,
        end: np.ndarray,
        ns: str,
        marker_id: int,
        color_rgb: tuple[float, float, float],
        scale_x: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = float(scale_x)
        marker.scale.y = float(scale_x * 1.8)
        marker.scale.z = float(scale_x * 2.4)
        marker.color.a = float(self.marker_alpha)
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.points = []
        start_point = start.reshape((3,))
        end_point = end.reshape((3,))
        marker.points.append(
            Point(x=float(start_point[0]), y=float(start_point[1]), z=float(start_point[2]))
        )
        marker.points.append(
            Point(x=float(end_point[0]), y=float(end_point[1]), z=float(end_point[2]))
        )
        return marker

    def _make_cylinder_marker(
        self,
        *,
        stamp,
        frame_id: str,
        basis: np.ndarray,
        center_world: np.ndarray,
        radius: float,
        height: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "target_primitive"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = float(max(0.002, 2.0 * radius))
        marker.scale.y = float(max(0.002, 2.0 * radius))
        marker.scale.z = float(max(0.002, height))
        marker.color.a = float(self.marker_alpha)
        marker.color.r = 0.18
        marker.color.g = 0.92
        marker.color.b = 0.95
        rotation = np.asarray(basis, dtype=np.float64)
        quaternion = _rotation_matrix_to_quaternion_xyzw(rotation)
        marker.pose.position.x = float(center_world[0])
        marker.pose.position.y = float(center_world[1])
        marker.pose.position.z = float(center_world[2])
        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])
        return marker

    def _make_box_marker(
        self,
        *,
        stamp,
        frame_id: str,
        center_world: np.ndarray,
        basis: np.ndarray,
        extents: np.ndarray,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "target_primitive"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = float(max(0.002, extents[0]))
        marker.scale.y = float(max(0.002, extents[1]))
        marker.scale.z = float(max(0.002, extents[2]))
        marker.color.a = float(self.marker_alpha)
        marker.color.r = 0.96
        marker.color.g = 0.78
        marker.color.b = 0.22
        quaternion = _rotation_matrix_to_quaternion_xyzw(np.asarray(basis, dtype=np.float64))
        marker.pose.position.x = float(center_world[0])
        marker.pose.position.y = float(center_world[1])
        marker.pose.position.z = float(center_world[2])
        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])
        return marker

    def _make_sphere_marker(
        self,
        *,
        stamp,
        frame_id: str,
        center_world: np.ndarray,
        radius: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "target_primitive"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = float(max(0.002, 2.0 * radius))
        marker.scale.y = float(max(0.002, 2.0 * radius))
        marker.scale.z = float(max(0.002, 2.0 * radius))
        marker.color.a = float(self.marker_alpha)
        marker.color.r = 0.52
        marker.color.g = 0.92
        marker.color.b = 0.42
        marker.pose.position.x = float(center_world[0])
        marker.pose.position.y = float(center_world[1])
        marker.pose.position.z = float(center_world[2])
        marker.pose.orientation.w = 1.0
        return marker

    def _make_line_list_marker(
        self,
        *,
        stamp,
        frame_id: str,
        line_points: list[np.ndarray],
        color_rgb: tuple[float, float, float],
        scale_x: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "target_primitive"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = float(scale_x)
        marker.color.a = float(self.marker_alpha)
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.points = []
        for point_value in line_points:
            point_arr = np.asarray(point_value, dtype=np.float64).reshape((3,))
            marker.points.append(
                Point(x=float(point_arr[0]), y=float(point_arr[1]), z=float(point_arr[2]))
            )
        return marker

    def _make_hemisphere_marker(
        self,
        *,
        stamp,
        frame_id: str,
        center_world: np.ndarray,
        basis: np.ndarray,
        radius: float,
    ) -> Marker:
        rotation = np.asarray(basis, dtype=np.float64).reshape((3, 3))
        center = np.asarray(center_world, dtype=np.float64).reshape((3,))
        theta = np.linspace(0.0, 2.0 * math.pi, num=20, endpoint=False)
        ring_local = np.column_stack(
            [radius * np.cos(theta), radius * np.sin(theta), np.zeros_like(theta)]
        )
        ring_world = (ring_local @ rotation.T) + center.reshape((1, 3))
        line_points: list[np.ndarray] = []
        for index in range(ring_world.shape[0]):
            line_points.append(ring_world[index])
            line_points.append(ring_world[(index + 1) % ring_world.shape[0]])
        pole = center + rotation[:, 2] * radius
        for index in range(0, ring_world.shape[0], 4):
            line_points.append(ring_world[index])
            line_points.append(pole)
        return self._make_line_list_marker(
            stamp=stamp,
            frame_id=frame_id,
            line_points=line_points,
            color_rgb=(0.42, 0.88, 0.52),
            scale_x=0.0025,
        )

    def _make_frustum_marker(
        self,
        *,
        stamp,
        frame_id: str,
        basis: np.ndarray,
        centerline_anchor: np.ndarray,
        h_min: float,
        h_max: float,
        lower_radius: float,
        upper_radius: float,
    ) -> Marker:
        rotation = np.asarray(basis, dtype=np.float64).reshape((3, 3))
        anchor = np.asarray(centerline_anchor, dtype=np.float64).reshape((3,))
        axis = rotation[:, 2]
        lateral_u = rotation[:, 0]
        lateral_v = rotation[:, 1]
        theta = np.linspace(0.0, 2.0 * math.pi, num=16, endpoint=False)
        lower_ring = (
            anchor.reshape((1, 3))
            + h_min * axis.reshape((1, 3))
            + lower_radius * np.cos(theta).reshape((-1, 1)) * lateral_u.reshape((1, 3))
            + lower_radius * np.sin(theta).reshape((-1, 1)) * lateral_v.reshape((1, 3))
        )
        upper_ring = (
            anchor.reshape((1, 3))
            + h_max * axis.reshape((1, 3))
            + upper_radius * np.cos(theta).reshape((-1, 1)) * lateral_u.reshape((1, 3))
            + upper_radius * np.sin(theta).reshape((-1, 1)) * lateral_v.reshape((1, 3))
        )
        line_points: list[np.ndarray] = []
        for index in range(lower_ring.shape[0]):
            next_index = (index + 1) % lower_ring.shape[0]
            line_points.append(lower_ring[index])
            line_points.append(lower_ring[next_index])
            line_points.append(upper_ring[index])
            line_points.append(upper_ring[next_index])
        for index in range(0, lower_ring.shape[0], 2):
            line_points.append(lower_ring[index])
            line_points.append(upper_ring[index])
        return self._make_line_list_marker(
            stamp=stamp,
            frame_id=frame_id,
            line_points=line_points,
            color_rgb=(0.95, 0.74, 0.22),
            scale_x=0.0025,
        )

    def _process_latest(self) -> None:
        if not self.enabled:
            return

        now_sec = time.time()
        detection = self._latest_detection
        cloud_msg = self._latest_cloud_msg
        if detection is None or cloud_msg is None:
            return
        if (now_sec - float(self._latest_detection_ts or 0.0)) > self.detection_stale_sec:
            return
        if (now_sec - float(self._latest_cloud_ts or 0.0)) > self.cloud_stale_sec:
            return
        if not bool(detection.accepted):
            return

        cloud_target_label = self._normalize_label(self._latest_cloud_target_label)
        semantic_label = self._normalize_label(self._latest_semantic_target_label)
        detection_label = self._normalize_label(detection.target_label)
        label = cloud_target_label or semantic_label or detection_label
        raw_points = self._read_xyz_points(cloud_msg)
        if raw_points.shape[0] < self.fit_min_points:
            return

        stamp = cloud_msg.header.stamp
        cloud_key = (
            int(getattr(stamp, "sec", 0)),
            int(getattr(stamp, "nanosec", 0)),
            int(raw_points.shape[0]),
            label,
        )
        stamp_sec = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9
        self._append_history(points=raw_points, label=label, stamp_sec=stamp_sec, cloud_key=cloud_key)
        current_centroid = np.mean(raw_points, axis=0, dtype=np.float64).astype(np.float32)
        history_points = self._select_history_points(label, current_centroid, now_sec)
        merged_points = history_points if history_points.shape[0] >= raw_points.shape[0] else raw_points
        merged_points = voxel_downsample(merged_points, self.output_voxel_size_m)

        strategy = self._strategy_for_label(label)
        marker = self._make_delete_marker(stamp=stamp, frame_id=str(cloud_msg.header.frame_id or ""))
        fit_debug: dict[str, Any] = {}
        try:
            if strategy == "cylinder":
                fitted_points, marker, fit_debug = self._fit_cylinder(merged_points)
            elif strategy == "sphere":
                fitted_points, marker, fit_debug = self._fit_sphere(merged_points)
            elif strategy == "hemisphere":
                fitted_points, marker, fit_debug = self._fit_hemisphere(merged_points)
            elif strategy == "frustum":
                fitted_points, marker, fit_debug = self._fit_frustum(merged_points)
            elif strategy == "axis_profile":
                fitted_points, marker, fit_debug = self._fit_axis_profile(merged_points)
            elif strategy == "box":
                fitted_points, marker, fit_debug = self._fit_box(merged_points)
            else:
                fitted_points = merged_points
        except Exception as exc:  # noqa: BLE001
            fitted_points = merged_points
            strategy = f"{strategy}_fallback"
            fit_debug["fit_error"] = str(exc)

        fitted_points = voxel_downsample(fitted_points, self.output_voxel_size_m)
        self.fitted_cloud_pub.publish(
            make_xyz_cloud(
                np.asarray(fitted_points, dtype=np.float32),
                str(cloud_msg.header.frame_id or ""),
                stamp,
            )
        )
        self.fitted_marker_pub.publish(marker)

        debug_payload = {
            "status": "ok",
            "target_label": label,
            "cloud_target_label": cloud_target_label,
            "semantic_label": semantic_label,
            "detection_label": detection_label,
            "strategy": strategy,
            "raw_points": int(raw_points.shape[0]),
            "merged_points": int(merged_points.shape[0]),
            "fitted_points": int(fitted_points.shape[0]),
            "history_frames": int(len(self._history)),
        }
        debug_payload.update(fit_debug)
        summary = compact_json(debug_payload)
        debug_msg = String()
        debug_msg.data = summary
        self.debug_pub.publish(debug_msg)

        if summary != self._last_summary and (now_sec - self._last_log_time) >= self.log_interval_sec:
            self._last_summary = summary
            self._last_log_time = now_sec
            self.get_logger().info(
                "primitive fit: "
                f"label={label or 'unknown'} strategy={strategy} "
                f"raw={raw_points.shape[0]} merged={merged_points.shape[0]} fitted={fitted_points.shape[0]}"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PrimitiveFitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
