from __future__ import annotations

import json
import math
import time
from typing import Any, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from tactile_vision.modular_common import compact_json, make_xyz_cloud, voxel_downsample


def point_cloud2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    if int(msg.point_step) < 12 or int(msg.width) <= 0:
        return np.empty((0, 3), dtype=np.float32)
    point_step_floats = max(3, int(msg.point_step) // 4)
    points = np.frombuffer(msg.data, dtype=np.float32)
    if points.size < int(msg.width) * point_step_floats:
        return np.empty((0, 3), dtype=np.float32)
    points = points.reshape((-1, point_step_floats))
    return points[:, :3].astype(np.float32)


def clamp01(value: float) -> float:
    return float(max(0.0, min(1.0, value)))


class GraspInputCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_input_cloud_node")

        self.declare_parameter("raw_cloud_topic", "/perception/target_cloud")
        self.declare_parameter("fitted_cloud_topic", "/perception/target_cloud_fitted")
        self.declare_parameter("fit_debug_topic", "/perception/target_fit_debug")
        self.declare_parameter("output_cloud_topic", "/perception/target_cloud_for_graspgen")
        self.declare_parameter("output_debug_topic", "/perception/target_cloud_for_graspgen_debug")
        self.declare_parameter("enabled", True)
        self.declare_parameter("publish_rate_hz", 6.0)
        self.declare_parameter("stale_sec", 4.0)
        self.declare_parameter("supported_strategies", ["cylinder", "box"])
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("raw_to_fit_median_max_m", 0.008)
        self.declare_parameter("raw_to_fit_p90_max_m", 0.016)
        self.declare_parameter("fit_to_raw_supported_max_m", 0.006)
        self.declare_parameter("fit_to_raw_completion_max_m", 0.028)
        self.declare_parameter("completion_ratio", 0.45)
        self.declare_parameter("completion_max_points", 240)
        self.declare_parameter("output_voxel_size_m", 0.003)
        self.declare_parameter("hold_sec", 1.2)
        self.declare_parameter("hold_match_center_m", 0.03)
        self.declare_parameter("hold_shrink_ratio", 0.78)
        self.declare_parameter("freeze_enabled", True)
        self.declare_parameter("freeze_frames_required", 4)
        self.declare_parameter("freeze_confidence_min", 0.62)
        self.declare_parameter("freeze_match_center_m", 0.018)
        self.declare_parameter("freeze_unlock_center_m", 0.045)
        self.declare_parameter("freeze_radius_tol_m", 0.0035)
        self.declare_parameter("freeze_height_tol_m", 0.008)
        self.declare_parameter("freeze_extent_tol_m", 0.008)
        self.declare_parameter("log_interval_sec", 8.0)

        self.raw_cloud_topic = str(self.get_parameter("raw_cloud_topic").value)
        self.fitted_cloud_topic = str(self.get_parameter("fitted_cloud_topic").value)
        self.fit_debug_topic = str(self.get_parameter("fit_debug_topic").value)
        self.output_cloud_topic = str(self.get_parameter("output_cloud_topic").value)
        self.output_debug_topic = str(self.get_parameter("output_debug_topic").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.publish_rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.stale_sec = max(0.1, float(self.get_parameter("stale_sec").value))
        self.supported_strategies = {
            str(value).strip().lower()
            for value in list(self.get_parameter("supported_strategies").value or [])
            if str(value).strip()
        }
        self.min_confidence = clamp01(float(self.get_parameter("min_confidence").value))
        self.raw_to_fit_median_max_m = max(
            0.001, float(self.get_parameter("raw_to_fit_median_max_m").value)
        )
        self.raw_to_fit_p90_max_m = max(
            self.raw_to_fit_median_max_m,
            float(self.get_parameter("raw_to_fit_p90_max_m").value),
        )
        self.fit_to_raw_supported_max_m = max(
            0.001, float(self.get_parameter("fit_to_raw_supported_max_m").value)
        )
        self.fit_to_raw_completion_max_m = max(
            self.fit_to_raw_supported_max_m,
            float(self.get_parameter("fit_to_raw_completion_max_m").value),
        )
        self.completion_ratio = max(
            0.0, min(1.0, float(self.get_parameter("completion_ratio").value))
        )
        self.completion_max_points = max(
            0, int(self.get_parameter("completion_max_points").value)
        )
        self.output_voxel_size_m = max(
            0.0, float(self.get_parameter("output_voxel_size_m").value)
        )
        self.hold_sec = max(0.0, float(self.get_parameter("hold_sec").value))
        self.hold_match_center_m = max(
            0.0, float(self.get_parameter("hold_match_center_m").value)
        )
        self.hold_shrink_ratio = max(
            0.05, min(1.0, float(self.get_parameter("hold_shrink_ratio").value))
        )
        self.freeze_enabled = bool(self.get_parameter("freeze_enabled").value)
        self.freeze_frames_required = max(
            2, int(self.get_parameter("freeze_frames_required").value)
        )
        self.freeze_confidence_min = clamp01(
            float(self.get_parameter("freeze_confidence_min").value)
        )
        self.freeze_match_center_m = max(
            0.0, float(self.get_parameter("freeze_match_center_m").value)
        )
        self.freeze_unlock_center_m = max(
            self.freeze_match_center_m,
            float(self.get_parameter("freeze_unlock_center_m").value),
        )
        self.freeze_radius_tol_m = max(
            0.0, float(self.get_parameter("freeze_radius_tol_m").value)
        )
        self.freeze_height_tol_m = max(
            0.0, float(self.get_parameter("freeze_height_tol_m").value)
        )
        self.freeze_extent_tol_m = max(
            0.0, float(self.get_parameter("freeze_extent_tol_m").value)
        )
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._latest_raw_msg: Optional[PointCloud2] = None
        self._latest_raw_ts = 0.0
        self._latest_fitted_msg: Optional[PointCloud2] = None
        self._latest_fitted_ts = 0.0
        self._latest_fit_debug: dict[str, Any] = {}
        self._latest_fit_debug_ts = 0.0
        self._last_output_state: dict[str, Any] = {}
        self._freeze_candidate_state: dict[str, Any] = {}
        self._frozen_output_state: dict[str, Any] = {}
        self._last_summary = ""
        self._last_log_time = 0.0

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
            PointCloud2, self.raw_cloud_topic, self._on_raw_cloud, qos_reliable
        )
        self.create_subscription(
            PointCloud2, self.fitted_cloud_topic, self._on_fitted_cloud, qos_reliable
        )
        self.create_subscription(String, self.fit_debug_topic, self._on_fit_debug, qos_reliable)
        self.output_cloud_pub = self.create_publisher(
            PointCloud2, self.output_cloud_topic, qos_latched
        )
        self.output_debug_pub = self.create_publisher(String, self.output_debug_topic, qos_reliable)
        self.create_timer(1.0 / self.publish_rate_hz, self._process_latest)

        self.get_logger().info(
            "grasp_input_cloud_node started: "
            f"raw={self.raw_cloud_topic}, fitted={self.fitted_cloud_topic}, "
            f"output={self.output_cloud_topic}"
        )

    def _on_raw_cloud(self, msg: PointCloud2) -> None:
        self._latest_raw_msg = msg
        self._latest_raw_ts = time.time()

    def _on_fitted_cloud(self, msg: PointCloud2) -> None:
        self._latest_fitted_msg = msg
        self._latest_fitted_ts = time.time()

    def _on_fit_debug(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data or "{}"))
        except Exception:  # noqa: BLE001
            return
        self._latest_fit_debug = dict(payload)
        self._latest_fit_debug_ts = time.time()

    def _stale(self, timestamp: float, now_sec: float) -> bool:
        return (now_sec - float(timestamp or 0.0)) > self.stale_sec

    def _nearest_distance_stats(
        self, source_points: np.ndarray, target_points: np.ndarray
    ) -> tuple[float, float]:
        src = np.asarray(source_points, dtype=np.float32).reshape((-1, 3))
        dst = np.asarray(target_points, dtype=np.float32).reshape((-1, 3))
        if src.shape[0] <= 0 or dst.shape[0] <= 0:
            return 999.0, 999.0
        min_dist_sq = np.full((src.shape[0],), np.inf, dtype=np.float32)
        chunk_size = max(64, min(512, 500000 // max(1, dst.shape[0])))
        for start in range(0, src.shape[0], chunk_size):
            chunk = src[start : start + chunk_size]
            diff = chunk[:, None, :] - dst[None, :, :]
            dist_sq = np.sum(diff * diff, axis=2)
            min_dist_sq[start : start + chunk.shape[0]] = np.min(dist_sq, axis=1)
        min_dist = np.sqrt(np.maximum(min_dist_sq, 0.0))
        return float(np.median(min_dist)), float(np.percentile(min_dist, 90.0))

    def _fit_to_raw_min_distances(self, fit_points: np.ndarray, raw_points: np.ndarray) -> np.ndarray:
        fit = np.asarray(fit_points, dtype=np.float32).reshape((-1, 3))
        raw = np.asarray(raw_points, dtype=np.float32).reshape((-1, 3))
        if fit.shape[0] <= 0 or raw.shape[0] <= 0:
            return np.empty((0,), dtype=np.float32)
        min_dist_sq = np.full((fit.shape[0],), np.inf, dtype=np.float32)
        chunk_size = max(64, min(384, 400000 // max(1, raw.shape[0])))
        for start in range(0, fit.shape[0], chunk_size):
            chunk = fit[start : start + chunk_size]
            diff = chunk[:, None, :] - raw[None, :, :]
            dist_sq = np.sum(diff * diff, axis=2)
            min_dist_sq[start : start + chunk.shape[0]] = np.min(dist_sq, axis=1)
        return np.sqrt(np.maximum(min_dist_sq, 0.0))

    def _strategy_confidence(
        self,
        strategy: str,
        debug: dict[str, Any],
        raw_to_fit_median_m: float,
        raw_to_fit_p90_m: float,
        fit_to_raw_p50_m: float,
    ) -> float:
        match_term = 0.6 * clamp01(1.0 - (raw_to_fit_median_m / self.raw_to_fit_median_max_m))
        match_term += 0.4 * clamp01(1.0 - (raw_to_fit_p90_m / self.raw_to_fit_p90_max_m))
        completion_term = clamp01(
            1.0 - (fit_to_raw_p50_m / self.fit_to_raw_completion_max_m)
        )
        if strategy == "cylinder":
            support_term = clamp01(float(debug.get("fit_side_support", 0.0)) / 0.35)
            coverage_term = clamp01(float(debug.get("fit_support_coverage", 0.0)) / 0.55)
            radial_term = clamp01(1.0 - (float(debug.get("fit_radial_error_m", 1.0)) / 0.0065))
            return (
                0.35 * match_term
                + 0.20 * completion_term
                + 0.20 * support_term
                + 0.15 * coverage_term
                + 0.10 * radial_term
            )
        if strategy == "box":
            face_term = clamp01(float(debug.get("fit_visible_face_count", 0.0)) / 3.0)
            patch_term = clamp01(float(debug.get("fit_patch_count", 0.0)) / 3.0)
            plane_term = clamp01(
                1.0 - (float(debug.get("fit_plane_residual_m", 1.0)) / 0.0055)
            )
            return (
                0.40 * match_term
                + 0.20 * completion_term
                + 0.20 * face_term
                + 0.10 * patch_term
                + 0.10 * plane_term
            )
        return 0.0

    def _select_completion_points(
        self,
        raw_points: np.ndarray,
        fitted_points: np.ndarray,
        confidence: float,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        raw = np.asarray(raw_points, dtype=np.float32).reshape((-1, 3))
        fit = np.asarray(fitted_points, dtype=np.float32).reshape((-1, 3))
        fit_to_raw = self._fit_to_raw_min_distances(fit, raw)
        if fit_to_raw.size <= 0:
            return np.empty((0, 3), dtype=np.float32), {
                "fit_to_raw_p50_m": 999.0,
                "fit_to_raw_p90_m": 999.0,
                "completion_candidates": 0,
                "completion_points_added": 0,
            }
        supported_mask = fit_to_raw <= self.fit_to_raw_supported_max_m
        completion_mask = (
            (fit_to_raw > self.fit_to_raw_supported_max_m)
            & (fit_to_raw <= self.fit_to_raw_completion_max_m)
        )
        completion = fit[completion_mask]
        completion = voxel_downsample(completion, self.output_voxel_size_m)
        target_count = int(
            min(
                self.completion_max_points,
                round(raw.shape[0] * self.completion_ratio * (0.55 + 0.45 * confidence)),
            )
        )
        if target_count > 0 and completion.shape[0] > target_count:
            indices = np.linspace(
                0.0,
                float(completion.shape[0] - 1),
                num=target_count,
                endpoint=True,
            ).round().astype(np.int32)
            completion = completion[np.unique(indices)]
        return completion.astype(np.float32), {
            "fit_to_raw_p50_m": float(np.percentile(fit_to_raw, 50.0)),
            "fit_to_raw_p90_m": float(np.percentile(fit_to_raw, 90.0)),
            "supported_fit_points": int(np.count_nonzero(supported_mask)),
            "completion_candidates": int(np.count_nonzero(completion_mask)),
            "completion_points_added": int(completion.shape[0]),
        }

    def _should_hold_previous(
        self,
        *,
        label: str,
        strategy: str,
        raw_centroid: np.ndarray,
        candidate_points: np.ndarray,
        confidence: float,
        mode: str,
        now_sec: float,
    ) -> bool:
        previous = self._last_output_state
        if not previous or self.hold_sec <= 0.0:
            return False
        if (now_sec - float(previous.get("timestamp", 0.0))) > self.hold_sec:
            return False
        if str(previous.get("label", "")) != str(label) or str(previous.get("strategy", "")) != str(strategy):
            return False
        previous_centroid = np.asarray(previous.get("raw_centroid", raw_centroid), dtype=np.float32).reshape((3,))
        if float(np.linalg.norm(previous_centroid - raw_centroid)) > self.hold_match_center_m:
            return False
        if mode != "fused" and str(previous.get("mode", "")) == "fused":
            return True
        previous_count = int(previous.get("point_count", 0))
        candidate_count = int(candidate_points.shape[0])
        if (
            str(previous.get("mode", "")) == "fused"
            and previous_count > 0
            and candidate_count < int(self.hold_shrink_ratio * previous_count)
            and confidence < max(self.min_confidence + 0.08, float(previous.get("confidence", 0.0)))
        ):
            return True
        return False

    def _shape_descriptor(self, strategy: str, debug: dict[str, Any]) -> np.ndarray:
        if strategy == "cylinder":
            return np.asarray(
                [
                    float(debug.get("fit_radius_m", 0.0)),
                    float(debug.get("fit_height_m", 0.0)),
                ],
                dtype=np.float32,
            )
        if strategy == "box":
            return np.asarray(
                [
                    float(debug.get("fit_extent_x_m", 0.0)),
                    float(debug.get("fit_extent_y_m", 0.0)),
                    float(debug.get("fit_extent_z_m", 0.0)),
                ],
                dtype=np.float32,
            )
        return np.zeros((0,), dtype=np.float32)

    def _descriptor_close(self, strategy: str, left: np.ndarray, right: np.ndarray) -> bool:
        lhs = np.asarray(left, dtype=np.float32).reshape((-1,))
        rhs = np.asarray(right, dtype=np.float32).reshape((-1,))
        if lhs.shape != rhs.shape or lhs.size <= 0:
            return False
        delta = np.abs(lhs - rhs)
        if strategy == "cylinder":
            return bool(
                delta[0] <= self.freeze_radius_tol_m
                and delta[1] <= self.freeze_height_tol_m
            )
        if strategy == "box":
            return bool(float(np.max(delta)) <= self.freeze_extent_tol_m)
        return False

    def _reset_freeze_candidate(self) -> None:
        self._freeze_candidate_state = {}

    def _update_freeze_candidate(
        self,
        *,
        label: str,
        strategy: str,
        raw_centroid: np.ndarray,
        descriptor: np.ndarray,
        confidence: float,
        output_points: np.ndarray,
        now_sec: float,
    ) -> tuple[int, bool]:
        if not self.freeze_enabled:
            self._reset_freeze_candidate()
            return 0, False
        if strategy not in self.supported_strategies or confidence < self.freeze_confidence_min:
            self._reset_freeze_candidate()
            return 0, False

        candidate = self._freeze_candidate_state
        stable = False
        if candidate:
            same_label = str(candidate.get("label", "")) == str(label)
            same_strategy = str(candidate.get("strategy", "")) == str(strategy)
            center_close = (
                float(
                    np.linalg.norm(
                        np.asarray(candidate.get("raw_centroid", raw_centroid), dtype=np.float32).reshape((3,))
                        - raw_centroid
                    )
                )
                <= self.freeze_match_center_m
            )
            descriptor_close = self._descriptor_close(
                strategy,
                descriptor,
                np.asarray(candidate.get("descriptor", descriptor), dtype=np.float32),
            )
            stable = same_label and same_strategy and center_close and descriptor_close

        if stable:
            count = int(self._freeze_candidate_state.get("stable_count", 0)) + 1
        else:
            count = 1

        self._freeze_candidate_state = {
            "timestamp": float(now_sec),
            "label": label,
            "strategy": strategy,
            "stable_count": int(count),
            "raw_centroid": np.asarray(raw_centroid, dtype=np.float32).copy(),
            "descriptor": np.asarray(descriptor, dtype=np.float32).copy(),
            "confidence": float(confidence),
            "points": np.asarray(output_points, dtype=np.float32).copy(),
            "point_count": int(output_points.shape[0]),
        }
        should_freeze = int(count) >= self.freeze_frames_required
        return int(count), bool(should_freeze)

    def _should_keep_frozen(
        self,
        *,
        label: str,
        strategy: str,
        raw_centroid: np.ndarray,
        descriptor: np.ndarray,
        now_sec: float,
    ) -> bool:
        if not self._frozen_output_state:
            return False
        frozen = self._frozen_output_state
        if str(frozen.get("label", "")) != str(label):
            return False
        if str(frozen.get("strategy", "")) != str(strategy):
            return False
        frozen_centroid = np.asarray(
            frozen.get("raw_centroid", raw_centroid), dtype=np.float32
        ).reshape((3,))
        if float(np.linalg.norm(frozen_centroid - raw_centroid)) > self.freeze_unlock_center_m:
            return False
        frozen_descriptor = np.asarray(
            frozen.get("descriptor", descriptor), dtype=np.float32
        ).reshape((-1,))
        if descriptor.size > 0 and not self._descriptor_close(strategy, descriptor, frozen_descriptor):
            wider_tolerance_ok = False
            delta = np.abs(descriptor - frozen_descriptor)
            if strategy == "cylinder":
                wider_tolerance_ok = bool(
                    delta[0] <= (2.0 * self.freeze_radius_tol_m)
                    and delta[1] <= (2.0 * self.freeze_height_tol_m)
                )
            elif strategy == "box":
                wider_tolerance_ok = bool(float(np.max(delta)) <= (2.0 * self.freeze_extent_tol_m))
            if not wider_tolerance_ok:
                return False
        return True

    def _set_frozen_output(
        self,
        *,
        label: str,
        strategy: str,
        raw_centroid: np.ndarray,
        descriptor: np.ndarray,
        confidence: float,
        output_points: np.ndarray,
        frame_id: str,
        now_sec: float,
    ) -> None:
        self._frozen_output_state = {
            "timestamp": float(now_sec),
            "frozen_since": float(now_sec),
            "label": label,
            "strategy": strategy,
            "raw_centroid": np.asarray(raw_centroid, dtype=np.float32).copy(),
            "descriptor": np.asarray(descriptor, dtype=np.float32).copy(),
            "confidence": float(confidence),
            "frame_id": str(frame_id or ""),
            "points": np.asarray(output_points, dtype=np.float32).copy(),
            "point_count": int(output_points.shape[0]),
        }

    def _clear_frozen_output(self) -> None:
        self._frozen_output_state = {}

    def _publish_cached_output(
        self,
        *,
        points: np.ndarray,
        frame_id: str,
        debug_payload: dict[str, Any],
    ) -> None:
        self.output_cloud_pub.publish(
            make_xyz_cloud(
                np.asarray(points, dtype=np.float32).reshape((-1, 3)),
                str(frame_id or ""),
                self.get_clock().now().to_msg(),
            )
        )
        debug_msg = String()
        debug_msg.data = compact_json(debug_payload)
        self.output_debug_pub.publish(debug_msg)

    def _process_latest(self) -> None:
        if not self.enabled:
            return
        now_sec = time.time()
        raw_msg = self._latest_raw_msg
        if self._stale(self._latest_raw_ts, now_sec):
            cached = self._frozen_output_state or self._last_output_state
            if not cached:
                return
            cached_points = np.asarray(cached.get("points", np.empty((0, 3), dtype=np.float32)), dtype=np.float32)
            if cached_points.size <= 0:
                return
            cached_points = cached_points.reshape((-1, 3))
            mode = "frozen_cached" if self._frozen_output_state else "held_cached"
            debug_payload = {
                "status": "ok",
                "target_label": str(cached.get("label", "")),
                "strategy": str(cached.get("strategy", "")),
                "mode": mode,
                "fit_accepted": bool(self._frozen_output_state or self._last_output_state),
                "hold_applied": bool(not self._frozen_output_state),
                "freeze_state": "frozen" if self._frozen_output_state else "locking",
                "freeze_stable_count": int(self._freeze_candidate_state.get("stable_count", 0)),
                "freeze_applied": bool(self._frozen_output_state),
                "fit_confidence": round(float(cached.get("confidence", 0.0)), 4),
                "raw_points": 0,
                "fitted_points": 0,
                "output_points": int(cached_points.shape[0]),
                "source_stale": True,
            }
            self._publish_cached_output(
                points=cached_points,
                frame_id=str(cached.get("frame_id", "")),
                debug_payload=debug_payload,
            )
            return
        fit_msg = self._latest_fitted_msg
        fit_debug = dict(self._latest_fit_debug)

        raw_points = point_cloud2_to_xyz_array(raw_msg)
        fitted_points = (
            point_cloud2_to_xyz_array(fit_msg)
            if fit_msg is not None
            else np.empty((0, 3), dtype=np.float32)
        )
        if raw_points.shape[0] <= 0:
            return
        raw_centroid = np.mean(raw_points, axis=0, dtype=np.float64).astype(np.float32)

        fit_available = (
            fit_msg is not None
            and not self._stale(self._latest_fitted_ts, now_sec)
            and bool(fit_debug)
            and not self._stale(self._latest_fit_debug_ts, now_sec)
            and fitted_points.shape[0] > 0
        )

        if not fit_available:
            output_points = raw_points.astype(np.float32)
            freeze_state = "live"
            freeze_stable_count = 0
            freeze_applied = False
            if self._frozen_output_state:
                frozen_label = str(self._frozen_output_state.get("label", ""))
                if frozen_label:
                    output_points = np.asarray(
                        self._frozen_output_state.get("points", output_points), dtype=np.float32
                    ).reshape((-1, 3))
                    freeze_state = "frozen"
                    freeze_applied = True
            debug_payload = {
                "status": "ok",
                "target_label": "",
                "strategy": "",
                "mode": "raw_passthrough",
                "fit_accepted": False,
                "hold_applied": False,
                "freeze_state": freeze_state,
                "freeze_stable_count": int(freeze_stable_count),
                "freeze_applied": bool(freeze_applied),
                "fit_confidence": 0.0,
                "raw_points": int(raw_points.shape[0]),
                "fitted_points": int(fitted_points.shape[0]),
                "output_points": int(output_points.shape[0]),
            }
            self.output_cloud_pub.publish(
                make_xyz_cloud(output_points, str(raw_msg.header.frame_id or ""), raw_msg.header.stamp)
            )
            debug_msg = String()
            debug_msg.data = compact_json(debug_payload)
            self.output_debug_pub.publish(debug_msg)
            return

        label = str(fit_debug.get("target_label", "") or "").strip().lower()
        strategy = str(fit_debug.get("strategy", "") or "").strip().lower()
        descriptor = self._shape_descriptor(strategy, fit_debug)

        raw_to_fit_median_m, raw_to_fit_p90_m = self._nearest_distance_stats(
            raw_points, fitted_points
        )
        completion_points, completion_debug = self._select_completion_points(
            raw_points, fitted_points, confidence=1.0
        )
        fit_to_raw_p50_m = float(completion_debug["fit_to_raw_p50_m"])

        confidence = 0.0
        accepted = False
        if strategy in self.supported_strategies:
            confidence = self._strategy_confidence(
                strategy,
                fit_debug,
                raw_to_fit_median_m=raw_to_fit_median_m,
                raw_to_fit_p90_m=raw_to_fit_p90_m,
                fit_to_raw_p50_m=fit_to_raw_p50_m,
            )
            accepted = confidence >= self.min_confidence

        if accepted:
            completion_points, completion_debug = self._select_completion_points(
                raw_points, fitted_points, confidence=confidence
            )
            output_points = np.concatenate([raw_points, completion_points], axis=0)
            output_points = voxel_downsample(output_points, self.output_voxel_size_m)
            mode = "fused"
        else:
            output_points = raw_points.astype(np.float32)
            mode = "raw_fallback"

        hold_applied = False
        freeze_state = "live"
        freeze_stable_count = 0
        freeze_applied = False
        if self._should_hold_previous(
            label=label,
            strategy=strategy,
            raw_centroid=raw_centroid,
            candidate_points=output_points,
            confidence=confidence,
            mode=mode,
            now_sec=now_sec,
        ):
            output_points = np.asarray(
                self._last_output_state.get("points", output_points), dtype=np.float32
            ).reshape((-1, 3))
            confidence = max(confidence, float(self._last_output_state.get("confidence", confidence)))
            mode = "held_fused"
            hold_applied = True

        if self._frozen_output_state:
            if self._should_keep_frozen(
                label=label,
                strategy=strategy,
                raw_centroid=raw_centroid,
                descriptor=descriptor,
                now_sec=now_sec,
            ):
                output_points = np.asarray(
                    self._frozen_output_state.get("points", output_points), dtype=np.float32
                ).reshape((-1, 3))
                confidence = max(
                    confidence,
                    float(self._frozen_output_state.get("confidence", confidence)),
                )
                mode = "frozen"
                freeze_state = "frozen"
                freeze_applied = True
            else:
                self._clear_frozen_output()
                self._reset_freeze_candidate()

        if freeze_state != "frozen":
            freeze_stable_count, should_freeze = self._update_freeze_candidate(
                label=label,
                strategy=strategy,
                raw_centroid=raw_centroid,
                descriptor=descriptor,
                confidence=confidence,
                output_points=output_points,
                now_sec=now_sec,
            )
            if should_freeze:
                self._set_frozen_output(
                    label=label,
                    strategy=strategy,
                    raw_centroid=raw_centroid,
                    descriptor=descriptor,
                    confidence=confidence,
                    output_points=output_points,
                    frame_id=str(raw_msg.header.frame_id or ""),
                    now_sec=now_sec,
                )
                freeze_state = "frozen"
                freeze_applied = True
                mode = "frozen"
            elif freeze_stable_count > 1:
                freeze_state = "locking"

        self.output_cloud_pub.publish(
            make_xyz_cloud(
                np.asarray(output_points, dtype=np.float32),
                str(raw_msg.header.frame_id or ""),
                raw_msg.header.stamp,
            )
        )

        debug_payload = {
            "status": "ok",
            "target_label": label,
            "strategy": strategy,
            "mode": mode,
            "fit_accepted": bool(accepted),
            "hold_applied": bool(hold_applied),
            "freeze_state": freeze_state,
            "freeze_stable_count": int(freeze_stable_count),
            "freeze_applied": bool(freeze_applied),
            "fit_confidence": round(float(confidence), 4),
            "raw_points": int(raw_points.shape[0]),
            "fitted_points": int(fitted_points.shape[0]),
            "output_points": int(output_points.shape[0]),
            "raw_to_fit_median_m": round(float(raw_to_fit_median_m), 4),
            "raw_to_fit_p90_m": round(float(raw_to_fit_p90_m), 4),
            "fit_to_raw_p50_m": round(float(completion_debug["fit_to_raw_p50_m"]), 4),
            "fit_to_raw_p90_m": round(float(completion_debug["fit_to_raw_p90_m"]), 4),
            "completion_candidates": int(completion_debug["completion_candidates"]),
            "completion_points_added": int(completion_debug["completion_points_added"]),
        }
        debug_msg = String()
        debug_msg.data = compact_json(debug_payload)
        self.output_debug_pub.publish(debug_msg)

        if mode in ("fused", "held_fused"):
            self._last_output_state = {
                "timestamp": float(now_sec),
                "label": label,
                "strategy": strategy,
                "mode": mode,
                "confidence": float(confidence),
                "raw_centroid": raw_centroid.copy(),
                "frame_id": str(raw_msg.header.frame_id or ""),
                "points": np.asarray(output_points, dtype=np.float32).copy(),
                "point_count": int(output_points.shape[0]),
            }

        summary = debug_msg.data
        if summary != self._last_summary and (now_sec - self._last_log_time) >= self.log_interval_sec:
            self._last_summary = summary
            self._last_log_time = now_sec
            self.get_logger().info(
                "grasp input cloud: "
                f"label={label or 'unknown'} strategy={strategy or 'raw'} "
                f"mode={mode} confidence={confidence:.3f} "
                f"raw={raw_points.shape[0]} output={output_points.shape[0]}"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspInputCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
