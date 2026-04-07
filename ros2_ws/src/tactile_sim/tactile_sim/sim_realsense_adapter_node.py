from __future__ import annotations

from collections import deque
from copy import deepcopy
import time
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from tactile_interfaces.msg import DetectionResult


class SimRealsenseAdapterNode(Node):
    """Expose Gazebo wrist sensors using RealSense-style ROS topics."""

    def __init__(self) -> None:
        super().__init__("sim_realsense_adapter_node")

        self.declare_parameter("input_depth_topic", "/sim/camera/depth/image_raw")
        self.declare_parameter("input_imu_topic", "/sim/camera/imu")
        self.declare_parameter("input_color_camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("input_detection_topic", "/perception/detection_result")
        self.declare_parameter("output_depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter(
            "output_aligned_depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter("output_depth_camera_info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter(
            "output_aligned_depth_camera_info_topic",
            "/camera/camera/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter("output_imu_topic", "/camera/camera/imu")
        self.declare_parameter("output_gyro_topic", "/camera/camera/gyro/sample")
        self.declare_parameter("output_accel_topic", "/camera/camera/accel/sample")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("aligned_depth_frame_id", "camera_color_optical_frame")
        self.declare_parameter("imu_frame_id", "camera_imu_optical_frame")
        self.declare_parameter("gyro_frame_id", "camera_gyro_optical_frame")
        self.declare_parameter("accel_frame_id", "camera_accel_optical_frame")
        self.declare_parameter("enable_temporal_depth_filter", True)
        self.declare_parameter("temporal_depth_filter_window_size", 3)
        self.declare_parameter("temporal_depth_filter_min_valid_frames", 2)
        self.declare_parameter("temporal_depth_filter_min_depth_m", 0.10)
        self.declare_parameter("temporal_depth_filter_max_depth_m", 2.00)
        self.declare_parameter("fill_invalid_depth_pixels", True)
        self.declare_parameter("synthetic_depth_fallback_enabled", True)
        self.declare_parameter("synthetic_depth_invalid_ratio_threshold", 0.01)
        self.declare_parameter("synthetic_detection_hold_sec", 1.5)
        self.declare_parameter("synthetic_default_depth_m", 0.60)
        self.declare_parameter("synthetic_min_depth_m", 0.12)
        self.declare_parameter("synthetic_cylinder_diameter_m", 0.044)
        self.declare_parameter("synthetic_cylinder_height_m", 0.088)
        self.declare_parameter("synthetic_box_depth_m", 0.040)
        self.declare_parameter("synthetic_depth_smoothing_alpha", 0.30)
        self.declare_parameter("synthetic_bbox_smoothing_alpha", 0.35)
        self.declare_parameter("synthetic_cache_hold_sec", 1.0)

        input_depth_topic = str(self.get_parameter("input_depth_topic").value)
        input_imu_topic = str(self.get_parameter("input_imu_topic").value)
        input_color_camera_info_topic = str(
            self.get_parameter("input_color_camera_info_topic").value
        )
        input_detection_topic = str(self.get_parameter("input_detection_topic").value)
        output_depth_topic = str(self.get_parameter("output_depth_topic").value)
        output_aligned_depth_topic = str(self.get_parameter("output_aligned_depth_topic").value)
        output_depth_camera_info_topic = str(
            self.get_parameter("output_depth_camera_info_topic").value
        )
        output_aligned_depth_camera_info_topic = str(
            self.get_parameter("output_aligned_depth_camera_info_topic").value
        )
        output_imu_topic = str(self.get_parameter("output_imu_topic").value)
        output_gyro_topic = str(self.get_parameter("output_gyro_topic").value)
        output_accel_topic = str(self.get_parameter("output_accel_topic").value)
        self.depth_frame_id = str(self.get_parameter("depth_frame_id").value)
        self.aligned_depth_frame_id = str(self.get_parameter("aligned_depth_frame_id").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.gyro_frame_id = str(self.get_parameter("gyro_frame_id").value)
        self.accel_frame_id = str(self.get_parameter("accel_frame_id").value)
        self.enable_temporal_depth_filter = bool(
            self.get_parameter("enable_temporal_depth_filter").value
        )
        self.temporal_depth_filter_window_size = max(
            1, int(self.get_parameter("temporal_depth_filter_window_size").value)
        )
        self.temporal_depth_filter_min_valid_frames = max(
            1, int(self.get_parameter("temporal_depth_filter_min_valid_frames").value)
        )
        self.temporal_depth_filter_min_depth_m = float(
            self.get_parameter("temporal_depth_filter_min_depth_m").value
        )
        self.temporal_depth_filter_max_depth_m = float(
            self.get_parameter("temporal_depth_filter_max_depth_m").value
        )
        self.fill_invalid_depth_pixels = bool(
            self.get_parameter("fill_invalid_depth_pixels").value
        )
        self.synthetic_depth_fallback_enabled = bool(
            self.get_parameter("synthetic_depth_fallback_enabled").value
        )
        self.synthetic_depth_invalid_ratio_threshold = max(
            0.0, float(self.get_parameter("synthetic_depth_invalid_ratio_threshold").value)
        )
        self.synthetic_detection_hold_sec = max(
            0.0, float(self.get_parameter("synthetic_detection_hold_sec").value)
        )
        self.synthetic_default_depth_m = float(
            self.get_parameter("synthetic_default_depth_m").value
        )
        self.synthetic_min_depth_m = float(
            self.get_parameter("synthetic_min_depth_m").value
        )
        self.synthetic_cylinder_diameter_m = float(
            self.get_parameter("synthetic_cylinder_diameter_m").value
        )
        self.synthetic_cylinder_height_m = float(
            self.get_parameter("synthetic_cylinder_height_m").value
        )
        self.synthetic_box_depth_m = float(
            self.get_parameter("synthetic_box_depth_m").value
        )
        self.synthetic_depth_smoothing_alpha = float(
            self.get_parameter("synthetic_depth_smoothing_alpha").value
        )
        self.synthetic_bbox_smoothing_alpha = float(
            self.get_parameter("synthetic_bbox_smoothing_alpha").value
        )
        self.synthetic_cache_hold_sec = max(
            0.0, float(self.get_parameter("synthetic_cache_hold_sec").value)
        )

        self.depth_pub = self.create_publisher(Image, output_depth_topic, qos_profile_sensor_data)
        self.aligned_depth_pub = self.create_publisher(
            Image, output_aligned_depth_topic, qos_profile_sensor_data
        )
        self.depth_camera_info_pub = self.create_publisher(
            CameraInfo,
            output_depth_camera_info_topic,
            qos_profile_sensor_data,
        )
        self.aligned_depth_camera_info_pub = self.create_publisher(
            CameraInfo,
            output_aligned_depth_camera_info_topic,
            qos_profile_sensor_data,
        )
        self.imu_pub = self.create_publisher(Imu, output_imu_topic, qos_profile_sensor_data)
        self.gyro_pub = self.create_publisher(Imu, output_gyro_topic, qos_profile_sensor_data)
        self.accel_pub = self.create_publisher(Imu, output_accel_topic, qos_profile_sensor_data)

        self._latest_color_camera_info: CameraInfo | None = None
        self._depth_history: deque[np.ndarray] = deque(
            maxlen=self.temporal_depth_filter_window_size
        )
        self._depth_history_shape: tuple[int, int] | None = None
        self._latest_detection_context: dict[str, object] | None = None
        self._last_synthetic_fallback_log_at = 0.0
        self._synthetic_cache: dict[str, object] | None = None

        self.create_subscription(
            CameraInfo,
            input_color_camera_info_topic,
            self._on_color_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DetectionResult,
            input_detection_topic,
            self._on_detection_result,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            input_depth_topic,
            self._on_depth_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            input_imu_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "sim_realsense_adapter_node started: "
            f"depth_in={input_depth_topic}, imu_in={input_imu_topic}, "
            f"depth_out={output_depth_topic}, aligned_depth_out={output_aligned_depth_topic}, "
            f"imu_out={output_imu_topic}, temporal_depth_filter={self.enable_temporal_depth_filter}, "
            f"window={self.temporal_depth_filter_window_size}, "
            f"min_valid_frames={self.temporal_depth_filter_min_valid_frames}, "
            f"synthetic_depth={self.synthetic_depth_fallback_enabled}"
        )

    def _on_color_camera_info(self, msg: CameraInfo) -> None:
        self._latest_color_camera_info = deepcopy(msg)

    def _on_detection_result(self, msg: DetectionResult) -> None:
        bbox_xyxy: tuple[int, int, int, int] | None = None
        if bool(msg.has_bbox):
            x1 = int(msg.bbox.x_offset)
            y1 = int(msg.bbox.y_offset)
            x2 = x1 + max(0, int(msg.bbox.width))
            y2 = y1 + max(0, int(msg.bbox.height))
            bbox_xyxy = (x1, y1, x2, y2)

        mask_u8 = self._decode_mono8_image(msg.mask) if bool(msg.has_mask) else None
        point_px = None
        if bool(msg.has_point) and len(msg.point_px) == 2:
            point_px = (int(msg.point_px[0]), int(msg.point_px[1]))

        self._latest_detection_context = {
            "accepted": bool(msg.accepted),
            "candidate_visible": bool(msg.candidate_visible),
            "target_label": str(msg.target_label or ""),
            "bbox_xyxy": bbox_xyxy,
            "mask_u8": mask_u8,
            "point_px": point_px,
            "image_width": int(msg.image_width or 0),
            "image_height": int(msg.image_height or 0),
            "stamp_sec": self._ros_stamp_to_sec(msg.header.stamp),
        }

    def _on_depth_image(self, msg: Image) -> None:
        depth_output, aligned_depth_output = self._build_depth_outputs(msg)

        self.depth_pub.publish(depth_output)
        self.aligned_depth_pub.publish(aligned_depth_output)

        if self._latest_color_camera_info is None:
            return

        depth_camera_info = self._copy_camera_info(
            self._latest_color_camera_info,
            msg.header.stamp,
            self.depth_frame_id,
        )
        aligned_depth_camera_info = self._copy_camera_info(
            self._latest_color_camera_info,
            msg.header.stamp,
            self.aligned_depth_frame_id,
        )
        self.depth_camera_info_pub.publish(depth_camera_info)
        self.aligned_depth_camera_info_pub.publish(aligned_depth_camera_info)

    def _build_depth_outputs(self, msg: Image) -> tuple[Image, Image]:
        depth_image_m = self._decode_depth_image(msg)
        if depth_image_m is None:
            return (
                self._copy_image(msg, self.depth_frame_id),
                self._copy_image(msg, self.aligned_depth_frame_id),
            )

        prepared_depth_m = self._prepare_depth_image(depth_image_m, msg)
        if not self.enable_temporal_depth_filter:
            filtered_depth_m = prepared_depth_m
        else:
            filtered_depth_m = self._apply_temporal_depth_filter(prepared_depth_m)
        return (
            self._encode_depth_image(filtered_depth_m, msg, self.depth_frame_id),
            self._encode_depth_image(filtered_depth_m, msg, self.aligned_depth_frame_id),
        )

    def _prepare_depth_image(self, depth_image_m: np.ndarray, msg: Image) -> np.ndarray:
        working_depth = np.ascontiguousarray(depth_image_m.astype(np.float32, copy=True))
        valid_mask = self._valid_depth_mask(working_depth)
        valid_ratio = float(np.count_nonzero(valid_mask)) / float(valid_mask.size or 1)

        synthetic_depth = None
        if self.synthetic_depth_fallback_enabled:
            synthetic_depth = self._build_synthetic_depth_image(msg)

        if synthetic_depth is not None and valid_ratio < self.synthetic_depth_invalid_ratio_threshold:
            self._maybe_log_synthetic_fallback(
                reason="depth_invalid",
                valid_ratio=valid_ratio,
            )
            return synthetic_depth

        if self.fill_invalid_depth_pixels and not np.all(valid_mask):
            fill_image = synthetic_depth
            if fill_image is None:
                fill_image = np.full(
                    working_depth.shape,
                    np.float32(
                        max(
                            self.synthetic_min_depth_m,
                            min(self.synthetic_default_depth_m, self.temporal_depth_filter_max_depth_m),
                        )
                    ),
                    dtype=np.float32,
                )
            working_depth[~valid_mask] = fill_image[~valid_mask]

        return working_depth

    def _apply_temporal_depth_filter(self, depth_image_m: np.ndarray) -> np.ndarray:
        image_shape = depth_image_m.shape[:2]
        if self._depth_history_shape != image_shape:
            self._depth_history.clear()
            self._depth_history_shape = image_shape

        latest_depth = np.ascontiguousarray(depth_image_m.astype(np.float32, copy=False))
        self._depth_history.append(latest_depth.copy())

        if len(self._depth_history) < self.temporal_depth_filter_min_valid_frames:
            return latest_depth

        depth_stack = np.stack(tuple(self._depth_history), axis=0)
        valid_mask = np.isfinite(depth_stack) & (
            depth_stack >= self.temporal_depth_filter_min_depth_m
        )
        if self.temporal_depth_filter_max_depth_m > self.temporal_depth_filter_min_depth_m:
            valid_mask &= depth_stack <= self.temporal_depth_filter_max_depth_m

        valid_count = np.count_nonzero(valid_mask, axis=0)
        enough_history = valid_count >= self.temporal_depth_filter_min_valid_frames
        if not np.any(enough_history):
            return latest_depth

        filtered_depth = latest_depth.copy()
        flat_enough = enough_history.reshape(-1)
        flat_stack = depth_stack.reshape(depth_stack.shape[0], -1)
        flat_valid = valid_mask.reshape(valid_mask.shape[0], -1)
        masked_values = np.where(flat_valid[:, flat_enough], flat_stack[:, flat_enough], np.nan)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            median_values = np.nanmedian(masked_values, axis=0)
        filtered_depth.reshape(-1)[flat_enough] = median_values.astype(np.float32, copy=False)
        return filtered_depth

    def _valid_depth_mask(self, depth_image_m: np.ndarray) -> np.ndarray:
        valid_mask = np.isfinite(depth_image_m) & (
            depth_image_m >= self.temporal_depth_filter_min_depth_m
        )
        if self.temporal_depth_filter_max_depth_m > self.temporal_depth_filter_min_depth_m:
            valid_mask &= depth_image_m <= self.temporal_depth_filter_max_depth_m
        return valid_mask

    def _build_synthetic_depth_image(self, msg: Image) -> np.ndarray | None:
        height = int(getattr(msg, "height", 0) or 0)
        width = int(getattr(msg, "width", 0) or 0)
        if height <= 0 or width <= 0:
            return None

        background_depth = np.float32(
            max(
                self.synthetic_min_depth_m,
                min(self.synthetic_default_depth_m, self.temporal_depth_filter_max_depth_m),
            )
        )
        depth_image = np.full((height, width), background_depth, dtype=np.float32)

        now_sec = self._ros_stamp_to_sec(msg.header.stamp)
        synthetic_context = self._resolve_synthetic_context(now_sec)
        if synthetic_context is None:
            return depth_image

        bbox_xyxy = synthetic_context["bbox_xyxy"]
        x1, y1, x2, y2 = [int(v) for v in bbox_xyxy]
        x1 = max(0, min(width - 1, x1))
        y1 = max(0, min(height - 1, y1))
        x2 = max(x1 + 1, min(width, x2))
        y2 = max(y1 + 1, min(height, y2))

        object_mask = self._build_detection_mask(
            height=height,
            width=width,
            bbox_xyxy=(x1, y1, x2, y2),
            mask_u8=synthetic_context.get("mask_u8"),
        )
        if not np.any(object_mask):
            return depth_image

        label = str(synthetic_context.get("target_label", "") or "")
        kind = str(synthetic_context.get("kind", "generic") or "generic")
        fx, fy = self._camera_focal_lengths()
        point_px = synthetic_context.get("point_px")
        center_x_px = float(point_px[0]) if isinstance(point_px, tuple) and len(point_px) == 2 else (float(x1) + float(x2)) * 0.5

        bbox_width_px = max(1.0, float(x2 - x1))
        bbox_height_px = max(1.0, float(y2 - y1))
        object_center_depth_m = float(
            synthetic_context.get("center_depth_m")
            or self._estimate_object_center_depth_m(
                kind=kind,
                bbox_width_px=bbox_width_px,
                bbox_height_px=bbox_height_px,
                fx=fx,
                fy=fy,
                background_depth=float(background_depth),
            )
        )

        if kind == "cylinder":
            object_depth = self._render_cylinder_depth(
                height=height,
                width=width,
                center_x_px=center_x_px,
                center_depth_m=object_center_depth_m,
                focal_length_px=fx,
                radius_m=max(0.001, 0.5 * self.synthetic_cylinder_diameter_m),
            )
        else:
            object_depth = np.full(
                (height, width),
                np.float32(
                    max(
                        self.synthetic_min_depth_m,
                        object_center_depth_m - max(0.001, 0.5 * self.synthetic_box_depth_m),
                    )
                ),
                dtype=np.float32,
            )

        depth_image[object_mask] = object_depth[object_mask]
        return depth_image

    def _resolve_synthetic_context(self, now_sec: float) -> dict[str, object] | None:
        detection_context = self._latest_detection_context
        cached = self._synthetic_cache

        if detection_context is None:
            return self._cached_synthetic_context(now_sec)

        stamp_sec = float(detection_context.get("stamp_sec", 0.0) or 0.0)
        if now_sec > 0.0 and stamp_sec > 0.0:
            if abs(now_sec - stamp_sec) > self.synthetic_detection_hold_sec:
                return self._cached_synthetic_context(now_sec)

        bbox_xyxy = detection_context.get("bbox_xyxy")
        if not isinstance(bbox_xyxy, tuple) or len(bbox_xyxy) != 4:
            return self._cached_synthetic_context(now_sec)

        label = str(detection_context.get("target_label", "") or "")
        kind = self._classify_detection_kind(label)
        point_px = detection_context.get("point_px")
        if not (isinstance(point_px, tuple) and len(point_px) == 2):
            point_px = None

        bbox_xyxy = tuple(int(v) for v in bbox_xyxy)
        if cached is not None and self._synthetic_cache_matches(cached, label=label, kind=kind, now_sec=now_sec):
            previous_bbox = cached.get("bbox_xyxy")
            if isinstance(previous_bbox, tuple) and len(previous_bbox) == 4:
                iou = self._bbox_iou(previous_bbox, bbox_xyxy)
                if iou >= 0.10:
                    bbox_xyxy = self._blend_bbox(
                        previous_bbox,
                        bbox_xyxy,
                        alpha=self.synthetic_bbox_smoothing_alpha,
                    )
                    previous_point = cached.get("point_px")
                    if (
                        isinstance(previous_point, tuple)
                        and len(previous_point) == 2
                        and isinstance(point_px, tuple)
                        and len(point_px) == 2
                    ):
                        alpha = min(1.0, max(0.0, self.synthetic_bbox_smoothing_alpha))
                        point_px = (
                            int(round((1.0 - alpha) * float(previous_point[0]) + alpha * float(point_px[0]))),
                            int(round((1.0 - alpha) * float(previous_point[1]) + alpha * float(point_px[1]))),
                        )
                elif cached.get("bbox_xyxy") is not None:
                    bbox_xyxy = tuple(int(v) for v in cached["bbox_xyxy"])
                    cached_point = cached.get("point_px")
                    if isinstance(cached_point, tuple) and len(cached_point) == 2:
                        point_px = (int(cached_point[0]), int(cached_point[1]))

        fx, fy = self._camera_focal_lengths()
        bbox_width_px = max(1.0, float(bbox_xyxy[2] - bbox_xyxy[0]))
        bbox_height_px = max(1.0, float(bbox_xyxy[3] - bbox_xyxy[1]))
        center_depth_m = self._estimate_object_center_depth_m(
            kind=kind,
            bbox_width_px=bbox_width_px,
            bbox_height_px=bbox_height_px,
            fx=fx,
            fy=fy,
            background_depth=float(
                max(
                    self.synthetic_min_depth_m,
                    min(self.synthetic_default_depth_m, self.temporal_depth_filter_max_depth_m),
                )
            ),
        )

        if cached is not None and self._synthetic_cache_matches(cached, label=label, kind=kind, now_sec=now_sec):
            previous_depth = cached.get("center_depth_m")
            if previous_depth is not None:
                alpha = min(1.0, max(0.0, self.synthetic_depth_smoothing_alpha))
                center_depth_m = (1.0 - alpha) * float(previous_depth) + alpha * float(center_depth_m)

        resolved = {
            "target_label": label,
            "kind": kind,
            "bbox_xyxy": bbox_xyxy,
            "point_px": point_px,
            "mask_u8": detection_context.get("mask_u8"),
            "center_depth_m": float(center_depth_m),
            "stamp_sec": float(stamp_sec or now_sec),
        }
        self._synthetic_cache = dict(resolved)
        return resolved

    def _cached_synthetic_context(self, now_sec: float) -> dict[str, object] | None:
        cached = self._synthetic_cache
        if cached is None:
            return None
        stamp_sec = float(cached.get("stamp_sec", 0.0) or 0.0)
        if now_sec > 0.0 and stamp_sec > 0.0:
            if abs(now_sec - stamp_sec) > self.synthetic_cache_hold_sec:
                return None
        return dict(cached)

    def _synthetic_cache_matches(
        self,
        cached: dict[str, object],
        *,
        label: str,
        kind: str,
        now_sec: float,
    ) -> bool:
        if str(cached.get("target_label", "") or "") != label:
            return False
        if str(cached.get("kind", "") or "") != kind:
            return False
        stamp_sec = float(cached.get("stamp_sec", 0.0) or 0.0)
        if now_sec > 0.0 and stamp_sec > 0.0:
            return abs(now_sec - stamp_sec) <= self.synthetic_cache_hold_sec
        return True

    @staticmethod
    def _blend_bbox(
        previous_bbox_xyxy: tuple[int, int, int, int],
        current_bbox_xyxy: tuple[int, int, int, int],
        *,
        alpha: float,
    ) -> tuple[int, int, int, int]:
        alpha = min(1.0, max(0.0, alpha))
        return tuple(
            int(round((1.0 - alpha) * float(prev) + alpha * float(curr)))
            for prev, curr in zip(previous_bbox_xyxy, current_bbox_xyxy)
        )

    @staticmethod
    def _bbox_iou(
        bbox_a_xyxy: tuple[int, int, int, int],
        bbox_b_xyxy: tuple[int, int, int, int],
    ) -> float:
        ax1, ay1, ax2, ay2 = bbox_a_xyxy
        bx1, by1, bx2, by2 = bbox_b_xyxy
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area <= 0:
            return 0.0
        area_a = max(1, ax2 - ax1) * max(1, ay2 - ay1)
        area_b = max(1, bx2 - bx1) * max(1, by2 - by1)
        union_area = max(1, area_a + area_b - inter_area)
        return float(inter_area) / float(union_area)

    def _build_detection_mask(
        self,
        *,
        height: int,
        width: int,
        bbox_xyxy: tuple[int, int, int, int],
        mask_u8: object,
    ) -> np.ndarray:
        if isinstance(mask_u8, np.ndarray) and mask_u8.shape[:2] == (height, width):
            mask = np.asarray(mask_u8 > 0, dtype=bool)
            if np.any(mask):
                return mask

        x1, y1, x2, y2 = bbox_xyxy
        center_x = (float(x1) + float(x2)) * 0.5
        center_y = (float(y1) + float(y2)) * 0.5
        radius_x = max(1.0, 0.46 * float(x2 - x1))
        radius_y = max(1.0, 0.46 * float(y2 - y1))
        yy, xx = np.ogrid[:height, :width]
        ellipse = (
            ((xx - center_x) / radius_x) ** 2 + ((yy - center_y) / radius_y) ** 2
        ) <= 1.0
        bbox_mask = np.zeros((height, width), dtype=bool)
        bbox_mask[y1:y2, x1:x2] = True
        return ellipse & bbox_mask

    def _classify_detection_kind(self, label: str) -> str:
        normalized = str(label or "").strip().lower()
        if any(token in normalized for token in ("cylinder", "cup", "mug", "bottle", "圆柱", "杯")):
            return "cylinder"
        if any(token in normalized for token in ("box", "cube", "block", "块", "方")):
            return "box"
        return "generic"

    def _camera_focal_lengths(self) -> tuple[float, float]:
        if self._latest_color_camera_info is not None and len(self._latest_color_camera_info.k) >= 5:
            fx = float(self._latest_color_camera_info.k[0] or 0.0)
            fy = float(self._latest_color_camera_info.k[4] or 0.0)
            if fx > 1.0 and fy > 1.0:
                return fx, fy
        return 600.0, 600.0

    def _estimate_object_center_depth_m(
        self,
        *,
        kind: str,
        bbox_width_px: float,
        bbox_height_px: float,
        fx: float,
        fy: float,
        background_depth: float,
    ) -> float:
        depth_candidates: list[float] = []
        if kind == "cylinder":
            if bbox_width_px > 1.0 and fx > 1.0:
                depth_candidates.append(fx * max(0.001, self.synthetic_cylinder_diameter_m) / bbox_width_px)
            if bbox_height_px > 1.0 and fy > 1.0:
                depth_candidates.append(fy * max(0.001, self.synthetic_cylinder_height_m) / bbox_height_px)
        else:
            nominal_extent_m = max(0.02, min(self.synthetic_cylinder_height_m, self.synthetic_box_depth_m))
            if bbox_width_px > 1.0 and fx > 1.0:
                depth_candidates.append(fx * nominal_extent_m / bbox_width_px)
            if bbox_height_px > 1.0 and fy > 1.0:
                depth_candidates.append(fy * nominal_extent_m / bbox_height_px)

        if not depth_candidates:
            estimate = max(self.synthetic_min_depth_m, background_depth - 0.08)
        else:
            estimate = float(np.median(np.asarray(depth_candidates, dtype=np.float32)))

        estimate = max(self.synthetic_min_depth_m, estimate)
        estimate = min(background_depth - 0.01, estimate)
        return estimate

    def _render_cylinder_depth(
        self,
        *,
        height: int,
        width: int,
        center_x_px: float,
        center_depth_m: float,
        focal_length_px: float,
        radius_m: float,
    ) -> np.ndarray:
        focal = max(1.0, float(focal_length_px))
        z_center = max(self.synthetic_min_depth_m + radius_m, float(center_depth_m))
        xs = np.arange(width, dtype=np.float32)
        x_offsets_m = (xs - np.float32(center_x_px)) * (np.float32(z_center) / np.float32(focal))
        radial_sq = np.float32(radius_m * radius_m) - x_offsets_m * x_offsets_m
        column_depth = np.full(width, np.float32(z_center), dtype=np.float32)
        valid_columns = radial_sq > 0.0
        if np.any(valid_columns):
            column_depth[valid_columns] = np.float32(z_center) - np.sqrt(
                radial_sq[valid_columns], dtype=np.float32
            )
        column_depth = np.clip(
            column_depth,
            np.float32(self.synthetic_min_depth_m),
            np.float32(z_center),
        )
        return np.broadcast_to(column_depth.reshape(1, width), (height, width)).copy()

    def _maybe_log_synthetic_fallback(self, *, reason: str, valid_ratio: float) -> None:
        now = time.time()
        if now - self._last_synthetic_fallback_log_at < 2.0:
            return
        self._last_synthetic_fallback_log_at = now
        label = ""
        if isinstance(self._latest_detection_context, dict):
            label = str(self._latest_detection_context.get("target_label", "") or "")
        self.get_logger().warn(
            f"using synthetic depth fallback: reason={reason} valid_ratio={valid_ratio:.4f} label={label or '<none>'}"
        )

    def _on_imu(self, msg: Imu) -> None:
        imu_msg = self._copy_imu(msg, self.imu_frame_id)
        gyro_msg = self._copy_imu(msg, self.gyro_frame_id)
        accel_msg = self._copy_imu(msg, self.accel_frame_id)

        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        gyro_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        gyro_msg.linear_acceleration.x = 0.0
        gyro_msg.linear_acceleration.y = 0.0
        gyro_msg.linear_acceleration.z = 0.0
        gyro_msg.linear_acceleration_covariance = [
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        accel_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        accel_msg.angular_velocity.x = 0.0
        accel_msg.angular_velocity.y = 0.0
        accel_msg.angular_velocity.z = 0.0
        accel_msg.angular_velocity_covariance = [
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        self.imu_pub.publish(imu_msg)
        self.gyro_pub.publish(gyro_msg)
        self.accel_pub.publish(accel_msg)

    @staticmethod
    def _copy_image(msg: Image, frame_id: str) -> Image:
        copy_msg = Image()
        copy_msg.header = deepcopy(msg.header)
        copy_msg.header.frame_id = frame_id
        copy_msg.height = msg.height
        copy_msg.width = msg.width
        copy_msg.encoding = msg.encoding
        copy_msg.is_bigendian = msg.is_bigendian
        copy_msg.step = msg.step
        copy_msg.data = msg.data
        return copy_msg

    @staticmethod
    def _decode_depth_image(msg: Image) -> np.ndarray | None:
        height = int(getattr(msg, "height", 0) or 0)
        width = int(getattr(msg, "width", 0) or 0)
        if height <= 0 or width <= 0:
            return None

        encoding = str(getattr(msg, "encoding", "") or "").lower()
        if encoding == "32fc1":
            data = np.frombuffer(getattr(msg, "data", b""), dtype=np.float32)
            expected = height * width
            if data.size < expected:
                return None
            return data[:expected].reshape((height, width))

        if encoding == "16uc1":
            data = np.frombuffer(getattr(msg, "data", b""), dtype=np.uint16)
            expected = height * width
            if data.size < expected:
                return None
            return data[:expected].reshape((height, width)).astype(np.float32) / 1000.0

        return None

    @staticmethod
    def _decode_mono8_image(msg: Image) -> np.ndarray | None:
        height = int(getattr(msg, "height", 0) or 0)
        width = int(getattr(msg, "width", 0) or 0)
        if height <= 0 or width <= 0:
            return None
        encoding = str(getattr(msg, "encoding", "") or "").lower()
        if encoding not in ("mono8", "8uc1"):
            return None
        data = np.frombuffer(getattr(msg, "data", b""), dtype=np.uint8)
        expected = height * width
        if data.size < expected:
            return None
        return data[:expected].reshape((height, width))

    @staticmethod
    def _encode_depth_image(depth_image_m: np.ndarray, msg: Image, frame_id: str) -> Image:
        depth_array = np.ascontiguousarray(depth_image_m.astype(np.float32, copy=False))
        copy_msg = Image()
        copy_msg.header = deepcopy(msg.header)
        copy_msg.header.frame_id = frame_id
        copy_msg.height = int(depth_array.shape[0])
        copy_msg.width = int(depth_array.shape[1])
        copy_msg.encoding = "32FC1"
        copy_msg.is_bigendian = 0
        copy_msg.step = int(depth_array.shape[1] * depth_array.dtype.itemsize)
        copy_msg.data = depth_array.tobytes()
        return copy_msg

    @staticmethod
    def _copy_camera_info(msg: CameraInfo, stamp, frame_id: str) -> CameraInfo:
        copy_msg = deepcopy(msg)
        copy_msg.header.stamp = stamp
        copy_msg.header.frame_id = frame_id
        return copy_msg

    @staticmethod
    def _copy_imu(msg: Imu, frame_id: str) -> Imu:
        copy_msg = deepcopy(msg)
        copy_msg.header.frame_id = frame_id
        return copy_msg

    @staticmethod
    def _ros_stamp_to_sec(stamp) -> float:
        try:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except Exception:
            return 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimRealsenseAdapterNode()
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
