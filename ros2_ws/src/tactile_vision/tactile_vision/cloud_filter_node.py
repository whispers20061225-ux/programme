from __future__ import annotations

import json
import threading
import time
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from tactile_vision.modular_common import (
    clamp_int,
    compact_json,
    decode_depth_image,
    make_xyz_cloud,
    quaternion_to_rotation_matrix,
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
        self.declare_parameter("target_pose_topic", "/sim/perception/target_pose")
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter(
            "candidate_visible_topic", "/sim/perception/target_candidate_visible"
        )
        self.declare_parameter("debug_topic", "/perception/target_cloud_debug")
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
        self.declare_parameter("voxel_size_m", 0.004)
        self.declare_parameter("stable_frames_required", 2)
        self.declare_parameter("lock_position_tol_m", 0.03)
        self.declare_parameter("log_interval_sec", 8.0)

        self.detection_result_topic = str(self.get_parameter("detection_result_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.target_cloud_topic = str(self.get_parameter("target_cloud_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.candidate_visible_topic = str(self.get_parameter("candidate_visible_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
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
        self.voxel_size_m = max(0.0, float(self.get_parameter("voxel_size_m").value))
        self.stable_frames_required = max(
            1, int(self.get_parameter("stable_frames_required").value)
        )
        self.lock_position_tol_m = max(
            0.001, float(self.get_parameter("lock_position_tol_m").value)
        )
        self.log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._detection_lock = threading.Lock()
        self._sensor_lock = threading.Lock()
        self._latest_detection: Optional[dict[str, Any]] = None
        self._latest_detection_ts = 0.0
        self._latest_depth_msg: Optional[Image] = None
        self._latest_camera_info: Optional[CameraInfo] = None
        self._last_world_position: Optional[np.ndarray] = None
        self._stable_count = 0
        self._last_log_ts = 0.0
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

        self.create_subscription(
            String, self.detection_result_topic, self._on_detection_result, qos_reliable
        )
        self.create_subscription(Image, self.depth_topic, self._on_depth_image, qos_sensor)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, qos_sensor
        )

        self.target_cloud_pub = self.create_publisher(
            PointCloud2, self.target_cloud_topic, qos_reliable
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped, self.target_pose_topic, qos_reliable
        )
        self.target_locked_pub = self.create_publisher(
            Bool, self.target_locked_topic, qos_reliable
        )
        self.candidate_visible_pub = self.create_publisher(
            Bool, self.candidate_visible_topic, qos_reliable
        )
        self.debug_pub = self.create_publisher(String, self.debug_topic, qos_reliable)
        self.create_timer(1.0 / self.publish_rate_hz, self._process_latest)

        self.get_logger().info(
            "cloud_filter_node started: "
            f"detection={self.detection_result_topic}, target_cloud={self.target_cloud_topic}, "
            f"target_pose={self.target_pose_topic}, target_frame={self.target_frame}"
        )

    def _on_detection_result(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data or "{}"))
        except Exception:  # noqa: BLE001
            payload = {"status": "parse_error", "accepted": False}
        with self._detection_lock:
            self._latest_detection = payload
            self._latest_detection_ts = time.time()

    def _on_depth_image(self, msg: Image) -> None:
        with self._sensor_lock:
            self._latest_depth_msg = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._sensor_lock:
            self._latest_camera_info = msg

    def _process_latest(self) -> None:
        if not self.enabled:
            return

        with self._detection_lock:
            detection = dict(self._latest_detection) if self._latest_detection is not None else None
            detection_ts = self._latest_detection_ts
        with self._sensor_lock:
            depth_msg = self._latest_depth_msg
            camera_info = self._latest_camera_info

        if detection is None or depth_msg is None or camera_info is None:
            return
        if time.time() - detection_ts > self.detection_stale_sec:
            self._reset_state("stale_detection")
            return
        if not bool(detection.get("accepted")):
            self._reset_state(str(detection.get("status") or "detection_not_accepted"))
            return

        depth_image = decode_depth_image(depth_msg)
        if depth_image is None:
            self._reset_state("invalid_depth_image")
            return

        bbox = detection.get("bbox_xyxy")
        point = detection.get("point_px")
        if not isinstance(bbox, list) or len(bbox) != 4:
            if isinstance(point, list) and len(point) == 2:
                radius_px = 24
                bbox = [
                    int(point[0]) - radius_px,
                    int(point[1]) - radius_px,
                    int(point[0]) + radius_px,
                    int(point[1]) + radius_px,
                ]
            else:
                self._reset_state("missing_bbox_or_point")
                return

        points_world = self._extract_target_cloud(depth_image, camera_info, depth_msg, bbox)
        if points_world is None or points_world.shape[0] < self.min_target_points:
            self._reset_state("insufficient_target_points")
            return

        points_world = voxel_downsample(points_world, self.voxel_size_m)
        centroid = np.mean(points_world, axis=0)
        if self._last_world_position is None:
            self._stable_count = 1
        elif np.linalg.norm(centroid - self._last_world_position) <= self.lock_position_tol_m:
            self._stable_count += 1
        else:
            self._stable_count = 1
        self._last_world_position = centroid
        locked = bool(self._stable_count >= self.stable_frames_required)

        self.target_cloud_pub.publish(
            make_xyz_cloud(points_world, self.target_frame, depth_msg.header.stamp)
        )
        pose_msg = PoseStamped()
        pose_msg.header.stamp = depth_msg.header.stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = float(centroid[0])
        pose_msg.pose.position.y = float(centroid[1])
        pose_msg.pose.position.z = float(centroid[2])
        pose_msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose_msg)
        self.target_locked_pub.publish(Bool(data=locked))
        self.candidate_visible_pub.publish(Bool(data=True))

        debug_msg = String()
        debug_msg.data = compact_json(
            {
                "status": "ok",
                "target_points": int(points_world.shape[0]),
                "centroid_xyz": [
                    round(float(centroid[0]), 4),
                    round(float(centroid[1]), 4),
                    round(float(centroid[2]), 4),
                ],
                "locked": locked,
                "stable_count": int(self._stable_count),
            }
        )
        self.debug_pub.publish(debug_msg)

        now = time.time()
        if now - self._last_log_ts >= self.log_interval_sec:
            self._last_log_ts = now
            self.get_logger().info(
                "cloud filter result: "
                f"points={points_world.shape[0]} "
                f"locked={locked} "
                f"stable_count={self._stable_count}"
            )

    def _extract_target_cloud(
        self,
        depth_image: np.ndarray,
        camera_info: CameraInfo,
        depth_msg: Image,
        bbox_xyxy: list[Any],
    ) -> Optional[np.ndarray]:
        image_h, image_w = depth_image.shape[:2]
        x1 = clamp_int(int(round(float(bbox_xyxy[0]))) - self.bbox_margin_px, 0, image_w - 1)
        y1 = clamp_int(int(round(float(bbox_xyxy[1]))) - self.bbox_margin_px, 0, image_h - 1)
        x2 = clamp_int(int(round(float(bbox_xyxy[2]))) + self.bbox_margin_px, 1, image_w)
        y2 = clamp_int(int(round(float(bbox_xyxy[3]))) + self.bbox_margin_px, 1, image_h)
        if x2 <= x1 or y2 <= y1:
            return None

        roi = depth_image[y1:y2, x1:x2]
        valid_mask = np.isfinite(roi) & (roi >= self.min_depth_m) & (roi <= self.max_depth_m)
        if int(np.count_nonzero(valid_mask)) < self.min_target_points:
            return None

        valid_depths = roi[valid_mask]
        anchor_depth = float(np.percentile(valid_depths, self.depth_percentile_low))
        target_mask = valid_mask & (roi <= anchor_depth + self.depth_band_m)
        if int(np.count_nonzero(target_mask)) < self.min_target_points:
            target_mask = valid_mask

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
        try:
            transform = self._tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(depth_msg.header.stamp),
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
        return (points_camera @ rotation.T + translation).astype(np.float32)

    def _reset_state(self, reason: str) -> None:
        self._stable_count = 0
        self._last_world_position = None
        self.target_locked_pub.publish(Bool(data=False))
        self.candidate_visible_pub.publish(Bool(data=False))
        msg = String()
        msg.data = compact_json({"status": "reset", "reason": reason})
        self.debug_pub.publish(msg)


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
