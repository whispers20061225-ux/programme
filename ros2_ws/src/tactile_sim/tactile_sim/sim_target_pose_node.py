from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class SimTargetPoseNode(Node):
    """Detect the tabletop blue cylinder and publish its pose in world."""

    def __init__(self) -> None:
        super().__init__("sim_target_pose_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter("target_pose_topic", "/sim/perception/target_pose")
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 2.00)
        self.declare_parameter("min_contour_area_px", 600.0)
        self.declare_parameter("edge_margin_px", 24)
        self.declare_parameter("stable_frames_required", 3)
        self.declare_parameter("lock_position_tol_m", 0.03)
        self.declare_parameter("lock_pixel_tol_px", 24.0)
        self.declare_parameter("mask_open_kernel_px", 5)
        self.declare_parameter("mask_close_kernel_px", 7)
        self.declare_parameter("blue_h_lower", 90)
        self.declare_parameter("blue_h_upper", 135)
        self.declare_parameter("blue_s_lower", 80)
        self.declare_parameter("blue_v_lower", 50)

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_contour_area_px = float(self.get_parameter("min_contour_area_px").value)
        self.edge_margin_px = int(self.get_parameter("edge_margin_px").value)
        self.stable_frames_required = max(
            1, int(self.get_parameter("stable_frames_required").value)
        )
        self.lock_position_tol_m = float(self.get_parameter("lock_position_tol_m").value)
        self.lock_pixel_tol_px = float(self.get_parameter("lock_pixel_tol_px").value)
        open_kernel_px = max(1, int(self.get_parameter("mask_open_kernel_px").value))
        close_kernel_px = max(1, int(self.get_parameter("mask_close_kernel_px").value))
        self._open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_kernel_px, open_kernel_px)
        )
        self._close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_kernel_px, close_kernel_px)
        )
        self._blue_lower = np.array(
            [
                int(self.get_parameter("blue_h_lower").value),
                int(self.get_parameter("blue_s_lower").value),
                int(self.get_parameter("blue_v_lower").value),
            ],
            dtype=np.uint8,
        )
        self._blue_upper = np.array(
            [
                int(self.get_parameter("blue_h_upper").value),
                255,
                255,
            ],
            dtype=np.uint8,
        )

        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.locked_pub = self.create_publisher(Bool, target_locked_topic, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._latest_depth_msg: Optional[Image] = None
        self._latest_camera_info: Optional[CameraInfo] = None
        self._stable_count = 0
        self._last_locked = False
        self._last_world_position: Optional[np.ndarray] = None
        self._last_center_px: Optional[np.ndarray] = None

        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            depth_topic,
            self._on_depth_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            color_topic,
            self._on_color_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "sim_target_pose_node started: "
            f"color={color_topic}, depth={depth_topic}, target_frame={self.target_frame}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _on_depth_image(self, msg: Image) -> None:
        self._latest_depth_msg = msg

    def _on_color_image(self, msg: Image) -> None:
        if self._latest_depth_msg is None or self._latest_camera_info is None:
            return

        color = self._decode_color_image(msg)
        depth = self._decode_depth_image(self._latest_depth_msg)
        if color is None or depth is None:
            self._publish_locked(False)
            return

        detection = self._detect_blue_target(color, depth)
        if detection is None:
            self._stable_count = 0
            self._last_world_position = None
            self._last_center_px = None
            self._publish_locked(False)
            return

        world_position: Optional[np.ndarray] = None
        source_frame = self._latest_camera_info.header.frame_id or msg.header.frame_id
        if detection["depth_m"] is not None:
            world_position = self._project_to_world(
                source_frame,
                self._latest_camera_info,
                detection["center_px"],
                detection["depth_m"],
            )

        if (
            detection["complete"]
            and self._last_center_px is not None
            and np.linalg.norm(detection["center_px"] - self._last_center_px) <= self.lock_pixel_tol_px
            and (
                world_position is None
                or self._last_world_position is None
                or np.linalg.norm(world_position - self._last_world_position)
                <= self.lock_position_tol_m
            )
        ):
            self._stable_count += 1
        elif detection["complete"]:
            self._stable_count = 1
        else:
            self._stable_count = 0

        self._last_center_px = detection["center_px"]
        self._last_world_position = world_position

        if world_position is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = self.target_frame
            pose_msg.pose.position.x = float(world_position[0])
            pose_msg.pose.position.y = float(world_position[1])
            pose_msg.pose.position.z = float(world_position[2])
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)

        self._publish_locked(self._stable_count >= self.stable_frames_required)

    def _publish_locked(self, locked: bool) -> None:
        self.locked_pub.publish(Bool(data=bool(locked)))
        if locked != self._last_locked:
            if locked:
                self.get_logger().info("target locked")
            else:
                self.get_logger().info("target lock lost")
            self._last_locked = locked

    def _detect_blue_target(
        self,
        color_rgb: np.ndarray,
        depth_m: np.ndarray,
    ) -> Optional[dict]:
        hsv = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self._blue_lower, self._blue_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_contour_area_px:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        image_h, image_w = color_rgb.shape[:2]
        complete = (
            x > self.edge_margin_px
            and y > self.edge_margin_px
            and (x + w) < (image_w - self.edge_margin_px)
            and (y + h) < (image_h - self.edge_margin_px)
        )

        moments = cv2.moments(contour)
        if moments["m00"] > 1e-6:
            center_u = float(moments["m10"] / moments["m00"])
            center_v = float(moments["m01"] / moments["m00"])
        else:
            center_u = float(x + w / 2.0)
            center_v = float(y + h / 2.0)

        roi_depth = depth_m[y : y + h, x : x + w]
        roi_mask = mask[y : y + h, x : x + w] > 0
        valid_depths = roi_depth[
            roi_mask
            & np.isfinite(roi_depth)
            & (roi_depth >= self.min_depth_m)
            & (roi_depth <= self.max_depth_m)
        ]
        depth_m: float | None = None
        if valid_depths.size >= 20:
            depth_m = float(np.median(valid_depths))

        return {
            "center_px": np.array([center_u, center_v], dtype=np.float32),
            "depth_m": depth_m,
            "complete": complete,
        }

    def _project_to_world(
        self,
        source_frame: str,
        camera_info: CameraInfo,
        center_px: np.ndarray,
        depth_m: float,
    ) -> Optional[np.ndarray]:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return None

        u = float(center_px[0])
        v = float(center_px[1])
        point_camera = np.array(
            [
                (u - cx) * depth_m / fx,
                (v - cy) * depth_m / fy,
                depth_m,
            ],
            dtype=np.float64,
        )

        try:
            transform = self._tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"failed to transform target pose from {source_frame} to {self.target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotated = self._rotate_vector_by_quaternion(
            point_camera,
            np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float64),
        )
        return rotated + np.array([translation.x, translation.y, translation.z], dtype=np.float64)

    @staticmethod
    def _rotate_vector_by_quaternion(vector: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
        x, y, z, w = quaternion_xyzw
        q_vec = np.array([x, y, z], dtype=np.float64)
        uv = np.cross(q_vec, vector)
        uuv = np.cross(q_vec, uv)
        return vector + 2.0 * (w * uv + uuv)

    @staticmethod
    def _decode_color_image(msg: Image) -> Optional[np.ndarray]:
        if msg.height <= 0 or msg.width <= 0:
            return None
        encoding = str(msg.encoding).lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in ("rgb8", "bgr8"):
            if data.size < msg.height * msg.width * 3:
                return None
            image = data[: msg.height * msg.width * 3].reshape((msg.height, msg.width, 3))
            return image[:, :, ::-1] if encoding == "bgr8" else image
        return None

    @staticmethod
    def _decode_depth_image(msg: Image) -> Optional[np.ndarray]:
        if msg.height <= 0 or msg.width <= 0:
            return None
        encoding = str(msg.encoding).lower()
        if encoding == "32fc1":
            data = np.frombuffer(msg.data, dtype=np.float32)
            if data.size < msg.height * msg.width:
                return None
            return data[: msg.height * msg.width].reshape((msg.height, msg.width))
        if encoding == "16uc1":
            data = np.frombuffer(msg.data, dtype=np.uint16)
            if data.size < msg.height * msg.width:
                return None
            return data[: msg.height * msg.width].reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimTargetPoseNode()
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
