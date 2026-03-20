from __future__ import annotations

from collections import deque
from copy import deepcopy
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu


class SimRealsenseAdapterNode(Node):
    """Expose Gazebo wrist sensors using RealSense-style ROS topics."""

    def __init__(self) -> None:
        super().__init__("sim_realsense_adapter_node")

        self.declare_parameter("input_depth_topic", "/sim/camera/depth/image_raw")
        self.declare_parameter("input_imu_topic", "/sim/camera/imu")
        self.declare_parameter("input_color_camera_info_topic", "/camera/camera/color/camera_info")
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

        input_depth_topic = str(self.get_parameter("input_depth_topic").value)
        input_imu_topic = str(self.get_parameter("input_imu_topic").value)
        input_color_camera_info_topic = str(
            self.get_parameter("input_color_camera_info_topic").value
        )
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

        self.create_subscription(
            CameraInfo,
            input_color_camera_info_topic,
            self._on_color_camera_info,
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
            f"min_valid_frames={self.temporal_depth_filter_min_valid_frames}"
        )

    def _on_color_camera_info(self, msg: CameraInfo) -> None:
        self._latest_color_camera_info = deepcopy(msg)

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
        if not self.enable_temporal_depth_filter:
            return (
                self._copy_image(msg, self.depth_frame_id),
                self._copy_image(msg, self.aligned_depth_frame_id),
            )

        depth_image_m = self._decode_depth_image(msg)
        if depth_image_m is None:
            return (
                self._copy_image(msg, self.depth_frame_id),
                self._copy_image(msg, self.aligned_depth_frame_id),
            )

        filtered_depth_m = self._apply_temporal_depth_filter(depth_image_m)
        return (
            self._encode_depth_image(filtered_depth_m, msg, self.depth_frame_id),
            self._encode_depth_image(filtered_depth_m, msg, self.aligned_depth_frame_id),
        )

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
