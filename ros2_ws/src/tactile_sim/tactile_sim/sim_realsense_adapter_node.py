from __future__ import annotations

from copy import deepcopy

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
            f"imu_out={output_imu_topic}"
        )

    def _on_color_camera_info(self, msg: CameraInfo) -> None:
        self._latest_color_camera_info = deepcopy(msg)

    def _on_depth_image(self, msg: Image) -> None:
        depth_image = self._copy_image(msg, self.depth_frame_id)
        aligned_depth_image = self._copy_image(msg, self.aligned_depth_frame_id)

        self.depth_pub.publish(depth_image)
        self.aligned_depth_pub.publish(aligned_depth_image)

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
