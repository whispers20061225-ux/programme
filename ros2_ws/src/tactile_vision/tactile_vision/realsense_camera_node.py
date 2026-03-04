from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

try:
    import pyrealsense2 as rs
except Exception:  # pragma: no cover - runtime dependency check
    rs = None


class RealsenseCameraNode(Node):
    """Publish RealSense color/depth stream without depending on realsense2_camera package."""

    def __init__(self) -> None:
        super().__init__("realsense_camera_node")

        if rs is None:
            raise RuntimeError("pyrealsense2 is not available. Install Intel RealSense Python bindings first.")

        self.declare_parameter("serial_no", "")
        self.declare_parameter("enable_color", True)
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("align_depth.enable", True)
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("color_fps", 15)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("depth_fps", 15)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")

        raw_serial = str(self.get_parameter("serial_no").value).strip()
        self.serial_no = raw_serial[1:] if raw_serial.startswith("_") else raw_serial
        self.enable_color = bool(self.get_parameter("enable_color").value)
        self.enable_depth = bool(self.get_parameter("enable_depth").value)
        self.align_depth = bool(self.get_parameter("align_depth.enable").value)
        self.color_width = int(self.get_parameter("color_width").value)
        self.color_height = int(self.get_parameter("color_height").value)
        self.color_fps = max(1, int(self.get_parameter("color_fps").value))
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)
        self.depth_fps = max(1, int(self.get_parameter("depth_fps").value))
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.color_frame_id = str(self.get_parameter("color_frame_id").value)
        self.depth_frame_id = str(self.get_parameter("depth_frame_id").value)

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.color_pub = self.create_publisher(Image, self.color_topic, qos_sensor)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, qos_sensor)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, qos_sensor)

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        if self.serial_no:
            self.config.enable_device(self.serial_no)

        if self.enable_color:
            self.config.enable_stream(
                rs.stream.color,
                self.color_width,
                self.color_height,
                rs.format.rgb8,
                self.color_fps,
            )
        if self.enable_depth:
            self.config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.depth_fps,
            )

        self.aligner: Optional["rs.align"] = None
        if self.enable_color and self.enable_depth and self.align_depth:
            self.aligner = rs.align(rs.stream.color)

        self.profile = self.pipeline.start(self.config)
        self.color_info_template = self._make_camera_info_template(rs.stream.color, self.color_frame_id)

        publish_hz = max(self.color_fps if self.enable_color else 0, self.depth_fps if self.enable_depth else 0, 5)
        self.create_timer(1.0 / float(publish_hz), self._on_timer)

        self.get_logger().info(
            "realsense_camera_node started: "
            f"serial={self.serial_no or 'auto'}, color={self.enable_color}, depth={self.enable_depth}, "
            f"align_depth={self.align_depth}, color_topic={self.color_topic}, depth_topic={self.depth_topic}"
        )

    def _make_camera_info_template(self, stream_type: "rs.stream", frame_id: str) -> Optional[CameraInfo]:
        try:
            stream_profile = self.profile.get_stream(stream_type).as_video_stream_profile()
            intr = stream_profile.get_intrinsics()
        except Exception:
            return None

        info = CameraInfo()
        info.width = int(intr.width)
        info.height = int(intr.height)
        info.distortion_model = "plumb_bob"
        info.d = list(intr.coeffs)
        info.k = [
            float(intr.fx),
            0.0,
            float(intr.ppx),
            0.0,
            float(intr.fy),
            float(intr.ppy),
            0.0,
            0.0,
            1.0,
        ]
        info.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        info.p = [
            float(intr.fx),
            0.0,
            float(intr.ppx),
            0.0,
            0.0,
            float(intr.fy),
            float(intr.ppy),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        info.header.frame_id = frame_id
        return info

    def _on_timer(self) -> None:
        try:
            frames = self.pipeline.poll_for_frames()
            if not frames:
                return
            if self.aligner is not None:
                frames = self.aligner.process(frames)

            now = self.get_clock().now().to_msg()

            if self.enable_color:
                color_frame = frames.get_color_frame()
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    color_msg = Image()
                    color_msg.header.stamp = now
                    color_msg.header.frame_id = self.color_frame_id
                    color_msg.height = int(color_image.shape[0])
                    color_msg.width = int(color_image.shape[1])
                    color_msg.encoding = "rgb8"
                    color_msg.is_bigendian = False
                    color_msg.step = int(color_image.shape[1] * 3)
                    color_msg.data = color_image.tobytes()
                    self.color_pub.publish(color_msg)

                    if self.color_info_template is not None:
                        info_msg = CameraInfo()
                        info_msg.header.stamp = now
                        info_msg.header.frame_id = self.color_frame_id
                        info_msg.height = self.color_info_template.height
                        info_msg.width = self.color_info_template.width
                        info_msg.distortion_model = self.color_info_template.distortion_model
                        info_msg.d = list(self.color_info_template.d)
                        info_msg.k = list(self.color_info_template.k)
                        info_msg.r = list(self.color_info_template.r)
                        info_msg.p = list(self.color_info_template.p)
                        self.info_pub.publish(info_msg)

            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    depth_msg = Image()
                    depth_msg.header.stamp = now
                    depth_msg.header.frame_id = self.depth_frame_id
                    depth_msg.height = int(depth_image.shape[0])
                    depth_msg.width = int(depth_image.shape[1])
                    depth_msg.encoding = "16UC1"
                    depth_msg.is_bigendian = False
                    depth_msg.step = int(depth_image.shape[1] * 2)
                    depth_msg.data = depth_image.tobytes()
                    self.depth_pub.publish(depth_msg)
        except Exception as exc:
            self.get_logger().warn(f"frame publish failed: {exc}")

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "pipeline"):
                self.pipeline.stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealsenseCameraNode()
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

