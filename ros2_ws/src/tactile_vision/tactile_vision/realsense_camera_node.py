from __future__ import annotations

import threading
import time
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
    """Publish RealSense color/depth stream without depending on realsense2_camera package.

    Architecture:
    - capture thread: blocks on wait_for_frames and stores the latest frame payload
    - timer callback: publishes cached payload only when a new frame is available
    """

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
        self.declare_parameter("frame_timeout_ms", 350)
        self.declare_parameter("max_consecutive_timeouts", 15)
        self.declare_parameter("restart_cooldown_sec", 5.0)
        self.declare_parameter("capture_stale_sec", 3.0)
        self.declare_parameter("publish_only_when_new_frame", True)

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
        self.frame_timeout_ms = max(100, int(self.get_parameter("frame_timeout_ms").value))
        self.max_consecutive_timeouts = max(3, int(self.get_parameter("max_consecutive_timeouts").value))
        self.restart_cooldown_sec = max(1.0, float(self.get_parameter("restart_cooldown_sec").value))
        self.capture_stale_sec = max(1.0, float(self.get_parameter("capture_stale_sec").value))
        self.publish_only_when_new_frame = bool(self.get_parameter("publish_only_when_new_frame").value)

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.color_pub = self.create_publisher(Image, self.color_topic, qos_sensor)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, qos_sensor)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, qos_sensor)

        self.pipeline: Optional["rs.pipeline"] = None
        self.profile = None
        self.aligner: Optional["rs.align"] = None
        self.color_info_template: Optional[CameraInfo] = None

        self._pipeline_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._restart_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._capture_thread: Optional[threading.Thread] = None
        self._restart_reason: Optional[str] = None

        self._latest_seq = 0
        self._last_published_seq = 0
        self._latest_color: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_capture_time = 0.0
        self._consecutive_timeouts = 0
        self._last_restart_time = 0.0
        self._start_pipeline(reason="initial", raise_on_failure=True)
        self._start_capture_thread()

        publish_hz = max(self.color_fps if self.enable_color else 0, self.depth_fps if self.enable_depth else 0, 5)
        self.create_timer(1.0 / float(publish_hz), self._on_timer)

        self.get_logger().info(
            "realsense_camera_node started: "
            f"serial={self.serial_no or 'auto'}, color={self.enable_color}, depth={self.enable_depth}, "
            f"align_depth={self.align_depth}, color_topic={self.color_topic}, depth_topic={self.depth_topic}, "
            f"frame_timeout_ms={self.frame_timeout_ms}, max_consecutive_timeouts={self.max_consecutive_timeouts}, "
            f"capture_stale_sec={self.capture_stale_sec}"
        )

    def _build_config(self) -> "rs.config":
        config = rs.config()
        if self.serial_no:
            config.enable_device(self.serial_no)

        if self.enable_color:
            config.enable_stream(
                rs.stream.color,
                self.color_width,
                self.color_height,
                rs.format.rgb8,
                self.color_fps,
            )
        if self.enable_depth:
            config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.depth_fps,
            )
        return config

    def _start_capture_thread(self) -> None:
        if self._capture_thread is not None and self._capture_thread.is_alive():
            return
        self._stop_event.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="realsense_capture_thread",
            daemon=True,
        )
        self._capture_thread.start()

    def _stop_capture_thread(self) -> None:
        self._stop_event.set()
        if self._capture_thread is not None and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        self._capture_thread = None

    def _stop_pipeline(self) -> None:
        with self._pipeline_lock:
            if self.pipeline is None:
                return
            try:
                self.pipeline.stop()
            except Exception:
                pass
            finally:
                self.pipeline = None
                self.profile = None
                self.aligner = None
                self.color_info_template = None

    def _start_pipeline(self, reason: str, raise_on_failure: bool = False) -> bool:
        now = time.monotonic()
        if reason != "initial" and now - self._last_restart_time < self.restart_cooldown_sec:
            return False

        self._last_restart_time = now
        self._stop_pipeline()
        pipeline = rs.pipeline()
        config = self._build_config()

        try:
            profile = pipeline.start(config)
        except Exception as exc:
            detected = self._detect_realsense_devices()
            requested = self.serial_no or "auto"
            if reason == "initial":
                if detected:
                    message = (
                        "Failed to start RealSense pipeline: "
                        f"requested_serial={requested}, detected={detected}, error={exc}"
                    )
                    if raise_on_failure:
                        raise RuntimeError(message) from exc
                    self.get_logger().warn(message)
                    return False
                message = (
                    "No RealSense device detected by pyrealsense2. "
                    f"requested_serial={requested}. Check USB ownership (VM), cable, and Intel RealSense driver."
                )
                if raise_on_failure:
                    raise RuntimeError(message) from exc
                self.get_logger().warn(message)
                return False
            self.get_logger().warn(
                f"pipeline restart failed ({reason}): requested_serial={requested}, "
                f"detected={detected or 'none'}, error={exc}"
            )
            return False

        with self._pipeline_lock:
            self.pipeline = pipeline
            self.profile = profile
            if self.enable_color and self.enable_depth and self.align_depth:
                self.aligner = rs.align(rs.stream.color)
            else:
                self.aligner = None
            self.color_info_template = self._make_camera_info_template(rs.stream.color, self.color_frame_id)
        self._consecutive_timeouts = 0
        self._latest_capture_time = time.monotonic()
        if reason != "initial":
            self.get_logger().warn(f"pipeline restarted successfully (reason={reason})")
        return True

    def _request_restart(self, reason: str) -> None:
        with self._restart_lock:
            if self._restart_reason is None:
                self._restart_reason = reason

    def _take_restart_reason(self) -> Optional[str]:
        with self._restart_lock:
            reason = self._restart_reason
            self._restart_reason = None
            return reason

    def _handle_capture_miss(self, reason: str) -> None:
        self._consecutive_timeouts += 1
        if self._consecutive_timeouts == 1 or self._consecutive_timeouts % 10 == 0:
            self.get_logger().warn(
                f"frame miss count={self._consecutive_timeouts}, reason={reason}"
            )
        if self._consecutive_timeouts >= self.max_consecutive_timeouts:
            self._request_restart("frame_timeout_recovery")

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            restart_reason = self._take_restart_reason()
            if restart_reason is not None:
                self._start_pipeline(reason=restart_reason, raise_on_failure=False)

            with self._pipeline_lock:
                pipeline = self.pipeline
                aligner = self.aligner

            if pipeline is None:
                self._start_pipeline(reason="pipeline_not_started_capture", raise_on_failure=False)
                time.sleep(0.1)
                continue

            try:
                frames = pipeline.wait_for_frames(timeout_ms=self.frame_timeout_ms)
                if aligner is not None:
                    frames = aligner.process(frames)
            except Exception as exc:
                self._handle_capture_miss(reason=f"wait/align failed: {exc}")
                continue

            color_np: Optional[np.ndarray] = None
            depth_np: Optional[np.ndarray] = None

            if self.enable_color:
                color_frame = frames.get_color_frame()
                if color_frame:
                    color_np = np.asanyarray(color_frame.get_data()).copy()

            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_np = np.asanyarray(depth_frame.get_data()).copy()

            if color_np is None and depth_np is None:
                self._handle_capture_miss(reason="frameset without color/depth payload")
                continue

            with self._frame_lock:
                self._latest_color = color_np
                self._latest_depth = depth_np
                self._latest_seq += 1
                self._latest_capture_time = time.monotonic()
            self._consecutive_timeouts = 0

    @staticmethod
    def _detect_realsense_devices() -> list[str]:
        devices = []
        try:
            ctx = rs.context()
            for dev in ctx.query_devices():
                name = "unknown"
                serial = "unknown"
                try:
                    if dev.supports(rs.camera_info.name):
                        name = dev.get_info(rs.camera_info.name)
                except Exception:
                    pass
                try:
                    if dev.supports(rs.camera_info.serial_number):
                        serial = dev.get_info(rs.camera_info.serial_number)
                except Exception:
                    pass
                devices.append(f"{name}:{serial}")
        except Exception:
            pass
        return devices

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
        if self._capture_thread is None or not self._capture_thread.is_alive():
            self.get_logger().warn("capture thread is not alive, restarting capture thread")
            self._start_capture_thread()
            return

        now_mono = time.monotonic()
        if self._latest_capture_time > 0.0 and now_mono - self._latest_capture_time > self.capture_stale_sec:
            self._request_restart("capture_stale")
            return

        with self._frame_lock:
            seq = self._latest_seq
            color_np = self._latest_color
            depth_np = self._latest_depth

        if seq <= 0:
            return
        if self.publish_only_when_new_frame and seq == self._last_published_seq:
            return

        now = self.get_clock().now().to_msg()
        published_any = False

        if color_np is not None:
            color_msg = Image()
            color_msg.header.stamp = now
            color_msg.header.frame_id = self.color_frame_id
            color_msg.height = int(color_np.shape[0])
            color_msg.width = int(color_np.shape[1])
            color_msg.encoding = "rgb8"
            color_msg.is_bigendian = False
            color_msg.step = int(color_np.shape[1] * 3)
            color_msg.data = color_np.tobytes()
            self.color_pub.publish(color_msg)
            published_any = True

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

        if depth_np is not None:
            depth_msg = Image()
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self.depth_frame_id
            depth_msg.height = int(depth_np.shape[0])
            depth_msg.width = int(depth_np.shape[1])
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = False
            depth_msg.step = int(depth_np.shape[1] * 2)
            depth_msg.data = depth_np.tobytes()
            self.depth_pub.publish(depth_msg)
            published_any = True

        if published_any:
            self._last_published_seq = seq

    def destroy_node(self) -> bool:
        self._stop_capture_thread()
        self._stop_pipeline()
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
