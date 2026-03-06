from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from threading import Lock
from typing import Any, Dict, Optional

import numpy as np

from PyQt5.QtCore import QMutex, QThread, QWaitCondition, pyqtSignal

from core.data_acquisition import DataBuffer, SensorData

try:
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
    try:
        from rclpy.exceptions import RCLError
    except Exception:
        RCLError = Exception
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from tactile_interfaces.msg import SystemHealth, TactileRaw

    ROS2_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - optional dependency at runtime
    rclpy = None
    ExternalShutdownException = Exception
    RCLError = Exception
    MultiThreadedExecutor = None
    SingleThreadedExecutor = None
    MutuallyExclusiveCallbackGroup = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    HistoryPolicy = None
    TactileRaw = None
    SystemHealth = None
    Image = None
    CameraInfo = None
    ROS2_IMPORT_ERROR = exc


@dataclass
class Ros2CameraFrame:
    """Shared camera frame structure consumed by MainWindow vision pipeline."""

    timestamp: float
    color_image: Optional[np.ndarray]
    depth_image: Optional[np.ndarray]
    intrinsics: Dict[str, float]


class _Ros2AcquisitionNode(Node):
    """Internal ROS2 node used by the Qt data thread."""

    def __init__(
        self,
        owner: "Ros2DataAcquisitionThread",
        tactile_topic: str,
        health_topic: str,
        *,
        vision_enabled: bool,
        color_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        vision_qos_mode: str,
    ):
        super().__init__("ros2_tactile_acquisition")
        self._owner = owner

        tactile_group = MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup is not None else None
        health_group = MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup is not None else None
        vision_group = MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup is not None else None

        tactile_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        mode = str(vision_qos_mode or "auto").strip().lower()
        vision_reliability = self._resolve_vision_reliability(
            mode=mode,
            color_topic=color_topic,
            depth_topic=depth_topic,
            camera_info_topic=camera_info_topic,
        )
        # Vision path only needs newest frame; keep queue depth=1 to reduce backlog/latency.
        vision_qos = QoSProfile(
            reliability=vision_reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        tactile_kwargs = {"callback_group": tactile_group} if tactile_group is not None else {}
        health_kwargs = {"callback_group": health_group} if health_group is not None else {}
        vision_kwargs = {"callback_group": vision_group} if vision_group is not None else {}

        self.create_subscription(TactileRaw, tactile_topic, self._on_tactile, tactile_qos, **tactile_kwargs)
        self.create_subscription(SystemHealth, health_topic, self._on_health, health_qos, **health_kwargs)
        if vision_enabled:
            self.get_logger().info(
                f"Vision QoS resolved: requested={mode} effective={self._reliability_label(vision_reliability)}"
            )
            self.create_subscription(Image, color_topic, self._on_color, vision_qos, **vision_kwargs)
            self.create_subscription(Image, depth_topic, self._on_depth, vision_qos, **vision_kwargs)
            self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, vision_qos, **vision_kwargs)

    def _resolve_vision_reliability(
        self,
        *,
        mode: str,
        color_topic: str,
        depth_topic: str,
        camera_info_topic: str,
    ):
        if mode == "reliable":
            return ReliabilityPolicy.RELIABLE
        if mode == "best_effort":
            return ReliabilityPolicy.BEST_EFFORT

        saw_reliable = False
        for topic in (color_topic, depth_topic, camera_info_topic):
            topic_infos = self._get_publishers_info(topic)
            for info in topic_infos:
                qos_profile = getattr(info, "qos_profile", None)
                reliability = getattr(qos_profile, "reliability", None)
                if self._reliability_matches(reliability, ReliabilityPolicy.BEST_EFFORT):
                    return ReliabilityPolicy.BEST_EFFORT
                if self._reliability_matches(reliability, ReliabilityPolicy.RELIABLE):
                    saw_reliable = True

        if saw_reliable:
            return ReliabilityPolicy.RELIABLE

        # Compatibility-first fallback when publisher QoS cannot be introspected yet.
        return ReliabilityPolicy.BEST_EFFORT

    def _get_publishers_info(self, topic: str):
        try:
            return list(self.get_publishers_info_by_topic(topic))
        except TypeError:
            try:
                return list(self.get_publishers_info_by_topic(topic, False))
            except Exception:
                return []
        except Exception:
            return []

    @staticmethod
    def _reliability_matches(value, expected_policy) -> bool:
        if value == expected_policy:
            return True

        raw_value = getattr(value, "value", value)
        expected_raw = getattr(expected_policy, "value", expected_policy)
        try:
            if int(raw_value) == int(expected_raw):
                return True
        except Exception:
            pass

        value_name = str(getattr(value, "name", raw_value)).strip().lower()
        expected_name = str(getattr(expected_policy, "name", expected_raw)).strip().lower()
        return value_name == expected_name

    @staticmethod
    def _reliability_label(policy) -> str:
        name = str(getattr(policy, "name", policy)).strip().lower()
        if name:
            return name
        raw_value = getattr(policy, "value", policy)
        return str(raw_value)

    def _on_tactile(self, msg: Any) -> None:
        self._owner.ingest_tactile(msg)

    def _on_health(self, msg: Any) -> None:
        self._owner.ingest_health(msg)

    def _on_color(self, msg: Any) -> None:
        self._owner.ingest_vision_color(msg)

    def _on_depth(self, msg: Any) -> None:
        self._owner.ingest_vision_depth(msg)

    def _on_camera_info(self, msg: Any) -> None:
        self._owner.ingest_vision_camera_info(msg)


class Ros2DataAcquisitionThread(QThread):
    """ROS2-based tactile acquisition thread for phase 1 migration."""

    new_data = pyqtSignal(object)
    vision_frame = pyqtSignal(object)
    vision_status = pyqtSignal(dict)
    status_changed = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, dict)

    def __init__(
        self,
        config: Any,
        tactile_topic: str = "/tactile/raw",
        health_topic: str = "/system/health",
        *,
        vision_enabled: bool = False,
        color_topic: str = "/camera/camera/color/image_raw",
        depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw",
        camera_info_topic: str = "/camera/camera/color/camera_info",
        vision_stale_timeout_sec: float = 1.5,
        vision_max_fps: float = 30.0,
        vision_emit_signal: bool = False,
        vision_error_log_interval_sec: float = 2.0,
        vision_qos_mode: str = "auto",
    ):
        super().__init__()
        self.config = config
        self.tactile_topic = tactile_topic
        self.health_topic = health_topic

        self.vision_enabled = bool(vision_enabled)
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.vision_stale_timeout_sec = max(0.2, float(vision_stale_timeout_sec))
        self.vision_max_fps = max(1.0, float(vision_max_fps))
        self.vision_emit_signal = bool(vision_emit_signal)
        self.vision_error_log_interval_sec = max(0.2, float(vision_error_log_interval_sec))
        self.vision_qos_mode = str(vision_qos_mode or "auto").strip().lower()

        self.running = False
        self.paused = False
        self.mutex = QMutex()
        self.condition = QWaitCondition()
        self._vision_lock = Lock()

        self.data_buffer = DataBuffer(max_size=1000)
        self.read_count = 0
        self.error_count = 0
        self.start_time = 0.0
        self._last_data_ts = 0.0
        self._status_emit_ts = 0.0
        self._sequence_id = 0

        self.data_callback = None
        self._ros_owned_context = False
        self._node = None
        self._executor = None
        self.logger = logging.getLogger(__name__)

        # Vision runtime caches.
        self._vision_stream_requested = False
        self._vision_connected = False
        self._vision_status_emit_ts = 0.0
        self._vision_last_emit_ts = 0.0
        self._vision_last_color_ts = 0.0
        self._vision_last_depth_ts = 0.0
        self._vision_latest_color: Optional[np.ndarray] = None
        self._vision_latest_depth: Optional[np.ndarray] = None
        self._vision_intrinsics: Dict[str, float] = {}
        self._vision_device_info = "ROS2 camera stream"
        self._vision_resolution = "N/A"
        self._vision_depth_profile = "off"
        self._vision_depth_target_fps = 0.0
        self._vision_last_depth_decode_ts = 0.0
        self._vision_color_count = 0
        self._vision_prev_color_count = 0
        self._vision_prev_status_ts = 0.0
        self._vision_fps = 0.0
        self._vision_dropped_frames = 0
        self._vision_queue_overwrite_count = 0
        self._vision_latest_frame: Optional[Ros2CameraFrame] = None
        self._vision_latest_frame_seq = 0
        self._vision_consumed_frame_seq = 0
        self._vision_error_last_emit_ts: Dict[str, float] = {}
        self._ui_focus_mode = "default"
        self._tactile_ui_emit_interval_sec = 1.0 / 10.0
        self._status_emit_interval_sec = 2.0
        self._tactile_ui_last_emit_ts = 0.0

    def run(self) -> None:
        if ROS2_IMPORT_ERROR is not None:
            self.error_count += 1
            self.error_occurred.emit(
                "ros2_import_error",
                {"error": f"{ROS2_IMPORT_ERROR}", "message": "rclpy or tactile_interfaces is not available"},
            )
            self.status_changed.emit("error", {"message": "ROS2 runtime unavailable"})
            return

        try:
            self.running = True
            self.start_time = time.time()
            self.status_changed.emit("starting", {"message": "ROS2 tactile acquisition starting"})

            if not rclpy.ok():
                rclpy.init(args=None)
                self._ros_owned_context = True

            self._node = _Ros2AcquisitionNode(
                self,
                self.tactile_topic,
                self.health_topic,
                vision_enabled=self.vision_enabled,
                color_topic=self.color_topic,
                depth_topic=self.depth_topic,
                camera_info_topic=self.camera_info_topic,
                vision_qos_mode=self.vision_qos_mode,
            )
            if MultiThreadedExecutor is not None:
                self._executor = MultiThreadedExecutor(num_threads=2)
            else:
                self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)

            self.status_changed.emit(
                "running",
                {
                    "message": "ROS2 tactile acquisition running",
                    "tactile_topic": self.tactile_topic,
                    "health_topic": self.health_topic,
                    "vision_enabled": self.vision_enabled,
                    "color_topic": self.color_topic if self.vision_enabled else None,
                    "depth_topic": self.depth_topic if self.vision_enabled else None,
                },
            )

            while self.running:
                self.mutex.lock()
                if self.paused:
                    self.condition.wait(self.mutex)
                self.mutex.unlock()

                if not self.running:
                    break

                try:
                    self._executor.spin_once(timeout_sec=0.02)
                except RCLError as exc:
                    if not self.running or (rclpy is not None and not rclpy.ok()):
                        break
                    raise exc

                now = time.time()
                if now - self._status_emit_ts >= self._status_emit_interval_sec:
                    self._status_emit_ts = now
                    self.status_changed.emit(
                        "running",
                        {
                            "message": "ROS2 tactile acquisition running",
                            "read_count": self.read_count,
                            "error_count": self.error_count,
                            "last_data_age_sec": (now - self._last_data_ts) if self._last_data_ts > 0 else None,
                        },
                    )
                    self._emit_vision_status(now=now, force=False)

        except ExternalShutdownException:
            # Expected during Ctrl-C / external ROS shutdown.
            pass
        except Exception as exc:
            self.error_count += 1
            self.error_occurred.emit("acquisition_error", {"error": str(exc)})
            self.logger.exception("ROS2 acquisition thread crashed")
        finally:
            try:
                if self._executor is not None and self._node is not None:
                    self._executor.remove_node(self._node)
            except Exception:
                pass
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:
                pass
            if self._ros_owned_context and rclpy is not None and rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

            if self.vision_enabled:
                self.request_vision_disconnect()

            self.running = False
            self.status_changed.emit("stopped", {"message": "ROS2 tactile acquisition stopped"})

    def ingest_tactile(self, msg: Any) -> None:
        try:
            rows = int(getattr(msg, "rows", 0) or 0)
            cols = int(getattr(msg, "cols", 0) or 0)
            base_count = rows * cols if rows > 0 and cols > 0 else 0

            fz = list(getattr(msg, "forces_fz", []) or [])
            force_data = list(getattr(msg, "forces", []) or [])
            fx = list(getattr(msg, "forces_fx", []) or [])
            fy = list(getattr(msg, "forces_fy", []) or [])

            if not force_data:
                force_data = fz

            count = max(base_count, len(force_data), len(fz), len(fx), len(fy))
            if count <= 0:
                count = 1

            force_data = (force_data + [0.0] * count)[:count]
            fx = (fx + [0.0] * count)[:count]
            fy = (fy + [0.0] * count)[:count]
            fz = (fz + force_data + [0.0] * count)[:count]

            force_vectors = [[fx[i], fy[i], fz[i]] for i in range(count)]

            stamp = getattr(msg, "header", None)
            if stamp is not None and hasattr(stamp, "stamp"):
                ts = float(stamp.stamp.sec) + float(stamp.stamp.nanosec) / 1e9
            elif stamp is not None and hasattr(stamp, "sec"):
                ts = float(stamp.sec) + float(stamp.nanosec) / 1e9
            else:
                ts = time.time()

            data = SensorData(
                timestamp=ts,
                force_data=force_data,
                force_vectors=force_vectors,
                temperature=None,
                status=0,
                sequence_id=self._sequence_id,
            )
            self._sequence_id += 1
            self._last_data_ts = time.time()
            self.read_count += 1

            if self.data_callback is not None:
                try:
                    callback_result = self.data_callback(data)
                    if isinstance(callback_result, tuple) and len(callback_result) >= 1:
                        data = callback_result[0]
                except Exception as callback_exc:
                    self.error_count += 1
                    self.error_occurred.emit(
                        "data_callback_error",
                        {"error": str(callback_exc)},
                    )

            self.data_buffer.add_data(data)
            emit_now = time.time()
            if (emit_now - self._tactile_ui_last_emit_ts) >= self._tactile_ui_emit_interval_sec:
                self._tactile_ui_last_emit_ts = emit_now
                self.new_data.emit(data)
        except Exception as exc:
            self.error_count += 1
            self.error_occurred.emit("message_parse_error", {"error": str(exc)})

    def ingest_health(self, msg: Any) -> None:
        try:
            info = {
                "node_name": str(getattr(msg, "node_name", "")),
                "healthy": bool(getattr(msg, "healthy", True)),
                "level": int(getattr(msg, "level", 0)),
                "message": str(getattr(msg, "message", "")),
            }
            if info["healthy"] and info["level"] <= 1:
                return
            self.status_changed.emit("health_warning", info)
        except Exception:
            pass

    def request_vision_connect(self) -> None:
        if not self.vision_enabled:
            self.vision_status.emit(
                {
                    "connected": False,
                    "streaming": False,
                    "message": "Vision subscription is disabled in current ROS2 run.",
                    "simulation": False,
                }
            )
            return
        with self._vision_lock:
            self._vision_stream_requested = True
            self._vision_connected = False
            self._vision_status_emit_ts = 0.0
            self._vision_dropped_frames = 0
            self._vision_queue_overwrite_count = 0
        self._emit_vision_status(now=time.time(), force=True, message="Waiting for ROS2 camera frames...")

    def request_vision_disconnect(self) -> None:
        with self._vision_lock:
            self._vision_stream_requested = False
            self._vision_connected = False
            self._vision_latest_color = None
            self._vision_latest_depth = None
            self._vision_latest_frame = None
            self._vision_fps = 0.0
            self._vision_dropped_frames = 0
            self._vision_queue_overwrite_count = 0
            self._vision_resolution = "N/A"
            self._vision_depth_profile = "off"
            self._vision_depth_target_fps = 0.0
            self._vision_last_depth_decode_ts = 0.0
            self._vision_last_depth_ts = 0.0
        self.vision_status.emit(
            {
                "connected": False,
                "streaming": False,
                "fps": 0.0,
                "rx_fps": 0.0,
                "render_fps": 0.0,
                "dropped_frames": 0,
                "queue_overwrite_count": 0,
                "last_frame_age_ms": None,
                "stall_count": 0,
                "resolution": "N/A",
                "message": "ROS2 vision stream disconnected",
                "simulation": False,
                "device_info": self._vision_device_info,
            }
        )

    def set_vision_depth_profile(self, profile: str) -> None:
        profile_key = str(profile or "off").strip().lower()
        depth_fps_map = {
            "off": 0.0,
            "analysis": 8.0,
            "depth": 12.0,
            "pointcloud": 8.0,
        }
        if profile_key not in depth_fps_map:
            profile_key = "off"
        with self._vision_lock:
            if self._vision_depth_profile == profile_key:
                return
            self._vision_depth_profile = profile_key
            self._vision_depth_target_fps = depth_fps_map[profile_key]
            self._vision_last_depth_decode_ts = 0.0
            if self._vision_depth_target_fps <= 0.0:
                self._vision_latest_depth = None
                self._vision_last_depth_ts = 0.0
        self.logger.info("ROS2 vision depth profile: %s (target_fps=%.1f)", profile_key, depth_fps_map[profile_key])

    def ingest_vision_color(self, msg: Any) -> None:
        if not self.vision_enabled:
            return
        try:
            frame_ts = self._message_stamp_to_time(msg)
            now = time.time()
            image = self._decode_image_message(msg)
            image = self._normalize_color_image(image, str(getattr(msg, "encoding", "")))
            if image is None:
                return

            emit_frame: Optional[Ros2CameraFrame] = None
            with self._vision_lock:
                if not self._vision_stream_requested:
                    return
                self._vision_latest_color = image
                self._vision_last_color_ts = now
                self._vision_color_count += 1
                h, w = image.shape[:2]
                self._vision_resolution = f"{w}x{h}"
                if (now - self._vision_last_emit_ts) >= (1.0 / self.vision_max_fps):
                    self._vision_last_emit_ts = now
                    if self._vision_latest_frame is not None and self._vision_latest_frame_seq != self._vision_consumed_frame_seq:
                        self._vision_queue_overwrite_count += 1
                    emit_frame = Ros2CameraFrame(
                        timestamp=frame_ts if frame_ts > 0 else now,
                        color_image=self._vision_latest_color,
                        depth_image=self._vision_latest_depth,
                        intrinsics=dict(self._vision_intrinsics),
                    )
                    self._vision_latest_frame = emit_frame
                    self._vision_latest_frame_seq += 1
                    self._vision_connected = True
                else:
                    self._vision_dropped_frames += 1
            if emit_frame is not None and self.vision_emit_signal:
                self.vision_frame.emit(emit_frame)
            self._emit_vision_status(now=now, force=False)
        except Exception as exc:
            self.error_count += 1
            self._emit_vision_error_throttled("vision_color_parse_error", exc)

    def ingest_vision_depth(self, msg: Any) -> None:
        if not self.vision_enabled:
            return
        try:
            now = time.time()
            with self._vision_lock:
                if not self._vision_stream_requested:
                    return
                target_fps = float(self._vision_depth_target_fps)
                last_decode_ts = float(self._vision_last_depth_decode_ts)
            if target_fps <= 0.0:
                return
            min_interval = 1.0 / target_fps if target_fps > 0.0 else 0.0
            if min_interval > 0.0 and last_decode_ts > 0.0 and (now - last_decode_ts) < min_interval:
                return
            depth = self._decode_image_message(msg)
            if depth is None:
                return
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            with self._vision_lock:
                if not self._vision_stream_requested:
                    return
                self._vision_latest_depth = depth
                self._vision_last_depth_ts = now
                self._vision_last_depth_decode_ts = now
            self._emit_vision_status(now=now, force=False)
        except Exception as exc:
            self.error_count += 1
            self._emit_vision_error_throttled("vision_depth_parse_error", exc)

    def ingest_vision_camera_info(self, msg: Any) -> None:
        if not self.vision_enabled:
            return
        try:
            k_raw = getattr(msg, "k", None)
            if k_raw is None:
                k = []
            else:
                # `msg.k` may be list/array/ndarray; avoid truth-value checks on ndarray.
                try:
                    k = [float(v) for v in list(k_raw)]
                except Exception:
                    k = []
            if len(k) >= 9:
                intrinsics = {
                    "fx": float(k[0]),
                    "fy": float(k[4]),
                    "cx": float(k[2]),
                    "cy": float(k[5]),
                    "width": float(getattr(msg, "width", 0) or 0),
                    "height": float(getattr(msg, "height", 0) or 0),
                }
            else:
                intrinsics = {}
            frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
            with self._vision_lock:
                if intrinsics:
                    self._vision_intrinsics = intrinsics
                if frame_id:
                    self._vision_device_info = f"ROS2 frame: {frame_id}"
            self._emit_vision_status(now=time.time(), force=False)
        except Exception as exc:
            self.error_count += 1
            self._emit_vision_error_throttled("vision_info_parse_error", exc)

    def consume_latest_vision_frame(self) -> Optional[Ros2CameraFrame]:
        """Return newest frame once (queue size=1 semantics)."""
        with self._vision_lock:
            if self._vision_latest_frame is None:
                return None
            if self._vision_latest_frame_seq == self._vision_consumed_frame_seq:
                return None
            self._vision_consumed_frame_seq = self._vision_latest_frame_seq
            frame = self._vision_latest_frame
        return frame

    def _emit_vision_error_throttled(self, key: str, exc: Exception) -> None:
        now = time.time()
        last = float(self._vision_error_last_emit_ts.get(key, 0.0))
        if now - last < self.vision_error_log_interval_sec:
            return
        self._vision_error_last_emit_ts[key] = now
        self.error_occurred.emit(key, {"error": str(exc)})

    @staticmethod
    def _normalize_color_image(image: np.ndarray, encoding: str) -> Optional[np.ndarray]:
        if image is None:
            return None
        enc = str(encoding or "").lower()
        if image.ndim == 2:
            return np.repeat(image[:, :, np.newaxis], 3, axis=2)
        if image.ndim != 3:
            return None
        channels = image.shape[2]
        if channels == 4:
            if "bgra" in enc:
                return image[:, :, :3][:, :, ::-1]
            return image[:, :, :3]
        if channels == 3:
            if "bgr" in enc:
                return image[:, :, ::-1]
            return image
        if channels == 1:
            mono = image[:, :, 0]
            return np.repeat(mono[:, :, np.newaxis], 3, axis=2)
        return None

    @staticmethod
    def _decode_image_message(msg: Any) -> Optional[np.ndarray]:
        encoding = str(getattr(msg, "encoding", "") or "").lower()
        height = int(getattr(msg, "height", 0) or 0)
        width = int(getattr(msg, "width", 0) or 0)
        step = int(getattr(msg, "step", 0) or 0)
        if height <= 0 or width <= 0:
            return None

        if encoding in ("rgb8", "bgr8"):
            dtype, channels = np.uint8, 3
        elif encoding in ("rgba8", "bgra8"):
            dtype, channels = np.uint8, 4
        elif encoding in ("mono8", "8uc1"):
            dtype, channels = np.uint8, 1
        elif encoding in ("mono16", "16uc1"):
            dtype, channels = np.uint16, 1
        elif encoding in ("32fc1",):
            dtype, channels = np.float32, 1
        else:
            raise ValueError(f"unsupported ROS image encoding: {encoding}")

        item_size = np.dtype(dtype).itemsize
        row_elems = (step // item_size) if step > 0 else width * channels
        expected_elems = height * row_elems

        arr = np.frombuffer(getattr(msg, "data", b""), dtype=dtype)
        if arr.size < expected_elems:
            raise ValueError(
                f"image buffer too small: got={arr.size}, expected={expected_elems}, "
                f"encoding={encoding}, width={width}, height={height}"
            )

        arr = arr[:expected_elems].reshape((height, row_elems))
        arr = arr[:, : width * channels]
        if channels > 1:
            arr = arr.reshape((height, width, channels))
        else:
            arr = arr.reshape((height, width))
        return np.array(arr, copy=True)

    @staticmethod
    def _message_stamp_to_time(msg: Any) -> float:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is not None and hasattr(stamp, "sec"):
            return float(stamp.sec) + float(stamp.nanosec) / 1e9
        return 0.0

    def _emit_vision_status(self, *, now: float, force: bool, message: str = "") -> None:
        if not self.vision_enabled:
            return
        with self._vision_lock:
            if not force and (now - self._vision_status_emit_ts) < 0.5:
                return

            elapsed = now - self._vision_prev_status_ts if self._vision_prev_status_ts > 0 else 0.0
            if elapsed > 0:
                frame_delta = self._vision_color_count - self._vision_prev_color_count
                instant_fps = frame_delta / elapsed
                if self._vision_fps <= 0.0:
                    self._vision_fps = instant_fps
                else:
                    self._vision_fps = self._vision_fps * 0.7 + instant_fps * 0.3
            self._vision_prev_color_count = self._vision_color_count
            self._vision_prev_status_ts = now
            self._vision_status_emit_ts = now

            color_fresh = (
                self._vision_last_color_ts > 0.0
                and (now - self._vision_last_color_ts) <= self.vision_stale_timeout_sec
            )
            connected = bool(self._vision_stream_requested and color_fresh)
            self._vision_connected = connected

            depth_fresh = (
                self._vision_last_depth_ts > 0.0
                and (now - self._vision_last_depth_ts) <= self.vision_stale_timeout_sec
            )
            last_frame_age_ms = None
            if self._vision_last_color_ts > 0.0:
                last_frame_age_ms = max(0.0, (now - self._vision_last_color_ts) * 1000.0)

            status = {
                "connected": connected,
                "streaming": connected,
                "simulation": False,
                "fps": float(self._vision_fps),
                "rx_fps": float(self._vision_fps),
                "render_fps": 0.0,
                "dropped_frames": int(self._vision_dropped_frames),
                "queue_overwrite_count": int(self._vision_queue_overwrite_count),
                "last_frame_age_ms": last_frame_age_ms,
                "stall_count": 0,
                "resolution": self._vision_resolution,
                "color_topic": self.color_topic,
                "depth_topic": self.depth_topic,
                "device_info": self._vision_device_info,
                "depth_ready": depth_fresh,
                "message": message or ("ROS2 camera stream active" if connected else "ROS2 camera stream waiting"),
            }
        self.vision_status.emit(status)

    def set_ui_focus_mode(self, mode: str) -> None:
        mode_key = str(mode or "default").strip().lower()
        if mode_key == self._ui_focus_mode:
            return
        self._ui_focus_mode = mode_key
        if mode_key == "vision":
            self._tactile_ui_emit_interval_sec = 0.2
        else:
            self._tactile_ui_emit_interval_sec = 1.0 / 10.0

    def start_acquisition(self) -> None:
        if not self.running:
            self.start()
        elif self.paused:
            self.resume_acquisition()

    def stop_acquisition(self) -> None:
        if self.vision_enabled:
            self.request_vision_disconnect()
        self.running = False
        self.resume_acquisition()
        self.wait()

    def pause_acquisition(self) -> None:
        self.mutex.lock()
        self.paused = True
        self.mutex.unlock()
        self.status_changed.emit("paused", {"message": "ROS2 tactile acquisition paused"})

    def resume_acquisition(self) -> None:
        self.mutex.lock()
        self.paused = False
        self.condition.wakeAll()
        self.mutex.unlock()
        self.status_changed.emit("running", {"message": "ROS2 tactile acquisition resumed"})

    def get_latest_data(self) -> Optional[SensorData]:
        return self.data_buffer.get_latest()

    def get_statistics(self) -> Dict[str, Any]:
        runtime = time.time() - self.start_time if self.start_time > 0 else 0.0
        return {
            "runtime": runtime,
            "read_count": self.read_count,
            "error_count": self.error_count,
            "read_rate": (self.read_count / runtime) if runtime > 0 else 0.0,
            "buffer_stats": self.data_buffer.get_statistics(),
            "mode": "ros2",
            "topics": {
                "tactile": self.tactile_topic,
                "health": self.health_topic,
                "vision_color": self.color_topic if self.vision_enabled else None,
                "vision_depth": self.depth_topic if self.vision_enabled else None,
                "vision_info": self.camera_info_topic if self.vision_enabled else None,
            },
            "vision": {
                "enabled": self.vision_enabled,
                "connected": self._vision_connected,
                "fps": self._vision_fps,
                "rx_fps": self._vision_fps,
                "render_fps": 0.0,
                "dropped_frames": self._vision_dropped_frames,
                "queue_overwrite_count": self._vision_queue_overwrite_count,
                "resolution": self._vision_resolution,
            },
        }

    def is_running(self) -> bool:
        return self.running and not self.paused

    def is_paused(self) -> bool:
        return self.paused
