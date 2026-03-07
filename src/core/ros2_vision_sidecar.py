from __future__ import annotations

import logging
import queue
import time
from typing import Any, Dict, Optional

import numpy as np

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    try:
        from rclpy.exceptions import RCLError
    except Exception:
        RCLError = Exception
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    ROS2_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - runtime dependency
    rclpy = None
    ExternalShutdownException = Exception
    RCLError = Exception
    SingleThreadedExecutor = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    HistoryPolicy = None
    Image = None
    CameraInfo = None
    ROS2_IMPORT_ERROR = exc


def _safe_put_latest(target_queue, item: Any) -> bool:
    try:
        target_queue.put_nowait(item)
        return False
    except queue.Full:
        try:
            target_queue.get_nowait()
        except queue.Empty:
            pass
        target_queue.put_nowait(item)
        return True


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


def _message_stamp_to_time(msg: Any) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and hasattr(stamp, "sec"):
        return float(stamp.sec) + float(stamp.nanosec) / 1e9
    return 0.0


class _VisionSidecarNode(Node):
    def __init__(
        self,
        *,
        color_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        vision_stale_timeout_sec: float,
        vision_max_fps: float,
        vision_qos_mode: str,
        frame_queue,
        status_queue,
        error_queue,
    ):
        super().__init__("programme_vision_sidecar")
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.vision_stale_timeout_sec = max(0.2, float(vision_stale_timeout_sec))
        self.vision_max_fps = max(1.0, float(vision_max_fps))
        self.vision_qos_mode = str(vision_qos_mode or "auto").strip().lower()
        self.frame_queue = frame_queue
        self.status_queue = status_queue
        self.error_queue = error_queue

        self._vision_stream_requested = False
        self._vision_connected = False
        self._vision_status_emit_ts = 0.0
        self._vision_last_emit_ts = 0.0
        self._vision_last_color_ts = 0.0
        self._vision_last_depth_ts = 0.0
        self._vision_latest_depth: Optional[np.ndarray] = None
        self._vision_intrinsics: Dict[str, float] = {}
        self._vision_device_info = "ROS2 camera stream"
        self._vision_resolution = "N/A"
        self._vision_depth_profile = "off"
        self._vision_depth_target_fps = 0.0
        self._vision_last_depth_decode_ts = 0.0
        self._vision_color_subscription_enabled = False
        self._vision_depth_subscription_enabled = False
        self._vision_info_subscription_enabled = False
        self._vision_transport_dirty = False
        self._vision_subscription_mode = (
            self.vision_qos_mode if self.vision_qos_mode in ("reliable", "best_effort") else "reliable"
        )
        self._vision_auto_qos_probe_active = False
        self._vision_auto_qos_probe_deadline_ts = 0.0
        self._vision_auto_qos_probe_color_count = 0
        self._vision_color_count = 0
        self._vision_prev_color_count = 0
        self._vision_prev_status_ts = 0.0
        self._vision_fps = 0.0
        self._vision_dropped_frames = 0
        self._vision_queue_overwrite_count = 0

        self._vision_color_sub = None
        self._vision_depth_sub = None
        self._vision_info_sub = None

    def _initial_vision_reliability(self, mode: str):
        if mode == "best_effort":
            return ReliabilityPolicy.BEST_EFFORT
        return ReliabilityPolicy.RELIABLE

    def _make_vision_qos(self, reliability=None):
        effective = reliability if reliability is not None else self._initial_vision_reliability(self._vision_subscription_mode)
        return QoSProfile(
            reliability=effective,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

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

    def configure_transport(
        self,
        *,
        color_enabled: bool,
        depth_enabled: bool,
        info_enabled: bool,
        reliability=None,
        log_change: bool = True,
    ) -> str:
        color_enabled = bool(color_enabled)
        depth_enabled = bool(depth_enabled)
        info_enabled = bool(info_enabled)
        desired_reliability = reliability if reliability is not None else self._initial_vision_reliability(self._vision_subscription_mode)
        qos_changed = not self._reliability_matches(self._initial_vision_reliability(self._vision_subscription_mode), desired_reliability)
        qos = self._make_vision_qos(desired_reliability)

        self._reconcile_subscription("_vision_color_sub", color_enabled, Image, self.color_topic, self._on_color, qos, qos_changed)
        self._reconcile_subscription("_vision_depth_sub", depth_enabled, Image, self.depth_topic, self._on_depth, qos, qos_changed)
        self._reconcile_subscription("_vision_info_sub", info_enabled, CameraInfo, self.camera_info_topic, self._on_camera_info, qos, qos_changed)

        changed = (
            qos_changed
            or color_enabled != self._vision_color_subscription_enabled
            or depth_enabled != self._vision_depth_subscription_enabled
            or info_enabled != self._vision_info_subscription_enabled
        )
        self._vision_color_subscription_enabled = color_enabled
        self._vision_depth_subscription_enabled = depth_enabled
        self._vision_info_subscription_enabled = info_enabled
        self._vision_subscription_mode = self._reliability_label(desired_reliability)

        if changed and log_change:
            self.get_logger().info(
                f"Vision transport reconfigured: qos={self._vision_subscription_mode} "
                f"color={color_enabled} depth={depth_enabled} info={info_enabled}"
            )
        return self._vision_subscription_mode

    def _reconcile_subscription(self, attr_name: str, enabled: bool, msg_type, topic: str, callback, qos, recreate: bool) -> None:
        sub = getattr(self, attr_name, None)
        if recreate and sub is not None:
            self._destroy_subscription(attr_name)
            sub = None
        if enabled:
            if sub is None:
                setattr(self, attr_name, self.create_subscription(msg_type, topic, callback, qos))
        else:
            if sub is not None:
                self._destroy_subscription(attr_name)

    def _destroy_subscription(self, attr_name: str) -> None:
        sub = getattr(self, attr_name, None)
        if sub is None:
            return
        try:
            self.destroy_subscription(sub)
        except Exception:
            pass
        setattr(self, attr_name, None)

    def request_connect(self) -> None:
        now = time.time()
        self._vision_stream_requested = True
        self._vision_connected = False
        self._vision_status_emit_ts = 0.0
        self._vision_last_emit_ts = 0.0
        self._vision_last_color_ts = 0.0
        self._vision_last_depth_ts = 0.0
        self._vision_latest_depth = None
        self._vision_color_count = 0
        self._vision_prev_color_count = 0
        self._vision_prev_status_ts = 0.0
        self._vision_fps = 0.0
        self._vision_dropped_frames = 0
        self._vision_queue_overwrite_count = 0
        self._reset_auto_qos_probe(now)
        self._update_transport_plan()
        self._emit_status(now=now, force=True, message="Waiting for ROS2 camera frames...")

    def request_disconnect(self) -> None:
        now = time.time()
        self._vision_stream_requested = False
        self._vision_connected = False
        self._vision_latest_depth = None
        self._vision_fps = 0.0
        self._vision_dropped_frames = 0
        self._vision_queue_overwrite_count = 0
        self._vision_resolution = "N/A"
        self._vision_depth_profile = "off"
        self._vision_depth_target_fps = 0.0
        self._vision_last_depth_decode_ts = 0.0
        self._vision_last_depth_ts = 0.0
        self._vision_auto_qos_probe_active = False
        self._vision_auto_qos_probe_deadline_ts = 0.0
        self._update_transport_plan()
        self._emit_status(now=now, force=True, message="ROS2 vision stream disconnected")

    def set_depth_profile(self, profile: str) -> None:
        profile_key = str(profile or "off").strip().lower()
        depth_fps_map = {"off": 0.0, "analysis": 8.0, "depth": 12.0, "pointcloud": 8.0}
        if profile_key not in depth_fps_map:
            profile_key = "off"
        if self._vision_depth_profile == profile_key:
            return
        self._vision_depth_profile = profile_key
        self._vision_depth_target_fps = depth_fps_map[profile_key]
        self._vision_last_depth_decode_ts = 0.0
        if self._vision_depth_target_fps <= 0.0:
            self._vision_latest_depth = None
            self._vision_last_depth_ts = 0.0
        self._update_transport_plan()
        self.get_logger().info(
            "ROS2 vision depth profile: %s (target_fps=%.1f)",
            profile_key,
            depth_fps_map[profile_key],
        )

    def _compute_transport_plan(self):
        color_enabled = bool(self._vision_stream_requested)
        depth_enabled = bool(color_enabled and self._vision_depth_target_fps > 0.0)
        info_enabled = bool(color_enabled and ((not self._vision_intrinsics) or depth_enabled))
        return color_enabled, depth_enabled, info_enabled

    def _reset_auto_qos_probe(self, now: float) -> None:
        mode = self.vision_qos_mode
        if mode == "best_effort":
            self._vision_subscription_mode = "best_effort"
            self._vision_auto_qos_probe_active = False
            self._vision_auto_qos_probe_deadline_ts = 0.0
            self._vision_auto_qos_probe_color_count = self._vision_color_count
            return
        self._vision_subscription_mode = "reliable"
        if mode == "auto":
            self._vision_auto_qos_probe_active = True
            self._vision_auto_qos_probe_deadline_ts = now + 2.5
            self._vision_auto_qos_probe_color_count = self._vision_color_count
        else:
            self._vision_auto_qos_probe_active = False
            self._vision_auto_qos_probe_deadline_ts = 0.0
            self._vision_auto_qos_probe_color_count = self._vision_color_count

    def _update_transport_plan(self) -> None:
        color_enabled, depth_enabled, info_enabled = self._compute_transport_plan()
        changed = (
            color_enabled != self._vision_color_subscription_enabled
            or depth_enabled != self._vision_depth_subscription_enabled
            or info_enabled != self._vision_info_subscription_enabled
        )
        self._vision_color_subscription_enabled = color_enabled
        self._vision_depth_subscription_enabled = depth_enabled
        self._vision_info_subscription_enabled = info_enabled
        if changed:
            self._vision_transport_dirty = True

    def tick(self, now: float) -> None:
        fallback_triggered = False
        if (
            self.vision_qos_mode == "auto"
            and self._vision_stream_requested
            and self._vision_auto_qos_probe_active
            and now >= self._vision_auto_qos_probe_deadline_ts
            and self._vision_color_count <= self._vision_auto_qos_probe_color_count
            and self._vision_subscription_mode != "best_effort"
        ):
            self._vision_subscription_mode = "best_effort"
            self._vision_auto_qos_probe_active = False
            self._vision_auto_qos_probe_deadline_ts = 0.0
            self._vision_transport_dirty = True
            fallback_triggered = True

        if fallback_triggered:
            self.get_logger().warning(
                "ROS2 vision QoS fallback: reliable subscription produced no color frames, switching to best_effort"
            )

        if self._vision_transport_dirty:
            self._vision_transport_dirty = False
            reliability = ReliabilityPolicy.RELIABLE if self._vision_subscription_mode == "reliable" else ReliabilityPolicy.BEST_EFFORT
            self.configure_transport(
                color_enabled=self._vision_color_subscription_enabled,
                depth_enabled=self._vision_depth_subscription_enabled,
                info_enabled=self._vision_info_subscription_enabled,
                reliability=reliability,
                log_change=True,
            )

        self._emit_status(now=now, force=False)

    def _on_color(self, msg: Any) -> None:
        try:
            now = time.time()
            frame_ts = _message_stamp_to_time(msg)
            image = _normalize_color_image(_decode_image_message(msg), str(getattr(msg, "encoding", "")))
            if image is None or not self._vision_stream_requested:
                return

            self._vision_last_color_ts = now
            self._vision_color_count += 1
            if self._vision_auto_qos_probe_active and self._vision_color_count > self._vision_auto_qos_probe_color_count:
                self._vision_auto_qos_probe_active = False
                self._vision_auto_qos_probe_deadline_ts = 0.0

            h, w = image.shape[:2]
            self._vision_resolution = f"{w}x{h}"
            if (now - self._vision_last_emit_ts) < (1.0 / self.vision_max_fps):
                self._vision_dropped_frames += 1
                return

            self._vision_last_emit_ts = now
            payload = {
                "timestamp": frame_ts if frame_ts > 0 else now,
                "color_image": image,
                "depth_image": self._vision_latest_depth,
                "intrinsics": dict(self._vision_intrinsics),
            }
            if _safe_put_latest(self.frame_queue, payload):
                self._vision_queue_overwrite_count += 1
            self._vision_connected = True
        except Exception as exc:
            self._emit_error("vision_color_parse_error", exc)

    def _on_depth(self, msg: Any) -> None:
        try:
            if not self._vision_stream_requested:
                return
            target_fps = float(self._vision_depth_target_fps)
            if target_fps <= 0.0:
                return
            now = time.time()
            min_interval = 1.0 / target_fps if target_fps > 0.0 else 0.0
            if min_interval > 0.0 and self._vision_last_depth_decode_ts > 0.0 and (now - self._vision_last_depth_decode_ts) < min_interval:
                return
            depth = _decode_image_message(msg)
            if depth is None:
                return
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            self._vision_latest_depth = depth
            self._vision_last_depth_ts = now
            self._vision_last_depth_decode_ts = now
        except Exception as exc:
            self._emit_error("vision_depth_parse_error", exc)

    def _on_camera_info(self, msg: Any) -> None:
        try:
            k_raw = getattr(msg, "k", None)
            if k_raw is None:
                k = []
            else:
                try:
                    k = [float(v) for v in list(k_raw)]
                except Exception:
                    k = []
            if len(k) >= 9:
                self._vision_intrinsics = {
                    "fx": float(k[0]),
                    "fy": float(k[4]),
                    "cx": float(k[2]),
                    "cy": float(k[5]),
                    "width": float(getattr(msg, "width", 0) or 0),
                    "height": float(getattr(msg, "height", 0) or 0),
                }
                if self._vision_depth_target_fps <= 0.0:
                    self._update_transport_plan()
            frame_id = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
            if frame_id:
                self._vision_device_info = f"ROS2 frame: {frame_id}"
        except Exception as exc:
            self._emit_error("vision_info_parse_error", exc)

    def _emit_status(self, *, now: float, force: bool, message: str = "") -> None:
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

        color_fresh = self._vision_last_color_ts > 0.0 and (now - self._vision_last_color_ts) <= self.vision_stale_timeout_sec
        connected = bool(self._vision_stream_requested and color_fresh)
        self._vision_connected = connected

        depth_fresh = self._vision_last_depth_ts > 0.0 and (now - self._vision_last_depth_ts) <= self.vision_stale_timeout_sec
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
            "subscription_mode": self._vision_subscription_mode,
            "depth_subscribed": bool(self._vision_depth_subscription_enabled),
            "info_subscribed": bool(self._vision_info_subscription_enabled),
            "message": message or ("ROS2 camera stream active" if connected else "ROS2 camera stream waiting"),
        }
        _safe_put_latest(self.status_queue, status)

    def _emit_error(self, key: str, exc: Exception) -> None:
        _safe_put_latest(self.error_queue, {"status": key, "info": {"error": str(exc)}})


def run_vision_sidecar(*, config: Dict[str, Any], command_queue, frame_queue, status_queue, error_queue) -> None:
    logging.basicConfig(
        level=getattr(logging, str(config.get("log_level", "INFO")).upper(), logging.INFO),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
    logger = logging.getLogger(__name__)

    if ROS2_IMPORT_ERROR is not None:
        _safe_put_latest(
            error_queue,
            {
                "status": "vision_sidecar_import_error",
                "info": {"error": str(ROS2_IMPORT_ERROR), "message": "rclpy or sensor_msgs is not available"},
            },
        )
        return

    own_context = False
    node = None
    executor = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
            own_context = True

        node = _VisionSidecarNode(
            color_topic=str(config["color_topic"]),
            depth_topic=str(config["depth_topic"]),
            camera_info_topic=str(config["camera_info_topic"]),
            vision_stale_timeout_sec=float(config["vision_stale_timeout_sec"]),
            vision_max_fps=float(config["vision_max_fps"]),
            vision_qos_mode=str(config["vision_qos_mode"]),
            frame_queue=frame_queue,
            status_queue=status_queue,
            error_queue=error_queue,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        running = True
        while running:
            while True:
                try:
                    cmd = command_queue.get_nowait()
                except queue.Empty:
                    break
                action = str(cmd.get("action", "")).strip().lower()
                if action == "shutdown":
                    running = False
                    break
                if action == "connect":
                    node.request_connect()
                elif action == "disconnect":
                    node.request_disconnect()
                elif action == "set_depth_profile":
                    node.set_depth_profile(str(cmd.get("profile", "off")))
            if not running:
                break

            try:
                executor.spin_once(timeout_sec=0.02)
            except RCLError as exc:
                if not rclpy.ok():
                    break
                raise exc
            except ExternalShutdownException:
                break
            node.tick(time.time())
    except Exception as exc:
        logger.exception("ROS2 vision sidecar crashed")
        _safe_put_latest(error_queue, {"status": "vision_sidecar_error", "info": {"error": str(exc)}})
    finally:
        try:
            if executor is not None and node is not None:
                executor.remove_node(node)
        except Exception:
            pass
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        if own_context and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
