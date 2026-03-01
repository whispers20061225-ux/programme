import logging
import time
from typing import Any, Dict, Optional

from PyQt5.QtCore import QMutex, QThread, QWaitCondition, pyqtSignal

from core.data_acquisition import DataBuffer, SensorData

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from tactile_interfaces.msg import SystemHealth, TactileRaw

    ROS2_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - optional dependency at runtime
    rclpy = None
    SingleThreadedExecutor = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    HistoryPolicy = None
    TactileRaw = None
    SystemHealth = None
    ROS2_IMPORT_ERROR = exc


class _Ros2TactileNode(Node):
    """Internal ROS2 node used by the Qt data thread."""

    def __init__(self, owner: "Ros2DataAcquisitionThread", tactile_topic: str, health_topic: str):
        super().__init__("ros2_tactile_acquisition")
        self._owner = owner

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(TactileRaw, tactile_topic, self._on_tactile, sensor_qos)
        self.create_subscription(SystemHealth, health_topic, self._on_health, health_qos)

    def _on_tactile(self, msg: Any) -> None:
        self._owner.ingest_tactile(msg)

    def _on_health(self, msg: Any) -> None:
        self._owner.ingest_health(msg)


class Ros2DataAcquisitionThread(QThread):
    """ROS2-based tactile acquisition thread for phase 1 migration."""

    new_data = pyqtSignal(object)
    status_changed = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, dict)

    def __init__(self, config: Any, tactile_topic: str = "/tactile/raw", health_topic: str = "/system/health"):
        super().__init__()
        self.config = config
        self.tactile_topic = tactile_topic
        self.health_topic = health_topic

        self.running = False
        self.paused = False
        self.mutex = QMutex()
        self.condition = QWaitCondition()

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

            self._node = _Ros2TactileNode(self, self.tactile_topic, self.health_topic)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)

            self.status_changed.emit(
                "running",
                {
                    "message": "ROS2 tactile acquisition running",
                    "tactile_topic": self.tactile_topic,
                    "health_topic": self.health_topic,
                },
            )

            while self.running:
                self.mutex.lock()
                if self.paused:
                    self.condition.wait(self.mutex)
                self.mutex.unlock()

                if not self.running:
                    break

                self._executor.spin_once(timeout_sec=0.05)

                now = time.time()
                if now - self._status_emit_ts >= 1.0:
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

    def start_acquisition(self) -> None:
        if not self.running:
            self.start()
        elif self.paused:
            self.resume_acquisition()

    def stop_acquisition(self) -> None:
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
            "topics": {"tactile": self.tactile_topic, "health": self.health_topic},
        }

    def is_running(self) -> bool:
        return self.running and not self.paused

    def is_paused(self) -> bool:
        return self.paused
