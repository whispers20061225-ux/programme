import json
import logging
import time
from typing import Any, Dict, Optional

from PyQt5.QtCore import QObject, QThread, pyqtSignal


class Ros2ControlThreadStub(QThread):
    """Phase 1 control stub.

    Keeps GUI/control signal wiring alive in ROS2 read-only mode.
    """

    status_updated = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, dict)

    def __init__(self) -> None:
        super().__init__()
        self._running = False
        self.logger = logging.getLogger(__name__)

    def run(self) -> None:
        self._running = True
        self.status_updated.emit(
            "ros2_mode_ready",
            {"message": "ROS2 phase 1 monitor mode is active. Control path is disabled."},
        )
        while self._running:
            time.sleep(0.2)

    def stop_control(self) -> None:
        self._running = False

    def send_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> None:
        self.status_updated.emit(
            "ros2_command_ignored",
            {
                "command": command,
                "params": params or {},
                "message": "Command ignored in ROS2 phase 1 monitor mode.",
            },
        )


class Ros2DemoManagerStub(QObject):
    """Minimal demo manager replacement for phase 1 read-only ROS2 mode."""

    status_changed = pyqtSignal(str, dict)
    error_occurred = pyqtSignal(str, dict)
    demo_started = pyqtSignal(str, dict)
    demo_stopped = pyqtSignal(str, dict)
    demo_progress = pyqtSignal(float, dict)
    vector_data_updated = pyqtSignal(object)
    contact_map_updated = pyqtSignal(object)
    tactile_mapping_ready = pyqtSignal(object)

    def __init__(self, config: Any, data_acquisition: Any, control_thread: Any):
        super().__init__()
        self.config = config
        self.data_acquisition = data_acquisition
        self.control_thread = control_thread
        self.hardware_interface = None
        self.logger = logging.getLogger(__name__)

    def handle_control_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> None:
        params = params or {}
        if command == "connect_hardware":
            self.status_changed.emit(
                "connected",
                {
                    "mode": "ros2_phase1",
                    "success": True,
                    "message": "Connected to ROS2 data stream (read-only).",
                },
            )
            return
        if command == "disconnect_hardware":
            self.status_changed.emit(
                "disconnected",
                {
                    "mode": "ros2_phase1",
                    "success": True,
                    "message": "Disconnected from ROS2 monitor mode.",
                },
            )
            return
        if command == "start_demo":
            demo_name = str(params.get("demo_name", "ros2_phase1"))
            self.demo_started.emit(demo_name, {"mode": "ros2_phase1", "params": params})
            self.status_changed.emit(
                "demo_started",
                {
                    "mode": "ros2_phase1",
                    "message": "Demo orchestration is not migrated in phase 1.",
                    "demo_name": demo_name,
                },
            )
            return
        if command == "stop_demo":
            self.demo_stopped.emit("ros2_phase1", {"mode": "ros2_phase1"})
            self.status_changed.emit(
                "demo_stopped",
                {"mode": "ros2_phase1", "message": "Demo stopped."},
            )
            return
        if command == "emergency_stop":
            self.status_changed.emit(
                "emergency_stop",
                {"mode": "ros2_phase1", "message": "Emergency stop acknowledged (monitor mode)."},
            )
            return

        self.status_changed.emit(
            "command_ignored",
            {
                "mode": "ros2_phase1",
                "command": command,
                "message": "Command is not supported in phase 1 monitor mode.",
            },
        )

    def start_demo(self, demo_name: str, params: Optional[Dict[str, Any]] = None) -> bool:
        self.handle_control_command("start_demo", {"demo_name": demo_name, **(params or {})})
        return True

    def stop_demo(self) -> bool:
        self.handle_control_command("stop_demo", {})
        return True

    def get_status(self) -> Dict[str, Any]:
        latest = None
        if self.data_acquisition is not None and hasattr(self.data_acquisition, "get_latest_data"):
            latest = self.data_acquisition.get_latest_data()
        return {
            "mode": "ros2_phase1",
            "sensor": {
                "connected": latest is not None,
                "simulation": True,
                "latest_timestamp": getattr(latest, "timestamp", None),
            },
            "arm": {"connected": False},
            "gripper": {"connected": False},
        }

    def export_data(self, file_path: str) -> bool:
        try:
            latest = None
            if self.data_acquisition is not None and hasattr(self.data_acquisition, "get_latest_data"):
                latest = self.data_acquisition.get_latest_data()
            payload = {
                "mode": "ros2_phase1",
                "export_time": time.time(),
                "latest_data": latest.to_dict() if latest is not None and hasattr(latest, "to_dict") else None,
            }
            with open(file_path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2)
            return True
        except Exception as exc:
            self.error_occurred.emit("export_error", {"error": str(exc)})
            return False
