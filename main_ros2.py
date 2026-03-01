#!/usr/bin/env python3
"""
Phase 1 ROS2 entrypoint (read-only monitor mode).

This keeps legacy `main.py` untouched and runs the existing PyQt GUI with:
- ROS2 tactile subscription as data source
- disabled control path (stubbed)
"""

import argparse
import logging
import signal
import sys
from pathlib import Path

project_root = Path(__file__).parent
src_root = project_root / "src"
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

class Ros2Phase1App:
    def __init__(
        self,
        config_path: str = None,
        sensor_type: str = "3x3",
        tactile_topic: str = "/tactile/raw",
        health_topic: str = "/system/health",
    ):
        self.logger = logging.getLogger(__name__)
        self.config = self._load_config(config_path, sensor_type)
        self.sensor_type = sensor_type
        self.tactile_topic = tactile_topic
        self.health_topic = health_topic

        self.app = None
        self.main_window = None
        self.data_acquisition_thread = None
        self.control_thread = None
        self.demo_manager = None
        self.is_running = False

        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _load_config(self, config_path: str, sensor_type: str):
        from config.demo_config import DemoConfig
        from config.paxini_gen3_config import PaxiniGen3Config, create_3x3_config

        if sensor_type == "3x3":
            if config_path:
                return PaxiniGen3Config(config_path)
            return create_3x3_config()
        if config_path:
            try:
                return DemoConfig.load(config_path)
            except Exception as exc:
                logging.getLogger(__name__).warning("Failed to load config, fallback to default: %s", exc)
        return DemoConfig()

    def initialize_modules(self) -> bool:
        try:
            from core.ros2_data_acquisition import Ros2DataAcquisitionThread
            from core.ros2_runtime_stubs import Ros2ControlThreadStub, Ros2DemoManagerStub

            self.data_acquisition_thread = Ros2DataAcquisitionThread(
                config=self.config,
                tactile_topic=self.tactile_topic,
                health_topic=self.health_topic,
            )
            self.control_thread = Ros2ControlThreadStub()
            self.demo_manager = Ros2DemoManagerStub(
                config=self.config,
                data_acquisition=self.data_acquisition_thread,
                control_thread=self.control_thread,
            )
            return True
        except Exception as exc:
            self.logger.error("Failed to initialize ROS2 modules: %s", exc)
            return False

    def initialize_gui(self) -> bool:
        try:
            from PyQt5.QtGui import QFont
            from PyQt5.QtWidgets import QApplication
            from gui.main_window import MainWindow

            self.app = QApplication(sys.argv)
            self.app.setApplicationName("触觉夹爪演示系统")
            self.app.setApplicationVersion("2.1.0-phase1-ros2")
            self.app.setFont(QFont("Microsoft YaHei", 10))

            self.main_window = MainWindow(
                demo_manager=self.demo_manager,
                data_acquisition_thread=self.data_acquisition_thread,
                control_thread=self.control_thread,
                config=self.config,
            )
            self.main_window.setWindowTitle("触觉夹爪控制系统 [ROS2 Phase1 Monitor]")
            self.main_window.resize(self.config.ui.window_width, self.config.ui.window_height)
            return True
        except Exception as exc:
            self.logger.error("Failed to initialize GUI: %s", exc)
            return False

    def connect_signals(self) -> None:
        from PyQt5.QtCore import Qt

        self.main_window.control_signal.connect(
            self.demo_manager.handle_control_command,
            Qt.QueuedConnection,
        )
        self.main_window.request_shutdown.connect(self.cleanup, Qt.QueuedConnection)

        self.data_acquisition_thread.error_occurred.connect(
            lambda status, info: self.logger.error("ROS2 data error [%s]: %s", status, info),
            Qt.QueuedConnection,
        )
        self.control_thread.error_occurred.connect(
            lambda status, info: self.logger.error("Control stub error [%s]: %s", status, info),
            Qt.QueuedConnection,
        )

    def start(self) -> int:
        if not self.initialize_modules():
            return 1
        if not self.initialize_gui():
            return 1

        self.connect_signals()

        self.is_running = True
        self.data_acquisition_thread.start()
        self.control_thread.start()

        self.main_window.show()
        self.demo_manager.status_changed.emit(
            "ready",
            {
                "mode": "ros2_phase1",
                "message": "ROS2 phase1 monitor mode started",
                "tactile_topic": self.tactile_topic,
                "health_topic": self.health_topic,
            },
        )

        self.logger.info("ROS2 phase1 monitor mode started")
        code = self.app.exec_()
        self.cleanup()
        return code

    def cleanup(self) -> None:
        if not self.is_running:
            return
        self.is_running = False

        if self.data_acquisition_thread is not None:
            self.data_acquisition_thread.stop_acquisition()
            self.data_acquisition_thread.wait()
        if self.control_thread is not None:
            self.control_thread.stop_control()
            self.control_thread.wait()

    def _handle_signal(self, signum, frame) -> None:  # noqa: ARG002
        self.logger.info("Received signal %s, shutting down ROS2 phase1 app", signum)
        self.cleanup()
        if self.app is not None:
            self.app.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="触觉夹爪演示系统 ROS2 Phase1 Monitor")
    parser.add_argument("--config", type=str, default=None, help="配置文件路径")
    parser.add_argument(
        "--sensor-type",
        type=str,
        default="3x3",
        choices=["default", "3x3"],
        help="传感器类型",
    )
    parser.add_argument("--tactile-topic", type=str, default="/tactile/raw", help="ROS2触觉主题")
    parser.add_argument("--health-topic", type=str, default="/system/health", help="ROS2健康主题")
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="日志级别",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    app = Ros2Phase1App(
        config_path=args.config,
        sensor_type=args.sensor_type,
        tactile_topic=args.tactile_topic,
        health_topic=args.health_topic,
    )
    sys.exit(app.start())


if __name__ == "__main__":
    main()
