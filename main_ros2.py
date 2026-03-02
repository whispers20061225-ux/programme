#!/usr/bin/env python3
"""
ROS2 entrypoint for phased migration.

This keeps legacy `main.py` untouched and runs the existing PyQt GUI with:
- ROS2 tactile subscription as data source
- ROS2 control thread (phase 4) or control stub fallback
"""

import argparse
import logging
import signal
import sys
import warnings
from pathlib import Path

project_root = Path(__file__).parent
src_root = project_root / "src"
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))


def _configure_matplotlib_runtime() -> None:
    """Reduce non-actionable matplotlib font noise in GUI runtime."""
    warnings.filterwarnings(
        "ignore",
        message=r".*Glyph .* missing from font\(s\).*",
        category=UserWarning,
    )
    warnings.filterwarnings(
        "ignore",
        message=r".*findfont:.*",
        category=UserWarning,
    )
    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

    try:
        from matplotlib import font_manager, rcParams

        preferred_names = [
            "Noto Sans CJK SC",
            "Noto Sans CJK TC",
            "Noto Sans CJK JP",
            "WenQuanYi Micro Hei",
            "WenQuanYi Zen Hei",
            "SimHei",
            "Microsoft YaHei",
            "PingFang SC",
            "STHeiti",
            "Arial Unicode MS",
        ]
        candidate_font_paths = [
            Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
            Path("/usr/share/fonts/opentype/noto/NotoSansCJKSC-Regular.otf"),
            Path("/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"),
            Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
            Path("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
        ]

        loaded_names = []
        for font_path in candidate_font_paths:
            if not font_path.exists():
                continue
            try:
                font_manager.fontManager.addfont(str(font_path))
                font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
                if font_name and font_name not in loaded_names:
                    loaded_names.append(font_name)
            except Exception:
                continue

        available_names = {font.name for font in font_manager.fontManager.ttflist}
        selected_fonts = loaded_names + [name for name in preferred_names if name in available_names]
        if not selected_fonts:
            selected_fonts = ["DejaVu Sans"]
            logging.getLogger(__name__).warning(
                "No CJK font detected for Matplotlib; install `fonts-noto-cjk` for Chinese labels."
            )

        # Keep order while de-duplicating.
        selected_fonts = list(dict.fromkeys(selected_fonts))
        rcParams["font.family"] = selected_fonts
        rcParams["font.sans-serif"] = selected_fonts + ["DejaVu Sans"]
        rcParams["axes.unicode_minus"] = False
        logging.getLogger(__name__).info("Matplotlib primary font: %s", selected_fonts[0])
    except Exception as exc:
        # Matplotlib may be unavailable in some stripped runtime environments.
        logging.getLogger(__name__).warning("Failed to configure Matplotlib fonts: %s", exc)


class Ros2PhaseApp:
    def __init__(
        self,
        config_path: str = None,
        sensor_type: str = "3x3",
        tactile_topic: str = "/tactile/raw",
        health_topic: str = "/system/health",
        arm_state_topic: str = "/arm/state",
        control_mode: str = "ros2",
        command_timeout_sec: float = 5.0,
    ):
        self.logger = logging.getLogger(__name__)
        self.config = self._load_config(config_path, sensor_type)
        self.sensor_type = sensor_type
        self.tactile_topic = tactile_topic
        self.health_topic = health_topic
        self.arm_state_topic = arm_state_topic
        self.control_mode = control_mode
        self.command_timeout_sec = command_timeout_sec

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
            except Exception as exc:  # noqa: BLE001
                logging.getLogger(__name__).warning("Failed to load config, fallback to default: %s", exc)
        return DemoConfig()

    def initialize_modules(self) -> bool:
        try:
            from core.ros2_data_acquisition import Ros2DataAcquisitionThread
            from core.ros2_runtime_stubs import (
                Ros2ControlThread,
                Ros2ControlThreadStub,
                Ros2DemoManagerStub,
            )

            self.data_acquisition_thread = Ros2DataAcquisitionThread(
                config=self.config,
                tactile_topic=self.tactile_topic,
                health_topic=self.health_topic,
            )

            if self.control_mode == "stub":
                self.control_thread = Ros2ControlThreadStub()
            else:
                self.control_thread = Ros2ControlThread(
                    arm_state_topic=self.arm_state_topic,
                    health_topic=self.health_topic,
                    command_timeout_sec=self.command_timeout_sec,
                )

            self.demo_manager = Ros2DemoManagerStub(
                config=self.config,
                data_acquisition=self.data_acquisition_thread,
                control_thread=self.control_thread,
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.logger.error("Failed to initialize ROS2 modules: %s", exc)
            return False

    def initialize_gui(self) -> bool:
        try:
            from PyQt5.QtGui import QFont
            from PyQt5.QtWidgets import QApplication
            from gui.main_window import MainWindow

            self.app = QApplication(sys.argv)
            self.app.setApplicationName("Tactile Gripper Demo")
            self.app.setApplicationVersion("2.2.0-phase4-ros2")
            self.app.setFont(QFont("Microsoft YaHei", 10))

            self.main_window = MainWindow(
                demo_manager=self.demo_manager,
                data_acquisition_thread=self.data_acquisition_thread,
                control_thread=self.control_thread,
                config=self.config,
            )
            title_suffix = "Phase4 Control" if self.control_mode != "stub" else "Monitor Stub"
            self.main_window.setWindowTitle(f"Tactile Gripper Control [ROS2 {title_suffix}]")
            self.main_window.resize(self.config.ui.window_width, self.config.ui.window_height)
            return True
        except Exception as exc:  # noqa: BLE001
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
            lambda status, info: self.logger.error("ROS2 control error [%s]: %s", status, info),
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
                "mode": "ros2_phase4" if self.control_mode != "stub" else "ros2_stub",
                "message": "ROS2 GUI mode started",
                "tactile_topic": self.tactile_topic,
                "health_topic": self.health_topic,
                "arm_state_topic": self.arm_state_topic,
                "control_mode": self.control_mode,
            },
        )

        self.logger.info("ROS2 app started: control_mode=%s", self.control_mode)
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
        self.logger.info("Received signal %s, shutting down ROS2 app", signum)
        self.cleanup()
        if self.app is not None:
            self.app.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description="Tactile Gripper ROS2 GUI Entrypoint")
    parser.add_argument("--config", type=str, default=None, help="Path to config file")
    parser.add_argument(
        "--sensor-type",
        type=str,
        default="3x3",
        choices=["default", "3x3"],
        help="Sensor profile",
    )
    parser.add_argument("--tactile-topic", type=str, default="/tactile/raw", help="ROS2 tactile topic")
    parser.add_argument("--health-topic", type=str, default="/system/health", help="ROS2 health topic")
    parser.add_argument("--arm-state-topic", type=str, default="/arm/state", help="ROS2 arm state topic")
    parser.add_argument(
        "--control-mode",
        type=str,
        default="ros2",
        choices=["ros2", "stub"],
        help="Control backend mode",
    )
    parser.add_argument(
        "--command-timeout-sec",
        type=float,
        default=5.0,
        help="ROS2 control service timeout in seconds",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Log level",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
    _configure_matplotlib_runtime()

    app = Ros2PhaseApp(
        config_path=args.config,
        sensor_type=args.sensor_type,
        tactile_topic=args.tactile_topic,
        health_topic=args.health_topic,
        arm_state_topic=args.arm_state_topic,
        control_mode=args.control_mode,
        command_timeout_sec=args.command_timeout_sec,
    )
    sys.exit(app.start())


if __name__ == "__main__":
    main()
