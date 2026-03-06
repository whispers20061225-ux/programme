#!/usr/bin/env python3
"""
ROS2 entrypoint for phased migration.

This keeps legacy `main.py` untouched and runs the existing PyQt GUI with:
- ROS2 tactile subscription as data source
- ROS2 control thread (phase 5) or control stub fallback
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
        vision_enabled: bool = True,
        vision_color_topic: str = "/camera/camera/color/image_raw",
        vision_depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw",
        vision_camera_info_topic: str = "/camera/camera/color/camera_info",
        vision_stale_timeout_sec: float = 1.5,
        vision_max_fps: float = 30.0,
        vision_qos_mode: str = "auto",
        control_mode: str = "ros2",
        command_timeout_sec: float = 5.0,
    ):
        self.logger = logging.getLogger(__name__)
        self.config = self._load_config(config_path, sensor_type)
        self.sensor_type = sensor_type
        self.tactile_topic = tactile_topic
        self.health_topic = health_topic
        self.arm_state_topic = arm_state_topic
        self.vision_enabled = vision_enabled
        self.vision_color_topic = vision_color_topic
        self.vision_depth_topic = vision_depth_topic
        self.vision_camera_info_topic = vision_camera_info_topic
        self.vision_stale_timeout_sec = vision_stale_timeout_sec
        self.vision_max_fps = vision_max_fps
        self.vision_qos_mode = str(vision_qos_mode or "auto").strip().lower()
        self.control_mode = control_mode
        self.command_timeout_sec = command_timeout_sec

        self.app = None
        self.main_window = None
        self.data_acquisition_thread = None
        self.control_thread = None
        self.demo_manager = None
        self.is_running = False
        self._error_log_last_ts = {}
        self._error_log_interval_sec = 2.0

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
                vision_enabled=self.vision_enabled,
                color_topic=self.vision_color_topic,
                depth_topic=self.vision_depth_topic,
                camera_info_topic=self.vision_camera_info_topic,
                vision_stale_timeout_sec=self.vision_stale_timeout_sec,
                vision_max_fps=self.vision_max_fps,
                vision_emit_signal=False,
                vision_qos_mode=self.vision_qos_mode,
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
            self.app.setApplicationVersion("2.4.0-phase6-vision")
            self.app.setFont(QFont("Microsoft YaHei", 10))

            self.main_window = MainWindow(
                demo_manager=self.demo_manager,
                data_acquisition_thread=self.data_acquisition_thread,
                control_thread=self.control_thread,
                config=self.config,
            )
            title_suffix = "Phase5 Task" if self.control_mode != "stub" else "Monitor Stub"
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
            lambda status, info: self._log_error_throttled("data", status, info),
            Qt.QueuedConnection,
        )
        self.control_thread.error_occurred.connect(
            lambda status, info: self._log_error_throttled("control", status, info),
            Qt.QueuedConnection,
        )

    def _log_error_throttled(self, source: str, status: str, info: dict) -> None:
        key = f"{source}:{status}"
        from time import monotonic
        now = monotonic()
        last = float(self._error_log_last_ts.get(key, 0.0))
        if now - last < self._error_log_interval_sec:
            return
        self._error_log_last_ts[key] = now
        self.logger.error("ROS2 %s error [%s]: %s", source, status, info)

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
                "mode": "ros2_phase5" if self.control_mode != "stub" else "ros2_stub",
                "message": "ROS2 GUI mode started",
                "tactile_topic": self.tactile_topic,
                "health_topic": self.health_topic,
                "arm_state_topic": self.arm_state_topic,
                "vision_enabled": self.vision_enabled,
                "vision_color_topic": self.vision_color_topic if self.vision_enabled else None,
                "vision_depth_topic": self.vision_depth_topic if self.vision_enabled else None,
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
        "--vision-enabled",
        type=lambda x: str(x).lower() in ("1", "true", "yes", "y", "on"),
        default=True,
        help="Enable ROS2 camera subscriptions for Vision UI",
    )
    parser.add_argument(
        "--vision-color-topic",
        type=str,
        default="/camera/camera/color/image_raw",
        help="ROS2 color image topic",
    )
    parser.add_argument(
        "--vision-depth-topic",
        type=str,
        default="/camera/camera/aligned_depth_to_color/image_raw",
        help="ROS2 depth image topic",
    )
    parser.add_argument(
        "--vision-camera-info-topic",
        type=str,
        default="/camera/camera/color/camera_info",
        help="ROS2 camera info topic",
    )
    parser.add_argument(
        "--vision-stale-timeout-sec",
        type=float,
        default=1.5,
        help="Vision stream stale timeout in seconds",
    )
    parser.add_argument(
        "--vision-max-fps",
        type=float,
        default=30.0,
        help="Max FPS pushed from ROS2 image stream to UI",
    )
    parser.add_argument(
        "--vision-qos-mode",
        type=str,
        default="auto",
        choices=["auto", "best_effort", "reliable"],
        help="ROS2 QoS profile for vision subscriptions",
    )
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
        vision_enabled=args.vision_enabled,
        vision_color_topic=args.vision_color_topic,
        vision_depth_topic=args.vision_depth_topic,
        vision_camera_info_topic=args.vision_camera_info_topic,
        vision_stale_timeout_sec=args.vision_stale_timeout_sec,
        vision_max_fps=args.vision_max_fps,
        vision_qos_mode=args.vision_qos_mode,
        control_mode=args.control_mode,
        command_timeout_sec=args.command_timeout_sec,
    )
    sys.exit(app.start())


if __name__ == "__main__":
    main()
