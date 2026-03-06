"""
触觉夹爪演示系统 - 主窗口
主窗口负责整合所有UI组件和管理用户交互。
支持三维力数据和矢量图可视化
"""

import sys
import os
import logging
import time
import numpy as np
import math
from threading import Lock
from datetime import datetime
from types import SimpleNamespace
from typing import Dict, Any, Optional, List, Tuple
import cv2

# 添加项目根目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)  # 项目根目录
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QTabWidget, QStatusBar, QMenuBar, QMenu,
    QAction, QToolBar, QMessageBox, QLabel, QPushButton,
    QGroupBox, QGridLayout, QSpinBox, QDoubleSpinBox,
    QComboBox, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QSize
from PyQt5.QtGui import QIcon, QFont, QColor, QPalette

# 使用绝对导入而不是相对导入
from config import (
    DemoConfig,
    CameraConfig,
    SimulationConfig,
    LearmArmConfig,
    create_default_camera_config,
    create_default_sim_config,
    create_default_learm_config,
)
from perception.vision.camera_capture import CameraCapture
from perception.vision.object_detector import ObjectDetector
from perception.vision.material_recognizer import MaterialRecognizer
from perception.vision.pose_estimator import PoseEstimator
from perception.vision.depth_processor import DepthProcessor
from perception.vision.pointcloud_fusion import MultiViewPointCloudFusion
from core.demo_manager import DemoManager
from core.data_acquisition import DataAcquisitionThread
from core.control_thread import ControlThread
from core.latest_task_worker import LatestTaskWorker

# GUI模块内部导入（使用绝对导入，方便独立运行）
from gui.control_panel import ControlPanel
from gui.plot_widget import (
    TactilePlotWidget,
    ForcePlotWidget,
    TactileSurfaceWidget,  # 新增：触觉曲面显示
)
from gui.modern_vision_viewer import VisionViewer, colorize_depth_for_display
from gui.simulation_viewer import SimulationViewer
from gui.arm_status_panel import ArmStatusPanel
from gui.dialogs import (
    ConfigDialog,
    CalibrationDialog,
    DemoSelectionDialog,
    LogViewerDialog,
    AboutDialog
)

# 添加缺失的导入
from PyQt5.QtWidgets import QFileDialog, QFormLayout, QProgressBar
import json
import yaml
import numpy as np
import matplotlib.pyplot as plt


class MainWindow(QMainWindow):
    """主窗口类 - 支持三维力数据和矢量图可视化"""
    
    # 自定义信号
    control_signal = pyqtSignal(str, dict)  # (command, parameters)
    request_shutdown = pyqtSignal()
    
    # 三维力数据信号
    vector_data_updated = pyqtSignal(object)  # 三维力向量数据
    contact_data_updated = pyqtSignal(object)  # 接触数据
    
    def __init__(self, demo_manager: DemoManager,
                 data_acquisition_thread: DataAcquisitionThread,
                 control_thread: ControlThread,
                 config: DemoConfig):
        """
        初始化主窗口
        
        Args:
            demo_manager: 演示管理器
            data_acquisition_thread: 数据采集线程
            control_thread: 控制线程
            config: 系统配置
        """
        super().__init__()
        
        # 保存引用
        self.demo_manager = demo_manager
        self.data_acquisition_thread = data_acquisition_thread
        self.control_thread = control_thread
        self.config = config
        
        # 设置窗口属性
        self.setWindowTitle("触觉夹爪控制系统 - 三维力与矢量可视化")
        self.setGeometry(100, 100, 
                        config.ui.window_width, 
                        config.ui.window_height)
        
        # 三维力数据状态
        self.force_vectors = None  # 当前三维力向量数据
        self.contact_map = None  # 接触地图
        self.vector_field_data = None  # 矢量场数据
        
        # 性能优化参数
        self.last_gui_update_time = 0
        self.gui_update_interval = 0.033  # 约30Hz (33ms)
        self.vector_update_interval = 0.1  # 矢量图更新间隔 (10Hz)
        self.frame_count = 0
        self.start_time = time.time()
        self.data_rate = 0
        self._last_packet_label_ts = 0.0
        self._last_control_panel_data_ts = 0.0
        
        # 数据缓冲区
        self.sensor_data_buffer = None
        self.force_data_buffer = None
        
        # 可视化模式
        self.visualization_mode = "heatmap"  # heatmap, vector_field, 3d_view
        self.vector_scale = 1.0  # 矢量缩放因子
        self.show_force_labels = True  # 显示力标签
        self.vector_tab = None
        self.tactile_tab = None
        self.force_tab = None
        self.arm_function_tab = None
        self.vision_tab = None
        self.simulation_tab = None
        self.arm_tab = None
        self.vision_viewer = None
        self.simulation_viewer = None
        self.arm_status_panel = None
        self.arm_function_mode_combo = None
        self.arm_function_execute_btn = None
        self.arm_function_approach_spin = None
        self.arm_function_speed_spin = None
        self.arm_function_gripper_check = None
        self.arm_function_gripper_spin = None
        self.camera_capture = None
        self.camera_timer = None
        self.object_detector: Optional[ObjectDetector] = None
        self.material_recognizer: Optional[MaterialRecognizer] = None
        self.pose_estimator: Optional[PoseEstimator] = None
        self.depth_processor: Optional[DepthProcessor] = None
        self.pointcloud_fusion: Optional[MultiViewPointCloudFusion] = None
        self.pointcloud_fusion_config: Optional[Dict[str, Any]] = None
        self.pointcloud_fusion_force_enabled = False
        self.pointcloud_auto_render = False
        self.last_detection_bboxes: List[List[float]] = []
        self.last_detection_entries: List[Dict[str, Any]] = []
        self.last_detection_time = 0.0
        self.auto_detect_enabled = False
        self.auto_detect_interval = 1.2
        self.last_auto_detect_time = 0.0
        self.detect_in_progress = False
        self._analysis_backoff_until_ts = 0.0
        self._analysis_stall_backoff_sec = 4.0
        self.last_detection_error_time = 0.0
        self.detection_error_cooldown = 3.0  # ???????
        self._material_recognizer_cfg: Optional[Dict[str, Any]] = None
        self._pose_estimator_cfg: Optional[Dict[str, Any]] = None
        self.auto_grasp_max_depth_mm = 1200.0
        self.camera_device_info_text = None
        self._ros2_vision_connected = False
        self._ros2_vision_streaming = False
        self._ros2_vision_simulation = False
        self._ros2_vision_fps = 0.0
        self._ros2_vision_rx_fps = 0.0
        self._ros2_vision_render_fps = 0.0
        self._ros2_vision_dropped_frames = 0
        self._ros2_vision_last_frame_age_ms = None
        self._ros2_vision_resolution = "N/A"
        self._ros2_vision_latest_frame = None
        self._ros2_vision_device_info = "ROS2 camera stream"
        self._ros2_vision_pending_frame = None
        self._ros2_vision_pending_lock = Lock()
        self._ros2_vision_frame_overwrite_count = 0
        self._ros2_vision_queue_overwrite_total = 0
        self._ros2_vision_last_self_check_ts = 0.0
        self._ros2_vision_last_diag_ts = 0.0
        self._ros2_vision_last_render_ts = 0.0
        self._ros2_vision_prev_render_ts = 0.0
        self._ros2_vision_render_target_interval_sec = 1.0 / 30.0
        self._ros2_vision_stall_count = 0
        self._ros2_vision_stall_active = False
        self._ros2_vision_stall_reason = ""
        self._ros2_vision_stall_started_at = 0.0
        self._ros2_vision_last_upstream_frame_wall_ts = 0.0
        self._vision_depth_profile = None
        self._analysis_worker = LatestTaskWorker("vision-analysis", self._run_analysis_task)
        self._depth_visual_worker = LatestTaskWorker("depth-visual", self._run_depth_visual_task)
        self._pointcloud_worker = LatestTaskWorker("pointcloud", self._run_pointcloud_task)
        self._analysis_result_overwrite_total = 0
        self._depth_visual_last_task_ts = 0.0
        self._depth_visual_result_overwrite_total = 0
        self._pointcloud_result_overwrite_total = 0
        self._pointcloud_schedule_interval_sec = 0.5
        self._pointcloud_last_schedule_ts = 0.0
        self._manual_detection_requested = False
        self._pointcloud_refresh_requested = False
        self._tactile_plot_dirty = False
        self._force_plot_dirty = False
        self._last_arm_state = None  # ??????????UI??
        self._last_missing_log_time = 0.0  # ????????????

        self.init_ui()
        self.init_menu()
        self.init_toolbar()
        self.init_status_bar()
        
        # 设置定时器
        self.init_timers()
        
        # 连接信号
        self.connect_signals()
        
        # ????
        self.logger = logging.getLogger(__name__)
        self._analysis_worker.start()
        self._depth_visual_worker.start()
        self._pointcloud_worker.start()

        self.logger.info("???????????")

    def init_ui(self):
        """初始化用户界面 - 支持三维力可视化"""
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        # 创建分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 创建左侧控制面板
        self.control_panel = ControlPanel(self.demo_manager, self.config)
        splitter.addWidget(self.control_panel)
        
        # 创建右侧显示区域
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(5)
        
        # 创建可视化控制栏
        self.init_visualization_toolbar(right_layout)
        
        # 创建标签页
        self.tab_widget = QTabWidget()
        
        # 触觉数据显示标签页
        self.tactile_tab = QWidget()
        tactile_layout = QVBoxLayout(self.tactile_tab)
        self.tactile_plot = TactilePlotWidget(self.config)
        tactile_layout.addWidget(self.tactile_plot)
        self.tab_widget.addTab(self.tactile_tab, "触觉数据")
        
        # 触觉映射3D视图标签页（替换原矢量场）
        self.vector_tab = QWidget()
        vector_layout = QVBoxLayout(self.vector_tab)
        self.vector_plot = TactileSurfaceWidget(self.config)  # 使用触觉曲面显示
        vector_layout.addWidget(self.vector_plot)
        self.tab_widget.addTab(self.vector_tab, "触觉映射3D")


        # 机械臂功能标签页（基础功能，后续可扩展）
        self.arm_function_tab = QWidget()
        arm_func_layout = QVBoxLayout(self.arm_function_tab)
        arm_func_layout.setContentsMargins(10, 10, 10, 10)
        arm_func_layout.setSpacing(8)

        arm_func_group = QGroupBox("机械臂功能")
        arm_func_grid = QGridLayout(arm_func_group)
        arm_func_grid.setHorizontalSpacing(10)
        arm_func_grid.setVerticalSpacing(6)

        arm_mode_label = QLabel("模式：")
        self.arm_function_mode_combo = QComboBox()
        self.arm_function_mode_combo.addItems(["自动夹取"])

        approach_label = QLabel("预抓取高度(mm)：")
        self.arm_function_approach_spin = QDoubleSpinBox()
        self.arm_function_approach_spin.setRange(0.0, 300.0)
        self.arm_function_approach_spin.setSingleStep(5.0)
        self.arm_function_approach_spin.setValue(50.0)

        speed_label = QLabel("运动速度(0-1)：")
        self.arm_function_speed_spin = QDoubleSpinBox()
        self.arm_function_speed_spin.setRange(0.05, 1.0)
        self.arm_function_speed_spin.setSingleStep(0.05)
        self.arm_function_speed_spin.setValue(0.3)

        self.arm_function_gripper_check = QCheckBox("执行夹爪闭合")
        self.arm_function_gripper_check.setChecked(True)
        gripper_label = QLabel("闭合位置(度)：")
        self.arm_function_gripper_spin = QDoubleSpinBox()
        self.arm_function_gripper_spin.setRange(0.0, 180.0)
        self.arm_function_gripper_spin.setSingleStep(5.0)
        self.arm_function_gripper_spin.setValue(40.0)

        self.arm_function_execute_btn = QPushButton("执行")
        self.arm_function_execute_btn.clicked.connect(self._handle_arm_function_execute)

        arm_func_grid.addWidget(arm_mode_label, 0, 0)
        arm_func_grid.addWidget(self.arm_function_mode_combo, 0, 1, 1, 3)
        arm_func_grid.addWidget(approach_label, 1, 0)
        arm_func_grid.addWidget(self.arm_function_approach_spin, 1, 1)
        arm_func_grid.addWidget(speed_label, 1, 2)
        arm_func_grid.addWidget(self.arm_function_speed_spin, 1, 3)
        arm_func_grid.addWidget(self.arm_function_gripper_check, 2, 0, 1, 2)
        arm_func_grid.addWidget(gripper_label, 2, 2)
        arm_func_grid.addWidget(self.arm_function_gripper_spin, 2, 3)
        arm_func_grid.addWidget(self.arm_function_execute_btn, 3, 0, 1, 4)

        arm_func_layout.addWidget(arm_func_group)
        self.tab_widget.addTab(self.arm_function_tab, "机械臂功能")
        
        # 力数据标签页
        self.force_tab = QWidget()
        force_layout = QVBoxLayout(self.force_tab)
        self.force_plot = ForcePlotWidget(self.config)
        force_layout.addWidget(self.force_plot)
        self.tab_widget.addTab(self.force_tab, "力数据")
        
        # 机械臂状态标签页
        arm_ui_cfg = getattr(self.config.ui, "arm_ui", {}) if hasattr(self.config, "ui") else {}
        arm_config = self._get_arm_config()
        if arm_config is not None and arm_ui_cfg.get("show_arm_status", True):
            self.arm_tab = QWidget()
            arm_layout = QVBoxLayout(self.arm_tab)
            self.arm_status_panel = ArmStatusPanel(arm_config)
            arm_layout.addWidget(self.arm_status_panel)
            self.tab_widget.addTab(self.arm_tab, "机械臂状态")

        # 视觉显示标签页
        vision_ui_cfg = getattr(self.config.ui, "vision_ui", {}) if hasattr(self.config, "ui") else {}
        camera_config = self._get_camera_config()
        if camera_config is not None and vision_ui_cfg.get("show_camera_view", True):
            self.vision_tab = QWidget()
            vision_layout = QVBoxLayout(self.vision_tab)
            self.vision_viewer = VisionViewer(camera_config)
            # 绑定相机连接请求
            self.vision_viewer.connect_request.connect(self._handle_connect_camera)
            self.vision_viewer.disconnect_request.connect(self._handle_disconnect_camera)
            self.vision_viewer.capture_request.connect(self._handle_capture_snapshot)
            self.vision_viewer.detection_request.connect(self._handle_detection_request)
            self.vision_viewer.save_image_request.connect(self._handle_save_image_request)
            self.vision_viewer.auto_detect_toggled.connect(self._handle_auto_detect_toggle)
            self.vision_viewer.self_check_request.connect(self._handle_self_check_request)
            self.vision_viewer.pointcloud_request.connect(self._handle_pointcloud_request)
            self.vision_viewer.pointcloud_save_request.connect(self._handle_pointcloud_save_request)
            # 初始状态与UI同步（默认勾选实时检测）
            self._handle_auto_detect_toggle(False)
            vision_layout.addWidget(self.vision_viewer)
            self.tab_widget.addTab(self.vision_tab, "视觉显示")

        # 仿真显示标签页（GUI 模式时隐藏，避免与 PyBullet 自带窗口冲突）
        sim_ui_cfg = getattr(self.config.ui, "simulation_ui", {}) if hasattr(self.config, "ui") else {}
        simulation_config = self._get_simulation_config()
        sim_mode = None
        if simulation_config is not None:
            if isinstance(simulation_config, dict):
                sim_mode = simulation_config.get("ENGINE", {}).get("mode")
            else:
                sim_mode = getattr(simulation_config, "ENGINE", {}).get("mode")

        if simulation_config is not None and sim_ui_cfg.get("show_simulation_view", False) and sim_mode != "gui":
            self.simulation_tab = QWidget()
            simulation_layout = QVBoxLayout(self.simulation_tab)
            self.simulation_viewer = SimulationViewer(simulation_config)
            simulation_layout.addWidget(self.simulation_viewer)
            self.tab_widget.addTab(self.simulation_tab, "仿真显示")
        
        # 添加标签页到右侧布局
        right_layout.addWidget(self.tab_widget)
        
        # 添加右侧部件到分割器
        splitter.addWidget(right_widget)
        
        # 设置分割器比例
        splitter.setSizes([self.config.ui.control_panel_width, 
                          self.config.ui.window_width - self.config.ui.control_panel_width])
        
        # 添加分割器到主布局
        main_layout.addWidget(splitter)
        
        # 设置控制面板信号
        self.control_panel.control_signal.connect(self.control_signal)
        self.control_panel.request_shutdown.connect(self.request_shutdown)
        
        # 连接三维力数据信号
        self.vector_data_updated.connect(self.vector_plot.update_data)
        self.contact_data_updated.connect(self.vector_plot.update_contact_data)

    def _is_simulation_mode(self) -> bool:
        """判断是否处于模拟模式（用于降低UI负载）"""
        try:
            if hasattr(self.demo_manager, "hardware_interface"):
                return getattr(self.demo_manager.hardware_interface, "simulation_mode", False)
        except Exception:
            return False
        return False
    
    def _get_camera_config(self):
        """获取视觉配置，缺失时使用默认配置"""
        if hasattr(self.config, "camera") and self.config.camera is not None:
            if isinstance(self.config.camera, CameraConfig):
                return self.config.camera
            if isinstance(self.config.camera, dict):
                # 将配置字典转换为CameraConfig，便于统一接口读取
                try:
                    return CameraConfig.from_dict(self.config.camera)
                except Exception:
                    pass
        try:
            return create_default_camera_config()
        except Exception:
            try:
                return CameraConfig()
            except Exception:
                return None

    def _is_camera_simulation(self) -> bool:
        """判断视觉设备是否处于仿真模式"""
        camera_config = self._get_camera_config()
        if camera_config is None:
            return False
        camera_type = None
        if isinstance(camera_config, dict):
            primary = camera_config.get("HARDWARE", {}).get("primary_camera", {})
            camera_type = primary.get("type")
        else:
            camera_type = getattr(camera_config, "camera_type", None)
        return str(camera_type).lower() == "simulation"

    def _is_ros2_vision_mode(self) -> bool:
        return bool(
            self.data_acquisition_thread is not None
            and hasattr(self.data_acquisition_thread, "request_vision_connect")
            and hasattr(self.data_acquisition_thread, "vision_frame")
            and hasattr(self.data_acquisition_thread, "vision_status")
        )

    def _handle_connect_camera(self):
        """处理视觉视图发起的连接请求"""
        if self._is_ros2_vision_mode():
            self._handle_disconnect_camera()
            try:
                self.data_acquisition_thread.request_vision_connect()
                if self.vision_viewer:
                    self.vision_viewer.update_camera_status(
                        connected=False,
                        streaming=False,
                        resolution="N/A",
                        fps=0,
                    )
                self.control_panel.update_device_status(
                    vision={"connected": False, "simulation": self._ros2_vision_simulation}
                )
            except Exception as e:
                if self.vision_viewer:
                    self.vision_viewer.show_connection_error(str(e))
                self.control_panel.update_device_status(
                    vision={"connected": False, "simulation": False}
                )
            return

        camera_config = self._get_camera_config()
        if camera_config is None:
            self.vision_viewer.show_connection_error("相机配置无效")
            self.control_panel.update_device_status(
                vision={"connected": False, "simulation": False}
            )
            return
        # 停止旧实例
        self._handle_disconnect_camera()
        try:
            self.camera_capture = CameraCapture(camera_config)
            self.camera_capture.start_capture()
            # 试取一帧确认
            frame = self.camera_capture.get_frame_with_timeout(1.0)
            if frame is None:
                raise RuntimeError("未获取到相机帧")
            res_str = f"{camera_config.width}x{camera_config.height}"
            self.vision_viewer.update_camera_status(
                connected=True, streaming=True, resolution=res_str, fps=camera_config.fps
            )
            # 连接后立即刷新序列号/深度状态，避免一直显示N/A
            self._refresh_self_check_status(frame)
            # 立即推送首帧并启动定时拉流
            self._push_camera_frame_to_viewer(frame)
            self._start_camera_timer()
            self.control_panel.update_device_status(
                vision={"connected": True, "simulation": self._is_camera_simulation()}
            )
        except Exception as e:
            if self.vision_viewer:
                self.vision_viewer.show_connection_error(str(e))
            self.camera_capture = None
            self.control_panel.update_device_status(
                vision={"connected": False, "simulation": False}
            )

    def _handle_disconnect_camera(self):
        """断开相机捕获"""
        self.pointcloud_auto_render = False
        self.pointcloud_fusion_force_enabled = False
        if self._is_ros2_vision_mode():
            try:
                self.data_acquisition_thread.request_vision_disconnect()
            except Exception:
                pass
            self._ros2_vision_connected = False
            self._ros2_vision_streaming = False
            self._ros2_vision_latest_frame = None
            self._ros2_vision_fps = 0.0
            self._ros2_vision_rx_fps = 0.0
            self._ros2_vision_render_fps = 0.0
            self._ros2_vision_dropped_frames = 0
            self._ros2_vision_last_frame_age_ms = None
            self._ros2_vision_queue_overwrite_total = 0
            self._pointcloud_refresh_requested = False
            self._depth_visual_last_task_ts = 0.0
            self._depth_visual_result_overwrite_total = 0
            self._manual_detection_requested = False
            self._vision_depth_profile = None
            with self._ros2_vision_pending_lock:
                self._ros2_vision_pending_frame = None
                self._ros2_vision_frame_overwrite_count = 0
            self._stop_camera_timer()
            if self.vision_viewer:
                self.vision_viewer.update_camera_status(
                    connected=False, streaming=False, resolution="N/A", fps=0
                )
            self.control_panel.update_device_status(
                vision={"connected": False, "simulation": self._ros2_vision_simulation}
            )
            return

        if self.camera_capture:
            try:
                self.camera_capture.stop_capture()
            except Exception:
                pass
            self.camera_capture = None
        self._stop_camera_timer()
        if self.vision_viewer:
            self.vision_viewer.update_camera_status(
                connected=False, streaming=False, resolution="N/A", fps=0
            )
        self.control_panel.update_device_status(
            vision={"connected": False, "simulation": False}
        )

    def _start_camera_timer(self):
        """启动定时拉流，将最新帧推送到视觉窗口"""
        if self._is_ros2_vision_mode():
            return
        if self.camera_timer is None:
            self.camera_timer = QTimer()
            self.camera_timer.timeout.connect(self._update_camera_view)
        if not self.camera_timer.isActive():
            # 约 15-20Hz，避免阻塞 UI
            self.camera_timer.start(50)

    def _stop_camera_timer(self):
        """停止相机更新定时器"""
        if self.camera_timer and self.camera_timer.isActive():
            self.camera_timer.stop()

    def _get_latest_camera_frame(self, timeout: float = 1.0) -> Optional[object]:
        """获取最新相机帧，失败时抛出异常"""
        if self._is_ros2_vision_mode():
            frame = self._ros2_vision_latest_frame
            if frame is None:
                raise RuntimeError("尚未接收到 ROS2 相机数据")
            return frame
        if not self.camera_capture:
            raise RuntimeError("请先连接相机")
        frame = self.camera_capture.get_frame_with_timeout(timeout)
        if frame is None:
            raise RuntimeError("未能获取相机帧")
        return frame

    def _show_camera_error(self, message: str):
        """统一的相机错误提示"""
        try:
            if self.vision_viewer:
                self.vision_viewer.show_connection_error(message)
            else:
                QMessageBox.warning(self, "相机", message)
        except Exception:
            pass

    def _push_camera_frame_to_viewer(self, frame):
        """?????????????"""
        if not self.vision_viewer or frame is None:
            return

        image = getattr(frame, "color_image", None)
        depth_image = getattr(frame, "depth_image", None)
        if image is not None:
            self.vision_viewer.update_image(image, "rgb")
        if depth_image is not None:
            if self._is_depth_view_active():
                self._schedule_depth_visual_for_frame(frame)
        else:
            self._depth_visual_last_task_ts = 0.0
            self.vision_viewer.update_image(None, "depth")

        resolution = self._ros2_vision_resolution
        if image is not None and hasattr(image, "shape") and len(image.shape) >= 2:
            h, w = image.shape[:2]
            resolution = f"{w}x{h}"
        self.vision_viewer.update_camera_status(
            connected=self._ros2_vision_connected or bool(image is not None),
            streaming=self._ros2_vision_streaming or bool(image is not None),
            resolution=resolution,
            fps=self._ros2_vision_render_fps,
            rx_fps=self._ros2_vision_rx_fps if self._ros2_vision_rx_fps > 0 else self._ros2_vision_fps,
            dropped_frames=self._ros2_vision_dropped_frames + self._ros2_vision_queue_overwrite_total + self._depth_visual_result_overwrite_total,
            last_frame_age_ms=self._ros2_vision_last_frame_age_ms,
            stall_count=self._ros2_vision_stall_count,
        )

    def _update_camera_view(self):
        """????????????UI"""
        if self._is_ros2_vision_mode():
            return
        if not self.camera_capture or not self.camera_capture.is_capturing:
            return
        frame = self.camera_capture.get_latest_frame()
        if frame is None:
            return
        now = time.time()
        self._ros2_vision_connected = True
        self._ros2_vision_streaming = True
        self._render_vision_frame(frame, now)
        self._schedule_analysis_for_frame(frame, reason="auto", show_error=False)
        self._schedule_pointcloud_for_frame(frame, force=False)

    @pyqtSlot(object)
    def _handle_ros2_vision_frame(self, frame):
        if frame is None:
            return
        self._queue_ros2_vision_frame(frame)

    def _queue_ros2_vision_frame(self, frame) -> None:
        if frame is None:
            return
        with self._ros2_vision_pending_lock:
            if self._ros2_vision_pending_frame is not None:
                self._ros2_vision_frame_overwrite_count += 1
                self._ros2_vision_queue_overwrite_total += 1
            self._ros2_vision_pending_frame = frame

    def _pull_ros2_vision_frame(self):
        if self.data_acquisition_thread is None:
            return None
        if bool(getattr(self.data_acquisition_thread, "vision_emit_signal", False)):
            return None
        if not hasattr(self.data_acquisition_thread, "consume_latest_vision_frame"):
            return None
        try:
            return self.data_acquisition_thread.consume_latest_vision_frame()
        except Exception:
            return None

    def _render_vision_frame(self, frame, now: float) -> None:
        self._ros2_vision_latest_frame = frame
        self._ros2_vision_last_render_ts = now
        frame_ts = float(getattr(frame, "timestamp", now) or now)
        self._ros2_vision_last_frame_age_ms = max(0.0, (now - frame_ts) * 1000.0)
        image = getattr(frame, "color_image", None)
        if image is not None and hasattr(image, "shape") and len(image.shape) >= 2:
            height, width = image.shape[:2]
            self._ros2_vision_resolution = f"{width}x{height}"
        if self._ros2_vision_prev_render_ts > 0.0:
            elapsed = now - self._ros2_vision_prev_render_ts
            if elapsed > 0.0:
                instant_fps = 1.0 / elapsed
                if self._ros2_vision_render_fps <= 0.0:
                    self._ros2_vision_render_fps = instant_fps
                else:
                    self._ros2_vision_render_fps = self._ros2_vision_render_fps * 0.7 + instant_fps * 0.3
        self._ros2_vision_prev_render_ts = now
        if not self._is_ros2_vision_mode():
            self._ros2_vision_fps = self._ros2_vision_render_fps
            self._ros2_vision_rx_fps = self._ros2_vision_render_fps
        self._push_camera_frame_to_viewer(frame)
        self._update_vision_stall_state(now=now, frame_rendered=True)
        self._update_vision_status_widgets()

    def _schedule_depth_visual_for_frame(self, frame) -> None:
        if frame is None or getattr(frame, "depth_image", None) is None:
            return
        frame_timestamp = float(getattr(frame, "timestamp", 0.0) or 0.0)
        if frame_timestamp <= 0.0:
            frame_timestamp = time.time()
        if abs(frame_timestamp - self._depth_visual_last_task_ts) < 1e-6:
            return
        self._depth_visual_last_task_ts = frame_timestamp
        self._depth_visual_worker.submit({
            "frame_timestamp": frame_timestamp,
            "depth_image": getattr(frame, "depth_image", None),
        })

    def _schedule_analysis_for_frame(self, frame, *, reason: str, show_error: bool) -> None:
        if frame is None or getattr(frame, "color_image", None) is None:
            return
        if reason == "auto":
            if not self._should_schedule_auto_analysis():
                return
            now = time.time()
            if now < self._analysis_backoff_until_ts:
                return
            if self._analysis_worker.is_busy():
                return
            auto_interval = self._get_auto_analysis_interval()
            if now - self.last_auto_detect_time < auto_interval:
                return
            self.last_auto_detect_time = now
        enable_pose = self._should_include_pose_for_analysis(reason)
        enable_material = self._should_include_material_for_analysis(reason)
        enable_depth_positions = self._should_include_depth_positions_for_analysis(reason, enable_pose)
        task = {
            "frame_timestamp": float(getattr(frame, "timestamp", 0.0) or 0.0),
            "color_image": frame.color_image,
            "depth_image": getattr(frame, "depth_image", None),
            "intrinsics": dict(getattr(frame, "intrinsics", {}) or {}),
            "detection_cfg": self._get_detection_config(),
            "material_cfg": self._get_material_config() if enable_material else None,
            "pose_cfg": self._get_pose_config() if enable_pose else None,
            "enable_depth_positions": bool(enable_depth_positions),
            "enable_material": bool(enable_material),
            "enable_pose": bool(enable_pose),
            "reason": reason,
            "show_error": bool(show_error),
        }
        self._analysis_worker.submit(task)

    def _is_depth_view_active(self) -> bool:
        if not self.vision_viewer or not self.vision_tab:
            return False
        if self.tab_widget.currentWidget() is not self.vision_tab:
            return False
        return bool(getattr(self.vision_viewer, "is_depth_tab_active", lambda: False)())

    def _is_detection_analysis_view_active(self) -> bool:
        if not self.vision_viewer or not self.vision_tab:
            return False
        if self.tab_widget.currentWidget() is not self.vision_tab:
            return False
        return bool(
            getattr(self.vision_viewer, "is_detection_tab_active", lambda: False)()
            or getattr(self.vision_viewer, "is_pose_tab_active", lambda: False)()
        )

    def _should_schedule_auto_analysis(self) -> bool:
        if not self.auto_detect_enabled:
            return False
        if self._is_detection_analysis_view_active():
            return True
        if self._is_pointcloud_view_active():
            cfg = self._get_pointcloud_fusion_config() or {}
            return bool(cfg.get("roi_use_detection", False))
        return False

    def _get_auto_analysis_interval(self) -> float:
        interval = max(1.2, float(self.auto_detect_interval))
        if self._ros2_vision_stall_active:
            interval = max(interval, 2.0)
        if self._ros2_vision_rx_fps > 0.0 and self._ros2_vision_rx_fps < 12.0:
            interval = max(interval, 2.0)
        if self._ros2_vision_render_fps > 0.0 and self._ros2_vision_render_fps < 12.0:
            interval = max(interval, 2.0)
        return interval

    def _should_include_pose_for_analysis(self, reason: str) -> bool:
        if not self.vision_viewer or not self.vision_tab:
            return False
        if self.tab_widget.currentWidget() is not self.vision_tab:
            return False
        return bool(getattr(self.vision_viewer, "is_pose_tab_active", lambda: False)())

    def _should_include_material_for_analysis(self, reason: str) -> bool:
        return False

    def _should_include_depth_positions_for_analysis(self, reason: str, enable_pose: bool) -> bool:
        if enable_pose:
            return True
        if self._pointcloud_refresh_requested or self._is_pointcloud_view_active():
            cfg = self._get_pointcloud_fusion_config() or {}
            return bool(cfg.get("roi_use_detection", False))
        return reason != "auto"

    def _is_pointcloud_view_active(self) -> bool:
        if not self.vision_viewer or not self.vision_tab:
            return False
        if self.tab_widget.currentWidget() is not self.vision_tab:
            return False
        return self.vision_viewer.image_tabs.currentWidget() is self.vision_viewer.pointcloud_tab

    def _get_desired_vision_depth_profile(self) -> str:
        if self._pointcloud_refresh_requested or self._is_pointcloud_view_active():
            return "pointcloud"
        if self._is_depth_view_active():
            return "depth"
        if self._manual_detection_requested:
            return "analysis"
        if self.auto_detect_enabled and self._is_detection_analysis_view_active():
            return "analysis"
        return "off"

    def _sync_vision_depth_profile(self) -> None:
        if not self._is_ros2_vision_mode():
            return
        if self.data_acquisition_thread is None or not hasattr(self.data_acquisition_thread, "set_vision_depth_profile"):
            return
        desired_profile = self._get_desired_vision_depth_profile()
        if desired_profile == self._vision_depth_profile:
            return
        self._vision_depth_profile = desired_profile
        try:
            self.data_acquisition_thread.set_vision_depth_profile(desired_profile)
        except Exception:
            pass

    def _schedule_pointcloud_for_frame(self, frame, *, force: bool) -> None:
        if frame is None or getattr(frame, "depth_image", None) is None:
            return
        now = time.time()
        pointcloud_active = self._is_pointcloud_view_active()
        if not force and not pointcloud_active:
            return
        if not force and (now - self._pointcloud_last_schedule_ts) < self._pointcloud_schedule_interval_sec:
            return
        cfg = self._get_pointcloud_fusion_config() or {}
        task = {
            "frame_timestamp": float(getattr(frame, "timestamp", 0.0) or 0.0),
            "color_image": getattr(frame, "color_image", None),
            "depth_image": getattr(frame, "depth_image", None),
            "intrinsics": dict(getattr(frame, "intrinsics", {}) or {}),
            "cfg": dict(cfg),
            "detection_entries": list(self.last_detection_entries or []),
            "force": bool(force),
        }
        self._pointcloud_last_schedule_ts = now
        self._pointcloud_worker.submit(task)

    def _poll_vision_workers(self) -> None:
        self._analysis_result_overwrite_total += self._analysis_worker.take_overwrite_count()
        self._depth_visual_result_overwrite_total += self._depth_visual_worker.take_overwrite_count()
        self._pointcloud_result_overwrite_total += self._pointcloud_worker.take_overwrite_count()

        analysis_result = self._analysis_worker.take_result()
        if analysis_result is not None:
            self._apply_analysis_result(analysis_result.task, analysis_result.value, analysis_result.error)

        depth_result = self._depth_visual_worker.take_result()
        if depth_result is not None:
            self._apply_depth_visual_result(depth_result.task, depth_result.value, depth_result.error)

        pointcloud_result = self._pointcloud_worker.take_result()
        if pointcloud_result is not None:
            self._apply_pointcloud_result(pointcloud_result.task, pointcloud_result.value, pointcloud_result.error)

        if self._is_pointcloud_view_active() and self._ros2_vision_latest_frame is not None:
            self._schedule_pointcloud_for_frame(self._ros2_vision_latest_frame, force=False)

        self._update_vision_status_widgets()
        self._update_vision_stall_state(now=time.time(), frame_rendered=False)

    def _apply_analysis_result(self, task: Dict[str, Any], result: Optional[Dict[str, Any]], error: Optional[str]) -> None:
        if error:
            self.logger.warning("Vision analysis worker failed: %s", error)
            if task.get("show_error"):
                now = time.time()
                if now - self.last_detection_error_time > self.detection_error_cooldown:
                    self._show_camera_error(error)
                    self.last_detection_error_time = now
            return
        if not result:
            return
        self.last_detection_bboxes = list(result.get("detection_bboxes", []))
        self.last_detection_entries = list(result.get("detection_entries", []))
        self.last_detection_time = float(result.get("detection_time", 0.0) or 0.0)
        self._manual_detection_requested = False
        self._sync_vision_depth_profile()
        if self.vision_viewer:
            self.vision_viewer.update_image(result.get("detection_results", []), "detection")
            self.vision_viewer.update_image(result.get("pose_results", []), "pose")
        if self._pointcloud_refresh_requested and self._ros2_vision_latest_frame is not None:
            self._schedule_pointcloud_for_frame(self._ros2_vision_latest_frame, force=True)

    def _apply_depth_visual_result(self, task: Dict[str, Any], result: Optional[Dict[str, Any]], error: Optional[str]) -> None:
        if not self.vision_viewer:
            return
        if error:
            self.logger.debug("Depth visual worker failed: %s", error)
            return
        if not result:
            return
        frame_timestamp = float(task.get("frame_timestamp", 0.0) or 0.0)
        latest_timestamp = float(getattr(self._ros2_vision_latest_frame, "timestamp", 0.0) or 0.0) if self._ros2_vision_latest_frame is not None else 0.0
        if latest_timestamp > 0.0 and frame_timestamp > 0.0 and frame_timestamp + 1e-6 < latest_timestamp:
            return
        self.vision_viewer.update_image(result.get("depth_visual"), "depth")

    def _apply_pointcloud_result(self, task: Dict[str, Any], result: Optional[Dict[str, Any]], error: Optional[str]) -> None:
        if error:
            self.logger.debug("Pointcloud worker failed: %s", error)
            if task.get("force"):
                self._show_camera_error(error)
            return
        if not self.vision_viewer:
            return
        pointcloud_data = None if not result else result.get("pointcloud_data")
        if pointcloud_data:
            self.pointcloud_auto_render = True
            self._pointcloud_refresh_requested = False
            self.vision_viewer.update_image(pointcloud_data, "pointcloud")
            return
        if task.get("force"):
            self._show_camera_error("Point cloud is empty")

    def _refresh_live_self_check_status(self) -> None:
        if not self.vision_viewer:
            return
        frame = self._ros2_vision_latest_frame if self._is_ros2_vision_mode() else (self.camera_capture.get_latest_frame() if self.camera_capture else None)
        if frame is not None:
            self._refresh_self_check_status(frame)

    def _update_vision_status_widgets(self) -> None:
        if not self.vision_viewer:
            return
        self.vision_viewer.update_camera_status(
            connected=self._ros2_vision_connected,
            streaming=self._ros2_vision_streaming,
            resolution=self._ros2_vision_resolution,
            fps=self._ros2_vision_render_fps,
            rx_fps=self._ros2_vision_rx_fps if self._ros2_vision_rx_fps > 0 else self._ros2_vision_fps,
            dropped_frames=self._ros2_vision_dropped_frames + self._ros2_vision_queue_overwrite_total + self._depth_visual_result_overwrite_total,
            last_frame_age_ms=self._ros2_vision_last_frame_age_ms,
            stall_count=self._ros2_vision_stall_count,
        )

    def _update_vision_stall_state(self, *, now: float, frame_rendered: bool) -> None:
        if frame_rendered:
            if self._ros2_vision_stall_active:
                self.logger.info("Vision stall recovered after %.3fs (%s)", now - self._ros2_vision_stall_started_at, self._ros2_vision_stall_reason)
            self._ros2_vision_stall_active = False
            self._ros2_vision_stall_reason = ""
            return
        if not self._ros2_vision_connected:
            return
        since_render = now - self._ros2_vision_last_render_ts if self._ros2_vision_last_render_ts > 0 else 0.0
        if since_render < 0.6:
            return
        depth_backlog = self._depth_visual_worker.get_backlog_size()
        analysis_backlog = self._analysis_worker.get_backlog_size()
        pointcloud_backlog = self._pointcloud_worker.get_backlog_size()
        reason = "gui_render_blocked"
        if self._ros2_vision_last_frame_age_ms is not None and float(self._ros2_vision_last_frame_age_ms) > 1500.0:
            reason = "upstream_no_frame"
        elif depth_backlog > 1:
            reason = "depth_visual_backlog"
        elif analysis_backlog > 1:
            reason = "analysis_backlog"
        elif pointcloud_backlog > 1:
            reason = "pointcloud_backlog"
        if not self._ros2_vision_stall_active:
            self._ros2_vision_stall_active = True
            self._ros2_vision_stall_reason = reason
            self._ros2_vision_stall_started_at = now
            self._ros2_vision_stall_count += 1
            if reason in ("gui_render_blocked", "upstream_no_frame"):
                self._analysis_backoff_until_ts = max(
                    self._analysis_backoff_until_ts,
                    now + self._analysis_stall_backoff_sec,
                )
            active_view = "unknown"
            if self.vision_viewer and hasattr(self.vision_viewer, "image_tabs"):
                try:
                    active_view = str(self.vision_viewer.image_tabs.tabText(self.vision_viewer.image_tabs.currentIndex()))
                except Exception:
                    active_view = "unknown"
            self.logger.warning(
                "Vision stall detected: reason=%s render_gap=%.3fs frame_age_ms=%s rx_fps=%.2f render_fps=%.2f acq_drop=%d acq_overwrite=%d depth_overwrite=%d depth_backlog=%d analysis_backlog=%d pointcloud_backlog=%d analysis_busy=%s auto_detect=%s active_view=%s",
                reason,
                since_render,
                self._ros2_vision_last_frame_age_ms,
                self._ros2_vision_rx_fps,
                self._ros2_vision_render_fps,
                self._ros2_vision_dropped_frames,
                self._ros2_vision_queue_overwrite_total,
                self._depth_visual_result_overwrite_total,
                depth_backlog,
                analysis_backlog,
                pointcloud_backlog,
                self._analysis_worker.is_busy(),
                self.auto_detect_enabled,
                active_view,
            )

    def _process_ros2_vision_stream(self) -> None:
        if not self._is_ros2_vision_mode():
            return

        self._sync_vision_depth_profile()
        pulled = self._pull_ros2_vision_frame()
        if pulled is not None:
            self._queue_ros2_vision_frame(pulled)

        frame = None
        with self._ros2_vision_pending_lock:
            if self._ros2_vision_pending_frame is not None:
                frame = self._ros2_vision_pending_frame
                self._ros2_vision_pending_frame = None

        now = time.time()
        if frame is None:
            self._update_vision_stall_state(now=now, frame_rendered=False)
            return

        self._ros2_vision_connected = True
        self._ros2_vision_streaming = True
        self._render_vision_frame(frame, now)
        self._schedule_analysis_for_frame(frame, reason="auto", show_error=False)
        self._schedule_pointcloud_for_frame(frame, force=False)

    @pyqtSlot(dict)
    def _handle_ros2_vision_status(self, info: Dict[str, Any]):
        if not isinstance(info, dict):
            return
        self._ros2_vision_connected = bool(info.get("connected", False))
        self._ros2_vision_streaming = bool(info.get("streaming", self._ros2_vision_connected))
        self._ros2_vision_simulation = bool(info.get("simulation", False))
        self._ros2_vision_fps = float(info.get("fps", self._ros2_vision_fps or 0.0))
        self._ros2_vision_rx_fps = float(info.get("rx_fps", self._ros2_vision_fps))
        render_fps = info.get("render_fps", None)
        if render_fps is not None:
            try:
                render_fps_value = float(render_fps)
                if render_fps_value > 0.0:
                    self._ros2_vision_render_fps = render_fps_value
            except Exception:
                pass
        self._ros2_vision_resolution = str(info.get("resolution", self._ros2_vision_resolution))
        self._ros2_vision_dropped_frames = int(info.get("dropped_frames", self._ros2_vision_dropped_frames))
        self._ros2_vision_queue_overwrite_total = int(info.get("queue_overwrite_count", self._ros2_vision_queue_overwrite_total))
        self._ros2_vision_last_frame_age_ms = info.get("last_frame_age_ms", self._ros2_vision_last_frame_age_ms)
        device_info = str(info.get("device_info", "") or "").strip()
        if device_info:
            self._ros2_vision_device_info = device_info
        if self._ros2_vision_last_frame_age_ms is not None:
            self._ros2_vision_last_upstream_frame_wall_ts = time.time() - (float(self._ros2_vision_last_frame_age_ms) / 1000.0)
        self._update_vision_status_widgets()
        self.control_panel.update_device_status(
            vision={
                "connected": self._ros2_vision_connected,
                "simulation": self._ros2_vision_simulation,
            }
        )

    def _get_detection_config(self) -> Dict[str, Any]:
        """生成检测配置，兼容 DemoConfig 或 CameraConfig"""
        # 优先使用 CameraConfig 类属性
        cfg_src = getattr(CameraConfig, "OBJECT_DETECTION", {})
        # DemoConfig 里可能有 camera 字典
        if hasattr(self.config, "camera") and isinstance(self.config.camera, dict):
            cfg_src = self.config.camera.get("OBJECT_DETECTION", cfg_src)

        ui_overrides = {}
        if self.vision_viewer:
            try:
                ui_overrides = self.vision_viewer.get_detection_settings()
            except Exception:
                ui_overrides = {}

        cfg_model_type = cfg_src.get("model_type", "yolov5")
        model_name = ui_overrides.get("model_name", cfg_model_type)
        confidence_threshold = ui_overrides.get("confidence_threshold", cfg_src.get("confidence_threshold", 0.5))
        iou_threshold = ui_overrides.get("iou_threshold", cfg_src.get("iou_threshold", 0.45))

        model_path = cfg_src.get("model_path")
        if ui_overrides and model_name != cfg_model_type:
            model_path = None
        if model_path:
            if not os.path.isabs(model_path):
                model_path = os.path.join(project_root, model_path)
            if not os.path.exists(model_path):
                model_path = None
        else:
            model_path = None

        device = "cpu"
        try:
            import torch  # type: ignore
            if torch.cuda.is_available():
                device = "cuda"
        except Exception:
            device = "cpu"

        class_names = cfg_src.get("classes", [])
        if str(model_name).startswith("yolo") and not model_path:
            class_names = []

        return {
            "model_name": model_name,
            "model_path": model_path,
            "confidence_threshold": confidence_threshold,
            "iou_threshold": iou_threshold,
            "class_names": class_names,
            "device": device
        }

    def _detector_needs_reload(self, cfg: Dict[str, Any]) -> bool:
        if self.object_detector is None:
            return True
        if self.object_detector.model_name != cfg.get("model_name"):
            return True
        if (self.object_detector.model_path or "") != (cfg.get("model_path") or ""):
            return True
        if self.object_detector.device != cfg.get("device"):
            return True
        return False

    def _ensure_object_detector(self, cfg: Optional[Dict[str, Any]] = None) -> Optional[ObjectDetector]:
        """?????????"""
        cfg = dict(cfg or self._get_detection_config())
        if self.object_detector is None or self._detector_needs_reload(cfg):
            try:
                self.object_detector = ObjectDetector(cfg)
            except Exception as e:
                self.logger.warning(f"??????????????: {e}")
                self.object_detector = None
        elif self.object_detector is not None:
            self.object_detector.confidence_threshold = cfg.get("confidence_threshold", 0.5)
            self.object_detector.iou_threshold = cfg.get("iou_threshold", 0.45)
            self.object_detector.class_names = cfg.get("class_names", [])
            self.object_detector.num_classes = len(self.object_detector.class_names)
            self.object_detector.config = cfg
        return self.object_detector

    def _convert_detections_for_viewer(self, detections: List[Any]) -> List[Dict[str, Any]]:
        """将DetectionResult转换为VisionViewer可绘制的dict"""
        results = []
        for det in detections or []:
            bbox = getattr(det, "bbox", getattr(det, "box", []))
            confidence = getattr(det, "confidence", getattr(det, "score", 0.0))
            label = getattr(det, "class_name", getattr(det, "label", ""))
            material = getattr(det, "material", None)

            label_text = label or "object"
            if material:
                mat_name = getattr(material, "material_class", None) or getattr(material, "material", None)
                mat_conf = getattr(material, "confidence", None)
                if mat_name:
                    label_text = f"{label_text} | {mat_name}"
                    if mat_conf is not None:
                        try:
                            label_text = f"{label_text} {float(mat_conf):.2f}"
                        except Exception:
                            pass

            results.append({
                "bbox": bbox,
                "label": label_text,
                "confidence": float(confidence),
                "color": (0, 255, 0)
            })
        return results

    def _ensure_material_recognizer(self, mat_cfg: Optional[Dict[str, Any]] = None) -> Optional[MaterialRecognizer]:
        """?????????"""
        cfg = None if mat_cfg is None else dict(mat_cfg)
        if cfg is None:
            cfg = self._get_material_config()
        cfg = None if cfg is None else dict(cfg)
        if cfg and not cfg.get("enabled", True):
            self.material_recognizer = None
            self._material_recognizer_cfg = cfg
            return None
        if self.material_recognizer is None or self._material_recognizer_cfg != cfg:
            try:
                self.material_recognizer = MaterialRecognizer(cfg or {})
                self._material_recognizer_cfg = cfg
            except Exception as e:
                self.logger.warning(f"??????????????????: {e}")
                self.material_recognizer = None
                self._material_recognizer_cfg = cfg
        return self.material_recognizer

    def _ensure_pose_estimator(self, pose_cfg: Optional[Dict[str, Any]] = None) -> Optional[PoseEstimator]:
        """?????????"""
        cfg = None if pose_cfg is None else dict(pose_cfg)
        if cfg is None:
            cfg = self._get_pose_config()
        if cfg is None:
            cfg = {"method": "stub"}
        if self.pose_estimator is None or self._pose_estimator_cfg != cfg:
            try:
                self.pose_estimator = PoseEstimator(cfg)
                self._pose_estimator_cfg = dict(cfg)
            except Exception as e:
                self.logger.warning(f"??????????????????: {e}")
                self.pose_estimator = None
                self._pose_estimator_cfg = dict(cfg)
        return self.pose_estimator


    def _ensure_depth_processor(self) -> Optional[DepthProcessor]:
        """惰性创建深度处理器，用于ROI深度解算"""
        if self.depth_processor is not None:
            return self.depth_processor

        depth_cfg: Dict[str, Any] = {}
        camera_cfg = self._get_camera_config()
        if isinstance(camera_cfg, dict):
            depth_cfg = camera_cfg.get("DEPTH", {}) or {}
            depth_hw = camera_cfg.get("HARDWARE", {}).get("depth_camera", {}) or {}
        else:
            depth_cfg = getattr(camera_cfg, "DEPTH", {}) or {}
            depth_hw = getattr(camera_cfg, "HARDWARE", {}).get("depth_camera", {}) or {}

        # 深度范围优先取硬件配置（米）
        depth_clipping = depth_hw.get("depth_clipping") if isinstance(depth_hw, dict) else None
        if isinstance(depth_clipping, (list, tuple)) and len(depth_clipping) >= 2:
            depth_cfg = dict(depth_cfg)
            depth_cfg["min_depth"] = float(depth_clipping[0])
            depth_cfg["max_depth"] = float(depth_clipping[1])

        try:
            self.depth_processor = DepthProcessor(depth_cfg or {})
        except Exception as e:
            self.logger.warning(f"深度处理器创建失败: {e}")
            self.depth_processor = None
        return self.depth_processor

    @staticmethod
    def _build_camera_matrix_from_intrinsics(intrinsics: Dict[str, Any]) -> Optional[np.ndarray]:
        """从intrinsics构造相机内参矩阵"""
        if not intrinsics:
            return None
        fx = intrinsics.get("fx")
        fy = intrinsics.get("fy")
        cx = intrinsics.get("cx")
        cy = intrinsics.get("cy")
        if fx is None or fy is None or cx is None or cy is None:
            return None
        return np.array(
            [
                [float(fx), 0.0, float(cx)],
                [0.0, float(fy), float(cy)],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    def _estimate_object_position_from_depth(
        self,
        depth_image: Optional[np.ndarray],
        bbox: List[float],
        intrinsics: Optional[Dict[str, Any]],
    ) -> Optional[np.ndarray]:
        """
        使用深度图 + 相机内参估计物体中心（相机坐标系，单位：米）。
        """
        if depth_image is None or not bbox or len(bbox) != 4:
            return None
        depth_processor = self._ensure_depth_processor()
        if depth_processor is None:
            return None

        # 将内参同步到深度处理器，保证解算一致
        cam_mtx = self._build_camera_matrix_from_intrinsics(intrinsics or {})
        if cam_mtx is not None:
            depth_processor.set_camera_params(cam_mtx)

        result = depth_processor.estimate_object_center_camera(
            depth_image=depth_image,
            bbox=bbox,
            intrinsics=intrinsics,
            shrink_ratio=0.2,
            min_valid_pixels=30,
            depth_percentile=25.0,
        )
        if not result:
            return None
        return result.get("position")

    def _handle_capture_snapshot(self):
        """处理捕获请求：抓取当前帧并更新显示"""
        try:
            frame = self._get_latest_camera_frame()
            self._push_camera_frame_to_viewer(frame)
        except Exception as e:
            self._show_camera_error(str(e))


    def _handle_arm_function_execute(self):
        """处理机械臂功能面板的执行按钮"""
        mode = ""
        if self.arm_function_mode_combo is not None:
            mode = self.arm_function_mode_combo.currentText()
        if "自动夹取" in mode:
            self._execute_auto_grasp()
            return
        QMessageBox.information(self, "机械臂", "当前模式未实现")

    def _select_detection_for_grasp(self) -> Optional[Dict[str, Any]]:
        """???????????????????"""
        if not self.last_detection_entries:
            return None

        candidates: List[Dict[str, Any]] = []
        for entry in self.last_detection_entries:
            cam_pos_mm = entry.get("camera_position_mm")
            if not cam_pos_mm or len(cam_pos_mm) != 3:
                continue
            try:
                depth_mm = float(cam_pos_mm[2])
            except (TypeError, ValueError):
                continue
            # ????/???????????????
            if depth_mm <= 0:
                continue
            if depth_mm > self.auto_grasp_max_depth_mm:
                continue
            candidates.append(entry)

        if not candidates:
            return None

        # ??????????????????????
        candidates.sort(
            key=lambda e: (
                float(e.get("camera_position_mm", [0.0, 0.0, 1e9])[2]),
                -float(e.get("confidence", 0.0)),
            )
        )
        return candidates[0]

    def _execute_auto_grasp(self):
        """自动夹取：相机识别 -> 深度解算 -> 发送机械臂指令"""
        try:
            frame = self._get_latest_camera_frame()
            if frame is None or frame.color_image is None:
                raise RuntimeError("未获取到相机画面")

            # 若检测结果过旧或不存在，先刷新检测
            now = time.time()
            if (not self.last_detection_entries) or (now - self.last_detection_time > 1.5):
                self._run_detection_on_image(frame.color_image, frame.depth_image, frame.intrinsics)

            target = self._select_detection_for_grasp()
            if not target:
                raise RuntimeError("未找到可用于抓取的目标（深度无效或未检测到物体）")

            cam_pos_mm = target.get("camera_position_mm")
            if not cam_pos_mm or len(cam_pos_mm) != 3:
                raise RuntimeError("目标深度位置无效，请确保深度图正常")

            approach = 50.0
            if self.arm_function_approach_spin is not None:
                approach = float(self.arm_function_approach_spin.value())

            speed = 0.3
            if self.arm_function_speed_spin is not None:
                speed = float(self.arm_function_speed_spin.value())

            close_gripper = True
            if self.arm_function_gripper_check is not None:
                close_gripper = bool(self.arm_function_gripper_check.isChecked())

            close_pos = 40.0
            if self.arm_function_gripper_spin is not None:
                close_pos = float(self.arm_function_gripper_spin.value())

            params = {
                "object_cam_mm": [float(v) for v in cam_pos_mm],
                "approach_offset_mm": approach,
                "speed": speed,
                "close_gripper": close_gripper,
                "gripper_close_position": close_pos,
                "gripper_open_position": 100.0,
                "object_class": target.get("class_name"),
            }
            self.control_signal.emit("auto_grasp", params)
        except Exception as e:
            QMessageBox.warning(self, "机械臂", str(e))

    def _run_analysis_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        image = task.get("color_image")
        if image is None:
            raise RuntimeError("?????")
        depth_image = task.get("depth_image")
        intrinsics = dict(task.get("intrinsics") or {})
        enable_depth_positions = bool(task.get("enable_depth_positions", False))
        enable_material = bool(task.get("enable_material", False))
        enable_pose = bool(task.get("enable_pose", False))
        detector = self._ensure_object_detector(task.get("detection_cfg"))
        if detector is None:
            raise RuntimeError("??????????????????")
        if not getattr(detector, "model_ready", False):
            detail = getattr(detector, "last_error", None)
            message = "???????"
            if detail:
                message = f"{message}: {detail}"
            raise RuntimeError(message)

        detections = []
        try:
            detections = detector.detect(image).detections
        except Exception as det_err:
            self.logger.warning("????????????: %s", det_err)
            detections = []
        if not detections:
            try:
                image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                detections = detector.detect(image_bgr).detections
            except Exception:
                pass

        detection_bboxes: List[List[float]] = []
        detection_entries: List[Dict[str, Any]] = []
        depth_positions: Dict[int, np.ndarray] = {}
        for det in detections:
            bbox = getattr(det, "bbox", getattr(det, "box", []))
            confidence = getattr(det, "confidence", getattr(det, "score", 0.0))
            class_name = getattr(det, "class_name", getattr(det, "label", None))
            if bbox and len(bbox) == 4:
                bbox_f = [float(v) for v in bbox]
                detection_bboxes.append(bbox_f)
                cam_pos = None
                if enable_depth_positions and depth_image is not None:
                    cam_pos = self._estimate_object_position_from_depth(depth_image, bbox_f, intrinsics)
                if cam_pos is not None:
                    depth_positions[id(det)] = cam_pos
                detection_entries.append(
                    {
                        "bbox": bbox_f,
                        "confidence": float(confidence),
                        "class_name": class_name,
                        "camera_position": cam_pos.tolist() if cam_pos is not None else None,
                        "camera_position_mm": (cam_pos * 1000.0).tolist() if cam_pos is not None else None,
                    }
                )

        mat_recognizer = self._ensure_material_recognizer(task.get("material_cfg")) if enable_material else None
        if mat_recognizer:
            h_img, w_img = image.shape[:2]
            for det in detections:
                bbox = getattr(det, "bbox", getattr(det, "box", []))
                if bbox and len(bbox) == 4:
                    x1, y1, x2, y2 = map(int, bbox)
                    x1 = max(0, min(w_img - 1, x1))
                    y1 = max(0, min(h_img - 1, y1))
                    x2 = max(0, min(w_img, x2))
                    y2 = max(0, min(h_img, y2))
                    if x2 > x1 and y2 > y1:
                        try:
                            obj_type = getattr(det, "class_name", None) or getattr(det, "label", None)
                            det.material = mat_recognizer.recognize_from_vision(image, [x1, y1, x2, y2], object_type=obj_type)
                        except Exception as exc:
                            self.logger.debug("?????????: %s", exc)

        pose_results: List[Dict[str, Any]] = []
        pose_estimator = self._ensure_pose_estimator(task.get("pose_cfg")) if enable_pose else None
        if pose_estimator and intrinsics:
            cam_mtx = self._build_camera_matrix_from_intrinsics(intrinsics)
            if cam_mtx is not None:
                pose_estimator.camera_matrix = cam_mtx
        h_img, w_img = image.shape[:2]
        if enable_pose:
            for det in detections:
                bbox = getattr(det, "bbox", getattr(det, "box", []))
                pose_dict = None
                if bbox and len(bbox) == 4 and pose_estimator:
                    try:
                        obj_class = getattr(det, "class_name", None) or "object"
                        pose_estimator.ensure_simple_model(obj_class)
                        pose_obj = pose_estimator.estimate_pose(image, depth_image, obj_class, bbox)
                        if pose_obj:
                            pose_dict = {
                                "bbox": bbox,
                                "rotation": pose_obj.rotation,
                                "translation": pose_obj.position,
                                "object_class": getattr(det, "class_name", "object"),
                                "confidence": pose_obj.confidence,
                                "method": getattr(pose_obj, "method", "pnp"),
                            }
                    except Exception as exc:
                        self.logger.debug("????????????: %s", exc)
                if pose_dict is None:
                    pose_dict = self._generate_pose_stub(bbox, w_img, h_img, getattr(det, "class_name", "object"))
                cam_pos = depth_positions.get(id(det))
                if cam_pos is not None:
                    pose_dict["translation"] = cam_pos
                    pose_dict["method"] = "depth_roi"
                pose_results.append(pose_dict)

        return {
            "detection_bboxes": detection_bboxes,
            "detection_entries": detection_entries,
            "detection_results": self._convert_detections_for_viewer(detections),
            "pose_results": pose_results,
            "detection_time": time.time() if detections else 0.0,
        }

    def _run_depth_visual_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        depth_image = task.get("depth_image")
        if depth_image is None:
            return {"depth_visual": None}
        return {"depth_visual": colorize_depth_for_display(depth_image)}

    def _run_pointcloud_task(self, task: Dict[str, Any]) -> Dict[str, Any]:
        depth_image = task.get("depth_image")
        if depth_image is None:
            raise RuntimeError("Depth frame not available")
        cfg = dict(task.get("cfg") or {})
        if not cfg:
            return {"pointcloud_data": None}
        frame = SimpleNamespace(
            timestamp=float(task.get("frame_timestamp", 0.0) or 0.0),
            color_image=task.get("color_image"),
            depth_image=depth_image,
            intrinsics=dict(task.get("intrinsics") or {}),
        )
        detection_entries = list(task.get("detection_entries") or [])
        roi_bbox = self._select_pointcloud_roi(frame, cfg, detection_entries=detection_entries)
        fusion = self._ensure_pointcloud_fusion()
        if fusion is not None:
            if task.get("force") and cfg.get("reset_on_request", False):
                fusion.reset()
            if (not cfg.get("roi_use_detection", False)) or roi_bbox is not None:
                fusion.update(frame.depth_image, frame.color_image, frame.intrinsics or {}, roi_bbox=roi_bbox)
        if roi_bbox is not None:
            pointcloud_data = self._build_pointcloud_from_depth(frame, roi_bbox, cfg)
        elif cfg.get("roi_use_detection", False):
            pointcloud_data = None
        else:
            render_max = int(cfg.get("render_max_points", 20000) or 0)
            pointcloud_data = self._get_fused_pointcloud_data(render_max)
            if pointcloud_data is None:
                pointcloud_data = self._build_pointcloud_from_depth(frame, None, cfg)
        return {"pointcloud_data": pointcloud_data}

    def _run_detection_on_image(
        self,
        image: np.ndarray,
        depth_image: Optional[np.ndarray] = None,
        intrinsics: Optional[Dict[str, Any]] = None,
    ):
        """??????????+????????????????????"""
        try:
            result = self._run_analysis_task(
                {
                    "color_image": image,
                    "depth_image": depth_image,
                    "intrinsics": dict(intrinsics or {}),
                    "detection_cfg": self._get_detection_config(),
                    "material_cfg": None,
                    "pose_cfg": None,
                    "enable_depth_positions": bool(depth_image is not None),
                    "enable_material": False,
                    "enable_pose": False,
                    "reason": "manual-sync",
                    "show_error": True,
                }
            )
            self._apply_analysis_result({"show_error": True}, result, None)
        except Exception as exc:
            now = time.time()
            if now - self.last_detection_error_time > self.detection_error_cooldown:
                self._show_camera_error(str(exc))
                self.last_detection_error_time = now

    def _generate_pose_stub(self, bbox, w: int, h: int, obj_class: str = "object") -> Dict[str, Any]:
        """生成基于检测框中心的占位姿态（固定XYZ轴）"""
        if not bbox or len(bbox) != 4:
            return {
                "bbox": [0, 0, 0, 0],
                "rotation": np.eye(3),
                "translation": np.array([0.0, 0.0, 0.0]),
                "object_class": obj_class,
                "confidence": 0.0,
            }
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        # 用像素中心归一化到 [0,1] 作为平移占位
        tx = (cx - w / 2) / max(1, w)
        ty = (cy - h / 2) / max(1, h)
        tz = 0.5  # 固定深度占位
        return {
            "bbox": bbox,
            "rotation": np.eye(3),
            "translation": np.array([tx, ty, tz]),
            "object_class": obj_class,
            "confidence": 0.0,
            "method": "stub",
        }

    def _handle_detection_request(self):
        """?????????????????"""
        try:
            frame = self._get_latest_camera_frame()
            image = frame.color_image
            if image is None:
                raise RuntimeError("?????")
            self._manual_detection_requested = True
            self._sync_vision_depth_profile()
            self._push_camera_frame_to_viewer(frame)
            self._schedule_analysis_for_frame(frame, reason="manual", show_error=True)
        except Exception as e:
            self._show_camera_error(str(e))
    def _handle_auto_detect_toggle(self, enabled: bool):
        """????????"""
        self.auto_detect_enabled = enabled
        self.detect_in_progress = False
        self._sync_vision_depth_profile()
        if not enabled:
            return
        self._analysis_backoff_until_ts = 0.0
        if self._ros2_vision_latest_frame is not None and self._should_schedule_auto_analysis():
            self._schedule_analysis_for_frame(self._ros2_vision_latest_frame, reason="manual", show_error=False)
    def _handle_save_image_request(self, filename: str):
        """处理保存图像请求"""
        try:
            frame = self._get_latest_camera_frame()
            image = frame.color_image
            if image is None:
                raise RuntimeError("相机帧为空")

            save_dir = os.path.join(project_root, "data", "captures")
            os.makedirs(save_dir, exist_ok=True)
            filepath = os.path.join(save_dir, filename)

            # 保存为BGR以兼容常见查看工具
            bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(filepath, bgr)
            QMessageBox.information(self, "保存图像", f"已保存到 {filepath}")
        except Exception as e:
            self._show_camera_error(str(e))

    def _refresh_pointcloud_viewer(self, frame=None) -> None:
        """Push point cloud to viewer when in live mode."""
        if not self.vision_viewer:
            return
        if not self.pointcloud_auto_render:
            return
        cfg = self._get_pointcloud_fusion_config() or {}
        if frame is not None and cfg.get("roi_use_detection", False):
            roi_bbox = self._select_pointcloud_roi(frame, cfg)
            if roi_bbox is not None:
                pointcloud_data = self._build_pointcloud_from_depth(frame, roi_bbox, cfg)
                if pointcloud_data:
                    self.vision_viewer.update_image(pointcloud_data, "pointcloud")
                    return
            return

        render_max = int(cfg.get("render_max_points", 20000) or 0)
        pointcloud_data = self._get_fused_pointcloud_data(render_max)
        if pointcloud_data:
            self.vision_viewer.update_image(pointcloud_data, "pointcloud")

    def _handle_pointcloud_request(self):
        """Handle point cloud display request."""
        if not self.vision_viewer:
            return
        try:
            frame = self._get_latest_camera_frame()
            if getattr(frame, "depth_image", None) is None:
                raise RuntimeError("Depth frame not available")
            self.pointcloud_auto_render = True
            self.pointcloud_fusion_force_enabled = True
            self._pointcloud_refresh_requested = True
            self._sync_vision_depth_profile()
            cfg = self._get_pointcloud_fusion_config() or {}
            if cfg.get("roi_use_detection", False):
                max_age = float(cfg.get("roi_max_age", 0.5))
                if (not self.last_detection_entries) or (time.time() - self.last_detection_time > max_age):
                    self._schedule_analysis_for_frame(frame, reason="manual", show_error=False)
            self._schedule_pointcloud_for_frame(frame, force=True)
        except Exception as e:
            self._show_camera_error(str(e))
    def _handle_pointcloud_save_request(self, filename: str):
        """Handle point cloud save request."""
        prev_force = self.pointcloud_fusion_force_enabled
        self.pointcloud_fusion_force_enabled = True
        try:
            frame = self._get_latest_camera_frame()
            if getattr(frame, "depth_image", None) is None:
                raise RuntimeError("Depth frame not available")
            self._update_pointcloud_fusion(frame)

            fusion = self.pointcloud_fusion
            pcd = fusion.get_global_cloud() if fusion else None
            if pcd is None or len(pcd.points) == 0:
                raise RuntimeError("Point cloud is empty")
            if fusion is None or fusion.o3d is None:
                raise RuntimeError("Open3D not available")

            save_dir = os.path.join(project_root, "data", "pointclouds")
            os.makedirs(save_dir, exist_ok=True)
            if not os.path.splitext(filename)[1]:
                filename = f"{filename}.ply"
            filepath = os.path.join(save_dir, filename)
            saved = fusion.o3d.io.write_point_cloud(filepath, pcd)
            if not saved:
                raise RuntimeError("Failed to save point cloud")
            QMessageBox.information(self, "Point Cloud", f"Saved to {filepath}")
        except Exception as e:
            self._show_camera_error(str(e))
        finally:
            self.pointcloud_fusion_force_enabled = prev_force

    def _handle_self_check_request(self):
        """
        设备连接自检：
        - 展示设备序列号
        - 检查是否收到有效深度帧
        """
        if not self.vision_viewer:
            return

        if self._is_ros2_vision_mode():
            frame = self._ros2_vision_latest_frame
            if frame is None:
                self.vision_viewer.update_self_check_result("未连接", "未连接", depth_ok=False)
                self._show_camera_error("请先连接 ROS2 相机后再执行自检")
                return
            self._refresh_self_check_status(frame)
            return

        if not self.camera_capture or not self.camera_capture.is_capturing:
            # 未连接时给出明确提示
            self.vision_viewer.update_self_check_result("未连接", "未连接", depth_ok=False)
            self._show_camera_error("请先连接相机后再执行自检")
            return

        frame = self.camera_capture.get_latest_frame()
        if frame is None:
            frame = self.camera_capture.get_frame_with_timeout(0.5)
        self._refresh_self_check_status(frame)

    def _format_device_info_text(self) -> str:
        """格式化设备信息文本（型号/序列号/固件）"""
        if self._is_ros2_vision_mode():
            return self._ros2_vision_device_info or "ROS2 camera stream"
        if not self.camera_capture:
            return "N/A"
        device_info = self.camera_capture.get_device_info()
        serial = device_info.get("serial") or "N/A"
        name = device_info.get("name")
        firmware = device_info.get("firmware_version")

        text = serial
        if name and serial != "N/A":
            text = f"{name} / {serial}"
        elif name:
            text = name
        if firmware:
            text = f"{text} (FW {firmware})"
        return text or "N/A"

    def _evaluate_depth_status(self, frame) -> Tuple[str, Optional[bool]]:
        """评估深度状态，返回(状态文本, 是否正常)"""
        if self._is_ros2_vision_mode():
            if frame is None:
                return "未连接", False
            depth_image = getattr(frame, "depth_image", None)
            if depth_image is None or depth_image.size == 0:
                return "未收到深度帧", False
            valid_ratio = float(np.count_nonzero(depth_image)) / float(depth_image.size)
            if valid_ratio > 0.01:
                return f"正常（有效像素 {valid_ratio:.0%}）", True
            return "深度无效（有效像素过少）", False

        if not self.camera_capture:
            return "未连接", False
        if not getattr(self.camera_capture.config, "enable_depth", False):
            return "未启用", None

        depth_image = getattr(frame, "depth_image", None) if frame else None
        if depth_image is None or depth_image.size == 0:
            return "未收到深度帧", False

        valid_ratio = float(np.count_nonzero(depth_image)) / float(depth_image.size)
        if valid_ratio > 0.01:
            return f"正常（有效像素 {valid_ratio:.0%}）", True
        return "深度无效（有效像素过少）", False

    def _refresh_self_check_status(self, frame):
        """刷新自检显示，避免长时间显示N/A"""
        if not self.vision_viewer:
            return
        serial_text = self._format_device_info_text()
        self.camera_device_info_text = serial_text
        depth_status, depth_ok = self._evaluate_depth_status(frame)
        self.vision_viewer.update_self_check_result(serial_text, depth_status, depth_ok)

    def _get_simulation_config(self):
        """获取仿真配置，缺失时使用默认配置"""
        if hasattr(self.config, "simulation") and self.config.simulation is not None:
            if isinstance(self.config.simulation, SimulationConfig):
                return self.config.simulation
        try:
            return create_default_sim_config()
        except Exception:
            try:
                return SimulationConfig()
            except Exception:
                return None

    def _get_material_config(self) -> Optional[Dict[str, Any]]:
        """获取材质识别配置（CameraConfig或DemoConfig字典）"""
        cfg = None
        # CameraConfig 的类属性
        if hasattr(CameraConfig, "MATERIAL_RECOGNITION"):
            cfg = getattr(CameraConfig, "MATERIAL_RECOGNITION")
        # DemoConfig 里的 camera 字典
        if hasattr(self.config, "camera") and isinstance(self.config.camera, dict):
            cfg = self.config.camera.get("MATERIAL_RECOGNITION", cfg)
        if cfg is None:
            return None

        # 标准化字段
        model_path = cfg.get("model_path", "")
        if model_path and not os.path.isabs(model_path):
            model_path = os.path.join(project_root, model_path)

        return {
            "enabled": cfg.get("enabled", True),
            "model_type": cfg.get("model_type", "resnet50"),
            "model_path": model_path,
            "classes": cfg.get("classes", None),
            "use_object_prior": cfg.get("use_object_prior", True),
        }

    def _get_pose_config(self) -> Optional[Dict[str, Any]]:
        """获取姿态估计配置"""
        cfg = None
        if hasattr(CameraConfig, "POSE_ESTIMATION"):
            cfg = getattr(CameraConfig, "POSE_ESTIMATION")
        if hasattr(self.config, "camera") and isinstance(self.config.camera, dict):
            cfg = self.config.camera.get("POSE_ESTIMATION", cfg)
        if cfg is None:
            return None
        model_path = cfg.get("model_path", "")
        if model_path and not os.path.isabs(model_path):
            model_path = os.path.join(project_root, model_path)
        # 优先使用相机配置中的内参
        cam_cfg = self._get_camera_config()
        fx = cfg.get("fx", getattr(cam_cfg, "fx", 500.0) if cam_cfg else 500.0)
        fy = cfg.get("fy", getattr(cam_cfg, "fy", 500.0) if cam_cfg else 500.0)
        cx = cfg.get("cx", getattr(cam_cfg, "cx", 320.0) if cam_cfg else 320.0)
        cy = cfg.get("cy", getattr(cam_cfg, "cy", 240.0) if cam_cfg else 240.0)
        dist = cfg.get("dist_coeffs", getattr(cam_cfg, "distortion_coeffs", [0.0, 0.0, 0.0, 0.0]) if cam_cfg else [0.0, 0.0, 0.0, 0.0])
        return {
            "method": cfg.get("method", "pnp"),
            "feature_detector": cfg.get("feature_detector", "orb"),
            "model_path": model_path,
            "camera_params": {
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy,
                "dist_coeffs": dist,
            },
        }

    def _get_pointcloud_fusion_config(self) -> Optional[Dict[str, Any]]:
        """获取多视角点云融合配置（由相机深度配置与UI配置共同决定）"""
        cfg_src = getattr(CameraConfig, "DEPTH", {})
        if hasattr(self.config, "camera") and isinstance(self.config.camera, dict):
            cfg_src = self.config.camera.get("DEPTH", cfg_src)

        point_cfg = cfg_src.get("point_cloud", {}) or {}
        ui_cfg = getattr(self.config.ui, "vision_ui", {}) if hasattr(self.config, "ui") else {}
        ui_enabled = bool(ui_cfg.get("show_point_cloud", False)) if isinstance(ui_cfg, dict) else False
        force_enabled = bool(getattr(self, "pointcloud_fusion_force_enabled", False))
        if not point_cfg.get("enabled", False) or not (ui_enabled or force_enabled):
            return None

        cam_cfg = self._get_camera_config()
        depth_min = getattr(cam_cfg, "depth_min", None) if cam_cfg else None
        depth_max = getattr(cam_cfg, "depth_max", None) if cam_cfg else None

        # 组装融合配置，缺失项使用默认值
        return {
            "voxel_size": point_cfg.get("voxel_size", 0.01),
            "fusion_max_correspondence": point_cfg.get("fusion_max_correspondence", 0.05),
            "fusion_icp_iterations": point_cfg.get("fusion_icp_iterations", 30),
            "fusion_min_interval": point_cfg.get("fusion_min_interval", 0.2),
            "fusion_max_points": point_cfg.get("fusion_max_points", 200000),
            "use_color": point_cfg.get("use_color", True),
            "roi_use_detection": point_cfg.get("roi_use_detection", False),
            "roi_min_confidence": point_cfg.get("roi_min_confidence", 0.5),
            "roi_padding": point_cfg.get("roi_padding", 0),
            "roi_sample_step": point_cfg.get("roi_sample_step", 2),
            "roi_strategy": point_cfg.get("roi_strategy", "highest_confidence"),
            "roi_max_age": point_cfg.get("roi_max_age", 0.5),
            "render_max_points": point_cfg.get("render_max_points", 20000),
            "reset_on_request": point_cfg.get("reset_on_request", False),
            "roi_use_detection": point_cfg.get("roi_use_detection", False),
            "roi_min_confidence": point_cfg.get("roi_min_confidence", 0.5),
            "roi_padding": point_cfg.get("roi_padding", 0),
            "roi_strategy": point_cfg.get("roi_strategy", "highest_confidence"),
            "roi_max_age": point_cfg.get("roi_max_age", 0.5),
            "render_max_points": point_cfg.get("render_max_points", 20000),
            "reset_on_request": point_cfg.get("reset_on_request", False),
            "statistical_outlier_removal": point_cfg.get("statistical_outlier_removal", {}),
            "min_depth": depth_min if depth_min is not None else point_cfg.get("min_depth", 0.1),
            "max_depth": depth_max if depth_max is not None else point_cfg.get("max_depth", 3.0),
        }

    def _ensure_pointcloud_fusion(self) -> Optional[MultiViewPointCloudFusion]:
        """惰性创建点云融合器，配置变化时自动重建"""
        cfg = self._get_pointcloud_fusion_config()
        if cfg is None:
            self.pointcloud_fusion = None
            self.pointcloud_fusion_config = None
            return None
        if self.pointcloud_fusion is None or self.pointcloud_fusion_config != cfg:
            self.pointcloud_fusion = MultiViewPointCloudFusion(cfg)
            self.pointcloud_fusion_config = cfg
        return self.pointcloud_fusion

    def _build_pointcloud_from_depth(self, frame, roi_bbox, cfg: Dict[str, Any]) -> Optional[Dict[str, np.ndarray]]:
        """根据深度图与ROI构建点云数据（用于显示）"""
        if frame is None or frame.depth_image is None:
            return None

        depth = frame.depth_image
        h, w = depth.shape[:2]
        if roi_bbox:
            x1, y1, x2, y2 = roi_bbox
            x1 = max(0, min(w - 1, int(x1)))
            y1 = max(0, min(h - 1, int(y1)))
            x2 = max(x1 + 1, min(w, int(x2)))
            y2 = max(y1 + 1, min(h, int(y2)))
        else:
            x1, y1, x2, y2 = 0, 0, w, h

        step = int(cfg.get("roi_sample_step", 2) or 1)
        if step < 1:
            step = 1

        depth_roi = depth[y1:y2:step, x1:x2:step]
        if depth_roi.size == 0:
            return None

        min_depth = cfg.get("min_depth", None)
        max_depth = cfg.get("max_depth", None)
        mask = depth_roi > 0
        if min_depth is not None:
            mask &= depth_roi >= float(min_depth)
        if max_depth is not None:
            mask &= depth_roi <= float(max_depth)
        if not mask.any():
            return None

        yy, xx = np.mgrid[y1:y2:step, x1:x2:step]
        zz = depth_roi[mask]
        uu = xx[mask].astype(np.float32)
        vv = yy[mask].astype(np.float32)

        intr = frame.intrinsics or {}
        fx = float(intr.get("fx", 0.0))
        fy = float(intr.get("fy", 0.0))
        cx = float(intr.get("cx", 0.0))
        cy = float(intr.get("cy", 0.0))
        if fx <= 0 or fy <= 0:
            return None

        x = (uu - cx) * zz / fx
        y = (vv - cy) * zz / fy
        points = np.stack([x, y, zz], axis=1)

        colors = None
        if frame.color_image is not None:
            color = frame.color_image[y1:y2:step, x1:x2:step]
            colors = color[mask]
            if colors is not None and len(colors) > 0:
                colors = colors.astype(np.float32) / 255.0

        max_points = int(cfg.get("render_max_points", 20000) or 0)
        if max_points > 0 and len(points) > max_points:
            indices = np.random.choice(len(points), size=max_points, replace=False)
            points = points[indices]
            if colors is not None:
                colors = colors[indices]

        return {"points": points, "colors": colors}

    def _select_pointcloud_roi(self, frame, cfg: Dict[str, Any], detection_entries: Optional[List[Dict[str, Any]]] = None) -> Optional[Tuple[int, int, int, int]]:
        """??????????ROI"""
        if not cfg or not cfg.get("roi_use_detection", False):
            return None

        entries = detection_entries if detection_entries is not None else (self.last_detection_entries or [])
        min_conf = float(cfg.get("roi_min_confidence", 0.5))
        candidates = [e for e in entries if e.get("confidence", 0.0) >= min_conf]
        if not candidates:
            return None

        strategy = str(cfg.get("roi_strategy", "highest_confidence")).lower()
        if strategy == "largest":
            candidates.sort(
                key=lambda e: (e["bbox"][2] - e["bbox"][0]) * (e["bbox"][3] - e["bbox"][1]),
                reverse=True,
            )
        else:
            candidates.sort(key=lambda e: e.get("confidence", 0.0), reverse=True)

        bbox = candidates[0].get("bbox")
        if not bbox or len(bbox) != 4:
            return None

        pad = int(cfg.get("roi_padding", 0))
        if frame.depth_image is not None:
            h, w = frame.depth_image.shape[:2]
        elif frame.color_image is not None:
            h, w = frame.color_image.shape[:2]
        else:
            return None

        x1, y1, x2, y2 = map(int, bbox)
        x1 = max(0, min(w - 1, x1 - pad))
        y1 = max(0, min(h - 1, y1 - pad))
        x2 = max(x1 + 1, min(w, x2 + pad))
        y2 = max(y1 + 1, min(h, y2 + pad))
        return (x1, y1, x2, y2)
    def _update_pointcloud_fusion(self, frame) -> None:
        """用最新深度帧更新多视角点云融合结果"""
        if frame is None or frame.depth_image is None:
            return

        fusion = self._ensure_pointcloud_fusion()
        if fusion is None:
            return

        cfg = self._get_pointcloud_fusion_config() or {}
        roi_bbox = self._select_pointcloud_roi(frame, cfg)
        if cfg.get("roi_use_detection", False) and roi_bbox is None and self.last_detection_bboxes:
            # 如果ROI为空则使用最近一次检测框
            roi_bbox = tuple(int(v) for v in self.last_detection_bboxes[0])

        if cfg.get("roi_use_detection", False) and roi_bbox is None:
            return

        fusion.update(
            frame.depth_image,
            frame.color_image,
            frame.intrinsics or {},
            roi_bbox=roi_bbox
        )

    def _get_fused_pointcloud_data(self, max_points: int = 20000) -> Optional[Dict[str, np.ndarray]]:
        """Prepare point cloud arrays for the UI renderer."""
        fusion = self.pointcloud_fusion
        if fusion is None:
            return None
        pcd = fusion.get_global_cloud()
        if pcd is None or len(pcd.points) == 0:
            return None

        points = np.asarray(pcd.points)
        colors = None
        has_colors = getattr(pcd, "has_colors", None)
        if callable(has_colors) and has_colors():
            colors = np.asarray(pcd.colors)

        if max_points and len(points) > max_points:
            indices = np.random.choice(len(points), size=max_points, replace=False)
            points = points[indices]
            if colors is not None:
                colors = colors[indices]

        return {"points": points, "colors": colors}

    def _get_arm_config(self):
        """获取机械臂配置，缺失时使用默认配置"""
        if hasattr(self.config, "learm_arm") and self.config.learm_arm is not None:
            if isinstance(self.config.learm_arm, LearmArmConfig):
                return self.config.learm_arm
        try:
            return create_default_learm_config()
        except Exception:
            try:
                return LearmArmConfig()
            except Exception:
                return None

    def init_visualization_toolbar(self, parent_layout):
        """初始化可视化控制工具栏"""
        self.viz_toolbar = QWidget()
        viz_layout = QHBoxLayout(self.viz_toolbar)
        viz_layout.setContentsMargins(5, 2, 5, 2)
        
        # 可视化模式选择
        mode_label = QLabel("可视化模式:")
        viz_layout.addWidget(mode_label)
        
        self.viz_mode_combo = QComboBox()
        self.viz_mode_combo.addItems(["热力图", "矢量图", "3D视图", "混合视图"])
        self.viz_mode_combo.currentTextChanged.connect(self.change_visualization_mode)
        viz_layout.addWidget(self.viz_mode_combo)
        
        # 矢量缩放控制
        scale_label = QLabel("矢量缩放:")
        viz_layout.addWidget(scale_label)
        
        self.vector_scale_spin = QDoubleSpinBox()
        self.vector_scale_spin.setRange(0.1, 5.0)
        self.vector_scale_spin.setSingleStep(0.1)
        self.vector_scale_spin.setValue(1.0)
        self.vector_scale_spin.valueChanged.connect(self.update_vector_scale)
        viz_layout.addWidget(self.vector_scale_spin)
        
        # 显示选项
        self.show_labels_check = QCheckBox("显示力标签")
        self.show_labels_check.setChecked(True)
        self.show_labels_check.stateChanged.connect(self.toggle_force_labels)
        viz_layout.addWidget(self.show_labels_check)
        
        self.show_grid_check = QCheckBox("显示网格")
        self.show_grid_check.setChecked(True)
        self.show_grid_check.stateChanged.connect(self.toggle_grid)
        viz_layout.addWidget(self.show_grid_check)
        
        viz_layout.addStretch()
        
        # 刷新按钮
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_visualization)
        viz_layout.addWidget(self.refresh_btn)
        
        parent_layout.addWidget(self.viz_toolbar)
    
    def init_menu(self):
        """初始化菜单栏 - 添加三维力相关菜单"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu("文件(&F)")
        
        # 新建配置
        new_config_action = QAction("新建配置(&N)", self)
        new_config_action.setShortcut("Ctrl+N")
        new_config_action.triggered.connect(self.new_config)
        file_menu.addAction(new_config_action)
        
        # 打开配置
        open_config_action = QAction("打开配置(&O)", self)
        open_config_action.setShortcut("Ctrl+O")
        open_config_action.triggered.connect(self.open_config)
        file_menu.addAction(open_config_action)
        
        # 保存配置
        save_config_action = QAction("保存配置(&S)", self)
        save_config_action.setShortcut("Ctrl+S")
        save_config_action.triggered.connect(self.save_config)
        file_menu.addAction(save_config_action)
        
        # 另存配置为
        save_as_action = QAction("另存配置为(&A)", self)
        save_as_action.setShortcut("Ctrl+Shift+S")
        save_as_action.triggered.connect(self.save_config_as)
        file_menu.addAction(save_as_action)
        
        file_menu.addSeparator()
        
        # 导出三维力数据
        export_3d_data_action = QAction("导出三维力数据(&3)", self)
        export_3d_data_action.triggered.connect(self.export_3d_data)
        file_menu.addAction(export_3d_data_action)
        
        # 导出数据
        export_data_action = QAction("导出数据(&E)", self)
        export_data_action.triggered.connect(self.export_data)
        file_menu.addAction(export_data_action)
        
        file_menu.addSeparator()
        
        # 退出
        exit_action = QAction("退出(&X)", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # 控制菜单
        control_menu = menubar.addMenu("控制(&C)")
        
        # 连接硬件
        connect_hardware_action = QAction("连接硬件(&C)", self)
        connect_hardware_action.setShortcut("Ctrl+H")
        connect_hardware_action.triggered.connect(self.connect_hardware)
        control_menu.addAction(connect_hardware_action)
        
        # 断开硬件
        disconnect_hardware_action = QAction("断开硬件(&D)", self)
        disconnect_hardware_action.triggered.connect(self.disconnect_hardware)
        control_menu.addAction(disconnect_hardware_action)
        
        control_menu.addSeparator()
        
        # 三维力校准
        calibrate_3d_action = QAction("三维力校准(&3)", self)
        calibrate_3d_action.setShortcut("Ctrl+3")
        calibrate_3d_action.triggered.connect(self.calibrate_3d)
        control_menu.addAction(calibrate_3d_action)
        
        # 校准
        calibrate_action = QAction("校准(&B)", self)
        calibrate_action.setShortcut("Ctrl+B")
        calibrate_action.triggered.connect(self.calibrate)
        control_menu.addAction(calibrate_action)
        
        control_menu.addSeparator()
        
        # 启动三维力演示
        start_3d_demo_action = QAction("启动三维力演示(&V)", self)
        start_3d_demo_action.setShortcut("Ctrl+V")
        start_3d_demo_action.triggered.connect(self.start_3d_demo)
        control_menu.addAction(start_3d_demo_action)
        
        # 启动演示
        start_demo_action = QAction("启动演示(&S)", self)
        start_demo_action.setShortcut("Ctrl+D")
        start_demo_action.triggered.connect(self.start_demo)
        control_menu.addAction(start_demo_action)
        
        # 停止演示
        stop_demo_action = QAction("停止演示(&T)", self)
        stop_demo_action.setShortcut("Ctrl+Shift+D")
        stop_demo_action.triggered.connect(self.stop_demo)
        control_menu.addAction(stop_demo_action)
        
        # 视图菜单
        view_menu = menubar.addMenu("视图(&V)")
        
        # 显示/隐藏工具栏
        self.toolbar_action = QAction("工具栏(&T)", self)
        self.toolbar_action.setCheckable(True)
        self.toolbar_action.setChecked(True)
        self.toolbar_action.triggered.connect(self.toggle_toolbar)
        view_menu.addAction(self.toolbar_action)
        
        # 显示/隐藏状态栏
        self.statusbar_action = QAction("状态栏(&S)", self)
        self.statusbar_action.setCheckable(True)
        self.statusbar_action.setChecked(True)
        self.statusbar_action.triggered.connect(self.toggle_statusbar)
        view_menu.addAction(self.statusbar_action)
        
        # 显示/隐藏矢量工具栏
        self.vector_toolbar_action = QAction("矢量工具栏(&V)", self)
        self.vector_toolbar_action.setCheckable(True)
        self.vector_toolbar_action.setChecked(True)
        self.vector_toolbar_action.triggered.connect(self.toggle_vector_toolbar)
        view_menu.addAction(self.vector_toolbar_action)
        
        view_menu.addSeparator()
        
        # 三维力视图
        view_3d_menu = view_menu.addMenu("三维力视图(&3)")
        
        view_3d_xy_action = QAction("XY平面视图", self)
        view_3d_xy_action.triggered.connect(lambda: self.change_3d_view('xy'))
        view_3d_menu.addAction(view_3d_xy_action)
        
        view_3d_xz_action = QAction("XZ平面视图", self)
        view_3d_xz_action.triggered.connect(lambda: self.change_3d_view('xz'))
        view_3d_menu.addAction(view_3d_xz_action)
        
        view_3d_yz_action = QAction("YZ平面视图", self)
        view_3d_yz_action.triggered.connect(lambda: self.change_3d_view('yz'))
        view_3d_menu.addAction(view_3d_yz_action)
        
        view_3d_3d_action = QAction("3D视图", self)
        view_3d_3d_action.triggered.connect(lambda: self.change_3d_view('3d'))
        view_3d_menu.addAction(view_3d_3d_action)
        
        # 主题
        theme_menu = view_menu.addMenu("主题(&M)")
        
        light_theme_action = QAction("浅色主题", self)
        light_theme_action.triggered.connect(lambda checked, name="light": self.change_theme(name))
        theme_menu.addAction(light_theme_action)
        
        dark_theme_action = QAction("深色主题", self)
        dark_theme_action.triggered.connect(lambda checked, name="dark": self.change_theme(name))
        theme_menu.addAction(dark_theme_action)
        
        tech_theme_action = QAction("科技主题", self)
        tech_theme_action.triggered.connect(lambda checked, name="tech": self.change_theme(name))
        theme_menu.addAction(tech_theme_action)
        
        # 工具菜单
        tools_menu = menubar.addMenu("工具(&T)")
        
        # 三维力分析
        force_analysis_action = QAction("三维力分析(&F)", self)
        force_analysis_action.triggered.connect(self.open_force_analysis)
        tools_menu.addAction(force_analysis_action)
        
        # 矢量场分析
        vector_analysis_action = QAction("矢量场分析(&V)", self)
        vector_analysis_action.triggered.connect(self.open_vector_analysis)
        tools_menu.addAction(vector_analysis_action)
        
        tools_menu.addSeparator()
        
        # 配置编辑器
        config_editor_action = QAction("配置编辑器(&E)", self)
        config_editor_action.triggered.connect(self.open_config_editor)
        tools_menu.addAction(config_editor_action)
        
        # 日志查看器
        log_viewer_action = QAction("日志查看器(&L)", self)
        log_viewer_action.triggered.connect(self.open_log_viewer)
        tools_menu.addAction(log_viewer_action)
        
        tools_menu.addSeparator()
        
        # 系统监控
        system_monitor_action = QAction("系统监控(&M)", self)
        system_monitor_action.triggered.connect(self.open_system_monitor)
        tools_menu.addAction(system_monitor_action)
        
        # 帮助菜单
        help_menu = menubar.addMenu("帮助(&H)")
        
        # 三维力用户手册
        force_manual_action = QAction("三维力使用手册(&3)", self)
        force_manual_action.triggered.connect(self.open_force_manual)
        help_menu.addAction(force_manual_action)
        
        # 用户手册
        user_manual_action = QAction("用户手册(&U)", self)
        user_manual_action.triggered.connect(self.open_user_manual)
        help_menu.addAction(user_manual_action)
        
        help_menu.addSeparator()
        
        # 关于
        about_action = QAction("关于(&A)", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
    
    def init_toolbar(self):
        """初始化工具栏 - 添加三维力控制按钮"""
        toolbar = self.addToolBar("主工具栏")
        toolbar.setMovable(False)
        self.toolbar_main = toolbar
        # 调小按钮尺寸
        toolbar.setIconSize(QSize(20, 20))
        
        # 连接硬件按钮
        self.connect_btn = QPushButton("连接硬件")
        self.connect_btn.clicked.connect(self.connect_hardware)
        toolbar.addWidget(self.connect_btn)
        
        # 断开硬件按钮
        self.disconnect_btn = QPushButton("断开硬件")
        self.disconnect_btn.clicked.connect(self.disconnect_hardware)
        self.disconnect_btn.setEnabled(False)
        toolbar.addWidget(self.disconnect_btn)
        
        toolbar.addSeparator()
        
        # 三维力校准按钮
        self.calibrate_3d_btn = QPushButton("三维力校准")
        self.calibrate_3d_btn.clicked.connect(self.calibrate_3d)
        self.calibrate_3d_btn.setToolTip("校准三维力传感器 (Ctrl+3)")
        toolbar.addWidget(self.calibrate_3d_btn)
        
        # 校准按钮
        self.calibrate_btn = QPushButton("校准")
        self.calibrate_btn.clicked.connect(self.calibrate)
        toolbar.addWidget(self.calibrate_btn)
        
        toolbar.addSeparator()
        
        # 启动三维力演示按钮
        self.start_3d_demo_btn = QPushButton("三维力演示")
        self.start_3d_demo_btn.clicked.connect(self.start_3d_demo)
        self.start_3d_demo_btn.setToolTip("启动三维力矢量可视化演示 (Ctrl+V)")
        toolbar.addWidget(self.start_3d_demo_btn)
        
        # 启动演示按钮
        self.start_demo_btn = QPushButton("启动演示")
        self.start_demo_btn.clicked.connect(self.start_demo)
        toolbar.addWidget(self.start_demo_btn)
        
        # 停止演示按钮
        self.stop_demo_btn = QPushButton("停止演示")
        self.stop_demo_btn.clicked.connect(self.stop_demo)
        self.stop_demo_btn.setEnabled(False)
        toolbar.addWidget(self.stop_demo_btn)
        
        toolbar.addSeparator()
        
        # 紧急停止按钮
        self.emergency_btn = QPushButton("紧急停止")
        self.emergency_btn.setCheckable(True)
        self.emergency_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.emergency_btn.toggled.connect(self.toggle_emergency)
        toolbar.addWidget(self.emergency_btn)

        toolbar.addSeparator()

        # 三维力视图切换按钮
        self.vector_view_btn = QPushButton("矢量视图")
        self.vector_view_btn.setCheckable(True)
        self.vector_view_btn.setChecked(False)
        self.vector_view_btn.toggled.connect(self.toggle_vector_view)
        toolbar.addWidget(self.vector_view_btn)

        # 统一缩小工具栏按钮尺寸和内边距
        small_buttons = [
            self.connect_btn, self.disconnect_btn,
            self.calibrate_3d_btn, self.calibrate_btn,
            self.start_3d_demo_btn, self.start_demo_btn,
            self.stop_demo_btn, self.emergency_btn,
            self.vector_view_btn
        ]
        for btn in small_buttons:
            btn.setFixedHeight(28)
            btn.setStyleSheet(btn.styleSheet() + "\nQPushButton { padding: 3px 8px; font-size: 11px; }")
    
    def init_status_bar(self):
        """初始化状态栏 - 添加三维力信息"""
        status_bar = self.statusBar()
        
        # 系统状态标签
        self.status_label = QLabel("系统状态: 就绪")
        self.status_label.setStyleSheet("QLabel { padding: 2px 10px; }")
        status_bar.addWidget(self.status_label)
        
        # 三维力状态标签
        self.force_3d_status_label = QLabel("三维力: 未激活")
        self.force_3d_status_label.setStyleSheet("""
            QLabel { 
                padding: 2px 10px; 
                background-color: #ffcccc; 
                border-radius: 3px;
            }
        """)
        status_bar.addWidget(self.force_3d_status_label)
        
        # 数据率标签
        self.data_rate_label = QLabel("数据率: 0.0 Hz")
        status_bar.addPermanentWidget(self.data_rate_label)
        
        # 数据包计数标签
        self.packet_label = QLabel("数据包: 0")
        status_bar.addPermanentWidget(self.packet_label)
        
        # 时间标签
        self.time_label = QLabel()
        self.update_time()
        status_bar.addPermanentWidget(self.time_label)
    
    def init_timers(self):
        """??????"""
        self.time_timer = QTimer()
        self.time_timer.timeout.connect(self.update_time)
        self.time_timer.start(1000)

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(500)

        self.data_update_timer = QTimer()
        self.data_update_timer.timeout.connect(self.update_all_plots)
        self.data_update_timer.start(60)

        self.vector_update_timer = QTimer()
        self.vector_update_timer.timeout.connect(self.update_vector_visualization)
        self.vector_update_timer.start(300)

        self.rate_timer = QTimer()
        self.rate_timer.timeout.connect(self.calculate_data_rate)
        self.rate_timer.start(1000)

        self.force_3d_timer = QTimer()
        self.force_3d_timer.timeout.connect(self.update_3d_force_status)
        self.force_3d_timer.start(200)

        self.ros2_vision_timer = QTimer()
        self.ros2_vision_timer.timeout.connect(self._process_ros2_vision_stream)
        self.ros2_vision_timer.start(int(self._ros2_vision_render_target_interval_sec * 1000.0))

        self.vision_worker_timer = QTimer()
        self.vision_worker_timer.timeout.connect(self._poll_vision_workers)
        self.vision_worker_timer.start(33)

        self.vision_self_check_timer = QTimer()
        self.vision_self_check_timer.timeout.connect(self._refresh_live_self_check_status)
        self.vision_self_check_timer.start(1000)

    def calculate_data_rate(self):
        """计算并更新数据率"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed > 0:
            current_rate = self.frame_count / elapsed
            # 平滑处理数据率
            if self.data_rate == 0:
                self.data_rate = current_rate
            else:
                self.data_rate = 0.7 * self.data_rate + 0.3 * current_rate
                
            # 更新状态栏显示
            self.data_rate_label.setText(f"数据率: {self.data_rate:.1f} Hz")
            
            # 重置计数器
            self.frame_count = 0
            self.start_time = current_time
    
    def connect_signals(self):
        """连接信号"""
        # 连接演示管理器信号
        if self.demo_manager is not None:
            if hasattr(self.demo_manager, "status_changed"):
                self.demo_manager.status_changed.connect(self.update_system_status)
            if hasattr(self.demo_manager, "vector_data_updated"):
                self.demo_manager.vector_data_updated.connect(self.handle_vector_data)
            if hasattr(self.demo_manager, "contact_map_updated"):
                self.demo_manager.contact_map_updated.connect(self.handle_contact_data)
        
        # 连接数据采集线程信号
        if self.data_acquisition_thread is not None and hasattr(self.data_acquisition_thread, "new_data"):
            self.data_acquisition_thread.new_data.connect(self.update_sensor_data)
        if self.data_acquisition_thread is not None and hasattr(self.data_acquisition_thread, "vision_frame"):
            self.data_acquisition_thread.vision_frame.connect(self._handle_ros2_vision_frame)
        if self.data_acquisition_thread is not None and hasattr(self.data_acquisition_thread, "vision_status"):
            self.data_acquisition_thread.vision_status.connect(self._handle_ros2_vision_status)
        
        # 连接控制线程信号
        if self.control_thread is not None and hasattr(self.control_thread, "status_updated"):
            self.control_thread.status_updated.connect(self.update_control_status)

        # 连接控制面板内的视觉连接按钮
        if self.control_panel is not None:
            self.control_panel.vision_connect_request.connect(self._handle_connect_camera)
            self.control_panel.vision_disconnect_request.connect(self._handle_disconnect_camera)

        # 连接机械臂控制信号
        if self.arm_status_panel:
            self.arm_status_panel.arm_connect_request.connect(
                lambda: self.control_signal.emit("connect_arm", {})
            )
            self.arm_status_panel.arm_disconnect_request.connect(
                lambda: self.control_signal.emit("disconnect_arm", {})
            )
            self.arm_status_panel.arm_enable_request.connect(
                lambda: self.control_signal.emit("arm_enable", {})
            )
            self.arm_status_panel.arm_disable_request.connect(
                lambda: self.control_signal.emit("arm_disable", {})
            )
            self.arm_status_panel.arm_home_request.connect(
                lambda: self.control_signal.emit("arm_home", {})
            )
            self.arm_status_panel.joint_angle_request.connect(self._handle_joint_angle_request)
    
    @pyqtSlot(object)
    def handle_vector_data(self, vector_data):
        """处理三维力向量数据"""
        self.force_vectors = vector_data
        
        # 发送信号更新矢量图
        self.vector_data_updated.emit(vector_data)
        
    
    @pyqtSlot(object)
    def handle_contact_data(self, contact_data):
        """处理接触数据"""
        self.contact_map = contact_data
        
        # 发送信号更新接触部件
        self.contact_data_updated.emit(contact_data)
        
    
    @pyqtSlot(str, dict)
    def update_system_status(self, status: str, info: dict):
        """
        更新系统状态
        
        Args:
            status: 状态字符串
            info: 状态信息字典
        """
        self.status_label.setText(f"系统状态: {status}")
        
        # 根据状态设置标签样式
        if status == "就绪":
            self.status_label.setStyleSheet("""
                QLabel { 
                    padding: 2px 10px; 
                    background-color: #ccffcc; 
                    border-radius: 3px;
                }
            """)
        elif status == "运行中":
            self.status_label.setStyleSheet("""
                QLabel { 
                    padding: 2px 10px; 
                    background-color: #ffffcc; 
                    border-radius: 3px;
                }
            """)
        elif status == "错误":
            self.status_label.setStyleSheet("""
                QLabel { 
                    padding: 2px 10px; 
                    background-color: #ffcccc; 
                    border-radius: 3px;
                    color: #cc0000;
                }
            """)
    
    @pyqtSlot(object)
    def update_sensor_data(self, data):
        """
        处理传感器数据回调，控制更新频率
        """
        self.frame_count += 1

        # 确保三维力数据为numpy数组，便于后续计算/打印shape
        if hasattr(data, 'force_vectors') and data.force_vectors is not None and not hasattr(data.force_vectors, 'shape'):
            try:
                data.force_vectors = np.array(data.force_vectors)
            except Exception:
                pass
        
        # 存储数据到缓冲区
        self.sensor_data_buffer = data
        self._tactile_plot_dirty = True
        self._force_plot_dirty = True
        
        # 提取力数据
        if hasattr(data, 'force_data'):
            self.force_data_buffer = data
        elif hasattr(data, 'force_vectors'):
            self.force_data_buffer = data
        
        # 更新控制面板
        now = time.time()
        if (now - self._last_control_panel_data_ts) >= 0.2:
            self._last_control_panel_data_ts = now
            QTimer.singleShot(0, lambda: self.control_panel.update_data_display(data))
        
        # 更新数据包计数
        if (now - self._last_packet_label_ts) >= 0.2:
            self._last_packet_label_ts = now
            self.packet_label.setText(f"Packets: {self.frame_count}")
    
    def update_all_plots(self):
        """????????????????"""
        current_time = time.time()

        if current_time - self.last_gui_update_time < self.gui_update_interval:
            return

        self.last_gui_update_time = current_time

        try:
            if not self.sensor_data_buffer:
                return
            if self._is_simulation_mode() and (self.frame_count % 3 != 0):
                return
            if not self._tactile_plot_dirty and not self._force_plot_dirty:
                return

            current_tab = self.tab_widget.currentWidget() if hasattr(self, 'tab_widget') else None
            if hasattr(self, 'tactile_plot') and self.sensor_data_buffer and self._tactile_plot_dirty and current_tab is self.tactile_tab:
                self.tactile_plot.update_data(self.sensor_data_buffer)
                self.tactile_plot.update_plots()
                self._tactile_plot_dirty = False

            if hasattr(self, 'force_plot') and self.force_data_buffer and self._force_plot_dirty and current_tab is self.force_tab:
                self.force_plot.update_data(self.force_data_buffer)
                self.force_plot.update_plots()
                self._force_plot_dirty = False

        except Exception as e:
            print(f"???????: {e}")
            import traceback
            traceback.print_exc()
    def update_vector_visualization(self):
        """更新矢量可视化"""
        try:
            # 如果当前显示的是矢量图标签页，更新矢量图
            if (
                hasattr(self, "vector_plot")
                and self.vector_tab is not None
                and self.tab_widget.currentWidget() is self.vector_tab
            ):
                if self.force_vectors is not None and len(self.force_vectors) > 0:
                    self.vector_plot.update_data(self.force_vectors)
        except Exception as e:
            print(f"更新矢量可视化时出错: {e}")
    
    def update_3d_force_status(self):
        """更新三维力状态"""
        if self.force_vectors is not None:
            self.force_3d_status_label.setText("三维力: 活跃")
            self.force_3d_status_label.setStyleSheet("""
                QLabel { 
                    padding: 2px 10px; 
                    background-color: #ccffcc; 
                    border-radius: 3px;
                }
            """)
        else:
            self.force_3d_status_label.setText("三维力: 未激活")
            self.force_3d_status_label.setStyleSheet("""
                QLabel { 
                    padding: 2px 10px; 
                    background-color: #ffcccc; 
                    border-radius: 3px;
                }
            """)
    
    @pyqtSlot(str, dict)
    def update_control_status(self, status: str, info: dict):
        """
        更新控制状态
        
        Args:
            status: 控制状态
            info: 控制信息
        """
        # 更新控制面板
        self.control_panel.update_control_status(status, info)

        if status in ("stm32_connect_result", "stm32_disconnect_result"):
            connected = bool(info.get("connected", False))
            simulation = bool(info.get("simulation", False))
            if status.endswith("disconnect_result"):
                connected = False
                simulation = False
            self.control_panel.update_device_status(
                stm32={"connected": connected, "simulation": simulation}
            )
            if status == "stm32_connect_result" and not info.get("success", False):
                QMessageBox.warning(
                    self,
                    "STM32连接失败",
                    info.get("message", "STM32连接失败")
                )
            return

        if status in ("tactile_connect_result", "tactile_disconnect_result"):
            connected = bool(info.get("connected", False))
            simulation = bool(info.get("simulation", False))
            if status.endswith("disconnect_result"):
                connected = False
                simulation = False
            self.control_panel.update_device_status(
                tactile={"connected": connected, "simulation": simulation}
            )
            if status == "tactile_connect_result" and not info.get("success", False):
                QMessageBox.warning(
                    self,
                    "触觉传感器连接失败",
                    info.get("message", "触觉传感器连接失败")
                )
            return

        if status == "arm_connect_result":
            if not info.get("success", False):
                QMessageBox.warning(
                    self,
                    "机械臂连接失败",
                    info.get("message", "机械臂连接失败")
                )
            return

        # 更新机械臂状态
        if status == "arm_state":
            self._last_arm_state = info
            self.control_panel.update_device_status(
                arm={
                    "connected": info.get("connected", False),
                    "simulation": info.get("connection_type") == "simulation",
                }
            )
        if status == "arm_state" and self.arm_status_panel:
            self.arm_status_panel.update_arm_status(
                connected=info.get("connected", False),
                enabled=info.get("enabled", False),
                homed=info.get("homed", False),
                safety=info.get("safety", "normal"),
                control_mode=info.get("control_mode", "joint"),
                connection_type=info.get("connection_type", "hardware")
            )
            joint_angles = info.get("joint_angles")
            if joint_angles:
                self.arm_status_panel.update_joint_data(
                    joint_angles,
                    velocities=info.get("joint_velocities"),
                    torques=info.get("joint_torques"),
                    targets=info.get("joint_targets")
                )
            return
        
        # 更新按钮状态
        if status == "运行中":
            self.start_demo_btn.setEnabled(False)
            self.start_3d_demo_btn.setEnabled(False)
            self.stop_demo_btn.setEnabled(True)
        else:
            self.start_demo_btn.setEnabled(True)
            self.start_3d_demo_btn.setEnabled(True)
            self.stop_demo_btn.setEnabled(False)

    def _handle_joint_angle_request(self, joint_index: int, angle: float):
        """处理关节角度控制请求（连续拖动）"""
        self.control_signal.emit(
            "move_arm_joint",
            {"joint_index": joint_index, "angle": angle, "wait": False}
        )
    
    def update_time(self):
        """更新时间显示"""
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.setText(current_time)
    
    def update_status(self):
        """更新状态显示"""
        try:
            # 汇总硬件状态（STM32/触觉/机械臂/视觉）
            status = {}
            # 优先从硬件接口读取真实连接状态，避免演示状态覆盖
            if (
                hasattr(self.demo_manager, "hardware_interface")
                and self.demo_manager.hardware_interface is not None
                and hasattr(self.demo_manager.hardware_interface, "get_status")
            ):
                status = self.demo_manager.hardware_interface.get_status() or {}
            elif hasattr(self.demo_manager, "get_status"):
                status = self.demo_manager.get_status() or {}

            servo_status = status.get("servo", {})
            sensor_status = status.get("sensor", {})
            arm_status = status.get("arm", {})

            stm32_connected = bool(servo_status.get("connected", False))
            stm32_simulation = bool(servo_status.get("simulation", False))
            tactile_connected = bool(sensor_status.get("connected", False))
            tactile_simulation = bool(sensor_status.get("simulation", False))

            arm_info = self._last_arm_state if isinstance(self._last_arm_state, dict) else arm_status
            arm_connected = bool(arm_info.get("connected", False))
            arm_simulation = bool(arm_info.get("simulation", False)) or arm_info.get("connection_type") == "simulation"

            if self._is_ros2_vision_mode():
                vision_connected = bool(self._ros2_vision_connected)
                vision_simulation = bool(self._ros2_vision_simulation)
            else:
                vision_connected = bool(self.camera_capture and getattr(self.camera_capture, "is_capturing", False))
                vision_simulation = self._is_camera_simulation()

            self.control_panel.update_device_status(
                stm32={"connected": stm32_connected, "simulation": stm32_simulation},
                tactile={"connected": tactile_connected, "simulation": tactile_simulation},
                arm={"connected": arm_connected, "simulation": arm_simulation},
                vision={"connected": vision_connected, "simulation": vision_simulation},
            )

            # 低频提示未连接设备，避免日志刷屏导致卡顿
            now = time.time()
            if now - self._last_missing_log_time >= 3.0:
                missing = []
                if not stm32_connected:
                    missing.append("STM32未连接")
                if not vision_connected:
                    missing.append("视觉未连接")
                if not tactile_connected:
                    missing.append("触觉未连接")
                if not arm_connected:
                    missing.append("机械臂未连接")
                if missing:
                    self.logger.warning("硬件未连接状态: " + "，".join(missing))
                self._last_missing_log_time = now

            # 更新按钮状态：演示/校准仍依赖 STM32 + 触觉
            hardware_ready = stm32_connected and tactile_connected
            all_connected = hardware_ready and arm_connected and vision_connected
            any_connected = stm32_connected or tactile_connected or arm_connected or vision_connected

            self.connect_btn.setEnabled(not all_connected)
            self.disconnect_btn.setEnabled(any_connected)
            self.calibrate_btn.setEnabled(hardware_ready)
            self.calibrate_3d_btn.setEnabled(hardware_ready)
            self.start_demo_btn.setEnabled(hardware_ready)
            self.start_3d_demo_btn.setEnabled(hardware_ready)

        except Exception as e:
            self.logger.error(f"更新状态显示时出错: {e}")
    
    def new_config(self):
        """新建配置"""
        reply = QMessageBox.question(
            self, "新建配置",
            "是否创建新的配置？当前配置将丢失。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                # 重置为默认配置
                self.config = DemoConfig()
                self.logger.info("创建新配置")
                
                # 更新UI
                self.update_ui_from_config()
                
            except Exception as e:
                self.logger.error(f"新建配置失败: {e}")
                QMessageBox.critical(self, "错误", f"创建新配置时出错: {str(e)}")
    
    def open_config(self):
        """打开配置"""
        try:
            dialog = ConfigDialog(self.config, self)
            if dialog.exec_() == ConfigDialog.Accepted:
                new_config = dialog.get_config()
                self.config = new_config
                self.logger.info("加载新配置")
                
                # 更新UI
                self.update_ui_from_config()
                
        except Exception as e:
            self.logger.error(f"打开配置失败: {e}")
            QMessageBox.critical(self, "错误", f"打开配置时出错: {str(e)}")
    
    def save_config(self):
        """保存配置"""
        try:
            # 确保config目录存在
            config_dir = os.path.join(project_root, "config")
            os.makedirs(config_dir, exist_ok=True)
            
            config_path = os.path.join(config_dir, "current_config.yaml")
            self.config.save(config_path)
            self.logger.info("配置已保存")
            QMessageBox.information(self, "保存成功", f"配置已保存到 {config_path}")
            
        except Exception as e:
            self.logger.error(f"保存配置失败: {e}")
            QMessageBox.critical(self, "保存失败", f"保存配置时出错: {str(e)}")
    
    def save_config_as(self):
        """另存配置为"""
        try:
            file_path, _ = QFileDialog.getSaveFileName(
                self, "保存配置",
                os.path.join(project_root, "config", "custom_config.yaml"),
                "配置文件 (*.yaml *.yml *.json);;所有文件 (*)"
            )
            
            if file_path:
                if file_path.endswith('.json'):
                    self.config.save(file_path, format='json')
                else:
                    self.config.save(file_path, format='yaml')
                
                self.logger.info(f"配置已保存到: {file_path}")
                QMessageBox.information(self, "保存成功", f"配置已保存到: {file_path}")
                
        except Exception as e:
            self.logger.error(f"保存配置失败: {e}")
            QMessageBox.critical(self, "保存失败", f"保存配置时出错: {str(e)}")
    
    def export_3d_data(self):
        """导出三维力数据"""
        try:
            file_path, _ = QFileDialog.getSaveFileName(
                self, "导出三维力数据",
                os.path.join(project_root, f"data/force_3d_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"),
                "JSON文件 (*.json);;CSV文件 (*.csv);;所有文件 (*)"
            )
            
            if file_path:
                # 准备三维力数据
                force_stats = getattr(self, "force_statistics", None)
                export_data = {
                    "metadata": {
                        "export_time": datetime.now().isoformat(),
                        "system_config": str(self.config),
                        "force_dimensions": 3
                    },
                    "force_data": self.force_vectors if self.force_vectors else {},
                    "force_statistics": force_stats if force_stats else {},
                    "contact_data": self.contact_map.tolist() if isinstance(self.contact_map, np.ndarray) else []
                }
                
                # 保存到文件
                if file_path.endswith('.csv'):
                    # 转换为CSV格式
                    import pandas as pd
                    # 这里需要根据实际数据结构转换为CSV
                    QMessageBox.warning(self, "导出警告", "CSV导出功能正在开发中，暂时导出为JSON格式")
                    file_path = file_path.replace('.csv', '.json')
                
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(export_data, f, indent=2, ensure_ascii=False)
                
                self.logger.info(f"三维力数据已导出到: {file_path}")
                QMessageBox.information(self, "导出成功", f"三维力数据已导出到: {file_path}")
                
        except Exception as e:
            self.logger.error(f"导出三维力数据失败: {e}")
            QMessageBox.critical(self, "导出失败", f"导出三维力数据时出错: {str(e)}")
    
    def export_data(self):
        """导出数据"""
        try:
            file_path, _ = QFileDialog.getSaveFileName(
                self, "导出数据",
                os.path.join(project_root, f"data/export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"),
                "CSV文件 (*.csv);;JSON文件 (*.json);;所有文件 (*)"
            )
            
            if file_path:
                # 导出数据
                if hasattr(self.demo_manager, 'export_data'):
                    self.demo_manager.export_data(file_path)
                    self.logger.info(f"数据已导出到: {file_path}")
                    QMessageBox.information(self, "导出成功", f"数据已导出到: {file_path}")
                else:
                    QMessageBox.warning(self, "导出失败", "演示管理器不支持数据导出功能")
                    
        except Exception as e:
            self.logger.error(f"导出数据失败: {e}")
            QMessageBox.critical(self, "导出失败", f"导出数据时出错: {str(e)}")
    
    def connect_hardware(self):
        """连接硬件"""
        self.logger.info("连接硬件...")
        
        # 发送连接命令
        self.control_signal.emit("connect_hardware", {})
        
        # 更新状态
        self.status_label.setText("系统状态: 连接中...")
        self.status_label.setStyleSheet("""
            QLabel { 
                padding: 2px 10px; 
                background-color: #ffffcc; 
                border-radius: 3px;
            }
        """)
    
    def disconnect_hardware(self):
        """断开硬件"""
        reply = QMessageBox.question(
            self, "断开硬件",
            "是否断开硬件连接？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.logger.info("断开硬件...")
            
            # 发送断开命令
            self.control_signal.emit("disconnect_hardware", {})
            
            # 更新状态
            self.status_label.setText("系统状态: 断开中...")
            self.status_label.setStyleSheet("""
                QLabel { 
                    padding: 2px 10px; 
                    background-color: #ffffcc; 
                    border-radius: 3px;
                }
            """)
    
    def calibrate_3d(self):
        """三维力校准"""
        try:
            reply = QMessageBox.question(
                self, "三维力校准",
                "开始三维力校准吗？\n请确保传感器处于自由状态。",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                # 发送三维力校准命令
                self.control_signal.emit("calibrate_3d", {
                    "type": "vector",
                    "auto": True
                })
                
                QMessageBox.information(self, "校准开始", "三维力校准已开始，请等待完成...")
        except Exception as e:
            self.logger.error(f"三维力校准失败: {e}")
            QMessageBox.critical(self, "错误", f"三维力校准失败: {str(e)}")
    
    def calibrate(self):
        """校准硬件"""
        try:
            dialog = CalibrationDialog(self.demo_manager, self.config, self)
            dialog.exec_()
        except Exception as e:
            self.logger.error(f"校准对话框打开失败: {e}")
            QMessageBox.critical(self, "错误", f"校准功能暂不可用: {str(e)}")
    
    def start_3d_demo(self):
        """启动三维力演示"""
        try:
            # 直接启动矢量可视化演示
            demo_name = "vector_visualization"
            params = {
                "duration": 15.0,
                "type": "vector_field"
            }
            
            self.logger.info(f"启动三维力演示: {demo_name} 参数: {params}")
            
            # 发送启动命令
            self.control_signal.emit("start_demo", {
                "demo_name": demo_name,
                "params": params
            })
            
            # 切换到矢量图标签页
            self.tab_widget.setCurrentIndex(1)
            
            # 更新状态
            self.status_label.setText(f"系统状态: 运行 {demo_name}...")
            self.status_label.setStyleSheet("""
                QLabel { 
                    padding: 2px 10px; 
                    background-color: #ffffcc; 
                    border-radius: 3px;
                }
            """)
            
        except Exception as e:
            self.logger.error(f"启动三维力演示失败: {e}")
            QMessageBox.critical(self, "错误", f"启动三维力演示时出错: {str(e)}")
    
    def start_demo(self):
        """启动演示"""
        try:
            dialog = DemoSelectionDialog(self.config, self)
            if dialog.exec_() == DemoSelectionDialog.Accepted:
                demo_name, params = dialog.get_selection()
                
                self.logger.info(f"启动演示: {demo_name} 参数: {params}")
                
                # 发送启动命令
                self.control_signal.emit("start_demo", {
                    "demo_name": demo_name,
                    "params": params
                })
                
                # 更新状态
                self.status_label.setText(f"系统状态: 运行 {demo_name}...")
                self.status_label.setStyleSheet("""
                    QLabel { 
                        padding: 2px 10px; 
                        background-color: #ffffcc; 
                        border-radius: 3px;
                    }
                """)
                
        except Exception as e:
            self.logger.error(f"启动演示失败: {e}")
            QMessageBox.critical(self, "错误", f"启动演示时出错: {str(e)}")
    
    def stop_demo(self):
        """停止演示"""
        self.logger.info("停止演示...")
        
        # 发送停止命令
        self.control_signal.emit("stop_demo", {})
        
        # 更新状态
        self.status_label.setText("系统状态: 停止中...")
        self.status_label.setStyleSheet("""
            QLabel { 
                padding: 2px 10px; 
                background-color: #ffffcc; 
                border-radius: 3px;
            }
        """)
    
    def emergency_stop(self):
        """紧急停止"""
        reply = QMessageBox.warning(
            self, "紧急停止",
            "是否执行紧急停止？\n系统将立即停止所有运动。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.toggle_emergency(True)

    def toggle_emergency(self, checked: bool):
        """紧急停止/继续 切换"""
        if checked:
            self.logger.warning("执行紧急停止")
            self.emergency_btn.setText("继续")
            self.emergency_btn.setStyleSheet("background-color: #ffcc00; color: black; font-weight: bold;")
            try:
                self.control_signal.emit("emergency_stop", {})
            except Exception as e:
                self.logger.error(f"紧急停止失败: {e}")
        else:
            self.logger.info("退出紧急停止，尝试恢复系统")
            self.emergency_btn.setText("紧急停止")
            self.emergency_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
            try:
                # 尝试重启采集并重连硬件
                if hasattr(self.data_acquisition_thread, "start_acquisition"):
                    self.data_acquisition_thread.start_acquisition()
                self.control_signal.emit("connect_hardware", {})
            except Exception as e:
                self.logger.error(f"恢复失败: {e}")

    def show_tactile_mapping(self, mapping_data):
        """在主线程显示触觉映射结果"""
        if not mapping_data:
            return
        latest = mapping_data[-1]
        fv = np.array(latest.get('force_vectors', []))
        if fv.size == 0:
            return
        magnitudes = np.linalg.norm(fv, axis=1)
        taxel_pos = self._get_taxel_positions()
        coords = (taxel_pos - 0.5) * 10.0  # 映射到 [-10,10]mm

        grid_lin = np.linspace(-10, 10, 60)
        grid_x, grid_y = np.meshgrid(grid_lin, grid_lin)
        grid_z = self._interpolate_grid(coords, magnitudes, grid_x, grid_y)

        fig = plt.figure(figsize=(12, 5))
        # 2D
        ax1 = fig.add_subplot(1, 2, 1)
        im = ax1.imshow(grid_z, extent=[-10, 10, -10, 10], origin='lower', cmap='viridis')
        ax1.scatter(coords[:, 0], coords[:, 1], c='red', s=50, label='触点')
        ax1.set_title('触觉映射 - 2D视图')
        ax1.set_xlabel('X 位置 (mm)')
        ax1.set_ylabel('Y 位置 (mm)')
        ax1.legend(loc='upper right')
        fig.colorbar(im, ax=ax1, label='力 (N)')

        # 3D
        ax2 = fig.add_subplot(1, 2, 2, projection='3d')
        ax2.plot_surface(grid_x, grid_y, grid_z, cmap='viridis', linewidth=0, antialiased=True, alpha=0.8)
        ax2.scatter(coords[:, 0], coords[:, 1], magnitudes, c='red', s=40)
        ax2.set_title('触觉映射 - 3D视图')
        ax2.set_xlabel('X 位置 (mm)')
        ax2.set_ylabel('Y 位置 (mm)')
        ax2.set_zlabel('力 (N)')
        fig.colorbar(plt.cm.ScalarMappable(cmap='viridis'), ax=ax2, fraction=0.046, pad=0.04, label='力 (N)')

        plt.tight_layout()
        plt.show(block=False)

    def _get_taxel_positions(self):
        """获取触点位置归一化坐标 (3x3)"""
        positions = []
        rows, cols = 3, 3
        for i in range(rows):
            for j in range(cols):
                x = j / (cols - 1) if cols > 1 else 0.5
                y = i / (rows - 1) if rows > 1 else 0.5
                positions.append([x, y])
        return np.array(positions)

    def _interpolate_grid(self, coords: np.ndarray, values: np.ndarray, grid_x: np.ndarray, grid_y: np.ndarray) -> np.ndarray:
        """简单反距离插值，避免依赖scipy"""
        eps = 1e-6
        grid_z = np.zeros_like(grid_x, dtype=float)
        for i in range(grid_x.shape[0]):
            dx = coords[:, 0][:, None] - grid_x[i, :][None, :]
            dy = coords[:, 1][:, None] - grid_y[i, :][None, :]
            dist = np.sqrt(dx * dx + dy * dy) + eps
            w = 1.0 / dist
            grid_z[i, :] = np.sum(w * values[:, None], axis=0) / np.sum(w, axis=0)
        return grid_z
    
    def toggle_toolbar(self, checked: bool):
        """显示/隐藏工具栏"""
        if hasattr(self, "toolbar_main") and self.toolbar_main is not None:
            self.toolbar_main.setVisible(checked)
    
    def toggle_statusbar(self, checked: bool):
        """显示/隐藏状态栏"""
        self.statusBar().setVisible(checked)
    
    def toggle_vector_toolbar(self, checked: bool):
        """显示/隐藏矢量工具栏"""
        if hasattr(self, "viz_toolbar") and self.viz_toolbar is not None:
            self.viz_toolbar.setVisible(checked)
    
    def change_3d_view(self, view_type: str):
        """更改三维视图"""
        try:
            if hasattr(self, 'vector_plot'):
                self.vector_plot.change_view(view_type)
                self.logger.info(f"切换三维视图: {view_type}")
        except Exception as e:
            self.logger.error(f"切换三维视图失败: {e}")
    
    def change_visualization_mode(self, mode_text: str):
        """更改可视化模式"""
        mode_map = {
            "热力图": "heatmap",
            "矢量图": "vector_field",
            "3D视图": "3d_view",
            "混合视图": "hybrid"
        }
        
        self.visualization_mode = mode_map.get(mode_text, "heatmap")
        
        # 更新绘图部件
        if hasattr(self, 'tactile_plot'):
            self.tactile_plot.set_visualization_mode(self.visualization_mode)
        
        if hasattr(self, 'vector_plot'):
            self.vector_plot.set_visualization_mode(self.visualization_mode)
        
        self.logger.info(f"切换可视化模式: {self.visualization_mode}")
    
    def update_vector_scale(self, scale: float):
        """更新矢量缩放"""
        self.vector_scale = scale
        
        # 更新矢量图部件
        if hasattr(self, 'vector_plot'):
            self.vector_plot.set_vector_scale(scale)
        
        self.logger.info(f"更新矢量缩放: {scale}")
    
    def toggle_force_labels(self, state: int):
        """切换力标签显示"""
        self.show_force_labels = (state == Qt.Checked)
        
        # 更新矢量图部件
        if hasattr(self, 'vector_plot'):
            self.vector_plot.set_show_labels(self.show_force_labels)
        
        self.logger.info(f"切换力标签显示: {self.show_force_labels}")
    
    def toggle_grid(self, state: int):
        """切换网格显示"""
        show_grid = (state == Qt.Checked)
        
        # 更新绘图部件
        if hasattr(self, 'tactile_plot'):
            self.tactile_plot.set_show_grid(show_grid)
        
        if hasattr(self, 'vector_plot'):
            self.vector_plot.set_show_grid(show_grid)
        
        self.logger.info(f"切换网格显示: {show_grid}")
    
    def refresh_visualization(self):
        """刷新可视化"""
        try:
            # 强制更新所有绘图
            self.update_all_plots()
            self.update_vector_visualization()
            
            # 重置数据
            if hasattr(self, 'force_plot'):
                self.force_plot.reset()
            
            self.logger.info("可视化已刷新")
            
        except Exception as e:
            self.logger.error(f"刷新可视化失败: {e}")
    
    def toggle_vector_view(self, checked: bool):
        """切换矢量视图"""
        if checked:
            # 切换到矢量图标签页
            self.tab_widget.setCurrentIndex(1)
            self.vector_view_btn.setText("矢量视图 ✓")
        else:
            self.vector_view_btn.setText("矢量视图")
    
    def change_theme(self, theme_name: str):
        """更改主题"""
        try:
            # 记录主题切换（控制台可见）
            print(f"[UI] 切换主题 -> {theme_name}")
            app = QApplication.instance()
            if theme_name == "dark":
                # 深色主题
                dark_style = """
                    QMainWindow, QWidget {
                        background-color: #2b2b2b;
                        color: #ffffff;
                    }
                    QLabel {
                        color: #ffffff;
                    }
                    QPushButton {
                        background-color: #3c3c3c;
                        color: #ffffff;
                        border: 1px solid #555555;
                        padding: 5px;
                        border-radius: 3px;
                    }
                    QPushButton:hover {
                        background-color: #4c4c4c;
                    }
                    QPushButton:pressed {
                        background-color: #2c2c2c;
                    }
                    QPushButton:checked {
                        background-color: #0066cc;
                    }
                    QTabWidget::pane {
                        border: 1px solid #555555;
                        background-color: #2b2b2b;
                    }
                    QTabBar::tab {
                        background-color: #3c3c3c;
                        color: #ffffff;
                        padding: 5px 10px;
                    }
                    QTabBar::tab:selected {
                        background-color: #5c5c5c;
                    }
                    QComboBox, QSpinBox, QDoubleSpinBox {
                        background-color: #3c3c3c;
                        color: #ffffff;
                        border: 1px solid #555555;
                        padding: 3px;
                    }
                    QCheckBox {
                        color: #ffffff;
                    }
                """
                self.setStyleSheet(dark_style)
                if app:
                    app.setStyleSheet(dark_style)
            elif theme_name == "tech":
                # 科技主题
                tech_style = """
                    QMainWindow, QWidget {
                        background-color: #0a192f;
                        color: #64ffda;
                    }
                    QLabel {
                        color: #ccd6f6;
                    }
                    QPushButton {
                        background-color: #112240;
                        color: #64ffda;
                        border: 1px solid #233554;
                        padding: 5px;
                        border-radius: 3px;
                    }
                    QPushButton:hover {
                        background-color: #1d3a5f;
                    }
                    QPushButton:pressed {
                        background-color: #0c1d36;
                    }
                    QPushButton:checked {
                        background-color: #0066cc;
                    }
                    QTabWidget::pane {
                        border: 1px solid #233554;
                        background-color: #0a192f;
                    }
                    QTabBar::tab {
                        background-color: #112240;
                        color: #8892b0;
                        padding: 5px 10px;
                    }
                    QTabBar::tab:selected {
                        background-color: #1d3a5f;
                        color: #64ffda;
                    }
                """
                self.setStyleSheet(tech_style)
                if app:
                    app.setStyleSheet(tech_style)
            else:
                # 浅色主题
                light_style = """
                    QMainWindow, QWidget {
                        background-color: #f0f0f0;
                        color: #000000;
                    }
                    QLabel {
                        color: #000000;
                    }
                    QPushButton {
                        background-color: #e0e0e0;
                        color: #000000;
                        border: 1px solid #cccccc;
                        padding: 5px;
                        border-radius: 3px;
                    }
                    QPushButton:hover {
                        background-color: #d0d0d0;
                    }
                    QPushButton:pressed {
                        background-color: #c0c0c0;
                    }
                    QPushButton:checked {
                        background-color: #0066cc;
                        color: white;
                    }
                """
                self.setStyleSheet(light_style)
                if app:
                    app.setStyleSheet(light_style)
            
            # 更新配置
            if hasattr(self.config.ui, 'theme'):
                self.config.ui.theme = theme_name
            
            self.logger.info(f"切换主题: {theme_name}")
            
        except Exception as e:
            self.logger.error(f"切换主题失败: {e}")
    
    def open_force_analysis(self):
        """打开三维力分析工具"""
        QMessageBox.information(self, "三维力分析", "三维力分析工具正在开发中")
    
    def open_vector_analysis(self):
        """打开矢量场分析工具"""
        QMessageBox.information(self, "矢量场分析", "矢量场分析工具正在开发中")
    
    def open_config_editor(self):
        """打开配置编辑器"""
        try:
            dialog = ConfigDialog(self.config, self)
            dialog.exec_()
        except Exception as e:
            self.logger.error(f"打开配置编辑器失败: {e}")
            QMessageBox.critical(self, "错误", f"配置编辑器打开失败: {str(e)}")
    
    def open_log_viewer(self):
        """打开日志查看器"""
        try:
            dialog = LogViewerDialog(self)
            dialog.exec_()
        except Exception as e:
            self.logger.error(f"打开日志查看器失败: {e}")
            QMessageBox.critical(self, "错误", f"日志查看器打开失败: {str(e)}")
    
    def open_system_monitor(self):
        """打开系统监控"""
        QMessageBox.information(self, "系统监控", "系统监控功能正在开发中")
    
    def open_force_manual(self):
        """打开三维力使用手册"""
        QMessageBox.information(self, "三维力使用手册", "三维力使用手册功能正在开发中")
    
    def open_user_manual(self):
        """打开用户手册"""
        QMessageBox.information(self, "用户手册", "用户手册功能正在开发中")
    
    def show_about(self):
        """显示关于对话框"""
        try:
            dialog = AboutDialog(self)
            dialog.exec_()
        except Exception as e:
            self.logger.error(f"显示关于对话框失败: {e}")
            QMessageBox.information(self, "关于", 
                "触觉夹爪控制系统 v2.0.0\n"
                "三维力与矢量可视化版本\n"
                "作者: Tactile Gripper Team\n"
                "支持: Fx, Fy, Fz 三维力数据\n"
                "功能: 矢量图、热力图、3D视图")
    
    def update_ui_from_config(self):
        """根据配置更新UI"""
        try:
            # 更新窗口大小
            self.resize(self.config.ui.window_width, self.config.ui.window_height)
            
            # 更新绘图部件的配置
            if self.tactile_plot:
                self.tactile_plot.set_config(self.config)
            if self.force_plot:
                self.force_plot.set_config(self.config)
            if self.vector_plot:
                self.vector_plot.set_config(self.config)
            if self.vision_viewer:
                camera_config = self._get_camera_config()
                if camera_config is not None:
                    self.vision_viewer.config = camera_config
            if self.simulation_viewer:
                simulation_config = self._get_simulation_config()
                if simulation_config is not None:
                    self.simulation_viewer.config = simulation_config
            if self.arm_status_panel:
                arm_config = self._get_arm_config()
                if arm_config is not None:
                    self.arm_status_panel.config = arm_config
            
            # 更新控制面板
            self.control_panel.set_config(self.config)
            
            # 应用主题
            if hasattr(self.config.ui, 'theme'):
                self.change_theme(self.config.ui.theme)
                
        except Exception as e:
            self.logger.error(f"更新UI配置失败: {e}")
    
    def closeEvent(self, event):
        """??????"""
        reply = QMessageBox.question(
            self, "????",
            "?????????????",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.logger.info("????")
            self.request_shutdown.emit()
            try:
                self._handle_disconnect_camera()
            except Exception:
                pass

            self.time_timer.stop()
            self.status_timer.stop()
            self.data_update_timer.stop()
            self.vector_update_timer.stop()
            self.rate_timer.stop()
            self.force_3d_timer.stop()
            if self.camera_timer:
                self.camera_timer.stop()
            if hasattr(self, "ros2_vision_timer") and self.ros2_vision_timer:
                self.ros2_vision_timer.stop()
            if hasattr(self, "vision_worker_timer") and self.vision_worker_timer:
                self.vision_worker_timer.stop()
            if hasattr(self, "vision_self_check_timer") and self.vision_self_check_timer:
                self.vision_self_check_timer.stop()
            try:
                self._analysis_worker.stop()
            except Exception:
                pass
            try:
                self._depth_visual_worker.stop()
            except Exception:
                pass
            try:
                self._pointcloud_worker.stop()
            except Exception:
                pass

            event.accept()
        else:
            event.ignore()

if __name__ == "__main__":
    # 测试主窗口
    from PyQt5.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    # 创建测试配置
    config = DemoConfig()
    
    # 创建测试对象
    demo_manager = None
    data_acquisition = None
    control_thread = None
    
    # 创建主窗口
    window = MainWindow(demo_manager, data_acquisition, control_thread, config)
    window.show()
    
    sys.exit(app.exec_())
