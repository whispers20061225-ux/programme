"""







触觉夹爪演示系统 - 控制面板







提供系统的控制界面，包括硬件控制、演示控制和参数设置。







"""















import logging







import os







import re







import subprocess







import sys







from typing import Dict, Any, Optional















from PyQt5.QtWidgets import (







    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QGridLayout,







    QLabel, QPushButton, QSlider, QSpinBox, QDoubleSpinBox,







    QComboBox, QCheckBox, QLineEdit, QFormLayout,







    QScrollArea, QFrame, QProgressBar, QTextEdit







)







from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QPointF, QTimer







from PyQt5.QtGui import QFont, QPalette, QColor















# 修改导入方式 - 使用绝对导入







try:







    # 方案1：先尝试绝对导入







    from config import DemoConfig







except ImportError:







    try:







        # 方案2：尝试相对导入







        from ..config import DemoConfig







    except ImportError:







        try:







            # 方案3：直接导入







            import sys







            import os







            # 添加父目录到路径







            sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))







            from config import DemoConfig







        except ImportError as e:







            raise ImportError(f"无法导入 DemoConfig: {e}")















# 导入 DemoManager - 同样使用绝对导入







try:







    from core.demo_manager import DemoManager







except ImportError:







    try:







        from ..core.demo_manager import DemoManager







    except ImportError:







        import sys







        import os







        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))







        from core.demo_manager import DemoManager























class ControlPanel(QWidget):







    """控制面板类"""







    







    # 自定义信号







    control_signal = pyqtSignal(str, dict)  # (command, parameters)







    vision_connect_request = pyqtSignal()







    vision_disconnect_request = pyqtSignal()







    request_shutdown = pyqtSignal()







    







    def __init__(self, demo_manager: DemoManager, config: DemoConfig):







        """







        初始化控制面板







        







        Args:







            demo_manager: 演示管理器







            config: 系统配置







        """







        super().__init__()







        







        # 保存引用







        self.demo_manager = demo_manager







        self.config = config
        # 初始化UI
        self.init_ui()
        # 初始化 STM32 / 触觉 连接参数

        self.stm32_port_input.setText(str(getattr(config.hardware.servo, 'port', '')))

        self.stm32_baud_spin.setValue(int(getattr(config.hardware.servo, 'baudrate', 115200)))

        self.stm32_timeout_spin.setValue(float(getattr(config.hardware.servo, 'timeout', 1.0)))



        self.tactile_port_input.setText(str(getattr(config.hardware.sensor, 'port', '')))

        self.tactile_baud_spin.setValue(int(getattr(config.hardware.sensor, 'baudrate', 115200)))

        self.tactile_timeout_spin.setValue(float(getattr(config.hardware.sensor, 'timeout', 0.1)))



        # 初始化机械臂连接参数

        arm_port = ""

        arm_baud = 115200

        arm_timeout = 1.0

        if hasattr(config, 'learm_arm') and config.learm_arm is not None:

            if isinstance(config.learm_arm, dict):

                conn_cfg = config.learm_arm.get('CONNECTION', config.learm_arm)

                arm_port = conn_cfg.get('serial_port', arm_port)

                arm_baud = conn_cfg.get('baud_rate', arm_baud)

                arm_timeout = conn_cfg.get('timeout', arm_timeout)

            else:

                conn_cfg = getattr(config.learm_arm, 'CONNECTION', {}) or {}

                arm_port = conn_cfg.get('serial_port', arm_port)

                arm_baud = conn_cfg.get('baud_rate', arm_baud)

                arm_timeout = conn_cfg.get('timeout', arm_timeout)

        self.arm_port_input.setText(str(arm_port))

        self.arm_baud_spin.setValue(int(arm_baud))

        self.arm_timeout_spin.setValue(float(arm_timeout))



        # 初始化相机参数

        camera_type = 'opencv'

        camera_id = 0

        cam_width = 640

        cam_height = 480

        cam_fps = 30

        camera_cfg = getattr(config, 'camera', None)

        if isinstance(camera_cfg, dict):

            primary = camera_cfg.get('HARDWARE', {}).get('primary_camera', {})

            camera_type = primary.get('type', camera_type)

            camera_id = primary.get('camera_id', camera_id)

            resolution = primary.get('resolution', [cam_width, cam_height])

            if isinstance(resolution, (list, tuple)) and len(resolution) >= 2:

                cam_width, cam_height = int(resolution[0]), int(resolution[1])

            cam_fps = int(primary.get('fps', cam_fps))

        else:

            if hasattr(camera_cfg, 'camera_type'):

                camera_type = camera_cfg.camera_type

            if hasattr(camera_cfg, 'camera_index'):

                camera_id = camera_cfg.camera_index

            if hasattr(camera_cfg, 'width'):

                cam_width = camera_cfg.width

            if hasattr(camera_cfg, 'height'):

                cam_height = camera_cfg.height

            if hasattr(camera_cfg, 'fps'):

                cam_fps = camera_cfg.fps

        if camera_type in ["opencv", "realsense", "simulation"]:

            self.camera_type_combo.setCurrentText(camera_type)

        self.camera_id_input.setText(str(camera_id))

        self.camera_width_spin.setValue(int(cam_width))

        self.camera_height_spin.setValue(int(cam_height))

        self.camera_fps_spin.setValue(int(cam_fps))








        self.current_demo_status = "idle"







        self.pybullet_process = None







        self._pybullet_log_path = os.path.abspath(







            os.path.join(os.path.dirname(__file__), "..", "logs", "pybullet_standalone.log")







        )







        self._pybullet_log_fp = None







        







        # 初始化UI














        







        # 设置日志







        self.logger = logging.getLogger(__name__)
        self._device_status_cache = {}







        







        self.logger.info("控制面板初始化完成")







    







    def init_ui(self):







        """初始化用户界面"""







        # 创建滚动区域







        scroll_area = QScrollArea()







        scroll_area.setWidgetResizable(True)







        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)







        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)







        







        # 创建滚动区域的内容部件







        content_widget = QWidget()







        main_layout = QVBoxLayout(content_widget)







        main_layout.setContentsMargins(10, 10, 10, 10)







        main_layout.setSpacing(15)







        







        # 添加系统状态组







        self.create_system_status_group(main_layout)







        







        # 添加硬件控制组







        self.create_hardware_control_group(main_layout)







        







        # 添加演示控制组







        self.create_demo_control_group(main_layout)







        







        # 添加参数设置组







        self.create_parameter_group(main_layout)







        







        # 添加数据监控组







        self.create_data_monitor_group(main_layout)







        







        # 添加系统控制组







        self.create_system_control_group(main_layout)







        







        # 设置滚动区域的内容部件







        scroll_area.setWidget(content_widget)







        







        # 设置主布局







        layout = QVBoxLayout(self)







        layout.addWidget(scroll_area)







        layout.setContentsMargins(0, 0, 0, 0)







    







    def create_system_status_group(self, layout):




        """创建系统状态组"""




        group = QGroupBox("系统状态")




        group_layout = QVBoxLayout(group)









        # 状态提示




        self.system_status_label = QLabel("状态: 就绪")




        self.system_status_label.setFont(QFont("", 10, QFont.Bold))




        self.system_status_label.setStyleSheet("""




            QLabel {




                padding: 8px;




                background-color: #e8f5e9;




                border-radius: 5px;




                border: 1px solid #c8e6c9;




            }




        """)




        group_layout.addWidget(self.system_status_label)









        # 硬件状态：STM32 / 视觉 / 机械臂 / 触觉




        status_layout = QGridLayout()




        status_layout.setColumnStretch(1, 1)




        status_layout.setHorizontalSpacing(6)









        # STM32




        self.stm32_status_label = QLabel("STM32: 未连接")




        self._apply_status_style(self.stm32_status_label, connected=False, simulation=False)




        self.stm32_connect_btn = QPushButton("连接")




        self.stm32_disconnect_btn = QPushButton("断开")




        self.stm32_disconnect_btn.setEnabled(False)




        self.stm32_connect_btn.clicked.connect(self.connect_stm32)




        self.stm32_disconnect_btn.clicked.connect(self.disconnect_stm32)




        status_layout.addWidget(QLabel("STM32"), 0, 0)




        status_layout.addWidget(self.stm32_status_label, 0, 1)




        status_layout.addWidget(self.stm32_connect_btn, 0, 2)




        status_layout.addWidget(self.stm32_disconnect_btn, 0, 3)









        # 视觉




        self.vision_status_label = QLabel("视觉: 未连接")




        self._apply_status_style(self.vision_status_label, connected=False, simulation=False)




        self.vision_connect_btn = QPushButton("连接")




        self.vision_disconnect_btn = QPushButton("断开")




        self.vision_disconnect_btn.setEnabled(False)




        self.vision_connect_btn.clicked.connect(self.connect_vision)




        self.vision_disconnect_btn.clicked.connect(self.disconnect_vision)




        status_layout.addWidget(QLabel("视觉"), 1, 0)




        status_layout.addWidget(self.vision_status_label, 1, 1)




        status_layout.addWidget(self.vision_connect_btn, 1, 2)




        status_layout.addWidget(self.vision_disconnect_btn, 1, 3)









        # 机械臂




        self.arm_status_label = QLabel("机械臂: 未连接")




        self._apply_status_style(self.arm_status_label, connected=False, simulation=False)




        self.arm_connect_btn = QPushButton("连接")




        self.arm_disconnect_btn = QPushButton("断开")




        self.arm_disconnect_btn.setEnabled(False)




        self.arm_connect_btn.clicked.connect(self.connect_arm)




        self.arm_disconnect_btn.clicked.connect(self.disconnect_arm)




        status_layout.addWidget(QLabel("机械臂"), 2, 0)




        status_layout.addWidget(self.arm_status_label, 2, 1)




        status_layout.addWidget(self.arm_connect_btn, 2, 2)




        status_layout.addWidget(self.arm_disconnect_btn, 2, 3)









        # 触觉




        self.tactile_status_label = QLabel("触觉: 未连接")




        self._apply_status_style(self.tactile_status_label, connected=False, simulation=False)




        self.tactile_connect_btn = QPushButton("连接")




        self.tactile_disconnect_btn = QPushButton("断开")




        self.tactile_disconnect_btn.setEnabled(False)




        self.tactile_connect_btn.clicked.connect(self.connect_tactile)




        self.tactile_disconnect_btn.clicked.connect(self.disconnect_tactile)




        status_layout.addWidget(QLabel("触觉"), 3, 0)




        status_layout.addWidget(self.tactile_status_label, 3, 1)




        status_layout.addWidget(self.tactile_connect_btn, 3, 2)




        status_layout.addWidget(self.tactile_disconnect_btn, 3, 3)









        group_layout.addLayout(status_layout)









        # 数据统计




        data_layout = QFormLayout()




        self.data_rate_label = QLabel("0")




        data_layout.addRow("数据率 (Hz):", self.data_rate_label)









        self.buffer_size_label = QLabel("0")




        data_layout.addRow("缓冲区大小:", self.buffer_size_label)









        self.packet_count_label = QLabel("0")




        data_layout.addRow("数据包数:", self.packet_count_label)









        group_layout.addLayout(data_layout)









        layout.addWidget(group)




    def create_hardware_control_group(self, layout):







        """创建硬件控制组"""







        group = QGroupBox("硬件控制")







        group_layout = QVBoxLayout(group)







        







        # 连接/断开按钮







        btn_layout = QHBoxLayout()







        







        self.connect_btn = QPushButton("连接硬件")







        self.connect_btn.clicked.connect(self.connect_hardware)







        self.connect_btn.setStyleSheet("""







            QPushButton {







                background-color: #4CAF50;







                color: white;







                font-weight: bold;







                padding: 8px;







                border-radius: 4px;







            }







            QPushButton:hover {







                background-color: #45a049;







            }







        """)







        btn_layout.addWidget(self.connect_btn)







        







        self.disconnect_btn = QPushButton("断开硬件")







        self.disconnect_btn.clicked.connect(self.disconnect_hardware)







        self.disconnect_btn.setEnabled(False)







        self.disconnect_btn.setStyleSheet("""







            QPushButton {







                background-color: #f44336;







                color: white;







                font-weight: bold;







                padding: 8px;







                border-radius: 4px;







            }







            QPushButton:hover {







                background-color: #d32f2f;







            }







            QPushButton:disabled {







                background-color: #cccccc;







            }







        """)







        btn_layout.addWidget(self.disconnect_btn)







        







        group_layout.addLayout(btn_layout)















        # 快捷预设按钮







        preset_layout = QHBoxLayout()







        self.preset_vector_btn = QPushButton("矢量演示")







        self.preset_vector_btn.setEnabled(False)







        self.preset_vector_btn.clicked.connect(lambda: self.start_preset_demo("vector_visualization", {"duration": 15}))







        preset_layout.addWidget(self.preset_vector_btn)















        self.preset_mapping_btn = QPushButton("触觉映射")







        self.preset_mapping_btn.setEnabled(False)







        self.preset_mapping_btn.clicked.connect(lambda: self.start_preset_demo("tactile_mapping", {"duration": 10, "save": True}))







        preset_layout.addWidget(self.preset_mapping_btn)















        preset_layout.addStretch()







        group_layout.addLayout(preset_layout)







        







        # 校准按钮







        self.calibrate_btn = QPushButton("校准硬件")







        self.calibrate_btn.clicked.connect(self.calibrate_hardware)







        self.calibrate_btn.setEnabled(False)







        group_layout.addWidget(self.calibrate_btn)







        







        # 传感器控制







        sensor_group = QGroupBox("传感器设置")







        sensor_layout = QFormLayout(sensor_group)







        







        self.sensor_enable_check = QCheckBox("启用传感器")







        self.sensor_enable_check.setChecked(True)







        sensor_layout.addRow(self.sensor_enable_check)







        







        self.sensor_rate_combo = QComboBox()







        self.sensor_rate_combo.addItems(["10", "50", "100", "200"])







        self.sensor_rate_combo.setCurrentText(str(self.config.hardware.sensor.sampling_rate))







        sensor_layout.addRow("采样率 (Hz):", self.sensor_rate_combo)







        







        group_layout.addWidget(sensor_group)







        







        # 舵机控制







        servo_group = QGroupBox("舵机控制")







        servo_layout = QVBoxLayout(servo_group)







        







        # 位置控制







        pos_layout = QHBoxLayout()







        pos_layout.addWidget(QLabel("位置:"))







        







        self.position_slider = QSlider(Qt.Horizontal)







        self.position_slider.setRange(0, 100)







        self.position_slider.setValue(50)







        self.position_slider.setEnabled(False)







        pos_layout.addWidget(self.position_slider)







        







        self.position_value = QLabel("50")







        self.position_value.setFixedWidth(30)







        pos_layout.addWidget(self.position_value)







        







        servo_layout.addLayout(pos_layout)







        







        # 速度控制







        speed_layout = QHBoxLayout()







        speed_layout.addWidget(QLabel("速度:"))







        







        self.speed_slider = QSlider(Qt.Horizontal)







        self.speed_slider.setRange(0, 100)







        self.speed_slider.setValue(self.config.hardware.servo.speed)







        self.speed_slider.setEnabled(False)







        speed_layout.addWidget(self.speed_slider)







        







        self.speed_value = QLabel(str(self.config.hardware.servo.speed))







        self.speed_value.setFixedWidth(30)







        speed_layout.addWidget(self.speed_value)







        







        servo_layout.addLayout(speed_layout)







        







        # 力控制







        force_layout = QHBoxLayout()







        force_layout.addWidget(QLabel("力限制:"))







        







        self.force_slider = QSlider(Qt.Horizontal)







        self.force_slider.setRange(0, 100)







        self.force_slider.setValue(int(self.config.hardware.servo.torque))







        self.force_slider.setEnabled(False)







        force_layout.addWidget(self.force_slider)







        







        self.force_value = QLabel(str(self.config.hardware.servo.torque))







        self.force_value.setFixedWidth(30)







        force_layout.addWidget(self.force_value)







        







        servo_layout.addLayout(force_layout)







        







        # 控制按钮







        servo_btn_layout = QHBoxLayout()







        







        self.open_gripper_btn = QPushButton("打开")







        self.open_gripper_btn.clicked.connect(lambda: self.move_gripper(100))







        self.open_gripper_btn.setEnabled(False)







        servo_btn_layout.addWidget(self.open_gripper_btn)







        







        self.close_gripper_btn = QPushButton("关闭")







        self.close_gripper_btn.clicked.connect(lambda: self.move_gripper(0))







        self.close_gripper_btn.setEnabled(False)







        servo_btn_layout.addWidget(self.close_gripper_btn)







        







        self.home_gripper_btn = QPushButton("回零")







        self.home_gripper_btn.clicked.connect(lambda: self.move_gripper(50))







        self.home_gripper_btn.setEnabled(False)







        servo_btn_layout.addWidget(self.home_gripper_btn)







        







        servo_layout.addLayout(servo_btn_layout)







        







        group_layout.addWidget(servo_group)







        







        layout.addWidget(group)







        







        # 连接信号







        self.position_slider.valueChanged.connect(







            lambda v: self.position_value.setText(str(v))







        )







        self.speed_slider.valueChanged.connect(







            lambda v: self.speed_value.setText(str(v))







        )







        self.force_slider.valueChanged.connect(







            lambda v: self.force_value.setText(str(v))







        )







        







        self.position_slider.valueChanged.connect(self.update_servo_position)







        self.speed_slider.valueChanged.connect(self.update_servo_speed)







        self.force_slider.valueChanged.connect(self.update_servo_force)







    







    def create_demo_control_group(self, layout):







        """创建演示控制组"""







        group = QGroupBox("演示控制")







        group_layout = QVBoxLayout(group)







        







        # 演示选择







        demo_layout = QHBoxLayout()







        demo_layout.addWidget(QLabel("演示模式:"))







        







        self.demo_combo = QComboBox()







        # 检查配置中是否有演示模式







        if hasattr(self.config, 'demo_modes'):







            for demo in self.config.demo_modes:







                self.demo_combo.addItem(demo.replace("_", " ").title(), demo)







        else:







            # 默认演示模式







            default_demos = ["calibration", "grasping", "slip_detection", "object_classification", "force_control", "learning"]







            for demo in default_demos:







                self.demo_combo.addItem(demo.replace("_", " ").title(), demo)







        







        demo_layout.addWidget(self.demo_combo)







        







        group_layout.addLayout(demo_layout)







        







        # 演示参数







        self.demo_params_text = QLineEdit()







        self.demo_params_text.setPlaceholderText('可直接输入力值(如 30)，或JSON: {"force":30,"duration":5}')







        group_layout.addWidget(QLabel("演示参数 (JSON):"))







        group_layout.addWidget(self.demo_params_text)







        







        # 控制按钮







        btn_layout = QHBoxLayout()







        







        self.start_demo_btn = QPushButton("开始演示")







        self.start_demo_btn.clicked.connect(self.start_demo)







        self.start_demo_btn.setEnabled(False)







        btn_layout.addWidget(self.start_demo_btn)







        







        self.stop_demo_btn = QPushButton("停止演示")







        self.stop_demo_btn.clicked.connect(self.stop_demo)







        self.stop_demo_btn.setEnabled(False)







        btn_layout.addWidget(self.stop_demo_btn)







        







        self.pause_demo_btn = QPushButton("暂停")







        self.pause_demo_btn.clicked.connect(self.pause_demo)







        self.pause_demo_btn.setEnabled(False)







        btn_layout.addWidget(self.pause_demo_btn)







        







        group_layout.addLayout(btn_layout)







        







        # 演示进度







        self.demo_progress = QProgressBar()







        self.demo_progress.setRange(0, 100)







        self.demo_progress.setValue(0)







        group_layout.addWidget(self.demo_progress)







        







        # 演示状态







        self.demo_status_label = QLabel("演示状态: 未开始")







        self.demo_status_label.setStyleSheet("""







            QLabel {







                padding: 5px;







                background-color: #f5f5f5;







                border-radius: 3px;







            }







        """)







        group_layout.addWidget(self.demo_status_label)







        







        layout.addWidget(group)







    







    def create_parameter_group(self, layout):




        """创建参数设置组"""




        group = QGroupBox("参数设置")




        group_layout = QVBoxLayout(group)









        # 参数配置




        tab_layout = QVBoxLayout()









        # STM32 参数




        stm32_group = QGroupBox("STM32 参数")




        stm32_layout = QFormLayout(stm32_group)




        self.stm32_port_input = QLineEdit(getattr(self.config.hardware.servo, 'port', ''))




        self.stm32_baud_spin = QSpinBox()




        self.stm32_baud_spin.setRange(1200, 2000000)




        self.stm32_baud_spin.setValue(int(getattr(self.config.hardware.servo, 'baudrate', 115200)))




        self.stm32_timeout_spin = QDoubleSpinBox()




        self.stm32_timeout_spin.setRange(0.01, 10.0)




        self.stm32_timeout_spin.setDecimals(2)




        self.stm32_timeout_spin.setValue(float(getattr(self.config.hardware.servo, 'timeout', 1.0)))




        stm32_layout.addRow("端口:", self.stm32_port_input)




        stm32_layout.addRow("波特率:", self.stm32_baud_spin)




        stm32_layout.addRow("超时(s):", self.stm32_timeout_spin)




        tab_layout.addWidget(stm32_group)









        # 触觉传感器参数




        sensor_group = QGroupBox("触觉传感器")




        sensor_layout = QFormLayout(sensor_group)




        self.tactile_port_input = QLineEdit(getattr(self.config.hardware.sensor, 'port', ''))




        self.tactile_baud_spin = QSpinBox()




        self.tactile_baud_spin.setRange(1200, 2000000)




        self.tactile_baud_spin.setValue(int(getattr(self.config.hardware.sensor, 'baudrate', 115200)))




        self.tactile_timeout_spin = QDoubleSpinBox()




        self.tactile_timeout_spin.setRange(0.01, 10.0)




        self.tactile_timeout_spin.setDecimals(2)




        self.tactile_timeout_spin.setValue(float(getattr(self.config.hardware.sensor, 'timeout', 0.1)))




        sensor_layout.addRow("端口:", self.tactile_port_input)




        sensor_layout.addRow("波特率:", self.tactile_baud_spin)




        sensor_layout.addRow("超时(s):", self.tactile_timeout_spin)









        sensor_threshold = getattr(self.config.hardware.sensor, 'force_threshold', 5.0)




        self.sensor_threshold_spin = QDoubleSpinBox()




        self.sensor_threshold_spin.setRange(0, 1000)




        self.sensor_threshold_spin.setValue(sensor_threshold)




        self.sensor_threshold_spin.setDecimals(2)




        sensor_layout.addRow("力阈值(N):", self.sensor_threshold_spin)









        if hasattr(self.config.hardware.sensor, 'filter_enabled'):




            self.sensor_filter_check = QCheckBox("启用滤波")




            self.sensor_filter_check.setChecked(self.config.hardware.sensor.filter_enabled)




        else:




            self.sensor_filter_check = QCheckBox("启用滤波")




            self.sensor_filter_check.setChecked(True)




        sensor_layout.addRow(self.sensor_filter_check)




        tab_layout.addWidget(sensor_group)









        # 机械臂参数




        arm_group = QGroupBox("机械臂参数")




        arm_layout = QFormLayout(arm_group)




        arm_port = ""




        arm_baud = 115200




        arm_timeout = 1.0




        if hasattr(self.config, 'learm_arm') and self.config.learm_arm is not None:




            if isinstance(self.config.learm_arm, dict):




                conn_cfg = self.config.learm_arm.get('CONNECTION', self.config.learm_arm)




                arm_port = conn_cfg.get('serial_port', arm_port)




                arm_baud = conn_cfg.get('baud_rate', arm_baud)




                arm_timeout = conn_cfg.get('timeout', arm_timeout)




            else:




                conn_cfg = getattr(self.config.learm_arm, 'CONNECTION', {}) or {}




                arm_port = conn_cfg.get('serial_port', arm_port)




                arm_baud = conn_cfg.get('baud_rate', arm_baud)




                arm_timeout = conn_cfg.get('timeout', arm_timeout)




        self.arm_port_input = QLineEdit(str(arm_port))




        self.arm_baud_spin = QSpinBox()




        self.arm_baud_spin.setRange(1200, 2000000)




        self.arm_baud_spin.setValue(int(arm_baud))




        self.arm_timeout_spin = QDoubleSpinBox()




        self.arm_timeout_spin.setRange(0.01, 10.0)




        self.arm_timeout_spin.setDecimals(2)




        self.arm_timeout_spin.setValue(float(arm_timeout))




        arm_layout.addRow("端口:", self.arm_port_input)




        arm_layout.addRow("波特率:", self.arm_baud_spin)




        arm_layout.addRow("超时(s):", self.arm_timeout_spin)




        tab_layout.addWidget(arm_group)









        # 视觉参数




        vision_group = QGroupBox("视觉参数")




        vision_layout = QFormLayout(vision_group)




        camera_type = 'opencv'




        camera_id = 0




        cam_width = 640




        cam_height = 480




        cam_fps = 30




        camera_cfg = getattr(self.config, 'camera', None)




        if isinstance(camera_cfg, dict):




            primary = camera_cfg.get('HARDWARE', {}).get('primary_camera', {})




            camera_type = primary.get('type', camera_type)




            camera_id = primary.get('camera_id', camera_id)




            resolution = primary.get('resolution', [cam_width, cam_height])




            if isinstance(resolution, (list, tuple)) and len(resolution) >= 2:




                cam_width, cam_height = int(resolution[0]), int(resolution[1])




            cam_fps = int(primary.get('fps', cam_fps))




        else:




            if hasattr(camera_cfg, 'camera_type'):




                camera_type = camera_cfg.camera_type




            if hasattr(camera_cfg, 'camera_index'):




                camera_id = camera_cfg.camera_index




            if hasattr(camera_cfg, 'width'):




                cam_width = camera_cfg.width




            if hasattr(camera_cfg, 'height'):




                cam_height = camera_cfg.height




            if hasattr(camera_cfg, 'fps'):




                cam_fps = camera_cfg.fps




        self.camera_type_combo = QComboBox()




        self.camera_type_combo.addItems(["opencv", "realsense", "simulation"])




        if camera_type in ["opencv", "realsense", "simulation"]:




            self.camera_type_combo.setCurrentText(camera_type)




        self.camera_id_input = QLineEdit(str(camera_id))




        self.camera_width_spin = QSpinBox()




        self.camera_width_spin.setRange(0, 4096)




        self.camera_width_spin.setValue(int(cam_width))




        self.camera_height_spin = QSpinBox()




        self.camera_height_spin.setRange(0, 4096)




        self.camera_height_spin.setValue(int(cam_height))




        self.camera_fps_spin = QSpinBox()




        self.camera_fps_spin.setRange(1, 240)




        self.camera_fps_spin.setValue(int(cam_fps))




        vision_layout.addRow("类型:", self.camera_type_combo)




        vision_layout.addRow("相机ID/设备号:", self.camera_id_input)




        vision_layout.addRow("宽度:", self.camera_width_spin)




        vision_layout.addRow("高度:", self.camera_height_spin)




        vision_layout.addRow("帧率:", self.camera_fps_spin)




        tab_layout.addWidget(vision_group)









        # 仿真参数




        sim_group = QGroupBox("仿真参数")




        sim_layout = QFormLayout(sim_group)









        sim_force_scale = getattr(self.config.hardware.sensor, 'simulation_force_scale', 2.5)




        sim_shear_scale = getattr(self.config.hardware.sensor, 'shear_scale', 0.05)




        sim_noise = getattr(self.config.hardware.sensor, 'force_noise_level', 0.1)




        sim_contact_radius = getattr(self.config.hardware.sensor, 'contact_radius', 0.35)









        self.sim_force_scale_spin = QDoubleSpinBox()




        self.sim_force_scale_spin.setRange(0.1, 10.0)




        self.sim_force_scale_spin.setValue(sim_force_scale)




        self.sim_force_scale_spin.setDecimals(2)




        sim_layout.addRow("法向力缩放:", self.sim_force_scale_spin)









        self.sim_shear_scale_spin = QDoubleSpinBox()




        self.sim_shear_scale_spin.setRange(0.0, 1.0)




        self.sim_shear_scale_spin.setValue(sim_shear_scale)




        self.sim_shear_scale_spin.setDecimals(3)




        sim_layout.addRow("切向力缩放:", self.sim_shear_scale_spin)









        self.sim_noise_spin = QDoubleSpinBox()




        self.sim_noise_spin.setRange(0.0, 1.0)




        self.sim_noise_spin.setValue(sim_noise)




        self.sim_noise_spin.setDecimals(3)




        sim_layout.addRow("噪声强度:", self.sim_noise_spin)









        self.sim_contact_radius_spin = QDoubleSpinBox()




        self.sim_contact_radius_spin.setRange(0.1, 1.0)




        self.sim_contact_radius_spin.setValue(sim_contact_radius)




        self.sim_contact_radius_spin.setDecimals(2)




        sim_layout.addRow("接触半径:", self.sim_contact_radius_spin)









        tab_layout.addWidget(sim_group)









        # 舵机参数




        servo_group = QGroupBox("舵机参数")




        servo_layout = QFormLayout(servo_group)




        min_angle = getattr(self.config.hardware.servo, 'min_angle', 0)




        max_angle = getattr(self.config.hardware.servo, 'max_angle', 180)




        self.servo_min_angle_spin = QSpinBox()




        self.servo_min_angle_spin.setRange(0, 180)




        self.servo_min_angle_spin.setValue(min_angle)




        servo_layout.addRow("最小角度:", self.servo_min_angle_spin)




        self.servo_max_angle_spin = QSpinBox()




        self.servo_max_angle_spin.setRange(0, 180)




        self.servo_max_angle_spin.setValue(max_angle)




        servo_layout.addRow("最大角度:", self.servo_max_angle_spin)




        tab_layout.addWidget(servo_group)









        # 算法参数




        algo_group = QGroupBox("算法参数")




        algo_layout = QFormLayout(algo_group)




        if hasattr(self.config, 'algorithm'):




            confidence_threshold = getattr(self.config.algorithm, 'confidence_threshold', 0.7)




            slip_detection_enabled = getattr(self.config.algorithm, 'slip_detection_enabled', True)




        else:




            confidence_threshold = 0.7




            slip_detection_enabled = True




        self.algo_confidence_spin = QDoubleSpinBox()




        self.algo_confidence_spin.setRange(0, 1)




        self.algo_confidence_spin.setValue(confidence_threshold)




        self.algo_confidence_spin.setDecimals(2)




        self.algo_confidence_spin.setSingleStep(0.05)




        algo_layout.addRow("置信度阈值:", self.algo_confidence_spin)




        self.algo_slip_check = QCheckBox("启用滑移检测")




        self.algo_slip_check.setChecked(slip_detection_enabled)




        algo_layout.addRow(self.algo_slip_check)




        tab_layout.addWidget(algo_group)









        group_layout.addLayout(tab_layout)









        # 应用参数




        self.apply_params_btn = QPushButton("应用参数")




        self.apply_params_btn.clicked.connect(self.apply_parameters)




        group_layout.addWidget(self.apply_params_btn)









        layout.addWidget(group)




    def create_data_monitor_group(self, layout):







        """创建数据监控组"""







        group = QGroupBox("数据监控")







        group_layout = QVBoxLayout(group)







        







        # 实时数据显示







        self.live_data_label = QLabel("Force: N/A")
        self.live_data_label.setWordWrap(True)
        self.live_data_label.setStyleSheet("color: #cccccc;")
        group_layout.addWidget(self.live_data_label)

        self.data_display = QTextEdit()







        self.data_display.setMaximumHeight(150)







        self.data_display.setReadOnly(True)
        self.data_display.setUndoRedoEnabled(False)
        self.data_display.document().setMaximumBlockCount(200)







        self.data_display.setFont(QFont("Courier New", 9))







        group_layout.addWidget(self.data_display)







        







        # 数据记录控制







        record_layout = QHBoxLayout()







        







        self.record_check = QCheckBox("记录数据")







        record_layout.addWidget(self.record_check)







        







        self.clear_data_btn = QPushButton("清除数据")







        self.clear_data_btn.clicked.connect(self.clear_data)







        record_layout.addWidget(self.clear_data_btn)







        







        self.save_data_btn = QPushButton("保存数据")







        self.save_data_btn.clicked.connect(self.save_data)







        record_layout.addWidget(self.save_data_btn)







        







        group_layout.addLayout(record_layout)







        







        layout.addWidget(group)







    







    def create_system_control_group(self, layout):







        """创建系统控制组"""







        group = QGroupBox("系统控制")







        group_layout = QVBoxLayout(group)







        







        # 紧急停止按钮







        self.emergency_btn = QPushButton("紧急停止")







        self.emergency_btn.setCheckable(True)







        self.emergency_btn.clicked.connect(self.emergency_stop)







        self.emergency_btn.setStyleSheet("""







            QPushButton {







                background-color: #f44336;







                color: white;







                font-weight: bold;







                padding: 12px;







                border-radius: 4px;







                font-size: 12pt;







            }







            QPushButton:hover {







                background-color: #d32f2f;







            }







        """)







        group_layout.addWidget(self.emergency_btn)







        







        # 系统控制按钮







        sys_btn_layout = QHBoxLayout()







        







        self.reset_btn = QPushButton("重置系统")







        self.reset_btn.clicked.connect(self.reset_system)







        sys_btn_layout.addWidget(self.reset_btn)







        







        self.shutdown_btn = QPushButton("关闭系统")







        self.shutdown_btn.clicked.connect(self.shutdown_system)







        sys_btn_layout.addWidget(self.shutdown_btn)







        







        group_layout.addLayout(sys_btn_layout)















        # PyBullet 仿真窗口控制







        sim_btn_layout = QHBoxLayout()







        self.launch_sim_btn = QPushButton("打开仿真窗口")







        self.launch_sim_btn.clicked.connect(self.launch_pybullet_gui)







        sim_btn_layout.addWidget(self.launch_sim_btn)















        self.stop_sim_btn = QPushButton("关闭仿真窗口")







        self.stop_sim_btn.clicked.connect(self.stop_pybullet_gui)







        sim_btn_layout.addWidget(self.stop_sim_btn)















        group_layout.addLayout(sim_btn_layout)







        







        layout.addWidget(group)







    







    # 信号处理函数







    def connect_hardware(self):







        """连接硬件"""







        self.logger.info("发送连接硬件命令")







        self.control_signal.emit("connect_hardware", {})







        







        # 更新按钮状态







        self.connect_btn.setEnabled(False)







        self.connect_btn.setText("连接中...")







    







    def disconnect_hardware(self):







        """断开硬件"""







        self.logger.info("发送断开硬件命令")







        self.control_signal.emit("disconnect_hardware", {})







        







        # 更新按钮状态







        self.disconnect_btn.setEnabled(False)







        self.disconnect_btn.setText("断开中...")







    







    def calibrate_hardware(self):







        """校准硬件"""







        self.logger.info("发送校准硬件命令")







        self.control_signal.emit("calibrate_hardware", {})







    







    def move_gripper(self, position):







        """移动夹爪到指定位置"""







        self.logger.info(f"移动夹爪到位置: {position}")







        self.control_signal.emit("move_gripper", {"position": position})







    







    def update_servo_position(self, position):







        """更新舵机位置"""







        self.control_signal.emit("set_servo_position", {"position": position})







    







    def update_servo_speed(self, speed):







        """更新舵机速度"""







        self.control_signal.emit("set_servo_speed", {"speed": speed})







    







    def update_servo_force(self, force):







        """更新舵机力限制"""







        self.control_signal.emit("set_servo_force", {"force": force})







    







    def start_demo(self):







        """开始演示"""







        demo_name = self.demo_combo.currentData()







        params_text = self.demo_params_text.text().strip()







        







        params = self._parse_demo_params(params_text)







        if params_text and not params:







            self.data_display.append("提示: 参数未解析，使用默认参数。支持直接输入数值表示 force，如 30，或完整 JSON。")







        







        self.logger.info(f"开始演示: {demo_name}, 参数: {params}")







        self.control_signal.emit("start_demo", {







            "demo_name": demo_name,







            "params": params







        })







        







        # 更新按钮状态







        self.start_demo_btn.setEnabled(False)







        self.stop_demo_btn.setEnabled(True)







        self.pause_demo_btn.setEnabled(True)















    def start_preset_demo(self, demo_name: str, params: dict):







        """一键预设演示"""







        self.demo_combo.setCurrentText(demo_name.replace("_", " ").title())







        self.logger.info(f"预设演示: {demo_name}, 参数: {params}")







        self.control_signal.emit("start_demo", {







            "demo_name": demo_name,







            "params": params







        })







        self.start_demo_btn.setEnabled(False)







        self.stop_demo_btn.setEnabled(True)







        self.pause_demo_btn.setEnabled(True)















    def _parse_demo_params(self, text: str) -> dict:







        """支持简写：直接填数值等同于 {'force': 数值}，也可填 force:30 / force=30 或 JSON"""







        if not text:







            return {}







        # 先尝试 JSON







        try:







            import json







            parsed = json.loads(text)







            return parsed if isinstance(parsed, dict) else {}







        except Exception:







            pass







        # 简写格式: "30" 或 "force:30" / "force=30" / "30N"







        m = re.match(r'^(?:force\s*[:=]?\s*)?(-?\d+(?:\.\d+)?)', text, re.IGNORECASE)







        if m:







            try:







                val = float(m.group(1))







                # 如果是整数值，转为 int，便于下游使用







                val = int(val) if val.is_integer() else val







                return {"force": val}







            except Exception:







                return {}







        return {}







    







    def stop_demo(self):







        """停止演示"""







        self.logger.info("停止演示")







        self.control_signal.emit("stop_demo", {})







        







        # 更新按钮状态







        self.start_demo_btn.setEnabled(True)







        self.stop_demo_btn.setEnabled(False)







        self.pause_demo_btn.setEnabled(False)







        self.pause_demo_btn.setText("暂停")







    







    def pause_demo(self):







        """暂停演示"""







        self.logger.info("暂停演示")







        if self.current_demo_status not in ("running", "starting"):







            self.data_display.append("提示: 当前没有运行中的演示，暂停无效")







            return







        if self.pause_demo_btn.text() == "暂停":







            self.control_signal.emit("pause_demo", {})







            self.pause_demo_btn.setText("继续")







        else:







            self.control_signal.emit("resume_demo", {})







            self.pause_demo_btn.setText("暂停")







    







    def apply_parameters(self):
        """应用参数"""
        self.logger.info("应用参数")

        sensor_rate = None
        if hasattr(self, 'sensor_rate_combo'):
            try:
                sensor_rate = int(self.sensor_rate_combo.currentText())
            except Exception:
                sensor_rate = None

        # 参数通过 DemoManager 转交给控制线程
        params = {
            "sensor": {
                "port": self.tactile_port_input.text().strip(),
                "baudrate": self.tactile_baud_spin.value(),
                "timeout": self.tactile_timeout_spin.value(),
                "force_threshold": self.sensor_threshold_spin.value(),
                "filter_enabled": self.sensor_filter_check.isChecked(),
            },
            "servo": {
                "port": self.stm32_port_input.text().strip(),
                "baudrate": self.stm32_baud_spin.value(),
                "timeout": self.stm32_timeout_spin.value(),
                "min_angle": self.servo_min_angle_spin.value(),
                "max_angle": self.servo_max_angle_spin.value(),
            },
            "algorithm": {
                "confidence_threshold": self.algo_confidence_spin.value(),
                "slip_detection_enabled": self.algo_slip_check.isChecked(),
            },
            "simulation": {
                "simulation_force_scale": self.sim_force_scale_spin.value(),
                "shear_scale": self.sim_shear_scale_spin.value(),
                "force_noise_level": self.sim_noise_spin.value(),
                "contact_radius": self.sim_contact_radius_spin.value(),
            }
        }

        if sensor_rate is not None:
            params["sensor"]["sampling_rate"] = sensor_rate

        # 同步机械臂配置到 DemoConfig
        arm_cfg = getattr(self.config, 'learm_arm', None)
        if not isinstance(arm_cfg, dict):
            arm_cfg = {}
        conn_cfg = arm_cfg.get('CONNECTION', {}) if isinstance(arm_cfg, dict) else {}
        conn_cfg["serial_port"] = self.arm_port_input.text().strip()
        conn_cfg["baud_rate"] = self.arm_baud_spin.value()
        conn_cfg["timeout"] = self.arm_timeout_spin.value()
        arm_cfg["CONNECTION"] = conn_cfg
        self.config.learm_arm = arm_cfg

        # 同步相机配置到 dict 结构
        camera_cfg = getattr(self.config, 'camera', None)
        if not isinstance(camera_cfg, dict):
            camera_cfg = {}
        hardware_cfg = camera_cfg.get('HARDWARE', {})
        primary_cfg = hardware_cfg.get('primary_camera', {})
        primary_cfg["type"] = self.camera_type_combo.currentText()
        primary_cfg["camera_id"] = self.camera_id_input.text().strip()
        primary_cfg["resolution"] = [self.camera_width_spin.value(), self.camera_height_spin.value()]
        primary_cfg["fps"] = self.camera_fps_spin.value()
        hardware_cfg["primary_camera"] = primary_cfg
        camera_cfg["HARDWARE"] = hardware_cfg
        self.config.camera = camera_cfg

        self.control_signal.emit("apply_parameters", params)

    def clear_data(self):







        """清除数据"""







        self.logger.info("清除数据")







        self.data_display.clear()







        self.control_signal.emit("clear_data", {})







    







    def save_data(self):







        """保存数据"""







        self.logger.info("保存数据")







        self.control_signal.emit("save_data", {})







    







    def emergency_stop(self):







        """紧急停止"""







        if self.emergency_btn.isChecked():







            self.logger.warning("发送紧急停止命令")







            self.emergency_btn.setText("继续")







            self.emergency_btn.setStyleSheet("background-color: #ffcc00; color: black; font-weight: bold;")







            self.control_signal.emit("emergency_stop", {})







        else:







            self.logger.info("退出紧急停止，尝试恢复系统")







            self.emergency_btn.setText("紧急停止")







            self.emergency_btn.setStyleSheet("")







            try:







                self.control_signal.emit("connect_hardware", {})







                if hasattr(self.demo_manager.data_acquisition, "start_acquisition"):







                    self.demo_manager.data_acquisition.start_acquisition()







            except Exception as e:







                self.logger.error(f"恢复失败: {e}")







    







    def reset_system(self):







        """重置系统"""







        self.logger.info("重置系统")







        self.control_signal.emit("reset_system", {})







    







    def shutdown_system(self):







        """关闭系统"""







        self.logger.info("请求关闭系统")







        self.request_shutdown.emit()















    def launch_pybullet_gui(self):







        """启动 PyBullet 自带窗口（独立进程）"""







        script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts", "run_pybullet_gui.py"))







        if not os.path.exists(script_path):







            if hasattr(self, "data_display"):







                self.data_display.append(f"仿真脚本不存在: {script_path}")







            return















        if self.pybullet_process and self.pybullet_process.poll() is None:







            if hasattr(self, "data_display"):







                self.data_display.append("PyBullet 仿真窗口已在运行")







            return















        env = os.environ.copy()







        env.setdefault("OBJC_DISABLE_INITIALIZE_FORK_SAFETY", "YES")







        env.setdefault("PYTHONUNBUFFERED", "1")







        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))







        try:







            os.makedirs(os.path.dirname(self._pybullet_log_path), exist_ok=True)







            self._pybullet_log_fp = open(self._pybullet_log_path, "a", encoding="utf-8")







            self.pybullet_process = subprocess.Popen(







                [sys.executable, script_path],







                cwd=project_root,







                env=env,







                stdout=self._pybullet_log_fp,







                stderr=self._pybullet_log_fp,







            )







            if hasattr(self, "data_display"):







                self.data_display.append("已启动 PyBullet 仿真窗口，正在检查运行状态...")







            QTimer.singleShot(600, self._check_pybullet_process)







        except Exception as exc:







            if hasattr(self, "data_display"):







                self.data_display.append(f"启动 PyBullet 仿真窗口失败: {exc}")







            if self._pybullet_log_fp:







                try:







                    self._pybullet_log_fp.close()







                except Exception:







                    pass







                self._pybullet_log_fp = None















    def stop_pybullet_gui(self):







        """关闭 PyBullet 窗口进程"""







        if self.pybullet_process and self.pybullet_process.poll() is None:







            try:







                self.pybullet_process.terminate()







                if hasattr(self, "data_display"):







                    self.data_display.append("已请求关闭 PyBullet 仿真窗口")







            except Exception as exc:







                if hasattr(self, "data_display"):







                    self.data_display.append(f"关闭 PyBullet 仿真窗口失败: {exc}")







        else:







            if hasattr(self, "data_display"):







                self.data_display.append("PyBullet 仿真窗口未运行")







        self.pybullet_process = None







        if self._pybullet_log_fp:







            try:







                self._pybullet_log_fp.close()







            except Exception:







                pass







            self._pybullet_log_fp = None















    def _check_pybullet_process(self):







        """启动后检查子进程是否立即退出"""







        if not self.pybullet_process:







            return







        if self.pybullet_process.poll() is None:







            if hasattr(self, "data_display"):







                self.data_display.append("PyBullet 仿真窗口运行中")







            return







        if hasattr(self, "data_display"):







            self.data_display.append(







                f"PyBullet 仿真窗口已退出，请查看日志: {self._pybullet_log_path}"







            )







        self.pybullet_process = None







    







    # 状态更新函数







    @pyqtSlot(str, dict)







    def update_control_status(self, status: str, info: dict):







        """







        更新控制状态







        







        Args:







            status: 状态字符串







            info: 状态信息字典







        """







        if status == "connected":







            # 硬件已连接







            self.connect_btn.setEnabled(False)







            self.connect_btn.setText("已连接")







            self.disconnect_btn.setEnabled(True)







            self.disconnect_btn.setText("断开硬件")







            self.calibrate_btn.setEnabled(True)







            







            # 启用舵机控制







            self.position_slider.setEnabled(True)







            self.speed_slider.setEnabled(True)







            self.force_slider.setEnabled(True)







            self.open_gripper_btn.setEnabled(True)







            self.close_gripper_btn.setEnabled(True)







            self.home_gripper_btn.setEnabled(True)







            







            # 启用演示控制







            self.start_demo_btn.setEnabled(True)







            self.preset_vector_btn.setEnabled(True)







            self.preset_mapping_btn.setEnabled(True)







            







            # 更新状态标签







            self.system_status_label.setText("状态: 已连接")







            self.system_status_label.setStyleSheet("""







                QLabel {







                    padding: 8px;







                    background-color: #e8f5e9;







                    border-radius: 5px;







                    border: 1px solid #c8e6c9;







                }







            """)







            







        elif status == "disconnected":







            # 硬件已断开







            self.connect_btn.setEnabled(True)







            self.connect_btn.setText("连接硬件")







            self.disconnect_btn.setEnabled(False)







            self.disconnect_btn.setText("断开硬件")







            self.calibrate_btn.setEnabled(False)







            







            # 禁用舵机控制







            self.position_slider.setEnabled(False)







            self.speed_slider.setEnabled(False)







            self.force_slider.setEnabled(False)







            self.open_gripper_btn.setEnabled(False)







            self.close_gripper_btn.setEnabled(False)







            self.home_gripper_btn.setEnabled(False)







            







            # 禁用演示控制







            self.start_demo_btn.setEnabled(False)







            self.stop_demo_btn.setEnabled(False)







            self.pause_demo_btn.setEnabled(False)







            self.preset_vector_btn.setEnabled(False)







            self.preset_mapping_btn.setEnabled(False)







            







            # 更新状态标签







            self.system_status_label.setText("状态: 未连接")







            self.system_status_label.setStyleSheet("""







                QLabel {







                    padding: 8px;







                    background-color: #ffebee;







                    border-radius: 5px;







                    border: 1px solid #ffcdd2;







                }







            """)







            







        elif status == "demo_started":







            # 演示已开始







            self.start_demo_btn.setEnabled(False)







            self.stop_demo_btn.setEnabled(True)







            self.pause_demo_btn.setEnabled(True)







            self.demo_status_label.setText("演示状态: 运行中")







            self.current_demo_status = "running"







        







        elif status == "demo_starting" or status == "demo_running":







            # 演示启动/运行中







            self.start_demo_btn.setEnabled(False)







            self.stop_demo_btn.setEnabled(True)







            self.pause_demo_btn.setEnabled(True)







            msg = info.get("message", "运行中")







            self.demo_status_label.setText(f"演示状态: {msg}")







            self.current_demo_status = "running"







            







        elif status == "demo_stopped":







            # 演示已停止







            self.start_demo_btn.setEnabled(True)







            self.stop_demo_btn.setEnabled(False)







            self.pause_demo_btn.setEnabled(False)







            self.pause_demo_btn.setText("暂停")







            self.demo_progress.setValue(0)







            self.demo_status_label.setText("演示状态: 已停止")







            self.current_demo_status = "idle"







        







        elif status == "demo_completed":







            self.start_demo_btn.setEnabled(True)







            self.stop_demo_btn.setEnabled(False)







            self.pause_demo_btn.setEnabled(False)







            self.pause_demo_btn.setText("暂停")







            self.demo_progress.setValue(100)







            self.demo_status_label.setText("演示状态: 已完成")







            self.current_demo_status = "idle"







        







        elif status == "demo_failed":







            self.start_demo_btn.setEnabled(True)







            self.stop_demo_btn.setEnabled(False)







            self.pause_demo_btn.setEnabled(False)







            self.pause_demo_btn.setText("暂停")







            self.demo_progress.setValue(0)







            err = info.get("error", "演示失败")







            self.demo_status_label.setText(f"演示状态: {err}")







            self.current_demo_status = "idle"







            







        elif status == "demo_progress":







            # 演示进度更新







            progress = info.get("progress", 0)







            self.demo_progress.setValue(int(progress * 100))







            if "message" in info:







                self.demo_status_label.setText(f"演示状态: {info['message']}")







            else:







                self.demo_status_label.setText(f"演示进度: {progress*100:.1f}%")







            







        elif status == "error":







            # 错误状态







            error_msg = info.get("message", "未知错误")







            self.system_status_label.setText(f"状态: 错误 - {error_msg}")







            self.system_status_label.setStyleSheet("""







                QLabel {







                    padding: 8px;







                    background-color: #ffebee;







                    border-radius: 5px;







                    border: 1px solid #ffcdd2;







                    color: #d32f2f;







                }







            """)







            







            # 在数据显示区显示错误







            self.data_display.append(f"[ERROR] {error_msg}")







    







    @pyqtSlot(object)







    def update_data_display(self, data):







        """







        更新数据显示







        







        Args:







            data: 数据对象







        """







        if hasattr(data, 'force_data'):







            # 显示力数据







            force_str = ", ".join([f"{f:.2f}" for f in data.force_data[:5]])







            if len(data.force_data) > 5:







                force_str += "..."







            







            # 更新数据率







            if hasattr(data, 'timestamp'):







                current_time = data.timestamp







                if hasattr(self, 'last_update_time'):







                    time_diff = current_time - self.last_update_time







                    if time_diff > 0:







                        data_rate = 1.0 / time_diff







                        self.data_rate_label.setText(f"{data_rate:.1f}")







                







                self.last_update_time = current_time







            







            # 在文本框中显示数据







            if hasattr(self, "live_data_label"):
                self.live_data_label.setText(f"Force: [{force_str}]")














    def _format_status_text(self, name: str, connected: bool, simulation: bool) -> str:



        """生成状态文本"""



        if connected and simulation:



            return f"{name}: 仿真已连接"



        if connected:



            return f"{name}: 已连接"



        return f"{name}: 未连接"







    def _apply_status_style(self, label: QLabel, connected: bool, simulation: bool):



        """根据连接/仿真状态设置样式"""



        if connected and simulation:



            label.setStyleSheet("""



                QLabel {



                    padding: 5px;



                    background-color: #fff8e1;



                    border-radius: 3px;



                    border: 1px solid #ffe0b2;



                }



            """)



            return



        if connected:



            label.setStyleSheet("""



                QLabel {



                    padding: 5px;



                    background-color: #e8f5e9;



                    border-radius: 3px;



                    border: 1px solid #c8e6c9;



                }



            """)



            return



        label.setStyleSheet("""



            QLabel {



                padding: 5px;



                background-color: #ffebee;



                border-radius: 3px;



                border: 1px solid #ffcdd2;



            }



        """)







    def _should_update_device_status(self, key: str, connected: bool, simulation: bool) -> bool:
        state = (bool(connected), bool(simulation))
        if self._device_status_cache.get(key) == state:
            return False
        self._device_status_cache[key] = state
        return True

    def update_device_status(self, stm32: Optional[Dict[str, Any]] = None,
                             vision: Optional[Dict[str, Any]] = None,
                             arm: Optional[Dict[str, Any]] = None,
                             tactile: Optional[Dict[str, Any]] = None):
        """Update device connection status."""
        if stm32 is not None:
            connected = bool(stm32.get("connected"))
            simulation = bool(stm32.get("simulation"))
            if self._should_update_device_status("stm32", connected, simulation):
                self.stm32_status_label.setText(self._format_status_text("STM32", connected, simulation))
                self._apply_status_style(self.stm32_status_label, connected, simulation)
                self.stm32_connect_btn.setEnabled(not connected)
                self.stm32_disconnect_btn.setEnabled(connected)
                self.position_slider.setEnabled(connected)
                self.speed_slider.setEnabled(connected)
                self.force_slider.setEnabled(connected)
                self.open_gripper_btn.setEnabled(connected)
                self.close_gripper_btn.setEnabled(connected)
                self.home_gripper_btn.setEnabled(connected)

        if vision is not None:
            connected = bool(vision.get("connected"))
            simulation = bool(vision.get("simulation"))
            if self._should_update_device_status("vision", connected, simulation):
                self.vision_status_label.setText(self._format_status_text("Vision", connected, simulation))
                self._apply_status_style(self.vision_status_label, connected, simulation)
                self.vision_connect_btn.setEnabled(not connected)
                self.vision_disconnect_btn.setEnabled(connected)

        if arm is not None:
            connected = bool(arm.get("connected"))
            simulation = bool(arm.get("simulation"))
            if self._should_update_device_status("arm", connected, simulation):
                self.arm_status_label.setText(self._format_status_text("Arm", connected, simulation))
                self._apply_status_style(self.arm_status_label, connected, simulation)
                self.arm_connect_btn.setEnabled(not connected)
                self.arm_disconnect_btn.setEnabled(connected)

        if tactile is not None:
            connected = bool(tactile.get("connected"))
            simulation = bool(tactile.get("simulation"))
            if self._should_update_device_status("tactile", connected, simulation):
                self.tactile_status_label.setText(self._format_status_text("Tactile", connected, simulation))
                self._apply_status_style(self.tactile_status_label, connected, simulation)
                self.tactile_connect_btn.setEnabled(not connected)
                self.tactile_disconnect_btn.setEnabled(connected)

    def connect_stm32(self):



        """连接 STM32"""



        self.control_signal.emit("connect_stm32", {})







    def disconnect_stm32(self):



        """断开 STM32"""



        self.control_signal.emit("disconnect_stm32", {})







    def connect_tactile(self):



        """连接触觉传感器"""



        self.control_signal.emit("connect_tactile", {})







    def disconnect_tactile(self):



        """断开触觉传感器"""



        self.control_signal.emit("disconnect_tactile", {})







    def connect_vision(self):



        """连接视觉"""



        self.vision_connect_request.emit()







    def disconnect_vision(self):



        """断开视觉"""



        self.vision_disconnect_request.emit()







    def connect_arm(self):



        """连接机械臂"""



        self.control_signal.emit("connect_arm", {})







    def disconnect_arm(self):



        """断开机械臂"""



        self.control_signal.emit("disconnect_arm", {})







    def update_hardware_status(self, sensor_connected: bool, servo_connected: bool):


        """更新硬件连接状态"""


        self.update_device_status(


            stm32={"connected": bool(servo_connected), "simulation": False},


            tactile={"connected": bool(sensor_connected), "simulation": False},


        )





    def set_config(self, config: DemoConfig):







        """







        设置新的配置







        







        Args:







            config: 新配置







        """







        self.config = config







        







        # 更新UI元素







        self.sensor_rate_combo.setCurrentText(str(config.hardware.sensor.sampling_rate))







        self.speed_slider.setValue(config.hardware.servo.speed)







        self.force_slider.setValue(int(config.hardware.servo.torque))







        







        # 使用getattr获取属性，避免属性不存在







        sensor_threshold = getattr(config.hardware.sensor, 'force_threshold', 5.0)







        self.sensor_threshold_spin.setValue(sensor_threshold)







        







        filter_enabled = getattr(config.hardware.sensor, 'filter_enabled', True)







        self.sensor_filter_check.setChecked(filter_enabled)







        







        min_angle = getattr(config.hardware.servo, 'min_angle', 0)







        max_angle = getattr(config.hardware.servo, 'max_angle', 180)







        self.servo_min_angle_spin.setValue(min_angle)







        self.servo_max_angle_spin.setValue(max_angle)







        







        # 算法参数







        if hasattr(config, 'algorithm'):







            confidence_threshold = getattr(config.algorithm, 'confidence_threshold', 0.7)







            slip_detection_enabled = getattr(config.algorithm, 'slip_detection_enabled', True)







        else:







            confidence_threshold = 0.7







            slip_detection_enabled = True







        







        self.algo_confidence_spin.setValue(confidence_threshold)







        self.algo_slip_check.setChecked(slip_detection_enabled)







        







        # 更新演示模式列表







        self.demo_combo.clear()







        if hasattr(config, 'demo_modes'):







            for demo in config.demo_modes:







                self.demo_combo.addItem(demo.replace("_", " ").title(), demo)







        else:







            # 默认演示模式







            default_demos = ["calibration", "grasping", "slip_detection", "object_classification", "force_control", "learning"]







            for demo in default_demos:







                self.demo_combo.addItem(demo.replace("_", " ").title(), demo)





