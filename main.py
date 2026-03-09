#!/usr/bin/env python3
"""
触觉夹爪演示系统主入口
主文件：main.py
功能：系统初始化、模块集成、GUI启动和主控制循环
"""

import sys
import os
import signal
import logging
import time
import json
import traceback
from pathlib import Path
from typing import Dict, Any, Optional, List

# 添加项目根目录到Python路径
project_root = Path(__file__).parent
src_root = project_root / "src"
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

# 导入PyQt5
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt, QTimer, QObject, pyqtSignal
from PyQt5.QtGui import QFont, QIcon

# 导入深度学习相关库
try:
    import torch
    import numpy as np
    DEEP_LEARNING_AVAILABLE = True
except ImportError:
    torch = None
    np = None
    DEEP_LEARNING_AVAILABLE = False
    print("警告: 深度学习库未安装，相关功能将不可用")

# 导入自定义模块
from config.demo_config import DemoConfig
from config.paxini_gen3_config import PaxiniGen3Config, create_3x3_config
from config.deep_learning_config import DeepLearningConfig, create_default_dl_config
from src.gui.main_window import MainWindow
from src.core.demo_manager import DemoManager
from src.core.hardware_interface import HardwareInterface
from src.core.data_acquisition import DataAcquisitionThread
from src.core.control_thread import ControlThread
from src.tactile_perception import create_default_pipeline
from src.servo_control.gripper_controller import GripperController
from src.arm_control.learm_interface import LearmInterface
from src.arm_control.joint_controller import JointController

# 导入深度学习模块
if DEEP_LEARNING_AVAILABLE:
    try:
        from src.deep_learning.inference import GraspInference
        from src.deep_learning.pid_controller import NeuralPIDController, AdaptivePID
        from src.deep_learning.data_loader import RealTimeDataLoader
        from src.deep_learning.grip_classifier import GraspStateClassifier
    except ImportError as e:
        print(f"警告: 深度学习模块导入失败: {e}")
        DEEP_LEARNING_AVAILABLE = False

class DeepLearningSignals(QObject):
    """深度学习模块信号"""
    inference_completed = pyqtSignal(dict)  # 推理完成信号
    training_progress = pyqtSignal(int, str)  # 训练进度信号
    error_occurred = pyqtSignal(str, str)  # 错误信号


class TactileGripperDemo:
    """触觉夹爪演示系统主类"""
    
    def __init__(self, config_path: str = None, sensor_type: str = "default"):
        """
        初始化演示系统
        
        Args:
            config_path: 配置文件路径，如果为None则使用默认配置
            sensor_type: 传感器类型 ("default" 或 "3x3")
        """
        # 设置日志系统
        self.setup_logging()
        
        # 根据传感器类型加载配置
        self.config = self.load_config(config_path, sensor_type)
        # 确保机械臂配置存在（用于固定零点与回零逻辑）
        if not getattr(self.config, "learm_arm", None):
            try:
                self.config.enable_arm_integration()
            except Exception as exc:
                self.logger.warning(f"初始化机械臂默认配置失败: {exc}")
        self.sensor_type = sensor_type

        # 检查深度学习可用性
        if not DEEP_LEARNING_AVAILABLE:
            self.logger.warning("深度学习库不可用，相关功能将被禁用")
        # 安全读取深度学习开关
        self.deep_learning_enabled = getattr(self.config, "deep_learning", None)
        if self.deep_learning_enabled is not None and hasattr(self.deep_learning_enabled, "enabled"):
            self.deep_learning_enabled = self.deep_learning_enabled.enabled
        else:
            self.deep_learning_enabled = False
        
        # 初始化应用状态
        self.is_running = False
        self.modules = {}
        
        # 创建Qt应用
        self.app = None
        self.main_window = None
        
        # 初始化硬件接口
        self.hardware_interface = None

        # 初始化机械臂接口
        self.arm_interface = None
        self.joint_controller = None
        
        # 初始化管理器
        self.demo_manager = None
        
        # 初始化线程
        self.data_acquisition_thread = None
        self.control_thread = None
        
        # 初始化传感器管道
        self.sensor_pipeline = None
        
        # 初始化舵机控制器
        self.gripper_controller = None
        
        # 初始化深度学习模块
        self.deep_learning_signals = DeepLearningSignals()
        self.inference_engine = None
        self.neural_pid = None
        self.adaptive_pid = None
        self.online_loader = None
        self.grasp_classifier = None
        self.deep_learning_enabled = False
        
        # 性能统计
        self.inference_count = 0
        self.total_inference_time = 0.0
        self.avg_inference_time = 0.0
        
        # 设置信号处理
        self.setup_signal_handlers()

        # 添加性能优化参数
        self.last_gui_update_time = 0
        self.gui_update_interval = 0.033  # 约30Hz (33ms)
        self.last_data_received_time = 0
        self.data_rate = 0
        self.frame_count = 0
        self.start_time = time.time()
        
    def load_config(self, config_path: str, sensor_type: str) -> DemoConfig:
        """
        加载配置
        
        Args:
            config_path: 配置文件路径
            sensor_type: 传感器类型
            
        Returns:
            配置实例
        """
        if sensor_type == "3x3":
            self.logger.info("使用3x3传感器配置 (Paxini Gen3)")
            
            if config_path:
                # 从文件加载3x3配置
                return PaxiniGen3Config(config_path)
            else:
                # 创建默认3x3配置
                return create_3x3_config()
        else:
            self.logger.info("使用默认传感器配置")
            if config_path:
                try:
                    # 正确加载 YAML/JSON 配置文件，避免仅把路径写进 version 字段
                    return DemoConfig.load(config_path)
                except Exception as exc:
                    self.logger.warning(f"配置加载失败，改用默认配置: {exc}")
            return DemoConfig()
        
    def setup_logging(self):
        """配置日志系统"""
        log_dir = project_root / "logs"
        log_dir.mkdir(exist_ok=True)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_dir / 'tactile_gripper.log'),
                logging.StreamHandler()
            ]
        )
        
        self.logger = logging.getLogger(__name__)
        self.logger.info("触觉夹爪演示系统初始化...")
        
    def setup_signal_handlers(self):
        """设置信号处理器"""
        signal.signal(signal.SIGINT, self.signal_handler)  # Ctrl+C
        signal.signal(signal.SIGTERM, self.signal_handler)  # 终止信号
        
    def signal_handler(self, signum, frame):
        """信号处理函数"""
        self.logger.info(f"收到信号 {signum}, 正在关闭系统...")
        self.cleanup()
        sys.exit(0)
        
    def initialize_hardware(self):
        """初始化硬件接口"""
        try:
            self.logger.info("初始化硬件接口...")
            
            # 初始化硬件接口
            self.hardware_interface = HardwareInterface(
                config=self.config.hardware
            )
            
            # 根据传感器类型初始化触觉传感器管道
            if self.sensor_type == "3x3":
                # 使用3x3专用管道
                if hasattr(self.config, 'create_sensor_pipeline'):
                    self.sensor_pipeline = self.config.create_sensor_pipeline()
                else:
                    # 回退到默认管道，但使用3x3参数
                    self.sensor_pipeline = create_default_pipeline(
                        port=self.config.hardware.sensor.port,
                        baudrate=self.config.hardware.sensor.baudrate,
                        num_taxels=9,
                        rows=3,
                        cols=3
                    )
            else:
                # 使用默认管道
                self.sensor_pipeline = create_default_pipeline(
                    port=self.config.hardware.sensor.port,
                    baudrate=self.config.hardware.sensor.baudrate
                )
            
            # 初始化舵机控制器
            self.gripper_controller = GripperController(
                config=self.config.hardware.servo,
                hardware_interface=self.hardware_interface
            )

            # 初始化机械臂接口与关节控制器
            try:
                self.arm_interface = LearmInterface(self.config)
                self.joint_controller = JointController(self.arm_interface)
            except Exception as e:
                self.logger.warning(f"机械臂接口初始化失败: {e}")
                self.arm_interface = None
                self.joint_controller = None
            
            self.logger.info("硬件接口初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"硬件初始化失败: {e}")
            return False
    
    def initialize_deep_learning(self):
        """初始化深度学习模块"""
        try:
            if not DEEP_LEARNING_AVAILABLE:
                self.logger.warning("深度学习库不可用，跳过初始化")
                return True
            
            # 检查配置中是否启用了深度学习
            if not hasattr(self.config, 'deep_learning') or not self.config.deep_learning.enabled:
                self.logger.info("深度学习模块未启用")
                return True
            
            self.logger.info("初始化深度学习模块...")
            
            # 确定设备
            if torch.cuda.is_available() and self.config.deep_learning.use_gpu:
                device = torch.device('cuda')
                self.logger.info("使用GPU进行深度学习推理")
            else:
                device = torch.device('cpu')
                self.logger.info("使用CPU进行深度学习推理")
            
            # 创建推理引擎
            self.inference_engine = GraspInference(
                model_path=self.config.deep_learning.model_path,
                device=device,
                config=self.config
            )
            
            # 初始化PID神经网络控制器
            self.neural_pid = NeuralPIDController()
            self.neural_pid.to(device)
            
            # 初始化在线学习数据收集器
            self.online_loader = RealTimeDataLoader(
                buffer_size=self.config.deep_learning.online_learning_buffer_size
            )
            
            # 初始化抓取状态分类器
            self.grasp_classifier = GraspStateClassifier()
            self.grasp_classifier.to(device)
            
            # 初始化自适应PID
            self.adaptive_pid = AdaptivePID(
                base_kp=self.config.deep_learning.base_pid_parameters.get('position_kp', 1.0),
                base_ki=self.config.deep_learning.base_pid_parameters.get('position_ki', 0.1),
                base_kd=self.config.deep_learning.base_pid_parameters.get('position_kd', 0.05)
            )
            
            # 设置深度学习启用标志
            self.deep_learning_enabled = True
            
            # 性能监控
            self.inference_count = 0
            self.total_inference_time = 0.0
            self.avg_inference_time = 0.0
            
            self.logger.info("深度学习模块初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"深度学习模块初始化失败: {e}")
            self.deep_learning_enabled = False
            return True  # 深度学习模块失败不应阻止系统启动
        
    def initialize_modules(self):
        """初始化系统模块"""
        try:
            self.logger.info("初始化系统模块...")
            
            # 初始化深度学习模块
            if not self.initialize_deep_learning():
                self.logger.warning("深度学习模块初始化失败，继续其他模块")
            
            # 初始化数据采集线程
            self.data_acquisition_thread = DataAcquisitionThread(
                hardware_interface=self.hardware_interface,
                config=self.config
            )
            
            # 初始化控制线程
            self.control_thread = ControlThread(
                hardware_interface=self.hardware_interface,
                config=self.config,
                arm_interface=self.arm_interface,
                joint_controller=self.joint_controller
            )
            
            # 初始化演示管理器
            self.demo_manager = DemoManager(
                config=self.config,
                hardware_interface=self.hardware_interface,
                data_acquisition=self.data_acquisition_thread,
                control_thread=self.control_thread
            )
            
            # 如果启用了深度学习，设置回调函数
            if self.deep_learning_enabled:
                self.demo_manager.deep_learning_callback = self.deep_learning_inference_callback
                self.data_acquisition_thread.data_callback = self.process_tactile_data_callback
            
            self.logger.info("系统模块初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"模块初始化失败: {e}")
            return False
            
    def initialize_gui(self):
        """初始化GUI"""
        try:
            self.logger.info("初始化GUI...")
            
            # 创建Qt应用
            self.app = QApplication(sys.argv)
            self.app.setApplicationName("触觉夹爪演示系统")
            self.app.setApplicationVersion("2.0.0")
            
            # 设置全局字体
            font = QFont("Microsoft YaHei", 10)
            self.app.setFont(font)
            
            # 创建主窗口
            self.main_window = MainWindow(
                demo_manager=self.demo_manager,
                data_acquisition_thread=self.data_acquisition_thread,
                control_thread=self.control_thread,
                config=self.config
            )
            
            # 如果启用了深度学习，传递相关信息到GUI
            if self.deep_learning_enabled:
                self.main_window.deep_learning_enabled = True
                self.main_window.inference_engine = self.inference_engine
                self.main_window.deep_learning_signals = self.deep_learning_signals
            
            # 设置窗口标题和大小
            title = "触觉夹爪控制系统"
            if self.sensor_type == "3x3":
                title += " (Paxini Gen3 3x3)"
            if self.deep_learning_enabled:
                title += " [深度学习模式]"
            self.main_window.setWindowTitle(title)
            self.main_window.resize(self.config.ui.window_width, self.config.ui.window_height)
            
            self.logger.info("GUI初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"GUI初始化失败: {e}")
            return False
            
    def connect_signals(self):
        """连接信号和槽"""
         # 修改数据采集线程信号连接，使用QueuedConnection确保线程安全
        self.data_acquisition_thread.new_data.connect(
            self.handle_sensor_data,
            Qt.QueuedConnection  # 使用队列连接，避免阻塞
        )
        
        # 修改控制线程信号连接
        self.control_thread.status_updated.connect(
            self.main_window.update_control_status,
            Qt.QueuedConnection
        )
        
        # 连接GUI控制信号
        self.main_window.control_signal.connect(
            self.demo_manager.handle_control_command,
            Qt.QueuedConnection
        )
        
        # 连接演示管理器信号
        self.demo_manager.status_changed.connect(
            self.main_window.update_system_status,
            Qt.QueuedConnection
        )
        # 显示触觉映射结果（主线程）
        if hasattr(self.demo_manager, "tactile_mapping_ready"):
            self.demo_manager.tactile_mapping_ready.connect(
                self.main_window.show_tactile_mapping,
                Qt.QueuedConnection
            )
        # 将演示状态和进度同步到控制面板
        self.demo_manager.status_changed.connect(
            self.main_window.control_panel.update_control_status,
            Qt.QueuedConnection
        )
        self.demo_manager.demo_started.connect(
            lambda name, info: self.main_window.control_panel.update_control_status("demo_started", info),
            Qt.QueuedConnection
        )
        self.demo_manager.demo_stopped.connect(
            lambda name, info: self.main_window.control_panel.update_control_status("demo_stopped", info),
            Qt.QueuedConnection
        )
        self.demo_manager.demo_progress.connect(
            lambda progress, info: self.main_window.control_panel.update_control_status(
                "demo_progress", {"progress": progress, **(info or {})}
            ),
            Qt.QueuedConnection
        )
        
        # 连接深度学习信号
        if self.deep_learning_enabled:
            self.deep_learning_signals.inference_completed.connect(
                self.main_window.update_deep_learning_status
            )
            self.deep_learning_signals.error_occurred.connect(
                self.main_window.show_error_message
            )
        
        # 连接错误信号
        self.data_acquisition_thread.error_occurred.connect(
            lambda status, info: self.logger.error(f"数据采集错误: {info}")
        )
        
        self.control_thread.error_occurred.connect(
            lambda status, info: self.logger.error(f"控制错误: {info}")
        )
        
        self.demo_manager.error_occurred.connect(
            lambda status, info: self.logger.error(f"演示错误: {info}")
        )
        
        self.logger.info("信号连接完成")


    def handle_sensor_data(self, data):
        """
        处理传感器数据的回调函数，控制更新频率
        
        Args:
            data: 传感器数据
        """
        current_time = time.time()
        
        # 计算数据率
        self.frame_count += 1
        elapsed = current_time - self.start_time
        if elapsed > 1.0:  # 每秒更新一次数据率
            self.data_rate = self.frame_count / elapsed
            self.frame_count = 0
            self.start_time = current_time
            
            # 记录数据率（用于调试）
            if self.frame_count % 10 == 0:
                self.logger.debug(f"数据率: {self.data_rate:.1f} Hz")
        
        # 控制GUI更新频率（约30Hz）
        if current_time - self.last_gui_update_time >= self.gui_update_interval:
            self.last_gui_update_time = current_time
            
            # 处理触觉数据（如果有深度学习模块）
            processed_data = data
            control_instruction = None
            
            if self.deep_learning_enabled:
                processed_data, control_instruction = self.process_tactile_data_callback(data)
            
            # 更新GUI（在主线程中执行）
            self.main_window.update_sensor_data(processed_data)
            
            # 如果有控制指令，发送给控制线程
            if control_instruction:
                self.control_thread.send_command('adaptive_control', control_instruction)
        
        # 即使不更新GUI，也更新数据缓冲区（用于历史记录）
        self.update_data_buffer(data)
    
    def update_data_buffer(self, data):
        """更新数据缓冲区（不阻塞主线程）"""
        # 这里可以添加数据缓冲逻辑，但不要进行繁重计算
        pass
    
    def deep_learning_inference_callback(self, tactile_data, context):
        """
        深度学习推理回调函数
        
        Args:
            tactile_data: 触觉传感器数据
            context: 上下文信息（物体类型、抓取状态等）
        """
        if not self.deep_learning_enabled:
            return None
        
        try:
            # 记录推理开始时间
            start_time = time.time()
            
            # 获取物体信息
            object_type = context.get('object_type', 'unknown')
            object_shape = context.get('shape', 'cube')
            object_weight = context.get('weight', 0.1)
            
            # 进行推理
            inference_result = self.inference_engine.predict(
                object_type=object_type,
                shape=object_shape,
                weight=object_weight
            )
            
            # 计算推理时间
            inference_time = time.time() - start_time
            
            # 更新性能统计
            self.inference_count += 1
            self.total_inference_time += inference_time
            self.avg_inference_time = self.total_inference_time / self.inference_count
            
            # 记录到日志
            if self.inference_count % 10 == 0:
                self.logger.debug(f"深度学习推理统计: 次数={self.inference_count}, "
                                 f"平均时间={self.avg_inference_time*1000:.2f}ms")
            
            # 发送推理完成信号
            inference_result['performance']['inference_id'] = self.inference_count
            self.deep_learning_signals.inference_completed.emit(inference_result)
            
            return inference_result
            
        except Exception as e:
            self.logger.error(f"深度学习推理失败: {e}")
            self.deep_learning_signals.error_occurred.emit(
                "deep_learning_inference_error",
                str(e)
            )
            return None
    
    def process_tactile_data_callback(self, tactile_data):
        """
        处理触觉数据的回调函数
        
        Args:
            tactile_data: 触觉传感器数据
            
        Returns:
            处理后的数据和控制指令
        """
        if not self.deep_learning_enabled:
            return tactile_data, None
        
        try:
            # 添加到推理引擎缓冲区
            success = self.inference_engine.add_to_buffer(tactile_data)
            
            if not success:
                return tactile_data, None
            
            # 根据配置的推理间隔进行推理
            inference_interval = self.config.deep_learning.inference_interval
            if self.inference_count % inference_interval == 0:
                # 进行推理
                context = {
                    'object_type': self.get_current_object_type(),
                    'shape': 'cube',  # 可以从配置或GUI获取
                    'weight': 0.2  # 可以从配置或GUI获取
                }
                
                inference_result = self.deep_learning_inference_callback(tactile_data, context)
                
                if inference_result:
                    # 提取控制指令
                    control_instruction = {
                        'position': inference_result['position']['target'],
                        'pid_params': inference_result['pid_parameters'],
                        'grasp_state': inference_result['grasp_state']['state']
                    }
                    
                    return tactile_data, control_instruction
            
            return tactile_data, None
            
        except Exception as e:
            self.logger.error(f"触觉数据处理失败: {e}")
            return tactile_data, None
    
    def get_current_object_type(self):
        """获取当前物体类型"""
        # 可以从GUI、配置文件或传感器获取
        # 暂时返回默认值
        if hasattr(self.config, 'current_object'):
            return self.config.current_object.type
        else:
            return 'hard_plastic'
    
    def update_pid_parameters(self, pid_params):
        """更新PID参数"""
        if not self.deep_learning_enabled:
            return False
        
        try:
            if self.gripper_controller:
                # 更新舵机控制器的PID参数
                success = self.gripper_controller.update_pid_parameters(pid_params)
                
                if success:
                    self.logger.info(f"更新PID参数: {pid_params}")
                
                return success
            else:
                self.logger.warning("舵机控制器未初始化")
                return False
                
        except Exception as e:
            self.logger.error(f"更新PID参数失败: {e}")
            return False
    
    def adaptive_grasp_control(self, tactile_data, target_object=None):
        """自适应抓取控制"""
        if not self.deep_learning_enabled:
            return {'success': False, 'reason': '深度学习模块未启用'}
        
        try:
            # 进行推理
            context = {
                'object_type': target_object or self.get_current_object_type(),
                'shape': 'cube',
                'weight': 0.2
            }
            
            inference_result = self.deep_learning_inference_callback(tactile_data, context)
            
            if not inference_result:
                return {'success': False, 'reason': '推理失败'}
            
            # 更新PID参数
            self.update_pid_parameters(inference_result['pid_parameters'])
            
            # 获取控制信号
            control_signal = inference_result['position']
            
            # 检查抓取状态并调整控制
            grasp_state = inference_result['grasp_state']
            state = grasp_state['state']
            confidence = grasp_state['confidence']
            
            # 根据状态调整控制
            if state == 'slip' and confidence > 0.7:
                # 滑动时增加抓取力
                control_signal['target'] *= 1.2
                self.logger.warning(f"检测到滑动(置信度{confidence:.2f})，增加抓取力20%")
            elif state == 'tight' and confidence > 0.7:
                # 过紧时减小抓取力
                control_signal['target'] *= 0.8
                self.logger.warning(f"抓取过紧(置信度{confidence:.2f})，减小抓取力20%")
            
            # 发送控制指令
            if self.control_thread:
                self.control_thread.send_command(
                    'adaptive_grasp',
                    {
                        'position': control_signal['target'],
                        'velocity': control_signal['velocity'],
                        'acceleration': control_signal['acceleration'],
                        'grasp_state': state,
                        'confidence': confidence
                    }
                )
            
            # 返回结果
            return {
                'success': True,
                'control_signal': control_signal,
                'grasp_state': grasp_state,
                'pid_parameters': inference_result['pid_parameters'],
                'inference_time': inference_result['performance']['inference_time_ms']
            }
            
        except Exception as e:
            self.logger.error(f"自适应抓取控制失败: {e}")
            return {'success': False, 'reason': str(e)}
    
    def collect_training_data(self, tactile_data, control_signal, 
                             object_info, grasp_success):
        """收集训练数据"""
        if not self.deep_learning_enabled or not self.online_loader:
            return False
        
        try:
            # 添加到在线学习缓冲区
            self.online_loader.add_sample(
                tactile_data=tactile_data,
                control_signal=control_signal,
                object_info=object_info,
                grasp_success=grasp_success
            )
            
            # 定期进行在线学习
            buffer_size = len(self.online_loader.buffer)
            if buffer_size >= 100 and self.inference_count % 50 == 0:
                self.online_learn_step()
                self.logger.info(f"在线学习: 缓冲区大小={buffer_size}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"收集训练数据失败: {e}")
            return False
    
    def online_learn_step(self):
        """在线学习步骤"""
        if not self.deep_learning_enabled or not self.online_loader:
            return False
        
        try:
            # 获取批次数据
            batch = self.online_loader.get_batch(32)
            
            if batch:
                # 这里可以添加在线学习逻辑
                # 例如：使用强化学习或增量学习更新模型
                
                # 统计成功抓取
                successes = batch['success'].cpu().numpy()
                success_count = np.sum(successes > 0.5)
                
                if success_count > 0:
                    self.logger.info(f"在线学习: 发现{success_count}个成功抓取样本")
                    # TODO: 实现实际的在线学习算法
                
                return True
            
        except Exception as e:
            self.logger.error(f"在线学习失败: {e}")
        
        return False
    
    def start(self):
        """启动系统"""
        try:
            self.logger.info("启动触觉夹爪演示系统...")
            
            # 初始化硬件
            if not self.initialize_hardware():
                raise RuntimeError("硬件初始化失败")
                
            # 初始化模块
            if not self.initialize_modules():
                raise RuntimeError("模块初始化失败")
                
            # 初始化GUI
            if not self.initialize_gui():
                raise RuntimeError("GUI初始化失败")
                
            # 连接信号
            self.connect_signals()
            
            # 启动系统
            self.is_running = True
            
            # 启动数据采集线程
            self.data_acquisition_thread.start()
            
            # 启动控制线程
            self.control_thread.start()
            
            # 显示主窗口
            self.main_window.show()
            
            # 记录系统状态
            self.log_system_status()
            
            # 启动Qt事件循环
            self.logger.info("系统启动完成，进入主循环")
            return_code = self.app.exec_()
            
            # 清理资源
            self.cleanup()
            
            return return_code
            
        except Exception as e:
            self.logger.error(f"系统启动失败: {e}")
            import traceback
            traceback.print_exc()
            self.cleanup()
            return 1
    
    def log_system_status(self):
        """记录系统状态"""
        status = {
            "sensor_type": self.sensor_type,
            "deep_learning_enabled": self.deep_learning_enabled,
            "hardware_initialized": self.hardware_interface is not None,
            "gui_initialized": self.main_window is not None,
            "threads_running": self.data_acquisition_thread.isRunning() if self.data_acquisition_thread else False
        }
        
        self.logger.info(f"系统状态: {status}")
            
    def cleanup(self):
        """清理资源"""
        if not self.is_running:
            return
            
        self.logger.info("清理系统资源...")
        
        # 停止线程
        if self.data_acquisition_thread:
            self.data_acquisition_thread.stop_acquisition()
            self.data_acquisition_thread.wait()
            
        if self.control_thread:
            self.control_thread.stop_control()
            self.control_thread.wait()
            
        # 关闭硬件接口
        if self.hardware_interface:
            self.hardware_interface.close()

        # 断开机械臂连接
        if self.arm_interface:
            try:
                self.arm_interface.disconnect()
            except Exception as e:
                self.logger.warning(f"断开机械臂时出错: {e}")
            
        # 关闭传感器管道
        if self.sensor_pipeline:
            if 'reader' in self.sensor_pipeline:
                reader = self.sensor_pipeline['reader']
                # reader 可能是 dict 或对象，尽量安全关闭
                try:
                    if hasattr(reader, "close"):
                        reader.close()
                except Exception as e:
                    self.logger.warning(f"关闭传感器管道 reader 时出错: {e}")
        
        # 清理深度学习资源
        if self.deep_learning_enabled:
            self.logger.info("清理深度学习资源...")
            # 保存在线学习数据（如果启用）
            if self.online_loader and len(self.online_loader.buffer) > 0:
                self.save_online_learning_data()
                
        self.is_running = False
        self.logger.info("资源清理完成")
    
    def save_online_learning_data(self):
        """保存在线学习数据"""
        try:
            import h5py
            from datetime import datetime
            
            # 创建数据目录
            data_dir = project_root / "data" / "online_learning"
            data_dir.mkdir(parents=True, exist_ok=True)
            
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = data_dir / f"online_learning_{timestamp}.h5"
            
            # 保存数据
            with h5py.File(filename, 'w') as f:
                # 保存每个样本
                for i, sample in enumerate(self.online_loader.buffer):
                    group = f.create_group(f"sample_{i:04d}")
                    group.create_dataset("tactile", data=sample['tactile'])
                    group.create_dataset("control", data=sample['control'])
                    group.attrs['object_info'] = json.dumps(sample['object_info'])
                    group.attrs['grasp_success'] = sample['success']
                
                # 保存元数据
                f.attrs['num_samples'] = len(self.online_loader.buffer)
                f.attrs['save_time'] = timestamp
                f.attrs['inference_count'] = self.inference_count
                f.attrs['avg_inference_time'] = self.avg_inference_time
            
            self.logger.info(f"在线学习数据已保存: {filename} ({len(self.online_loader.buffer)}个样本)")
            
        except Exception as e:
            self.logger.error(f"保存在线学习数据失败: {e}")
        
    def run_demo_mode(self, demo_name: str, params: Dict[str, Any] = None):
        """
        运行演示模式
        
        Args:
            demo_name: 演示名称
            params: 演示参数
        """
        if not self.is_running:
            self.logger.error("系统未启动")
            return
            
        self.demo_manager.start_demo(demo_name, params)
        
    def emergency_stop(self):
        """紧急停止"""
        self.logger.warning("执行紧急停止")
        
        # 停止所有控制
        if self.control_thread:
            self.control_thread.send_command("emergency_stop", {})
            
        # 停止演示
        if self.demo_manager:
            self.demo_manager.stop_demo()
            
        # 关闭夹爪
        if self.gripper_controller:
            self.gripper_controller.emergency_stop()
            
        # 重置深度学习状态
        if self.deep_learning_enabled:
            self.inference_engine.reset_buffer()
            
        self.logger.info("紧急停止完成")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="触觉夹爪演示系统 v2.1.0")
    parser.add_argument("--config", type=str, default=None,
                       help="配置文件路径")
    parser.add_argument("--sensor-type", type=str, default="3x3",
                       choices=["default", "3x3"],
                       help="传感器类型: 'default' 或 '3x3' (默认3x3)")
    parser.add_argument("--sensor-port", type=str, default="COM3",
                       help="传感器串口 (默认: COM3)")
    parser.add_argument("--servo-port", type=str, default="COM4",
                       help="舵机串口 (默认: COM4)")
    parser.add_argument("--demo", type=str, default=None,
                       help="演示模式名称")
    parser.add_argument("--params", type=str, default=None,
                       help="演示参数（JSON格式）")
    parser.add_argument("--log-level", type=str, default="INFO",
                       choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                       help="日志级别")
    parser.add_argument("--deep-learning", action="store_true",
                       help="启用深度学习模块")
    parser.add_argument("--model-path", type=str, default=None,
                       help="深度学习模型路径")
    parser.add_argument("--object-type", type=str, default="hard_plastic",
                       choices=["hard_plastic", "soft_rubber", "fragile_glass", "metal", "fabric"],
                       help="目标物体类型")
    
    args = parser.parse_args()
    
    # 设置日志级别
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    try:
        # 创建演示系统实例
        demo_system = TactileGripperDemo(
            config_path=args.config,
            sensor_type=args.sensor_type
        )
        
        # 如果指定了端口，更新配置
        if args.sensor_port and demo_system.config:
            demo_system.config.hardware.sensor.port = args.sensor_port
            
        if args.servo_port and demo_system.config:
            demo_system.config.hardware.servo.port = args.servo_port
        
        # 如果启用了深度学习，更新配置
        if args.deep_learning:
            if not DEEP_LEARNING_AVAILABLE:
                print("警告: 深度学习库不可用，无法启用深度学习模块")
            else:
                # 确保配置中有深度学习部分
                if not hasattr(demo_system.config, 'deep_learning'):
                    from config.deep_learning_config import create_default_dl_config
                    demo_system.config.deep_learning = create_default_dl_config()
                
                # 启用深度学习
                demo_system.config.deep_learning.enabled = True
                
                # 如果指定了模型路径，更新配置
                if args.model_path:
                    demo_system.config.deep_learning.model_path = args.model_path
                
                print(f"深度学习模块已启用 (物体类型: {args.object_type})")
        
        # 解析演示参数
        params = {}
        if args.params:
            import json
            params = json.loads(args.params)
            
        # 启动系统
        if args.demo:
            # 演示模式
            demo_system.start()
            demo_system.run_demo_mode(args.demo, params)
        else:
            # 正常启动
            return_code = demo_system.start()
            sys.exit(return_code)
            
    except KeyboardInterrupt:
        print("\n用户中断，正在关闭...")
        sys.exit(0)
    except Exception as e:
        print(f"系统运行出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
