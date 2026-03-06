"""
机械臂状态面板 - 显示机械臂关节状态、末端位姿、力/扭矩等信息
"""

import numpy as np
from typing import Dict, Any, Optional, List, Tuple
import time

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox,
    QPushButton, QCheckBox, QSpinBox, QDoubleSpinBox, QSlider,
    QFormLayout, QTableWidget, QTableWidgetItem, QHeaderView,
    QProgressBar, QComboBox, QSplitter, QTabWidget, QStyle, QStyleOptionSlider,
    QScrollArea, QFrame, QGridLayout, QTreeWidget, QTreeWidgetItem
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer, QSize
from PyQt5.QtGui import QFont, QColor, QPen, QBrush, QPainter, QLinearGradient

try:
    from config import DemoConfig, LearmArmConfig
except ImportError:
    try:
        from ..config import DemoConfig, LearmArmConfig
    except ImportError:
        import sys
        import os
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from config import DemoConfig, LearmArmConfig


class JointStatusWidget(QWidget):
    """关节状态部件"""
    
    home_requested = pyqtSignal()
    enable_requested = pyqtSignal()
    disable_requested = pyqtSignal()
    joint_angle_commanded = pyqtSignal(int, float)
    
    def __init__(self, num_joints=6, joint_limits_min=None, joint_limits_max=None, parent=None):
        """
        初始化关节状态部件
        
        Args:
            num_joints: 关节数量
            joint_limits_min: 关节最小角度列表（度）
            joint_limits_max: 关节最大角度列表（度）
            parent: 父部件
        """
        super().__init__(parent)
        
        self.num_joints = num_joints
        
        # 关节数据
        self.joint_positions = np.zeros(num_joints)
        self.joint_velocities = np.zeros(num_joints)
        self.joint_torques = np.zeros(num_joints)
        self.joint_targets = np.zeros(num_joints)
        self.joint_errors = np.zeros(num_joints)
        
        # 关节限制（角度）
        if joint_limits_min is None:
            joint_limits_min = [-125.0] * num_joints
        if joint_limits_max is None:
            joint_limits_max = [125.0] * num_joints
        self.joint_limits_min = np.array(joint_limits_min, dtype=float)
        self.joint_limits_max = np.array(joint_limits_max, dtype=float)
        # 全局范围用于范围线段比例显示
        self._range_global_min = float(np.min(self.joint_limits_min))
        self._range_global_max = float(np.max(self.joint_limits_max))

        # 控制输入
        self._control_change_guard = False
        self._pending_joint_targets = [0.0] * num_joints
        self._joint_send_timers = []
        
        # 初始化UI
        self.init_ui()
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # 创建标签页
        self.tab_widget = QTabWidget()
        
        # 表格视图标签页
        table_tab = QWidget()
        table_layout = QVBoxLayout(table_tab)
        
        # 创建关节状态表格
        self.joint_table = QTableWidget(self.num_joints, 7)
        self.joint_table.setHorizontalHeaderLabels([
            "关节", "当前位置 (°)", "目标位置 (°)", "误差 (°)", 
            "速度 (°/s)", "扭矩 (Nm)", "状态"
        ])
        
        # 设置表格属性
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.joint_table.setAlternatingRowColors(True)
        self.joint_table.setStyleSheet("""
            QTableWidget {
                gridline-color: #cccccc;
            }
            QTableWidget::item {
                padding: 5px;
            }
        """)
        
        # 初始化表格数据
        for i in range(self.num_joints):
            # 关节编号
            joint_item = QTableWidgetItem(f"J{i+1}")
            joint_item.setTextAlignment(Qt.AlignCenter)
            self.joint_table.setItem(i, 0, joint_item)
            
            # 其他列
            for col in range(1, 7):
                item = QTableWidgetItem("0.00")
                item.setTextAlignment(Qt.AlignCenter)
                self.joint_table.setItem(i, col, item)
        
        table_layout.addWidget(self.joint_table)
        self.tab_widget.addTab(table_tab, "表格视图")
        
        # 仪表盘视图标签页
        gauge_tab = QWidget()
        gauge_layout = QVBoxLayout(gauge_tab)
        
        # 创建关节仪表盘网格
        gauge_grid = QGridLayout()
        
        self.joint_gauges = []
        for i in range(self.num_joints):
            gauge = JointGaugeWidget(f"J{i+1}", parent=self)
            row = i // 3
            col = i % 3
            gauge_grid.addWidget(gauge, row, col)
            self.joint_gauges.append(gauge)
        
        gauge_layout.addLayout(gauge_grid)
        self.tab_widget.addTab(gauge_tab, "仪表盘视图")
        
        layout.addWidget(self.tab_widget)

        # 关节角度控制
        control_group = QGroupBox("关节角度控制")
        control_layout = QGridLayout(control_group)
        control_layout.setContentsMargins(6, 6, 6, 6)
        control_layout.setHorizontalSpacing(8)
        control_layout.setVerticalSpacing(6)

        self.joint_control_sliders = []
        self.joint_control_spins = []
        slider_scale = 10  # 0.1度分辨率

        for i in range(self.num_joints):
            joint_label = QLabel(f"J{i+1}")
            joint_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            slider = RangeSlider(
                Qt.Horizontal,
                min_deg=float(self.joint_limits_min[i]),
                max_deg=float(self.joint_limits_max[i]),
                global_min=self._range_global_min,
                global_max=self._range_global_max,
                scale=slider_scale,
                parent=self,
            )
            slider.setRange(
                int(self._range_global_min * slider_scale),
                int(self._range_global_max * slider_scale),
            )
            slider.setSingleStep(1)
            slider.setPageStep(5)

            spin = QDoubleSpinBox()
            spin.setRange(self.joint_limits_min[i], self.joint_limits_max[i])
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setValue(0.0)

            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.setInterval(50)
            timer.timeout.connect(lambda idx=i: self._emit_pending_joint_command(idx))

            self.joint_control_sliders.append(slider)
            self.joint_control_spins.append(spin)
            self._joint_send_timers.append(timer)

            slider.valueChanged.connect(lambda value, idx=i: self._on_joint_slider_changed(idx, value, slider_scale))
            spin.valueChanged.connect(lambda value, idx=i: self._on_joint_spin_changed(idx, value, slider_scale))

            control_layout.addWidget(joint_label, i, 0)
            control_layout.addWidget(slider, i, 1)
            control_layout.addWidget(spin, i, 2)

        layout.addWidget(control_group)
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        self.home_btn = QPushButton("回零")
        self.home_btn.clicked.connect(self.home_arm)
        control_layout.addWidget(self.home_btn)
        
        self.enable_btn = QPushButton("使能所有关节")
        self.enable_btn.clicked.connect(self.enable_arm)
        control_layout.addWidget(self.enable_btn)
        
        self.disable_btn = QPushButton("禁用所有关节")
        self.disable_btn.clicked.connect(self.disable_arm)
        control_layout.addWidget(self.disable_btn)
        
        control_layout.addStretch()
        
        layout.addLayout(control_layout)

    def update_connection_state(self, connected: bool, enabled: bool):
        """根据连接状态更新控制可用性"""
        can_control = bool(connected and enabled)
        for slider in self.joint_control_sliders:
            slider.setEnabled(can_control)
        for spin in self.joint_control_spins:
            spin.setEnabled(can_control)

        self.home_btn.setEnabled(bool(connected))
        self.enable_btn.setEnabled(bool(connected and not enabled))
        self.disable_btn.setEnabled(bool(connected and enabled))
    
    def update_joint_data(self, positions, velocities=None, torques=None, targets=None):
        """
        更新关节数据
        
        Args:
            positions: 关节位置数组
            velocities: 关节速度数组
            torques: 关节扭矩数组
            targets: 关节目标位置数组
        """
        # 更新数据（角度）
        self.joint_positions = np.array(positions, dtype=float)
        
        if velocities is not None:
            self.joint_velocities = np.array(velocities, dtype=float)
        
        if torques is not None:
            self.joint_torques = np.array(torques, dtype=float)
        
        if targets is not None:
            self.joint_targets = np.array(targets, dtype=float)
        elif np.allclose(self.joint_targets, 0.0):
            self.joint_targets = self.joint_positions.copy()

        self.joint_errors = self.joint_targets - self.joint_positions

        # 同步控制输入（避免与用户拖动冲突）
        self._sync_control_inputs(self.joint_positions)
        
        # 更新表格
        self.update_table()
        
        # 更新仪表盘
        self.update_gauges()
    
    def update_table(self):
        """更新表格数据"""
        for i in range(self.num_joints):
            # 当前位置
            pos_item = self.joint_table.item(i, 1)
            pos_item.setText(f"{self.joint_positions[i]:.2f}")
            
            # 目标位置
            target_item = self.joint_table.item(i, 2)
            target_item.setText(f"{self.joint_targets[i]:.2f}")
            
            # 误差
            error_item = self.joint_table.item(i, 3)
            error_deg = self.joint_errors[i]
            error_item.setText(f"{error_deg:.2f}")
            
            # 根据误差设置颜色
            error_threshold = 1.0  # 1度阈值
            if abs(error_deg) > error_threshold:
                error_item.setBackground(QColor(255, 200, 200))
            else:
                error_item.setBackground(QColor(200, 255, 200))
            
            # 速度
            if i < len(self.joint_velocities):
                vel_item = self.joint_table.item(i, 4)
                vel_item.setText(f"{self.joint_velocities[i]:.2f}")
            
            # 扭矩
            if i < len(self.joint_torques):
                torque_item = self.joint_table.item(i, 5)
                torque_item.setText(f"{self.joint_torques[i]:.2f}")
                
                # 根据扭矩设置颜色（假设限制为10Nm）
                torque_limit = 10.0
                torque_ratio = abs(self.joint_torques[i]) / torque_limit
                if torque_ratio > 0.8:
                    torque_item.setBackground(QColor(255, 100, 100))
                elif torque_ratio > 0.5:
                    torque_item.setBackground(QColor(255, 200, 100))
                else:
                    torque_item.setBackground(QColor(200, 255, 200))
            
            # 状态
            status_item = self.joint_table.item(i, 6)
            
            # 简化的状态判断
            position_in_limit = (
                self.joint_limits_min[i] <= self.joint_positions[i] <= self.joint_limits_max[i]
            )
            error_small = abs(self.joint_errors[i]) < 1.0  # 1度阈值
            
            if not position_in_limit:
                status_item.setText("超限")
                status_item.setBackground(QColor(255, 100, 100))
            elif error_small:
                status_item.setText("就绪")
                status_item.setBackground(QColor(100, 255, 100))
            else:
                status_item.setText("运动中")
                status_item.setBackground(QColor(255, 255, 100))
    
    def update_gauges(self):
        """更新仪表盘"""
        for i, gauge in enumerate(self.joint_gauges):
            if i < len(self.joint_positions):
                # 计算归一化位置 (0-100%)
                pos_deg = self.joint_positions[i]
                min_deg = self.joint_limits_min[i]
                max_deg = self.joint_limits_max[i]
                
                if max_deg > min_deg:
                    normalized = (pos_deg - min_deg) / (max_deg - min_deg) * 100
                else:
                    normalized = 50
                
                # 更新仪表盘
                gauge.update_value(normalized, pos_deg)

    def _on_joint_slider_changed(self, joint_index: int, value: int, scale: int):
        if self._control_change_guard:
            return
        angle = value / float(scale)
        min_deg = float(self.joint_limits_min[joint_index])
        max_deg = float(self.joint_limits_max[joint_index])
        if angle < min_deg or angle > max_deg:
            angle = max(min_deg, min(max_deg, angle))
            value = int(round(angle * scale))
        self._control_change_guard = True
        try:
            self.joint_control_spins[joint_index].setValue(angle)
            self.joint_control_sliders[joint_index].setValue(value)
        finally:
            self._control_change_guard = False
        self._queue_joint_command(joint_index, angle)

    def _on_joint_spin_changed(self, joint_index: int, value: float, scale: int):
        if self._control_change_guard:
            return
        min_deg = float(self.joint_limits_min[joint_index])
        max_deg = float(self.joint_limits_max[joint_index])
        if value < min_deg or value > max_deg:
            value = max(min_deg, min(max_deg, float(value)))
        slider_value = int(round(value * scale))
        self._control_change_guard = True
        try:
            self.joint_control_sliders[joint_index].setValue(slider_value)
            self.joint_control_spins[joint_index].setValue(value)
        finally:
            self._control_change_guard = False
        self._queue_joint_command(joint_index, value)

    def _queue_joint_command(self, joint_index: int, angle: float):
        self._pending_joint_targets[joint_index] = float(angle)
        timer = self._joint_send_timers[joint_index]
        timer.start()
        self.joint_targets[joint_index] = float(angle)
        self.joint_errors[joint_index] = self.joint_targets[joint_index] - self.joint_positions[joint_index]
        self.update_table()

    def _emit_pending_joint_command(self, joint_index: int):
        angle = self._pending_joint_targets[joint_index]
        self.joint_angle_commanded.emit(joint_index, float(angle))

    def _sync_control_inputs(self, positions: np.ndarray):
        self._control_change_guard = True
        try:
            for i in range(self.num_joints):
                slider = self.joint_control_sliders[i]
                spin = self.joint_control_spins[i]
                if slider.isSliderDown() or spin.hasFocus():
                    continue
                slider_scale = 10
                min_deg = float(self.joint_limits_min[i])
                max_deg = float(self.joint_limits_max[i])
                angle = max(min_deg, min(max_deg, float(positions[i])))
                slider_value = int(round(angle * slider_scale))
                slider.setValue(slider_value)
                spin.setValue(float(angle))
        finally:
            self._control_change_guard = False

    def home_arm(self):
        """请求回零，由上层处理"""
        self.home_requested.emit()

    def enable_arm(self):
        """请求使能，由上层处理"""
        self.enable_requested.emit()

    def disable_arm(self):
        """请求禁用，由上层处理"""
        self.disable_requested.emit()


class JointGaugeWidget(QWidget):
    """关节仪表盘部件"""
    
    def __init__(self, joint_name="J1", parent=None):
        """
        初始化关节仪表盘部件
        
        Args:
            joint_name: 关节名称
            parent: 父部件
        """
        super().__init__(parent)
        
        self.joint_name = joint_name
        self.current_value = 50  # 0-100%
        self.current_angle = 0.0  # 角度值
        
        # 设置固定大小
        self.setMinimumSize(150, 150)
        
        # 颜色
        self.bg_color = QColor(240, 240, 240)
        self.gauge_color = QColor(70, 130, 180)
        self.text_color = QColor(50, 50, 50)
        
    def update_value(self, value, angle):
        """
        更新仪表盘值
        
        Args:
            value: 归一化值 (0-100%)
            angle: 角度值
        """
        self.current_value = max(0, min(100, value))
        self.current_angle = angle
        self.update()
    
    def paintEvent(self, event):
        """绘制事件"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 获取部件尺寸
        width = self.width()
        height = self.height()
        size = min(width, height) - 20
        center_x = width // 2
        center_y = height // 2
        
        # 绘制背景
        painter.fillRect(0, 0, width, height, self.bg_color)
        
        # 绘制外圆
        painter.setPen(QPen(QColor(200, 200, 200), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center_x - size//2, center_y - size//2, size, size)
        
        # 绘制刻度
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        for i in range(0, 360, 30):
            angle_rad = np.radians(i - 90)  # 从顶部开始
            inner_radius = size // 2 - 10
            outer_radius = size // 2
            
            x1 = center_x + inner_radius * np.cos(angle_rad)
            y1 = center_y + inner_radius * np.sin(angle_rad)
            x2 = center_x + outer_radius * np.cos(angle_rad)
            y2 = center_y + outer_radius * np.sin(angle_rad)
            
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            
            # 刻度标签
            if i % 90 == 0:
                label_radius = outer_radius + 15
                x_label = center_x + label_radius * np.cos(angle_rad)
                y_label = center_y + label_radius * np.sin(angle_rad)
                
                painter.setPen(QPen(self.text_color, 1))
                painter.drawText(int(x_label - 10), int(y_label - 10), 20, 20, 
                               Qt.AlignCenter, f"{i}°")
        
        # 绘制仪表盘弧
        start_angle = 30 * 16  # 从30度开始 (240度弧长)
        span_angle = -300 * 16  # 300度弧长
        
        # 创建渐变
        gradient = QLinearGradient(0, 0, width, height)
        gradient.setColorAt(0.0, QColor(100, 200, 100))
        gradient.setColorAt(0.5, QColor(200, 200, 100))
        gradient.setColorAt(1.0, QColor(255, 100, 100))
        
        painter.setPen(QPen(QColor(150, 150, 150), 3))
        painter.setBrush(Qt.NoBrush)
        painter.drawArc(center_x - size//2 + 5, center_y - size//2 + 5, 
                       size - 10, size - 10, start_angle, span_angle)
        
        # 绘制当前值指示器
        indicator_angle = 30 + (self.current_value / 100) * 300  # 映射到30-330度
        indicator_angle_rad = np.radians(indicator_angle - 90)
        
        indicator_length = size // 2 - 15
        x_indicator = center_x + indicator_length * np.cos(indicator_angle_rad)
        y_indicator = center_y + indicator_length * np.sin(indicator_angle_rad)
        
        painter.setPen(QPen(QColor(255, 0, 0), 3))
        painter.drawLine(center_x, center_y, int(x_indicator), int(y_indicator))
        
        # 绘制中心点
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(200, 200, 200)))
        painter.drawEllipse(center_x - 5, center_y - 5, 10, 10)
        
        # 绘制关节名称和当前值
        painter.setPen(QPen(self.text_color, 2))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        painter.drawText(center_x - 50, center_y - 40, 100, 20, 
                        Qt.AlignCenter, self.joint_name)
        
        painter.setFont(QFont("Arial", 12))
        painter.drawText(center_x - 50, center_y + 20, 100, 30, 
                        Qt.AlignCenter, f"{self.current_angle:.1f}°")
        
        # 绘制状态指示器
        status_color = QColor(100, 255, 100)  # 绿色表示正常
        if abs(self.current_angle) > 150:  # 角度过大
            status_color = QColor(255, 100, 100)  # 红色
        
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(status_color))
        painter.drawEllipse(center_x - 8, center_y - size//2 + 10, 16, 16)


class RangeSlider(QSlider):
    """带范围线段的滑条（线段覆盖轨道，表示可调范围）。"""

    def __init__(
        self,
        orientation: Qt.Orientation,
        min_deg: float,
        max_deg: float,
        global_min: float,
        global_max: float,
        scale: int = 10,
        parent=None,
    ):
        super().__init__(orientation, parent)
        self._allowed_min = float(min_deg)
        self._allowed_max = float(max_deg)
        self._global_min = float(global_min)
        self._global_max = float(global_max)
        self._scale = float(scale)
        self._range_color = QColor(60, 180, 100)
        self._range_width = 6

    def set_range_limits(
        self,
        min_deg: float,
        max_deg: float,
        global_min: float = None,
        global_max: float = None,
    ):
        """更新范围参数并重绘。"""
        self._allowed_min = float(min_deg)
        self._allowed_max = float(max_deg)
        if global_min is not None:
            self._global_min = float(global_min)
        if global_max is not None:
            self._global_max = float(global_max)
        self.update()

    def _clamp_value(self, value: int) -> int:
        min_value = int(round(self._allowed_min * self._scale))
        max_value = int(round(self._allowed_max * self._scale))
        if value < min_value:
            return min_value
        if value > max_value:
            return max_value
        return int(value)

    def setValue(self, value: int) -> None:
        super().setValue(self._clamp_value(int(value)))

    def paintEvent(self, event):
        super().paintEvent(event)

        opt = QStyleOptionSlider()
        self.initStyleOption(opt)
        groove = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderGroove, self)
        if not groove.isValid():
            return

        span = self._global_max - self._global_min
        if span <= 0:
            return

        min_norm = (self._allowed_min - self._global_min) / span
        max_norm = (self._allowed_max - self._global_min) / span
        min_norm = max(0.0, min(1.0, min_norm))
        max_norm = max(0.0, min(1.0, max_norm))
        if max_norm < min_norm:
            min_norm, max_norm = max_norm, min_norm

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(self._range_color, self._range_width, Qt.SolidLine, Qt.RoundCap))

        if self.orientation() == Qt.Horizontal:
            start_x = groove.left() + int(groove.width() * min_norm)
            end_x = groove.left() + int(groove.width() * max_norm)
            y = groove.center().y()
            painter.drawLine(start_x, y, end_x, y)
        else:
            start_y = groove.bottom() - int(groove.height() * min_norm)
            end_y = groove.bottom() - int(groove.height() * max_norm)
            x = groove.center().x()
            painter.drawLine(x, start_y, x, end_y)


class EndEffectorWidget(QWidget):
    """末端执行器状态部件"""

    # 末端执行器“回零”请求信号（由上层统一处理）
    home_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        """
        初始化末端执行器状态部件
        
        Args:
            parent: 父部件
        """
        super().__init__(parent)
        
        # 末端执行器数据
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion (x, y, z, w)
        self.velocity_linear = [0.0, 0.0, 0.0]
        self.velocity_angular = [0.0, 0.0, 0.0]
        self.force = [0.0, 0.0, 0.0]
        self.torque = [0.0, 0.0, 0.0]
        
        # 目标位姿
        self.target_position = [0.0, 0.0, 0.0]
        self.target_orientation = [0.0, 0.0, 0.0, 1.0]
        
        # 初始化UI
        self.init_ui()
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)
        
        # 位置和姿态组
        pose_group = QGroupBox("位置和姿态")
        pose_layout = QFormLayout(pose_group)
        
        # 位置显示
        self.pos_x_label = QLabel("0.000")
        self.pos_y_label = QLabel("0.000")
        self.pos_z_label = QLabel("0.000")
        
        pos_layout = QHBoxLayout()
        pos_layout.addWidget(QLabel("X:"))
        pos_layout.addWidget(self.pos_x_label)
        pos_layout.addWidget(QLabel("Y:"))
        pos_layout.addWidget(self.pos_y_label)
        pos_layout.addWidget(QLabel("Z:"))
        pos_layout.addWidget(self.pos_z_label)
        
        pose_layout.addRow("位置 (m):", pos_layout)
        
        # 姿态显示 (欧拉角)
        self.roll_label = QLabel("0.00")
        self.pitch_label = QLabel("0.00")
        self.yaw_label = QLabel("0.00")
        
        orient_layout = QHBoxLayout()
        orient_layout.addWidget(QLabel("Roll:"))
        orient_layout.addWidget(self.roll_label)
        orient_layout.addWidget(QLabel("Pitch:"))
        orient_layout.addWidget(self.pitch_label)
        orient_layout.addWidget(QLabel("Yaw:"))
        orient_layout.addWidget(self.yaw_label)
        
        pose_layout.addRow("姿态 (°):", orient_layout)
        
        # 四元数显示
        self.quat_x_label = QLabel("0.000")
        self.quat_y_label = QLabel("0.000")
        self.quat_z_label = QLabel("0.000")
        self.quat_w_label = QLabel("1.000")
        
        quat_layout = QHBoxLayout()
        quat_layout.addWidget(QLabel("X:"))
        quat_layout.addWidget(self.quat_x_label)
        quat_layout.addWidget(QLabel("Y:"))
        quat_layout.addWidget(self.quat_y_label)
        quat_layout.addWidget(QLabel("Z:"))
        quat_layout.addWidget(self.quat_z_label)
        quat_layout.addWidget(QLabel("W:"))
        quat_layout.addWidget(self.quat_w_label)
        
        pose_layout.addRow("四元数:", quat_layout)
        
        layout.addWidget(pose_group)
        
        # 速度和力组
        velocity_force_group = QGroupBox("速度和力")
        vf_layout = QFormLayout(velocity_force_group)
        
        # 线速度
        self.lin_vel_x_label = QLabel("0.000")
        self.lin_vel_y_label = QLabel("0.000")
        self.lin_vel_z_label = QLabel("0.000")
        
        lin_vel_layout = QHBoxLayout()
        lin_vel_layout.addWidget(QLabel("X:"))
        lin_vel_layout.addWidget(self.lin_vel_x_label)
        lin_vel_layout.addWidget(QLabel("Y:"))
        lin_vel_layout.addWidget(self.lin_vel_y_label)
        lin_vel_layout.addWidget(QLabel("Z:"))
        lin_vel_layout.addWidget(self.lin_vel_z_label)
        
        vf_layout.addRow("线速度 (m/s):", lin_vel_layout)
        
        # 角速度
        self.ang_vel_x_label = QLabel("0.000")
        self.ang_vel_y_label = QLabel("0.000")
        self.ang_vel_z_label = QLabel("0.000")
        
        ang_vel_layout = QHBoxLayout()
        ang_vel_layout.addWidget(QLabel("X:"))
        ang_vel_layout.addWidget(self.ang_vel_x_label)
        ang_vel_layout.addWidget(QLabel("Y:"))
        ang_vel_layout.addWidget(self.ang_vel_y_label)
        ang_vel_layout.addWidget(QLabel("Z:"))
        ang_vel_layout.addWidget(self.ang_vel_z_label)
        
        vf_layout.addRow("角速度 (rad/s):", ang_vel_layout)
        
        # 力
        self.force_x_label = QLabel("0.00")
        self.force_y_label = QLabel("0.00")
        self.force_z_label = QLabel("0.00")
        
        force_layout = QHBoxLayout()
        force_layout.addWidget(QLabel("X:"))
        force_layout.addWidget(self.force_x_label)
        force_layout.addWidget(QLabel("Y:"))
        force_layout.addWidget(self.force_y_label)
        force_layout.addWidget(QLabel("Z:"))
        force_layout.addWidget(self.force_z_label)
        
        vf_layout.addRow("力 (N):", force_layout)
        
        # 扭矩
        self.torque_x_label = QLabel("0.00")
        self.torque_y_label = QLabel("0.00")
        self.torque_z_label = QLabel("0.00")
        
        torque_layout = QHBoxLayout()
        torque_layout.addWidget(QLabel("X:"))
        torque_layout.addWidget(self.torque_x_label)
        torque_layout.addWidget(QLabel("Y:"))
        torque_layout.addWidget(self.torque_y_label)
        torque_layout.addWidget(QLabel("Z:"))
        torque_layout.addWidget(self.torque_z_label)
        
        vf_layout.addRow("扭矩 (Nm):", torque_layout)
        
        layout.addWidget(velocity_force_group)
        
        # 坐标变换控制
        transform_group = QGroupBox("坐标变换")
        transform_layout = QFormLayout(transform_group)
        
        # 坐标系选择
        self.coordinate_combo = QComboBox()
        self.coordinate_combo.addItems(["基座坐标系", "世界坐标系", "工具坐标系", "工件坐标系"])
        transform_layout.addRow("参考系:", self.coordinate_combo)
        
        # 变换按钮
        btn_layout = QHBoxLayout()
        
        self.set_home_btn = QPushButton("设为零点")
        self.set_home_btn.clicked.connect(self.home_arm)
        btn_layout.addWidget(self.set_home_btn)
        
        self.calibrate_btn = QPushButton("校准")
        btn_layout.addWidget(self.calibrate_btn)
        
        transform_layout.addRow("操作:", btn_layout)
        
        layout.addWidget(transform_group)
        
        # 添加弹簧
        layout.addStretch()
    
    def update_end_effector_data(self, position, orientation=None, 
                                linear_velocity=None, angular_velocity=None,
                                force=None, torque=None):
        """
        更新末端执行器数据
        
        Args:
            position: 位置 [x, y, z]
            orientation: 姿态四元数 [x, y, z, w]
            linear_velocity: 线速度 [vx, vy, vz]
            angular_velocity: 角速度 [wx, wy, wz]
            force: 力 [fx, fy, fz]
            torque: 扭矩 [tx, ty, tz]
        """
        # 更新位置
        if position is not None and len(position) == 3:
            self.position = list(position)
            self.pos_x_label.setText(f"{position[0]:.3f}")
            self.pos_y_label.setText(f"{position[1]:.3f}")
            self.pos_z_label.setText(f"{position[2]:.3f}")
        
        # 更新姿态
        if orientation is not None and len(orientation) == 4:
            self.orientation = list(orientation)
            
            # 更新四元数显示
            self.quat_x_label.setText(f"{orientation[0]:.3f}")
            self.quat_y_label.setText(f"{orientation[1]:.3f}")
            self.quat_z_label.setText(f"{orientation[2]:.3f}")
            self.quat_w_label.setText(f"{orientation[3]:.3f}")
            
            # 转换为欧拉角显示
            roll, pitch, yaw = self.quaternion_to_euler(orientation)
            self.roll_label.setText(f"{np.degrees(roll):.2f}")
            self.pitch_label.setText(f"{np.degrees(pitch):.2f}")
            self.yaw_label.setText(f"{np.degrees(yaw):.2f}")
        
        # 更新线速度
        if linear_velocity is not None and len(linear_velocity) == 3:
            self.velocity_linear = list(linear_velocity)
            self.lin_vel_x_label.setText(f"{linear_velocity[0]:.3f}")
            self.lin_vel_y_label.setText(f"{linear_velocity[1]:.3f}")
            self.lin_vel_z_label.setText(f"{linear_velocity[2]:.3f}")
        
        # 更新角速度
        if angular_velocity is not None and len(angular_velocity) == 3:
            self.velocity_angular = list(angular_velocity)
            self.ang_vel_x_label.setText(f"{angular_velocity[0]:.3f}")
            self.ang_vel_y_label.setText(f"{angular_velocity[1]:.3f}")
            self.ang_vel_z_label.setText(f"{angular_velocity[2]:.3f}")
        
        # 更新力
        if force is not None and len(force) == 3:
            self.force = list(force)
            self.force_x_label.setText(f"{force[0]:.2f}")
            self.force_y_label.setText(f"{force[1]:.2f}")
            self.force_z_label.setText(f"{force[2]:.2f}")
        
        # 更新扭矩
        if torque is not None and len(torque) == 3:
            self.torque = list(torque)
            self.torque_x_label.setText(f"{torque[0]:.2f}")
            self.torque_y_label.setText(f"{torque[1]:.2f}")
            self.torque_z_label.setText(f"{torque[2]:.2f}")

    def home_arm(self):
        """请求回零，由上层处理"""
        self.home_requested.emit()

    def enable_arm(self):
        """请求使能，由上层处理"""
        self.enable_requested.emit()

    def disable_arm(self):
        """请求禁用，由上层处理"""
        self.disable_requested.emit()
    
    def quaternion_to_euler(self, quaternion):
        """
        四元数转欧拉角
        
        Args:
            quaternion: 四元数 [x, y, z, w]
            
        Returns:
            欧拉角 (roll, pitch, yaw)
        """
        x, y, z, w = quaternion
        
        # 计算roll (x轴旋转)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # 计算pitch (y轴旋转)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # 计算yaw (z轴旋转)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


class ArmStatusPanel(QWidget):
    """机械臂状态面板类"""
    
    # 自定义信号
    arm_connect_request = pyqtSignal()
    arm_disconnect_request = pyqtSignal()
    arm_enable_request = pyqtSignal()
    arm_disable_request = pyqtSignal()
    arm_home_request = pyqtSignal()
    arm_calibrate_request = pyqtSignal()
    control_mode_changed = pyqtSignal(str)
    joint_angle_request = pyqtSignal(int, float)
    
    def __init__(self, config: LearmArmConfig, parent=None):
        """
        初始化机械臂状态面板
        
        Args:
            config: 机械臂配置
            parent: 父部件
        """
        super().__init__(parent)
        
        # 保存配置
        self.config = config
        
        # 机械臂状态
        self.arm_connected = False
        self.arm_enabled = False
        self.arm_homed = False
        self.control_mode = "joint"  # joint, cartesian, force
        self.safety_status = "normal"  # normal, warning, error, emergency
        self.connection_type = "hardware"  # hardware / simulation
        
        # 关节数量与限制
        self.num_joints = self._resolve_joint_count()
        self.joint_limits_min, self.joint_limits_max = self._resolve_joint_limits()

        # ????????? UI ?????????????
        self.use_simulation_data = False
        self._live_updates_enabled = False
        self._last_overview_signature = None
        
        # ???UI
        self.init_ui()
        
        # ???????????????????
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.setInterval(250)
        
        # ???????????
        self.sim_time = 0.0
        # 模拟数据生成（测试用）
        self.sim_time = 0.0
    
    def init_ui(self):
        """初始化用户界面"""
        # 创建主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        # 创建分割器
        splitter = QSplitter(Qt.Vertical)
        
        # 顶部：状态概览
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建状态概览组
        self.create_status_overview_group(top_layout)
        
        # 添加顶部部件到分割器
        splitter.addWidget(top_widget)
        
        # 中部：关节状态
        middle_widget = QWidget()
        middle_layout = QVBoxLayout(middle_widget)
        middle_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建关节状态部件
        self.joint_status = JointStatusWidget(
            self.num_joints,
            joint_limits_min=self.joint_limits_min,
            joint_limits_max=self.joint_limits_max
        )
        self.joint_status.home_requested.connect(self.home_arm)
        self.joint_status.enable_requested.connect(self.enable_arm)
        self.joint_status.disable_requested.connect(self.disable_arm)
        self.joint_status.joint_angle_commanded.connect(self.request_joint_angle)
        self.joint_status.update_connection_state(False, False)
        middle_layout.addWidget(self.joint_status)
        
        # 添加中部部件到分割器
        splitter.addWidget(middle_widget)
        
        # 底部：末端执行器状态
        bottom_widget = QWidget()
        bottom_layout = QVBoxLayout(bottom_widget)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建末端执行器部件
        self.end_effector_status = EndEffectorWidget()
        bottom_layout.addWidget(self.end_effector_status)
        
        # 添加底部部件到分割器
        splitter.addWidget(bottom_widget)
        
        # 设置分割器比例
        splitter.setSizes([120, 320, 240])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        
        # 使用滚动区域避免高度过高
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(5)
        scroll_layout.addWidget(splitter)
        scroll_area.setWidget(scroll_content)
        
        # 添加到主布局
        main_layout.addWidget(scroll_area)
    
    def create_status_overview_group(self, layout):
        """创建状态概览组"""
        group = QGroupBox("机械臂状态概览")
        group_layout = QGridLayout(group)
        
        # 连接状态
        self.connection_status_label = QLabel("未连接")
        self.connection_status_label.setStyleSheet("""
            QLabel {
                color: #ff6b6b;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        group_layout.addWidget(QLabel("连接状态:"), 0, 0)
        group_layout.addWidget(self.connection_status_label, 0, 1)
        
        # 使能状态
        self.enable_status_label = QLabel("禁用")
        self.enable_status_label.setStyleSheet("""
            QLabel {
                color: #ff6b6b;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        group_layout.addWidget(QLabel("使能状态:"), 0, 2)
        group_layout.addWidget(self.enable_status_label, 0, 3)
        
        # 回零状态
        self.home_status_label = QLabel("未回零")
        self.home_status_label.setStyleSheet("""
            QLabel {
                color: #ff6b6b;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        group_layout.addWidget(QLabel("回零状态:"), 1, 0)
        group_layout.addWidget(self.home_status_label, 1, 1)
        
        # 安全状态
        self.safety_status_label = QLabel("正常")
        self.safety_status_label.setStyleSheet("""
            QLabel {
                color: #4CAF50;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        group_layout.addWidget(QLabel("安全状态:"), 1, 2)
        group_layout.addWidget(self.safety_status_label, 1, 3)
        
        # 控制模式
        self.control_mode_label = QLabel("关节空间")
        group_layout.addWidget(QLabel("控制模式:"), 2, 0)
        group_layout.addWidget(self.control_mode_label, 2, 1)
        
        # 操作空间
        self.workspace_label = QLabel("可达")
        self.workspace_label.setStyleSheet("""
            QLabel {
                color: #4CAF50;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        group_layout.addWidget(QLabel("操作空间:"), 2, 2)
        group_layout.addWidget(self.workspace_label, 2, 3)
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.connect_arm)
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 6px 12px;
                border-radius: 4px;
            }
        """)
        control_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.disconnect_arm)
        self.disconnect_btn.setEnabled(False)
        control_layout.addWidget(self.disconnect_btn)
        
        self.enable_btn = QPushButton("使能")
        self.enable_btn.clicked.connect(self.enable_arm)
        self.enable_btn.setEnabled(False)
        control_layout.addWidget(self.enable_btn)
        
        self.disable_btn = QPushButton("禁用")
        self.disable_btn.clicked.connect(self.disable_arm)
        self.disable_btn.setEnabled(False)
        control_layout.addWidget(self.disable_btn)
        
        group_layout.addLayout(control_layout, 3, 0, 1, 4)
        
        # 控制模式选择
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("控制模式:"))
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["关节空间控制", "笛卡尔空间控制", "力控制", "混合控制"])
        self.mode_combo.currentTextChanged.connect(self.change_control_mode)
        mode_layout.addWidget(self.mode_combo)
        
        mode_layout.addStretch()
        
        group_layout.addLayout(mode_layout, 4, 0, 1, 4)
        
        layout.addWidget(group)
    
    def set_live_updates_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._live_updates_enabled:
            return
        self._live_updates_enabled = enabled
        if enabled and self.use_simulation_data:
            self.update_timer.start()
        else:
            self.update_timer.stop()

    def update_display(self):
        """????"""
        if not self._live_updates_enabled:
            return
        # ????????????????
        if not self.arm_connected:
            self.update_status_overview()
            return

        # ????????????
        if self.use_simulation_data:
            self.generate_simulated_data()
        
        # ????????
        self.update_joint_display()
        
        # ?????????
        self.update_end_effector_display()
        
        # ??????
        self.update_status_overview()

    def generate_simulated_data(self):
        """生成模拟数据（测试用）"""
        self.sim_time += 0.1
        
        # 生成模拟关节数据
        joint_positions = []
        joint_velocities = []
        joint_torques = []
        joint_targets = []
        
        for i in range(self.num_joints):
            # 基础角度
            base_angle = np.sin(self.sim_time * 0.5 + i * 0.5) * 30.0
            
            # 位置（添加一些噪声）
            position = base_angle + np.random.normal(0, 0.01)
            joint_positions.append(position)
            
            # 速度
            velocity = np.cos(self.sim_time * 0.5 + i * 0.5) * 10.0
            joint_velocities.append(velocity)
            
            # 扭矩
            torque = np.sin(self.sim_time * 0.8 + i * 0.3) * 2.0
            joint_torques.append(torque)
            
            # 目标位置
            target = np.sin(self.sim_time * 0.3 + i * 0.7) * 45.0
            joint_targets.append(target)
        
        # 更新关节数据
        self.joint_status.update_joint_data(
            joint_positions, joint_velocities, joint_torques, joint_targets
        )
        
        # 生成模拟末端执行器数据
        position = [
            0.3 + 0.1 * np.sin(self.sim_time * 0.3),
            0.1 * np.cos(self.sim_time * 0.4),
            0.4 + 0.05 * np.sin(self.sim_time * 0.5)
        ]
        
        # 简单的四元数（绕Z轴旋转）
        angle = self.sim_time * 0.2
        orientation = [0, 0, np.sin(angle/2), np.cos(angle/2)]
        
        # 速度
        linear_velocity = [
            0.1 * np.cos(self.sim_time * 0.3) * 0.3,
            -0.1 * np.sin(self.sim_time * 0.4) * 0.4,
            0.05 * np.cos(self.sim_time * 0.5) * 0.5
        ]
        
        angular_velocity = [0, 0, 0.2]
        
        # 力/扭矩（模拟接触）
        force = [
            0.5 * np.sin(self.sim_time * 1.0),
            0.3 * np.cos(self.sim_time * 1.2),
            2.0 + 0.5 * np.sin(self.sim_time * 0.8)
        ]
        
        torque = [
            0.1 * np.sin(self.sim_time * 1.5),
            0.1 * np.cos(self.sim_time * 1.3),
            0.05 * np.sin(self.sim_time * 1.1)
        ]
        
        # 更新末端执行器数据
        self.end_effector_status.update_end_effector_data(
            position, orientation, linear_velocity, angular_velocity, force, torque
        )
    
    def update_joint_display(self):
        """更新关节显示"""
        # 已经在joint_status部件中更新
        pass
    
    def update_end_effector_display(self):
        """更新末端执行器显示"""
        # 已经在end_effector_status部件中更新
        pass
    
    def update_status_overview(self):
        """更新状态概览"""
        # 更新连接状态
        if self.arm_connected:
            # 仿真连接在 UI 上标注出来，避免与真机混淆
            if self.connection_type == "simulation":
                self.connection_status_label.setText("已连接(仿真)")
            else:
                self.connection_status_label.setText("已连接")
            self.connection_status_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
        else:
            self.connection_status_label.setText("未连接")
            self.connection_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold; font-size: 12px;")
        
        # 更新使能状态
        if self.arm_enabled:
            self.enable_status_label.setText("使能")
            self.enable_status_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
        else:
            self.enable_status_label.setText("禁用")
            self.enable_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold; font-size: 12px;")
        
        # 更新回零状态
        if self.arm_homed:
            self.home_status_label.setText("已回零")
            self.home_status_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
        else:
            self.home_status_label.setText("未回零")
            self.home_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold; font-size: 12px;")
        
        # 更新安全状态
        if self.safety_status == "normal":
            self.safety_status_label.setText("正常")
            self.safety_status_label.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 12px;")
        elif self.safety_status == "warning":
            self.safety_status_label.setText("警告")
            self.safety_status_label.setStyleSheet("color: #ff9800; font-weight: bold; font-size: 12px;")
        elif self.safety_status == "error":
            self.safety_status_label.setText("错误")
            self.safety_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold; font-size: 12px;")
        elif self.safety_status == "emergency":
            self.safety_status_label.setText("紧急停止")
            self.safety_status_label.setStyleSheet("color: #d32f2f; font-weight: bold; font-size: 12px;")
        
        # 更新控制模式
        mode_text = {
            "joint": "关节空间",
            "cartesian": "笛卡尔空间",
            "force": "力控制",
            "hybrid": "混合控制"
        }
        self.control_mode_label.setText(mode_text.get(self.control_mode, "未知"))
        
        # 更新按钮状态
        self.connect_btn.setEnabled(not self.arm_connected)
        self.disconnect_btn.setEnabled(self.arm_connected)
        self.enable_btn.setEnabled(self.arm_connected and not self.arm_enabled)
        self.disable_btn.setEnabled(self.arm_connected and self.arm_enabled)
    
    # 槽函数
    def connect_arm(self):
        """连接机械臂"""
        self.arm_connect_request.emit()

    def disconnect_arm(self):
        """断开机械臂连接"""
        self.arm_disconnect_request.emit()
    
    def enable_arm(self):
        """使能机械臂"""
        self.arm_enable_request.emit()
    
    def disable_arm(self):
        """禁用机械臂"""
        self.arm_disable_request.emit()

    def home_arm(self):
        """回零操作"""
        self.arm_home_request.emit()
    
    def change_control_mode(self, mode_text):
        """更改控制模式"""
        mode_map = {
            "关节空间控制": "joint",
            "笛卡尔空间控制": "cartesian",
            "力控制": "force",
            "混合控制": "hybrid"
        }
        
        new_mode = mode_map.get(mode_text, "joint")
        if new_mode != self.control_mode:
            self.control_mode = new_mode
            self.control_mode_changed.emit(new_mode)
    
    def update_arm_status(self, connected=False, enabled=False, homed=False,
                         safety="normal", control_mode="joint", connection_type="hardware"):
        """
        更新机械臂状态
        
        Args:
            connected: 是否连接
            enabled: 是否使能
            homed: 是否回零
            safety: 安全状态
            control_mode: 控制模式
            connection_type: 连接类型（hardware/simulation）
        """
        self.arm_connected = connected
        self.arm_enabled = enabled
        self.arm_homed = homed
        self.safety_status = safety
        self.control_mode = control_mode
        self.connection_type = connection_type
        
        if self.joint_status:
            self.joint_status.update_connection_state(connected, enabled)

        self.update_status_overview()
    
    def update_joint_data(self, positions, velocities=None, torques=None, targets=None):
        """
        更新关节数据
        
        Args:
            positions: 关节位置数组
            velocities: 关节速度数组
            torques: 关节扭矩数组
            targets: 关节目标位置数组
        """
        self.use_simulation_data = False
        self.joint_status.update_joint_data(positions, velocities, torques, targets)
    
    def update_end_effector_data(self, position, orientation=None, 
                                linear_velocity=None, angular_velocity=None,
                                force=None, torque=None):
        """
        更新末端执行器数据
        
        Args:
            position: 位置 [x, y, z]
            orientation: 姿态四元数 [x, y, z, w]
            linear_velocity: 线速度 [vx, vy, vz]
            angular_velocity: 角速度 [wx, wy, wz]
            force: 力 [fx, fy, fz]
            torque: 扭矩 [tx, ty, tz]
        """
        self.end_effector_status.update_end_effector_data(
            position, orientation, linear_velocity, angular_velocity, force, torque
        )

    def request_joint_angle(self, joint_index: int, angle: float):
        """请求关节角度控制"""
        self.joint_angle_request.emit(joint_index, angle)

    def _resolve_joint_count(self) -> int:
        """获取关节数量"""
        if isinstance(self.config, dict):
            physical = self.config.get("PHYSICAL", {})
            return int(physical.get("dof", 6))
        if hasattr(self.config, "PHYSICAL"):
            return int(getattr(self.config, "PHYSICAL", {}).get("dof", 6))
        return int(getattr(self.config, "num_joints", 6))

    def _resolve_joint_limits(self):
        """获取关节角度限制（度）"""
        default_min = [-125.0] * self.num_joints
        default_max = [125.0] * self.num_joints

        limits = None
        if isinstance(self.config, dict):
            limits = self.config.get("PHYSICAL", {}).get("joint_limits")
        elif hasattr(self.config, "PHYSICAL"):
            limits = getattr(self.config, "PHYSICAL", {}).get("joint_limits")

        if not limits:
            return default_min, default_max

        joint_mins = []
        joint_maxs = []
        for i in range(self.num_joints):
            key = f"joint{i+1}"
            if key in limits and isinstance(limits[key], (list, tuple)) and len(limits[key]) >= 2:
                joint_mins.append(float(limits[key][0]))
                joint_maxs.append(float(limits[key][1]))
            else:
                joint_mins.append(default_min[i])
                joint_maxs.append(default_max[i])

        return joint_mins, joint_maxs


if __name__ == "__main__":
    # 测试机械臂状态面板
    import sys
    from PyQt5.QtWidgets import QApplication
    
    # 创建简单的配置类用于测试
    class TestLearnArmConfig:
        num_joints = 6
    
    app = QApplication(sys.argv)
    
    config = TestLearnArmConfig()
    panel = ArmStatusPanel(config)
    panel.setWindowTitle("机械臂状态面板测试")
    panel.resize(800, 900)
    panel.show()
    
    # 模拟连接机械臂
    QTimer.singleShot(1000, lambda: panel.update_arm_status(connected=True))
    QTimer.singleShot(2000, lambda: panel.update_arm_status(connected=True, enabled=True))
    
    sys.exit(app.exec_())
