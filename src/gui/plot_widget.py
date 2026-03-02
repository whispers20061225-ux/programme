"""
触觉夹爪演示系统 - 绘图部件
完整版：支持三维力数据、矢量图可视化，并包含所有绘图部件
"""

import numpy as np
from datetime import datetime
from typing import List, Dict, Any, Optional, Tuple
import time
import math
import logging
import matplotlib

# 设置Matplotlib性能参数
matplotlib.use('Qt5Agg')
matplotlib.rcParams['path.simplify'] = True
matplotlib.rcParams['path.simplify_threshold'] = 1.0
matplotlib.rcParams['agg.path.chunksize'] = 10000
matplotlib.rcParams['savefig.dpi'] = 100
matplotlib.rcParams['figure.dpi'] = 100

# 导入Matplotlib组件
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection

logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

# 设置中文字体
chinese_fonts = [
    'Noto Sans CJK SC',  # Linux CJK if installed
    'SimHei',            # 黑体 (Windows)
    'Microsoft YaHei',   # 微软雅黑 (Windows)
    'PingFang SC',       # 苹方 (macOS)
    'STHeiti',           # 华文黑体 (macOS)
    'Arial Unicode MS',  # 通用
    'DejaVu Sans',       # Linux default fallback
]

# Merge global runtime font selection with widget defaults.
existing_fonts = list(rcParams.get('font.sans-serif', []))
rcParams['font.sans-serif'] = list(dict.fromkeys(chinese_fonts + existing_fonts))
rcParams['axes.unicode_minus'] = False

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QGroupBox, QGridLayout, QSlider
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QSize

try:
    from config import DemoConfig
except ImportError:
    try:
        from ..config import DemoConfig
    except ImportError:
        import sys
        import os
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from config import DemoConfig


class ForceVectorGenerator:
    """三维力向量生成器 - 模拟真实夹取物体的受力情况"""
    
    def __init__(self, num_taxels=9, rows=3, cols=3):
        """
        初始化力向量生成器
        
        Args:
            num_taxels: 触觉单元数量
            rows: 行数
            cols: 列数
        """
        self.num_taxels = num_taxels
        self.rows = rows
        self.cols = cols
        
        # 触觉单元位置（在夹爪表面）
        self.positions = self.generate_positions()
        
        # 真实夹取物理参数
        self.friction_coefficient = 0.3  # 摩擦系数
        self.normal_force_range = (0, 100)  # 法向力范围 (N)
        self.shear_force_range = (0, 30)   # 剪切力范围 (N)
        
        # 夹取物体参数
        self.object_center = np.array([0.5, 0.5, 0.0])  # 物体中心位置
        self.object_radius = 0.3  # 物体半径
        
        # 当前夹取状态
        self.grip_force = 50.0  # 夹紧力 (N)
        self.object_weight = 1.0  # 物体重量 (kg)
        self.slip_direction = np.array([0.0, 0.0, 0.0])  # 滑移方向
        
    def generate_positions(self):
        """生成触觉单元位置"""
        positions = []
        
        for i in range(self.rows):
            for j in range(self.cols):
                if len(positions) >= self.num_taxels:
                    break
                    
                # 在夹爪表面均匀分布
                x = j / (self.cols - 1) if self.cols > 1 else 0.5
                y = i / (self.rows - 1) if self.rows > 1 else 0.5
                z = 0.0  # 表面位置
                
                positions.append([x, y, z])
        
        # 如果位置数量不足，填充剩余位置
        while len(positions) < self.num_taxels:
            positions.append([np.random.random(), np.random.random(), 0.0])
            
        return np.array(positions)
    
    def generate_realistic_force_vectors(self):
        """
        生成符合真实夹取情况的力向量
        
        返回:
            force_vectors: 三维力向量数组 (num_taxels x 3)
            contact_points: 接触点数组 (num_taxels x 3)
            normals: 法向量数组 (num_taxels x 3)
        """
        force_vectors = np.zeros((self.num_taxels, 3))
        contact_points = np.zeros((self.num_taxels, 3))
        normals = np.zeros((self.num_taxels, 3))
        
        # 1. 确定哪些触觉单元与物体接触
        contact_flags = np.zeros(self.num_taxels, dtype=bool)
        
        for i in range(self.num_taxels):
            # 计算到物体中心的距离
            pos_2d = self.positions[i, :2]  # 只考虑XY平面
            obj_center_2d = self.object_center[:2]
            distance = np.linalg.norm(pos_2d - obj_center_2d)
            
            # 如果距离小于物体半径，则发生接触
            if distance < self.object_radius:
                contact_flags[i] = True
                
                # 接触点位置（轻微陷入物体表面）
                contact_points[i, :2] = pos_2d
                contact_points[i, 2] = 0.05 * (1.0 - distance / self.object_radius)
                
                # 法向量（指向夹爪中心）
                normal = np.array([0.0, 0.0, 1.0])  # 默认法向
                normals[i] = normal
        
        # 2. 计算总接触点数
        num_contacts = np.sum(contact_flags)
        if num_contacts == 0:
            # 如果没有接触，返回零向量
            return force_vectors, contact_points, normals
        
        # 3. 模拟物体重量分布
        total_weight_force = self.object_weight * 9.8  # 重力 (N)
        weight_per_contact = total_weight_force / num_contacts if num_contacts > 0 else 0
        
        # 4. 模拟夹紧力分布
        grip_force_per_contact = self.grip_force / num_contacts if num_contacts > 0 else 0
        
        # 5. 模拟摩擦力（防止滑移）
        friction_magnitude = self.friction_coefficient * grip_force_per_contact
        
        # 6. 为每个接触点生成力向量
        for i in range(self.num_taxels):
            if contact_flags[i]:
                # 法向力（垂直于接触表面）
                normal_force = np.random.uniform(
                    self.normal_force_range[0] * 0.5,
                    self.normal_force_range[1] * 0.8
                ) * grip_force_per_contact
                
                # 剪切力（平行于接触表面）
                shear_x = np.random.uniform(
                    -self.shear_force_range[1],
                    self.shear_force_range[1]
                )
                shear_y = np.random.uniform(
                    -self.shear_force_range[1],
                    self.shear_force_range[1]
                )
                
                # 重力分量（向下）
                gravity_z = -weight_per_contact * np.random.uniform(0.5, 1.5)
                
                # 摩擦力（与滑移方向相反）
                if np.linalg.norm(self.slip_direction) > 0.01:
                    friction_direction = -self.slip_direction[:2] / np.linalg.norm(self.slip_direction[:2])
                    friction_x = friction_magnitude * friction_direction[0]
                    friction_y = friction_magnitude * friction_direction[1]
                else:
                    friction_x = 0.0
                    friction_y = 0.0
                
                # 组合力向量
                force_vectors[i, 0] = shear_x + friction_x  # X方向力
                force_vectors[i, 1] = shear_y + friction_y  # Y方向力
                force_vectors[i, 2] = normal_force + gravity_z  # Z方向力
                
                # 添加一些噪声
                noise_scale = 0.1
                force_vectors[i] += np.random.normal(0, noise_scale, 3)
        
        # 7. 更新滑移方向（模拟动态变化）
        self.update_slip_direction(force_vectors, contact_flags)
        
        # 8. 随机改变物体位置（模拟物体移动）
        if np.random.random() < 0.1:
            self.object_center[:2] += np.random.uniform(-0.05, 0.05, 2)
            self.object_center[:2] = np.clip(self.object_center[:2], 0.2, 0.8)
        
        return force_vectors, contact_points, normals
    
    def update_slip_direction(self, force_vectors, contact_flags):
        """更新滑移方向"""
        # 计算合力
        total_force = np.sum(force_vectors, axis=0)
        
        # 如果合力较大，物体可能滑移
        if np.linalg.norm(total_force[:2]) > self.grip_force * 0.5:
            # 滑移方向与合力方向相反
            slip_direction_2d = -total_force[:2] / np.linalg.norm(total_force[:2])
            
            # 平滑更新滑移方向
            smooth_factor = 0.2
            if np.linalg.norm(self.slip_direction[:2]) > 0:
                self.slip_direction[:2] = (
                    smooth_factor * slip_direction_2d + 
                    (1 - smooth_factor) * self.slip_direction[:2]
                )
            else:
                self.slip_direction[:2] = slip_direction_2d
            
            # 归一化
            slip_norm = np.linalg.norm(self.slip_direction[:2])
            if slip_norm > 0:
                self.slip_direction[:2] /= slip_norm
        else:
            # 逐渐减小滑移方向
            decay_factor = 0.9
            self.slip_direction[:2] *= decay_factor
            
            # 如果太小，重置为零
            if np.linalg.norm(self.slip_direction[:2]) < 0.01:
                self.slip_direction[:2] = np.array([0.0, 0.0])
    
    def get_force_statistics(self, force_vectors, contact_flags):
        """
        计算力统计信息
        
        Args:
            force_vectors: 力向量数组
            contact_flags: 接触标志数组
            
        Returns:
            统计信息字典
        """
        stats = {}
        
        # 只考虑实际接触的单元
        contact_indices = np.where(contact_flags)[0]
        
        if len(contact_indices) == 0:
            stats['total_force'] = np.array([0.0, 0.0, 0.0])
            stats['resultant_force'] = np.array([0.0, 0.0, 0.0])
            stats['mean_force'] = np.array([0.0, 0.0, 0.0])
            stats['max_force'] = np.array([0.0, 0.0, 0.0])
            stats['contact_count'] = 0
            return stats
        
        contact_forces = force_vectors[contact_indices]
        
        # 合力（所有力向量之和）
        total_force = np.sum(contact_forces, axis=0)
        
        # 合力大小和方向
        resultant_magnitude = np.linalg.norm(total_force)
        resultant_direction = total_force / resultant_magnitude if resultant_magnitude > 0 else np.array([0.0, 0.0, 0.0])
        
        # 平均力
        mean_force = np.mean(contact_forces, axis=0)
        
        # 最大力
        force_magnitudes = np.linalg.norm(contact_forces, axis=1)
        max_force_idx = np.argmax(force_magnitudes)
        max_force = contact_forces[max_force_idx]
        
        stats['total_force'] = total_force
        stats['resultant_force'] = resultant_direction * resultant_magnitude
        stats['resultant_magnitude'] = resultant_magnitude
        stats['resultant_direction'] = resultant_direction
        stats['mean_force'] = mean_force
        stats['max_force'] = max_force
        stats['max_force_magnitude'] = force_magnitudes[max_force_idx]
        stats['contact_count'] = len(contact_indices)
        stats['force_magnitudes'] = force_magnitudes
        stats['force_distribution'] = contact_forces
        
        return stats


class PlotWidget(QWidget):
    """绘图部件基类"""
    
    def __init__(self, config: DemoConfig, parent=None):
        """
        初始化绘图部件
        
        Args:
            config: 系统配置
            parent: 父部件
        """
        super().__init__(parent)
        self.config = config
        self.data_buffer = []
        self.max_data_points = 3000  # 增加最大数据点，避免帧数限制
        self.pending_update = False
        self.last_update_time = 0
        self.update_interval = 0.033  # 30Hz
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # 创建Matplotlib图形
        self.figure = Figure(figsize=(8, 6), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        
        # 添加导航工具栏
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        # 调小导航工具栏图标尺寸
        try:
            self.toolbar.setIconSize(QSize(16, 16))
        except Exception:
            pass
        self.toolbar.setIconSize(QSize(16, 16))
        
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
    
    def init_plot(self):
        """初始化绘图"""
        # 子类应该重写这个方法
        pass
    
    def update_data(self, data):
        """
        更新数据
        
        Args:
            data: 新数据
        """
        # 子类应该重写这个方法
        pass
    
    def update_plots(self):
        """
        更新绘图（无参数版本）
        在定时器中调用
        """
        # 子类应该重写这个方法
        pass
    
    def clear_plot(self):
        """清除绘图"""
        self.figure.clear()
        self.init_plot()
        self.canvas.draw()
    
    def set_config(self, config: DemoConfig):
        """
        设置新的配置
        
        Args:
            config: 新配置
        """
        self.config = config
    
    def save_plot(self, filename: str):
        """
        保存绘图到文件
        
        Args:
            filename: 文件名
        """
        self.figure.savefig(filename, dpi=300, bbox_inches='tight')


class TactilePlotWidget(PlotWidget):
    """触觉数据绘图部件 - 增强版：支持三维力数据和矢量图"""
    
    def __init__(self, config: DemoConfig, parent=None):
        """
        初始化触觉数据绘图部件
        
        Args:
            config: 系统配置
            parent: 父部件
        """
        super().__init__(config, parent)
        
        # 触觉数据特定的初始化
        self.num_taxels = getattr(config.hardware.sensor, 'num_taxels', 9)
        self.rows = getattr(config.hardware.sensor, 'rows', 3)
        self.cols = getattr(config.hardware.sensor, 'cols', 3)
        self.force_range = getattr(config.hardware.sensor, 'pressure_range', (0, 100))
        
        # 三维力数据支持
        self.force_vectors = np.zeros((self.num_taxels, 3))  # 三维力向量 (Fx, Fy, Fz)
        self.force_magnitudes = np.zeros(self.num_taxels)    # 力大小
        self.contact_flags = np.zeros(self.num_taxels, dtype=bool)  # 接触标志
        
        # 动态纵坐标调整参数
        self.dynamic_y_axis = True  # 启用动态纵坐标
        self.force_margin_percent = 15  # 纵坐标边距百分比，增加边距
        
        # 历史数据跟踪
        self.recent_min_values = []  # 最近的最小值
        self.recent_max_values = []  # 最近的最大值
        self.history_window = 50  # 历史窗口大小
        
        # 平滑处理参数
        self.smoothing_factor = 0.8  # 平滑因子，0.7表示70%历史数据+30%新数据
        self.smoothed_force_data = np.zeros(self.num_taxels)
        self.prev_force_data = None  # 上一帧数据，用于计算变化率
        
        # 环形缓冲区参数
        self.buffer_index = 0
        self.buffer_full = False
        
        # 初始化数据数组
        self.force_data = np.zeros(self.num_taxels)
        self.history_data = np.zeros((self.max_data_points, self.num_taxels))
        self.frame_numbers = np.arange(self.max_data_points)  # 帧号数组
        
        # 三维力历史数据
        self.history_vectors = np.zeros((self.max_data_points, self.num_taxels, 3))
        
        # 统计信息显示位置控制
        self.stats_position = (0.05, 0.95)  # 左上角
        self.color_info_position = (0.55, 0.05)  # 右下角
        self.vector_info_position = (0.05, 0.05)  # 左下角
        
        # 当前y轴范围
        self.current_y_min = 0
        self.current_y_max = 10
        
        # 可视化模式
        self.visualization_mode = "heatmap"  # heatmap, vector_field, 3d_view
        self.show_vectors = True  # 是否显示力向量
        self.vector_scale = 1.0  # 力向量缩放因子
        
        # 初始化UI和绘图
        self.init_ui()
        self.init_plot()
    
    def init_ui(self):
        """初始化用户界面"""
        super().init_ui()
    
    def init_plot(self):
        """初始化触觉数据绘图 - 支持三维力显示"""
        self.figure.clear()
        
        # 创建子图
        self.ax1 = self.figure.add_subplot(221)  # 触觉阵列图（热力图+矢量图）
        self.ax2 = self.figure.add_subplot(222)  # 力分布条形图（三维力分量）
        self.ax3 = self.figure.add_subplot(223)  # 时间序列图（三维力分量）
        self.ax4 = self.figure.add_subplot(224)  # 统计信息
        
        # 设置图形标题
        self.figure.suptitle('触觉数据可视化（三维力）', fontsize=14, fontweight='bold')
        
        # 触觉阵列图 - 支持热力图和矢量图叠加
        if self.num_taxels == self.rows * self.cols:
            tactile_array = np.zeros((self.rows, self.cols))
        elif self.num_taxels >= 9:
            tactile_array = np.zeros((3, 3))
        elif self.num_taxels >= 4:
            tactile_array = np.zeros((2, 2))
        else:
            tactile_array = np.zeros((1, self.num_taxels))
            
        # 创建自定义颜色映射，增加对比度
        colors = [(0, 0.3, 0.6), (0, 0.7, 0.9), (0.9, 0.9, 0), (1, 0.5, 0), (1, 0, 0)]
        custom_cmap = LinearSegmentedColormap.from_list('custom_viridis', colors, N=256)
        
        self.tactile_im = self.ax1.imshow(tactile_array, cmap=custom_cmap, 
                                          interpolation='nearest', vmin=0, vmax=100)
        self.ax1.set_title('触觉阵列（热力图+矢量图）')
        self.ax1.set_xlabel('列')
        self.ax1.set_ylabel('行')
        
        # 初始化力向量箭头（quiver）
        self.force_quiver = None
        
        # 修正行列标签显示
        if tactile_array.shape[0] > 0 and tactile_array.shape[1] > 0:
            self.ax1.set_xticks(range(tactile_array.shape[1]))
            self.ax1.set_yticks(range(tactile_array.shape[0]))
            self.ax1.set_xticklabels([str(i+1) for i in range(tactile_array.shape[1])])
            self.ax1.set_yticklabels([str(i+1) for i in range(tactile_array.shape[0])])
        
        self.figure.colorbar(self.tactile_im, ax=self.ax1, label='法向力 Fz (kPa)')
        
        # 力分布条形图 - 显示三维力分量
        x_pos = np.arange(self.num_taxels)
        bar_width = 0.25  # 减小宽度以容纳三个条形
        
        # 创建三组条形图，分别表示Fx, Fy, Fz
        self.bars_fx = self.ax2.bar(x_pos - bar_width, np.zeros(self.num_taxels), 
                                   width=bar_width, color='r', alpha=0.7, label='Fx')
        self.bars_fy = self.ax2.bar(x_pos, np.zeros(self.num_taxels), 
                                   width=bar_width, color='g', alpha=0.7, label='Fy')
        self.bars_fz = self.ax2.bar(x_pos + bar_width, np.zeros(self.num_taxels), 
                                   width=bar_width, color='b', alpha=0.7, label='Fz')
        
        self.ax2.set_title('三维力分布')
        self.ax2.set_xlabel('触觉单元')
        self.ax2.set_ylabel('力 (kPa)')
        self.ax2.legend(loc='upper right', fontsize=8)
        
        # 初始化纵坐标范围 - 动态调整
        if self.dynamic_y_axis:
            self.ax2.set_ylim(-50, 50)  # 允许负值
        else:
            self.ax2.set_ylim(-self.force_range[1], self.force_range[1])
            
        self.ax2.grid(True, alpha=0.3, axis='y')
        
        # 时间序列图 - 显示三维力分量
        num_lines = min(6, self.num_taxels)  # 减少线数以避免拥挤
        self.time_lines_fx = []
        self.time_lines_fy = []
        self.time_lines_fz = []
        
        colors_x = plt.cm.Reds(np.linspace(0.3, 0.8, num_lines))
        colors_y = plt.cm.Greens(np.linspace(0.3, 0.8, num_lines))
        colors_z = plt.cm.Blues(np.linspace(0.3, 0.8, num_lines))
        
        for i in range(num_lines):
            line_fx, = self.ax3.plot([], [], color=colors_x[i], linewidth=1.0, 
                                     linestyle='--', alpha=0.7, label=f'单元{i+1}-Fx' if i < 3 else "")
            line_fy, = self.ax3.plot([], [], color=colors_y[i], linewidth=1.0, 
                                     linestyle='-.', alpha=0.7, label=f'单元{i+1}-Fy' if i < 3 else "")
            line_fz, = self.ax3.plot([], [], color=colors_z[i], linewidth=1.5, 
                                     alpha=0.9, label=f'单元{i+1}-Fz' if i < 3 else "")
            
            self.time_lines_fx.append(line_fx)
            self.time_lines_fy.append(line_fy)
            self.time_lines_fz.append(line_fz)
            
        self.ax3.set_title(f'时间序列（前{num_lines}个单元的三维力）')
        self.ax3.set_xlabel('时间 (帧)')
        self.ax3.set_ylabel('力 (kPa)')
        self.ax3.legend(loc='upper right', fontsize=7, ncol=2)
        self.ax3.grid(True, alpha=0.3)
        
        # 统计信息 - 使用文本位置控制避免重叠
        self.ax4.axis('off')
        
        # 调整布局
        self.figure.tight_layout(rect=[0, 0, 1, 0.95])
        
        # 绘制初始图形
        self.canvas.draw()
    
    def apply_smoothing(self, new_data, prev_data, alpha=0.7):
        """
        应用指数加权移动平均平滑
        
        Args:
            new_data: 新数据
            prev_data: 上一帧数据
            alpha: 平滑因子 (0-1)，越大越依赖历史数据
            
        Returns:
            平滑后的数据
        """
        if prev_data is None or len(prev_data) != len(new_data):
            return new_data
        
        # 指数加权移动平均
        smoothed = alpha * prev_data + (1 - alpha) * new_data
        return smoothed
    
    def update_data(self, data):
        """
        更新触觉数据（支持三维力数据）
        
        Args:
            data: 触觉数据对象或字典，可能包含三维力数据
        """
        try:
            # 处理不同类型的输入数据
            has_3d_force = False
            
            # 1. 检查是否有三维力数据
            if hasattr(data, 'force_vectors'):
                # 直接有force_vectors属性
                new_force_vectors = np.array(data.force_vectors)
                if new_force_vectors.ndim == 2 and new_force_vectors.shape[1] >= 3:
                    has_3d_force = True
                    force_vectors_3d = new_force_vectors[:, :3]
                    
                    # 计算力大小（用于向后兼容）
                    force_magnitudes = np.linalg.norm(force_vectors_3d, axis=1)
                    
                    # 提取Z方向力（法向力）
                    z_forces = force_vectors_3d[:, 2]
                    
                    # 应用平滑处理
                    force_magnitudes = self.apply_smoothing(
                        force_magnitudes, 
                        self.prev_force_data, 
                        self.smoothing_factor
                    )
                    
                    self.prev_force_data = force_magnitudes.copy()
                    
                    # 更新力向量数据
                    self.force_vectors = force_vectors_3d
                    self.force_magnitudes = force_magnitudes
                    self.force_data = z_forces  # 使用Z方向力作为主要显示数据
                    
            # 2. 如果没有三维力数据，检查是否有tactile_array
            elif hasattr(data, 'tactile_array'):
                # 从tactile_array中提取数据
                tactile_array = data.tactile_array
                
                if tactile_array.ndim == 2 and tactile_array.shape[1] >= 3:
                    # 有三维数据
                    has_3d_force = True
                    force_vectors_3d = tactile_array[:, :3]
                    
                    # 计算力大小
                    force_magnitudes = np.linalg.norm(force_vectors_3d, axis=1)
                    z_forces = force_vectors_3d[:, 2]
                    
                    # 应用平滑处理
                    force_magnitudes = self.apply_smoothing(
                        force_magnitudes, 
                        self.prev_force_data, 
                        self.smoothing_factor
                    )
                    
                    self.prev_force_data = force_magnitudes.copy()
                    
                    # 更新力向量数据
                    self.force_vectors = force_vectors_3d
                    self.force_magnitudes = force_magnitudes
                    self.force_data = z_forces
                    
                elif tactile_array.ndim == 1 or (tactile_array.ndim == 2 and tactile_array.shape[1] == 1):
                    # 只有一维数据（Z方向或总力大小）
                    force_magnitudes = tactile_array.flatten() if tactile_array.ndim == 2 else tactile_array
                    
                    # 应用平滑处理
                    force_magnitudes = self.apply_smoothing(
                        force_magnitudes, 
                        self.prev_force_data, 
                        self.smoothing_factor
                    )
                    
                    self.prev_force_data = force_magnitudes.copy()
                    
                    # 生成模拟的三维力数据
                    self.generate_simulated_3d_forces(force_magnitudes)
                    
                    # 更新力数据
                    self.force_magnitudes = force_magnitudes
                    self.force_data = self.force_vectors[:, 2]  # 使用Z方向力
            
            # 3. 如果还没有数据，使用模拟数据
            else:
                # 生成模拟数据
                import random
                force_magnitudes = np.random.uniform(10, 50, self.num_taxels)
                
                # 应用平滑处理
                force_magnitudes = self.apply_smoothing(
                    force_magnitudes, 
                    self.prev_force_data, 
                    self.smoothing_factor
                )
                
                self.prev_force_data = force_magnitudes.copy()
                
                # 生成模拟的三维力数据
                self.generate_simulated_3d_forces(force_magnitudes)
                
                # 更新力数据
                self.force_magnitudes = force_magnitudes
                self.force_data = self.force_vectors[:, 2]
            
            # 限制在合理范围内
            self.force_data = np.clip(self.force_data, 0, 100)
            
            # 更新接触标志（基于力大小）
            contact_threshold = 5.0
            self.contact_flags = self.force_magnitudes > contact_threshold
            
            # 更新历史数据（环形缓冲区）
            self.history_data[self.buffer_index] = self.force_data
            self.history_vectors[self.buffer_index] = self.force_vectors
            
            # 更新动态纵坐标范围
            if self.dynamic_y_axis and len(self.force_data) > 0:
                current_min = np.min(self.force_data)
                current_max = np.max(self.force_data)
                
                # 记录最近的值
                self.recent_min_values.append(current_min)
                self.recent_max_values.append(current_max)
                
                # 保持历史记录长度
                if len(self.recent_min_values) > self.history_window:
                    self.recent_min_values.pop(0)
                    self.recent_max_values.pop(0)
            
            # 更新缓冲区索引
            self.buffer_index = (self.buffer_index + 1) % self.max_data_points
            if self.buffer_index == 0:
                self.buffer_full = True
            
            self.pending_update = True
            
            # 调试信息
            if time.time() - getattr(self, '_last_debug_time', 0) > 2.0:
                if has_3d_force:
                    print(f"三维力数据 - 单元数: {self.num_taxels}")
                    print(f"  Fx范围: {np.min(self.force_vectors[:,0]):.1f}-{np.max(self.force_vectors[:,0]):.1f}")
                    print(f"  Fy范围: {np.min(self.force_vectors[:,1]):.1f}-{np.max(self.force_vectors[:,1]):.1f}")
                    print(f"  Fz范围: {np.min(self.force_vectors[:,2]):.1f}-{np.max(self.force_vectors[:,2]):.1f}")
                    print(f"  合力大小范围: {np.min(self.force_magnitudes):.1f}-{np.max(self.force_magnitudes):.1f}")
                else:
                    print(f"一维力数据 - 范围: {np.min(self.force_data):.1f}-{np.max(self.force_data):.1f}")
                
                self._last_debug_time = time.time()
                
        except Exception as e:
            print(f"更新触觉数据错误: {e}")
            import traceback
            traceback.print_exc()
    
    def generate_simulated_3d_forces(self, force_magnitudes):
        """
        生成模拟的三维力数据，模拟真实夹取情况
        
        Args:
            force_magnitudes: 力大小数组
        """
        # 创建力向量生成器
        if not hasattr(self, 'force_generator'):
            self.force_generator = ForceVectorGenerator(
                num_taxels=self.num_taxels,
                rows=self.rows,
                cols=self.cols
            )
        
        # 生成三维力向量
        force_vectors, contact_points, normals = self.force_generator.generate_realistic_force_vectors()
        
        # 调整力大小以匹配输入
        if len(force_vectors) == len(force_magnitudes):
            # 保持方向，调整大小
            for i in range(len(force_vectors)):
                current_mag = np.linalg.norm(force_vectors[i])
                if current_mag > 0.001:
                    scale = force_magnitudes[i] / current_mag
                    force_vectors[i] *= scale
        
        self.force_vectors = force_vectors
    
    def update_plots(self):
        """更新所有子图（在定时器中调用）"""
        if not self.pending_update:
            return
            
        self.pending_update = False
        current_time = time.time()
        
        # 控制更新频率
        if current_time - self.last_update_time < self.update_interval:
            return
            
        self.last_update_time = current_time
        
        try:
            # 1. 更新时间序列图的数据
            display_points = min(200, self.max_data_points)  # 显示最近200个点
            
            if display_points > 1:
                # 确定显示的数据范围
                if self.buffer_full:
                    if self.buffer_index >= display_points:
                        start_idx = self.buffer_index - display_points
                        end_idx = self.buffer_index
                    else:
                        part1_len = display_points - self.buffer_index
                        start_idx = self.max_data_points - part1_len
                        end_idx = self.max_data_points
                else:
                    start_idx = max(0, self.buffer_index - display_points)
                    end_idx = self.buffer_index
                
                # 获取要显示的数据
                if start_idx < end_idx:
                    x_data = self.frame_numbers[start_idx:end_idx]
                    
                    # 更新前几个单元的三维力时间序列
                    for i in range(min(len(self.time_lines_fx), self.num_taxels)):
                        # Fx数据
                        if i < len(self.time_lines_fx):
                            y_data_fx = self.history_vectors[start_idx:end_idx, i, 0]
                            self.time_lines_fx[i].set_data(x_data, y_data_fx)
                        
                        # Fy数据
                        if i < len(self.time_lines_fy):
                            y_data_fy = self.history_vectors[start_idx:end_idx, i, 1]
                            self.time_lines_fy[i].set_data(x_data, y_data_fy)
                        
                        # Fz数据
                        if i < len(self.time_lines_fz):
                            y_data_fz = self.history_vectors[start_idx:end_idx, i, 2]
                            self.time_lines_fz[i].set_data(x_data, y_data_fz)
                    
                    # 获取显示的数据用于范围计算
                    display_data_fx = self.history_vectors[start_idx:end_idx, :min(6, self.num_taxels), 0]
                    display_data_fy = self.history_vectors[start_idx:end_idx, :min(6, self.num_taxels), 1]
                    display_data_fz = self.history_vectors[start_idx:end_idx, :min(6, self.num_taxels), 2]
                    
                    display_data = np.concatenate([display_data_fx.flatten(), 
                                                   display_data_fy.flatten(), 
                                                   display_data_fz.flatten()])
                
                # 调整时间序列图的轴范围
                if 'display_data' in locals() and display_data.size > 0:
                    # x轴范围
                    if start_idx < end_idx:
                        x_min = self.frame_numbers[start_idx]
                        x_max = self.frame_numbers[end_idx-1]
                    else:
                        x_min = self.frame_numbers[start_idx]
                        x_max = self.frame_numbers[end_idx-1] if end_idx > 0 else self.frame_numbers[start_idx]
                    
                    self.ax3.set_xlim(x_min, x_max)
                    
                    # y轴范围 - 基于所有显示的数据
                    data_min = np.min(display_data)
                    data_max = np.max(display_data)
                    
                    # 确保有可见的范围
                    if data_min == data_max:
                        margin = 5
                    else:
                        margin = max(5, (data_max - data_min) * 0.2)
                    
                    self.ax3.set_ylim(data_min - margin, data_max + margin)
            
            # 2. 更新触觉阵列图（热力图+矢量图）
            if self.num_taxels >= 9:
                try:
                    # 创建触觉阵列（使用Z方向力）
                    if self.num_taxels >= 9:
                        tactile_array = self.force_data[:9].reshape(3, 3)
                    else:
                        # 对于非标准数量，尝试其他布局
                        tactile_array = np.zeros((self.rows, self.cols))
                        for i in range(min(self.num_taxels, self.rows * self.cols)):
                            row = i // self.cols
                            col = i % self.cols
                            tactile_array[row, col] = self.force_data[i]
                    
                    self.tactile_im.set_data(tactile_array)
                    
                    # 动态调整颜色范围
                    array_min = np.min(tactile_array)
                    array_max = np.max(tactile_array)
                    
                    if array_max > array_min:
                        color_margin = max(5, (array_max - array_min) * 0.1)
                        self.tactile_im.set_clim(vmin=max(0, array_min - color_margin), 
                                                vmax=min(100, array_max + color_margin))
                    else:
                        self.tactile_im.set_clim(vmin=0, vmax=max(20, array_max))
                    
                    # 更新力向量箭头（如果显示矢量图）
                    if self.show_vectors and self.force_vectors is not None:
                        self.update_force_vectors()
                        
                except Exception as e:
                    print(f"更新触觉阵列图错误: {e}")
            
            # 3. 更新力分布条形图（三维力分量）
            if len(self.force_vectors) > 0:
                # 更新每个条形的高度
                for i in range(min(self.num_taxels, len(self.bars_fx))):
                    self.bars_fx[i].set_height(self.force_vectors[i, 0])
                    self.bars_fy[i].set_height(self.force_vectors[i, 1])
                    self.bars_fz[i].set_height(self.force_vectors[i, 2])
                
                # 动态调整y轴范围
                if self.dynamic_y_axis and self.force_vectors.size > 0:
                    # 获取当前数据的范围（考虑所有三个分量）
                    fx_min = np.min(self.force_vectors[:, 0])
                    fx_max = np.max(self.force_vectors[:, 0])
                    fy_min = np.min(self.force_vectors[:, 1])
                    fy_max = np.max(self.force_vectors[:, 1])
                    fz_min = np.min(self.force_vectors[:, 2])
                    fz_max = np.max(self.force_vectors[:, 2])
                    
                    current_min = min(fx_min, fy_min, fz_min)
                    current_max = max(fx_max, fy_max, fz_max)
                    
                    # 使用历史数据平滑调整
                    if len(self.recent_min_values) > 0 and len(self.recent_max_values) > 0:
                        hist_min = min(self.recent_min_values)
                        hist_max = max(self.recent_max_values)
                        
                        if hist_min > hist_max:
                            hist_min, hist_max = hist_max, hist_min
                        
                        # 考虑负值（剪切力可能是负的）
                        if hist_max - hist_min < 10:
                            if abs(current_max) < 10 and abs(current_min) < 10:
                                y_min = -20
                                y_max = 20
                            else:
                                y_min = current_min - 20
                                y_max = current_max + 20
                        else:
                            margin = max(10, (hist_max - hist_min) * self.force_margin_percent / 100)
                            y_min = current_min - margin
                            y_max = current_max + margin
                    else:
                        if current_max - current_min < 10:
                            y_min = -20
                            y_max = 20
                        else:
                            margin = max(10, (current_max - current_min) * 0.2)
                            y_min = current_min - margin
                            y_max = current_max + margin
                    
                    # 确保最小值小于最大值
                    if y_min >= y_max:
                        y_max = y_min + 20
                    
                    # 平滑过渡到新的y轴范围
                    current_ylim = self.ax2.get_ylim()
                    smooth_factor = 0.3
                    y_min_smooth = smooth_factor * current_ylim[0] + (1 - smooth_factor) * y_min
                    y_max_smooth = smooth_factor * current_ylim[1] + (1 - smooth_factor) * y_max
                    
                    # 应用新的y轴范围
                    self.ax2.set_ylim(y_min_smooth, y_max_smooth)
                    
                    # 保存当前范围
                    self.current_y_min = y_min_smooth
                    self.current_y_max = y_max_smooth
            
            # 4. 更新统计信息 - 包括三维力信息
            self.update_statistics()
            
            # 使用draw_idle进行异步重绘
            self.canvas.draw_idle()
            
        except Exception as e:
            print(f"更新触觉绘图错误: {e}")
            import traceback
            traceback.print_exc()
    
    def update_force_vectors(self):
        """更新力向量箭头显示"""
        # 清除旧的箭头
        if self.force_quiver is not None:
            try:
                self.force_quiver.remove()
            except:
                pass
        
        # 只显示实际接触的单元的力向量
        contact_indices = np.where(self.contact_flags)[0]
        if len(contact_indices) == 0:
            return
        
        # 准备箭头数据
        x_positions = []
        y_positions = []
        u_components = []
        v_components = []
        
        for idx in contact_indices:
            if idx < self.num_taxels:
                # 计算触觉单元在图像中的位置
                if self.num_taxels >= 9 and idx < 9:
                    row = idx // 3
                    col = idx % 3
                else:
                    row = idx // self.cols
                    col = idx % self.cols
                
                # 力向量（在图像平面上的投影）
                fx = self.force_vectors[idx, 0]
                fy = self.force_vectors[idx, 1]
                
                # 只显示XY平面上的力分量
                magnitude = np.sqrt(fx**2 + fy**2)
                if magnitude > 0.1:  # 只显示足够大的力
                    # 缩放因子
                    scale = self.vector_scale * 0.5  # 减小箭头长度
                    
                    x_positions.append(col)
                    y_positions.append(self.rows - 1 - row)  # 反转Y轴
                    u_components.append(fx * scale)
                    v_components.append(fy * scale)
        
        if len(x_positions) > 0:
            # 创建箭头
            self.force_quiver = self.ax1.quiver(
                x_positions, y_positions, 
                u_components, v_components,
                color='red', alpha=0.8,
                scale=20, scale_units='inches',
                width=0.005, headwidth=3, headlength=4
            )
    
    def update_statistics(self):
        """更新统计信息，包括三维力信息"""
        self.ax4.clear()
        self.ax4.axis('off')
        
        if len(self.force_data) > 0 and len(self.force_vectors) > 0:
            # 计算统计信息
            mean_val = np.mean(self.force_data)
            max_val = np.max(self.force_data)
            min_val = np.min(self.force_data)
            std_val = np.std(self.force_data)
            median_val = np.median(self.force_data)
            
            # 三维力统计
            fx_mean = np.mean(self.force_vectors[:, 0])
            fy_mean = np.mean(self.force_vectors[:, 1])
            fz_mean = np.mean(self.force_vectors[:, 2])
            
            fx_max = np.max(self.force_vectors[:, 0])
            fy_max = np.max(self.force_vectors[:, 1])
            fz_max = np.max(self.force_vectors[:, 2])
            
            # 合力计算
            total_force = np.sum(self.force_vectors, axis=0)
            resultant_magnitude = np.linalg.norm(total_force)
            
            # 激活单元数量（压力大于阈值）
            threshold = mean_val * 0.5 if mean_val > 5 else 5
            active_taxels = np.sum(self.force_data > threshold)
            
            # 接触单元数量（基于三维力）
            contact_threshold = 5.0
            contact_taxels = np.sum(self.force_magnitudes > contact_threshold)
            
            # 1. 统计信息放在左上角
            stats_text = (
                f"统计信息:\n"
                f"单元数: {self.num_taxels}\n"
                f"接触单元: {contact_taxels}\n"
                f"激活单元: {active_taxels}\n"
                f"平均力: {mean_val:.1f} kPa\n"
                f"最大力: {max_val:.1f} kPa\n"
                f"标准差: {std_val:.1f} kPa\n"
                f"数据点数: {self.buffer_index if not self.buffer_full else self.max_data_points}"
            )
            
            self.ax4.text(self.stats_position[0], self.stats_position[1], stats_text, 
                         transform=self.ax4.transAxes,
                         verticalalignment='top', fontsize=8,
                         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9,
                                  edgecolor='orange', linewidth=1))
            
            # 2. 三维力信息放在中间
            force_3d_text = (
                f"三维力统计:\n"
                f"平均 Fx: {fx_mean:.1f} kPa\n"
                f"平均 Fy: {fy_mean:.1f} kPa\n"
                f"平均 Fz: {fz_mean:.1f} kPa\n"
                f"最大 Fx: {fx_max:.1f} kPa\n"
                f"最大 Fy: {fy_max:.1f} kPa\n"
                f"最大 Fz: {fz_max:.1f} kPa\n"
                f"合力大小: {resultant_magnitude:.1f} kPa"
            )
            
            self.ax4.text(0.5, 0.5, force_3d_text, 
                         transform=self.ax4.transAxes,
                         verticalalignment='center', horizontalalignment='center',
                         fontsize=8,
                         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8,
                                  edgecolor='blue', linewidth=1))
            
            # 3. 颜色和矢量说明放在右下角
            vector_info_text = (
                f"颜色说明:\n"
                f"红色: Fx (剪切力)\n"
                f"绿色: Fy (剪切力)\n"
                f"蓝色: Fz (法向力)\n"
                f"箭头: XY平面力向量\n"
                f"范围: {self.current_y_min:.1f}-{self.current_y_max:.1f} kPa"
            )
            
            self.ax4.text(self.vector_info_position[0], self.vector_info_position[1], vector_info_text,
                         transform=self.ax4.transAxes,
                         verticalalignment='bottom', fontsize=7,
                         bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8,
                                  edgecolor='green', linewidth=1))
    
    def set_visualization_mode(self, mode):
        """设置可视化模式"""
        self.visualization_mode = mode
        
        if mode == "vector_field":
            self.show_vectors = True
        elif mode == "heatmap":
            self.show_vectors = False
        elif mode == "3d_view":
            self.show_vectors = True
            # 这里可以切换到3D视图
    
    def set_show_vectors(self, show):
        """设置是否显示力向量"""
        self.show_vectors = show
    
    def set_vector_scale(self, scale):
        """设置力向量缩放因子"""
        self.vector_scale = scale


class VectorFieldWidget(PlotWidget):
    """矢量场绘图部件 - 专门显示三维力矢量场"""
    
    def __init__(self, config: DemoConfig, parent=None):
        """
        初始化矢量场绘图部件
        
        Args:
            config: 系统配置
            parent: 父部件
        """
        super().__init__(config, parent)
        
        # 矢量场特定的初始化
        self.num_taxels = getattr(config.hardware.sensor, 'num_taxels', 9)
        self.rows = getattr(config.hardware.sensor, 'rows', 3)
        self.cols = getattr(config.hardware.sensor, 'cols', 3)
        
        # 力向量数据
        self.force_vectors = np.zeros((self.num_taxels, 3))
        self.contact_points = np.zeros((self.num_taxels, 3))
        self.contact_flags = np.zeros(self.num_taxels, dtype=bool)
        
        # 可视化参数
        self.vector_scale = 2.0  # 放大默认矢量显示
        self.show_labels = True
        self.show_grid = True
        self.view_mode = '3d'  # '3d', 'top', 'front', 'side'
        self.contact_threshold = 0.08  # 显示接触的力阈值 (N)
        
        # 颜色映射
        self.color_map = plt.cm.jet
        
        # 初始化UI和绘图
        self.init_ui()
        self.init_plot()
    
    def init_ui(self):
        """初始化用户界面"""
        super().init_ui()
    
    def init_plot(self):
        """初始化矢量场绘图"""
        self.figure.clear()
        
        # 创建3D子图
        self.ax = self.figure.add_subplot(111, projection='3d')
        
        # 设置图形标题
        self.ax.set_title('三维力矢量场', fontsize=14, fontweight='bold')
        
        # 设置坐标轴标签
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        
        # 设置初始视角
        self.ax.view_init(elev=30, azim=45)
        
        # 设置坐标轴范围
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(-50, 50)
        self.ax.set_zlim(0, 100)
        
        # 添加网格
        self.ax.grid(self.show_grid)
        
        # 初始化箭头集合
        self.arrows = []
        self.scatter = None
        self.force_labels = []
        
        # 绘制初始图形
        self.canvas.draw()
    
    def update_data(self, data):
        """
        更新矢量场数据
        
        Args:
            data: 包含力向量和接触点的数据
        """
        try:
            # 从数据中提取力向量和接触点
            if hasattr(data, 'force_vectors'):
                self.force_vectors = np.array(data.force_vectors)
            elif isinstance(data, dict) and 'force_vectors' in data:
                self.force_vectors = np.array(data['force_vectors'])
            elif isinstance(data, np.ndarray) and data.ndim == 2 and data.shape[1] >= 3:
                self.force_vectors = data[:, :3]
            
            # 提取接触点信息
            if hasattr(data, 'contact_points'):
                self.contact_points = np.array(data.contact_points)
            elif isinstance(data, dict) and 'contact_points' in data:
                self.contact_points = np.array(data['contact_points'])
            else:
                # 生成默认接触点位置
                self.generate_contact_points()
            
            # 更新接触标志
            if hasattr(data, 'contact_flags'):
                self.contact_flags = np.array(data.contact_flags)
            else:
                # 基于力大小判断接触
                force_magnitudes = np.linalg.norm(self.force_vectors, axis=1)
                self.contact_flags = force_magnitudes > self.contact_threshold
            
            self.pending_update = True
            
        except Exception as e:
            print(f"更新矢量场数据错误: {e}")
            import traceback
            traceback.print_exc()
    
    def update_contact_data(self, contact_data):
        """
        更新接触数据 - 用于从main_window接收接触数据信号
        
        Args:
            contact_data: 接触数据，可以是接触标志数组或接触点数据
        """
        try:
            # 如果接触数据是数组
            if isinstance(contact_data, np.ndarray):
                # 如果是一维数组，假设是接触标志
                if contact_data.ndim == 1 and len(contact_data) == self.num_taxels:
                    self.contact_flags = contact_data.astype(bool)
                # 如果是二维数组，假设是接触点位置
                elif contact_data.ndim == 2 and contact_data.shape[1] >= 3:
                    if contact_data.shape[0] == self.num_taxels:
                        self.contact_points = contact_data[:, :3]
                        self.contact_flags = np.ones(self.num_taxels, dtype=bool)
                else:
                    print(f"未知的接触数据格式: {contact_data.shape}")
            
            self.pending_update = True
            
        except Exception as e:
            print(f"更新接触数据错误: {e}")
            import traceback
            traceback.print_exc()
    
    def generate_contact_points(self):
        """生成接触点位置"""
        self.contact_points = np.zeros((self.num_taxels, 3))
        
        # 在夹爪表面均匀分布接触点
        for i in range(self.num_taxels):
            if self.rows > 1 and self.cols > 1:
                row = i // self.cols
                col = i % self.cols
                
                # 将位置映射到实际尺寸（毫米）
                x = (col - (self.cols - 1) / 2) * 20  # X方向
                y = (row - (self.rows - 1) / 2) * 20  # Y方向
                z = 0  # Z方向（夹爪表面）
            else:
                # 对于非网格排列，使用随机位置
                x = np.random.uniform(-40, 40)
                y = np.random.uniform(-40, 40)
                z = 0
            
            self.contact_points[i] = [x, y, z]
    
    def update_plots(self):
        """更新矢量场绘图"""
        if not self.pending_update:
            return
            
        self.pending_update = False
        current_time = time.time()
        
        # 控制更新频率
        if current_time - self.last_update_time < self.update_interval:
            return
            
        self.last_update_time = current_time
        
        try:
            # 清除旧的箭头和标签
            for arrow in self.arrows:
                arrow.remove()
            self.arrows.clear()
            
            for label in self.force_labels:
                label.remove()
            self.force_labels.clear()
            
            if self.scatter is not None:
                self.scatter.remove()
            
            # 只显示实际接触的单元
            contact_indices = np.where(self.contact_flags)[0]
            if len(contact_indices) == 0:
                # 如果没有接触，显示一个空场景
                self.canvas.draw_idle()
                return
            
            contact_points = self.contact_points[contact_indices]
            contact_vectors = self.force_vectors[contact_indices]
            
            # 计算力的大小，用于颜色编码
            force_magnitudes = np.linalg.norm(contact_vectors, axis=1)
            if len(force_magnitudes) > 0:
                max_magnitude = np.max(force_magnitudes)
                if max_magnitude > 0:
                    normalized_magnitudes = force_magnitudes / max_magnitude
                else:
                    normalized_magnitudes = np.zeros_like(force_magnitudes)
            else:
                normalized_magnitudes = np.array([])
            
            # 绘制接触点
            self.scatter = self.ax.scatter(
                contact_points[:, 0], contact_points[:, 1], contact_points[:, 2],
                c=normalized_magnitudes, cmap=self.color_map,
                s=50, alpha=0.8, edgecolors='k', linewidths=0.5
            )
            
            # 绘制力向量箭头
            for i, idx in enumerate(contact_indices):
                if i < len(contact_points) and i < len(contact_vectors):
                    start_point = contact_points[i]
                    force_vector = contact_vectors[i]
                    
                    # 缩放力向量以便可视化（提升系数便于观察边缘触点）
                    scaled_vector = force_vector * self.vector_scale * 0.3
                    
                    # 计算箭头颜色
                    if len(normalized_magnitudes) > i:
                        color = self.color_map(normalized_magnitudes[i])
                    else:
                        color = 'blue'
                    
                    # 绘制3D箭头
                    arrow = self.ax.quiver(
                        start_point[0], start_point[1], start_point[2],
                        scaled_vector[0], scaled_vector[1], scaled_vector[2],
                        color=color, alpha=0.8,
                        arrow_length_ratio=0.3, linewidth=2
                    )
                    self.arrows.append(arrow)
                    
                    # 添加力标签
                    if self.show_labels and i % 2 == 0:  # 每隔一个显示标签，避免拥挤
                        magnitude = force_magnitudes[i] if i < len(force_magnitudes) else 0
                        label_text = f'{magnitude:.1f}'
                        
                        # 标签位置（箭头末端）
                        end_point = start_point + scaled_vector
                        label = self.ax.text(
                            end_point[0], end_point[1], end_point[2],
                            label_text, fontsize=7, color='darkred'
                        )
                        self.force_labels.append(label)
            
            # 不再绘制合力箭头，专注于每个触点的力
            
            # 更新图形
            self.canvas.draw_idle()
            
        except Exception as e:
            print(f"更新矢量场绘图错误: {e}")
            import traceback
            traceback.print_exc()
    
    def change_view(self, view_type):
        """
        更改视图
        
        Args:
            view_type: 视图类型 ('3d', 'top', 'front', 'side')
        """
        if view_type == 'top':
            self.ax.view_init(elev=90, azim=-90)
        elif view_type == 'front':
            self.ax.view_init(elev=0, azim=-90)
        elif view_type == 'side':
            self.ax.view_init(elev=0, azim=0)
        else:  # '3d'
            self.ax.view_init(elev=30, azim=45)
        
        self.view_mode = view_type
        self.canvas.draw_idle()
    
    def set_vector_scale(self, scale):
        """设置矢量缩放因子"""
        self.vector_scale = scale
    
    def set_show_labels(self, show):
        """设置是否显示标签"""
        self.show_labels = show
    
    def set_show_grid(self, show):
        """设置是否显示网格"""
        self.show_grid = show
        self.ax.grid(show)
    
    def update_plot(self):
        """更新绘图（简化的update_plots别名）"""
        self.update_plots()


class TactileSurfaceWidget(PlotWidget):
    """触觉曲面可视化（3x3触点插值显示3D曲面）"""
    
    def __init__(self, config: DemoConfig, parent=None):
        super().__init__(config, parent)
        self.num_taxels = getattr(config.hardware.sensor, 'num_taxels', 9)
        self.rows = getattr(config.hardware.sensor, 'rows', 3)
        self.cols = getattr(config.hardware.sensor, 'cols', 3)
        # 减少网格尺寸以提升性能
        self.grid_lin = np.linspace(-10, 10, 40)
        self.grid_x, self.grid_y = np.meshgrid(self.grid_lin, self.grid_lin)
        self.force_vectors = None
        self.last_update_time = 0
        self.update_interval = 0.2  # 5Hz 更新，减轻卡顿
        self.vector_scale = 1.0
        self.show_labels = True
        self.show_grid = True
        self.visualization_mode = "3d_view"
        self.init_ui()
        self.init_plot()
    
    def init_plot(self):
        self.figure.clear()
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X 位置 (mm)')
        self.ax.set_ylabel('Y 位置 (mm)')
        self.ax.set_zlabel('力 (N)')
        self.ax.set_title('触觉映射 - 3D视图')
        self.surface = None
        self.scatter = None
        self.canvas.draw()
    
    def _get_taxel_positions(self):
        positions = []
        for i in range(self.rows):
            for j in range(self.cols):
                x = j / (self.cols - 1) if self.cols > 1 else 0.5
                y = i / (self.rows - 1) if self.rows > 1 else 0.5
                positions.append([x, y])
        return np.array(positions)
    
    def _interpolate_grid(self, coords: np.ndarray, values: np.ndarray) -> np.ndarray:
        eps = 1e-6
        grid_z = np.zeros_like(self.grid_x, dtype=float)
        for i in range(self.grid_x.shape[0]):
            dx = coords[:, 0][:, None] - self.grid_x[i, :][None, :]
            dy = coords[:, 1][:, None] - self.grid_y[i, :][None, :]
            dist = np.sqrt(dx * dx + dy * dy) + eps
            w = 1.0 / dist
            grid_z[i, :] = np.sum(w * values[:, None], axis=0) / np.sum(w, axis=0)
        return grid_z
    
    def update_data(self, data):
        try:
            now = time.time()
            if now - self.last_update_time < self.update_interval:
                return
            if hasattr(data, 'force_vectors'):
                fv = np.array(data.force_vectors)
            elif isinstance(data, dict) and 'force_vectors' in data:
                fv = np.array(data['force_vectors'])
            else:
                return
            if fv.size == 0:
                return
            magnitudes = np.linalg.norm(fv, axis=1) * self.vector_scale
            coords = (self._get_taxel_positions() - 0.5) * 10.0
            grid_z = self._interpolate_grid(coords, magnitudes)
            
            # 清理旧图元
            self.ax.cla()
            self.ax.set_xlabel('X 位置 (mm)')
            self.ax.set_ylabel('Y 位置 (mm)')
            self.ax.set_zlabel('力 (N)')
            self.ax.set_title('触觉映射 - 3D视图')
            self.ax.grid(self.show_grid)

            self.surface = self.ax.plot_surface(self.grid_x, self.grid_y, grid_z, cmap=plt.cm.viridis,
                                                linewidth=0, antialiased=True, alpha=0.8)
            self.scatter = self.ax.scatter(coords[:, 0], coords[:, 1], magnitudes, c='red', s=40)
            self.canvas.draw_idle()
            self.last_update_time = now
        except Exception as e:
            print(f"更新触觉曲面错误: {e}")

    # 与 VectorFieldWidget 接口兼容，占位实现
    def update_contact_data(self, contact_data):
        return

    def set_visualization_mode(self, mode):
        """兼容主窗口的模式切换，3D曲面默认模式"""
        self.visualization_mode = mode

    def set_vector_scale(self, scale: float):
        """兼容矢量缩放控制"""
        self.vector_scale = scale

    def set_show_labels(self, show: bool):
        """曲面视图不显示标签，占位以防调用出错"""
        self.show_labels = show

    def set_show_grid(self, show: bool):
        """切换网格显示"""
        self.show_grid = show
        self.ax.grid(show)


class ForcePlotWidget(PlotWidget):
    """力数据绘图部件 - 增强版：支持三维力统计"""
    
    def __init__(self, config: DemoConfig, parent=None):
        """
        初始化力数据绘图部件
        
        Args:
            config: 系统配置
            parent: 父部件
        """
        super().__init__(config, parent)
        
        # 力数据特定的初始化
        self.force_range = (0, 100)
        self.total_force_history = []
        self.average_force_history = []
        self.max_force_history = []
        self.time_history = []
        self.current_force_data = None
        
        # 三维力数据
        self.force_vectors = None
        
        # 平滑处理参数
        self.smoothing_factor = 0.8
        self.prev_force_data = None
        
        # 初始化UI和绘图
        self.init_ui()
        self.init_plot()
    
    def init_ui(self):
        """初始化用户界面"""
        super().init_ui()
    
    def init_plot(self):
        """初始化力数据绘图 - 增强三维力显示"""
        self.figure.clear()
        
        # 创建子图
        self.ax1 = self.figure.add_subplot(221)  # 总力（三维力合力）
        self.ax2 = self.figure.add_subplot(222)  # 平均力（三维力分量）
        self.ax3 = self.figure.add_subplot(223)  # 最大力（三维力最大分量）
        self.ax4 = self.figure.add_subplot(224)  # 力分布（三维力分量分布）
        
        # 设置图形标题
        self.figure.suptitle('三维力数据统计', fontsize=14, fontweight='bold')
        
        # 总力图（显示合力大小）
        self.total_force_line, = self.ax1.plot([], [], 'r-', linewidth=2, label='合力大小')
        self.total_fx_line, = self.ax1.plot([], [], 'g--', linewidth=1, alpha=0.7, label='Fx')
        self.total_fy_line, = self.ax1.plot([], [], 'b--', linewidth=1, alpha=0.7, label='Fy')
        self.total_fz_line, = self.ax1.plot([], [], 'k--', linewidth=1, alpha=0.7, label='Fz')
        self.ax1.set_title('合力与分量')
        self.ax1.set_xlabel('时间 (帧)')
        self.ax1.set_ylabel('力 (N)')
        self.ax1.legend(loc='upper right', fontsize=8)
        self.ax1.grid(True, alpha=0.3)
        
        # 平均力图（显示三维力分量的平均值）
        self.mean_fx_line, = self.ax2.plot([], [], 'g-', linewidth=2, label='平均 Fx')
        self.mean_fy_line, = self.ax2.plot([], [], 'b-', linewidth=2, label='平均 Fy')
        self.mean_fz_line, = self.ax2.plot([], [], 'k-', linewidth=2, label='平均 Fz')
        self.ax2.set_title('平均力分量')
        self.ax2.set_xlabel('时间 (帧)')
        self.ax2.set_ylabel('平均力 (N)')
        self.ax2.legend(loc='upper right', fontsize=8)
        self.ax2.grid(True, alpha=0.3)
        
        # 最大力图（显示三维力分量的最大值）
        self.max_fx_line, = self.ax3.plot([], [], 'g-.', linewidth=2, label='最大 Fx')
        self.max_fy_line, = self.ax3.plot([], [], 'b-.', linewidth=2, label='最大 Fy')
        self.max_fz_line, = self.ax3.plot([], [], 'k-.', linewidth=2, label='最大 Fz')
        self.ax3.set_title('最大力分量')
        self.ax3.set_xlabel('时间 (帧)')
        self.ax3.set_ylabel('最大力 (N)')
        self.ax3.legend(loc='upper right', fontsize=8)
        self.ax3.grid(True, alpha=0.3)
        
        # 力分布图（三维力分量分布）
        self.force_distribution_bars = self.ax4.bar([], [])
        self.ax4.set_title('三维力分量分布')
        self.ax4.set_xlabel('力分量')
        self.ax4.set_ylabel('统计值')
        self.ax4.grid(True, alpha=0.3, axis='y')
        
        # 调整布局 - 修复tight_layout警告
        self.figure.subplots_adjust(left=0.1, right=0.95, bottom=0.1, top=0.9, hspace=0.4, wspace=0.4)
        
        # 绘制初始图形
        self.canvas.draw()
    
    def update_data(self, data):
        """
        更新力数据（支持三维力数据）
        
        Args:
            data: 数据对象
        """
        # 尝试提取三维力数据
        force_vectors = None
        
        if hasattr(data, 'force_vectors'):
            # 直接有force_vectors属性
            force_vectors = np.array(data.force_vectors)
        elif hasattr(data, 'tactile_array'):
            # 从tactile_array中提取三维力
            tactile_array = data.tactile_array
            if tactile_array.ndim == 2 and tactile_array.shape[1] >= 3:
                force_vectors = tactile_array[:, :3]
        elif isinstance(data, dict) and 'force_vectors' in data:
            # 字典格式
            force_vectors = np.array(data['force_vectors'])
        elif isinstance(data, np.ndarray) and data.ndim == 2 and data.shape[1] >= 3:
            # 直接是三维力数组
            force_vectors = data[:, :3]
        
        if force_vectors is not None and len(force_vectors) > 0:
            try:
                self.force_vectors = force_vectors
                
                # 计算三维力统计
                total_force = np.sum(force_vectors, axis=0)  # 合力向量
                resultant_magnitude = np.linalg.norm(total_force)  # 合力大小
                
                mean_force = np.mean(force_vectors, axis=0)  # 平均力分量
                max_force = np.max(force_vectors, axis=0)    # 最大力分量
                
                # 更新时间戳
                current_time = getattr(data, 'timestamp', len(self.time_history))
                
                # 更新历史数据
                self.total_force_history.append(resultant_magnitude)
                self.average_force_history.append(mean_force)
                self.max_force_history.append(max_force)
                self.time_history.append(current_time)
                
                # 限制历史数据长度
                if len(self.time_history) > self.max_data_points:
                    self.total_force_history.pop(0)
                    self.average_force_history.pop(0)
                    self.max_force_history.pop(0)
                    self.time_history.pop(0)
                
                self.pending_update = True
                
            except Exception as e:
                print(f"更新力数据错误: {e}")
    
    def update_plots(self):
        """
        更新所有子图（无参数版本）
        在定时器中调用
        """
        if not self.pending_update or self.force_vectors is None:
            return
            
        self.pending_update = False
        current_time = time.time()
        
        # 控制更新频率
        if current_time - self.last_update_time < self.update_interval:
            return
            
        self.last_update_time = current_time
        
        try:
            if len(self.time_history) == 0:
                return
            
            # 更新总力图
            time_indices = range(len(self.total_force_history))
            self.total_force_line.set_data(time_indices, self.total_force_history)
            
            # 动态调整总力图范围
            if self.total_force_history:
                y_min = min(self.total_force_history)
                y_max = max(self.total_force_history)
                margin = max(10, (y_max - y_min) * 0.2)  # 增加边距
                self.ax1.set_xlim(0, len(self.total_force_history)-1)
                self.ax1.set_ylim(y_min - margin, y_max + margin)
            
            # 更新平均力图
            if len(self.average_force_history) > 0:
                # 提取三维力分量的平均值
                fx_history = [f[0] for f in self.average_force_history]
                fy_history = [f[1] for f in self.average_force_history]
                fz_history = [f[2] for f in self.average_force_history]
                
                self.mean_fx_line.set_data(time_indices, fx_history)
                self.mean_fy_line.set_data(time_indices, fy_history)
                self.mean_fz_line.set_data(time_indices, fz_history)
                
                # 动态调整平均力图范围
                all_avg_data = fx_history + fy_history + fz_history
                if all_avg_data:
                    y_min = min(all_avg_data)
                    y_max = max(all_avg_data)
                    margin = max(2, (y_max - y_min) * 0.2)
                    self.ax2.set_xlim(0, len(self.average_force_history)-1)
                    self.ax2.set_ylim(y_min - margin, y_max + margin)
            
            # 更新最大力图
            if len(self.max_force_history) > 0:
                # 提取三维力分量的最大值
                fx_max_history = [f[0] for f in self.max_force_history]
                fy_max_history = [f[1] for f in self.max_force_history]
                fz_max_history = [f[2] for f in self.max_force_history]
                
                self.max_fx_line.set_data(time_indices, fx_max_history)
                self.max_fy_line.set_data(time_indices, fy_max_history)
                self.max_fz_line.set_data(time_indices, fz_max_history)
                
                # 动态调整最大力图范围
                all_max_data = fx_max_history + fy_max_history + fz_max_history
                if all_max_data:
                    y_min = min(all_max_data)
                    y_max = max(all_max_data)
                    margin = max(10, (y_max - y_min) * 0.2)
                    self.ax3.set_xlim(0, len(self.max_force_history)-1)
                    self.ax3.set_ylim(y_min - margin, y_max + margin)
            
            # 更新力分布图
            self.update_force_distribution()
            
            # 异步重绘
            self.canvas.draw_idle()
            
        except Exception as e:
            print(f"更新力数据绘图错误: {e}")
    
    def update_force_distribution(self):
        """更新力分布图"""
        if self.force_vectors is None:
            return
            
        self.ax4.clear()
        
        try:
            # 创建力分布直方图 - 显示三维力分量的统计
            force_ranges = [-50, -30, -10, 10, 30, 50]  # 调整范围，包含负值
            hist_fx, _ = np.histogram(self.force_vectors[:, 0], bins=force_ranges)
            hist_fy, _ = np.histogram(self.force_vectors[:, 1], bins=force_ranges)
            hist_fz, _ = np.histogram(self.force_vectors[:, 2], bins=force_ranges)
            
            # 绘制分组条形图
            x_pos = np.arange(len(force_ranges)-1)
            bar_width = 0.25
            
            bars_fx = self.ax4.bar(x_pos - bar_width, hist_fx, width=bar_width, color='r', alpha=0.7, label='Fx')
            bars_fy = self.ax4.bar(x_pos, hist_fy, width=bar_width, color='g', alpha=0.7, label='Fy')
            bars_fz = self.ax4.bar(x_pos + bar_width, hist_fz, width=bar_width, color='b', alpha=0.7, label='Fz')
            
            # 设置x轴标签
            bin_labels = []
            for i in range(len(force_ranges)-1):
                bin_labels.append(f'{force_ranges[i]}-{force_ranges[i+1]}')
            
            self.ax4.set_xticks(x_pos)
            self.ax4.set_xticklabels(bin_labels, rotation=45)
            self.ax4.set_title('三维力分量分布')
            self.ax4.set_xlabel('力范围 (N)')
            self.ax4.set_ylabel('单元数')
            self.ax4.legend(loc='upper right', fontsize=8)
            self.ax4.grid(True, alpha=0.3, axis='y')
            
            # 动态调整y轴范围
            all_hist = list(hist_fx) + list(hist_fy) + list(hist_fz)
            if len(all_hist) > 0:
                y_max = max(all_hist)
                self.ax4.set_ylim(0, y_max * 1.3)
            
        except Exception as e:
            print(f"更新力分布图错误: {e}")
