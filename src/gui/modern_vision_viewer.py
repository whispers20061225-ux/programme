from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

try:
    from config import CameraConfig
except ImportError:
    try:
        from ..config import CameraConfig
    except ImportError:
        CameraConfig = object  # type: ignore[assignment]

from utils.transformations import rotation_matrix_to_euler

_DEPTH_FALLBACK_MAX_METERS = 5.0


def colorize_depth_for_display(depth: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if depth is None:
        return None
    arr = np.asarray(depth)
    if arr.ndim == 3 and arr.shape[2] == 3:
        if arr.dtype != np.uint8:
            arr = np.clip(arr, 0, 255).astype(np.uint8)
        return np.require(arr, requirements=["C"])
    if arr.ndim == 3:
        arr = arr[:, :, 0]
    if arr.ndim != 2:
        return None

    arrf = arr.astype(np.float32, copy=False)
    if arr.dtype == np.uint16:
        valid = arrf > 0.0
        arrf = arrf / 1000.0
    else:
        valid = np.isfinite(arrf) & (arrf > 0.0)

    if not np.any(valid):
        return np.zeros(arr.shape + (3,), dtype=np.uint8)

    sample_step = 4 if arrf.size >= (160 * 120) else 1
    sample = arrf[::sample_step, ::sample_step]
    sample_valid = valid[::sample_step, ::sample_step]
    valid_values = sample[sample_valid]
    if valid_values.size < 64:
        valid_values = arrf[valid]
    near = float(np.percentile(valid_values, 5.0))
    far = float(np.percentile(valid_values, 95.0))
    if not np.isfinite(near):
        near = 0.1
    if not np.isfinite(far):
        far = near + 1.0
    near = max(0.05, near)
    far = max(near + 0.1, far)

    norm = np.zeros_like(arrf, dtype=np.float32)
    norm[valid] = np.clip((arrf[valid] - near) / (far - near), 0.0, 1.0)
    norm = 1.0 - norm

    anchors = np.array([0.0, 0.25, 0.5, 0.75, 1.0], dtype=np.float32)
    colors = np.array(
        [
            [0, 32, 128],
            [0, 128, 255],
            [0, 255, 255],
            [255, 255, 0],
            [255, 96, 0],
        ],
        dtype=np.float32,
    )

    flat = norm.reshape(-1)
    rgb = np.empty((flat.size, 3), dtype=np.uint8)
    for channel in range(3):
        rgb[:, channel] = np.interp(flat, anchors, colors[:, channel]).astype(np.uint8)
    rgb = rgb.reshape(arr.shape + (3,))
    rgb[~valid] = 0
    return np.require(rgb, requirements=["C"])


def rgb_array_to_qimage(image: Optional[np.ndarray]) -> Optional[QImage]:
    if image is None:
        return None
    arr = np.asarray(image)
    if arr.ndim != 3 or arr.shape[2] != 3:
        return None
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    arr = np.require(arr, requirements=["C"])
    height, width, _ = arr.shape
    return QImage(arr.data, width, height, arr.strides[0], QImage.Format_RGB888).copy()


def depth_array_to_qimage(depth: Optional[np.ndarray]) -> Optional[QImage]:
    if depth is None:
        return None
    arr = np.asarray(depth)
    if arr.ndim == 3 and arr.shape[2] == 3:
        return rgb_array_to_qimage(arr)
    if arr.ndim == 3:
        arr = arr[:, :, 0]
    if arr.ndim != 2:
        return None
    colorized = colorize_depth_for_display(arr)
    if colorized is not None:
        return rgb_array_to_qimage(colorized)
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    arr8 = np.require(arr, requirements=["C"])
    height, width = arr8.shape
    return QImage(arr8.data, width, height, arr8.strides[0], QImage.Format_Grayscale8).copy()


class VideoFrameWidget(QWidget):
    def __init__(self, *, mode: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.mode = mode
        self.setMinimumSize(640, 480)
        self._image_scale = 1.0
        self._frame: Optional[QImage] = None
        self._placeholder = "等待图像"
        self._detections: List[Dict[str, Any]] = []
        self._poses: List[Dict[str, Any]] = []
        self._show_bounding_boxes = True
        self._show_labels = True
        self._show_confidence = True
        self._show_keypoints = True
        self._show_pose_axes = True

    def set_image_scale(self, scale: float) -> None:
        self._image_scale = max(0.1, float(scale))
        self.update()

    def clear_frame(self, placeholder: Optional[str] = None) -> None:
        self._frame = None
        if placeholder:
            self._placeholder = placeholder
        self.update()

    def set_rgb_frame(self, image: Optional[np.ndarray]) -> None:
        self._frame = rgb_array_to_qimage(image)
        self.update()

    def set_qimage_frame(self, frame: Optional[QImage], placeholder: Optional[str] = None) -> None:
        self._frame = None if frame is None else QImage(frame)
        if placeholder:
            self._placeholder = placeholder
        self.update()

    def set_depth_frame(self, depth: Optional[np.ndarray]) -> None:
        self._frame = depth_array_to_qimage(depth)
        self.update()

    def set_detection_results(self, detections: Optional[List[Dict[str, Any]]]) -> None:
        self._detections = list(detections or [])
        self.update()

    def set_pose_estimations(self, poses: Optional[List[Dict[str, Any]]]) -> None:
        self._poses = list(poses or [])
        self.update()

    def set_overlay_options(self, *, show_bounding_boxes: bool, show_labels: bool, show_confidence: bool, show_keypoints: bool, show_pose_axes: bool) -> None:
        self._show_bounding_boxes = bool(show_bounding_boxes)
        self._show_labels = bool(show_labels)
        self._show_confidence = bool(show_confidence)
        self._show_keypoints = bool(show_keypoints)
        self._show_pose_axes = bool(show_pose_axes)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#2b2b2b"))
        if self._frame is None or self._frame.isNull():
            painter.setPen(QColor("#cccccc"))
            painter.setFont(QFont("Microsoft YaHei", 12))
            painter.drawText(self.rect(), Qt.AlignCenter, self._placeholder)
            return
        target_rect = self._compute_target_rect(self._frame.width(), self._frame.height())
        painter.drawImage(target_rect, self._frame)
        if self.mode == "detection":
            self._draw_detections(painter, target_rect)
        elif self.mode == "pose":
            self._draw_poses(painter, target_rect)

    def _compute_target_rect(self, image_width: int, image_height: int) -> QRectF:
        fit_scale = min(self.width() / max(1, image_width), self.height() / max(1, image_height))
        scale = fit_scale * self._image_scale
        draw_width = image_width * scale
        draw_height = image_height * scale
        return QRectF((self.width() - draw_width) * 0.5, (self.height() - draw_height) * 0.5, draw_width, draw_height)

    def _draw_detections(self, painter: QPainter, target_rect: QRectF) -> None:
        if not self._show_bounding_boxes or not self._detections or self._frame is None:
            return
        sx = target_rect.width() / max(1, self._frame.width())
        sy = target_rect.height() / max(1, self._frame.height())
        for detection in self._detections:
            bbox = detection.get("bbox") or []
            if len(bbox) != 4:
                continue
            x1, y1, x2, y2 = [float(v) for v in bbox]
            color = self._to_qcolor(detection.get("color", (0, 255, 0)))
            painter.setPen(QPen(color, 2))
            painter.drawRect(QRectF(target_rect.left() + x1 * sx, target_rect.top() + y1 * sy, max(1.0, (x2 - x1) * sx), max(1.0, (y2 - y1) * sy)))
            if not self._show_labels:
                continue
            label = str(detection.get("label", "object") or "object")
            confidence = float(detection.get("confidence", 0.0) or 0.0)
            text = label if not self._show_confidence else f"{label}: {confidence:.2f}"
            painter.fillRect(QRectF(target_rect.left() + x1 * sx, target_rect.top() + max(0.0, y1 * sy - 22.0), max(90.0, len(text) * 8.0), 20.0), color)
            painter.setPen(QColor("white"))
            painter.setFont(QFont("Microsoft YaHei", 9))
            painter.drawText(QPointF(target_rect.left() + x1 * sx + 4.0, target_rect.top() + max(14.0, y1 * sy - 6.0)), text)

    def _draw_poses(self, painter: QPainter, target_rect: QRectF) -> None:
        if not self._poses or self._frame is None:
            return
        sx = target_rect.width() / max(1, self._frame.width())
        sy = target_rect.height() / max(1, self._frame.height())
        for pose in self._poses:
            bbox = pose.get("bbox") or []
            if self._show_bounding_boxes and len(bbox) == 4:
                x1, y1, x2, y2 = [float(v) for v in bbox]
                painter.setPen(QPen(QColor("#4CAF50"), 2))
                painter.drawRect(QRectF(target_rect.left() + x1 * sx, target_rect.top() + y1 * sy, max(1.0, (x2 - x1) * sx), max(1.0, (y2 - y1) * sy)))
            keypoints = pose.get("keypoints") or []
            if self._show_keypoints:
                painter.setPen(QPen(QColor("#4CAF50"), 2))
                painter.setBrush(QColor("#4CAF50"))
                for idx in range(0, len(keypoints), 2):
                    if idx + 1 >= len(keypoints):
                        break
                    x = float(keypoints[idx])
                    y = float(keypoints[idx + 1])
                    painter.drawEllipse(QPointF(target_rect.left() + x * sx, target_rect.top() + y * sy), 3.0, 3.0)
            if self._show_pose_axes and len(bbox) == 4:
                x_center = (float(bbox[0]) + float(bbox[2])) * 0.5
                y_center = (float(bbox[1]) + float(bbox[3])) * 0.5
                center = QPointF(target_rect.left() + x_center * sx, target_rect.top() + y_center * sy)
                painter.setPen(QPen(QColor("red"), 2))
                painter.drawLine(center, QPointF(center.x() + 30.0, center.y()))
                painter.setPen(QPen(QColor("green"), 2))
                painter.drawLine(center, QPointF(center.x(), center.y() - 30.0))
                painter.setPen(QPen(QColor("blue"), 2))
                painter.drawLine(center, QPointF(center.x(), center.y() + 30.0))

    @staticmethod
    def _to_qcolor(color: Any) -> QColor:
        if isinstance(color, QColor):
            return color
        if isinstance(color, (tuple, list)) and len(color) >= 3:
            return QColor(int(color[2]), int(color[1]), int(color[0]))
        return QColor("#4CAF50")

class VisionViewer(QWidget):
    capture_request = pyqtSignal()
    detection_request = pyqtSignal()
    calibration_request = pyqtSignal()
    save_image_request = pyqtSignal(str)
    auto_detect_toggled = pyqtSignal(bool)
    connect_request = pyqtSignal()
    disconnect_request = pyqtSignal()
    self_check_request = pyqtSignal()
    pointcloud_request = pyqtSignal()
    pointcloud_save_request = pyqtSignal(str)

    def __init__(self, config: CameraConfig, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.config = config
        self.current_image: Optional[np.ndarray] = None
        self.original_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self._latest_rgb_qimage: Optional[QImage] = None
        self.detection_results: List[Dict[str, Any]] = []
        self.pose_estimations: List[Dict[str, Any]] = []
        self.pointcloud_data: Optional[Dict[str, np.ndarray]] = None
        self.show_bounding_boxes = True
        self.show_keypoints = True
        self.show_pose_axes = True
        self.show_depth_map = True
        self.detection_confidence = 0.5
        self.auto_detect_enabled = False
        self.image_scale = 1.0
        self.camera_connected = False
        self.camera_streaming = False
        self.camera_calibrated = False
        self.connect_pending = False
        self._last_camera_status_signature: Optional[Tuple[Any, ...]] = None
        self.init_ui()
        self._sync_overlay_options()

    def init_ui(self) -> None:
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        splitter = QSplitter(Qt.Horizontal)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.image_tabs = QTabWidget()

        self.rgb_view = VideoFrameWidget(mode="rgb")
        self.depth_view = VideoFrameWidget(mode="depth")
        self.detection_view = VideoFrameWidget(mode="detection")
        self.pose_view = VideoFrameWidget(mode="pose")

        for label, view in (("RGB图像", self.rgb_view), ("深度图", self.depth_view), ("检测结果", self.detection_view), ("姿态估计", self.pose_view)):
            tab = QWidget()
            layout = QVBoxLayout(tab)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.addWidget(view)
            self.image_tabs.addTab(tab, label)
        self.rgb_tab = self.image_tabs.widget(0)
        self.depth_tab = self.image_tabs.widget(1)
        self.detection_tab = self.image_tabs.widget(2)
        self.pose_tab = self.image_tabs.widget(3)
        self.pointcloud_tab = QWidget()
        pointcloud_layout = QVBoxLayout(self.pointcloud_tab)
        self.pointcloud_figure = Figure(figsize=(5, 4))
        self.pointcloud_canvas = FigureCanvas(self.pointcloud_figure)
        self.pointcloud_ax = self.pointcloud_figure.add_subplot(111, projection="3d")
        self.pointcloud_ax.set_title("融合点云")
        self.pointcloud_ax.set_xlabel("X")
        self.pointcloud_ax.set_ylabel("Y")
        self.pointcloud_ax.set_zlabel("Z")
        pointcloud_layout.addWidget(self.pointcloud_canvas)
        self.image_tabs.addTab(self.pointcloud_tab, "??")
        self.image_tabs.currentChanged.connect(self._handle_tab_changed)

        left_layout.addWidget(self.image_tabs)

        controls = QWidget()
        controls_layout = QHBoxLayout(controls)
        controls_layout.setContentsMargins(0, 5, 0, 5)
        controls_layout.addWidget(QLabel("缩放:"))
        self.zoom_combo = QComboBox()
        self.zoom_combo.addItems(["25%", "50%", "100%", "150%", "200%"])
        self.zoom_combo.setCurrentText("100%")
        self.zoom_combo.currentTextChanged.connect(self.change_zoom)
        controls_layout.addWidget(self.zoom_combo)
        self.capture_btn = QPushButton("捕获")
        self.capture_btn.clicked.connect(self.capture_image)
        controls_layout.addWidget(self.capture_btn)
        self.detect_btn = QPushButton("检测")
        self.detect_btn.clicked.connect(self.detect_objects)
        controls_layout.addWidget(self.detect_btn)
        self.save_btn = QPushButton("保存")
        self.save_btn.clicked.connect(self.save_image)
        controls_layout.addWidget(self.save_btn)
        controls_layout.addStretch()
        left_layout.addWidget(controls)
        splitter.addWidget(left_widget)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        content_widget = QWidget()
        content_layout = QVBoxLayout(content_widget)
        content_layout.setContentsMargins(5, 5, 5, 5)
        content_layout.setSpacing(10)
        self.create_camera_status_group(content_layout)
        self.create_detection_settings_group(content_layout)
        self.create_pose_info_group(content_layout)
        self.create_pointcloud_group(content_layout)
        self.create_display_options_group(content_layout)
        self.create_camera_control_group(content_layout)
        scroll_area.setWidget(content_widget)
        right_layout.addWidget(scroll_area)
        splitter.addWidget(right_widget)
        splitter.setSizes([800, 400])
        main_layout.addWidget(splitter)

    def create_camera_status_group(self, layout: QVBoxLayout) -> None:
        group = QGroupBox("相机状态")
        form = QFormLayout(group)
        self.connection_status_label = QLabel("未连接")
        self.connection_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold;")
        form.addRow("连接状态:", self.connection_status_label)
        self.stream_status_label = QLabel("未流式传输")
        self.stream_status_label.setStyleSheet("color: #ff6b6b;")
        form.addRow("流状态:", self.stream_status_label)
        self.calibration_status_label = QLabel("未校准")
        self.calibration_status_label.setStyleSheet("color: #ff6b6b;")
        form.addRow("校准状态:", self.calibration_status_label)
        self.resolution_label = QLabel("N/A")
        form.addRow("分辨率:", self.resolution_label)
        self.fps_label = QLabel("0.0 FPS")
        form.addRow("显示帧率:", self.fps_label)
        self.rx_fps_label = QLabel("0.0 FPS")
        form.addRow("接收帧率:", self.rx_fps_label)
        self.drop_label = QLabel("0")
        form.addRow("丢帧计数:", self.drop_label)
        self.age_label = QLabel("N/A")
        form.addRow("最新帧延迟:", self.age_label)
        self.stall_label = QLabel("0")
        form.addRow("卡顿次数:", self.stall_label)
        self.serial_label = QLabel("N/A")
        form.addRow("序列号:", self.serial_label)
        self.depth_status_label = QLabel("N/A")
        form.addRow("深度状态:", self.depth_status_label)
        layout.addWidget(group)

        auto_group = QGroupBox("实时检测")
        auto_layout = QHBoxLayout(auto_group)
        self.auto_detect_checkbox = QCheckBox("启用实时检测")
        self.auto_detect_checkbox.stateChanged.connect(self.toggle_auto_detect)
        auto_layout.addWidget(self.auto_detect_checkbox)
        layout.addWidget(auto_group)

    def create_detection_settings_group(self, layout: QVBoxLayout) -> None:
        group = QGroupBox("检测设置")
        group_layout = QVBoxLayout(group)
        model_layout = QHBoxLayout()
        model_layout.addWidget(QLabel("检测模型"))
        self.model_combo = QComboBox()
        self.model_combo.addItems(["YOLOv5", "YOLOv8", "SSD", "Faster R-CNN", "自定义模型"])
        self.model_combo.setCurrentText(getattr(self.config, "detection_model", "YOLOv5"))
        model_layout.addWidget(self.model_combo)
        group_layout.addLayout(model_layout)
        confidence_layout = QHBoxLayout()
        confidence_layout.addWidget(QLabel("置信度"))
        self.confidence_spin = QDoubleSpinBox()
        self.confidence_spin.setRange(0.0, 1.0)
        self.confidence_spin.setSingleStep(0.05)
        self.confidence_spin.setValue(getattr(self.config, "confidence_threshold", 0.5))
        self.confidence_spin.valueChanged.connect(self.update_detection_confidence)
        confidence_layout.addWidget(self.confidence_spin)
        group_layout.addLayout(confidence_layout)
        nms_layout = QHBoxLayout()
        nms_layout.addWidget(QLabel("NMS阈值"))
        self.nms_spin = QDoubleSpinBox()
        self.nms_spin.setRange(0.0, 1.0)
        self.nms_spin.setSingleStep(0.05)
        self.nms_spin.setValue(getattr(self.config, "nms_threshold", 0.45))
        nms_layout.addWidget(self.nms_spin)
        group_layout.addLayout(nms_layout)
        layout.addWidget(group)

    def create_pose_info_group(self, layout: QVBoxLayout) -> None:
        group = QGroupBox("姿态估计结果")
        form = QFormLayout(group)
        self.pose_object_label = QLabel("N/A")
        self.pose_method_label = QLabel("N/A")
        self.pose_position_label = QLabel("N/A")
        self.pose_rotation_label = QLabel("N/A")
        self.pose_confidence_label = QLabel("N/A")
        form.addRow("目标:", self.pose_object_label)
        form.addRow("方法:", self.pose_method_label)
        form.addRow("位置(x,y,z):", self.pose_position_label)
        form.addRow("姿态(r,p,y):", self.pose_rotation_label)
        form.addRow("置信度:", self.pose_confidence_label)
        layout.addWidget(group)
    def create_pointcloud_group(self, layout: QVBoxLayout) -> None:
        group = QGroupBox("点云")
        group_layout = QVBoxLayout(group)
        self.show_pointcloud_btn = QPushButton("显示点云")
        self.show_pointcloud_btn.clicked.connect(self.request_pointcloud_display)
        group_layout.addWidget(self.show_pointcloud_btn)
        self.save_pointcloud_btn = QPushButton("保存融合点云")
        self.save_pointcloud_btn.clicked.connect(self.request_pointcloud_save)
        group_layout.addWidget(self.save_pointcloud_btn)
        layout.addWidget(group)

    def create_display_options_group(self, layout: QVBoxLayout) -> None:
        group = QGroupBox("显示选项")
        group_layout = QVBoxLayout(group)
        self.show_bbox_check = QCheckBox("显示边界框")
        self.show_bbox_check.setChecked(True)
        self.show_bbox_check.stateChanged.connect(self.toggle_bounding_boxes)
        group_layout.addWidget(self.show_bbox_check)
        self.show_labels_check = QCheckBox("显示标签")
        self.show_labels_check.setChecked(True)
        self.show_labels_check.stateChanged.connect(lambda _: self._sync_overlay_options())
        group_layout.addWidget(self.show_labels_check)
        self.show_confidence_check = QCheckBox("显示置信度")
        self.show_confidence_check.setChecked(True)
        self.show_confidence_check.stateChanged.connect(lambda _: self._sync_overlay_options())
        group_layout.addWidget(self.show_confidence_check)
        self.show_keypoints_check = QCheckBox("显示关键点")
        self.show_keypoints_check.setChecked(True)
        self.show_keypoints_check.stateChanged.connect(self.toggle_keypoints)
        group_layout.addWidget(self.show_keypoints_check)
        self.show_pose_axes_check = QCheckBox("显示姿态轴")
        self.show_pose_axes_check.setChecked(True)
        self.show_pose_axes_check.stateChanged.connect(self.toggle_pose_axes)
        group_layout.addWidget(self.show_pose_axes_check)
        self.show_depth_check = QCheckBox("显示深度图")
        self.show_depth_check.setChecked(True)
        self.show_depth_check.stateChanged.connect(self.toggle_depth_map)
        group_layout.addWidget(self.show_depth_check)
        layout.addWidget(group)

    def create_camera_control_group(self, layout: QVBoxLayout) -> None:
        group = QGroupBox("相机控制")
        group_layout = QVBoxLayout(group)
        btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("连接相机")
        self.connect_btn.clicked.connect(self.connect_camera)
        btn_layout.addWidget(self.connect_btn)
        self.disconnect_btn = QPushButton("断开相机")
        self.disconnect_btn.clicked.connect(self.disconnect_camera)
        btn_layout.addWidget(self.disconnect_btn)
        group_layout.addLayout(btn_layout)
        stream_layout = QHBoxLayout()
        self.start_stream_btn = QPushButton("开始流")
        self.start_stream_btn.clicked.connect(self.start_stream)
        stream_layout.addWidget(self.start_stream_btn)
        self.stop_stream_btn = QPushButton("停止流")
        self.stop_stream_btn.clicked.connect(self.stop_stream)
        stream_layout.addWidget(self.stop_stream_btn)
        group_layout.addLayout(stream_layout)
        self.calibrate_btn = QPushButton("校准相机")
        self.calibrate_btn.clicked.connect(self.calibrate_camera)
        group_layout.addWidget(self.calibrate_btn)
        self.self_check_btn = QPushButton("设备自检")
        self.self_check_btn.clicked.connect(self.request_self_check)
        group_layout.addWidget(self.self_check_btn)
        layout.addWidget(group)

    def display_pointcloud(self) -> None:
        if self.pointcloud_data is None:
            self.pointcloud_ax.clear()
            self.pointcloud_ax.set_title("暂无点云数据")
            self.pointcloud_canvas.draw_idle()
            return
        points = self.pointcloud_data.get("points")
        colors = self.pointcloud_data.get("colors")
        if points is None or len(points) == 0:
            self.pointcloud_ax.clear()
            self.pointcloud_ax.set_title("点云为空")
            self.pointcloud_canvas.draw_idle()
            return
        self.pointcloud_ax.clear()
        if colors is not None and len(colors) == len(points):
            self.pointcloud_ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=1, depthshade=False)
        else:
            self.pointcloud_ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="tab:blue", s=1, depthshade=False)
        self.pointcloud_ax.set_title("融合点云")
        self.pointcloud_ax.set_xlabel("X")
        self.pointcloud_ax.set_ylabel("Y")
        self.pointcloud_ax.set_zlabel("Z")
        self.pointcloud_ax.view_init(elev=20, azim=45)
        self.pointcloud_canvas.draw_idle()

    def update_rgb_qimage(self, image: Any, qimage: Optional[QImage]) -> None:
        self.current_image = image
        self.original_image = image
        self._latest_rgb_qimage = None if qimage is None else QImage(qimage)
        self._refresh_rgb_views()

    def update_image(self, image: Any, image_type: str = "rgb") -> None:
        if image_type == "rgb":
            self.current_image = image
            self.original_image = image
            self._latest_rgb_qimage = None if image is None else rgb_array_to_qimage(image)
            self._refresh_rgb_views()
        elif image_type == "depth":
            self.depth_image = image
            if self.show_depth_map:
                self.depth_view.set_depth_frame(image)
            else:
                self.depth_view.clear_frame("深度图已关闭")
        elif image_type == "detection":
            self.detection_results = list(image or [])
            self.detection_view.set_detection_results(self.detection_results)
        elif image_type == "pose":
            self.pose_estimations = list(image or [])
            self.pose_view.set_pose_estimations(self.pose_estimations)
            self.update_pose_info()
        elif image_type == "pointcloud":
            self.pointcloud_data = image
            if self.image_tabs.currentWidget() is self.pointcloud_tab:
                self.display_pointcloud()

    def update_pose_info(self) -> None:
        if not self.pose_estimations:
            self.pose_object_label.setText("N/A")
            self.pose_method_label.setText("N/A")
            self.pose_position_label.setText("N/A")
            self.pose_rotation_label.setText("N/A")
            self.pose_confidence_label.setText("N/A")
            return
        pose = self.pose_estimations[0]
        translation = pose.get("translation")
        rotation = pose.get("rotation")
        self.pose_object_label.setText(str(pose.get("object_class", "object")))
        self.pose_method_label.setText(str(pose.get("method", "stub")))
        if translation is not None and hasattr(translation, "__len__") and len(translation) >= 3:
            self.pose_position_label.setText(f"{translation[0]:.2f}, {translation[1]:.2f}, {translation[2]:.2f}")
        else:
            self.pose_position_label.setText("N/A")
        if rotation is not None:
            try:
                roll, pitch, yaw = np.degrees(rotation_matrix_to_euler(rotation))
                self.pose_rotation_label.setText(f"{roll:.1f}, {pitch:.1f}, {yaw:.1f}")
            except Exception:
                self.pose_rotation_label.setText("N/A")
        else:
            self.pose_rotation_label.setText("N/A")
        self.pose_confidence_label.setText(f"{float(pose.get('confidence', 0.0)):.2f}")
    def update_camera_status(self, connected: bool = False, streaming: bool = False, calibrated: bool = False, resolution: str = "N/A", fps: float = 0.0, *, rx_fps: Optional[float] = None, dropped_frames: int = 0, last_frame_age_ms: Optional[float] = None, stall_count: int = 0) -> None:
        self.camera_connected = connected
        self.camera_streaming = streaming
        self.camera_calibrated = calibrated
        effective_rx_fps = float(rx_fps if rx_fps is not None else fps)
        status_signature = (
            bool(connected),
            bool(streaming),
            bool(calibrated),
            str(resolution),
            round(float(fps), 1),
            round(effective_rx_fps, 1),
            int(dropped_frames),
            None if last_frame_age_ms is None else int(float(last_frame_age_ms) // 100),
            int(stall_count),
        )
        if status_signature == self._last_camera_status_signature:
            return
        self._last_camera_status_signature = status_signature
        if connected:
            self.connection_status_label.setText("已连接")
            self.connection_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.connection_status_label.setText("未连接")
            self.connection_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold;")
            self.connect_pending = False
        if streaming:
            self.stream_status_label.setText("流式传输中")
            self.stream_status_label.setStyleSheet("color: #4CAF50;")
        else:
            self.stream_status_label.setText("未流式传输")
            self.stream_status_label.setStyleSheet("color: #ff6b6b;")
        if calibrated:
            self.calibration_status_label.setText("已校准")
            self.calibration_status_label.setStyleSheet("color: #4CAF50;")
        else:
            self.calibration_status_label.setText("未校准")
            self.calibration_status_label.setStyleSheet("color: #ff6b6b;")
        self.resolution_label.setText(resolution)
        self.fps_label.setText(f"{float(fps):.1f} FPS")
        self.rx_fps_label.setText(f"{effective_rx_fps:.1f} FPS")
        self.drop_label.setText(str(int(dropped_frames)))
        self.age_label.setText("N/A" if last_frame_age_ms is None else f"{float(last_frame_age_ms):.0f} ms")
        self.stall_label.setText(str(int(stall_count)))
        if not connected:
            self.serial_label.setText("N/A")
            self.depth_status_label.setText("N/A")
            self.depth_status_label.setStyleSheet("color: #ff6b6b;")
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.start_stream_btn.setEnabled(connected and not streaming)
        self.stop_stream_btn.setEnabled(connected and streaming)
        self.calibrate_btn.setEnabled(connected)
        self.capture_btn.setEnabled(streaming)
        self.detect_btn.setEnabled(streaming)

    def change_zoom(self, zoom_text: str) -> None:
        scale_map = {"25%": 0.25, "50%": 0.5, "100%": 1.0, "150%": 1.5, "200%": 2.0}
        self.image_scale = scale_map.get(zoom_text, 1.0)
        for view in (self.rgb_view, self.depth_view, self.detection_view, self.pose_view):
            view.set_image_scale(self.image_scale)

    def capture_image(self) -> None:
        self.capture_request.emit()

    def detect_objects(self) -> None:
        self.detection_request.emit()

    def toggle_auto_detect(self, state: int) -> None:
        enabled = state == Qt.Checked
        self.auto_detect_enabled = enabled
        self.auto_detect_toggled.emit(enabled)

    def save_image(self) -> None:
        self.save_image_request.emit(f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")

    def request_pointcloud_display(self) -> None:
        self.image_tabs.setCurrentWidget(self.pointcloud_tab)
        self.pointcloud_request.emit()

    def request_pointcloud_save(self) -> None:
        self.pointcloud_save_request.emit(f"pointcloud_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply")

    def connect_camera(self) -> None:
        self.connect_pending = True
        self.connect_request.emit()
        QTimer.singleShot(1500, self._check_connect_result)

    def disconnect_camera(self) -> None:
        self.disconnect_request.emit()

    def request_self_check(self) -> None:
        self.self_check_request.emit()

    def start_stream(self) -> None:
        pass

    def stop_stream(self) -> None:
        pass

    def show_connection_error(self, message: str = "连接失败") -> None:
        self.update_camera_status(connected=False, streaming=False, calibrated=False, resolution="N/A", fps=0.0)
        try:
            QMessageBox.warning(self, "相机连接", message)
        except Exception:
            self.connection_status_label.setText(message)
            self.connection_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold;")

    def _check_connect_result(self) -> None:
        if self.connect_pending and not self.camera_connected:
            self.connect_pending = False
            self.show_connection_error("相机连接失败，请检查 USB 或配置")

    def calibrate_camera(self) -> None:
        self.calibration_request.emit()

    def toggle_bounding_boxes(self, state: int) -> None:
        self.show_bounding_boxes = state == Qt.Checked
        self._sync_overlay_options()

    def toggle_keypoints(self, state: int) -> None:
        self.show_keypoints = state == Qt.Checked
        self._sync_overlay_options()

    def toggle_pose_axes(self, state: int) -> None:
        self.show_pose_axes = state == Qt.Checked
        self._sync_overlay_options()

    def toggle_depth_map(self, state: int) -> None:
        self.show_depth_map = state == Qt.Checked
        if self.show_depth_map and self.depth_image is not None:
            self.depth_view.set_depth_frame(self.depth_image)
        elif not self.show_depth_map:
            self.depth_view.clear_frame("深度图已关闭")

    def update_detection_confidence(self, value: float) -> None:
        self.detection_confidence = float(value)
        self._sync_overlay_options()

    def get_detection_settings(self) -> Dict[str, Any]:
        model_text = self.model_combo.currentText() if self.model_combo else "YOLOv5"
        model_map = {"YOLOv5": "yolov5", "YOLOv8": "yolov8", "SSD": "ssd", "Faster R-CNN": "faster_rcnn", "自定义模型": "custom"}
        return {"model_name": model_map.get(model_text, "yolov5"), "confidence_threshold": float(self.confidence_spin.value()) if self.confidence_spin else 0.5, "iou_threshold": float(self.nms_spin.value()) if self.nms_spin else 0.45}

    def update_self_check_result(self, serial_text: str, depth_status: str, depth_ok: Optional[bool] = None) -> None:
        self.serial_label.setText(serial_text or "N/A")
        self.depth_status_label.setText(depth_status or "N/A")
        if depth_ok is True:
            self.depth_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        elif depth_ok is False:
            self.depth_status_label.setStyleSheet("color: #ff6b6b; font-weight: bold;")
        else:
            self.depth_status_label.setStyleSheet("color: #999999;")

    def _sync_overlay_options(self) -> None:
        options = {
            "show_bounding_boxes": self.show_bounding_boxes,
            "show_labels": self.show_labels_check.isChecked() if hasattr(self, "show_labels_check") else True,
            "show_confidence": self.show_confidence_check.isChecked() if hasattr(self, "show_confidence_check") else True,
            "show_keypoints": self.show_keypoints,
            "show_pose_axes": self.show_pose_axes,
        }
        self.detection_view.set_overlay_options(**options)
        self.pose_view.set_overlay_options(**options)

    def is_depth_tab_active(self) -> bool:
        return hasattr(self, "depth_tab") and self.depth_tab is not None and self.image_tabs.currentWidget() is self.depth_tab

    def is_detection_tab_active(self) -> bool:
        return hasattr(self, "detection_tab") and self.detection_tab is not None and self.image_tabs.currentWidget() is self.detection_tab

    def is_pose_tab_active(self) -> bool:
        return hasattr(self, "pose_tab") and self.pose_tab is not None and self.image_tabs.currentWidget() is self.pose_tab

    def _refresh_rgb_views(self) -> None:
        active_widget = self.image_tabs.currentWidget() if hasattr(self, "image_tabs") else None
        placeholder = "Waiting for image"
        if active_widget is self.rgb_tab:
            self.rgb_view.set_qimage_frame(self._latest_rgb_qimage, placeholder)
        if active_widget is self.detection_tab:
            self.detection_view.set_qimage_frame(self._latest_rgb_qimage, placeholder)
        if active_widget is self.pose_tab:
            self.pose_view.set_qimage_frame(self._latest_rgb_qimage, placeholder)

    def _handle_tab_changed(self, index: int) -> None:
        if not hasattr(self, "pointcloud_tab") or self.pointcloud_tab is None:
            return
        self._refresh_rgb_views()
        if self.image_tabs.widget(index) is self.pointcloud_tab:
            self.display_pointcloud()


if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication

    class TestCameraConfig:
        detection_model = "YOLOv5"
        confidence_threshold = 0.5
        nms_threshold = 0.45

    app = QApplication(sys.argv)
    viewer = VisionViewer(TestCameraConfig())
    viewer.resize(1200, 800)
    viewer.show()
    viewer.update_image(np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8), "rgb")
    sys.exit(app.exec_())
