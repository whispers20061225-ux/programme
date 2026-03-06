"""
物体检测模块 - 基于深度学习的物体检测
支持YOLO, SSD, Faster R-CNN等模型
"""

import os
import torch
import torchvision
import cv2
import numpy as np
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from pathlib import Path
import time

from utils.logging_config import get_logger

logger = get_logger(__name__)

_RUNTIME_THREADS_CONFIGURED = False


def _configure_runtime_threads() -> None:
    global _RUNTIME_THREADS_CONFIGURED
    if _RUNTIME_THREADS_CONFIGURED:
        return
    try:
        cpu_count = os.cpu_count() or 2
        torch.set_num_threads(max(1, min(2, cpu_count // 2)))
    except Exception:
        pass
    try:
        if hasattr(torch, "set_num_interop_threads"):
            torch.set_num_interop_threads(1)
    except Exception:
        pass
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass
    _RUNTIME_THREADS_CONFIGURED = True

@dataclass
class DetectionResult:
    """检测结果数据结构"""
    bbox: List[float]  # [x1, y1, x2, y2]
    confidence: float
    class_id: int
    class_name: str
    mask: Optional[np.ndarray] = None  # 实例分割掩码

@dataclass
class DetectionFrame:
    """检测帧数据结构"""
    timestamp: float
    detections: List[DetectionResult]
    original_image: np.ndarray
    processed_image: np.ndarray

class ObjectDetector:
    """
    物体检测器
    支持多种检测模型
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化物体检测器
        
        Args:
            config: 检测器配置
        """
        _configure_runtime_threads()
        self.config = config
        self.model_name = config.get('model_name', 'yolov5')
        self.model_path = config.get('model_path')
        self.confidence_threshold = config.get('confidence_threshold', 0.5)
        self.iou_threshold = config.get('iou_threshold', 0.45)
        self.device = config.get('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        
        # 类别信息
        self.class_names = config.get('class_names', [])
        self.num_classes = len(self.class_names)
        
        # 模型
        self.model = None
        self.transform = None
        self.model_ready = False
        self.last_error = None
        
        # 性能统计
        self.inference_times = []
        self.frame_count = 0
        
        # 初始化模型
        self._initialize_model()
        
        logger.info(f"ObjectDetector initialized with model: {self.model_name}")
    
    def _initialize_model(self):
        """初始化检测模型"""
        try:
            if self.model_name.startswith('yolo'):
                self._initialize_yolo()
            elif self.model_name.startswith('ssd'):
                self._initialize_ssd()
            elif self.model_name.startswith('faster'):
                self._initialize_faster_rcnn()
            elif self.model_name.startswith('mask'):
                self._initialize_mask_rcnn()
            elif self.model_name.startswith('detr'):
                self._initialize_detr()
            else:
                raise ValueError(f"Unknown model type: {self.model_name}")
            self.model_ready = True
            self.last_error = None
        except Exception as e:
            logger.error(f"Failed to initialize model '{self.model_name}': {str(e)}; fallback to stub detector")
            self.model = None
            self.transform = None
            self.model_ready = False
            self.last_error = str(e)
    
    def _initialize_yolo(self):
        """初始化YOLO模型"""
        model_name = str(self.model_name).lower()
        if model_name == 'yolov5':
            # 使用ultralytics YOLOv5
            try:
                from ultralytics import YOLO
                
                if self.model_path:
                    self.model = YOLO(self.model_path)
                else:
                    # 加载预训练模型
                    self.model = YOLO('yolov5s.pt')
                
                logger.info(f"YOLOv5 model loaded: {self.model_path or 'pretrained'}")
                
            except ImportError:
                logger.warning("ultralytics not available, falling back to torchvision")
                # 回退到torchvision的实现
                self._initialize_torchvision_yolo()
        
        elif model_name == 'yolov8':
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.model_path or 'yolov8n.pt')
                logger.info("YOLOv8 model loaded")
                
            except ImportError:
                raise ImportError("Please install ultralytics: pip install ultralytics")
        else:
            raise ValueError(f"Unsupported YOLO variant: {self.model_name}")
    
    def _initialize_torchvision_yolo(self):
        """初始化torchvision版本的YOLO"""
        # 使用torchvision的YOLO实现
        # 注意：部分 torchvision 版本可能没有 YOLOv5；这里尝试加载，失败则抛出异常交由上层回退
        weights = torchvision.models.detection.YOLOv5Weights.DEFAULT
        self.model = torchvision.models.detection.yolov5s(weights=weights)
        self.model.eval()
        self.model.to(self.device)
        
        self.transform = weights.transforms()
        
        # 获取类别名称
        if not self.class_names:
            self.class_names = weights.meta.get("categories", [])
            self.num_classes = len(self.class_names)
        
        logger.info(f"Torchvision YOLO model loaded with {self.num_classes} classes")
    
    def _initialize_ssd(self):
        """初始化SSD模型"""
        if self.model_name == 'ssd':
            weights = torchvision.models.detection.SSD300_VGG16_Weights.DEFAULT
            self.model = torchvision.models.detection.ssd300_vgg16(weights=weights)
        elif self.model_name == 'ssdlite':
            weights = torchvision.models.detection.SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
            self.model = torchvision.models.detection.ssdlite320_mobilenet_v3_large(weights=weights)
        
        self.model.eval()
        self.model.to(self.device)
        
        self.transform = weights.transforms()
        
        if not self.class_names:
            self.class_names = weights.meta["categories"]
            self.num_classes = len(self.class_names)
        
        logger.info(f"{self.model_name.upper()} model loaded with {self.num_classes} classes")
    
    def _initialize_faster_rcnn(self):
        """初始化Faster R-CNN模型"""
        weights = torchvision.models.detection.FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights=weights)
        
        self.model.eval()
        self.model.to(self.device)
        
        self.transform = weights.transforms()
        
        if not self.class_names:
            self.class_names = weights.meta["categories"]
            self.num_classes = len(self.class_names)
        
        logger.info(f"Faster R-CNN model loaded with {self.num_classes} classes")
    
    def _initialize_mask_rcnn(self):
        """初始化Mask R-CNN模型"""
        weights = torchvision.models.detection.MaskRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = torchvision.models.detection.maskrcnn_resnet50_fpn(weights=weights)
        
        self.model.eval()
        self.model.to(self.device)
        
        self.transform = weights.transforms()
        
        if not self.class_names:
            self.class_names = weights.meta["categories"]
            self.num_classes = len(self.class_names)
        
        logger.info(f"Mask R-CNN model loaded with {self.num_classes} classes")
    
    def _initialize_detr(self):
        """初始化DETR模型"""
        weights = torchvision.models.detection.Detr_ResNet50_Weights.DEFAULT
        self.model = torchvision.models.detection.detr_resnet50(weights=weights)
        
        self.model.eval()
        self.model.to(self.device)
        
        self.transform = weights.transforms()
        
        if not self.class_names:
            self.class_names = weights.meta["categories"]
            self.num_classes = len(self.class_names)
        
        logger.info(f"DETR model loaded with {self.num_classes} classes")
    
    def detect(self, image: np.ndarray) -> DetectionFrame:
        """
        检测图像中的物体
        
        Args:
            image: 输入图像 (RGB格式)
            
        Returns:
            检测结果帧
        """
        start_time = time.time()
        timestamp = time.time()
        
        # 预处理图像
        if self.model is None or not self.model_ready:
            detections = self._detect_stub(image)
        elif self.model_name.startswith('yolo') and hasattr(self.model, 'predict'):
            # 使用ultralytics YOLO的预测接口
            detections = self._detect_yolo_ultralytics(image)
        else:
            # 使用torchvision模型
            detections = self._detect_torchvision(image)
        
        inference_time = time.time() - start_time
        self.inference_times.append(inference_time)
        self.frame_count += 1
        
        # 创建检测帧
        detection_frame = DetectionFrame(
            timestamp=timestamp,
            detections=detections,
            original_image=image,
            processed_image=image
        )
        
        # 记录性能
        if self.frame_count % 100 == 0:
            avg_time = np.mean(self.inference_times[-100:])
            logger.debug(f"Average inference time (last 100 frames): {avg_time*1000:.1f}ms")
        
        return detection_frame

    def _detect_stub(self, image: np.ndarray) -> List[DetectionResult]:
        """兜底检测：无模型时返回空结果，避免崩溃"""
        if self.frame_count == 0:
            logger.warning("ObjectDetector running in stub mode (no model loaded)")
        return []
    
    def _detect_yolo_ultralytics(self, image: np.ndarray) -> List[DetectionResult]:
        """使用ultralytics YOLO进行检测"""
        # 运行推理
        results = self.model(image, conf=self.confidence_threshold, 
                           iou=self.iou_threshold, verbose=False)
        
        detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # 获取边界框
                    bbox = box.xyxy[0].cpu().numpy().tolist()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    
                    # 获取类别名称
                    class_name = self._resolve_class_name(class_id, result.names)
                    
                    # 创建检测结果
                    detection = DetectionResult(
                        bbox=bbox,
                        confidence=confidence,
                        class_id=class_id,
                        class_name=class_name
                    )
                    
                    detections.append(detection)
        
        return detections

    def _resolve_class_name(self, class_id: int, default_names) -> str:
        """安全获取类别名称，避免索引越界"""
        try:
            if self.class_names and 0 <= class_id < len(self.class_names):
                return self.class_names[class_id]
        except Exception:
            pass

        # default_names 可能是 dict 或 list
        try:
            if isinstance(default_names, dict):
                return default_names.get(class_id, f"class_{class_id}")
            if isinstance(default_names, (list, tuple)) and 0 <= class_id < len(default_names):
                return default_names[class_id]
        except Exception:
            pass

        return f"class_{class_id}"
    
    def _detect_torchvision(self, image: np.ndarray) -> List[DetectionResult]:
        """使用torchvision模型进行检测"""
        # 预处理
        if self.transform:
            input_tensor = self.transform(image)
        else:
            # 基本预处理
            input_tensor = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
        
        input_batch = input_tensor.unsqueeze(0).to(self.device)
        
        # 推理
        with torch.no_grad():
            predictions = self.model(input_batch)
        
        detections = []
        
        # 处理预测结果
        for pred in predictions:
            boxes = pred['boxes'].cpu().numpy()
            scores = pred['scores'].cpu().numpy()
            labels = pred['labels'].cpu().numpy()
            
            # 获取掩码（如果可用）
            masks = None
            if 'masks' in pred:
                masks = pred['masks'].cpu().numpy()
            
            for i, (box, score, label) in enumerate(zip(boxes, scores, labels)):
                if score < self.confidence_threshold:
                    continue
                
                # 获取掩码
                mask = None
                if masks is not None:
                    mask = masks[i, 0]  # 取第一个通道
                    mask = (mask > 0.5).astype(np.uint8) * 255
                
                # 获取类别名称
                class_id = int(label) - 1  # torchvision的标签从1开始
                if 0 <= class_id < len(self.class_names):
                    class_name = self.class_names[class_id]
                else:
                    class_name = f"class_{class_id}"
                
                detection = DetectionResult(
                    bbox=box.tolist(),
                    confidence=float(score),
                    class_id=class_id,
                    class_name=class_name,
                    mask=mask
                )
                
                detections.append(detection)
        
        return detections
    
    def _draw_detections(self, image: np.ndarray, 
                        detections: List[DetectionResult]) -> np.ndarray:
        """在图像上绘制检测结果"""
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.shape[2] == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        height, width = image.shape[:2]
        
        # 颜色映射
        colors = [
            (0, 255, 0),    # 绿色
            (255, 0, 0),    # 蓝色
            (0, 0, 255),    # 红色
            (255, 255, 0),  # 青色
            (255, 0, 255),  # 紫色
            (0, 255, 255),  # 黄色
        ]
        
        for i, detection in enumerate(detections):
            # 获取颜色
            color = colors[i % len(colors)]
            
            # 绘制边界框
            x1, y1, x2, y2 = map(int, detection.bbox)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制标签
            label = f"{detection.class_name}: {detection.confidence:.2f}"
            
            # 计算标签背景大小
            (label_width, label_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            
            # 绘制标签背景
            cv2.rectangle(
                image,
                (x1, y1 - label_height - baseline),
                (x1 + label_width, y1),
                color,
                -1
            )
            
            # 绘制标签文本
            cv2.putText(
                image,
                label,
                (x1, y1 - baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )
            
            # 绘制掩码（如果存在）
            if detection.mask is not None:
                # 调整掩码大小以匹配边界框
                mask_resized = cv2.resize(
                    detection.mask,
                    (x2 - x1, y2 - y1),
                    interpolation=cv2.INTER_NEAREST
                )
                
                # 创建彩色掩码
                mask_color = np.zeros((y2 - y1, x2 - x1, 3), dtype=np.uint8)
                mask_color[:, :] = color
                
                # 应用透明度
                alpha = 0.3
                roi = image[y1:y2, x1:x2]
                
                # 创建布尔掩码
                bool_mask = mask_resized > 0
                
                # 混合颜色
                for c in range(3):
                    roi_channel = roi[:, :, c].astype(float)
                    mask_channel = mask_color[:, :, c].astype(float)
                    roi_channel[bool_mask] = (
                        alpha * mask_channel[bool_mask] + 
                        (1 - alpha) * roi_channel[bool_mask]
                    )
                    roi[:, :, c] = np.clip(roi_channel, 0, 255).astype(np.uint8)
        
        return image
    
    def filter_detections(self, detections: List[DetectionResult],
                         min_confidence: float = None,
                         target_classes: List[str] = None) -> List[DetectionResult]:
        """过滤检测结果"""
        if min_confidence is None:
            min_confidence = self.confidence_threshold
        
        filtered = []
        
        for detection in detections:
            # 置信度过滤
            if detection.confidence < min_confidence:
                continue
            
            # 类别过滤
            if target_classes and detection.class_name not in target_classes:
                continue
            
            filtered.append(detection)
        
        return filtered
    
    def get_detection_statistics(self, detections: List[DetectionResult]) -> Dict[str, Any]:
        """获取检测统计信息"""
        if not detections:
            return {
                'num_detections': 0,
                'avg_confidence': 0.0,
                'class_distribution': {}
            }
        
        confidences = [d.confidence for d in detections]
        class_names = [d.class_name for d in detections]
        
        # 计算类别分布
        class_distribution = {}
        for class_name in class_names:
            class_distribution[class_name] = class_distribution.get(class_name, 0) + 1
        
        return {
            'num_detections': len(detections),
            'avg_confidence': np.mean(confidences),
            'max_confidence': np.max(confidences),
            'min_confidence': np.min(confidences),
            'class_distribution': class_distribution
        }
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """获取性能统计"""
        if not self.inference_times:
            return {'avg_inference_time': 0.0, 'total_frames': 0}
        
        return {
            'avg_inference_time': np.mean(self.inference_times),
            'min_inference_time': np.min(self.inference_times),
            'max_inference_time': np.max(self.inference_times),
            'fps': 1.0 / np.mean(self.inference_times) if self.inference_times else 0,
            'total_frames': self.frame_count,
            'model_name': self.model_name,
            'device': self.device
        }
    
    def save_detections(self, image: np.ndarray, 
                       detections: List[DetectionResult],
                       filepath: str):
        """保存检测结果到文件"""
        import json
        
        # 创建可序列化的数据结构
        detection_data = []
        for detection in detections:
            detection_dict = {
                'bbox': detection.bbox,
                'confidence': detection.confidence,
                'class_id': detection.class_id,
                'class_name': detection.class_name
            }
            
            # 如果掩码存在，保存为图像
            if detection.mask is not None:
                mask_path = filepath.replace('.json', f'_mask_{len(detection_data)}.png')
                cv2.imwrite(mask_path, detection.mask)
                detection_dict['mask_path'] = mask_path
            
            detection_data.append(detection_dict)
        
        # 保存元数据
        metadata = {
            'timestamp': time.time(),
            'image_shape': image.shape,
            'num_detections': len(detections),
            'detections': detection_data
        }
        
        with open(filepath, 'w') as f:
            json.dump(metadata, f, indent=2, default=str)
        
        logger.info(f"Detections saved to {filepath}")
    
    def load_detections(self, filepath: str) -> Tuple[np.ndarray, List[DetectionResult]]:
        """从文件加载检测结果"""
        import json
        
        with open(filepath, 'r') as f:
            metadata = json.load(f)
        
        # 重新创建检测结果对象
        detections = []
        for det_dict in metadata['detections']:
            mask = None
            if 'mask_path' in det_dict:
                mask = cv2.imread(det_dict['mask_path'], cv2.IMREAD_GRAYSCALE)
            
            detection = DetectionResult(
                bbox=det_dict['bbox'],
                confidence=det_dict['confidence'],
                class_id=det_dict['class_id'],
                class_name=det_dict['class_name'],
                mask=mask
            )
            
            detections.append(detection)
        
        # 注意：原始图像不包含在保存的数据中
        image = None
        
        return image, detections
