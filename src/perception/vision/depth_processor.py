"""
深度处理模块 - 深度图像处理和分析
"""

import numpy as np
import cv2
from typing import Dict, Any, Optional, Tuple, List
import open3d as o3d
from scipy import ndimage
import matplotlib.pyplot as plt

from utils.logging_config import get_logger

logger = get_logger(__name__)

class DepthProcessor:
    """
    深度处理器
    提供深度图像处理、点云生成和分析功能
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化深度处理器
        
        Args:
            config: 深度处理配置
        """
        self.config = config
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 深度处理参数
        self.max_depth = config.get('max_depth', 3.0)  # 米
        self.min_depth = config.get('min_depth', 0.1)  # 米
        
        # 滤波参数
        self.bilateral_d = config.get('bilateral_d', 5)
        self.bilateral_sigma_color = config.get('bilateral_sigma_color', 50)
        self.bilateral_sigma_space = config.get('bilateral_sigma_space', 50)
        
        # 点云参数
        self.voxel_size = config.get('voxel_size', 0.01)  # 米
        
        logger.info("DepthProcessor initialized")
    
    def set_camera_params(self, camera_matrix: np.ndarray, 
                         dist_coeffs: Optional[np.ndarray] = None):
        """
        设置相机参数
        
        Args:
            camera_matrix: 相机内参矩阵 (3x3)
            dist_coeffs: 畸变系数
        """
        new_camera_matrix = camera_matrix.astype(np.float32)
        if dist_coeffs is not None:
            new_dist_coeffs = dist_coeffs.astype(np.float32)
        else:
            new_dist_coeffs = np.zeros((4,), dtype=np.float32)

        if self.camera_matrix is not None and self.dist_coeffs is not None:
            try:
                if np.allclose(self.camera_matrix, new_camera_matrix) and np.allclose(self.dist_coeffs, new_dist_coeffs):
                    return
            except Exception:
                pass

        self.camera_matrix = new_camera_matrix
        self.dist_coeffs = new_dist_coeffs
        logger.info(f"Camera parameters set: fx={new_camera_matrix[0,0]}, fy={new_camera_matrix[1,1]}")
    
    def preprocess_depth(self, depth_image: np.ndarray) -> np.ndarray:
        """
        预处理深度图像
        
        Args:
            depth_image: 原始深度图像
            
        Returns:
            处理后的深度图像
        """
        if depth_image is None:
            return None
        
        # 转换为浮点数（如果是16位）
        if depth_image.dtype == np.uint16:
            depth_float = depth_image.astype(np.float32) / 1000.0  # 毫米转米
        elif depth_image.dtype == np.float32:
            depth_float = depth_image.copy()
        else:
            raise ValueError(f"Unsupported depth image dtype: {depth_image.dtype}")
        
        # 应用深度限制
        depth_float = np.clip(depth_float, self.min_depth, self.max_depth)
        
        # 无效深度值设为0
        depth_float[depth_float < self.min_depth] = 0
        depth_float[depth_float > self.max_depth] = 0
        
        return depth_float
    
    def filter_depth(self, depth_image: np.ndarray, 
                    method: str = 'bilateral') -> np.ndarray:
        """
        滤波深度图像
        
        Args:
            depth_image: 深度图像
            method: 滤波方法 ('bilateral', 'median', 'gaussian', 'nlm')
            
        Returns:
            滤波后的深度图像
        """
        if depth_image is None:
            return None
        
        method = method.lower()
        if method == 'bilateral':
            filtered = cv2.bilateralFilter(
                depth_image,
                d=self.bilateral_d,
                sigmaColor=self.bilateral_sigma_color,
                sigmaSpace=self.bilateral_sigma_space
            )
        elif method == 'median':
            filtered = cv2.medianBlur(depth_image, 5)
        elif method == 'gaussian':
            filtered = cv2.GaussianBlur(depth_image, (5, 5), 0)
        elif method == 'nlm':
            filtered = cv2.fastNlMeansDenoising(
                depth_image, None, h=10, templateWindowSize=7, searchWindowSize=21
            )
        elif method == 'temporal':
            # 简易时间滤波（滑动平均）
            filtered = self._temporal_smooth(depth_image)
        else:
            raise ValueError(f"Unknown filter method: {method}")
        
        return filtered
    
    def fill_depth_holes(self, depth_image: np.ndarray,
                        max_hole_size: int = 10) -> np.ndarray:
        """
        填充深度图像中的空洞
        
        Args:
            depth_image: 深度图像
            max_hole_size: 最大空洞大小（像素）
            
        Returns:
            填充后的深度图像
        """
        if depth_image is None:
            return None
        
        # 创建掩码：有效深度区域为1，空洞为0
        mask = (depth_image > self.min_depth).astype(np.uint8)
        
        # 形态学闭操作填充小空洞
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # 找到空洞
        holes = mask_closed - mask
        
        # 标记连通区域
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(holes, connectivity=8)
        
        # 填充小空洞
        filled_depth = depth_image.copy()
        
        for label in range(1, num_labels):  # 跳过背景标签0
            hole_size = stats[label, cv2.CC_STAT_AREA]
            
            if hole_size <= max_hole_size:
                # 小空洞：使用周围像素的平均值填充
                hole_mask = (labels == label).astype(bool)
                
                # 膨胀空洞掩码以获取周围像素
                dilated_mask = cv2.dilate(hole_mask.astype(np.uint8), kernel, iterations=1)
                border_mask = dilated_mask - hole_mask.astype(np.uint8)
                
                # 计算周围像素的平均深度
                border_depths = depth_image[border_mask > 0]
                if len(border_depths) > 0:
                    avg_depth = np.mean(border_depths)
                    filled_depth[hole_mask] = avg_depth
        
        return filled_depth

    def estimate_object_center_camera(
        self,
        depth_image: np.ndarray,
        bbox: List[float],
        intrinsics: Optional[Dict[str, Any]] = None,
        shrink_ratio: float = 0.2,
        min_valid_pixels: int = 30,
        depth_percentile: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        基于检测框与深度图估计物体中心的相机坐标（单位：米）。

        Args:
            depth_image: 深度图（单位：米，float32）。
            bbox: 目标框 [x1, y1, x2, y2]。
            intrinsics: 相机内参（fx/fy/cx/cy/width/height）。
            shrink_ratio: 对ROI进行收缩，降低边缘干扰（0~0.5）。
            min_valid_pixels: ROI内有效深度像素最小数量阈值。

        Returns:
            dict: 包含 position(x,y,z)、pixel(cx,cy)、depth、valid_ratio 等信息。
        """
        if depth_image is None or bbox is None or len(bbox) != 4:
            return None

        h, w = depth_image.shape[:2]
        x1, y1, x2, y2 = [int(v) for v in bbox]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        if x2 <= x1 or y2 <= y1:
            return None

        # ROI收缩，避免边缘深度抖动
        shrink_ratio = max(0.0, min(0.5, float(shrink_ratio)))
        dx = int((x2 - x1) * shrink_ratio * 0.5)
        dy = int((y2 - y1) * shrink_ratio * 0.5)
        x1s = min(x2 - 1, x1 + dx)
        x2s = max(x1s + 1, x2 - dx)
        y1s = min(y2 - 1, y1 + dy)
        y2s = max(y1s + 1, y2 - dy)

        roi = depth_image[y1s:y2s, x1s:x2s].astype(np.float32)
        if roi.size == 0:
            return None

        # 过滤无效深度值
        valid_mask = (roi > self.min_depth) & (roi < self.max_depth)
        valid_depths = roi[valid_mask]
        if valid_depths.size < int(min_valid_pixels):
            return None

        if depth_percentile is None:
            depth_m = float(np.median(valid_depths))
        else:
            depth_percentile = max(0.0, min(100.0, float(depth_percentile)))
            depth_m = float(np.percentile(valid_depths, depth_percentile))
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # 优先使用传入内参，其次使用已设置的 camera_matrix
        fx = fy = cx_i = cy_i = None
        if intrinsics:
            fx = intrinsics.get("fx")
            fy = intrinsics.get("fy")
            cx_i = intrinsics.get("cx")
            cy_i = intrinsics.get("cy")
        if (fx is None or fy is None or cx_i is None or cy_i is None) and self.camera_matrix is not None:
            fx = float(self.camera_matrix[0, 0])
            fy = float(self.camera_matrix[1, 1])
            cx_i = float(self.camera_matrix[0, 2])
            cy_i = float(self.camera_matrix[1, 2])

        if fx is None or fy is None or cx_i is None or cy_i is None:
            return None

        # 相机坐标系：X向右，Y向下，Z向前（与常见RGB-D坐标一致）
        x = (cx - cx_i) * depth_m / fx
        y = (cy - cy_i) * depth_m / fy
        z = depth_m

        return {
            "position": np.array([x, y, z], dtype=np.float32),
            "pixel": (cx, cy),
            "depth": depth_m,
            "valid_ratio": float(valid_depths.size) / float(roi.size),
        }

    def remove_outliers(self, depth_image: np.ndarray, z_thresh: float = 3.0) -> np.ndarray:
        """
        简单异常值剔除（基于 Z-score）
        """
        if depth_image is None:
            return None
        valid = depth_image[depth_image > 0]
        if valid.size == 0:
            return depth_image
        mean = valid.mean()
        std = valid.std() + 1e-6
        mask = np.abs(depth_image - mean) > z_thresh * std
        cleaned = depth_image.copy()
        cleaned[mask] = 0
        return cleaned

    def to_point_cloud(self, depth_image: np.ndarray, rgb_image: Optional[np.ndarray] = None) -> Optional[o3d.geometry.PointCloud]:
        """
        深度转点云（需要相机内参）
        """
        if depth_image is None or self.camera_matrix is None:
            return None
        h, w = depth_image.shape
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        xs, ys = np.meshgrid(np.arange(w), np.arange(h))
        z = depth_image
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        mask = (z > self.min_depth) & (z < self.max_depth)
        mask = mask.reshape(-1)
        points = points[mask]
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        if rgb_image is not None:
            colors = rgb_image.reshape(-1, 3)[mask] / 255.0
            pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    # ---------- 内部辅助 ----------
    def _temporal_smooth(self, depth_image: np.ndarray, window: int = 5) -> np.ndarray:
        """
        简易时间平滑：滑动窗口均值（需外部维护历史栈；此处占位返回原图）
        """
        # TODO: 可在调用侧维护一个队列传入，当前占位返回原图
        return depth_image
    
    def compute_normals(self, depth_image: np.ndarray) -> np.ndarray:
        """
        计算深度图像的法线
        
        Args:
            depth_image: 深度图像
            
        Returns:
            法线图像 (H, W, 3)
        """
        if depth_image is None:
            return None
        
        # 使用Sobel算子计算梯度
        dx = cv2.Sobel(depth_image, cv2.CV_32F, 1, 0, ksize=3)
        dy = cv2.Sobel(depth_image, cv2.CV_32F, 0, 1, ksize=3)
        
        # 计算法线
        # 对于每个像素，法线 n = (-dx, -dy, 1)
        normals = np.stack([-dx, -dy, np.ones_like(depth_image)], axis=-1)
        
        # 归一化
        norm = np.linalg.norm(normals, axis=-1, keepdims=True)
        normals = normals / np.clip(norm, 1e-7, None)
        
        return normals
    
    def depth_to_pointcloud(self, depth_image: np.ndarray,
                           color_image: Optional[np.ndarray] = None,
                           use_intrinsics: bool = True) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        深度图像转点云
        
        Args:
            depth_image: 深度图像
            color_image: 彩色图像（可选）
            use_intrinsics: 是否使用相机内参
            
        Returns:
            点云坐标 (N, 3) 和颜色 (N, 3)（如果有）
        """
        if depth_image is None:
            return None, None
        
        height, width = depth_image.shape
        
        if use_intrinsics and self.camera_matrix is not None:
            # 使用相机内参
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        else:
            # 默认内参（假设图像中心）
            fx = fy = width
            cx = width / 2
            cy = height / 2
        
        # 创建像素坐标网格
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)
        
        # 转换为3D坐标
        z = depth_image.astype(np.float32)
        
        # 有效深度掩码
        valid_mask = (z > self.min_depth) & (z < self.max_depth)
        
        if not np.any(valid_mask):
            logger.warning("No valid depth points")
            return np.zeros((0, 3)), None
        
        # 应用掩码
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]
        z_valid = z[valid_mask]
        
        x = (u_valid - cx) * z_valid / fx
        y = (v_valid - cy) * z_valid / fy
        
        # 组合点云
        points = np.stack([x, y, z_valid], axis=1)
        
        # 处理颜色
        colors = None
        if color_image is not None and color_image.shape[:2] == (height, width):
            if len(color_image.shape) == 2:
                # 灰度图
                color_valid = color_image[valid_mask]
                colors = np.stack([color_valid, color_valid, color_valid], axis=1) / 255.0
            elif len(color_image.shape) == 3:
                # 彩色图
                if color_image.shape[2] == 3:
                    colors = color_image[valid_mask] / 255.0
                elif color_image.shape[2] == 4:
                    colors = color_image[valid_mask, :3] / 255.0
        
        return points, colors
    
    def create_o3d_pointcloud(self, depth_image: np.ndarray,
                             color_image: Optional[np.ndarray] = None) -> Optional[o3d.geometry.PointCloud]:
        """
        创建Open3D点云对象
        
        Args:
            depth_image: 深度图像
            color_image: 彩色图像（可选）
            
        Returns:
            Open3D点云对象
        """
        try:
            points, colors = self.depth_to_pointcloud(depth_image, color_image)
            
            if points is None or len(points) == 0:
                return None
            
            # 创建Open3D点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            if colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            return pcd
            
        except Exception as e:
            logger.error(f"Failed to create Open3D point cloud: {str(e)}")
            return None
    
    def voxel_downsample(self, pcd: o3d.geometry.PointCloud,
                        voxel_size: float = None) -> o3d.geometry.PointCloud:
        """
        体素下采样点云
        
        Args:
            pcd: Open3D点云
            voxel_size: 体素大小
            
        Returns:
            下采样后的点云
        """
        if voxel_size is None:
            voxel_size = self.voxel_size
        
        return pcd.voxel_down_sample(voxel_size)
    
    def remove_outliers(self, pcd: o3d.geometry.PointCloud,
                       nb_neighbors: int = 20,
                       std_ratio: float = 2.0) -> o3d.geometry.PointCloud:
        """
        移除点云离群点
        
        Args:
            pcd: Open3D点云
            nb_neighbors: 邻居数量
            std_ratio: 标准差比率
            
        Returns:
            去噪后的点云
        """
        cl, ind = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        
        return cl
    
    def compute_pointcloud_features(self, pcd: o3d.geometry.PointCloud,
                                   radius: float = 0.05,
                                   max_nn: int = 100) -> np.ndarray:
        """
        计算点云特征
        
        Args:
            pcd: Open3D点云
            radius: 特征计算半径
            max_nn: 最大邻居数
            
        Returns:
            点特征（法线、曲率等）
        """
        # 计算法线
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius, max_nn=max_nn
            )
        )
        
        # 计算FPFH特征
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=100)
        )
        
        return pcd_fpfh.data.T
    
    def register_pointclouds(self, source_pcd: o3d.geometry.PointCloud,
                            target_pcd: o3d.geometry.PointCloud,
                            method: str = 'fgr') -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
        """
        配准点云
        
        Args:
            source_pcd: 源点云
            target_pcd: 目标点云
            method: 配准方法 ('fgr', 'icp', 'ransac')
            
        Returns:
            配准后的点云和变换矩阵
        """
        if method == 'fgr':
            # 快速全局配准
            result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
                source_pcd, target_pcd,
                o3d.pipelines.registration.FastGlobalRegistrationOption(
                    maximum_correspondence_distance=0.05
                )
            )
        elif method == 'icp':
            # ICP配准
            result = o3d.pipelines.registration.registration_icp(
                source_pcd, target_pcd,
                0.05,  # 最大对应距离
                np.identity(4),  # 初始变换
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
        elif method == 'ransac':
            # RANSAC配准
            result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source_pcd, target_pcd,
                o3d.pipelines.registration.FeatureMatchingOption(
                    mutual_filter=True,
                    max_correspondence_distance=0.05
                )
            )
        else:
            raise ValueError(f"Unknown registration method: {method}")
        
        # 应用变换
        transformed_pcd = source_pcd.transform(result.transformation)
        
        return transformed_pcd, result.transformation
    
    def compute_depth_histogram(self, depth_image: np.ndarray) -> Dict[str, Any]:
        """
        计算深度直方图
        
        Args:
            depth_image: 深度图像
            
        Returns:
            直方图统计
        """
        if depth_image is None:
            return {}
        
        # 有效深度值
        valid_depths = depth_image[(depth_image > self.min_depth) & 
                                  (depth_image < self.max_depth)]
        
        if len(valid_depths) == 0:
            return {}
        
        # 计算统计
        hist, bins = np.histogram(valid_depths, bins=50)
        
        return {
            'mean_depth': float(np.mean(valid_depths)),
            'median_depth': float(np.median(valid_depths)),
            'std_depth': float(np.std(valid_depths)),
            'min_depth': float(np.min(valid_depths)),
            'max_depth': float(np.max(valid_depths)),
            'histogram': hist.tolist(),
            'bins': bins.tolist(),
            'valid_pixel_count': len(valid_depths),
            'total_pixel_count': depth_image.size,
            'valid_ratio': len(valid_depths) / depth_image.size
        }
    
    def detect_depth_discontinuities(self, depth_image: np.ndarray,
                                    threshold: float = 0.1) -> np.ndarray:
        """
        检测深度不连续（边缘）
        
        Args:
            depth_image: 深度图像
            threshold: 边缘检测阈值
            
        Returns:
            边缘掩码
        """
        if depth_image is None:
            return None
        
        # 计算深度梯度
        grad_x = cv2.Sobel(depth_image, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_image, cv2.CV32F, 0, 1, ksize=3)
        
        # 计算梯度幅值
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        
        # 二值化边缘
        edges = (grad_mag > threshold).astype(np.uint8) * 255
        
        # 细化边缘
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, 
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        
        return edges
    
    def compute_surface_curvature(self, depth_image: np.ndarray,
                                 window_size: int = 3) -> np.ndarray:
        """
        计算表面曲率
        
        Args:
            depth_image: 深度图像
            window_size: 窗口大小
            
        Returns:
            曲率图像
        """
        if depth_image is None:
            return None
        
        # 使用局部平面拟合计算曲率
        height, width = depth_image.shape
        curvature = np.zeros_like(depth_image)
        
        half_window = window_size // 2
        
        for i in range(half_window, height - half_window):
            for j in range(half_window, width - half_window):
                # 提取局部窗口
                window = depth_image[i-half_window:i+half_window+1,
                                   j-half_window:j+half_window+1]
                
                # 忽略无效深度
                valid_window = window[window > 0]
                if len(valid_window) < window_size * window_size // 2:
                    continue
                
                # 计算局部平面参数
                # 简化：使用深度值的二阶差分作为曲率估计
                if window_size >= 3:
                    # 计算二阶导数
                    dxx = window[1, 0] - 2*window[1, 1] + window[1, 2]
                    dyy = window[0, 1] - 2*window[1, 1] + window[2, 1]
                    dxy = (window[0, 0] - window[0, 2] - window[2, 0] + window[2, 2]) / 4
                    
                    # 计算平均曲率
                    curvature[i, j] = np.abs(dxx + dyy)
        
        return curvature
    
    def visualize_depth(self, depth_image: np.ndarray,
                       colormap: str = 'jet') -> np.ndarray:
        """
        可视化深度图像
        
        Args:
            depth_image: 深度图像
            colormap: 颜色映射 ('jet', 'viridis', 'plasma', 'inferno')
            
        Returns:
            彩色深度图像
        """
        if depth_image is None:
            return None
        
        # 归一化深度到[0, 1]
        depth_normalized = np.clip((depth_image - self.min_depth) / 
                                  (self.max_depth - self.min_depth), 0, 1)
        
        # 应用颜色映射
        if colormap == 'jet':
            depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), 
                                            cv2.COLORMAP_JET)
        elif colormap == 'viridis':
            depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), 
                                            cv2.COLORMAP_VIRIDIS)
        elif colormap == 'plasma':
            depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), 
                                            cv2.COLORMAP_PLASMA)
        elif colormap == 'inferno':
            depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), 
                                            cv2.COLORMAP_INFERNO)
        else:
            # 默认使用jet
            depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), 
                                            cv2.COLORMAP_JET)
        
        return depth_colored
    
    def save_pointcloud(self, points: np.ndarray,
                       colors: Optional[np.ndarray] = None,
                       filepath: str = "pointcloud.ply"):
        """
        保存点云到文件
        
        Args:
            points: 点云坐标
            colors: 点云颜色
            filepath: 文件路径
        """
        if points is None or len(points) == 0:
            logger.warning("No points to save")
            return
        
        # 创建Open3D点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 保存
        o3d.io.write_point_cloud(filepath, pcd)
        logger.info(f"Point cloud saved to {filepath} with {len(points)} points")
