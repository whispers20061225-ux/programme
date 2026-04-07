# Phase8 点云与几何说明书

- [返回 Phase8 总览](phase8_function_manual.md)
- [返回文档索引](README.md)
- [术语与缩写表](phase8_glossary.md)

## 这份文档讲什么

这份说明书只讲一个问题：  
系统是怎么把 2D 目标区域，变成后续真正可抓的 3D 表示的。

关键入口文件：

- [cloud_filter_node.py](../ros2_ws/src/tactile_vision/tactile_vision/cloud_filter_node.py)
- [primitive_fit_node.py](../ros2_ws/src/tactile_vision/tactile_vision/primitive_fit_node.py)
- [grasp_input_cloud_node.py](../ros2_ws/src/tactile_vision/tactile_vision/grasp_input_cloud_node.py)
- [modular_common.py](../ros2_ws/src/tactile_vision/tactile_vision/modular_common.py)

## 总体流程

可以把这一段理解成五步：

1. 从检测结果拿到 bbox 或 mask
2. 用深度图和相机内参反投影成点云
3. 做滤波、平面剔除、聚类和稳定化
4. 对目标做几何拟合与补全
5. 产出一个更适合抓取策略使用的输入点云

## 第一层：2D 到 3D 反投影

核心节点：

- [cloud_filter_node.py](../ros2_ws/src/tactile_vision/tactile_vision/cloud_filter_node.py)

它会结合这些输入：

- `DetectionResult`
- 对齐深度图
- 相机内参
- TF 变换

把目标区域转换成世界坐标系中的点云。

### 它具体做什么

- 选择目标 bbox / mask
- 必要时复用历史 mask
- 从深度图中取目标区域
- 选 anchor pixel 和连通区域
- 反投影成相机坐标系点云
- 变换到 `world`
- 输出目标点云和目标位姿

### 关键输出

- `/perception/target_cloud`
- `/perception/target_cloud_markers`
- `/sim/perception/target_pose`
- `/sim/perception/target_locked`

## 第二层：点云滤波

`cloud_filter_node` 里当前主要做这些清洗：

- 深度范围裁剪
- voxel downsample
- statistical outlier filter
- dominant plane removal
- cluster selection

这一层不是为了让点云“看起来漂亮”，而是为了让它：

- 更稳定
- 更接近目标真实几何
- 更适合后续抓取候选生成

## 第三层：几何拟合

核心节点：

- [primitive_fit_node.py](../ros2_ws/src/tactile_vision/tactile_vision/primitive_fit_node.py)

它解决的问题是：  
真实场景里看到的点云往往是不完整的，直接拿原始点云去做抓取会很抖。

当前支持的几何策略包括：

- cylinder
- box
- sphere
- hemisphere
- frustum
- axis profile

### 为什么圆柱拟合特别重要

你当前常测的蓝色圆柱体，是这一层最典型的受益对象。

原因很简单：

- 圆柱常常只有一侧被相机看到
- 原始点云容易只剩一个弧面
- 拟合后可以更稳定地恢复轴向、半径和高度

这会直接影响抓取候选质量和 pregrasp 可执行性。

## 第四层：点云补全与冻结

核心节点：

- [grasp_input_cloud_node.py](../ros2_ws/src/tactile_vision/tactile_vision/grasp_input_cloud_node.py)

它做的不是简单转发，而是“给抓取后端做最终输入准备”。

### 主要作用

- 比较原始点云和拟合点云的一致性
- 评估当前几何策略置信度
- 从拟合点云里选择补全点
- 保持上一帧稳定输出
- 条件满足时冻结结果，避免执行过程中输入抖动

### 关键输出

- `/perception/target_cloud_for_graspgen`
- `/perception/target_cloud_for_graspgen_debug`

这就是当前系统里“点云特征匹配、补全、冻结”的主要实现位置。

## 和 2D 显示的关系

二维显示主要在 Web `Vision` 页。  
点云链本身不直接负责前端页面设计，但它决定了：

- 当前目标有没有稳定 3D 位姿
- 当前点云是否足以支持抓取候选生成

所以常见现象是：

- Vision 页看起来框对了
- 但后面抓不起来

这时问题往往已经不在 2D，而在 3D 点云质量。

## 调试建议

先看这些：

- `/perception/target_cloud`
- `/perception/target_cloud_debug`
- `/perception/target_cloud_fitted`
- `/perception/target_cloud_for_graspgen_debug`

最常见问题：

- 点太少：看深度区域和 mask 是否有效
- 目标点云被桌面吃掉：看平面剔除参数
- 拟合跳动：看历史稳定参数
- 执行时输入云抖：看 freeze / hold 逻辑
