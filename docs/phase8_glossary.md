# Phase8 术语与缩写表

- [返回仓库 README](../README.md)
- [返回文档索引](README.md)
- [返回 Phase8 总览](phase8_function_manual.md)

这份文档只做一件事：把第一次读 Phase8 时最容易卡住的缩写、术语和口语化说法拆开讲清楚。

如果你在其他说明书里看到不熟悉的英文缩写，先回到这里查一遍，再继续看模块手册会轻松很多。

## 最常见的缩写

| 缩写 / 术语 | 全称 / 中文 | 在本项目里具体指什么 |
| --- | --- | --- |
| ROI | Region of Interest，感兴趣区域 | 视觉模块当前锁定的目标区域。后面的反投影、点云处理和抓取规划都围绕这个区域展开。 |
| VLM | Vision-Language Model，视觉语言模型 | 既能看图又能理解文本的模型。本项目里主要用于把自然语言任务整理成结构化语义。 |
| LLM | Large Language Model，大语言模型 | 只看文本或以文本推理为主的模型。在本项目里通常作为语义理解能力的上层概念。 |
| Qwen | 通义千问模型家族 | 当前 Web 任务理解链路里用到的对话/语义模型来源。 |
| bbox | bounding box，边界框 | 2D 检测结果里的矩形框，用来粗定位目标。 |
| mask | 分割掩码 | 比 bbox 更细的实例区域，用于更准确地截取目标像素。 |
| top-k | 前 K 个候选 | 检测或抓取模块保留的分数最高的一批候选。 |
| prompt | 提示词 | 发给检测器或语义模型的文本提示，比如 `blue cylinder`。 |
| semantic task | 结构化语义任务 | 把“抓取蓝色圆柱体”整理成目标类别、颜色、约束、排除项等结构化结果。 |

## 视觉与点云相关术语

| 缩写 / 术语 | 全称 / 中文 | 在本项目里具体指什么 |
| --- | --- | --- |
| open-vocabulary detection | 开放词汇检测 | 不提前把类别写死，而是根据语义提示动态找目标。 |
| OWLv2 | Open-World Localization v2 | 一类开放词汇检测模型。项目里有相关兼容路径。 |
| SAM | Segment Anything Model | 一类通用分割模型。项目里可用于把检测框细化成实例 mask。 |
| MobileCLIP | 轻量 CLIP 类视觉语义模型 | 用于相似度匹配、候选重排或语义对齐的轻量化视觉文本模型。 |
| 2D->3D 反投影 | re-projection / back-projection | 把图像里的 bbox 或 mask，结合深度图和相机内参，变成三维点云。 |
| intrinsics | 相机内参 | 用于从像素坐标恢复相机坐标系下三维点的位置。 |
| TF | transform tree，坐标变换树 | ROS 里管理各坐标系关系的机制，用来把点云从相机系变到 `world`。 |
| point cloud | 点云 | 目标表面或场景表面的一组三维点。 |
| voxel downsample | 体素降采样 | 用更稀疏但更稳定的点云表示替代原始稠密点云。 |
| plane removal | 平面剔除 | 把桌面这类大平面从目标点云里去掉。 |
| cluster selection | 聚类筛选 | 当点云里有多个连通块时，挑出最像目标的那一块。 |
| primitive fitting | 基元拟合 | 把不完整点云拟合成圆柱、长方体等更稳定的几何体。 |
| completion | 点云补全 | 补足相机看不到或缺失的那部分形状，让抓取策略更稳定。 |
| ICP | Iterative Closest Point，迭代最近点 | 一类点云配准方法，用于把模板或补全结果和当前点云对齐。 |

## 抓取与执行相关术语

| 缩写 / 术语 | 全称 / 中文 | 在本项目里具体指什么 |
| --- | --- | --- |
| grasp candidate | 抓取候选 | 一组可能可行的抓取姿态，不是最终动作，而是给规划器筛选的候选集。 |
| grasp proposal | 抓取提案 | 在消息层里承载抓取候选的具体数据结构，含抓取中心、接触点、姿态和分数。 |
| GraspGen | 外部抓取候选生成后端 | 当前 Phase8 默认主要使用的外部候选生成服务。 |
| GG-CNN | Generative Grasping CNN | 一类从视觉输入直接预测抓取参数的抓取网络，项目里保留兼容后端。 |
| Contact-GraspNet | 点云抓取候选网络 | 另一个点云抓取候选生成后端，项目里也保留兼容路径。 |
| approach direction | 接近方向 | 夹爪在抓取前沿哪条方向接近目标。 |
| closing direction | 闭合方向 | 夹爪两指闭合时的方向。 |
| pregrasp | 预抓取位姿 | 真正闭合抓取前的过渡位姿。系统通常先规划到这里，再进入 grasp。 |
| grasp | 抓取位姿 | 夹爪最终执行闭合动作的位姿。 |
| screening | 候选筛选 | 在真正执行前，先检查候选的 IK、碰撞和 pregrasp 可达性。 |
| IK | Inverse Kinematics，逆运动学 | 检查机械臂是否能以某个关节解到达目标位姿。 |
| MoveIt | ROS 机械臂规划框架 | 当前 pregrasp/grasp 规划、碰撞检查和执行的核心框架。 |
| return home | 返回 home 位 | 把机械臂回到默认起始关节位，通常用于复测前复位。 |
| reset scene | 重置场景 | 把仿真里的物体和环境恢复到初始状态。当前 Web 端通常会和 return home 配合使用。 |

## Web 端与交互相关术语

| 缩写 / 术语 | 全称 / 中文 | 在本项目里具体指什么 |
| --- | --- | --- |
| Control | 控制页 | 任务输入、执行、重规划、回 home、重置场景的主页面。 |
| Vision | 视觉页 | 看检测框、实例候选、目标锁定结果的页面。 |
| Tactile | 触觉页 | 看触觉阵列、力分量和更新状态的页面。 |
| Logs | 日志页 | 看任务 stepper、后端事件和错误提示的页面。 |
| Review mode | 审阅模式 | 先让用户确认结构化任务或候选，再继续执行。 |
| Auto mode | 自动模式 | 尽量减少中间确认，直接顺着主链路执行。 |
| Open Debug | 调试视图 | 展示更细的内部状态和中间结果，用于调试。 |

## 读说明书时怎么理解这些词

- 看到 `ROI`，就把它理解成“当前系统准备围绕它继续往后算的目标区域”。
- 看到 `pregrasp`，就把它理解成“抓之前的站位”，不是最终夹取动作。
- 看到 `grasp candidate` 或 `proposal`，就把它理解成“待筛选的抓法”，不是已经能执行的最终解。
- 看到 `reset scene + return home`，就把它理解成“一次完整复位”，前者恢复环境，后者恢复机械臂。

## 下一步该看什么

- 如果你想先学会操作 Web：看 [Web 端说明书](phase8_web_manual.md)
- 如果你想搞懂 ROI 是怎么锁住目标的：看 [语义与视觉说明书](phase8_semantic_vision_manual.md)
- 如果你想搞懂点云、反投影和补全：看 [点云与几何说明书](phase8_pointcloud_manual.md)
- 如果你想搞懂 pregrasp / grasp 为什么会失败：看 [抓取与执行说明书](phase8_grasp_execution_manual.md)
