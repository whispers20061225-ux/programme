# Phase8 语义与视觉说明书

- [返回 Phase8 总览](phase8_function_manual.md)
- [返回文档索引](README.md)
- [术语与缩写表](phase8_glossary.md)

## 这份文档讲什么

这份说明书解释“系统怎么理解你说的话”和“系统怎么从画面里锁定你要抓的那个目标”。

关键入口文件：

- [qwen_semantic_node.py](../ros2_ws/src/tactile_vision/tactile_vision/qwen_semantic_node.py)
- [detector_seg_node.py](../ros2_ws/src/tactile_vision/tactile_vision/detector_seg_node.py)
- [SemanticTask.msg](../ros2_ws/src/tactile_interfaces/msg/SemanticTask.msg)
- [DetectionResult.msg](../ros2_ws/src/tactile_interfaces/msg/DetectionResult.msg)

## 第一层：任务语义理解

核心节点：

- [qwen_semantic_node.py](../ros2_ws/src/tactile_vision/tactile_vision/qwen_semantic_node.py)

它解决的问题是：

- 用户到底要做什么任务
- 目标物是什么
- 有没有颜色、左右、目标实例、夹爪或约束要求

它不会直接去抓，而是把自然语言整理成结构化语义。

### 输入

- `/qwen/user_prompt`
- 可选的当前 RGB 图像

### 输出

- `/qwen/semantic_task`
- `/qwen/semantic_result`

### 结果里通常会包含什么

- `task`
- `target_label`
- `target_hint`
- `constraints`
- `excluded_labels`
- `confidence`
- `need_human_confirm`

也就是说，这一层的核心作用是给后面的视觉和执行链一个“明确任务上下文”。

## 第二层：开放词汇检测与实例筛选

核心节点：

- [detector_seg_node.py](../ros2_ws/src/tactile_vision/tactile_vision/detector_seg_node.py)

当前 phase8 默认不是固定类别检测，而是语义驱动的开放词汇检测。

例如你说：

- `抓取蓝色圆柱体`

系统不会只把它当作一句自然语言，而会提炼成更适合检测器使用的提示词，例如：

- `blue cylinder`
- `cylinder`
- `container`

## 当前主检测后端

从当前参数看，默认主路径是：

- backend: `yoloe_local`

相关配置：

- [phase8_modular_grasp.yaml](../ros2_ws/src/tactile_bringup/config/phase8_modular_grasp.yaml)

这意味着：

- 检测并不依赖固定类别词表
- 类别提示可以由语义模块动态驱动

## ROI 锁定为什么重要

视觉侧不只是“每帧跑一次全图检测”。

当前实现还做了：

- 基于历史 bbox 的 ROI 推理
- 候选跟踪
- 语义匹配加权
- 置信度与稳定度联合排序

所以视觉侧真正要解决的是：

- 看见目标
- 选中正确实例
- 连续多帧稳定锁定

不是只有“框出来”这么简单。

## 候选是怎么被选中的

当前排序通常会综合这些因素：

- detector 本身置信度
- 语义匹配程度
- track 稳定度
- 可选的 relabel/VLM 修正

因此一个框即使存在，也不一定会被作为最终目标。

## 视觉侧的主要输出

最终最重要的消息是：

- [DetectionResult.msg](../ros2_ws/src/tactile_interfaces/msg/DetectionResult.msg)

里面会包含：

- `accepted`
- `candidate_visible`
- `candidate_complete`
- `target_label`
- `confidence`
- `bbox`
- `point_px`
- `mask`

这一点很关键：  
视觉侧不是只输出一个 2D 框，而是会尽可能把实例 mask 也一起给后续模块。

## 这层和 Web 页的关系

你在 Web `Vision` 页看到的核心状态，大部分都来自这里：

- 当前 detection 是否被接受
- 当前实例 bbox
- 当前候选列表
- 当前选择的目标实例

如果你看到“目标明明框住了，但系统还是没继续”，通常还要看：

- 锁定是否稳定
- 语义是否一致
- 下游点云和执行链是否接受了这个目标

## 调试建议

先看这些内容：

- [VisionPage.tsx](../ros2_ws/src/tactile_web_bridge/frontend/src/VisionPage.tsx)
- `/perception/detection_result`
- `detector_seg_node` 日志

最常见的几类问题：

- 目标框不准：先看 prompt classes 和 detector 候选
- 锁到错误实例：先看 ROI 跟踪和候选排序
- 语义没对上：先看 `qwen_semantic_node` 的 `target_hint`
