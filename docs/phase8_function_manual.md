# Phase8 总览与手册导航

这份文档不再承担“把所有模块一次讲完”的任务。  
它现在是 Phase8 文档首页，负责把当前 Web 主链路拆成多个可单独阅读的模块说明书。

## 快速入口

- [返回仓库 README](../README.md)
- [返回文档索引](README.md)
- [术语与缩写表](phase8_glossary.md)

## 推荐阅读顺序

1. [术语与缩写表](phase8_glossary.md)
2. [系统总览](phase8_function_manual.md)
3. [Web 端说明书](phase8_web_manual.md)
4. [语义与视觉说明书](phase8_semantic_vision_manual.md)
5. [点云与几何说明书](phase8_pointcloud_manual.md)
6. [抓取与执行说明书](phase8_grasp_execution_manual.md)
7. [触觉与运行维护说明书](phase8_tactile_ops_manual.md)

## 一张图看主链路

```mermaid
flowchart LR
    A["Web UI"] --> B["语义理解"]
    B --> C["开放词汇检测与 ROI 锁定"]
    C --> D["深度反投影与点云滤波"]
    D --> E["几何拟合与点云补全"]
    E --> F["抓取候选生成"]
    F --> G["pregrasp / grasp 规划"]
    G --> H["执行、回家位、场景复位"]
    H --> A
```

## 模块导航

| 模块 | 说明书 | 你能在里面看到什么 |
| --- | --- | --- |
| Web 端 | [phase8_web_manual.md](phase8_web_manual.md) | 四个页面分别做什么、怎么对话、AI 用在哪里 |
| 语义与视觉 | [phase8_semantic_vision_manual.md](phase8_semantic_vision_manual.md) | `qwen_semantic_node`、`detector_seg_node`、ROI 锁定和开放词汇检测 |
| 点云与几何 | [phase8_pointcloud_manual.md](phase8_pointcloud_manual.md) | 2D 到 3D 反投影、点云滤波、几何拟合、补全、冻结 |
| 抓取与执行 | [phase8_grasp_execution_manual.md](phase8_grasp_execution_manual.md) | GraspGen、GG-CNN、执行编排、pregrasp/grasp 规划 |
| 触觉与运维 | [phase8_tactile_ops_manual.md](phase8_tactile_ops_manual.md) | 触觉接入、健康状态、return home、reset scene |

## 当前关键入口文件

- [phase8_web_ui.launch.py](../ros2_ws/src/tactile_bringup/launch/phase8_web_ui.launch.py)
- [phase8_modular_grasp.launch.py](../ros2_ws/src/tactile_bringup/launch/phase8_modular_grasp.launch.py)
- [phase8_modular_grasp.yaml](../ros2_ws/src/tactile_bringup/config/phase8_modular_grasp.yaml)
- [web_gateway.py](../ros2_ws/src/tactile_web_bridge/tactile_web_bridge/web_gateway.py)

## 如果你现在在排错

| 你看到的问题 | 先看哪份说明书 |
| --- | --- |
| Web 页面看不懂 | [Web 端说明书](phase8_web_manual.md) |
| 目标框不对、实例没锁住 | [语义与视觉说明书](phase8_semantic_vision_manual.md) |
| 点云不稳定、目标位姿飘 | [点云与几何说明书](phase8_pointcloud_manual.md) |
| 有候选但 pregrasp 失败 | [抓取与执行说明书](phase8_grasp_execution_manual.md) |
| 触觉页没数据、return home/reset scene 异常 | [触觉与运行维护说明书](phase8_tactile_ops_manual.md) |
