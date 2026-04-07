# 文档索引

## 当前推荐先看

- [仓库 README](../README.md)
  - 当前 phase8 Web 端系统的环境准备、前置依赖、全链路启动与自检
- [术语与缩写表](phase8_glossary.md)
  - 先把 ROI、VLM、bbox、pregrasp、GraspGen 这些高频缩写讲清楚
- [Phase8 总览与手册导航](phase8_function_manual.md)
  - Phase8 手册首页，负责把总链路拆成多个模块说明书

## Phase8 模块说明书

- [Web 端说明书](phase8_web_manual.md)
  - Web 四个页面、交互方式、对话模式、常用按钮和 API
- [语义与视觉说明书](phase8_semantic_vision_manual.md)
  - 语义结构化、开放词汇检测、ROI 锁定、候选排序
- [点云与几何说明书](phase8_pointcloud_manual.md)
  - 2D 到 3D 反投影、点云滤波、几何拟合、补全与冻结
- [抓取与执行说明书](phase8_grasp_execution_manual.md)
  - 抓取候选、GraspGen、pregrasp/grasp 规划、执行编排
- [触觉与运行维护说明书](phase8_tactile_ops_manual.md)
  - 触觉接入、健康状态、return home、reset scene

## 当前主链路相关文档

- [Windows ROS2 RealSense Quickstart](windows_ros2_realsense_quickstart.md)
- [Windows VM One Click Runbook](windows_vm_one_click_runbook.md)
- [Windows VM Split Phase A](windows_vm_split_phaseA.md)
- [Windows VM Split Phase B](windows_vm_split_phaseB.md)

## 历史迁移文档

- [ROS2 Refactor Plan](ros2_refactor_plan.md)
- [Phase 1 Kickoff](phase1_kickoff.md)
- [Phase 2 Hardware Kickoff](phase2_hardware_kickoff.md)
- [Phase 3 Control Kickoff](phase3_control_kickoff.md)
- [Phase 4 UI Bridge Kickoff](phase4_ui_bridge_kickoff.md)
- [Phase 5 Task Kickoff](phase5_task_kickoff.md)
- [Phase 6 Vision Kickoff](phase6_vision_kickoff.md)
- [Phase 6 Sim Base Kickoff](phase6_sim_base_kickoff.md)
- [Phase 6 Sim Gazebo Kickoff](phase6_sim_gazebo_kickoff.md)

## 如何使用这些文档

- 如果你要把系统跑起来：先看 [仓库 README](../README.md)
- 如果你先被缩写和术语卡住：先看 [术语与缩写表](phase8_glossary.md)
- 如果你要理解当前主链路：先看 [Phase8 总览与手册导航](phase8_function_manual.md)
- 如果你要按模块理解：直接点上面的五份说明书
- 如果你要追历史演进：再看 phase1~phase6 的 kickoff 文档
