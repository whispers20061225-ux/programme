# 文档索引

## 当前推荐先看

- [仓库 README](../README.md)
  - 当前 Tactile Grasp Studio Web 主链的环境准备、前置依赖、全链路启动与自检
- [ROS 2 环境与依赖安装指南](ros2_environment_setup_guide.md)
  - 从零安装 ROS 2 Jazzy、初始化 rosdep、补齐当前项目的 apt 和 pip 依赖
- [ROS2 工作区 Quickstart](../ros2_ws/README.md)
  - 面向当前 `ros2_ws` 的最短命令路径：装依赖、构建、前端打包、启动主链
- [术语与缩写表](terminology_guide.md)
  - 先把 ROI、VLM、bbox、pregrasp、GraspGen 这些高频缩写讲清楚
- [系统总览说明书](system_overview_manual.md)
  - 当前主链的总览首页，负责把总链路拆成多个模块说明书

## 当前主链模块说明书

- [Web 控制台说明书](web_console_manual.md)
  - Web 四个页面、交互方式、对话模式、常用按钮和 API
- [语义感知说明书](semantic_perception_manual.md)
  - 语义结构化、开放词汇检测、ROI 锁定、候选排序
- [点云与几何说明书](pointcloud_geometry_manual.md)
  - 2D 到 3D 反投影、点云滤波、几何拟合、补全与冻结
- [抓取规划与执行说明书](grasp_execution_manual.md)
  - 抓取候选、GraspGen、pregrasp/grasp 规划、执行编排
- [触觉与运维说明书](tactile_operations_manual.md)
  - 触觉接入、健康状态、return home、reset scene

## 当前主链路相关文档

- [Windows ROS2 RealSense Quickstart](windows_ros2_realsense_quickstart.md)
- [Windows VM One Click Runbook](windows_vm_one_click_runbook.md)
- [Windows VM Split Phase A](windows_vm_split_phaseA.md)
- [Windows VM Split Phase B](windows_vm_split_phaseB.md)
- [Windows ROS2 Terminal Setup](windows_ros2_terminal_setup.md)

## 历史迁移与工程文档

- [ROS2 Refactor Plan](ros2_refactor_plan.md)
- [GitHub Code Architecture Style](github_code_architecture_style.md)
- [Phase 1 Kickoff](phase1_kickoff.md)
- [Phase 2 Hardware Kickoff](phase2_hardware_kickoff.md)
- [Phase 3 Control Kickoff](phase3_control_kickoff.md)
- [Phase 4 UI Bridge Kickoff](phase4_ui_bridge_kickoff.md)
- [Phase 5 Task Kickoff](phase5_task_kickoff.md)
- [Phase 6 Vision Kickoff](vision_pipeline_kickoff.md)
- [Phase 6 Sim Base Kickoff](phase6_sim_base_kickoff.md)
- [Phase 6 Sim Gazebo Kickoff](phase6_sim_gazebo_kickoff.md)
- [Changelog](../CHANGELOG.md)

## 如何使用这些文档

- 如果你要把系统跑起来：先看 [仓库 README](../README.md)
- 如果你先被缩写和术语卡住：先看 [术语与缩写表](terminology_guide.md)
- 如果你要理解当前主链路：先看 [系统总览说明书](system_overview_manual.md)
- 如果你要按模块理解：直接点上面的五份说明书
- 如果你要追历史演进：再看 phase1~phase6 的 kickoff 文档
