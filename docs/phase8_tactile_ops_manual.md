# Phase8 触觉与运行维护说明书

- [返回 Phase8 总览](phase8_function_manual.md)
- [返回文档索引](README.md)
- [术语与缩写表](phase8_glossary.md)

## 这份文档讲什么

这份说明书讲两类内容：

- 触觉和健康状态是怎么接进系统的
- 日常测试时最常用的运行维护动作是什么

关键入口文件：

- [tactile_sensor_node.py](../ros2_ws/src/tactile_hardware/tactile_hardware/tactile_sensor_node.py)
- [arm_control_node.py](../ros2_ws/src/tactile_control/tactile_control/arm_control_node.py)
- [TactileRaw.msg](../ros2_ws/src/tactile_interfaces/msg/TactileRaw.msg)
- [SystemHealth.msg](../ros2_ws/src/tactile_interfaces/msg/SystemHealth.msg)

## 触觉模块现在是什么状态

当前触觉已经接入 Web 端，但角色主要还是：

- 原始阵列显示
- 力分量显示
- 系统健康与状态监控

它还没有完全发展成“基于触觉主动修正抓取动作”的闭环控制器。

## `tactile_sensor_node` 在做什么

对应文件：

- [tactile_sensor_node.py](../ros2_ws/src/tactile_hardware/tactile_hardware/tactile_sensor_node.py)

它负责发布：

- `/tactile/raw`
- `/system/health`

当前支持两种来源：

- 真实传感器
- simulation fallback

这也是为什么即使没有真实触觉硬件，Web `Tactile` 页通常也还能看到数据。

## `arm_control_node` 在做什么

对应文件：

- [arm_control_node.py](../ros2_ws/src/tactile_control/tactile_control/arm_control_node.py)

这是当前控制层安全代理。  
Web 端点按钮时，真正走的是控制层代理，而不是直接越过它去调底层硬件驱动。

它主要负责：

- enable / home / move joints 转发
- 监视 arm state
- 监视 system health
- 在故障时做控制阻断

## Web 端里触觉和运维相关的页面

### Tactile 页

主要看：

- 热力图
- `forces / fx / fy / fz`
- 更新时间

### Logs 页

主要看：

- 任务 stepper
- backend events
- frontend events

### Control 页

和日常测试关系最紧的两个按钮：

- `Return Home`
- `Reset Scene`

## 日常测试最常用的维护动作

### Return Home

适合：

- 当前姿态跑偏了
- 想把机械臂回到统一起始位
- 不想重置整个场景

### Reset Scene

适合：

- 目标物体位置乱了
- 仿真状态已经不干净
- 需要开始新一轮抓取测试

当前 phase8 配置里，它通常会在 reset 后自动执行 return home。

## 触觉和健康为什么重要

即使当前触觉还不是闭环控制主角，它仍然有两个现实价值：

- 帮你快速确认系统是不是在正常发布实时状态
- 帮你在演示时给出“系统有感知反馈”的可视化证据

而健康状态的价值更直接：

- arm 是否在线
- 节点是否健康
- 当前是不是处于可执行状态

## 当前限制

当前要明确几件事：

- Web 端里的急停仍是显式占位，不代表完整急停链已经接通
- 触觉页当前主要展示原始数据，不代表已经做了高级触觉策略
- 运行维护动作仍然以 `return home + reset scene` 为核心

## 调试建议

你如果遇到这些问题，可以先看对应位置：

| 问题 | 先看哪里 |
| --- | --- |
| Tactile 页空白 | `/tactile/raw`、`tactile_sensor_node` |
| 机械臂状态不更新 | `/arm/state`、`arm_control_node` |
| Return Home 失败 | 控制层服务、机械臂状态、`sim_pick_task_node` 日志 |
| Reset Scene 后还是乱 | 场景复位服务和 home 联动逻辑 |
