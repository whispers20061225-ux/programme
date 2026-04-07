# Phase8 Web 端说明书

- [返回 Phase8 总览](phase8_function_manual.md)
- [返回文档索引](README.md)
- [术语与缩写表](phase8_glossary.md)

## 这份文档讲什么

这份说明书只讲当前 Web 端怎么用、它背后连了什么、以及页面上的每个功能到底对应什么系统能力。

当前 Web 端入口：

- [tactile_web_gateway](../ros2_ws/src/tactile_web_bridge/tactile_web_bridge/web_gateway.py)
- [前端 App](../ros2_ws/src/tactile_web_bridge/frontend/src/App.tsx)

## Web 端的定位

当前 Web 端不是一个“看日志的壳”，而是整个系统的统一操作台：

- 输入任务
- 看目标有没有识别对
- 看抓取有没有锁定对
- 发起执行、重规划、回家位、场景重置
- 看触觉和系统状态
- 看执行过程和后端日志

如果你面对的是第一次接触这个项目的人，他不需要先理解 ROS 节点，先会用这个 Web 端就够了。

## 四个页面分别做什么

### 1. Control

对应文件：

- [ControlPage.tsx](../ros2_ws/src/tactile_web_bridge/frontend/src/ControlPage.tsx)

这是当前真正的任务控制中心，主要功能有：

- 多轮对话输入
- `Review` 和 `Auto` 模式切换
- 回复语言切换
- 查看结构化任务草稿
- `Execute`
- `Re-plan`
- `Return Home`
- `Reset Scene`
- `Open Debug`

这里有一个常见误解：  
当前 Web 端没有单独名为 `Task` 的页签，任务控制实际上就在 `Control` 页里。

### 2. Vision

对应文件：

- [VisionPage.tsx](../ros2_ws/src/tactile_web_bridge/frontend/src/VisionPage.tsx)

这里主要看三件事：

- 检测 overlay 是不是框对了
- 当前锁定的是不是你要抓的那个实例
- top-k 候选里有没有更合适的目标

它支持点击候选实例，把该实例直接提交为当前目标 override。

### 3. Tactile

对应文件：

- [TactilePage.tsx](../ros2_ws/src/tactile_web_bridge/frontend/src/TactilePage.tsx)

这里目前更像监控页，不是触觉闭环策略页。  
当前主要显示：

- taxel 热力图
- 力分量曲线
- 更新时间、序列号、frame id

### 4. Logs

对应文件：

- [LogsPage.tsx](../ros2_ws/src/tactile_web_bridge/frontend/src/LogsPage.tsx)

这里主要回答“系统现在做到哪一步了”：

- stepper
- backend events
- frontend session events
- export JSON

## Web 端用了哪些 AI

当前至少有三层 AI 相关能力接入到 Web 交互里：

| 能力 | 当前默认实现 | 作用 |
| --- | --- | --- |
| 对话控制 | OpenAI-compatible chat endpoint | 解析用户输入、维持多轮会话 |
| 任务语义结构化 | `qwen_semantic_node` | 产出 `SemanticTask` |
| 视觉语义增强 | scene prompt / relabel 路径 | 让视觉侧更容易找到正确实例 |

默认对话模型参数可以在这里看到：

- [phase8_modular_grasp.yaml](../ros2_ws/src/tactile_bringup/config/phase8_modular_grasp.yaml)
- [web_gateway.py](../ros2_ws/src/tactile_web_bridge/tactile_web_bridge/web_gateway.py)

## 你应该怎么和它对话

建议说法尽量“短、明确、带约束”。

示例：

- `抓取蓝色圆柱体`
- `抓右边那个蓝色圆柱`
- `只用平行夹爪抓取蓝色圆柱`
- `先不要执行，只更新目标`
- `现在开始执行`

英文也可以：

- `pick the blue cylinder`
- `pick the rightmost blue cylinder`
- `update the task only, do not execute yet`

## Review 和 Auto 的区别

### Review

只更新任务，不自动执行。  
适合调试语义理解、目标锁定和约束设置。

### Auto

语义足够清楚且目标锁定后，会自动推进执行。  
适合演示或熟练操作时减少人工点击。

## 和后端对接的主要 API

对应文件：

- [api.ts](../ros2_ws/src/tactile_web_bridge/frontend/src/api.ts)

当前最常用接口：

- `/api/bootstrap`
- `/api/dialog/message`
- `/api/dialog/mode`
- `/api/dialog/reply-language`
- `/api/dialog/reset`
- `/api/task/override`
- `/api/task/replan`
- `/api/execution/execute`
- `/api/execution/return-home`
- `/api/execution/reset-scene`
- `/api/debug/open-views`
- `/ws/live`

## 你最该记住的两个按钮

### Return Home

把机械臂回到默认 home 关节位。  
它解决的是“姿态乱了但场景还不需要重置”的问题。

### Reset Scene

把仿真场景重置回初始状态。  
当前 phase8 配置里，`Reset Scene` 之后会自动执行 return home。

## 建议的使用顺序

1. 先在 `Control` 页输入任务。
2. 用 `Review` 模式确认结构化任务是否正确。
3. 到 `Vision` 页确认是否锁定了正确实例。
4. 必要时点选候选做 override。
5. 回到 `Control` 页点击 `Execute`。
6. 每轮测试后优先用 `Reset Scene` 回到初始状态。
