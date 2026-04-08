# 抓取规划与执行说明书

- [返回系统总览](system_overview_manual.md)
- [返回文档索引](README.md)
- [术语与缩写表](terminology_guide.md)

## 这份文档讲什么

这份说明书解释：

- 抓取候选是怎么来的
- pregrasp / grasp 是怎么被规划和筛选的
- 为什么有时“视觉都对了”但还是抓不起来

关键入口文件：

- [grasp_backend_node.py](../ros2_ws/src/tactile_vision/tactile_vision/grasp_backend_node.py)
- [task_executive_node.py](../ros2_ws/src/tactile_task/tactile_task/task_executive_node.py)
- [search_target_skill_node.py](../ros2_ws/src/tactile_task/tactile_task/search_target_skill_node.py)
- [sim_pick_task_node.cpp](../ros2_ws/src/tactile_task_cpp/src/sim_pick_task_node.cpp)
- [GraspProposal.msg](../ros2_ws/src/tactile_interfaces/msg/GraspProposal.msg)
- [GraspProposalArray.msg](../ros2_ws/src/tactile_interfaces/msg/GraspProposalArray.msg)

## 第一层：抓取候选生成

统一入口：

- [grasp_backend_node.py](../ros2_ws/src/tactile_vision/tactile_vision/grasp_backend_node.py)

当前支持的后端主要有：

- `graspgen_zmq`
- `ggcnn_local`
- `contact_graspnet_http`

### 当前主路径

从当前主链参数看，默认主抓取候选后端是：

- `graspgen_zmq`

默认连接：

- host: `127.0.0.1`
- port: `5556`
- repo: `/home/whispers/GraspGen`

输入点云来自：

- `/perception/target_cloud_for_graspgen`

输出候选来自：

- `/grasp/candidate_grasp_proposals`

## 候选里到底包含什么

每个 `GraspProposal` 不只是一个分数，而是一套抓取几何描述：

- `contact_point_1`
- `contact_point_2`
- `grasp_center`
- `approach_direction`
- `closing_direction`
- `grasp_pose`
- `pregrasp_pose`
- `grasp_width_m`
- `confidence_score`
- `semantic_score`

这意味着当前系统里的“抓取策略”不是一句建议，而是真正可交给规划器使用的候选集。

## 第二层：任务编排

任务执行不是 `grasp_backend_node` 自己完成的，而是交给编排层。

主要角色：

- [task_executive_node.py](../ros2_ws/src/tactile_task/tactile_task/task_executive_node.py)
- [search_target_skill_node.py](../ros2_ws/src/tactile_task/tactile_task/search_target_skill_node.py)

它们负责：

- 接收结构化任务
- 发起搜索或等待目标锁定
- 触发 pick sequence
- 维护执行状态和回传消息

## 第三层：真正的执行规划

最终执行核心在：

- [sim_pick_task_node.cpp](../ros2_ws/src/tactile_task_cpp/src/sim_pick_task_node.cpp)

它主要负责：

- 接收外部抓取候选
- 对候选做 executable screening
- 规划到 pregrasp
- 再规划到 grasp
- 执行 return home
- reset pick session

## 为什么 pregrasp 是最常见失败点

这是当前系统里最值得单独理解的一点。

很多时候你会看到：

- 视觉已经找到目标
- 抓取后端已经给出候选
- 甚至 IK 看起来也能过

但系统还是失败。

根因通常在这里：

- 候选抓取姿态存在
- 但从当前机械臂姿态到 pregrasp 姿态的路径，在当前约束、时间预算和采样条件下不一定能规划出来

所以：

- 有候选，不等于可执行
- 有 grasp pose，不等于 pregrasp 一定成功

## 当前主链的执行风格

从参数上看，当前系统比较偏向：

- 优先使用 external grasp candidates
- 先筛掉不可执行候选
- 再决定是否继续执行

相关参数入口：

- [phase8_modular_grasp.yaml](../ros2_ws/src/tactile_bringup/config/phase8_modular_grasp.yaml)

你当前调试时最容易碰到的失败，也通常在这里：

- external candidate 质量不差
- 但 screening 后 `feasible=0`
- 最终报 pregrasp planning fail

## GG-CNN 在这里是什么角色

在当前系统里，GG-CNN 更多是：

- shadow 分支
- grasp overlay 来源
- 调试与对照工具

不要把它和当前主执行链混为一谈。  
你在视觉页看到 grasp overlay，不代表当前真正执行一定来自 GG-CNN。

## reset scene 和 return home

当前执行链和 Web 端是联动的：

- `Return Home`：回到默认关节位
- `Reset Scene`：重置场景

当前配置下，`Reset Scene` 之后会自动做 return home。  
这对连续测试很重要，因为可以快速回到统一起点。

## 调试建议

如果你面对的是“老师预抓取失败”这类问题，先看这些：

- `/grasp/candidate_grasp_proposals`
- `/task/execution_status`
- `sim_pick_task_node` 日志
- `grasp_backend_node` 日志

最常见的问题分类：

- 没有候选：先看抓取后端和点云输入
- 有候选但不可执行：先看 pregrasp screening
- 规划能过但执行乱：再看 home/reset 和机械臂状态
