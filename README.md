# Programme: 触觉抓取 Phase8 Web 端系统

> 当前推荐运行路径：WSL2 / Ubuntu 24.04 / ROS 2 Jazzy / phase8 web stack。  
> 旧的 `main.py` + Qt UI 仍保留用于历史兼容，但不再是当前主链路。

Programme 是一个把“看懂目标、理解指令、规划抓取、执行动作、展示状态”整合到同一个 Web 界面里的桌面抓取系统。对不熟悉机器人系统的使用者来说，它最直接的吸引力不是某个单独算法，而是你可以用自然语言告诉它“抓哪个东西”，然后在同一页面里看到目标识别、抓取候选、执行状态和复位控制，整个过程清楚、直观、可演示。

如果把它当成一个面向客户的系统来看，它的核心卖点是：

- 更容易上手：不用先理解 ROS 话题和节点，直接通过 Web 页面和自然语言开始操作
- 更容易展示：从识别、锁定、规划到执行，关键过程都能可视化
- 更容易调试：控制、视觉、触觉、日志都集中在一个入口，不需要来回切多个工具
- 更贴近真实任务：不是只做检测或只做机械臂控制，而是覆盖完整抓取链路
- 更方便扩展：语义理解、点云处理、抓取策略和执行规划是解耦的，后续替换模型或策略成本更低

## 文档导航

如果你是第一次接触这套系统，建议先看术语表，再按模块读说明书。

- [文档总索引](docs/README.md)
- [术语与缩写表](docs/phase8_glossary.md)
- [Phase8 总览与手册导航](docs/phase8_function_manual.md)
- [Web 端说明书](docs/phase8_web_manual.md)
- [语义与视觉说明书](docs/phase8_semantic_vision_manual.md)
- [点云与几何说明书](docs/phase8_pointcloud_manual.md)
- [抓取与执行说明书](docs/phase8_grasp_execution_manual.md)
- [触觉与运行维护说明书](docs/phase8_tactile_ops_manual.md)

## 1. 项目现在在做什么

这个仓库当前最重要的链路，是一个面向桌面抓取任务的 ROS 2 Web 端系统：

`自然语言指令 -> 多模态语义理解 -> 开放词汇分割与 ROI 锁定 -> 深度反投影与点云滤波 -> 几何拟合/补全 -> 抓取候选生成 -> pregrasp/grasp 规划 -> 执行 -> Web 端监控与复位`

当前主链路围绕 [phase8_web_ui.launch.py](ros2_ws/src/tactile_bringup/launch/phase8_web_ui.launch.py) 展开，核心能力包括：

- 中文或英文任务输入，自动结构化为 `SemanticTask`
- 基于语义提示的开放词汇实例分割与目标锁定
- 从 2D 检测结果反投影到 3D 点云，并做滤波、平面剔除、聚类与几何补全
- 使用 GraspGen 生成外部抓取候选，使用 MoveIt 做 pregrasp/grasp 规划
- Web 端统一提供 `Control / Vision / Tactile / Logs` 四个工作页
- 支持 `Re-plan`、`Return Home`、`Reset Scene` 与调试视图

## 2. 当前推荐看的包

| 包 | 作用 |
| --- | --- |
| `ros2_ws/src/tactile_bringup` | 当前 phase8 的 launch 和参数总入口 |
| `ros2_ws/src/tactile_web_bridge` | FastAPI/WebSocket 网关 + React/Vite Web 前端 |
| `ros2_ws/src/tactile_vision` | 语义理解、开放词汇分割、点云处理、几何拟合、抓取候选后端 |
| `ros2_ws/src/tactile_task` | 搜索与任务编排 |
| `ros2_ws/src/tactile_task_cpp` | MoveIt 执行、pregrasp/grasp 规划与 return home |
| `ros2_ws/src/tactile_control` | 控制层安全代理，向硬件/仿真驱动转发 |
| `ros2_ws/src/tactile_hardware` | 机械臂与触觉传感器驱动，含仿真 fallback |
| `ros2_ws/src/tactile_sim` | Gazebo 仿真、场景复位、搜索扫描等仿真能力 |

如果你现在只是想理解当前系统，不建议先从旧 `src/`、`main.py`、PyQt 开始看。

## 3. 运行环境建议

### 3.1 推荐运行环境

- WSL2
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim / MoveIt 2
- Node.js + npm（用于构建前端）

当前这份 README 以实际运行仓库 `/home/whispers/programme` 为准。

### 3.2 关于 Python 环境

这个仓库同时保留了两套历史：

- `environment.yml` / `requirements.txt`：偏 legacy Python 工具链
- `ros2_ws/`：当前 phase8 ROS 2 主链

当前 phase8 Web 端建议在 WSL 的 ROS 2 shell 中运行，不要把 Windows 侧 Python 环境当作当前主运行环境。

### 3.3 系统依赖

至少需要这些系统级依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-fastapi \
  python3-uvicorn \
  python3-websockets \
  python3-requests \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-xacro \
  ros-jazzy-rmw-cyclonedds-cpp \
  npm
```

另外，运行 phase8 的 Python 解释器还需要能导入这些包：

- `numpy`
- `opencv-python` / `cv2`
- `requests`
- `ultralytics`
- `fastapi`
- `uvicorn`
- `websockets`
- `open3d`（可选但强烈建议，用于更好的点云滤波）

## 4. Phase8 启动前置条件

### 4.1 构建 ROS 2 工作区

```bash
source /opt/ros/jazzy/setup.bash
cd /home/whispers/programme/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 4.2 构建 Web 前端

`tactile_web_gateway` 默认会直接托管 `frontend/dist`，所以第一次运行前要先构建：

```bash
cd /home/whispers/programme/ros2_ws/src/tactile_web_bridge/frontend
npm install
npm run build
```

如果你在改前端，也可以在开发模式下单独跑：

```bash
cd /home/whispers/programme/ros2_ws/src/tactile_web_bridge/frontend
npm run dev
```

### 4.3 配置多模态语义模型

当前 phase8 会从 `~/.config/programme/remote_vlm.env` 读取对话/语义模型配置。一个最小例子：

```bash
mkdir -p ~/.config/programme
cat > ~/.config/programme/remote_vlm.env <<'EOF'
PROGRAMME_DIALOG_MODEL_ENDPOINT=http://127.0.0.1:8000/v1/chat/completions
PROGRAMME_DIALOG_MODEL_NAME=Qwen/Qwen2.5-VL-3B-Instruct-AWQ
PROGRAMME_DIALOG_API_KEY=EMPTY
EOF
```

如果你用的是 DashScope 或其他 OpenAI-compatible 服务，也可以填：

- `DASHSCOPE_API_KEY`
- `DASHSCOPE_BASE_URL`
- `DASHSCOPE_MODEL`
- `OPENAI_API_KEY`
- `OPENAI_BASE_URL`
- `OPENAI_MODEL`

### 4.4 启动 GraspGen ZMQ 服务

当前主抓取候选后端配置为 `graspgen_zmq`，默认连接：

- host: `127.0.0.1`
- port: `5556`
- repo: `/home/whispers/GraspGen`

先确保 GraspGen 服务已经运行。按本机现有 GraspGen 仓库的 client-server 文档，一个常用启动方式是：

```bash
cd /home/whispers/GraspGen
source .venv-cu128/bin/activate
pip install pyzmq msgpack msgpack-numpy
python client-server/graspgen_server.py \
  --gripper_config /path/to/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml \
  --port 5556
```

如果你已经有自己的 GraspGen server 管理方式，只要保证 `127.0.0.1:5556` 可用即可。

### 4.5 启动对话/VLM 服务

README 不强绑定具体服务实现；你只需要保证上面 `remote_vlm.env` 里的 endpoint 已可访问。  
当前代码默认把它当作 OpenAI-compatible 的 chat completion 接口来调用。

## 5. 启动当前全链路

### 5.1 推荐启动命令

```bash
source /opt/ros/jazzy/setup.bash
cd /home/whispers/programme/ros2_ws
source install/setup.bash
ros2 launch tactile_bringup phase8_web_ui.launch.py
```

这条 launch 会拉起当前主链路中的关键节点，包括：

- `qwen_semantic_node`
- `detector_seg_node`
- `cloud_filter_node`
- `primitive_fit_node`
- `grasp_input_cloud_node`
- `grasp_backend_node`
- `task_executive_node`
- `search_target_skill_node`
- `sim_pick_task_node`
- `tactile_web_gateway`

### 5.2 打开 Web 页面

默认地址：

```text
http://127.0.0.1:8765
```

默认网关参数来自 `tactile_web_gateway`：

- host: `127.0.0.1`
- port: `8765`

## 6. 推荐操作流程

1. 打开 `Control` 页，先确认后端连通。
2. 在对话框里输入任务，例如：
   - `抓取蓝色圆柱体`
   - `抓右边那个蓝色圆柱`
   - `只用平行夹爪抓取蓝色圆柱`
3. 先用 `Review` 模式检查结构化任务是否正确；确认后再 `Execute`。
4. 到 `Vision` 页确认目标框、候选列表和当前锁定实例是否正确。
5. 抓取结束后，优先使用 `Reset Scene` 回到初始状态。

当前配置里，`Reset Scene` 会在重置场景后自动 `Return Home`。

## 7. 快速自检

### 7.1 看节点是否都起来了

```bash
ros2 node list | grep -E "tactile_web_gateway|qwen_semantic_node|detector_seg_node|cloud_filter_node|primitive_fit_node|grasp_backend_node|task_executive_node|sim_pick_task_node"
```

### 7.2 看 Web 网关是否可响应

```bash
curl http://127.0.0.1:8765/api/bootstrap
```

### 7.3 看语义与视觉是否有输出

```bash
ros2 topic echo /qwen/semantic_task --once
ros2 topic echo /perception/detection_result --once
ros2 topic echo /grasp/candidate_grasp_proposals --once
```

## 8. 现在该看哪份文档

- 第一次看这些英文缩写：先看 [术语与缩写表](docs/phase8_glossary.md)
- 当前系统的手册首页：[Phase8 总览与手册导航](docs/phase8_function_manual.md)
- Web 交互入口：[Web 端说明书](docs/phase8_web_manual.md)
- 语义与目标锁定：[语义与视觉说明书](docs/phase8_semantic_vision_manual.md)
- 点云、反投影和拟合：[点云与几何说明书](docs/phase8_pointcloud_manual.md)
- 抓取候选与执行规划：[抓取与执行说明书](docs/phase8_grasp_execution_manual.md)
- 触觉与测试复位：[触觉与运行维护说明书](docs/phase8_tactile_ops_manual.md)
- 文档总索引：[docs/README.md](docs/README.md)
- Windows/VM 和 RealSense 的历史部署文档：
  - [windows_ros2_realsense_quickstart.md](docs/windows_ros2_realsense_quickstart.md)
  - [windows_vm_one_click_runbook.md](docs/windows_vm_one_click_runbook.md)
  - [windows_vm_split_phaseA.md](docs/windows_vm_split_phaseA.md)
  - [windows_vm_split_phaseB.md](docs/windows_vm_split_phaseB.md)

## 9. 说明

- 本 README 只覆盖当前 phase8 Web 主链路。
- 旧 Qt UI 不再作为当前推荐入口。
- `main.py`、PyQt、旧 `src/` 目录仍保留，但它们主要属于历史兼容路径。
