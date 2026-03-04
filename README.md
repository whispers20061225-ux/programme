# 触觉夹爪演示系统（programme）

一个面向机械臂抓取场景的触觉感知与控制项目，包含：

- 触觉传感数据采集与处理
- 夹爪/舵机控制
- 视觉与姿态估计模块
- 仿真（PyBullet）与 GUI
- 深度学习相关推理与训练组件

仓库已按主流 GitHub Python 项目结构标准化，便于后续开发、协作与扩展。

## 1. 项目结构

```text
programme/
├─ src/                    # 核心源码（业务模块）
│  ├─ arm_control/         # 机械臂接口与关节/笛卡尔控制
│  ├─ communication/       # API / MQTT 通信
│  ├─ core/                # 系统控制、数据采集、演示管理
│  ├─ data_management/     # 数据集构建与实验记录
│  ├─ deep_learning/       # 模型、推理、训练与在线学习
│  ├─ gui/                 # PyQt 图形界面
│  ├─ motion_planning/     # 轨迹与路径规划
│  ├─ perception/          # 视觉/触觉感知处理
│  ├─ servo_control/       # 夹爪与力控制
│  ├─ simulation/          # PyBullet 仿真
│  ├─ tactile_perception/  # 触觉映射与分析
│  └─ utils/               # 通用工具
├─ config/                 # 配置文件与配置数据结构
├─ scripts/                # 工程脚本（串口、仿真、手眼工具等）
├─ examples/               # 演示样例脚本
├─ tests/                  # 自动化测试目录（规范占位）
├─ docs/                   # 项目文档
├─ models/                 # URDF / mesh / 模型权重
├─ data/                   # 数据与采集结果
├─ robotic_arm/            # 独立机械臂 Python 包（历史兼容）
├─ stm32_bridge/           # STM32 侧桥接代码
├─ main.py                 # 主入口
├─ requirements.txt        # pip 依赖
└─ environment.yml         # conda 环境
```

## 2. 环境准备

建议 Python 3.12。

### 2.1 Conda（推荐）

```bash
conda env create -f environment.yml
conda activate dayiprogramme312
```

### 2.2 pip

```bash
python -m venv .venv
# Windows:
.venv\Scripts\activate
# macOS/Linux:
# source .venv/bin/activate
pip install -r requirements.txt
```

## 3. 运行方式

在仓库根目录执行。

### 3.1 启动主程序（GUI + 控制流程）

```bash
python main.py
```

### 3.2 常用参数

```bash
python main.py --sensor-type 3x3 --sensor-port COM3 --servo-port COM4
```

可用参数（来自 `main.py`）：

- `--config`：配置文件路径（yaml/json）
- `--sensor-type`：`default` 或 `3x3`
- `--sensor-port`：传感器串口，默认 `COM3`
- `--servo-port`：舵机串口，默认 `COM4`
- `--demo`：指定演示模式
- `--params`：演示参数（JSON 字符串）
- `--log-level`：`DEBUG/INFO/WARNING/ERROR`
- `--deep-learning`：启用深度学习模块
- `--model-path`：模型文件路径
- `--object-type`：目标物体类型（如 `hard_plastic`）

### 3.3 仿真与工具脚本

```bash
python scripts/run_pybullet_gui.py
python scripts/ds_servo_test.py --port COM4 --id 1
python scripts/hand_eye_quick_calc.py
```

### 3.4 示例脚本

```bash
python examples/tactile_mapper_demo.py
```

## 4. 演示模式

`core.demo_manager` 中定义了多种模式，可通过 `--demo` 启动：

- `calibration`
- `grasping`
- `slip_detection`
- `object_classification`
- `force_control`
- `learning`
- `vector_visualization`
- `tactile_mapping`

示例：

```bash
python main.py --demo tactile_mapping --params "{\"duration\": 10}"
```

## 5. 配置说明

主要配置在 `config/`：

- `default_config.yaml` / `default_config.json`：默认配置
- `demo_config.py`：配置数据结构与加载逻辑
- `paxini_gen3_config.py`：3x3 触觉传感器相关配置
- `simulation_config.py`：仿真配置
- `deep_learning_config.py`：深度学习相关配置

建议优先通过配置文件管理参数，不要在业务代码中硬编码端口和阈值。

## 6. 开发规范

- 新业务代码放在 `src/` 下对应子模块
- 自动化测试放在 `tests/`，命名建议 `test_*.py`
- 手动调试脚本放在 `scripts/` 或 `examples/`
- 文档放在 `docs/`
- 依赖优先维护在 `requirements.txt` 与 `environment.yml`

仓库已提供：

- `.editorconfig`：统一编码、缩进与换行
- `.gitattributes`：统一文本 LF 行尾并标记二进制资源

## 7. 已知事项

- 本项目包含硬件相关代码（串口、舵机、传感器），部分功能需真实设备才能完整运行。
- `logs/` 已加入 `.gitignore`，避免大体积日志进入版本库。
- `main.py` 会自动将 `src/` 加入 `PYTHONPATH`，兼容当前 `src` 布局。

## 8. 后续建议

- 在 `tests/` 中补充可持续运行的单元测试/集成测试（尽量与硬件解耦）
- 增加 CI（例如 GitHub Actions）进行 `lint + test`
- 逐步补齐 `docs/`（架构图、模块接口、部署说明）

## 9. ROS2 重构进展

- 重构开发主线位于 `develop` 分支。
- 第 1 阶段骨架已创建在 `ros2_ws/`，包括：
- `tactile_interfaces`
- `tactile_bringup`
- `tactile_ui_bridge`
- 新增 `main_ros2.py` 作为第 1 阶段 ROS2 监控入口（只读数据链路）。
- 详细说明见 `docs/phase1_kickoff.md` 与 `docs/ros2_refactor_plan.md`。
