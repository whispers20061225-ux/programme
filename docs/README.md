# Project Structure

This repository follows a common GitHub Python layout:

- `src/`: application source code packages
- `tests/`: automated test code
- `examples/`: demo scripts and visualization examples
- `config/`: runtime and environment configuration files
- `docs/`: project documentation
- `scripts/`: runnable helper scripts
- `models/`: model and mesh assets
- `data/`: sample/input data
- `robotic_arm/`: standalone robotic arm Python package
- `stm32_bridge/`: STM32 side firmware sources

Notes:

- The root `main.py` keeps backward compatibility and adds `src/` to `PYTHONPATH` at runtime.
- New modules should be added under `src/`.
- Python dependencies are managed via `requirements.txt` and `environment.yml`.

## Engineering Docs

- `ros2_refactor_plan.md`: phased ROS2 refactor plan with rollback and GitHub sync strategy.
- `github_code_architecture_style.md`: GitHub-side architecture style during refactor (branches, PR, release, repo layout).
- `phase1_kickoff.md`: concrete phase 1 implementation notes and run/verify steps.
- `phase2_hardware_kickoff.md`: phase 2 hardware-layer node migration notes and run/verify steps.
- `phase3_control_kickoff.md`: phase 3 control-layer migration notes (`/control/arm/*`, `MoveArmJoints` action, emergency reset).
- `phase4_ui_bridge_kickoff.md`: phase 4 UI bridge migration status and verification steps.
- `phase5_task_kickoff.md`: phase 5 ROS2 task orchestration (`/task/execute_demo`, pause/resume/stop services, UI bridge routing).
- `phase6_vision_kickoff.md`: phase 6.1 vision integration (`realsense_monitor_node`, `phase6_vision.launch.py`).
- `windows_vm_split_phaseA.md`: Windows host + Linux VM split deployment (NAT + Host-only + CycloneDDS peer mode).
- `windows_vm_split_phaseB.md`: Phase B split deployment (Windows hardware nodes + VM app/control nodes).
- `windows_ros2_realsense_quickstart.md`: Windows-side ROS2 RealSense node quickstart (build/start/verify/troubleshooting commands).
- `windows_vm_one_click_runbook.md`: one-click runbook for Windows hardware bringup and VM end-to-end debug startup.
- `../CHANGELOG.md`: repository-level change log (phase progress and merged updates).
- `ros2_refactor_plan.md` includes SmolVLA policy migration and legacy AI fallback strategy.

## Current Refactor Snapshot (2026-03-04)

- `develop` now includes phase 6.1 vision ROS2 kickoff scaffolding.
- `develop` now includes phase A split deployment scaffolding.
- `develop` now includes Phase B kickoff files for Windows-hardware + VM-app runtime split.
- `main` currently tracks phase 5 merged baseline.
- GUI demo commands are routed to task Action/Services in ROS2 mode.
- Next target: complete split-mode cross-machine regression and then move to 6b simulation-policy integration.
