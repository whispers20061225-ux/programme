# Phase 4 Kickoff (Develop Branch)

This file tracks the first implementation batch of UI-to-ROS2 control migration.

## Implemented in this kickoff

- Upgraded `main_ros2.py` from phase1 monitor entrypoint to phase4 UI bridge entrypoint.
- Added real ROS2 control thread in `src/core/ros2_runtime_stubs.py`:
  - `Ros2ControlThread` subscribes:
    - `/arm/state`
    - `/system/health`
  - and calls control-layer services:
    - `/control/arm/enable`
    - `/control/arm/home`
    - `/control/arm/move_joint`
    - `/control/arm/move_joints`
    - `/system/reset_emergency`
- Preserved fallback mode:
  - `Ros2ControlThreadStub`
  - runtime switch: `--control-mode ros2|stub`
- Updated ROS2 demo manager bridge:
  - forwards UI commands to ROS2 control thread:
    - `connect_hardware`, `disconnect_hardware`
    - `connect_stm32`, `disconnect_stm32`
    - `connect_tactile`, `disconnect_tactile`
    - `connect_arm`, `disconnect_arm`
    - `arm_enable`, `arm_disable`, `arm_home`
    - `move_arm_joint`, `move_arm_joints`
    - `emergency_stop`, `reset_system`
  - keeps non-migrated orchestration commands in compatibility mode.

## Current behavior boundary

- Migrated:
  - GUI arm command path -> `/control/arm/*`
  - GUI arm state display <- `/arm/state`
  - health warnings <- `/system/health`
- Single-arm architecture:
  - No dedicated gripper node is required in current hardware model.
  - Gripper-related UI commands are mapped to an arm joint service call in ROS2 bridge layer.
- Task orchestration:
  - This document reflects phase-4 completion baseline.
  - Full ROS2 Action task orchestration is tracked in `docs/phase5_task_kickoff.md`.

## Runtime commands

Terminal A:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch tactile_bringup phase3_control.launch.py
```

Terminal B (repository root):

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python main_ros2.py --control-mode ros2
```

Fallback (read-only/stub):

```bash
python main_ros2.py --control-mode stub
```

## Compatibility and rollback

- `main.py` legacy workflow remains unchanged.
- `main_ros2.py --control-mode stub` keeps previous monitor semantics.
- Rollback target: previous phase3 commit on `develop`.

## Handoff to phase 5

- Phase 5 migrates `start_demo/pause_demo/resume_demo/stop_demo` to task Action/Services.
- Use `phase5_task.launch.py` for the task-orchestration acceptance route.
