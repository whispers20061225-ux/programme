# Changelog

All notable project updates are recorded in this file.

## [Unreleased]

- Phase 5 kickoff:
  - Added `ExecuteDemo.action` in `tactile_interfaces`.
  - Added new package `tactile_task` with `demo_task_node`.
  - Added task endpoints:
    - `/task/execute_demo` (Action)
    - `/task/pause_demo`, `/task/resume_demo`, `/task/stop_demo` (Services)
  - Updated ROS2 GUI bridge to route:
    - `start_demo`, `pause_demo`, `resume_demo`, `stop_demo`
    through the task endpoints.
  - Added bringup and config:
    - `phase5_task.launch.py`
    - `phase5_task.yaml`
- Next planned step:
  - Phase 5 stabilization and acceptance in VM/hardware.
  - Phase 6 policy backend migration (SmolVLA integration).

## [2026-03-03] - Phase 4 merged to `main`

- Merge:
  - Merged `develop` into `main` with phase-4 ROS2 UI bridge updates.
- Control bridge coverage expanded:
  - Added ROS2 runtime compatibility handling for:
    - `move_gripper`
    - `set_servo_position`
    - `set_servo_speed`
    - `set_servo_force`
    - `calibrate_hardware`
    - `calibrate_3d`
    - `auto_grasp` (compatibility mode)
- Stability:
  - Graceful shutdown handling for ROS2 threads on Ctrl+C.
  - Reduced Matplotlib font warning noise and improved CJK font loading behavior.
- Documentation:
  - Updated phase 4 status documentation.
  - Updated docs index to reflect merged state and next-phase target.

## [2026-03-02] - Phase 4 rollout updates

- Introduced ROS2 GUI control thread path and command forwarding to `/control/arm/*`.
- Added `main_ros2.py` runtime improvements and environment run guidance.
- Fixed multiple runtime warning/error issues discovered during VM acceptance.

## [2026-03-01] - Phase 3 baseline

- Added control-layer arm node and bringup pipeline.
- Added ROS2 arm joint services and action baseline.
