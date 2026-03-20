# Changelog

All notable project updates are recorded in this file.

## [Unreleased]

- Next planned step:
  - Phase 6.2 Gazebo + MoveIt2 + SmolVLA shadow-policy integration.
- Qwen2.5-VL shadow-mode kickoff:
  - Added `tactile_vision/qwen_vl_target_node`.
  - Added bringup entrypoint:
    - `phase6_qwen_shadow.launch.py`
    - `phase6_qwen_shadow.yaml`
  - The node consumes RGB frames and optional prompt text, calls a local
    OpenAI-compatible multimodal endpoint, and publishes normalized JSON to
    `/qwen/vision_result` without driving the existing pick pipeline.
- Split-stream stability optimization (Windows + VM):
  - Added C++ latest-frame relay package:
    - `ros2_ws/src/tactile_vision_cpp`
    - node: `latest_frame_relay_node`
  - Wired relay node into launch/config:
    - `split_vm_app.launch.py` / `split_vm_app.yaml`
    - `phase6_vision.launch.py` / `phase6_vision.yaml`
  - UI path now consumes relay topics:
    - `/camera/relay/color/image_raw`
    - `/camera/relay/aligned_depth_to_color/image_raw`
    - `/camera/relay/color/camera_info`
  - `Ros2DataAcquisitionThread` now supports queue-size-1 frame consumption:
    - added `consume_latest_vision_frame()`
    - vision frame signal push can be disabled (`vision_emit_signal=False`)
    - vision parse errors are rate-limited.
  - `MainWindow` ROS2 vision path switched to pull+latest-frame rendering:
    - single-slot pending frame overwrite
    - fixed-rate render pump (15Hz target)
    - reduced Qt event queue buildup under long runs.
  - Windows startup warmup hardening:
    - `env_ros2_windows.ps1` supports ROS graph warmup (`ros2 daemon stop/start`)
    - `start_realsense_ready.ps1`, `start_hw_nodes.ps1`, `realsense_watchdog.ps1` enabled warmup by default.
  - VM one-click startup hardening:
    - stale process cleanup before launch
    - relay topic availability guard before UI start.
- Windows + VM split deployment kickoff (Phase A):
  - Added CycloneDDS peer-mode templates:
    - `config/dds/cyclonedds_windows.xml`
    - `config/dds/cyclonedds_vm.xml`
  - Added deployment scripts:
    - `deploy/windows/env_ros2_windows.ps1`
    - `deploy/windows/check_connectivity.ps1`
    - `deploy/windows/start_hw_nodes.ps1`
    - `deploy/vm/env_ros2_vm.sh`
    - `deploy/vm/check_connectivity.sh`
  - Added runbook:
    - `docs/windows_vm_split_phaseA.md`
  - Added Windows runtime hardening in `deploy/windows/env_ros2_windows.ps1`:
    - clean conda-injected env vars / PATH entries
    - auto prepend pixi OpenSSL DLL path for CycloneDDS dependency chain
    - use runtime-local `CYCLONEDDS_URI` file path to avoid Windows URI parsing issues
  - Added operator docs:
    - `docs/windows_ros2_terminal_setup.md`
    - `docs/windows_ros2_realsense_quickstart.md`
  - Goal: keep NAT for internet, use Host-only for deterministic ROS2 communication.
- Windows + VM split execution kickoff (Phase B):
  - Added VM app-side launch:
    - `ros2_ws/src/tactile_bringup/launch/split_vm_app.launch.py`
    - `ros2_ws/src/tactile_bringup/config/split_vm_app.yaml`
  - Added Windows hardware-side arm config:
    - `ros2_ws/src/tactile_bringup/config/split_windows_hardware.yaml`
  - Added VM app start helper:
    - `deploy/vm/start_split_vm_app.sh`
  - Updated Windows hardware start helper:
    - `deploy/windows/start_hw_nodes.ps1` now defaults to `split_windows_hardware.yaml`
    - fallback RealSense publish path:
      - uses `tactile_vision/realsense_camera_node` when `realsense2_camera` package is not available on Windows
    - added RealSense-only helper:
      - `deploy/windows/start_realsense_only.ps1`
  - Added RealSense publisher node in `tactile_vision`:
    - `realsense_camera_node.py` (pyrealsense2 based)
  - Added runbook:
    - `docs/windows_vm_split_phaseB.md`
- Phase 6.1 kickoff:
  - Added package `tactile_vision` with `realsense_monitor_node`.
  - Added `phase6_vision.launch.py` and `phase6_vision.yaml`.
  - Added D455 stream health reporting to `/system/health`.
  - Clarified runtime boundary:
    - RealSense image stream uses USB ROS2 topics.
    - STM32 remains serial chain.

## [2026-03-03] - Phase 5 stabilization complete

- Task orchestration:
  - Added `ExecuteDemo.action` in `tactile_interfaces`.
  - Added new package `tactile_task` with `demo_task_node`.
  - Added task endpoints:
    - `/task/execute_demo` (Action)
    - `/task/pause_demo`, `/task/resume_demo`, `/task/stop_demo` (Services)
- UI bridge migration:
  - ROS2 GUI bridge now routes `start_demo/pause_demo/resume_demo/stop_demo` to `/task/*`.
  - Demo feedback/result is re-emitted to existing GUI status channels.
  - Removed hard-coded simulation flag in ROS2 UI bridge:
    - STM32/tactile `simulation` status now derives from ROS2 health messages.
- Hardware/control chain fixes:
  - Fixed `arm_driver_node` startup crash on array parameters by using explicit ROS2 parameter types.
  - Fixed ROS2 path so `joint_limit_disabled_ids` is correctly propagated to legacy arm interface.
  - Kept calibrated J3 zero offset and focused the fix on limit-control path.
  - `phase5_task.launch.py` now defaults to hardware profile (`phase5_task_hardware.yaml`).
  - `tactile_sensor_node` no longer publishes simulated tactile frames when `use_simulation=false`.
  - STM32 connect flow in ROS2 GUI bridge now performs real `/control/arm/enable` call instead of optimistic mock status.
  - Improved diagnostics for missing arm state:
    - `arm state not received (check arm_driver_node and /arm/state)`.
- Bringup and config:
  - Added/updated:
    - `phase5_task.launch.py`
    - `phase5_task.yaml`
    - `phase5_task_hardware.yaml`
  - Added launch argument for config selection:
    - `ros2 launch tactile_bringup phase5_task.launch.py param_file:=...`
- Acceptance:
  - VM + hardware validation passed for phase 5 control/task chain.

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
