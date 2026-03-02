# Phase 3 Kickoff (Develop Branch)

This file tracks the implemented control-layer migration scope in phase 3.

## Implemented in this kickoff

- Added ROS2 arm command contracts:
  - `srv/MoveArmJoint.srv`
  - `srv/MoveArmJoints.srv`
  - `action/MoveArmJoints.action`
- Registered new interfaces in:
  - `ros2_ws/src/tactile_interfaces/CMakeLists.txt`
- Extended `arm_driver_node` with command services:
  - `/arm/move_joint`
  - `/arm/move_joints`
- Added `tactile_control` package and `arm_control_node`:
  - proxies hardware services (`/arm/*`) to control services (`/control/arm/*`)
  - enforces state freshness, arm readiness, id/angle validation
  - latches emergency by `/system/health` and exposes `/system/reset_emergency`
  - provides action endpoint `/control/arm/move_joints_action`
- Command execution path reuses legacy backend:
  - `LearmInterface.move_joint(...)`
  - `LearmInterface.move_joints(...)`
- Added command validation and safety checks:
  - joint id range check (`1..arm_num_joints`)
  - finite-angle check
  - optional angle range guard (`min_angle_deg..max_angle_deg`)
  - duplicate-id guard for multi-joint command
- Added phase config parameters for control services:
  - `arm_num_joints`
  - `command_default_duration_ms`
  - `enforce_angle_limits`
  - `min_angle_deg`
  - `max_angle_deg`
  - `arm_state_timeout_sec`
  - `command_timeout_sec`
  - `emergency_latch_level`
  - `client_retry_count`
- Added dedicated phase 3 bringup:
  - `ros2_ws/src/tactile_bringup/launch/phase3_control.launch.py`
  - `ros2_ws/src/tactile_bringup/config/phase3_control.yaml`

## Service API summary

- Hardware layer:
  - `/arm/enable` (`std_srvs/srv/SetBool`)
  - `/arm/home` (`std_srvs/srv/Trigger`)
  - `/arm/move_joint` (`tactile_interfaces/srv/MoveArmJoint`)
  - `/arm/move_joints` (`tactile_interfaces/srv/MoveArmJoints`)
- Control layer (phase 3 target API):
  - `/control/arm/enable` (`std_srvs/srv/SetBool`)
  - `/control/arm/home` (`std_srvs/srv/Trigger`)
  - `/control/arm/move_joint` (`tactile_interfaces/srv/MoveArmJoint`)
  - `/control/arm/move_joints` (`tactile_interfaces/srv/MoveArmJoints`)
  - `/control/arm/move_joints_action` (`tactile_interfaces/action/MoveArmJoints`)
  - `/system/reset_emergency` (`std_srvs/srv/Trigger`)

## Validation commands

```bash
ros2 launch tactile_bringup phase3_control.launch.py
ros2 service call /control/arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/arm/move_joint tactile_interfaces/srv/MoveArmJoint "{joint_id: 1, angle_deg: 25.0, duration_ms: 1000, wait: true}"
ros2 service call /control/arm/move_joints tactile_interfaces/srv/MoveArmJoints "{joint_ids: [1,2,3], angles_deg: [15.0, 30.0, 20.0], duration_ms: 1200, wait: true}"
ros2 action send_goal /control/arm/move_joints_action tactile_interfaces/action/MoveArmJoints "{joint_ids: [1,2,3], angles_deg: [15.0, 30.0, 20.0], duration_ms: 1200, wait: true}" --feedback
ros2 service call /control/arm/home std_srvs/srv/Trigger "{}"
ros2 service call /system/reset_emergency std_srvs/srv/Trigger "{}"
```

## Compatibility and rollback

- Phase 2 launch and APIs remain available (`phase2_hardware.launch.py` unchanged).
- `main.py` legacy path remains unchanged.
- Rollback target: previous `develop` phase-2 commit.
