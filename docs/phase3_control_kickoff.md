# Phase 3 Kickoff (Develop Branch)

This file tracks the first implementation step of control-layer migration.

## Implemented in this kickoff

- Added ROS2 arm control service contracts:
  - `srv/MoveArmJoint.srv`
  - `srv/MoveArmJoints.srv`
- Registered new interfaces in:
  - `ros2_ws/src/tactile_interfaces/CMakeLists.txt`
- Extended `arm_driver_node` with command services:
  - `/arm/move_joint`
  - `/arm/move_joints`
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

## Service API summary

- `/arm/enable` (`std_srvs/srv/SetBool`)
- `/arm/home` (`std_srvs/srv/Trigger`)
- `/arm/move_joint` (`tactile_interfaces/srv/MoveArmJoint`)
- `/arm/move_joints` (`tactile_interfaces/srv/MoveArmJoints`)

## Validation commands

```bash
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm/move_joint tactile_interfaces/srv/MoveArmJoint "{joint_id: 1, angle_deg: 25.0, duration_ms: 1000, wait: true}"
ros2 service call /arm/move_joints tactile_interfaces/srv/MoveArmJoints "{joint_ids: [1,2,3], angles_deg: [15.0, 30.0, 20.0], duration_ms: 1200, wait: true}"
ros2 service call /arm/home std_srvs/srv/Trigger "{}"
```

## Compatibility and rollback

- Phase 2 topics/services remain available.
- `main.py` legacy path remains unchanged.
- Rollback target: previous `develop` commit before this kickoff.
