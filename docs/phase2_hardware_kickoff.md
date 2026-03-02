# Phase 2 Kickoff (Develop Branch)

This file tracks the second implementation step of the ROS2 migration plan.

## Implemented in this kickoff

- Extended `tactile_interfaces` with hardware-layer contracts:
  - `msg/ArmState.msg`
  - `msg/GripperState.msg`
  - `srv/SetGripperForce.srv`
- Added `tactile_hardware` package with 3 nodes:
  - `tactile_sensor_node`
  - `arm_driver_node`
  - `gripper_driver_node`
- Added `tactile_bringup` phase 2 launch and parameters:
  - `launch/phase2_hardware.launch.py`
  - `config/phase2_hardware.yaml`
- Kept phase 1 and legacy entrypoints untouched:
  - `main.py` unchanged
  - `main_ros2.py` unchanged
  - `phase1_fake_chain.launch.py` unchanged

## Topics and services in phase 2

- Topics:
  - `/tactile/raw`
  - `/arm/state`
  - `/gripper/state`
  - `/system/health`
- Services:
  - `/arm/enable` (`std_srvs/srv/SetBool`)
  - `/arm/home` (`std_srvs/srv/Trigger`)
  - `/gripper/set_force` (`tactile_interfaces/srv/SetGripperForce`)

## Run guide (Ubuntu + ROS2 Jazzy)

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch tactile_bringup phase2_hardware.launch.py
```

## Validation checklist

- [ ] Launch starts without missing package errors.
- [ ] `ros2 topic list` contains `/tactile/raw`, `/arm/state`, `/gripper/state`, `/system/health`.
- [ ] `ros2 service list` contains `/arm/enable`, `/arm/home`, `/gripper/set_force`.
- [ ] `ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"` returns success on valid hardware setup.
- [ ] `ros2 service call /gripper/set_force tactile_interfaces/srv/SetGripperForce "{force: 3.0, block: true, timeout_sec: 5.0}"` returns success in simulation mode.
- [ ] `python main.py` legacy flow remains unaffected.

## Rollback

- Safe rollback target: previous `develop` commit before phase 2 kickoff.
- Runtime fallback: keep using phase 1 chain and/or legacy `main.py`.
