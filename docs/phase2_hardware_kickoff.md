# Phase 2 Kickoff (Develop Branch)

This file tracks the second implementation step of the ROS2 migration plan.

## Implemented in this kickoff

- Extended `tactile_interfaces` with hardware-layer contracts:
  - `msg/ArmState.msg`
  - `msg/GripperState.msg` (reserved for optional compatibility path)
  - `srv/SetGripperForce.srv` (reserved for optional compatibility path)
- Added `tactile_hardware` package with nodes:
  - `tactile_sensor_node`
  - `arm_driver_node`
- Added `tactile_bringup` phase 2 launch and parameters:
  - `launch/phase2_hardware.launch.py`
  - `config/phase2_hardware.yaml`
- Current default phase 2 chain is arm-centric:
  - only `tactile_sensor_node + arm_driver_node + tactile_ui_subscriber`
- Kept phase 1 and legacy entrypoints untouched:
  - `main.py` unchanged
  - `main_ros2.py` unchanged
  - `phase1_fake_chain.launch.py` unchanged

## Topics and services in phase 2

- Topics:
  - `/tactile/raw`
  - `/arm/state`
  - `/system/health`
- Services:
  - `/arm/enable` (`std_srvs/srv/SetBool`)
  - `/arm/home` (`std_srvs/srv/Trigger`)

## Run guide (Ubuntu + ROS2 Jazzy)

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch tactile_bringup phase2_hardware.launch.py
```

## Validation checklist

- [ ] Launch starts without missing package errors.
- [ ] `ros2 topic list` contains `/tactile/raw`, `/arm/state`, `/system/health`.
- [ ] `ros2 service list` contains `/arm/enable`, `/arm/home`.
- [ ] `ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"` returns success on valid hardware setup.
- [ ] `python main.py` legacy flow remains unaffected.

## Rollback

- Safe rollback target: previous `develop` commit before phase 2 kickoff.
- Runtime fallback: keep using phase 1 chain and/or legacy `main.py`.
