# ROS2 Workspace (Phase 1 + Phase 2)

This workspace now includes:

- phase 1: minimal fake tactile chain for GUI read-only validation
- phase 2: hardware-layer ROS2 nodes for tactile sensor, arm, and gripper

## Workspace layout

```text
ros2_ws/
  src/
    tactile_interfaces/
    tactile_hardware/
    tactile_bringup/
    tactile_ui_bridge/
```

## Build (Ubuntu + ROS2 Jazzy)

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## Run phase 1

```bash
ros2 launch tactile_bringup phase1_fake_chain.launch.py

# in another terminal at repository root
python main_ros2.py --tactile-topic /tactile/raw --health-topic /system/health
```

Expected topics:

- `/tactile/raw`
- `/system/health`

## Run phase 2

```bash
ros2 launch tactile_bringup phase2_hardware.launch.py
```

Default phase 2 behavior:

- tactile sensor node publishes `/tactile/raw` (simulation fallback enabled)
- arm driver node publishes `/arm/state` (manual enable by service)
- gripper driver node publishes `/gripper/state` and serves `/gripper/set_force`
- health is reported on `/system/health`

Useful service calls:

```bash
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm/home std_srvs/srv/Trigger "{}"
ros2 service call /gripper/set_force tactile_interfaces/srv/SetGripperForce "{force: 3.0, block: true, timeout_sec: 5.0}"
```
