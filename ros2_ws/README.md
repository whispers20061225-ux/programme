# ROS2 Workspace (Phase 1 + Phase 2 + Phase 3 + Phase 4 + Phase 5 + Phase 6.1)

This workspace now includes:

- phase 1: minimal fake tactile chain for GUI read-only validation
- phase 2: hardware-layer ROS2 nodes for tactile sensor and arm
- phase 3: control-layer ROS2 node with safety gate and action/service APIs
- phase 4: GUI command bridge to ROS2 control-layer services
- phase 5: task orchestration (`/task/execute_demo` Action + pause/resume/stop services)
- phase 6.1: vision ROS2 kickoff (`realsense_monitor_node` + `phase6_vision.launch.py`)

## Workspace layout

```text
ros2_ws/
  src/
    tactile_interfaces/
    tactile_hardware/
    tactile_control/
    tactile_task/
    tactile_bringup/
    tactile_ui_bridge/
    tactile_vision/
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
- arm driver node publishes `/arm/state` and controls joint ids `1..6` (manual enable by service)
- health is reported on `/system/health`

Useful service calls:

```bash
ros2 service call /arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm/home std_srvs/srv/Trigger "{}"
ros2 service call /arm/move_joint tactile_interfaces/srv/MoveArmJoint "{joint_id: 1, angle_deg: 30.0, duration_ms: 1200, wait: true}"
ros2 service call /arm/move_joints tactile_interfaces/srv/MoveArmJoints "{joint_ids: [1,2,3], angles_deg: [20.0, 35.0, 15.0], duration_ms: 1500, wait: true}"
```

## Run phase 3

```bash
ros2 launch tactile_bringup phase3_control.launch.py
```

Phase 3 adds `arm_control_node` between UI/task side and hardware services:

- control services:
  - `/control/arm/enable`
  - `/control/arm/home`
  - `/control/arm/move_joint`
  - `/control/arm/move_joints`
- control action:
  - `/control/arm/move_joints_action`
- emergency reset:
  - `/system/reset_emergency`

Useful control-layer calls:

```bash
ros2 service call /control/arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/arm/move_joint tactile_interfaces/srv/MoveArmJoint "{joint_id: 1, angle_deg: 25.0, duration_ms: 1000, wait: true}"
ros2 service call /control/arm/move_joints tactile_interfaces/srv/MoveArmJoints "{joint_ids: [1,2,3], angles_deg: [15.0, 30.0, 20.0], duration_ms: 1200, wait: true}"
ros2 action send_goal /control/arm/move_joints_action tactile_interfaces/action/MoveArmJoints "{joint_ids: [1,2,3], angles_deg: [15.0, 30.0, 20.0], duration_ms: 1200, wait: true}" --feedback
ros2 service call /system/reset_emergency std_srvs/srv/Trigger "{}"
```

## Run phase 4 (GUI bridge)

Phase 4 keeps launch topology from phase 3 and upgrades `main_ros2.py`
from monitor mode to ROS2 control mode.

Terminal A:

```bash
ros2 launch tactile_bringup phase3_control.launch.py
```

Terminal B (repository root):

```bash
python main_ros2.py --control-mode ros2
```

Fallback mode:

```bash
python main_ros2.py --control-mode stub
```

## Run phase 5 (task orchestration + GUI)

Terminal A:

```bash
ros2 launch tactile_bringup phase5_task.launch.py
```

Hardware profile example:

```bash
ros2 launch tactile_bringup phase5_task.launch.py \
  param_file:=/home/zhuyiwei/programme/programme/ros2_ws/src/tactile_bringup/config/phase5_task_hardware.yaml
```

Terminal B (repository root):

```bash
python main_ros2.py --control-mode ros2 --log-level INFO
```

Terminal C (task/API quick check):

```bash
ros2 action list | grep /task/execute_demo
ros2 service list | grep /task/
ros2 service call /control/arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 action send_goal /task/execute_demo tactile_interfaces/action/ExecuteDemo "{demo_name: 'vector_visualization', params_json: '{\"duration\": 8}', duration_sec: 8.0}" --feedback
```

## Run phase 6.1 (vision kickoff)

Important:

- D455 image data comes through USB camera topics, not serial.
- Keep serial ports for STM32 only.

Terminal A:

```bash
ros2 launch tactile_bringup phase6_vision.launch.py
```

If your camera driver is already started elsewhere:

```bash
ros2 launch tactile_bringup phase6_vision.launch.py start_realsense:=false
```

Terminal B:

```bash
ros2 node list | grep -E "realsense2_camera|realsense_monitor_node"
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /system/health --once
```
