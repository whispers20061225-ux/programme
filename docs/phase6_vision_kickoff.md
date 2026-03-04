# Phase 6.1 Kickoff (Vision ROS2 Integration)

This file tracks phase 6.1 scope: wiring RealSense vision into ROS2 before Gazebo + MoveIt2 + SmolVLA.

## Implemented in this kickoff

- Added new package:
  - `ros2_ws/src/tactile_vision`
  - node: `realsense_monitor_node`
- `realsense_monitor_node` subscribes:
  - `/camera/camera/color/image_raw`
  - `/camera/camera/aligned_depth_to_color/image_raw`
  - `/camera/camera/color/camera_info`
- Health output:
  - publishes `tactile_interfaces/msg/SystemHealth` to `/system/health`
  - reports stale/ok state for color/depth/camera_info streams
- Added phase 6 vision bringup:
  - `ros2_ws/src/tactile_bringup/launch/phase6_vision.launch.py`
  - `ros2_ws/src/tactile_bringup/config/phase6_vision.yaml`

## Important hardware note

- RealSense D455 image data is USB camera stream (ROS2 image topics), not serial bytes.
- Serial port reading remains for STM32 control chain.

## Runtime verify commands

Terminal A:

```bash
cd /home/zhuyiwei/programme/programme/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tactile_vision tactile_bringup
source install/setup.bash
ros2 launch tactile_bringup phase6_vision.launch.py
```

If you already started `realsense2_camera` elsewhere, disable duplicated camera launch:

```bash
ros2 launch tactile_bringup phase6_vision.launch.py start_realsense:=false
```

Terminal B:

```bash
source /opt/ros/jazzy/setup.bash
source /home/zhuyiwei/programme/programme/ros2_ws/install/setup.bash
ros2 node list | grep -E "realsense2_camera|realsense_monitor_node"
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /system/health --once
```

## Next step

- Add vision stream adapter for task/policy input (`tactile_policy` shadow mode).
- Add timestamp sync checks for camera + arm state.
