# ROS2 Workspace (Phase 1)

This workspace contains the initial ROS2 migration skeleton.

## Scope in Phase 1

- Define core interfaces (`tactile_interfaces`)
- Publish fake tactile data
- Subscribe through a UI bridge node
- Launch a minimal end-to-end chain

## Workspace layout

```text
ros2_ws/
  src/
    tactile_interfaces/
    tactile_bringup/
    tactile_ui_bridge/
```

## Build and run (Ubuntu + ROS2 Jazzy)

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch tactile_bringup phase1_fake_chain.launch.py

# in another terminal at repository root
python main_ros2.py --tactile-topic /tactile/raw --health-topic /system/health
```

Expected topics:

- `/tactile/raw`
- `/system/health`
