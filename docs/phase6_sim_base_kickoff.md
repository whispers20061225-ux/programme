# Phase 6.2 Kickoff (Simulation Baseline)

This file tracks the initial phase 6.2 scope: creating a ROS2 simulation baseline chain before Gazebo + MoveIt2 integration.

## Implemented in this kickoff

- Added new package:
  - `ros2_ws/src/tactile_sim`
  - nodes:
    - `tactile_sim_node`
    - `arm_sim_driver_node`
- Added phase 6.2 baseline bringup:
  - `ros2_ws/src/tactile_bringup/launch/phase6_sim_base.launch.py`
  - `ros2_ws/src/tactile_bringup/config/phase6_sim_base.yaml`
- Launch chain now supports a pure ROS2 simulation hardware path:
  - `tactile_sim_node` -> `/tactile/raw`
  - `arm_sim_driver_node` -> `/arm/state` + `/arm/*` services
  - `arm_control_node` remains unchanged and proxies to `/arm/*`
  - `demo_task_node` and `tactile_ui_subscriber` remain unchanged

## Compatibility intent

- Keep control/task/ui interfaces unchanged for rollback safety.
- Existing phase 5 and phase 6.1 launch files are untouched.
- Legacy `main.py` remains unchanged.

## Runtime verify commands

Terminal A:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch tactile_bringup phase6_sim_base.launch.py
```

Terminal B:

```bash
python main_ros2.py --control-mode ros2 --log-level INFO
```

Terminal C:

```bash
ros2 topic list | grep -E "/tactile/raw|/arm/state|/system/health"
ros2 service list | grep -E "/arm/|/control/arm/"
ros2 service call /control/arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/arm/move_joint tactile_interfaces/srv/MoveArmJoint "{joint_id: 1, angle_deg: 25.0, duration_ms: 800, wait: true}"
```

## Next step

- Replace `arm_sim_driver_node` internal kinematics with Gazebo + ros2_control adapter while preserving `/arm/*` compatibility APIs.
