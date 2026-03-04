# Phase 6.2.1 Kickoff (Gazebo + ros2_control)

This file tracks the first Gazebo integration step for phase 6.2.

## Implemented in this kickoff

- Extended `tactile_sim` package with Gazebo assets:
  - `urdf/phase6_arm.urdf.xacro`
  - `worlds/phase6_tabletop.world`
  - `config/ros2_controllers.yaml`
  - `launch/gazebo_arm.launch.py`
- Extended `arm_sim_driver_node` with backend switch:
  - `backend=memory` (default, phase6_sim_base)
  - `backend=ros2_control` (trajectory action + joint state feedback)
- Added Gazebo bringup entry:
  - `ros2_ws/src/tactile_bringup/launch/phase6_sim_gazebo.launch.py`
  - `ros2_ws/src/tactile_bringup/config/phase6_sim_gazebo.yaml`

## Compatibility intent

- Keep `/arm/*` service APIs unchanged.
- Keep `arm_control_node`, `demo_task_node`, and UI bridge unchanged.
- Preserve phase6 baseline launch (`phase6_sim_base.launch.py`) as fallback.

## Runtime verify commands

Terminal A:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch tactile_bringup phase6_sim_gazebo.launch.py
```

Terminal B:

```bash
python main_ros2.py --control-mode ros2 --log-level INFO
```

Terminal C:

```bash
ros2 topic list | grep -E "/joint_states|/arm/state|/tactile/raw|/system/health"
ros2 action list | grep /joint_trajectory_controller/follow_joint_trajectory
ros2 service call /control/arm/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/arm/move_joints tactile_interfaces/srv/MoveArmJoints "{joint_ids: [1,2,3], angles_deg: [10.0, 20.0, 15.0], duration_ms: 1200, wait: true}"
```

## Next step

- Add tactile contact coupling between Gazebo contacts and `tactile_sim_node` output.
- Add startup health gating to delay task/UI commands until controllers are ready.
