# Phase 5 Kickoff (Task Orchestration)

This file tracks the phase 5 migration scope: moving demo orchestration to ROS2 Action/Services.

## Implemented in this kickoff

- Added task action contract:
  - `ros2_ws/src/tactile_interfaces/action/ExecuteDemo.action`
- Registered interface generation in:
  - `ros2_ws/src/tactile_interfaces/CMakeLists.txt`
- Added new package:
  - `ros2_ws/src/tactile_task`
  - node: `demo_task_node`
- `demo_task_node` provides:
  - Action: `/task/execute_demo` (`tactile_interfaces/action/ExecuteDemo`)
  - Service: `/task/pause_demo` (`std_srvs/srv/Trigger`)
  - Service: `/task/resume_demo` (`std_srvs/srv/Trigger`)
  - Service: `/task/stop_demo` (`std_srvs/srv/Trigger`)
- Added phase 5 bringup:
  - `ros2_ws/src/tactile_bringup/launch/phase5_task.launch.py`
  - `ros2_ws/src/tactile_bringup/config/phase5_task.yaml`
- UI bridge migration step:
  - `src/core/ros2_runtime_stubs.py` now routes:
    - `start_demo` -> `/task/execute_demo` action goal
    - `pause_demo` -> `/task/pause_demo`
    - `resume_demo` -> `/task/resume_demo`
    - `stop_demo` -> `/task/stop_demo`
  - Demo feedback/result is re-emitted to existing GUI status channel (`demo_starting/demo_started/demo_progress/demo_completed/demo_stopped/demo_failed`).

## Runtime verify commands

Terminal A:

```bash
cd /home/zhuyiwei/programme/programme
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dayiprogramme312
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch tactile_bringup phase5_task.launch.py
```

Terminal B:

```bash
cd /home/zhuyiwei/programme/programme
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dayiprogramme312
source /opt/ros/jazzy/setup.bash
source /home/zhuyiwei/programme/programme/ros2_ws/install/setup.bash
python main_ros2.py --control-mode ros2 --log-level INFO
```

Terminal C (CLI action/service check):

```bash
cd /home/zhuyiwei/programme/programme
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dayiprogramme312
source /opt/ros/jazzy/setup.bash
source /home/zhuyiwei/programme/programme/ros2_ws/install/setup.bash
ros2 action list | grep /task/execute_demo
ros2 service list | grep /task/
ros2 action send_goal /task/execute_demo tactile_interfaces/action/ExecuteDemo "{demo_name: 'vector_visualization', params_json: '{\"duration\": 8}', duration_sec: 8.0}" --feedback
```

## Notes

- Current orchestration is intentionally conservative:
  - generic demos run as timed task loops;
  - grasp-style demos can optionally trigger open/close joint sequences through `/control/arm/move_joints`.
- Legacy `main.py` remains unchanged for rollback safety.
- SmolVLA (or other VLA backend) remains phase 6 scope.
