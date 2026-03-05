# Windows + VM Split (Phase B)

## Goal

Move hardware-facing nodes to Windows host and keep control/task/UI nodes on Linux VM.

Phase B focus:

- Windows host publishes hardware capabilities:
  - `arm_driver_node`
  - `realsense2_camera_node`
- Linux VM runs app/control chain:
  - `arm_control_node`
  - `demo_task_node`
  - `tactile_ui_subscriber`
  - `realsense_monitor_node`

This keeps USB/serial close to physical devices and keeps orchestration on VM.

## Added Files

- `ros2_ws/src/tactile_bringup/launch/split_vm_app.launch.py`
- `ros2_ws/src/tactile_bringup/config/split_vm_app.yaml`
- `ros2_ws/src/tactile_bringup/config/split_windows_hardware.yaml`
- `deploy/vm/start_split_vm_app.sh`

Updated:

- `deploy/windows/start_hw_nodes.ps1`:
  - default arm parameter file now points to `split_windows_hardware.yaml` when available
- `deploy/windows/env_ros2_windows.ps1`:
  - robust `CYCLONEDDS_URI` runtime file path for Windows

## Startup Order

1. Windows host: start hardware nodes
2. Linux VM: start app nodes
3. Linux VM: start GUI (`main_ros2.py`)

## Windows Prerequisites

Before launching Windows hardware nodes, make sure project ROS2 packages are built on Windows:

```powershell
cd C:\Users\whisp\Desktop\大一年度项目\programme
. .\deploy\windows\env_ros2_windows.ps1 -DomainId 0 -RosSetup C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat

cd .\ros2_ws
colcon build --merge-install --symlink-install --packages-select tactile_interfaces tactile_hardware tactile_bringup
```

After build, `ros2_ws\install\local_setup.bat` will be auto-detected by `start_hw_nodes.ps1`.

If you hit `VisualStudioVersion is not set`, use the helper script instead:

```powershell
cd C:\Users\whisp\Desktop\大一年度项目\programme
. .\deploy\windows\build_ws_minimal.ps1 -RosSetup C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat -DomainId 0
```

This script tries to auto-load MSVC build environment and gives an install hint if VC++ tools are missing.

## Windows Host Commands

```powershell
cd C:\Users\whisp\Desktop\大一年度项目\programme

. .\deploy\windows\env_ros2_windows.ps1 -DomainId 0 -RosSetup C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat

# Dry-run: print commands only
. .\deploy\windows\start_hw_nodes.ps1 -DomainId 0 -RosSetup C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat

# Execute: spawn hardware node windows
. .\deploy\windows\start_hw_nodes.ps1 -DomainId 0 -RosSetup C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat -Execute
```

RealSense-only mode (recommended when your current focus is camera stream only):

```powershell
cd C:\Users\whisp\Desktop\大一年度项目\programme
. .\deploy\windows\start_realsense_only.ps1 -DomainId 0 -RosSetup C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat -Execute
```

`start_hw_nodes.ps1` will automatically use:

- `realsense2_camera` package if available
- otherwise fallback to `tactile_vision/realsense_camera_node` (pyrealsense2-based)

Fallback options:

- If `realsense2_camera` package is missing on Windows, run without camera:

```powershell
. .\deploy\windows\start_hw_nodes.ps1 -DomainId 0 -RosSetup C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat -StartRealsense:$false -Execute
```

- If `tactile_hardware` package is missing, build Windows workspace first (see prerequisites section).

## Linux VM Commands

```bash
cd /home/zhuyiwei/programme/programme
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Domain ID 0, tactile sensor simulation enabled by default
bash deploy/vm/start_split_vm_app.sh 0
```

If you need to disable VM tactile simulation:

```bash
bash deploy/vm/start_split_vm_app.sh 0 "" false
```

## GUI Command (Linux VM)

```bash
cd /home/zhuyiwei/programme/programme
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dayiprogramme312
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python main_ros2.py --control-mode ros2 --log-level INFO
```

## Phase B Acceptance (Positive Checks)

1. Windows:
   - `ros2 node list` includes `arm_driver_node` and `realsense2_camera`
2. VM:
   - `ros2 node list` includes `arm_control_node`, `demo_task_node`, `realsense_monitor_node`
3. VM:
   - `ros2 topic hz /arm/state` has non-zero rate
4. VM:
   - `ros2 topic hz /camera/camera/color/image_raw` has non-zero rate
5. VM:
   - `ros2 service call /control/arm/enable std_srvs/srv/SetBool "{data: true}"` returns success in connected hardware state

## Next Step

After Phase B acceptance:

- lock down split deployment scripts
- start cross-machine task regression
- then enter 6b (Gazebo + MoveIt2 + SmolVLA shadow-policy)
