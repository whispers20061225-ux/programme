# Windows + VM One-Click Runbook

This runbook provides two one-click scripts for daily bringup:

1. Windows one-click hardware start + health check (RealSense)
2. VM one-click end-to-end debug startup (guard + launch + UI)

## 1) Windows: One-Click RealSense Ready

Script:

- `deploy/windows/start_realsense_ready.ps1`

What it does:

- loads ROS2 Windows environment
- starts RealSense node (`tactile_vision` fallback or `realsense2_camera` if present)
- waits for required topics
- samples color/depth fps and checks thresholds
- prints final `[READY]` status

Command (PowerShell):

```powershell
cd C:\Users\whisp\Desktop\...your-repo...\programme
. .\deploy\windows\start_realsense_ready.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -DomainId 0 -TopicTimeoutSec 20 -HzSampleSec 10 -MinColorHz 3.0 -MinDepthHz 3.0
```

Success example:

- `[OK] topics discovered`
- `[OK] RealSense READY: color=...Hz depth=...Hz`
- `[READY] You can now start VM one-click debug script.`

## 2) VM: One-Click Debug (Guard -> Launch -> UI)

Script:

- `deploy/vm/start_ui_with_realsense_guard.sh`

What it does:

- validates Windows->VM RealSense link using `deploy/vm/test_realsense_stream.sh`
- starts `split_vm_app.launch.py` in background
- starts `main_ros2.py --control-mode ros2`
- when UI exits, it stops launch process by default

Command (VM terminal):

```bash
cd /home/zhuyiwei/programme/programme
bash deploy/vm/start_ui_with_realsense_guard.sh 0 20 12 3.0 3.0 true dayiprogramme312
```

Parameters:

- `0`: ROS domain ID
- `20`: topic discovery timeout seconds
- `12`: hz sampling window seconds
- `3.0`: minimum color fps
- `3.0`: minimum depth fps
- `true`: `start_tactile_sensor` argument for VM launch
- `dayiprogramme312`: conda environment name for UI

Optional:

- keep launch alive after UI exits:

```bash
KEEP_LAUNCH=true bash deploy/vm/start_ui_with_realsense_guard.sh 0 20 12 3.0 3.0 true dayiprogramme312
```

## Troubleshooting

1. VM reports CycloneDDS RMW missing:
   - `sudo apt update && sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp`
2. Windows script reports missing `pyrealsense2`:
   - `python -m pip install pyrealsense2`
3. Windows build reports missing `pkg_resources`:
   - `python -m pip install "setuptools<81"`

