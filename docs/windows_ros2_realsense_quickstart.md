# Windows ROS2 RealSense Node Quickstart

This guide is the minimal command flow to start the ROS2 RealSense node on Windows.

## Scope

- Platform: Windows host
- ROS2: Jazzy (pixi workspace layout)
- Goal: publish camera topics from Windows (`/camera/camera/*`)

## Prerequisites

1. RealSense camera is physically connected to Windows host.
2. ROS2 setup script exists:
   - `C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat`
3. Set project root once in each new terminal:
   - `$PROJECT_ROOT = "C:\Users\whisp\Desktop\...your-repo...\programme"`

Example:

```powershell
$PROJECT_ROOT = "C:\Users\whisp\Desktop\...your-repo...\programme"
```

## Terminal 1: Build minimal workspace

```powershell
cd $PROJECT_ROOT
. .\deploy\windows\build_ws_minimal.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -DomainId 0 -Clean
```

Expected result:

- `Summary: 2 packages finished`
- `Build finished successfully.`

## Terminal 2: Start RealSense node (Windows)

```powershell
cd $PROJECT_ROOT
. .\deploy\windows\start_realsense_only.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -DomainId 0 -Execute
```

Expected result:

- New PowerShell window opens and runs:
  - `ros2 run tactile_vision realsense_camera_node ...`
- Node startup log shows:
  - `realsense_camera_node started: serial=auto ...`

Note:

- Current default uses `realsense_watchdog.ps1` under the hood.
- Watchdog automatically restarts RealSense node when consecutive sampling windows are below threshold.

## Optional: Run watchdog directly in foreground

```powershell
cd $PROJECT_ROOT
. .\deploy\windows\realsense_watchdog.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -WorkspaceSetup ".\ros2_ws\install\local_setup.ps1" -DomainId 0 -CheckIntervalSec 20 -HzSampleSec 8 -MinColorHz 3.0 -MinDepthHz 3.0
```

This is the recommended long-running mode during VM + UI debugging.

## Terminal 3: Verify topics on Windows

```powershell
cd $PROJECT_ROOT
. .\deploy\windows\env_ros2_windows.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -WorkspaceSetup ".\ros2_ws\install\local_setup.ps1" -DomainId 0
ros2 topic list | Select-String "camera/camera"
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

Expected result:

- camera topics are listed
- `ros2 topic hz` shows non-zero rate

## Optional: Verify device visibility directly

Use this if node reports `No device connected`.

```powershell
cd $PROJECT_ROOT
. .\deploy\windows\env_ros2_windows.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -WorkspaceSetup ".\ros2_ws\install\local_setup.ps1" -DomainId 0
python -c "import pyrealsense2 as rs; devs=list(rs.context().query_devices()); print('count=',len(devs)); [print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in devs]"
```

Interpretation:

- `count > 0`: device is visible to Windows process
- `count = 0`: camera is not visible to current process (USB ownership/cable/driver issue)

## Troubleshooting

1. `ModuleNotFoundError: No module named 'pyrealsense2'`
   - Run:
   ```powershell
   python -m pip install pyrealsense2
   ```

2. Build error contains `No module named 'pkg_resources'`
   - Run:
   ```powershell
   python -m pip install "setuptools<81"
   ```
   - Re-run `build_ws_minimal.ps1`.

3. `realsense2_camera` package not found
   - This is acceptable in current flow.
   - Script auto-falls back to:
     - `tactile_vision/realsense_camera_node`

## VM One-Command Validation

After Windows node is running, validate from VM:

```bash
cd /home/zhuyiwei/programme/programme
bash deploy/vm/test_realsense_stream.sh 0 20 12 3.0 3.0
```

Parameters:

- `0`: `ROS_DOMAIN_ID`
- `20`: topic discovery timeout (seconds)
- `12`: hz sampling duration (seconds)
- `3.0`: minimum acceptable color fps
- `3.0`: minimum acceptable depth fps

## VM Build Requirement (C++ Relay)

`split_vm_app.launch.py` now includes a C++ latest-frame relay node (`tactile_vision_cpp/latest_frame_relay_node`).
Before running VM one-click UI script, build VM workspace at least once:

```bash
cd /home/zhuyiwei/programme/programme/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tactile_interfaces tactile_vision tactile_vision_cpp tactile_bringup
source install/setup.bash
```

Relay topics used by UI path:

- `/camera/relay/color/image_raw`
- `/camera/relay/aligned_depth_to_color/image_raw`
- `/camera/relay/color/camera_info`

## Simultaneous Windows-vs-VM Hz Compare

Use this to compare Windows local publish rate and VM receive rate at the same moment.
Keep the Windows RealSense publisher terminal running during the whole test.

1. On Windows, create a shared start epoch (UTC seconds):

```powershell
$START_EPOCH = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds() + 15
$START_EPOCH
```

2. On Windows (publisher side):

```powershell
cd $PROJECT_ROOT
. .\deploy\windows\sample_realsense_hz.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -WorkspaceSetup ".\ros2_ws\install\local_setup.ps1" -DomainId 0 -SampleSec 12 -StartEpochSec $START_EPOCH
```

3. On VM (receiver side), use the same `START_EPOCH` number:

```bash
cd /home/zhuyiwei/programme/programme
START_EPOCH=1700000000   # replace with the number printed on Windows
bash deploy/vm/sample_realsense_hz.sh 0 12 "${START_EPOCH}" 20
```

Expected output format:

- Windows: `[RESULT][windows] ... color_hz=... depth_hz=...`
- VM: `[RESULT][vm] ... color_hz=... depth_hz=...`

Interpretation:

- If Windows high + VM low: cross-machine DDS/network path is the bottleneck.
- If both low: Windows publisher/camera pipeline is the bottleneck.

## 15-Minute Soak Test (Recommended)

Use this for your current stability check target (15 minutes).

1. Keep Windows RealSense publisher running.

2. Windows side soak:

```powershell
cd $PROJECT_ROOT
. .\deploy\windows\soak_realsense_hz.ps1 -RosSetup "C:\pixi_ws\ros2-windows\ros2-windows\local_setup.bat" -WorkspaceSetup ".\ros2_ws\install\local_setup.ps1" -DomainId 0 -DurationMin 15 -IntervalSec 60 -SampleSec 8
```

3. VM side soak:

```bash
cd /home/zhuyiwei/programme/programme
bash deploy/vm/soak_realsense_hz.sh 0 15 60 8 30
```

Outputs:

- Windows CSV: `ros2_ws/log/soak/windows_soak_*.csv`
- VM CSV: `ros2_ws/log/soak/vm_soak_*.csv`
- Both scripts print summary lines:
  - average/min/max fps
  - `<1Hz` sample ratio
