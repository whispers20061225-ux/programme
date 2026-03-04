# Windows + VM Split (Phase A)

## Goal

Keep `NAT` for internet access and add a `Host-only` path for stable ROS2 traffic between Windows host and Linux VM. This avoids dependency on bridge mode.

Phase A only covers network + middleware validation. No business logic changes.

## Target Topology

- Windows host:
  - Adapter 1: normal physical network
  - Adapter 2: VMware Host-only (example `192.168.56.1`)
- Linux VM:
  - Adapter 1: NAT
  - Adapter 2: Host-only (example `192.168.56.20`)

## Added Files

- `config/dds/cyclonedds_windows.xml`
- `config/dds/cyclonedds_vm.xml`
- `deploy/windows/env_ros2_windows.ps1`
- `deploy/windows/check_connectivity.ps1`
- `deploy/windows/start_hw_nodes.ps1`
- `deploy/vm/env_ros2_vm.sh`
- `deploy/vm/check_connectivity.sh`

## Setup Steps

1. VMware NIC setup:
   - Keep VM adapter 1 as NAT.
   - Set VM adapter 2 to Host-only (for example VMnet1).
2. On Windows, run `ipconfig` and record the Host-only IPv4.
3. On VM, run `ip a` and record the Host-only IPv4.
4. Update DDS peer files:
   - Windows file: set Windows address + VM peer address.
   - VM file: set VM address + Windows peer address.

## Windows Commands (PowerShell)

```powershell
cd <repo_root>

# 1) Load ROS2 + DDS env
. .\deploy\windows\env_ros2_windows.ps1 -DomainId 0 -RosSetup <your_ros2_setup.ps1_or_local_setup.bat>

# 2) Basic connectivity check (ping only)
. .\deploy\windows\check_connectivity.ps1 -VmIp 192.168.56.20 -RosSetup <your_ros2_setup.ps1_or_local_setup.bat>

# 3) DDS validation (Windows as talker)
. .\deploy\windows\check_connectivity.ps1 -VmIp 192.168.56.20 -RosSetup <your_ros2_setup.ps1_or_local_setup.bat> -RunTalker
```

## Linux VM Commands

```bash
cd /home/zhuyiwei/programme/programme

# 1) Load ROS2 + DDS env
source deploy/vm/env_ros2_vm.sh 0

# 2) Basic connectivity check (ping only)
bash deploy/vm/check_connectivity.sh 192.168.56.1

# 3) DDS validation (VM as listener)
bash deploy/vm/check_connectivity.sh 192.168.56.1 listener
```

For reverse-direction validation, switch talker/listener roles.

## Acceptance Criteria

- Windows and VM can ping each other over Host-only IPs.
- `demo_nodes_cpp talker/listener` works in both directions.
- Reopening terminals only requires re-running env scripts.

## Next Phase

- Move `arm_driver_node` to Windows host side.
- Keep VM side for `arm_control_node + task + ui_bridge`.
- Move D455 stream publishing to Windows host, VM subscribes only.
