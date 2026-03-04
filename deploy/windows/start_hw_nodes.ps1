param(
    [string]$RosSetup = "C:\\opt\\ros\\jazzy\\x64\\local_setup.ps1",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [string]$ArmParamFile = "",
    [string]$RealsenseSerial = "_333422301846",
    [switch]$StartArm = $true,
    [switch]$StartRealsense = $true,
    [switch]$Execute = $false
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = (Resolve-Path (Join-Path $scriptDir "..\\..")).Path
$shellExe = "pwsh"
if (-not (Get-Command $shellExe -ErrorAction SilentlyContinue)) {
    $shellExe = "powershell"
}
$defaultWorkspaceSetup = Join-Path $projectRoot "ros2_ws\\install\\local_setup.bat"

if (-not $WorkspaceSetup -and (Test-Path $defaultWorkspaceSetup)) {
    $WorkspaceSetup = $defaultWorkspaceSetup
}

if (-not $ArmParamFile) {
    $defaultArmParam = Join-Path $projectRoot "ros2_ws\\src\\tactile_bringup\\config\\split_windows_hardware.yaml"
    if (Test-Path $defaultArmParam) {
        $ArmParamFile = $defaultArmParam
    }
}

. (Join-Path $scriptDir "env_ros2_windows.ps1") -RosSetup $RosSetup -WorkspaceSetup $WorkspaceSetup -DomainId $DomainId

function Test-RosPackage {
    param([string]$PackageName)
    & ros2 pkg prefix $PackageName 2>$null | Out-Null
    return ($LASTEXITCODE -eq 0)
}

$realsenseCmd = ""
$realsenseMode = ""

$armCmd = ""
if ($ArmParamFile) {
    $armCmd = "ros2 run tactile_hardware arm_driver_node --ros-args --params-file `"$ArmParamFile`""
} else {
    $armCmd = "ros2 run tactile_hardware arm_driver_node  # add --ros-args --params-file <path>"
}

$hasRealsensePkg = Test-RosPackage -PackageName "realsense2_camera"
$hasRealsenseFallbackPkg = Test-RosPackage -PackageName "tactile_vision"
$hasArmPkg = Test-RosPackage -PackageName "tactile_hardware"

if ($StartRealsense) {
    if ($hasRealsensePkg) {
        $realsenseMode = "realsense2_camera"
        $realsenseCmd = @(
            "ros2 run realsense2_camera realsense2_camera_node --ros-args",
            "-p serial_no:=$RealsenseSerial",
            "-p enable_color:=true",
            "-p enable_depth:=true",
            "-p align_depth.enable:=true",
            "-p rgb_camera.profile:=640x480x15",
            "-p depth_module.profile:=640x480x15"
        ) -join " "
    } elseif ($hasRealsenseFallbackPkg) {
        $realsenseMode = "tactile_vision.realsense_camera_node"
        $realsenseCmd = @(
            "ros2 run tactile_vision realsense_camera_node --ros-args",
            "-p serial_no:=$RealsenseSerial",
            "-p enable_color:=true",
            "-p enable_depth:=true",
            "-p align_depth.enable:=true",
            "-p color_width:=640",
            "-p color_height:=480",
            "-p color_fps:=15",
            "-p depth_width:=640",
            "-p depth_height:=480",
            "-p depth_fps:=15"
        ) -join " "
        Write-Warning "Package 'realsense2_camera' not found. Falling back to tactile_vision/realsense_camera_node."
    } else {
        Write-Warning "No RealSense publisher package found. Build ros2_ws (including tactile_vision) on Windows first."
        Write-Warning "Hint: cd ros2_ws; colcon build --merge-install --symlink-install --packages-select tactile_interfaces tactile_vision tactile_bringup"
        $StartRealsense = $false
    }
}

if ($StartArm -and -not $hasArmPkg) {
    Write-Warning "Package 'tactile_hardware' not found. Skipping arm node. Build ros2_ws on Windows first."
    if (-not $WorkspaceSetup) {
        Write-Warning "Hint: no WorkspaceSetup loaded. Build and source ros2_ws\\install\\local_setup.bat."
    }
    $StartArm = $false
}

Write-Host ""
Write-Host "Windows hardware node commands:"
if ($StartRealsense) {
    Write-Host "  [RealSense][$realsenseMode] $realsenseCmd"
}
if ($StartArm) {
    Write-Host "  [Arm]       $armCmd"
}
Write-Host ""

if (-not $Execute) {
    Write-Host "Dry-run mode. Re-run with -Execute to spawn new PowerShell windows."
    exit 0
}

if (-not $StartRealsense -and -not $StartArm) {
    Write-Error "No launchable hardware nodes found in current Windows ROS2 environment."
    exit 1
}

if ($StartRealsense) {
    $realsenseLaunch = "& { . `"$scriptDir\\env_ros2_windows.ps1`" -RosSetup `"$RosSetup`" -DomainId $DomainId"
    if ($WorkspaceSetup) {
        $realsenseLaunch += " -WorkspaceSetup `"$WorkspaceSetup`""
    }
    $realsenseLaunch += "; $realsenseCmd }"
    Start-Process $shellExe -ArgumentList "-NoExit", "-Command", $realsenseLaunch
}

if ($StartArm) {
    $armLaunch = "& { . `"$scriptDir\\env_ros2_windows.ps1`" -RosSetup `"$RosSetup`" -DomainId $DomainId"
    if ($WorkspaceSetup) {
        $armLaunch += " -WorkspaceSetup `"$WorkspaceSetup`""
    }
    $armLaunch += "; $armCmd }"
    Start-Process $shellExe -ArgumentList "-NoExit", "-Command", $armLaunch
}
