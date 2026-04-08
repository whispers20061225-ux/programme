param(
    [string]$Distro = "Ubuntu-24.04",
    [switch]$OpenGazeboGui,
    [switch]$OpenRviz
)

$ErrorActionPreference = "Stop"

$runtimeDir = Join-Path $env:LOCALAPPDATA "ProgrammeWebUI"
New-Item -ItemType Directory -Path $runtimeDir -Force | Out-Null

$wsRoot = "/home/whispers/programme/ros2_ws"
$gazeboLog = "$wsRoot/.runtime/programme-ui/debug-gazebo-gui.log"
$rvizLog = "$wsRoot/.runtime/programme-ui/debug-rviz.log"
$rvizConfig = "$wsRoot/install/tactile_moveit_config/share/tactile_moveit_config/config/moveit.rviz"
$wslgExports = @(
    "export DISPLAY=:0",
    "export WAYLAND_DISPLAY=wayland-0",
    "export XDG_RUNTIME_DIR=/run/user/1000",
    "export PULSE_SERVER=unix:/mnt/wslg/PulseServer",
    "export WSL2_GUI_APPS_ENABLED=1",
    "export LIBGL_ALWAYS_SOFTWARE=0",
    "export GALLIUM_DRIVER=d3d12",
    "export MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA'",
    "export QT_X11_NO_MITSHM=1"
) -join " && "

function Test-DebugProc {
    param([Parameter(Mandatory = $true)][string]$Token)

    $matches = Get-CimInstance Win32_Process |
        Where-Object {
            $_.Name -eq "wsl.exe" -and
            $_.CommandLine -like "*$Token*" -and
            $_.CommandLine -notlike "*grep*"
        }
    return @($matches).Count -gt 0
}

function New-DebugLauncherScript {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Command
    )

    $scriptPath = Join-Path $runtimeDir ("debug-" + $Name + ".cmd")
    $content = @"
@echo off
title Tactile Grasp Studio Debug - $Name
wsl.exe -d $Distro -- bash -lc "$Command"
echo.
echo [debug] $Name exited with code %ERRORLEVEL%
pause
"@
    Set-Content -Path $scriptPath -Value $content -Encoding ASCII
    return $scriptPath
}

function Start-DebugWslProcess {
    param(
        [Parameter(Mandatory = $true)][string]$Token,
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Command
    )

    if (Test-DebugProc -Token $Token) {
        return "already running"
    }

    $launcher = New-DebugLauncherScript -Name $Name -Command $Command
    Start-Process -FilePath "cmd.exe" `
        -ArgumentList @("/k", $launcher) `
        -WindowStyle Normal | Out-Null
    return "launched"
}

$actions = @()

if ($OpenGazeboGui) {
    $gazeboCmd = "$wslgExports && source /opt/ros/jazzy/setup.bash && source $wsRoot/install/setup.bash && mkdir -p $wsRoot/.runtime/programme-ui && gz sim -g --force-version 8 2>&1 | tee -a $gazeboLog"
    $actions += "gazebo gui: $(Start-DebugWslProcess -Token 'gz sim -g --force-version 8' -Name 'gazebo-gui' -Command $gazeboCmd)"
}

if ($OpenRviz) {
    $rvizCmd = "$wslgExports && source /opt/ros/jazzy/setup.bash && source $wsRoot/install/setup.bash && mkdir -p $wsRoot/.runtime/programme-ui && rviz2 -d $rvizConfig 2>&1 | tee -a $rvizLog"
    $actions += "rviz: $(Start-DebugWslProcess -Token 'rviz2 -d' -Name 'rviz' -Command $rvizCmd)"
}

if ($actions.Count -eq 0) {
    $actions += "debug views disabled"
}

Write-Output ($actions -join " | ")
