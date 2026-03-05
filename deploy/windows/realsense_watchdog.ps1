param(
    [string]$RosSetup = "C:\\pixi_ws\\ros2-windows\\ros2-windows\\local_setup.bat",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [string]$RealsenseSerial = "",
    [switch]$UseRealsense2Camera = $false,
    [int]$TopicTimeoutSec = 20,
    [int]$CheckIntervalSec = 20,
    [int]$HzSampleSec = 8,
    [double]$MinColorHz = 3.0,
    [double]$MinDepthHz = 3.0,
    [int]$MaxConsecutiveFailures = 2,
    [int]$RestartBackoffSec = 3
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

function Write-Step([string]$msg) { Write-Host "[STEP] $msg" }
function Write-Ok([string]$msg) { Write-Host "[OK] $msg" -ForegroundColor Green }
function Write-WarnMsg([string]$msg) { Write-Host "[WARN] $msg" -ForegroundColor Yellow }
function Write-Fail([string]$msg) { Write-Host "[FAIL] $msg" -ForegroundColor Red }

function Wait-Topic {
    param(
        [string]$TopicName,
        [int]$TimeoutSec
    )

    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        $topics = & ros2 topic list 2>$null
        if ($LASTEXITCODE -eq 0 -and ($topics -contains $TopicName)) {
            return $true
        }
        Start-Sleep -Milliseconds 500
    }
    return $false
}

function Get-TopicAverageHz {
    param(
        [string]$TopicName,
        [int]$SampleSec
    )

    $outFile = Join-Path $env:TEMP ("programme_hz_" + [guid]::NewGuid().ToString() + ".out.log")
    $errFile = Join-Path $env:TEMP ("programme_hz_" + [guid]::NewGuid().ToString() + ".err.log")
    try {
        $proc = Start-Process -FilePath ros2 -ArgumentList @("topic", "hz", $TopicName) -PassThru -RedirectStandardOutput $outFile -RedirectStandardError $errFile -WindowStyle Hidden
        Start-Sleep -Seconds $SampleSec
        if (-not $proc.HasExited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        }

        $raw = ""
        if (Test-Path $outFile) { $raw += (Get-Content -Raw $outFile) + "`n" }
        if (Test-Path $errFile) { $raw += (Get-Content -Raw $errFile) + "`n" }
        $matches = [regex]::Matches($raw, "average rate:\s*([0-9]+(?:\.[0-9]+)?)")
        if ($matches.Count -eq 0) {
            return $null
        }
        return [double]$matches[$matches.Count - 1].Groups[1].Value
    }
    finally {
        Remove-Item $outFile, $errFile -Force -ErrorAction SilentlyContinue
    }
}

function New-RealsenseNodeArgs {
    if ($UseRealsense2Camera) {
        $args = @(
            "run",
            "realsense2_camera",
            "realsense2_camera_node",
            "--ros-args",
            "-p", "enable_color:=true",
            "-p", "enable_depth:=true",
            "-p", "align_depth.enable:=true",
            "-p", "rgb_camera.profile:=640x480x15",
            "-p", "depth_module.profile:=640x480x15"
        )
        if ($RealsenseSerial) {
            $args += @("-p", "serial_no:=$RealsenseSerial")
        }
        return $args
    }

    $args = @(
        "run",
        "tactile_vision",
        "realsense_camera_node",
        "--ros-args",
        "-p", "enable_color:=true",
        "-p", "enable_depth:=true",
        "-p", "align_depth.enable:=true",
        "-p", "color_width:=640",
        "-p", "color_height:=480",
        "-p", "color_fps:=15",
        "-p", "depth_width:=640",
        "-p", "depth_height:=480",
        "-p", "depth_fps:=15",
        "-p", "frame_timeout_ms:=350",
        "-p", "max_consecutive_timeouts:=12",
        "-p", "restart_cooldown_sec:=3.0",
        "-p", "capture_stale_sec:=2.0",
        "-p", "publish_only_when_new_frame:=true"
    )
    if ($RealsenseSerial) {
        $args += @("-p", "serial_no:=$RealsenseSerial")
    }
    return $args
}

function Start-RealsenseNodeProcess {
    param([string[]]$NodeArgs)
    return Start-Process -FilePath ros2 -ArgumentList $NodeArgs -PassThru -WindowStyle Hidden
}

function Stop-RealsenseNodeProcess {
    param([System.Diagnostics.Process]$Process)
    if ($null -eq $Process) {
        return
    }
    try {
        if (-not $Process.HasExited) {
            Stop-Process -Id $Process.Id -Force -ErrorAction SilentlyContinue
        }
    }
    catch {
        # ignore
    }
}

Write-Step "loading ROS2 Windows environment"
. (Join-Path $scriptDir "env_ros2_windows.ps1") -RosSetup $RosSetup -WorkspaceSetup $WorkspaceSetup -DomainId $DomainId -WarmupRosGraph $true

if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    Write-Fail "ros2 command unavailable after environment setup"
    exit 1
}

$mode = if ($UseRealsense2Camera) { "realsense2_camera_node" } else { "tactile_vision/realsense_camera_node" }
$nodeArgs = New-RealsenseNodeArgs
$nodeProc = $null
$consecutiveFailures = 0
$colorTopic = "/camera/camera/color/image_raw"
$depthTopic = "/camera/camera/aligned_depth_to_color/image_raw"
$infoTopic = "/camera/camera/color/camera_info"

try {
    Write-Step "starting watchdog for $mode"
    $nodeProc = Start-RealsenseNodeProcess -NodeArgs $nodeArgs
    Write-Ok "node started (pid=$($nodeProc.Id))"

    if (-not (Wait-Topic -TopicName $colorTopic -TimeoutSec $TopicTimeoutSec)) {
        Write-WarnMsg "color topic not discovered within ${TopicTimeoutSec}s"
    }
    if (-not (Wait-Topic -TopicName $depthTopic -TimeoutSec $TopicTimeoutSec)) {
        Write-WarnMsg "depth topic not discovered within ${TopicTimeoutSec}s"
    }
    if (-not (Wait-Topic -TopicName $infoTopic -TimeoutSec $TopicTimeoutSec)) {
        Write-WarnMsg "camera_info topic not discovered within ${TopicTimeoutSec}s"
    }

    while ($true) {
        if ($nodeProc.HasExited) {
            Write-WarnMsg "node process exited with code $($nodeProc.ExitCode)"
            $consecutiveFailures += 1
        } else {
            $colorHz = Get-TopicAverageHz -TopicName $colorTopic -SampleSec $HzSampleSec
            $depthHz = Get-TopicAverageHz -TopicName $depthTopic -SampleSec $HzSampleSec

            $sampleOk = $true
            if ($null -eq $colorHz -or $colorHz -lt $MinColorHz) {
                $sampleOk = $false
            }
            if ($null -eq $depthHz -or $depthHz -lt $MinDepthHz) {
                $sampleOk = $false
            }

            if ($sampleOk) {
                $consecutiveFailures = 0
                Write-Ok ("sample healthy: color={0:N3}Hz depth={1:N3}Hz" -f $colorHz, $depthHz)
            } else {
                $consecutiveFailures += 1
                Write-WarnMsg ("sample degraded: color={0} depth={1} fail_count={2}/{3}" -f `
                    ($(if ($null -eq $colorHz) { "n/a" } else { "{0:N3}Hz" -f $colorHz })), `
                    ($(if ($null -eq $depthHz) { "n/a" } else { "{0:N3}Hz" -f $depthHz })), `
                    $consecutiveFailures, $MaxConsecutiveFailures)
            }
        }

        if ($consecutiveFailures -ge $MaxConsecutiveFailures) {
            Write-WarnMsg "failure threshold reached, restarting RealSense node"
            Stop-RealsenseNodeProcess -Process $nodeProc
            Start-Sleep -Seconds $RestartBackoffSec
            $nodeProc = Start-RealsenseNodeProcess -NodeArgs $nodeArgs
            Write-Ok "node restarted (pid=$($nodeProc.Id))"
            $consecutiveFailures = 0
        }

        Start-Sleep -Seconds $CheckIntervalSec
    }
}
finally {
    Stop-RealsenseNodeProcess -Process $nodeProc
}
