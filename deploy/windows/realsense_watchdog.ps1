param(
    [string]$RosSetup = "C:\\pixi_ws\\ros2-windows\\ros2-windows\\local_setup.bat",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [string]$RealsenseSerial = "",
    [int]$ColorWidth = 640,
    [int]$ColorHeight = 480,
    [int]$ColorFps = 30,
    [int]$DepthWidth = 640,
    [int]$DepthHeight = 480,
    [int]$DepthFps = 30,
    [bool]$AlignDepth = $true,
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

function Get-PreferredShell {
    if (Get-Command pwsh -ErrorAction SilentlyContinue) {
        return "pwsh"
    }
    return "powershell"
}

function Read-TextSafe {
    param([string]$Path)

    if (-not (Test-Path $Path)) {
        return ""
    }
    $contentObj = Get-Content -Raw -Path $Path -ErrorAction SilentlyContinue
    if ($null -eq $contentObj) {
        return ""
    }
    return ([string]$contentObj).Trim()
}

function New-Ros2CommandExpression {
    param([string[]]$Args)

    $quotedArgs = @(
        $Args | ForEach-Object { "'" + ([string]$_ -replace "'", "''") + "'" }
    ) -join ", "
    return "& { & ros2 @($quotedArgs) }"
}

function Start-Ros2CommandProcess {
    param(
        [string[]]$Args,
        [string]$StdoutPath,
        [string]$StderrPath
    )

    $shellExe = Get-PreferredShell
    $command = New-Ros2CommandExpression -Args $Args
    return Start-Process -FilePath $shellExe `
        -ArgumentList @("-NoLogo", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", $command) `
        -PassThru `
        -RedirectStandardOutput $StdoutPath `
        -RedirectStandardError $StderrPath `
        -WindowStyle Hidden
}

function Invoke-Ros2CommandCapture {
    param(
        [string[]]$Args,
        [int]$TimeoutSec
    )

    $outFile = Join-Path $env:TEMP ("programme_ros2_" + [guid]::NewGuid().ToString() + ".out.log")
    $errFile = Join-Path $env:TEMP ("programme_ros2_" + [guid]::NewGuid().ToString() + ".err.log")
    try {
        $proc = Start-Ros2CommandProcess -Args $Args -StdoutPath $outFile -StderrPath $errFile
        $exited = $proc.WaitForExit($TimeoutSec * 1000)
        if (-not $exited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        }

        return [PSCustomObject]@{
            Exited   = $exited
            ExitCode = if ($exited) { $proc.ExitCode } else { $null }
            Stdout   = Read-TextSafe -Path $outFile
            Stderr   = Read-TextSafe -Path $errFile
        }
    }
    catch {
        return [PSCustomObject]@{
            Exited   = $false
            ExitCode = $null
            Stdout   = ""
            Stderr   = ""
        }
    }
    finally {
        Remove-Item $outFile, $errFile -Force -ErrorAction SilentlyContinue
    }
}

function Get-TopicListSafe {
    param(
        [int]$CommandTimeoutSec = 4
    )

    $timeoutSec = [Math]::Max(1, $CommandTimeoutSec)
    $result = Invoke-Ros2CommandCapture -Args @("topic", "list") -TimeoutSec $timeoutSec
    if ((-not $result.Exited) -or ($result.ExitCode -ne 0) -or (-not $result.Stdout)) {
        return @()
    }

    return @(
        $result.Stdout -split "`r?`n" |
            ForEach-Object { ([string]$_).Trim() } |
            Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
    )
}

function Test-TopicMatch {
    param(
        [string[]]$Topics,
        [string]$TopicName
    )

    $topicAlt = $TopicName.TrimStart('/')
    $topicWithSlash = if ($topicAlt) { "/$topicAlt" } else { $TopicName }
    return (($Topics -contains $TopicName) -or ($Topics -contains $topicAlt) -or ($Topics -contains $topicWithSlash))
}

function Wait-Topic {
    param(
        [string]$TopicName,
        [int]$TimeoutSec
    )

    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        $remainingSec = [int][Math]::Ceiling(($deadline - (Get-Date)).TotalSeconds)
        if ($remainingSec -le 0) {
            break
        }
        $commandTimeoutSec = [Math]::Max(1, [Math]::Min(4, $remainingSec))
        $topics = Get-TopicListSafe -CommandTimeoutSec $commandTimeoutSec
        if (Test-TopicMatch -Topics $topics -TopicName $TopicName) {
            return $true
        }
        Start-Sleep -Milliseconds 500
    }

    $finalTopics = Get-TopicListSafe -CommandTimeoutSec ([Math]::Max(1, [Math]::Min(4, $TimeoutSec)))
    return (Test-TopicMatch -Topics $finalTopics -TopicName $TopicName)
}

function Wait-MessageOnce {
    param(
        [string]$TopicName,
        [int]$TimeoutSec
    )

    $outFile = Join-Path $env:TEMP ("programme_echo_" + [guid]::NewGuid().ToString() + ".out.log")
    $errFile = Join-Path $env:TEMP ("programme_echo_" + [guid]::NewGuid().ToString() + ".err.log")
    try {
        $proc = Start-Ros2CommandProcess -Args @("topic", "echo", $TopicName, "--once") -StdoutPath $outFile -StderrPath $errFile
        $exited = $proc.WaitForExit($TimeoutSec * 1000)
        if (-not $exited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
            return $false
        }
        return ($proc.ExitCode -eq 0)
    }
    finally {
        Remove-Item $outFile, $errFile -Force -ErrorAction SilentlyContinue
    }
}
function Get-TopicAverageHz {
    param(
        [string]$TopicName,
        [int]$SampleSec
    )

    $outFile = Join-Path $env:TEMP ("programme_hz_" + [guid]::NewGuid().ToString() + ".out.log")
    $errFile = Join-Path $env:TEMP ("programme_hz_" + [guid]::NewGuid().ToString() + ".err.log")
    try {
        $proc = Start-Ros2CommandProcess -Args @("topic", "hz", $TopicName) -StdoutPath $outFile -StderrPath $errFile
        Start-Sleep -Seconds $SampleSec
        if (-not $proc.HasExited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        }
        $proc.WaitForExit(2000) | Out-Null

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
    $alignDepthValue = $AlignDepth.ToString().ToLower()

    if ($UseRealsense2Camera) {
        $args = @(
            "run",
            "realsense2_camera",
            "realsense2_camera_node",
            "--ros-args",
            "-p", "enable_color:=true",
            "-p", "enable_depth:=true",
            "-p", "align_depth.enable:=$alignDepthValue",
            "-p", "rgb_camera.profile:=${ColorWidth}x${ColorHeight}x${ColorFps}",
            "-p", "depth_module.profile:=${DepthWidth}x${DepthHeight}x${DepthFps}"
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
        "-p", "align_depth.enable:=$alignDepthValue",
        "-p", "color_width:=$ColorWidth",
        "-p", "color_height:=$ColorHeight",
        "-p", "color_fps:=$ColorFps",
        "-p", "depth_width:=$DepthWidth",
        "-p", "depth_height:=$DepthHeight",
        "-p", "depth_fps:=$DepthFps",
        "-p", "frame_timeout_ms:=300",
        "-p", "max_consecutive_timeouts:=8",
        "-p", "restart_cooldown_sec:=1.5",
        "-p", "capture_stale_sec:=1.5",
        "-p", "publish_only_when_new_frame:=true",
        "-p", "use_reliable_qos:=true"
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
$monitorMode = "warmup"
$healthySamples = 0
$lightweightTimeoutSec = [Math]::Max(3, [Math]::Min(6, $TopicTimeoutSec))
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
            $monitorMode = "degraded"
            $healthySamples = 0
            $consecutiveFailures += 1
        } else {
            if ($monitorMode -eq "steady") {
                $consecutiveFailures = 0
            } else {
                $colorHz = Get-TopicAverageHz -TopicName $colorTopic -SampleSec $HzSampleSec
                $depthHz = Get-TopicAverageHz -TopicName $depthTopic -SampleSec $HzSampleSec

                $sampleOk = $true
                $sampleInconclusive = $false
                if ($null -eq $colorHz) {
                    $sampleInconclusive = $true
                } elseif ($colorHz -lt $MinColorHz) {
                    $sampleOk = $false
                }
                if ($null -eq $depthHz) {
                    $sampleInconclusive = $true
                } elseif ($depthHz -lt $MinDepthHz) {
                    $sampleOk = $false
                }

                if ($sampleOk -and -not $sampleInconclusive) {
                    $consecutiveFailures = 0
                    $healthySamples += 1
                    Write-Ok ("sample healthy: color={0:N3}Hz depth={1:N3}Hz" -f $colorHz, $depthHz)
                    if ($healthySamples -ge 1) {
                        $monitorMode = "steady"
                        Write-Ok "watchdog switched to steady lightweight mode"
                    }
                } elseif ($sampleInconclusive) {
                    $consecutiveFailures = 0
                    $healthySamples = 0
                    Write-WarnMsg ("sample inconclusive: color={0} depth={1}; keeping degraded mode" -f
                        ($(if ($null -eq $colorHz) { "n/a" } else { "{0:N3}Hz" -f $colorHz })),
                        ($(if ($null -eq $depthHz) { "n/a" } else { "{0:N3}Hz" -f $depthHz })))
                } else {
                    $healthySamples = 0
                    $consecutiveFailures += 1
                    Write-WarnMsg ("sample degraded: color={0} depth={1} fail_count={2}/{3}" -f
                        ($(if ($null -eq $colorHz) { "n/a" } else { "{0:N3}Hz" -f $colorHz })),
                        ($(if ($null -eq $depthHz) { "n/a" } else { "{0:N3}Hz" -f $depthHz })),
                        $consecutiveFailures, $MaxConsecutiveFailures)
                }
            }
        }

        if ($consecutiveFailures -ge $MaxConsecutiveFailures) {
            Write-WarnMsg "failure threshold reached, restarting RealSense node"
            Stop-RealsenseNodeProcess -Process $nodeProc
            Start-Sleep -Seconds $RestartBackoffSec
            $nodeProc = Start-RealsenseNodeProcess -NodeArgs $nodeArgs
            Write-Ok "node restarted (pid=$($nodeProc.Id))"
            $consecutiveFailures = 0
            $monitorMode = "warmup"
            $healthySamples = 0
        }

        Start-Sleep -Seconds $CheckIntervalSec
    }
}
finally {
    Stop-RealsenseNodeProcess -Process $nodeProc
}
