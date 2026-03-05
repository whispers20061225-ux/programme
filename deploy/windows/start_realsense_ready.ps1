param(
    [string]$RosSetup = "C:\\pixi_ws\\ros2-windows\\ros2-windows\\local_setup.bat",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [string]$RealsenseSerial = "",
    [int]$TopicTimeoutSec = 20,
    [int]$HzSampleSec = 10,
    [double]$MinColorHz = 3.0,
    [double]$MinDepthHz = 3.0,
    [int]$StartupRetryCount = 3,
    [int]$RetryBackoffSec = 4
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
        Start-Sleep -Seconds 1
    }
    return $false
}

function Wait-MessageOnce {
    param(
        [string]$TopicName,
        [int]$TimeoutSec
    )

    $outFile = Join-Path $env:TEMP ("programme_echo_" + [guid]::NewGuid().ToString() + ".out.log")
    $errFile = Join-Path $env:TEMP ("programme_echo_" + [guid]::NewGuid().ToString() + ".err.log")
    try {
        $proc = Start-Process -FilePath ros2 -ArgumentList @("topic", "echo", $TopicName, "--once") -PassThru -RedirectStandardOutput $outFile -RedirectStandardError $errFile -WindowStyle Hidden
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
        $last = $matches[$matches.Count - 1].Groups[1].Value
        return [double]$last
    }
    finally {
        Remove-Item $outFile, $errFile -Force -ErrorAction SilentlyContinue
    }
}

function Stop-RealsenseProcesses {
    $patterns = @(
        "realsense_watchdog.ps1",
        "realsense_camera_node",
        "realsense2_camera_node",
        "realsense2_camera"
    )
    $killed = 0
    $procs = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue
    foreach ($proc in $procs) {
        $cmd = [string]$proc.CommandLine
        if ([string]::IsNullOrWhiteSpace($cmd)) {
            continue
        }
        foreach ($pattern in $patterns) {
            if ($cmd -match [regex]::Escape($pattern)) {
                try {
                    Stop-Process -Id $proc.ProcessId -Force -ErrorAction SilentlyContinue
                    $killed += 1
                }
                catch {
                    # ignore
                }
                break
            }
        }
    }
    if ($killed -gt 0) {
        Write-WarnMsg "stopped $killed existing RealSense process(es) before restart"
        Start-Sleep -Seconds 1
    }
}

function Test-RealsenseReady {
    param(
        [string]$ColorTopic,
        [string]$DepthTopic,
        [string]$InfoTopic,
        [int]$TopicWaitSec,
        [int]$SampleWindowSec,
        [double]$MinColor,
        [double]$MinDepth
    )

    if (-not (Wait-Topic -TopicName $ColorTopic -TimeoutSec $TopicWaitSec)) {
        return [PSCustomObject]@{ Success = $false; Message = "color topic not discovered: $ColorTopic"; ColorHz = $null; DepthHz = $null }
    }
    if (-not (Wait-Topic -TopicName $DepthTopic -TimeoutSec $TopicWaitSec)) {
        return [PSCustomObject]@{ Success = $false; Message = "depth topic not discovered: $DepthTopic"; ColorHz = $null; DepthHz = $null }
    }
    if (-not (Wait-Topic -TopicName $InfoTopic -TimeoutSec $TopicWaitSec)) {
        return [PSCustomObject]@{ Success = $false; Message = "camera_info topic not discovered: $InfoTopic"; ColorHz = $null; DepthHz = $null }
    }
    if (-not (Wait-MessageOnce -TopicName $InfoTopic -TimeoutSec 8)) {
        return [PSCustomObject]@{ Success = $false; Message = "camera_info message not received"; ColorHz = $null; DepthHz = $null }
    }

    $colorHz = Get-TopicAverageHz -TopicName $ColorTopic -SampleSec $SampleWindowSec
    $depthHz = Get-TopicAverageHz -TopicName $DepthTopic -SampleSec $SampleWindowSec
    if ($null -eq $colorHz) {
        return [PSCustomObject]@{ Success = $false; Message = "unable to calculate color hz"; ColorHz = $null; DepthHz = $depthHz }
    }
    if ($null -eq $depthHz) {
        return [PSCustomObject]@{ Success = $false; Message = "unable to calculate depth hz"; ColorHz = $colorHz; DepthHz = $null }
    }
    if ($colorHz -lt $MinColor) {
        return [PSCustomObject]@{ Success = $false; Message = ("color hz {0:N3} < min {1:N3}" -f $colorHz, $MinColor); ColorHz = $colorHz; DepthHz = $depthHz }
    }
    if ($depthHz -lt $MinDepth) {
        return [PSCustomObject]@{ Success = $false; Message = ("depth hz {0:N3} < min {1:N3}" -f $depthHz, $MinDepth); ColorHz = $colorHz; DepthHz = $depthHz }
    }
    return [PSCustomObject]@{ Success = $true; Message = "ok"; ColorHz = $colorHz; DepthHz = $depthHz }
}

Write-Step "loading ROS2 Windows environment"
. (Join-Path $scriptDir "env_ros2_windows.ps1") -RosSetup $RosSetup -WorkspaceSetup $WorkspaceSetup -DomainId $DomainId

if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    Write-Fail "ros2 command is unavailable after environment setup."
    exit 1
}

$colorTopic = "/camera/camera/color/image_raw"
$depthTopic = "/camera/camera/aligned_depth_to_color/image_raw"
$infoTopic = "/camera/camera/color/camera_info"

$lastResult = $null
for ($attempt = 1; $attempt -le $StartupRetryCount; $attempt++) {
    Write-Step "RealSense startup attempt $attempt/$StartupRetryCount"
    Stop-RealsenseProcesses

    . (Join-Path $scriptDir "start_realsense_only.ps1") `
        -RosSetup $RosSetup `
        -WorkspaceSetup $WorkspaceSetup `
        -DomainId $DomainId `
        -RealsenseSerial $RealsenseSerial `
        -Execute

    Start-Sleep -Seconds 3

    $lastResult = Test-RealsenseReady `
        -ColorTopic $colorTopic `
        -DepthTopic $depthTopic `
        -InfoTopic $infoTopic `
        -TopicWaitSec $TopicTimeoutSec `
        -SampleWindowSec $HzSampleSec `
        -MinColor $MinColorHz `
        -MinDepth $MinDepthHz

    if ($lastResult.Success) {
        Write-Ok ("RealSense READY: color={0:N3}Hz depth={1:N3}Hz" -f $lastResult.ColorHz, $lastResult.DepthHz)
        Write-Host "[READY] You can now start VM one-click debug script."
        exit 0
    }

    Write-WarnMsg ("attempt $attempt failed: " + $lastResult.Message)
    if ($attempt -lt $StartupRetryCount) {
        Start-Sleep -Seconds $RetryBackoffSec
    }
}

Write-Fail ("RealSense did not reach ready state after {0} attempts. Last error: {1}" -f $StartupRetryCount, $lastResult.Message)
Write-Host "[DIAG] current camera topics:"
& ros2 topic list | Select-String "camera/camera"
exit 1
