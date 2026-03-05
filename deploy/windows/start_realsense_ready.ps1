param(
    [string]$RosSetup = "C:\\pixi_ws\\ros2-windows\\ros2-windows\\local_setup.bat",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [string]$RealsenseSerial = "",
    [int]$TopicTimeoutSec = 20,
    [int]$HzSampleSec = 10,
    [double]$MinColorHz = 3.0,
    [double]$MinDepthHz = 3.0
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

Write-Step "loading ROS2 Windows environment"
. (Join-Path $scriptDir "env_ros2_windows.ps1") -RosSetup $RosSetup -WorkspaceSetup $WorkspaceSetup -DomainId $DomainId

if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    Write-Fail "ros2 command is unavailable after environment setup."
    exit 1
}

Write-Step "starting RealSense node in a dedicated window"
. (Join-Path $scriptDir "start_realsense_only.ps1") -RosSetup $RosSetup -WorkspaceSetup $WorkspaceSetup -DomainId $DomainId -RealsenseSerial $RealsenseSerial -Execute

$colorTopic = "/camera/camera/color/image_raw"
$depthTopic = "/camera/camera/aligned_depth_to_color/image_raw"
$infoTopic = "/camera/camera/color/camera_info"

Write-Step "waiting for camera topics"
if (-not (Wait-Topic -TopicName $colorTopic -TimeoutSec $TopicTimeoutSec)) {
    Write-Fail "color topic not discovered within ${TopicTimeoutSec}s: $colorTopic"
    exit 1
}
if (-not (Wait-Topic -TopicName $depthTopic -TimeoutSec $TopicTimeoutSec)) {
    Write-Fail "depth topic not discovered within ${TopicTimeoutSec}s: $depthTopic"
    exit 1
}
if (-not (Wait-Topic -TopicName $infoTopic -TimeoutSec $TopicTimeoutSec)) {
    Write-Fail "camera_info topic not discovered within ${TopicTimeoutSec}s: $infoTopic"
    exit 1
}
Write-Ok "topics discovered"

Write-Step "sampling color/depth hz"
$colorHz = Get-TopicAverageHz -TopicName $colorTopic -SampleSec $HzSampleSec
$depthHz = Get-TopicAverageHz -TopicName $depthTopic -SampleSec $HzSampleSec

if ($null -eq $colorHz) {
    Write-Fail "unable to calculate color hz from ros2 topic hz output"
    exit 1
}
if ($null -eq $depthHz) {
    Write-Fail "unable to calculate depth hz from ros2 topic hz output"
    exit 1
}

if ($colorHz -lt $MinColorHz) {
    Write-Fail ("color hz {0:N3} < min {1:N3}" -f $colorHz, $MinColorHz)
    exit 1
}
if ($depthHz -lt $MinDepthHz) {
    Write-Fail ("depth hz {0:N3} < min {1:N3}" -f $depthHz, $MinDepthHz)
    exit 1
}

Write-Ok ("RealSense READY: color={0:N3}Hz depth={1:N3}Hz" -f $colorHz, $depthHz)
Write-Host "[READY] You can now start VM one-click debug script."

