param(
    [string]$RosSetup = "C:\\pixi_ws\\ros2-windows\\ros2-windows\\local_setup.bat",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [int]$SampleSec = 12,
    [long]$StartEpochSec = 0
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

function Get-NowEpochSec {
    return [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
}

function Wait-UntilEpoch {
    param([long]$TargetEpoch)
    if ($TargetEpoch -le 0) {
        return
    }
    while ((Get-NowEpochSec) -lt $TargetEpoch) {
        Start-Sleep -Milliseconds 200
    }
}

function Get-TopicAverageHz {
    param(
        [string]$TopicName,
        [int]$WindowSec
    )

    $outFile = Join-Path $env:TEMP ("programme_hz_" + [guid]::NewGuid().ToString() + ".out.log")
    $errFile = Join-Path $env:TEMP ("programme_hz_" + [guid]::NewGuid().ToString() + ".err.log")
    try {
        $proc = Start-Process -FilePath ros2 -ArgumentList @("topic", "hz", $TopicName) -PassThru -RedirectStandardOutput $outFile -RedirectStandardError $errFile -WindowStyle Hidden
        Start-Sleep -Seconds $WindowSec
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

. (Join-Path $scriptDir "env_ros2_windows.ps1") -RosSetup $RosSetup -WorkspaceSetup $WorkspaceSetup -DomainId $DomainId

$colorTopic = "/camera/camera/color/image_raw"
$depthTopic = "/camera/camera/aligned_depth_to_color/image_raw"

Wait-UntilEpoch -TargetEpoch $StartEpochSec
$startEpoch = Get-NowEpochSec

$colorHz = Get-TopicAverageHz -TopicName $colorTopic -WindowSec $SampleSec
$depthHz = Get-TopicAverageHz -TopicName $depthTopic -WindowSec $SampleSec

if ($null -eq $colorHz -or $null -eq $depthHz) {
    Write-Error "failed to sample hz (color=$colorHz, depth=$depthHz)"
    exit 1
}

$endEpoch = Get-NowEpochSec
Write-Host ("[RESULT][windows] start={0} end={1} sample={2}s color_hz={3:N3} depth_hz={4:N3}" -f $startEpoch, $endEpoch, $SampleSec, $colorHz, $depthHz)
