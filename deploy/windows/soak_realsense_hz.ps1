param(
    [string]$RosSetup = "C:\\pixi_ws\\ros2-windows\\ros2-windows\\local_setup.bat",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [int]$DurationMin = 15,
    [int]$IntervalSec = 60,
    [int]$SampleSec = 8,
    [int]$TopicTimeoutSec = 30,
    [string]$OutputCsv = ""
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = (Resolve-Path (Join-Path $scriptDir "..\\..")).Path

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

Write-Step "waiting for topics"
if (-not (Wait-Topic -TopicName $colorTopic -TimeoutSec $TopicTimeoutSec)) {
    Write-Fail "color topic not found: $colorTopic"
    exit 1
}
if (-not (Wait-Topic -TopicName $depthTopic -TimeoutSec $TopicTimeoutSec)) {
    Write-Fail "depth topic not found: $depthTopic"
    exit 1
}
Write-Ok "topics discovered"

$logDir = Join-Path $projectRoot "ros2_ws\\log\\soak"
New-Item -ItemType Directory -Path $logDir -Force | Out-Null

if (-not $OutputCsv) {
    $ts = Get-Date -Format "yyyyMMdd_HHmmss"
    $OutputCsv = Join-Path $logDir ("windows_soak_{0}.csv" -f $ts)
}

$records = New-Object System.Collections.Generic.List[object]
$endTime = (Get-Date).AddMinutes([Math]::Max(1, $DurationMin))
$sampleIndex = 0

Write-Step "starting soak test (duration=${DurationMin}min, interval=${IntervalSec}s, sample=${SampleSec}s)"
while ((Get-Date) -lt $endTime) {
    $sampleIndex += 1
    $stamp = [DateTimeOffset]::UtcNow
    $colorHz = Get-TopicAverageHz -TopicName $colorTopic -WindowSec $SampleSec
    $depthHz = Get-TopicAverageHz -TopicName $depthTopic -WindowSec $SampleSec

    $status = "ok"
    if ($null -eq $colorHz -or $null -eq $depthHz) {
        $status = "no_rate"
    }

    $row = [PSCustomObject]@{
        sample_index = $sampleIndex
        timestamp_iso = $stamp.ToString("o")
        timestamp_epoch = $stamp.ToUnixTimeSeconds()
        color_hz = if ($null -eq $colorHz) { 0.0 } else { [Math]::Round($colorHz, 3) }
        depth_hz = if ($null -eq $depthHz) { 0.0 } else { [Math]::Round($depthHz, 3) }
        status = $status
    }
    $records.Add($row)

    if ($status -eq "ok") {
        Write-Host ("[SAMPLE][windows] idx={0} color={1:N3} depth={2:N3}" -f $sampleIndex, $row.color_hz, $row.depth_hz)
    }
    else {
        Write-WarnMsg ("[SAMPLE][windows] idx={0} no average rate captured" -f $sampleIndex)
    }

    if ((Get-Date) -ge $endTime) {
        break
    }
    Start-Sleep -Seconds ([Math]::Max(1, $IntervalSec))
}

$records | Export-Csv -Path $OutputCsv -NoTypeInformation -Encoding UTF8
Write-Ok "csv exported: $OutputCsv"

$count = $records.Count
if ($count -le 0) {
    Write-Fail "no samples captured"
    exit 1
}

$colorStats = $records | Measure-Object -Property color_hz -Minimum -Maximum -Average
$depthStats = $records | Measure-Object -Property depth_hz -Minimum -Maximum -Average
$zeroColor = ($records | Where-Object { [double]$_.color_hz -lt 1.0 }).Count
$zeroDepth = ($records | Where-Object { [double]$_.depth_hz -lt 1.0 }).Count
$zeroColorPct = [Math]::Round(($zeroColor * 100.0) / $count, 1)
$zeroDepthPct = [Math]::Round(($zeroDepth * 100.0) / $count, 1)

Write-Host ("[SUMMARY][windows] samples={0} color(avg/min/max)={1:N3}/{2:N3}/{3:N3} depth(avg/min/max)={4:N3}/{5:N3}/{6:N3}" -f `
    $count, $colorStats.Average, $colorStats.Minimum, $colorStats.Maximum, $depthStats.Average, $depthStats.Minimum, $depthStats.Maximum)
Write-Host ("[SUMMARY][windows] color<1Hz={0}/{1} ({2}%) depth<1Hz={3}/{1} ({4}%)" -f `
    $zeroColor, $count, $zeroColorPct, $zeroDepth, $zeroDepthPct)
