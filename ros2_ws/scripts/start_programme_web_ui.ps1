param(
    [string]$Distro = "Ubuntu-24.04",
    [switch]$NoBrowser,
    [switch]$Watchdog
)

$ErrorActionPreference = "Stop"

$runtimeDir = Join-Path $env:LOCALAPPDATA "ProgrammeWebUI"
$watchdogPidFile = Join-Path $runtimeDir "watchdog.win.pid"
$watchdogLog = Join-Path $runtimeDir "watchdog.log"
$linuxStartScript = "/home/whispers/programme/ros2_ws/scripts/start_programme_web_ui.sh"
$stopScript = Join-Path $PSScriptRoot "stop_programme_web_ui.ps1"

function Test-HttpReady {
    param([Parameter(Mandatory = $true)][string]$Url)
    try {
        $response = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 3
        return ($response.StatusCode -ge 200 -and $response.StatusCode -lt 300)
    } catch {
        return $false
    }
}

function Wait-HttpReady {
    param(
        [Parameter(Mandatory = $true)][string]$Url,
        [Parameter(Mandatory = $true)][string]$Label,
        [Parameter(Mandatory = $true)][int]$TimeoutSec
    )

    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (Test-HttpReady -Url $Url) {
            Write-Host "[start] $Label ready: $Url"
            return
        }
        Start-Sleep -Seconds 1
    }

    throw "[start] $Label did not become ready: $Url"
}

function Invoke-WslStart {
    $arguments = @("-d", $Distro, "--", $linuxStartScript, "--no-browser")
    & wsl.exe @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "[watchdog] start_programme_web_ui.sh exited with code $LASTEXITCODE"
    }
}

if ($Watchdog) {
    New-Item -ItemType Directory -Path $runtimeDir -Force | Out-Null
    Add-Content -Path $watchdogLog -Value ("[watchdog] started at {0}" -f (Get-Date -Format o))
    $consecutiveFailures = 0
    while ($true) {
        $frontendOk = Test-HttpReady -Url "http://127.0.0.1:5173/api/bootstrap"
        $gatewayOk = Test-HttpReady -Url "http://127.0.0.1:8765/api/bootstrap"
        if ($frontendOk -and $gatewayOk) {
            $consecutiveFailures = 0
        } else {
            $consecutiveFailures += 1
            Add-Content -Path $watchdogLog -Value (
                "[watchdog] healthcheck failed at {0}: frontend={1} gateway={2} consecutive={3}" -f
                (Get-Date -Format o), $frontendOk, $gatewayOk, $consecutiveFailures
            )
            if ($consecutiveFailures -ge 2) {
                Add-Content -Path $watchdogLog -Value ("[watchdog] restarting stack at {0}" -f (Get-Date -Format o))
                try {
                    Invoke-WslStart
                    $consecutiveFailures = 0
                } catch {
                    Add-Content -Path $watchdogLog -Value (
                        "[watchdog] restart failed at {0}: {1}" -f (Get-Date -Format o), $_.Exception.Message
                    )
                }
            }
        }
        Start-Sleep -Seconds 5
    }
}

New-Item -ItemType Directory -Path $runtimeDir -Force | Out-Null
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $stopScript -Distro $Distro | Out-Null

Invoke-WslStart

Wait-HttpReady -Url "http://127.0.0.1:8765/api/bootstrap" -Label "gateway" -TimeoutSec 30
Wait-HttpReady -Url "http://127.0.0.1:5173/control" -Label "frontend" -TimeoutSec 20

$watchdogProcess = Start-Process -FilePath "powershell.exe" `
    -ArgumentList @(
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        $PSCommandPath,
        "-Distro",
        $Distro,
        "-Watchdog"
    ) `
    -WindowStyle Hidden `
    -PassThru
Set-Content -Path $watchdogPidFile -Value ([string]$watchdogProcess.Id) -Encoding ascii

if (-not $NoBrowser) {
    Start-Process "http://127.0.0.1:5173/control" | Out-Null
}

Write-Host "[start] Programme Web UI is ready"
Write-Host "[start] control page: http://127.0.0.1:5173/control"
Write-Host "[start] gateway root: http://127.0.0.1:8765/"
Write-Host "[start] watchdog pid file: $watchdogPidFile"
Write-Host "[start] watchdog log: $watchdogLog"
