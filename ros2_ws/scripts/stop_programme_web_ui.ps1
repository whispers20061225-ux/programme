param(
    [string]$Distro = "Ubuntu-24.04"
)

$ErrorActionPreference = "SilentlyContinue"

$runtimeDir = Join-Path $env:LOCALAPPDATA "ProgrammeWebUI"
$watchdogPidFile = Join-Path $runtimeDir "watchdog.win.pid"

if (Test-Path $watchdogPidFile) {
    $rawPid = (Get-Content -Path $watchdogPidFile | Select-Object -First 1).Trim()
    $pidValue = 0
    if ([int]::TryParse($rawPid, [ref]$pidValue)) {
        Stop-Process -Id $pidValue -Force -ErrorAction SilentlyContinue
    }
    Remove-Item -Path $watchdogPidFile -Force -ErrorAction SilentlyContinue
}

Get-CimInstance Win32_Process |
    Where-Object {
        $_.Name -eq "powershell.exe" -and
        $_.CommandLine -match "start_programme_web_ui\.ps1" -and
        $_.CommandLine -match "-Watchdog"
    } |
    ForEach-Object {
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }

& wsl.exe -d $Distro -- /home/whispers/programme/ros2_ws/scripts/stop_programme_web_ui.sh
exit $LASTEXITCODE
