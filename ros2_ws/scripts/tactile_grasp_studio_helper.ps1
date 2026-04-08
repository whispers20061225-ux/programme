param(
    [string]$Distro = "Ubuntu-24.04"
)

$ErrorActionPreference = "Stop"

$runtimeDir = Join-Path $env:LOCALAPPDATA "ProgrammeWebUI"
$requestFile = Join-Path $runtimeDir "debug-open-request.json"
$stateFile = Join-Path $runtimeDir "debug-open-request.state"
$helperLog = Join-Path $runtimeDir "debug-helper.log"
$openScript = Join-Path $PSScriptRoot "open_tactile_debug_views.ps1"
$windowsPowerShell = "C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe"

New-Item -ItemType Directory -Path $runtimeDir -Force | Out-Null

function Get-LastHandledId {
    if (Test-Path $stateFile) {
        return ((Get-Content -Path $stateFile -ErrorAction SilentlyContinue | Select-Object -First 1) | ForEach-Object { $_.Trim() })
    }
    return ""
}

function Set-LastHandledId {
    param([string]$Id)
    Set-Content -Path $stateFile -Value $Id -Encoding ASCII
}

function Write-HelperLog {
    param([string]$Message)
    Add-Content -Path $helperLog -Value ("[{0}] {1}" -f (Get-Date -Format o), $Message)
}

$lastHandledId = Get-LastHandledId
Write-HelperLog "started distro=$Distro lastHandledId=$lastHandledId"

while ($true) {
    try {
        if (Test-Path $requestFile) {
            $request = Get-Content -Path $requestFile -Raw -ErrorAction Stop | ConvertFrom-Json
            $requestId = ""
            if ($null -ne $request -and $null -ne $request.request_id) {
                $requestId = [string]$request.request_id
            }
            if ($requestId -and $requestId -ne $lastHandledId) {
                $requestDistro = $Distro
                if ($null -ne $request -and $null -ne $request.distro -and [string]$request.distro) {
                    $requestDistro = [string]$request.distro
                }
                $args = @(
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    $openScript,
                    "-Distro",
                    $requestDistro
                )
                if ([bool]$request.open_gazebo_gui) {
                    $args += "-OpenGazeboGui"
                }
                if ([bool]$request.open_rviz) {
                    $args += "-OpenRviz"
                }
                Write-HelperLog ("launch request id={0} gazebo={1} rviz={2}" -f $requestId, [bool]$request.open_gazebo_gui, [bool]$request.open_rviz)
                $result = & $windowsPowerShell @args 2>&1
                $resultText = ($result | Out-String).Trim()
                if ($resultText) {
                    Write-HelperLog ("launcher result id={0}: {1}" -f $requestId, $resultText)
                }
                $lastHandledId = $requestId
                Set-LastHandledId -Id $requestId
            }
        }
    } catch {
        Write-HelperLog ("error: {0}" -f $_.Exception.Message)
    }
    Start-Sleep -Milliseconds 700
}
