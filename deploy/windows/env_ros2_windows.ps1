param(
    [int]$DomainId = 0,
    [string]$RosSetup = "C:\\opt\\ros\\jazzy\\x64\\local_setup.ps1",
    [string]$WorkspaceSetup = ""
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = (Resolve-Path (Join-Path $scriptDir "..\\..")).Path
$ddsFile = (Resolve-Path (Join-Path $projectRoot "config\\dds\\cyclonedds_windows.xml")).Path
$ddsUriPath = $ddsFile -replace "\\", "/"

if (Test-Path $RosSetup) {
    $ext = [System.IO.Path]::GetExtension($RosSetup).ToLowerInvariant()
    if ($ext -eq ".ps1") {
        . $RosSetup
    } elseif ($ext -eq ".bat" -or $ext -eq ".cmd") {
        $dump = cmd.exe /c "call `"$RosSetup`" && set"
        foreach ($line in $dump) {
            if ($line -match "^(.*?)=(.*)$") {
                $name = $matches[1]
                $value = $matches[2]
                if ($name) {
                    [System.Environment]::SetEnvironmentVariable($name, $value, "Process")
                }
            }
        }
    } else {
        Write-Warning "Unsupported ROS2 setup script type: $RosSetup"
    }
} else {
    Write-Warning "ROS2 setup script not found: $RosSetup"
}

if ($WorkspaceSetup -and (Test-Path $WorkspaceSetup)) {
    . $WorkspaceSetup
} elseif ($WorkspaceSetup) {
    Write-Warning "Workspace setup script not found: $WorkspaceSetup"
}

$env:ROS_DOMAIN_ID = "$DomainId"
$env:RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
$env:ROS_LOCALHOST_ONLY = "0"
$env:CYCLONEDDS_URI = "file:///$ddsUriPath"

Write-Host "ROS2 Windows environment ready."
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "RMW_IMPLEMENTATION=$env:RMW_IMPLEMENTATION"
Write-Host "CYCLONEDDS_URI=$env:CYCLONEDDS_URI"
