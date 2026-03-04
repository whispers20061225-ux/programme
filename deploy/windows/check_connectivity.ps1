param(
    [string]$VmIp = "192.168.56.20",
    [string]$RosSetup = "C:\\opt\\ros\\jazzy\\x64\\local_setup.ps1",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [switch]$RunTalker = $false,
    [switch]$RunListener = $false
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

. (Join-Path $scriptDir "env_ros2_windows.ps1") -RosSetup $RosSetup -WorkspaceSetup $WorkspaceSetup -DomainId $DomainId

Write-Host "Pinging VM host-only IP: $VmIp"
if (-not (Test-Connection -ComputerName $VmIp -Count 2 -Quiet)) {
    Write-Error "Cannot reach VM IP $VmIp. Check VMware host-only adapter settings."
    exit 1
}

Write-Host "Ping OK."

if ($RunTalker) {
    Write-Host "Starting ROS2 talker on Windows..."
    ros2 run demo_nodes_cpp talker
    exit $LASTEXITCODE
}

if ($RunListener) {
    Write-Host "Starting ROS2 listener on Windows..."
    ros2 run demo_nodes_cpp listener
    exit $LASTEXITCODE
}

Write-Host "Connectivity check complete. Use -RunTalker or -RunListener for DDS verification."
