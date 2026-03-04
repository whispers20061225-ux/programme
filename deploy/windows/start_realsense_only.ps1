param(
    [string]$RosSetup = "C:\\opt\\ros\\jazzy\\x64\\local_setup.ps1",
    [string]$WorkspaceSetup = "",
    [int]$DomainId = 0,
    [string]$RealsenseSerial = "_333422301846",
    [switch]$Execute = $false
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

. (Join-Path $scriptDir "start_hw_nodes.ps1") `
    -RosSetup $RosSetup `
    -WorkspaceSetup $WorkspaceSetup `
    -DomainId $DomainId `
    -RealsenseSerial $RealsenseSerial `
    -StartArm:$false `
    -StartRealsense:$true `
    -Execute:$Execute

