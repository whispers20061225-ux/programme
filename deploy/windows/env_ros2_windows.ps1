param(
    [int]$DomainId = 0,
    [string]$RosSetup = "C:\\opt\\ros\\jazzy\\x64\\local_setup.ps1",
    [string]$WorkspaceSetup = "",
    [bool]$CleanConda = $true,
    [string]$PixiOpenSslBin = "",
    [bool]$WarmupRosGraph = $false,
    [int]$RosDaemonCmdTimeoutSec = 8,
    [string]$WindowsHostOnlyIp = "",
    [string]$VmHostOnlyIp = ""
)

function Remove-CondaEntriesFromPath {
    param([string]$PathValue)

    if (-not $PathValue) {
        return ""
    }

    $parts = $PathValue -split ';'
    $filtered = $parts | Where-Object {
        $_ -and $_ -notmatch '(?i)(miniconda|anaconda|\\conda\\|\\mamba\\|\\micromamba\\)'
    }
    return ($filtered -join ';')
}

function Prepend-Path {
    param([string]$PathValue)

    if (-not $PathValue) {
        return
    }
    if (-not (Test-Path $PathValue)) {
        return
    }

    $parts = @()
    if ($env:PATH) {
        $parts = $env:PATH -split ';'
    }
    if ($parts -contains $PathValue) {
        return
    }
    $env:PATH = "$PathValue;$env:PATH"
}

function Add-PathListEntry {
    param(
        [string]$VariableName,
        [string]$Entry
    )

    if (-not $Entry) {
        return
    }
    if (-not (Test-Path $Entry)) {
        return
    }

    $separator = [System.IO.Path]::PathSeparator
    $current = [System.Environment]::GetEnvironmentVariable($VariableName, "Process")
    if ([string]::IsNullOrWhiteSpace($current)) {
        [System.Environment]::SetEnvironmentVariable($VariableName, $Entry, "Process")
        return
    }

    $parts = @($current -split [regex]::Escape("$separator"))
    if ($parts -contains $Entry) {
        return
    }

    [System.Environment]::SetEnvironmentVariable($VariableName, "$Entry$separator$current", "Process")
}

function Configure-GazeboResourcePaths {
    param([string]$ProjectRoot)

    $resourceDirs = @(
        (Join-Path $ProjectRoot "ros2_ws\\src\\tactile_sim"),
        (Join-Path $ProjectRoot "ros2_ws\\src\\tactile_bringup")
    )

    foreach ($packageName in @("tactile_sim", "tactile_bringup")) {
        try {
            $packagePrefix = (& ros2 pkg prefix $packageName 2>$null)
            if ($LASTEXITCODE -eq 0 -and $packagePrefix) {
                $resourceDirs += (Join-Path $packagePrefix "share\\$packageName")
            }
        }
        catch {
        }
    }

    foreach ($resourceDir in $resourceDirs) {
        Add-PathListEntry -VariableName "GZ_SIM_RESOURCE_PATH" -Entry $resourceDir
        Add-PathListEntry -VariableName "IGN_GAZEBO_RESOURCE_PATH" -Entry $resourceDir
    }
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = (Resolve-Path (Join-Path $scriptDir "..\\..")).Path
$ddsTemplateFile = (Resolve-Path (Join-Path $projectRoot "config\\dds\\cyclonedds_windows.xml")).Path
$ddsRuntimeFile = Join-Path $env:TEMP "programme_cyclonedds_windows.xml"

# Ensure native process stdout is decoded as UTF-8 in this shell.
# This avoids garbled non-ASCII paths when sourcing colcon-generated setup scripts.
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
[Console]::OutputEncoding = $utf8NoBom
$OutputEncoding = $utf8NoBom
$env:PYTHONIOENCODING = "utf-8"

function Import-SetupScript {
    param(
        [string]$SetupPath,
        [string]$Label = "setup"
    )

    if (-not $SetupPath) {
        return
    }
    if (-not (Test-Path $SetupPath)) {
        Write-Warning "$Label script not found: $SetupPath"
        return
    }

    $ext = [System.IO.Path]::GetExtension($SetupPath).ToLowerInvariant()
    if ($ext -eq ".ps1") {
        . $SetupPath
        return
    }

    if ($ext -eq ".bat" -or $ext -eq ".cmd") {
        $ps1Candidate = [System.IO.Path]::ChangeExtension($SetupPath, ".ps1")
        if (Test-Path $ps1Candidate) {
            . $ps1Candidate
            return
        }

        $dump = cmd.exe /c "call `"$SetupPath`" && set"
        foreach ($line in $dump) {
            if ($line -match "^(.*?)=(.*)$") {
                $name = $matches[1]
                $value = $matches[2]
                if ($name) {
                    [System.Environment]::SetEnvironmentVariable($name, $value, "Process")
                }
            }
        }
        return
    }

    Write-Warning "Unsupported $Label script type: $SetupPath"
}

function Invoke-Ros2DaemonCommand {
    param(
        [string]$Subcommand,
        [int]$TimeoutSec = 8
    )

    if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
        return $false
    }

    try {
        $proc = Start-Process -FilePath ros2 -ArgumentList @("daemon", $Subcommand) -PassThru -WindowStyle Hidden
        $exited = $proc.WaitForExit($TimeoutSec * 1000)
        if (-not $exited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
            return $false
        }
        return ($proc.ExitCode -eq 0)
    }
    catch {
        return $false
    }
}

function Get-Ipv4Candidates {
    $addresses = @()
    try {
        $addresses = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop | Where-Object {
                $_.IPAddress -and
                $_.IPAddress -notmatch '^127\.' -and
                $_.IPAddress -notmatch '^169\.254\.' -and
                $_.IPAddress -notmatch '^0\.'
            })
    }
    catch {
        $addresses = @()
    }
    return $addresses
}

function Get-DefaultRouteInterfaceIndices {
    try {
        return @(
            Get-NetRoute -AddressFamily IPv4 -DestinationPrefix "0.0.0.0/0" -ErrorAction Stop |
            Sort-Object -Property RouteMetric, InterfaceMetric |
            Select-Object -ExpandProperty InterfaceIndex -Unique
        )
    }
    catch {
        return @()
    }
}

function Resolve-WindowsHostOnlyIp {
    param([string]$RequestedIp)

    $candidates = @(Get-Ipv4Candidates)
    if ($RequestedIp) {
        if ($candidates.IPAddress -contains $RequestedIp) {
            return $RequestedIp
        }
        Write-Warning "Requested WindowsHostOnlyIp $RequestedIp is not present on this host. Auto-detecting instead."
    }

    $adapterByIndex = @{}
    try {
        Get-NetAdapter -IncludeHidden -ErrorAction Stop | ForEach-Object {
            $adapterByIndex[$_.ifIndex] = $_
        }
    }
    catch {
    }

    $vmwareCandidates = @(
        $candidates | Where-Object {
            $adapter = $adapterByIndex[$_.InterfaceIndex]
            $adapterName = if ($adapter) { "$($adapter.Name) $($adapter.InterfaceDescription)" } else { $_.InterfaceAlias }
            $adapterName -match 'VMware|VMnet|Host-Only'
        }
    )
    if ($vmwareCandidates.Count -gt 0) {
        return ($vmwareCandidates | Select-Object -First 1).IPAddress
    }

    $defaultRouteIndices = @(Get-DefaultRouteInterfaceIndices)
    $nonDefaultCandidates = @($candidates | Where-Object { $defaultRouteIndices -notcontains $_.InterfaceIndex })
    if ($nonDefaultCandidates.Count -gt 0) {
        return ($nonDefaultCandidates | Select-Object -First 1).IPAddress
    }

    if ($candidates.Count -gt 0) {
        return ($candidates | Select-Object -First 1).IPAddress
    }

    throw "Failed to resolve a usable Windows IPv4 for CycloneDDS."
}

function Resolve-VmPeerIp {
    param(
        [string]$RequestedIp,
        [string]$LocalIp
    )

    if ($RequestedIp) {
        return $RequestedIp
    }

    if ($LocalIp -match '^(\d+)\.(\d+)\.(\d+)\.\d+$') {
        return "$($Matches[1]).$($Matches[2]).$($Matches[3]).128"
    }

    return "192.168.147.128"
}

function Write-CycloneDDSRuntimeFile {
    param(
        [string]$RuntimeFile,
        [string]$LocalIp,
        [string]$PeerIp
    )

    $xml = @"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="$LocalIp" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="127.0.0.1" />
        <Peer Address="$LocalIp" />
        <Peer Address="$PeerIp" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"@

    [System.IO.File]::WriteAllText($RuntimeFile, $xml, [System.Text.UTF8Encoding]::new($false))
}

if (-not (Test-Path $ddsTemplateFile)) {
    throw "CycloneDDS template not found: $ddsTemplateFile"
}

$resolvedWindowsHostOnlyIp = Resolve-WindowsHostOnlyIp -RequestedIp $WindowsHostOnlyIp
$resolvedVmHostOnlyIp = Resolve-VmPeerIp -RequestedIp $VmHostOnlyIp -LocalIp $resolvedWindowsHostOnlyIp
Write-CycloneDDSRuntimeFile -RuntimeFile $ddsRuntimeFile -LocalIp $resolvedWindowsHostOnlyIp -PeerIp $resolvedVmHostOnlyIp

if ($CleanConda) {
    $env:PATH = Remove-CondaEntriesFromPath -PathValue $env:PATH
    foreach ($name in @(
            "CONDA_PREFIX",
            "CONDA_DEFAULT_ENV",
            "CONDA_PROMPT_MODIFIER",
            "CONDA_EXE",
            "CONDA_PYTHON_EXE",
            "CONDA_SHLVL",
            "PYTHONHOME",
            "PYTHONPATH"
        )) {
        [System.Environment]::SetEnvironmentVariable($name, $null, "Process")
    }
}
Import-SetupScript -SetupPath $RosSetup -Label "ROS2 setup"

if (-not $PixiOpenSslBin -and (Test-Path $RosSetup)) {
    $rosRoot = Split-Path -Parent $RosSetup
    $candidateWorkspace = Split-Path -Parent (Split-Path -Parent $rosRoot)
    $candidatePixiDll = Join-Path $candidateWorkspace ".pixi\\envs\\default\\Library\\bin"
    $candidatePixiScripts = Join-Path $candidateWorkspace ".pixi\\envs\\default\\Scripts"
    $candidatePixiRoot = Join-Path $candidateWorkspace ".pixi\\envs\\default"

    if (Test-Path $candidatePixiScripts) {
        Prepend-Path -PathValue $candidatePixiScripts
    }
    if (Test-Path $candidatePixiRoot) {
        Prepend-Path -PathValue $candidatePixiRoot
    }
    if (Test-Path $candidatePixiDll) {
        $PixiOpenSslBin = $candidatePixiDll
    }
}

if ($PixiOpenSslBin) {
    Prepend-Path -PathValue $PixiOpenSslBin
}

Import-SetupScript -SetupPath $WorkspaceSetup -Label "Workspace setup"
Configure-GazeboResourcePaths -ProjectRoot $projectRoot

$env:ROS_DOMAIN_ID = "$DomainId"
$env:RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
$env:ROS_AUTOMATIC_DISCOVERY_RANGE = "SUBNET"
[System.Environment]::SetEnvironmentVariable("ROS_LOCALHOST_ONLY", $null, "Process")
$env:PROGRAMME_WINDOWS_HOST_ONLY_IP = $resolvedWindowsHostOnlyIp
$env:PROGRAMME_VM_HOST_ONLY_IP = $resolvedVmHostOnlyIp
$env:CYCLONEDDS_URI = $ddsRuntimeFile

if ($WarmupRosGraph -and (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    $stopOk = Invoke-Ros2DaemonCommand -Subcommand "stop" -TimeoutSec $RosDaemonCmdTimeoutSec
    Start-Sleep -Milliseconds 200
    $startOk = Invoke-Ros2DaemonCommand -Subcommand "start" -TimeoutSec $RosDaemonCmdTimeoutSec
    if (-not ($stopOk -and $startOk)) {
        Write-Warning "ROS graph warmup partially failed or timed out (stop=$stopOk, start=$startOk). Continuing."
    }
}

Write-Host "ROS2 Windows environment ready."
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "RMW_IMPLEMENTATION=$env:RMW_IMPLEMENTATION"
Write-Host "CYCLONEDDS_URI=$env:CYCLONEDDS_URI"
Write-Host "PROGRAMME_WINDOWS_HOST_ONLY_IP=$env:PROGRAMME_WINDOWS_HOST_ONLY_IP"
Write-Host "PROGRAMME_VM_HOST_ONLY_IP=$env:PROGRAMME_VM_HOST_ONLY_IP"
Write-Host "GZ_SIM_RESOURCE_PATH=$env:GZ_SIM_RESOURCE_PATH"
if ($CleanConda) {
    Write-Host "CONDA_CLEAN=enabled"
}
if ($PixiOpenSslBin) {
    Write-Host "PIXi_OPENSSL_BIN=$PixiOpenSslBin"
}
if ($WarmupRosGraph) {
    Write-Host "ROS_GRAPH_WARMUP=enabled"
}
