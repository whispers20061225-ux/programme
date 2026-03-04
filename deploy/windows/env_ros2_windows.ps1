param(
    [int]$DomainId = 0,
    [string]$RosSetup = "C:\\opt\\ros\\jazzy\\x64\\local_setup.ps1",
    [string]$WorkspaceSetup = "",
    [bool]$CleanConda = $true,
    [string]$PixiOpenSslBin = ""
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

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = (Resolve-Path (Join-Path $scriptDir "..\\..")).Path
$ddsFile = (Resolve-Path (Join-Path $projectRoot "config\\dds\\cyclonedds_windows.xml")).Path
$ddsRuntimeFile = Join-Path $env:TEMP "programme_cyclonedds_windows.xml"
Copy-Item -Path $ddsFile -Destination $ddsRuntimeFile -Force

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

if (-not $PixiOpenSslBin -and (Test-Path $RosSetup)) {
    $rosRoot = Split-Path -Parent $RosSetup
    $candidateWorkspace = Split-Path -Parent (Split-Path -Parent $rosRoot)
    $candidatePixiDll = Join-Path $candidateWorkspace ".pixi\\envs\\default\\Library\\bin"
    if (Test-Path $candidatePixiDll) {
        $PixiOpenSslBin = $candidatePixiDll
    }
}

if ($PixiOpenSslBin) {
    Prepend-Path -PathValue $PixiOpenSslBin
}

if ($WorkspaceSetup -and (Test-Path $WorkspaceSetup)) {
    . $WorkspaceSetup
} elseif ($WorkspaceSetup) {
    Write-Warning "Workspace setup script not found: $WorkspaceSetup"
}

$env:ROS_DOMAIN_ID = "$DomainId"
$env:RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
$env:ROS_LOCALHOST_ONLY = "0"
$env:CYCLONEDDS_URI = $ddsRuntimeFile

Write-Host "ROS2 Windows environment ready."
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "RMW_IMPLEMENTATION=$env:RMW_IMPLEMENTATION"
Write-Host "CYCLONEDDS_URI=$env:CYCLONEDDS_URI"
if ($CleanConda) {
    Write-Host "CONDA_CLEAN=enabled"
}
if ($PixiOpenSslBin) {
    Write-Host "PIXi_OPENSSL_BIN=$PixiOpenSslBin"
}
