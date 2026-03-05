param(
    [int]$DomainId = 0,
    [string]$RosSetup = "C:\\opt\\ros\\jazzy\\x64\\local_setup.ps1",
    [string]$WorkspaceSetup = "",
    [bool]$CleanConda = $true,
    [string]$PixiOpenSslBin = "",
    [bool]$WarmupRosGraph = $false
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

$env:ROS_DOMAIN_ID = "$DomainId"
$env:RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
$env:ROS_AUTOMATIC_DISCOVERY_RANGE = "SUBNET"
[System.Environment]::SetEnvironmentVariable("ROS_LOCALHOST_ONLY", $null, "Process")
$env:CYCLONEDDS_URI = $ddsRuntimeFile

if ($WarmupRosGraph -and (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    try {
        & ros2 daemon stop 1>$null 2>$null
    } catch {
        # ignore
    }
    Start-Sleep -Milliseconds 200
    try {
        & ros2 daemon start 1>$null 2>$null
    } catch {
        # ignore
    }
}

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
if ($WarmupRosGraph) {
    Write-Host "ROS_GRAPH_WARMUP=enabled"
}
