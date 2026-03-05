param(
    [string]$RosSetup = "C:\\pixi_ws\\ros2-windows\\ros2-windows\\local_setup.bat",
    [int]$DomainId = 0,
    [string]$WorkspaceRoot = "",
    [string[]]$Packages = @("tactile_interfaces", "tactile_vision", "tactile_bringup"),
    [switch]$Clean = $false
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = (Resolve-Path (Join-Path $scriptDir "..\\..")).Path
if (-not $WorkspaceRoot) {
    $WorkspaceRoot = Join-Path $projectRoot "ros2_ws"
}

if (-not (Test-Path $WorkspaceRoot)) {
    Write-Error "Workspace root not found: $WorkspaceRoot"
    exit 1
}

. (Join-Path $scriptDir "env_ros2_windows.ps1") -RosSetup $RosSetup -DomainId $DomainId

function Import-VsBuildEnv {
    if ((Get-Command cl.exe -ErrorAction SilentlyContinue) -and $env:VisualStudioVersion) {
        return $true
    }

    $vswhereCandidates = @(
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\\Installer\\vswhere.exe"),
        (Join-Path $env:ProgramFiles "Microsoft Visual Studio\\Installer\\vswhere.exe")
    ) | Where-Object { $_ -and (Test-Path $_) }

    $vswhereList = @($vswhereCandidates)

    if ($vswhereList.Count -eq 0) {
        Write-Error "vswhere.exe not found. Install Visual Studio Build Tools first."
        Write-Host "Install hint:"
        Write-Host "  winget install -e --id Microsoft.VisualStudio.2022.BuildTools --override `"--wait --passive --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended`""
        return $false
    }

    $vswhere = $vswhereList[0]
    $allInstallPaths = @(
        & $vswhere -all -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    ) | Where-Object { $_ }

    if ($allInstallPaths.Count -eq 0) {
        Write-Error "No Visual Studio instance with VC++ tools found."
        Write-Host "Install hint:"
        Write-Host "  winget install -e --id Microsoft.VisualStudio.2022.BuildTools --override `"--wait --passive --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended`""
        return $false
    }

    $devCmdCandidates = @()
    foreach ($installPath in $allInstallPaths) {
        $devCmdCandidates += @(
            (Join-Path $installPath "Common7\\Tools\\VsDevCmd.bat"),
            (Join-Path $installPath "Common7\\Tools\\LaunchDevCmd.bat")
        )
    }
    $devCmdCandidates += @(
        (Join-Path $env:ProgramFiles "Microsoft Visual Studio\\2022\\BuildTools\\Common7\\Tools\\VsDevCmd.bat"),
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\\2022\\BuildTools\\Common7\\Tools\\VsDevCmd.bat"),
        (Join-Path $env:ProgramFiles "Microsoft Visual Studio\\2022\\Community\\Common7\\Tools\\VsDevCmd.bat"),
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\\2022\\Community\\Common7\\Tools\\VsDevCmd.bat")
    )
    $devCmdCandidates = @($devCmdCandidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -Unique)

    if ($devCmdCandidates.Count -eq 0) {
        Write-Error "VC tools env script not found in discovered VS instances."
        Write-Host "Detected VS install paths:"
        foreach ($p in $allInstallPaths) {
            Write-Host "  - $p"
        }
        Write-Host "Install/repair Visual Studio C++ workload, then rerun."
        Write-Host "Install hint:"
        Write-Host "  winget install -e --id Microsoft.VisualStudio.2022.BuildTools --override `"--wait --passive --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended`""
        return $false
    }

    $devCmd = $devCmdCandidates[0]
    $dump = cmd.exe /c "call `"$devCmd`" -arch=x64 -host_arch=x64 && set"
    foreach ($line in $dump) {
        if ($line -match "^(.*?)=(.*)$") {
            $name = $matches[1]
            $value = $matches[2]
            if ($name) {
                [System.Environment]::SetEnvironmentVariable($name, $value, "Process")
            }
        }
    }

    if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
        Write-Error "MSVC compiler still unavailable after loading DevCmd."
        return $false
    }

    Write-Host "MSVC build environment ready."
    return $true
}

if (-not (Import-VsBuildEnv)) {
    exit 1
}

Push-Location $WorkspaceRoot
try {
    if ($Clean) {
        Remove-Item -Recurse -Force build, install, log -ErrorAction SilentlyContinue
    }

    $args = @(
        "build",
        "--merge-install",
        "--symlink-install",
        "--packages-select"
    ) + $Packages

    Write-Host "Running: colcon $($args -join ' ')"
    & colcon @args
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}

Write-Host "Build finished successfully."
