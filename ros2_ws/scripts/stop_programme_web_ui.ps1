param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ForwardArgs
)

$scriptPath = Join-Path $PSScriptRoot "stop_tactile_grasp_studio.ps1"
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $scriptPath @ForwardArgs
exit $LASTEXITCODE
