param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ForwardArgs
)

$scriptPath = Join-Path $PSScriptRoot "tactile_grasp_studio_helper.ps1"
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $scriptPath @ForwardArgs
exit $LASTEXITCODE
