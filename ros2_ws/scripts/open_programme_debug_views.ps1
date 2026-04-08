param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ForwardArgs
)

$scriptPath = Join-Path $PSScriptRoot "open_tactile_debug_views.ps1"
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $scriptPath @ForwardArgs
exit $LASTEXITCODE
