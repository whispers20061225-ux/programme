@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%start_tactile_grasp_studio.ps1"
exit /b %ERRORLEVEL%
