@echo off
rem ============================================================
rem serial_cmd.bat -- send one ASCII command to the board
rem
rem Usage (from repo root):
rem   scripts\serial_cmd.bat "?"
rem   scripts\serial_cmd.bat "S=36.0"
rem   scripts\serial_cmd.bat "Z"
rem   scripts\serial_cmd.bat "?" COM3
rem
rem Binary telemetry frames (A5 5A + 32 bytes) are stripped, only
rem the ASCII reply is printed.
rem ============================================================
setlocal
set "HERE=%~dp0"

where pwsh >nul 2>nul
if %errorlevel%==0 (
    pwsh -NoProfile -ExecutionPolicy Bypass -File "%HERE%serial_cmd.ps1" %*
    exit /b %errorlevel%
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%HERE%serial_cmd.ps1" %*
exit /b %errorlevel%
