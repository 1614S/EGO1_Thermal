@echo off
rem ============================================================
rem capture.bat -- cmd entry for telemetry capture
rem
rem Usage (from repo root):
rem   scripts\capture.bat -Port COM3 -Seconds 30 -Csv results\x.csv
rem   scripts\capture.bat -Port COM3 -Seconds 5
rem
rem It picks PowerShell 7 (pwsh) if available, otherwise falls back
rem to Windows PowerShell 5.1. The .ps1 files carry a UTF-8 BOM so
rem 5.1 can parse their Chinese comments correctly.
rem ============================================================
setlocal
set "HERE=%~dp0"

where pwsh >nul 2>nul
if %errorlevel%==0 (
    pwsh -NoProfile -ExecutionPolicy Bypass -File "%HERE%capture_serial.ps1" %*
    exit /b %errorlevel%
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%HERE%capture_serial.ps1" %*
exit /b %errorlevel%
