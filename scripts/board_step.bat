@echo off
rem ============================================================
rem board_step.bat -- cmd entry for the setpoint-step test
rem
rem Usage (from repo root):
rem   scripts\board_step.bat -Port COM3 -StepTo 36.0 -PreSeconds 15 -PostSeconds 70
rem
rem Picks PowerShell 7 (pwsh) if available, else Windows PowerShell 5.1.
rem ============================================================
setlocal
set "HERE=%~dp0"

where pwsh >nul 2>nul
if %errorlevel%==0 (
    pwsh -NoProfile -ExecutionPolicy Bypass -File "%HERE%board_step_test.ps1" %*
    exit /b %errorlevel%
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%HERE%board_step_test.ps1" %*
exit /b %errorlevel%
