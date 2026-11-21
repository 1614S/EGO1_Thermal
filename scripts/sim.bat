@echo off
rem =====================================================================
rem sim.bat  <tb_name> [gui]
rem   sim.bat tb_uart            compile + run + print PASS/FAIL
rem   sim.bat tb_top_smoke gui   open with waveform
rem =====================================================================
setlocal
set VIVADO_BIN=D:\2026-1\FPGA\2026.1\Vivado\bin
set ROOT=%~dp0..
set TB=%1
if "%TB%"=="" set TB=tb_top_smoke

if not exist "%ROOT%\work" mkdir "%ROOT%\work"
cd /d "%ROOT%\work"

call "%VIVADO_BIN%\xvlog.bat" -sv -d SIM_MODEL -i "%ROOT%\rtl\pkg" -i "%ROOT%\rtl\vplant" -i "%ROOT%\sim\tb" -f "%ROOT%\sim\filelist.f"
if errorlevel 1 goto err

call "%VIVADO_BIN%\xelab.bat" %TB% -s %TB%_sim -timescale 1ns/1ps
if errorlevel 1 goto err

if /i "%2"=="gui" (
    call "%VIVADO_BIN%\xsim.bat" %TB%_sim -gui
) else (
    call "%VIVADO_BIN%\xsim.bat" %TB%_sim -runall
    if errorlevel 1 goto err
)
goto end

:err
echo.
echo [SIM FAILED] %TB%
exit /b 1

:end
endlocal
