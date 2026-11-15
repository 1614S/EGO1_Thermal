@echo off
rem =====================================================================
rem build.bat  --  ???????????/??
rem   ???????/????? cd ????????????????
rem =====================================================================
setlocal
cd /d "%~dp0.."
echo [build] project dir = %CD%

call "D:\2026-1\FPGA\2026.1\Vivado\bin\vivado.bat" -mode batch -source scripts\build.tcl -nojournal -nolog
if errorlevel 1 goto err
echo [build] done. bitstream: %CD%\Thermal.runs\impl_1\ego1_thermal_top.bit
goto end

:err
echo [build] FAILED
exit /b 1

:end
endlocal
