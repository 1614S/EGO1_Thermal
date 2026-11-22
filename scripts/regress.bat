@echo off
rem =====================================================================
rem regress.bat ???? ?????鼶 + ??????????????? PASS/FAIL ???
rem
rem   regress.bat          ???????鼶??? 1 ?????
rem   regress.bat full     ?????????з??? 3?C5 ?????
rem
rem ??? TB ????? -s <top> ???? elaborate???κ???????????????
rem ?????"?????黵??"????????
rem =====================================================================
setlocal enabledelayedexpansion
set ROOT=%~dp0..
set IVERILOG=
for %%p in ("C:\iverilog\bin\iverilog.exe" "D:\Apps\iverilog\bin\iverilog.exe") do if not defined IVERILOG if exist %%p set IVERILOG=%%~p
if not defined IVERILOG for %%i in (iverilog.exe) do set IVERILOG=%%~$PATH:i
set VVP=%IVERILOG:iverilog.exe=vvp.exe%
if not exist "%IVERILOG%" (
    echo [ERR] ????? Icarus Verilog: %IVERILOG%
    echo       ???? Vivado ???xelab ??? top???? xsim -runall
    exit /b 1
)
if not exist "%ROOT%\.work\iv" mkdir "%ROOT%\.work\iv"

set SRC=
for %%f in ("%ROOT%\rtl\common\*.v") do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\clk\*.v")    do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\acquire\*.v")do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\dsp\*.v")    do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\ctrl\*.v")   do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\load\*.v")   do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\telemetry\*.v") do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\ui\*.v")     do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\vplant\*.v") do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\rtl\top\*.v")    do set SRC=!SRC! "%%f"
for %%f in ("%ROOT%\sim\models\*.v") do set SRC=!SRC! "%%f"

set PASS=0
set FAIL=0
set REPORT=%ROOT%\.work\regress_report.txt
if exist "%REPORT%" del "%REPORT%"

call :run tb_tick         "%ROOT%\sim\tb\tb_units_clk.sv"
call :run tb_fmon         "%ROOT%\sim\tb\tb_units_clk.sv"
call :run tb_ma           "%ROOT%\sim\tb\tb_units_dsp.sv"
call :run tb_iir          "%ROOT%\sim\tb\tb_units_dsp.sv"
call :run tb_calib        "%ROOT%\sim\tb\tb_units_dsp.sv"
call :run tb_pwm          "%ROOT%\sim\tb\tb_units_dsp.sv"
call :run tb_sup          "%ROOT%\sim\tb\tb_units_ctrl.sv"
call :run tb_gear         "%ROOT%\sim\tb\tb_units_ctrl.sv"
call :run tb_acq          "%ROOT%\sim\tb\tb_units_io.sv"
call :run tb_frame        "%ROOT%\sim\tb\tb_units_io.sv"
call :run tb_cmd          "%ROOT%\sim\tb\tb_cmd.sv"
call :run tb_uart115      "%ROOT%\sim\tb\tb_units_io.sv"
call :run tb_vplant_open  "%ROOT%\sim\tb\tb_vplant_open.sv"
call :run tb_vplant_if    "%ROOT%\sim\tb\tb_vplant_if.sv"
call :run tb_top_smoke    "%ROOT%\sim\tb\tb_top_smoke.sv"
call :run tb_ext_loop     "%ROOT%\sim\tb\tb_ext_loop.sv"
call :run tb_vplant_sys   "%ROOT%\sim\tb\tb_vplant_sys.sv" 10000

if /i "%~1"=="full" call :closed

rem ---- 帧格式交叉验证：RTL 的 frame_pack 与 Python 的 host_logger 必须解出同样字段 ----
where python >nul 2>nul
if errorlevel 1 goto cmp_skip
python "%ROOT%\sw\check_frame_compat.py" > "%ROOT%\.work\iv\compat.log" 2>&1
if errorlevel 1 goto cmp_fail
echo [PASS] 帧格式交叉验证>>"%REPORT%"
set /a PASS+=1
goto cmp_done
:cmp_fail
echo [FAIL] 帧格式交叉验证>>"%REPORT%"
set /a FAIL+=1
goto cmp_done
:cmp_skip
echo [SKIP] python 不在 PATH，跳过帧格式交叉验证>>"%REPORT%"
:cmp_done

echo.
echo ================= ????? =================
type "%REPORT%"
echo ============================================
echo   PASS=%PASS%  FAIL=%FAIL%
if not "%FAIL%"=="0" exit /b 1
endlocal
exit /b 0

rem %1=top  %2=tb???  %3=??? SIM_ACCEL
:run
set TB=%~1
set ACC=
if not "%~3"=="" set ACC=-DSIM_ACCEL=%~3
"%IVERILOG%" -g2012 -DSIM_MODEL %ACC% -I "%ROOT%\rtl\pkg" -I "%ROOT%\rtl\vplant" -I "%ROOT%\sim\tb" ^
    -s %TB% -o "%ROOT%\.work\iv\%TB%.vvp" !SRC! "%~2" > "%ROOT%\.work\iv\%TB%.build" 2>&1
if errorlevel 1 (
    echo [BUILD FAIL] %TB%  ^(?? .work\iv\%TB%.build^)>>"%REPORT%"
    set /a FAIL+=1
    goto :eof
)
"%VVP%" "%ROOT%\.work\iv\%TB%.vvp" > "%ROOT%\.work\iv\%TB%.log" 2>&1
findstr /R /C:" : PASS " /C:"TEST PASSED" /C:"==== PASS" "%ROOT%\.work\iv\%TB%.log" >nul
if errorlevel 1 (
    echo [FAIL] %TB%  ^(?? .work\iv\%TB%.log^)>>"%REPORT%"
    set /a FAIL+=1
) else (
    echo [PASS] %TB%>>"%REPORT%"
    set /a PASS+=1
)
goto :eof

:closed
echo ---- ???????з??????? 3 min??----
"%IVERILOG%" -g2012 -DSIM_MODEL -DSIM_ACCEL=10000 -I "%ROOT%\rtl\pkg" -I "%ROOT%\rtl\vplant" -I "%ROOT%\sim\tb" ^
    -s tb_closed_loop -o "%ROOT%\.work\iv\tb_closed_loop.vvp" !SRC! "%ROOT%\sim\tb\tb_closed_loop.sv" > "%ROOT%\.work\iv\tb_closed_loop.build" 2>&1
if errorlevel 1 (
    echo [BUILD FAIL] tb_closed_loop>>"%REPORT%"
    set /a FAIL+=1
    goto :eof
)
"%VVP%" "%ROOT%\.work\iv\tb_closed_loop.vvp" > "%ROOT%\.work\iv\tb_closed_loop.log" 2>&1
findstr /R /C:" : PASS " /C:"TEST PASSED" /C:"==== PASS" "%ROOT%\.work\iv\tb_closed_loop.log" >nul
if errorlevel 1 (
    echo [FAIL] tb_closed_loop>>"%REPORT%"
    set /a FAIL+=1
) else (
    echo [PASS] tb_closed_loop>>"%REPORT%"
    set /a PASS+=1
)
goto :eof

