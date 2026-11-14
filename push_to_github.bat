@echo off
rem ============================================================
rem push_to_github.bat —— 首次推送（仓库已由本地提交准备好）
rem 运行前提：本机已登录 GitHub（HTTPS 会弹凭据窗口，或已配 SSH）
rem ============================================================
setlocal
cd /d "%~dp0"

where git >nul 2>nul
if errorlevel 1 (
    echo [ERROR] 未找到 git，请先安装 Git for Windows
    pause
    exit /b 1
)

echo ---- 当前远端 ----
git remote -v

echo.
echo ---- 待推送提交 ----
git log --oneline -n 5

echo.
echo ---- 开始推送 ----
git push -u origin main
if errorlevel 1 (
    echo.
    echo [FAILED] 推送失败。常见原因：
    echo   1^) 没登录 GitHub  ^(浏览器弹窗取消过^)
    echo   2^) 远端已有提交，需要先执行: git pull --rebase origin main
    echo   3^) 网络/代理问题
    pause
    exit /b 1
)

echo.
echo [OK] 已推送到 https://github.com/1614S/EGO1_Thermal
git log --oneline -n 1
pause
