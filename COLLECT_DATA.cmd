@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
title PICO + MANUS + TACTILE DATA COLLECTION

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\collect_windows.ps1" %*
set "COLLECT_EXIT=%ERRORLEVEL%"

echo.
if not "%COLLECT_EXIT%"=="0" (
    echo [FAILED] Collection workflow exited with code %COLLECT_EXIT%.
) else (
    echo [OK] Collection workflow finished.
)
echo Press any key to close this window.
pause >nul
exit /b %COLLECT_EXIT%
