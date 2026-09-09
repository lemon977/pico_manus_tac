@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
title DELETE BATCH DATA - KEEP ORIGINAL INDICES

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\delete_batch_data.ps1" %*
set "DELETE_EXIT=%ERRORLEVEL%"

echo.
if not "%DELETE_EXIT%"=="0" (
    echo [FAILED] Batch edit exited with code %DELETE_EXIT%.
) else (
    echo [OK] Batch edit finished.
)
echo Press any key to close this window.
pause >nul
exit /b %DELETE_EXIT%
