@echo off
rem ============================================================================
rem  run_scope.bat -- J280 waveform scope launcher (摆杆/摆臂 波形上位机)
rem ----------------------------------------------------------------------------
rem  ASCII-only on purpose: cmd uses the OEM code page (936/GBK here), so a
rem  UTF-8 .bat with Chinese comments gets mangled or breaks parsing.
rem  Chinese documentation lives in 文档\波形上位机使用说明.txt
rem
rem  What it does:
rem    1) find a Python on this machine (PATH first, then the usual installs)
rem    2) run a quick self test (no hardware needed)
rem    3) start the GUI
rem  If no Python is found, double-click J280_Scope.exe instead (no Python
rem  needed, pyserial is already bundled).
rem ----------------------------------------------------------------------------
rem  NOTE: this app and J280_Host.exe CANNOT open the same COM port at the
rem        same time. Close the other one first.
rem ============================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

set PY=
where python >nul 2>nul && set PY=python
if "!PY!"=="" (
    for %%D in (
        "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
        "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
        "%LOCALAPPDATA%\Programs\Python\Python310\python.exe"
        "C:\Python312\python.exe"
        "C:\Python311\python.exe"
        "C:\Python310\python.exe"
    ) do (
        if "!PY!"=="" if exist %%D set PY=%%~D
    )
)

if "!PY!"=="" (
    echo.
    echo [No Python found] Please double-click J280_Scope.exe instead.
    echo.
    pause
    exit /b 1
)

echo Using Python: !PY!
echo.
"!PY!" wave_scope.py --selftest
if errorlevel 1 (
    echo.
    echo [Self test FAILED] pyserial missing? try:  "!PY!" -m pip install pyserial
    echo Or double-click J280_Scope.exe (pyserial already bundled).
    pause
    exit /b 1
)

echo.
"!PY!" wave_scope.py %*
pause
endlocal
