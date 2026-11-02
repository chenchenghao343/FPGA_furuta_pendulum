@echo off
rem ============================================================================
rem  J280 倒立摆上位机 —— 双击启动（需要本机装有 Python + pyserial）
rem ----------------------------------------------------------------------------
rem  如果本机没有 Python，请直接双击 J280_Host.exe（免 Python 免安装）。
rem  本文件用 GBK 编码保存：中文 Windows 控制台代码页是 936。
rem ============================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

rem ---- 优先使用 PATH 里的 python ----
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
    echo [未找到 Python] 请直接双击 J280_Host.exe —— 它不需要 Python。
    echo.
    pause
    exit /b 1
)

echo 使用 Python: !PY!
echo.
"!PY!" serial_host.py --selftest
if errorlevel 1 (
    echo.
    echo [自检失败] 可能缺少 pyserial，请执行：
    echo     "!PY!" -m pip install pyserial
    echo 或者直接双击 J280_Host.exe（已内置，无需安装）。
    pause
    exit /b 1
)

echo.
"!PY!" serial_host.py %*
pause
endlocal
