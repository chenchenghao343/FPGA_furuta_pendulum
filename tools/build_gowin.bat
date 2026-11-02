@echo off
rem ============================================================================
rem  文件名 : build_gowin.bat
rem  项目   : J280 旋转倒立摆 FPGA 控制系统
rem  功能   : 一键调用 Gowin gw_sh 完成 综合 → 布局布线 → 生成位流
rem ----------------------------------------------------------------------------
rem  ★★ 红线：Gowin 对中文路径支持极差 ★★
rem     1) 本工程必须放在纯英文路径下（例如 C:\J280_work\Balance_Ctrl），
rem        不要放在 C:\Users\一元\Desktop\新整理参考 这类含中文的目录里；
rem     2) HOME 环境变量必须重定向到纯英文路径，否则 Gowin 写临时文件时
rem        会因路径乱码化失败；
rem     3) 本脚本会自动把工程复制到 %WORK% 指向的纯英文目录再编译。
rem ----------------------------------------------------------------------------
rem  用法：双击运行，或在 cmd 中执行  tools\build_gowin.bat
rem ============================================================================

setlocal

rem ---- 纯英文工作目录（可自行修改）----
set WORK=C:\J280_work
set PRJ=%WORK%\Balance_Ctrl

rem ---- Gowin 安装路径（本机实测路径）----
set GW_SH=D:\Gowin_V1.9\Gowin\Gowin_V1.9.12.02_SP2_x64\IDE\bin\gw_sh.exe

rem ---- 本脚本所在目录的上一级 = 工程根目录 ----
set ROOT=%~dp0..
pushd "%ROOT%"
set ROOT=%CD%
popd

echo ============================================================
echo  J280 Balance_Ctrl  Gowin 一键编译
echo ------------------------------------------------------------
echo  工程根目录 : %ROOT%
echo  纯英文工作目录 : %PRJ%
echo ============================================================
echo.

if not exist "%GW_SH%" (
    echo [ERROR] 找不到 gw_sh.exe：
    echo         %GW_SH%
    echo         请修改本脚本顶部的 GW_SH 变量为实际安装路径。
    pause
    exit /b 1
)

rem ---------------------------------------------------------------------------
rem  把工程镜像到纯英文目录
rem ---------------------------------------------------------------------------
echo [1/3] 镜像工程到纯英文路径 ...
if not exist "%WORK%" mkdir "%WORK%"
if not exist "%PRJ%"  mkdir "%PRJ%"
xcopy "%ROOT%\src" "%PRJ%\src\" /E /I /Y /Q >nul
if errorlevel 1 (
    echo [ERROR] 复制 src 失败。若目标目录无写权限，请改用其它纯英文目录。
    pause
    exit /b 1
)

rem ---------------------------------------------------------------------------
rem  HOME 重定向 + 调用 gw_sh
rem ---------------------------------------------------------------------------
echo [2/3] 设置 HOME=%WORK% 并调用 gw_sh ...
cd /d "%PRJ%"
copy /Y "%ROOT%\tools\flow.tcl" "%PRJ%\flow.tcl" >nul

set HOME=%WORK%
set USERPROFILE=%WORK%

"%GW_SH%" flow.tcl
set GWEXIT=%ERRORLEVEL%

echo.
if %GWEXIT% NEQ 0 (
    echo [FAIL] gw_sh 返回 %GWEXIT%，综合未通过。请查看上面输出定位错误。
    pause
    exit /b %GWEXIT%
)

rem ---------------------------------------------------------------------------
rem  检查产物
rem ---------------------------------------------------------------------------
echo [3/3] 检查产物 ...
if exist "%PRJ%\impl\pnr\Balance_Ctrl.fs" (
    echo [OK] 位流已生成：%PRJ%\impl\pnr\Balance_Ctrl.fs
) else (
    echo [WARN] 未找到 .fs 位流文件，请检查综合日志。
)

if exist "%PRJ%\impl\gwsynthesis\Balance_Ctrl_syn_resource.html" (
    echo [OK] 资源报告：%PRJ%\impl\gwsynthesis\Balance_Ctrl_syn_resource.html
)

echo.
echo 完成。把 .fs 文件用 Gowin Programmer 下载到 SRAM 即可上板验证。
pause
endlocal
