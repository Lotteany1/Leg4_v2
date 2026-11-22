@echo off
setlocal
if not defined VSCMD_VER call "E:\DevelopLotteany1\VisualStudio2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
pushd "%~dp0\..\.."
if /i "%~1"=="online" goto online
if not exist build\host mkdir build\host
set "TINYMPC14_OBJECT_DIR=build\host\objects_%RANDOM%_%RANDOM%"
mkdir "%TINYMPC14_OBJECT_DIR%"
if errorlevel 1 exit /b 1
set "AUDIT_FLAGS=/O2"
set "INCLUDE=..\..\Wheel_Leg_Code\Code\Config;%INCLUDE%"
if "%~1"=="asan" set "AUDIT_FLAGS=/O1 /Zi /fsanitize=address"
rem Leg_Comp.c / Leg_SpringF.c 来自 Tool 的部署副本，先跑 Matlab/Tool/build_leg_tool 再执行本脚本
cl /nologo /std:c11 /W4 /utf-8 %AUDIT_FLAGS% /Ideploy/solver /Ibuild/staging /Iverify/host /Ibuild/host /I..\..\Wheel_Leg_Code\Code\Math /I..\Tool\solver deploy/solver/tinympc14.c build/staging/tinympc14_data.c ..\..\Wheel_Leg_Code\Code\Math\Leg_Comp.c ..\..\Wheel_Leg_Code\Code\Math\Leg_SpringF.c verify/host/tinympc14_port_host.c verify/host/tinympc14_solver_host_test.c /Fo%TINYMPC14_OBJECT_DIR%\ /Fd:build\host\tinympc14_host.pdb /Fe:build\host\tinympc14_solver_test.exe
if errorlevel 1 (popd & exit /b 1)
build\host\tinympc14_solver_test.exe
set "AUDIT_RESULT=%ERRORLEVEL%"
popd
exit /b %AUDIT_RESULT%

:online
set "PACKAGE=%~f2"
set "TESTDIR=%PACKAGE%\host"
cl /nologo /std:c11 /W4 /utf-8 /O2 /I"%PACKAGE%\solver" /Iverify/host /I"%TESTDIR%" /I..\..\Wheel_Leg_Code\Code\Math /I..\Tool\solver "%PACKAGE%\solver\tinympc14.c" "%PACKAGE%\solver\tinympc14_data.c" ..\..\Wheel_Leg_Code\Code\Math\Leg_Comp.c ..\..\Wheel_Leg_Code\Code\Math\Leg_SpringF.c verify/host/tinympc14_port_host.c verify/host/tinympc14_solver_host_test.c /Fo"%TESTDIR%\\" /Fd"%TESTDIR%\tinympc14_online_host.pdb" /Fe"%TESTDIR%\tinympc14_online_test.exe"
if errorlevel 1 (popd & exit /b 1)
"%TESTDIR%\tinympc14_online_test.exe"
set "AUDIT_RESULT=%ERRORLEVEL%"
popd
exit /b %AUDIT_RESULT%
