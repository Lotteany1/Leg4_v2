@echo off
setlocal
if not defined VSCMD_VER call "E:\DevelopLotteany1\VisualStudio2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
pushd "%~dp0\..\.."
set "PACKAGE=%~f1"
set "TINYMPC14_OBJECT_DIR=build\host\objects_%RANDOM%_%RANDOM%"
mkdir "%TINYMPC14_OBJECT_DIR%"
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /W4 /utf-8 /O2 /I"%PACKAGE%" /Iverify/host /Ibuild/host /I..\..\Wheel_Leg_Code\Code\Math /I..\Tool\solver "%PACKAGE%\tinympc14.c" "%PACKAGE%\tinympc14_data.c" "%PACKAGE%\tinympc14_schedule.c" "%PACKAGE%\tinympc14_cost.c" ..\..\Wheel_Leg_Code\Code\Math\Leg_Comp.c ..\..\Wheel_Leg_Code\Code\Math\Leg_SpringF.c verify/host/tinympc14_port_host.c verify/host/tinympc14_solver_host_test.c /Fo%TINYMPC14_OBJECT_DIR%\ /Fe:build\host\tinympc14_split_test.exe
if errorlevel 1 (popd & exit /b 1)
build\host\tinympc14_split_test.exe
set "RESULT=%ERRORLEVEL%"
popd
exit /b %RESULT%
