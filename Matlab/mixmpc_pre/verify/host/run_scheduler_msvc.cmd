@echo off
setlocal
if not defined VSCMD_VER call "E:\DevelopLotteany1\VisualStudio2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
set "SCHEDULER_ROOT=%~dp0..\.."
set "SCHEDULER_DIR=%~f1"
if not defined SCHEDULER_DIR set "SCHEDULER_DIR=%SCHEDULER_ROOT%\build\host"
pushd "%SCHEDULER_DIR%" || exit /b 1
cl /nologo /std:c11 /W4 /utf-8 /O2 /DTINYMPC14_TERMINAL_CODEC_TEST /I"%SCHEDULER_ROOT%\build\staging" "%SCHEDULER_ROOT%\build\staging\tinympc14_data.c" "%~dp0tinympc14_scheduler_host_test.c" /Fe:tinympc14_scheduler_test.exe
if errorlevel 1 (popd & exit /b 1)
.\tinympc14_scheduler_test.exe scheduler_points.bin scheduler_matrices.bin scheduler_coords.bin
set "SCHEDULER_RESULT=%ERRORLEVEL%"
popd
exit /b %SCHEDULER_RESULT%
