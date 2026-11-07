@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "CONFIG=%~1"
if "%CONFIG%"=="" set "CONFIG=Release"
call "%~dp0build.bat" "%CONFIG%" --tests
if errorlevel 1 exit /b %ERRORLEVEL%
set "OTHER_CONFIG=Debug"
if /I "%CONFIG%"=="Debug" set "OTHER_CONFIG=Release"
for %%C in (%CONFIG% %OTHER_CONFIG%) do (
    ctest --test-dir build\windows -C %%C --output-on-failure
    if errorlevel 1 exit /b 1
)
exit /b 0
