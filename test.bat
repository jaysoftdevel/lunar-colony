@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "CONFIG=%~1"
if "%CONFIG%"=="" set "CONFIG=Release"
call build.bat %CONFIG% --tests
if errorlevel 1 exit /b 1
ctest --test-dir build\windows -C %CONFIG% --output-on-failure
exit /b %ERRORLEVEL%
