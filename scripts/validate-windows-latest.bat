@echo off
setlocal EnableExtensions
cd /d "%~dp0.."
if not exist VERSION exit /b 2
set /p VER=<VERSION
if not "%VER%"=="0.54.3" (
  echo Expected source version 0.54.3, found %VER%. 1>&2
  exit /b 2
)
call build.bat Release --tests
if errorlevel 1 exit /b 1
ctest --test-dir build\windows -C Release --output-on-failure
if errorlevel 1 exit /b 1
ctest --test-dir build\windows -C Debug --output-on-failure
if errorlevel 1 exit /b 1
echo Windows Release and Debug build/tests passed.
exit /b 0
