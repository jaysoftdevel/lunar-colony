@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "CONFIG=%~1"
if "%CONFIG%"=="" set "CONFIG=Release"
if /I "%CONFIG%"=="--tests" set "CONFIG=Release"
if /I not "%CONFIG%"=="Debug" if /I not "%CONFIG%"=="Release" if /I not "%CONFIG%"=="RelWithDebInfo" if /I not "%CONFIG%"=="MinSizeRel" (
    echo Usage: configure.bat [Debug^|Release^|RelWithDebInfo^|MinSizeRel] [--tests]
    exit /b 2
)
set "TESTS=OFF"
if /I "%~1"=="--tests" set "TESTS=ON"
if /I "%~2"=="--tests" set "TESTS=ON"
where cmake >nul 2>nul
if errorlevel 1 (
    echo Required tool 'cmake' was not found in PATH.
    exit /b 127
)
cmake -S . -B build\windows -G "Visual Studio 17 2022" -A x64 -DGAME_BUILD_NAVIGATION_TESTS=%TESTS%
if errorlevel 1 exit /b 1
echo Configured Windows/Win32/Direct3D 12: build\windows ^(%CONFIG%, tests=%TESTS%^)
exit /b 0
