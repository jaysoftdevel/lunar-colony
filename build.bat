@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "CONFIG=Release"
set "TESTS=OFF"
set "CLEAN=0"

:parse
if "%~1"=="" goto parsed
if /I "%~1"=="Debug" goto cfg_debug
if /I "%~1"=="Release" goto cfg_release
if /I "%~1"=="RelWithDebInfo" goto cfg_reldeb
if /I "%~1"=="MinSizeRel" goto cfg_minsize
if /I "%~1"=="--tests" goto flag_tests
if /I "%~1"=="--clean" goto flag_clean
echo Usage: build.bat [Debug^|Release^|RelWithDebInfo^|MinSizeRel] [--tests] [--clean]
exit /b 2

:cfg_debug
set "CONFIG=Debug"
shift
goto parse
:cfg_release
set "CONFIG=Release"
shift
goto parse
:cfg_reldeb
set "CONFIG=RelWithDebInfo"
shift
goto parse
:cfg_minsize
set "CONFIG=MinSizeRel"
shift
goto parse
:flag_tests
set "TESTS=ON"
shift
goto parse
:flag_clean
set "CLEAN=1"
shift
goto parse

:parsed
where cmake >nul 2>nul
if errorlevel 1 (
    echo Required tool 'cmake' was not found in PATH.
    exit /b 127
)
if "%CLEAN%"=="1" if exist "build\windows" rmdir /s /q "build\windows"
cmake -S . -B build\windows -G "Visual Studio 17 2022" -A x64 -DGAME_BUILD_NAVIGATION_TESTS=%TESTS%
if errorlevel 1 exit /b 1
cmake --build build\windows --config %CONFIG% --parallel
if errorlevel 1 exit /b 1
echo.
echo Windows build complete: %CD%\build\windows\bin\game.exe
echo Backend: Win32 + Direct3D 12
echo Run: run.bat
exit /b 0
