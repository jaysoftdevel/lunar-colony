@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "CONFIG=Release"
set "TESTS=%GAME_BUILD_TESTS%"
set "CLEAN_ARG="

:parse
if "%~1"=="" goto parsed
if /I "%~1"=="Release" (
    set "CONFIG=Release"
    goto next
)
if /I "%~1"=="Debug" (
    set "CONFIG=Debug"
    goto next
)
if /I "%~1"=="RelWithDebInfo" (
    set "CONFIG=RelWithDebInfo"
    goto next
)
if /I "%~1"=="MinSizeRel" (
    set "CONFIG=MinSizeRel"
    goto next
)
if /I "%~1"=="--tests" (
    set "TESTS=ON"
    goto next
)
if /I "%~1"=="--clean" (
    set "CLEAN_ARG=--clean-first"
    goto next
)
echo Usage: build.bat [Release^|Debug^|RelWithDebInfo^|MinSizeRel] [--tests] [--clean]
exit /b 2

:next
shift
goto parse

:parsed
set "GAME_BUILD_TESTS=%TESTS%"
call :clock_ms build_start_ms
call "%~dp0configure.bat" "%CONFIG%"
if errorlevel 1 exit /b %ERRORLEVEL%
set "OTHER_CONFIG=Debug"
if /I "%CONFIG%"=="Debug" set "OTHER_CONFIG=Release"
call :build_variant "%CONFIG%" first_ms
if errorlevel 1 exit /b 1
call :build_variant "%OTHER_CONFIG%" second_ms
if errorlevel 1 exit /b 1
if not exist "build\windows\bin\game.exe" exit /b 1
if not exist "build\windows\bin\game-debug.exe" exit /b 1

call :clock_ms build_end_ms
set /a "total_ms=(build_end_ms-build_start_ms+86400000) %% 86400000"
echo.
echo Release: %CD%\build\windows\bin\game.exe
echo Debug:   %CD%\build\windows\bin\game-debug.exe
cmake "-DRUNTIME_DIR=%CD%/build/windows/bin" ^
    "-DFIRST_CONFIG=%CONFIG%" "-DFIRST_MS=%first_ms%" ^
    "-DSECOND_CONFIG=%OTHER_CONFIG%" "-DSECOND_MS=%second_ms%" ^
    "-DTOTAL_MS=%total_ms%" -P "%~dp0scripts\build-summary.cmake"
exit /b %ERRORLEVEL%

:build_variant
call :clock_ms variant_start_ms
cmake --build build\windows --config %~1 --parallel %GAME_BUILD_JOBS% %CLEAN_ARG%
if errorlevel 1 exit /b 1
call :clock_ms variant_end_ms
set /a "%~2=(variant_end_ms-variant_start_ms+86400000) %% 86400000"
exit /b 0

:clock_ms
rem Pad a single-digit hour and force decimal arithmetic for 08 and 09.
for /f "tokens=1-4 delims=:., " %%H in ("%TIME: =0%") do (
    set /a "%~1=((1%%H-100)*3600+(1%%I-100)*60+1%%J-100)*1000+(1%%K-100)*10"
)
exit /b 0
