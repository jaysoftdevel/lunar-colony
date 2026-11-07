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
call "%~dp0configure.bat" "%CONFIG%"
if errorlevel 1 exit /b %ERRORLEVEL%
set "OTHER_CONFIG=Debug"
if /I "%CONFIG%"=="Debug" set "OTHER_CONFIG=Release"
for %%C in (%CONFIG% %OTHER_CONFIG%) do (
    cmake --build build\windows --config %%C --parallel %GAME_BUILD_JOBS% %CLEAN_ARG%
    if errorlevel 1 exit /b 1
)
if not exist "build\windows\bin\game.exe" exit /b 1
if not exist "build\windows\bin\game-debug.exe" exit /b 1
echo.
echo Release: %CD%\build\windows\bin\game.exe
echo Debug:   %CD%\build\windows\bin\game-debug.exe
exit /b 0
