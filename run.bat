@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "GAME_EXE=game.exe"
if /I "%~1"=="Debug" goto debug
if /I "%~1"=="--debug" goto debug
if /I "%~1"=="Release" shift
goto check
:debug
set "GAME_EXE=game-debug.exe"
shift
:check
if exist "build\windows\bin\game.exe" if exist "build\windows\bin\game-debug.exe" goto verify
call "%~dp0build.bat"
if errorlevel 1 exit /b %ERRORLEVEL%
:verify
set /p "EXPECTED_VERSION="<VERSION
set "ACTUAL_VERSION="
for /f "delims=" %%V in ('"build\windows\bin\%GAME_EXE%" --version') do set "ACTUAL_VERSION=%%V"
if not "%ACTUAL_VERSION%"=="game %EXPECTED_VERSION%" (
    echo Rebuild with build.bat --engine PATH before launching version %EXPECTED_VERSION%.
    exit /b 1
)
"build\windows\bin\%GAME_EXE%" %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %ERRORLEVEL%
