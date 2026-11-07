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
if exist "build\windows\bin\game.exe" if exist "build\windows\bin\game-debug.exe" goto run
call "%~dp0build.bat"
if errorlevel 1 exit /b %ERRORLEVEL%

:run
"build\windows\bin\%GAME_EXE%" %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %ERRORLEVEL%
