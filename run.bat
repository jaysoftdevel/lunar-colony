@echo off
setlocal EnableExtensions
cd /d "%~dp0"
if exist "build\windows\bin\game.exe" goto run

echo No Windows executable found. Building Release first...
call build.bat Release
if errorlevel 1 exit /b 1

:run
"build\windows\bin\game.exe" %*
exit /b %ERRORLEVEL%
