@echo off
setlocal EnableExtensions
cd /d "%~dp0"
if exist "build\windows" rmdir /s /q "build\windows"
echo Removed Windows build tree: %CD%\build\windows
exit /b 0
