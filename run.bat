@echo off
setlocal EnableExtensions
cd /d "%~dp0"
if /I "%~1"=="Debug" goto retired
if /I "%~1"=="--debug" goto retired
if /I "%~1"=="Release" goto retired
if exist "build\windows\bin\game.exe" goto verify
call "%~dp0build.bat"
if errorlevel 1 exit /b %ERRORLEVEL%
:verify
set /p "EXPECTED_VERSION="<VERSION
set "ACTUAL_VERSION="
for /f "delims=" %%V in ('"build\windows\bin\game.exe" --version') do set "ACTUAL_VERSION=%%V"
if not "%ACTUAL_VERSION%"=="game %EXPECTED_VERSION%" (
    echo Rebuild with build.bat --engine PATH before launching version %EXPECTED_VERSION%.
    exit /b 1
)
"build\windows\bin\game.exe" %*
exit /b %ERRORLEVEL%
:retired
echo Select debug features at build time: build.bat [--debug]; then run.bat
exit /b 2
