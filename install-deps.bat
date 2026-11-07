@echo off
setlocal EnableExtensions
cd /d "%~dp0"

where cmake >nul 2>nul
if not errorlevel 1 goto check_git
where winget >nul 2>nul
if errorlevel 1 goto missing_winget
winget install --id Kitware.CMake --exact --accept-package-agreements --accept-source-agreements
if errorlevel 1 exit /b %ERRORLEVEL%

:check_git
where git >nul 2>nul
if not errorlevel 1 goto audio
where winget >nul 2>nul
if errorlevel 1 goto missing_winget
winget install --id Git.Git --exact --accept-package-agreements --accept-source-agreements
if errorlevel 1 exit /b %ERRORLEVEL%

:audio
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\install-openal.ps1"
if errorlevel 1 exit /b %ERRORLEVEL%

echo.
echo Visual Studio 2022 requires Desktop development with C++, MSVC v143 and the Windows 11 SDK.
echo Open a new command prompt if CMake or Git was just installed.
echo Build both binaries with build.bat.
exit /b 0

:missing_winget
echo Install CMake and Git for Windows, then rerun install-deps.bat. 1>&2
exit /b 127
