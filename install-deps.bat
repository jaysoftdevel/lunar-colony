@echo off
setlocal EnableExtensions
where winget >nul 2>nul
if errorlevel 1 (
    echo winget was not found. Install manually:
    echo   - Visual Studio 2022 with "Desktop development with C++"
    echo   - Windows 11 SDK / Windows SDK 10.0.26100 or newer
    echo   - CMake 3.20 or newer
    exit /b 1
)
echo Installing CMake...
winget install --id Kitware.CMake --exact --accept-package-agreements --accept-source-agreements
if errorlevel 1 exit /b %errorlevel%
echo.
echo Visual Studio's C++ workload and Windows SDK may require Visual Studio Installer.
echo Required workload: Desktop development with C++
echo Required components: MSVC v143 x64/x86 tools and Windows 11 SDK.
exit /b 0
