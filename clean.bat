@echo off
setlocal EnableExtensions
cd /d "%~dp0"
if not exist "build\windows\CMakeCache.txt" goto done
for %%C in (Release Debug RelWithDebInfo MinSizeRel) do (
    cmake --build build\windows --config %%C --target clean
    if errorlevel 1 exit /b 1
)
:done
exit /b 0
