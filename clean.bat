@echo off
setlocal EnableExtensions
cd /d "%~dp0"
if not exist "build\windows\CMakeCache.txt" goto done
for %%C in (Release RelWithDebInfo MinSizeRel) do (
    cmake --build build\windows --config %%C --target clean
    if errorlevel 1 exit /b 1
)
:done
for %%F in (game.exe game-debug.exe game-debug.pdb SDL2d.dll debug\0-54-0-debug-start.gmsave) do if exist "build\windows\bin\%%F" del /q "build\windows\bin\%%F"
exit /b 0
