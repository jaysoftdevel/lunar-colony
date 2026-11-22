@echo off
setlocal EnableExtensions
where py >nul 2>nul
if errorlevel 1 goto python
py -3 "%~dp0scripts\windows-build.py" test %*
exit /b %ERRORLEVEL%
:python
python "%~dp0scripts\windows-build.py" test %*
exit /b %ERRORLEVEL%
