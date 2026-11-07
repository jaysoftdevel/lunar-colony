@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "CMD=%~1"
if "%CMD%"=="" set "CMD=build"
if not "%~1"=="" shift
if /I "%CMD%"=="configure" goto do_configure
if /I "%CMD%"=="build" goto do_build
if /I "%CMD%"=="rebuild" goto do_rebuild
if /I "%CMD%"=="run" goto do_run
if /I "%CMD%"=="test" goto do_test
if /I "%CMD%"=="clean" goto do_clean
if /I "%CMD%"=="deps" goto do_deps
echo Usage: dev.bat {configure^|build^|rebuild^|run^|test^|clean^|deps} [arguments]
exit /b 2

:do_configure
call configure.bat %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %ERRORLEVEL%
:do_build
call build.bat %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %ERRORLEVEL%
:do_rebuild
call build.bat %1 %2 %3 %4 %5 %6 %7 %8 %9 --clean
exit /b %ERRORLEVEL%
:do_run
call run.bat %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %ERRORLEVEL%
:do_test
call test.bat %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %ERRORLEVEL%
:do_clean
call clean.bat
exit /b %ERRORLEVEL%
:do_deps
call install-deps.bat %1 %2 %3 %4 %5 %6 %7 %8 %9
exit /b %ERRORLEVEL%
