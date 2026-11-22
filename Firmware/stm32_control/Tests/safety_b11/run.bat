@echo off
setlocal
set "B11_COMPILER=%~1"
if not defined B11_COMPILER set "B11_COMPILER=gcc"
set "B11_OUTPUT=%~dp0..\..\build\safety_tests"
if not exist "%B11_OUTPUT%" mkdir "%B11_OUTPUT%"
if errorlevel 1 exit /b 1
call :run_case test_safety ""
if errorlevel 1 exit /b 1
call :run_case test_auto1 "-DNAV_AUTO2_ENABLED=0"
if errorlevel 1 exit /b 1
call :run_case test_auto2_turn_left "-DNAV_EXPERIMENTAL_TURN_ENABLED=1 -DNAV_TURN_LEFT=1"
if errorlevel 1 exit /b 1
call :run_case test_auto2_turn_right "-DNAV_EXPERIMENTAL_TURN_ENABLED=1 -DNAV_TURN_LEFT=0"
if errorlevel 1 exit /b 1
call :run_case test_navigation_legacy "-DNAV_AUTO1_ODOMETRY=0"
if errorlevel 1 exit /b 1
call :run_case test_turn_left "-DNAV_AUTO1_ODOMETRY=0 -DNAV_EXPERIMENTAL_TURN_ENABLED=1 -DNAV_TURN_LEFT=1"
if errorlevel 1 exit /b 1
call :run_case test_turn_right "-DNAV_AUTO1_ODOMETRY=0 -DNAV_EXPERIMENTAL_TURN_ENABLED=1 -DNAV_TURN_LEFT=0"
if errorlevel 1 exit /b 1
exit /b 0

:run_case
set "B77_LOG=%B11_OUTPUT%\%~1_results.log"
if "%~1"=="test_safety" set "B77_LOG=%B11_OUTPUT%\results.log"
"%B11_COMPILER%" -std=c11 -Wall -Wextra -Werror -finstrument-functions -msse2 -mfpmath=sse %~2 -I"%~dp0mocks" -I"%~dp0..\..\Core\Inc" "%~dp0test_safety.c" -o "%B11_OUTPUT%\%~1.exe"
if errorlevel 1 exit /b 1
"%B11_OUTPUT%\%~1.exe" > "%B77_LOG%"
set "B77_EXIT=%ERRORLEVEL%"
type "%B77_LOG%"
exit /b %B77_EXIT%
