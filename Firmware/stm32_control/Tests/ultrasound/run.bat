@echo off
setlocal
set "ULTRASOUND_COMPILER=%~1"
if not defined ULTRASOUND_COMPILER set "ULTRASOUND_COMPILER=gcc"
set "ULTRASOUND_OUTPUT=%~dp0..\..\build\ultrasound_tests"
if not exist "%ULTRASOUND_OUTPUT%" mkdir "%ULTRASOUND_OUTPUT%"
if errorlevel 1 exit /b 1
"%ULTRASOUND_COMPILER%" -std=c11 -Wall -Wextra -Werror -msse2 -mfpmath=sse -I"%~dp0mocks" -I"%~dp0..\..\Core\Inc" "%~dp0test_ultrasound.c" -o "%ULTRASOUND_OUTPUT%\test_ultrasound.exe"
if errorlevel 1 exit /b 1
"%ULTRASOUND_OUTPUT%\test_ultrasound.exe" > "%ULTRASOUND_OUTPUT%\results.log"
set "ULTRASOUND_EXIT=%ERRORLEVEL%"
type "%ULTRASOUND_OUTPUT%\results.log"
exit /b %ULTRASOUND_EXIT%
