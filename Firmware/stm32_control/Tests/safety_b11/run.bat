@echo off
setlocal
set "B11_COMPILER=%~1"
if not defined B11_COMPILER set "B11_COMPILER=gcc"
set "B11_OUTPUT=%~dp0..\..\build\etapa_b11\tests"
if not exist "%B11_OUTPUT%" mkdir "%B11_OUTPUT%"
if errorlevel 1 exit /b 1
"%B11_COMPILER%" -std=c11 -Wall -Wextra -Werror -finstrument-functions -msse2 -mfpmath=sse -I"%~dp0mocks" -I"%~dp0..\..\Core\Inc" "%~dp0test_safety.c" -o "%B11_OUTPUT%\test_safety.exe"
if errorlevel 1 exit /b 1
"%B11_OUTPUT%\test_safety.exe" > "%B11_OUTPUT%\results.log"
set "B11_EXIT=%ERRORLEVEL%"
type "%B11_OUTPUT%\results.log"
exit /b %B11_EXIT%
