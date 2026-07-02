@echo off
chcp 65001 > nul

:: ========================================
:: Configuration: EDGE or CHROME
set PREFERRED_BROWSER=CHROME
:: ========================================

echo ========================================
echo  Pybricks BLE Monitor
echo ========================================
echo.
echo Preferred browser: %PREFERRED_BROWSER%
echo.

set HTML_PATH=%~dp0index.html

if "%PREFERRED_BROWSER%"=="EDGE" goto TRY_EDGE
goto TRY_CHROME

:TRY_EDGE
if exist "C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe" (
    set "BROWSER=C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
    goto LAUNCH
)
if exist "C:\Program Files\Microsoft\Edge\Application\msedge.exe" (
    set "BROWSER=C:\Program Files\Microsoft\Edge\Application\msedge.exe"
    goto LAUNCH
)
echo Edge not found, trying Chrome...

:TRY_CHROME
if exist "C:\Program Files\Google\Chrome\Application\chrome.exe" (
    set "BROWSER=C:\Program Files\Google\Chrome\Application\chrome.exe"
    goto LAUNCH
)
if exist "C:\Program Files (x86)\Google\Chrome\Application\chrome.exe" (
    set "BROWSER=C:\Program Files (x86)\Google\Chrome\Application\chrome.exe"
    goto LAUNCH
)
if "%PREFERRED_BROWSER%"=="CHROME" goto TRY_EDGE
echo.
echo Chrome or Edge not found.
pause
exit /b 1

:LAUNCH
echo Using: %BROWSER%
echo Launching...
echo.
"%BROWSER%"  "%HTML_PATH%"
echo.
echo Browser closed.
pause
