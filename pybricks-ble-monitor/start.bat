@echo off
chcp 65001 > nul
setlocal enabledelayedexpansion

:: ========================================
:: Configuration: EDGE or CHROME
set PREFERRED_BROWSER=CHROME
set PORT=8765
:: ========================================

echo ========================================
echo  Pybricks BLE Monitor
echo ========================================
echo.
echo Preferred browser: %PREFERRED_BROWSER%
echo.

set SCRIPT_DIR=%~dp0

:: file://wsl.localhost/... のようなホスト名付きURLで直接index.htmlを開くと、
:: ChromeがそのoriginにFile System Access API(自動保存機能が使う)の権限を
:: 正しく与えないことがあるため、ローカルサーバー経由でhttp://localhostとして
:: 開けるように試みる。Node.js / Pythonのどちらも無ければ、従来通りfile://で開く。

set HTML_PATH=%SCRIPT_DIR%index.html
set SERVER_STARTED=0

where node >nul 2>nul
if %errorlevel%==0 (
    echo Starting local server with Node.js...
    start "Pybricks BLE Monitor Server" /min node "%SCRIPT_DIR%serve.js" %PORT%
    set SERVER_STARTED=1
    goto SERVER_DONE
)

where python >nul 2>nul
if %errorlevel%==0 (
    echo Starting local server with Python...
    start "Pybricks BLE Monitor Server" /min python -m http.server %PORT% --bind 127.0.0.1 --directory "%SCRIPT_DIR%"
    set SERVER_STARTED=1
    goto SERVER_DONE
)

where py >nul 2>nul
if %errorlevel%==0 (
    echo Starting local server with Python (py launcher)...
    start "Pybricks BLE Monitor Server" /min py -m http.server %PORT% --bind 127.0.0.1 --directory "%SCRIPT_DIR%"
    set SERVER_STARTED=1
    goto SERVER_DONE
)

echo Node.js/Python not found. Opening index.html directly (auto-save may not work).

:SERVER_DONE
if %SERVER_STARTED%==1 (
    :: サーバーの起動を待つ
    timeout /t 1 /nobreak > nul
    set HTML_PATH=http://localhost:%PORT%/index.html
)

echo.

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
echo Opening: %HTML_PATH%
echo Launching...
echo.
"%BROWSER%"  "%HTML_PATH%"
echo.
if %SERVER_STARTED%==1 (
    echo Note: A local server window is running in the background for auto-save.
    echo Close that window ^(or this one^) to stop it when you're done.
)
echo Browser closed.
pause
