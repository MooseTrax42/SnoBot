@echo off
setlocal

set SERIAL_PORT=%1
set SANDBOX=
if /I "%SERIAL_PORT%"=="sandbox" (
  set SANDBOX=1
  set SERIAL_PORT=
)
if "%SERIAL_PORT%"=="" set SERIAL_PORT=COM12

echo Starting SnoBot odometry web viewer...
if defined SANDBOX (
  echo Serial: sandbox mode
) else (
  echo Serial: %SERIAL_PORT%
)
echo URL: http://localhost:3000
echo Rates: publish=15Hz subscribe=20Hz telemetry=20Hz
echo.

if defined SANDBOX (
  py src\tools\odometry_web_view.py --sandbox --web-port 3000 --publish-rate 15 --subscribe-rate 20 --telemetry-rate 20 --open-browser --browser-app
) else (
  py src\tools\odometry_web_view.py --port %SERIAL_PORT% --web-port 3000 --publish-rate 15 --subscribe-rate 20 --telemetry-rate 20 --open-browser --browser-app
)

endlocal
