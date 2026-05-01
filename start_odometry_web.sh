#!/usr/bin/env bash

set -euo pipefail

SERIAL_PORT="${1:-}"
SANDBOX=""

if [[ "${SERIAL_PORT}" == "sandbox" ]]; then
  SANDBOX=1
  SERIAL_PORT=""
fi

if [[ -z "${SERIAL_PORT}" ]]; then
  SERIAL_PORT="/dev/ttyACM0"
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"

echo "Starting SnoBot odometry web viewer..."
if [[ -n "${SANDBOX}" ]]; then
  echo "Serial: sandbox mode"
else
  echo "Serial: ${SERIAL_PORT}"
fi
echo "URL: http://localhost:3000"
echo "Rates: publish=15Hz subscribe=20Hz telemetry=20Hz"
echo

if [[ -n "${SANDBOX}" ]]; then
  exec "${PYTHON_BIN}" src/tools/odometry_web_view.py \
    --sandbox \
    --web-port 3000 \
    --publish-rate 15 \
    --subscribe-rate 20 \
    --telemetry-rate 20 \
    --open-browser \
    --no-browser-app
else
  exec "${PYTHON_BIN}" src/tools/odometry_web_view.py \
    --port "${SERIAL_PORT}" \
    --web-port 3000 \
    --publish-rate 15 \
    --subscribe-rate 20 \
    --telemetry-rate 20 \
    --open-browser \
    --no-browser-app
fi
