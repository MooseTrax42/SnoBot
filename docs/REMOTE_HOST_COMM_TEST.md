# Remote-to-Host Communication Test Instructions

This guide verifies communication between the ESP32 handheld remote and the host machine running the SnoBot remote server.

## 1. Goal

Confirm that:
- Bluetooth RFCOMM connection is established.
- Remote commands are received and acknowledged.
- Telemetry is streamed back to the remote.
- Keepalive/watchdog behavior works correctly.

## 2. Prerequisites

- Repo is available on the host machine.
- Python environment is set up for this project.
- ESP32 remote firmware can connect via Bluetooth SPP.
- For real hardware tests: Arduino is connected to host serial port.

## 3. Recommended Test Order

1. Run **dummy transport** test first (isolates Bluetooth/API path).
2. Run **real hardware** test second (includes Arduino serial link).

## 4. Start Host Server

Run from repo root.

### Dummy transport (recommended first)

```bash
python src/main_remote.py --dummy
```

### Real transport (example)

```bash
python src/main_remote.py --port /dev/ttyACM0 --baud 115200
```

Expected startup output includes:
- `Listening on RFCOMM channel ...`
- `Waiting for ESP32 controller connection...`

## 5. One-Time Bluetooth Pairing (Jetson/Linux host)

```bash
bluetoothctl
scan on
pair <ESP32_MAC>
trust <ESP32_MAC>
```

After pairing, connect from ESP32 as SPP client to the host device.

## 6. Command/Response Validation

Send one JSON command per line (`\n` terminated, UTF-8) from ESP32.

### Minimum smoke test

1. Send:
```json
{"cmd":"ping"}
```
Expect:
```json
{"ack":"ping","ok":true}
```

2. Send:
```json
{"cmd":"mode","mode":"MANUAL"}
```
Expect ACK with `"ok":true`.

3. Send:
```json
{"cmd":"vel","v":0.3,"w":0.0}
```
Expect ACK with `"ok":true`.

4. Send:
```json
{"cmd":"stop"}
```
Expect ACK with `"ok":true`.

## 7. Telemetry Validation

While connected, host pushes telemetry automatically (~5 Hz by default), for example:

```json
{"t":1,"st":"MANUAL","v":[0.0,0.0],"...":"..."}
```

Pass condition:
- Remote receives periodic telemetry without manual polling.

## 8. Heartbeat and Watchdog Validation

When idle, ESP32 should send heartbeat approximately every 1 second:

```json
{"cmd":"hb"}
```

Expected behavior:
- No ACK is returned for heartbeat.
- Connection remains active if heartbeat continues.

Watchdog test:
1. Stop sending all commands/heartbeats for more than 3 seconds.
2. Verify host disconnects client due to timeout.
3. Verify safety behavior is triggered (motion stop, auger off, salt off).

## 9. Pass/Fail Criteria

Pass:
- Connection succeeds.
- `ping`, `mode`, `vel`, and `stop` receive valid ACKs.
- Telemetry stream is continuous.
- Watchdog disconnect triggers correctly after RX silence.

Fail examples:
- No connection established.
- ACKs are missing or contain `"ok":false` for valid inputs.
- Telemetry missing or intermittent under stable link.
- Watchdog does not disconnect after prolonged silence.

## 10. Troubleshooting

- Bluetooth unavailable on host:
  - Ensure test is run on Jetson/Linux with Bluetooth support.
  - Confirm Bluetooth service is active.
- `vel` returns `"motion not enabled"`:
  - Set mode to `MANUAL` first and check telemetry `mot` field.
- Frequent disconnects:
  - Confirm ESP32 sends heartbeat if no other commands are being sent.
- Real transport initialization fails:
  - Verify serial port and baud (`115200`), cabling, and Arduino power.

