# SnoBot ESP32 Controller Protocol

Specification for the handheld ESP32 controller communicating with the
Jetson over Bluetooth Classic (SPP / RFCOMM).

**Protocol version:** 1.0
**Transport:** Bluetooth RFCOMM, channel 1 (configurable)
**Framing:** One JSON object per line (`\n` terminated), UTF-8
**Library suggestion:** [ArduinoJson](https://arduinojson.org/) for parsing

---

## 1. Connection

The Jetson runs an RFCOMM server.  The ESP32 connects as a **Bluetooth SPP client**.

```cpp
#include "BluetoothSerial.h"
BluetoothSerial bt;

bt.begin("SnoBot_Controller", true);   // true = client/master mode
bt.connect("JETSON_BLUETOOTH_NAME");   // or bt.connect(mac_addr)
```

**One-time pairing** must be done first on the Jetson:
```bash
bluetoothctl
> scan on
> pair <ESP32_MAC>
> trust <ESP32_MAC>
```

After connection the Jetson immediately begins pushing telemetry.

---

## 2. Message Types

| Direction | Identified by | Description |
|-----------|--------------|-------------|
| ESP32 -> Jetson | `"cmd"` field | Command request |
| Jetson -> ESP32 | `"ack"` field | Command response |
| Jetson -> ESP32 | `"t":1` field | Telemetry push (~5 Hz) |

---

## 3. Commands (ESP32 -> Jetson)

Every command is a JSON object with a `"cmd"` string.
An optional `"seq"` integer can be included — the Jetson echoes it back in the response for request/response correlation.

### 3.1 Motion

| Command | JSON | Notes |
|---------|------|-------|
| Set velocity | `{"cmd":"vel","v":0.5,"w":0.1}` | `v` = linear m/s, `w` = angular rad/s. Clamped to robot limits (1.5 m/s, 3.14 rad/s). Send at **10-20 Hz** while sticks are active. |
| Graceful stop | `{"cmd":"stop"}` | Ramps velocity to zero smoothly. |
| Emergency stop | `{"cmd":"estop"}` | Immediate hard stop. Latches into ESTOPPED state. Requires `resume` to clear. |
| Resume | `{"cmd":"resume"}` | Resume from STOPPED or ESTOPPED state. |

### 3.2 Mode

| Command | JSON | Notes |
|---------|------|-------|
| Set mode | `{"cmd":"mode","mode":"MANUAL"}` | Accepts: `IDLE`, `MANUAL`, `AUTO` |

### 3.3 Actuators

| Command | JSON | Notes |
|---------|------|-------|
| Auger | `{"cmd":"auger","on":true}` | Snow auger motor on/off |
| Salt spreader | `{"cmd":"salt","on":true}` | Salt dispenser on/off |
| Chute angle | `{"cmd":"chute","angle":45.0}` | Degrees, clamped to [-90, 90] |

### 3.4 Lights

| Command | JSON | Notes |
|---------|------|-------|
| Headlight | `{"cmd":"light","on":true}` | On/off |
| Status light | `{"cmd":"slight","state":"BLINK"}` | `ON`, `OFF`, or `BLINK` |

### 3.5 Faults

| Command | JSON | Notes |
|---------|------|-------|
| Reset all faults | `{"cmd":"reset_faults"}` | Clears all active faults |
| Reset specific | `{"cmd":"reset_faults","codes":[70,71]}` | Clears only listed fault codes |

### 3.6 Perimeter Recording

| Command | JSON | Notes |
|---------|------|-------|
| Start recording | `{"cmd":"rec_start"}` | Resets odometry to (0,0,0), begins waypoint recording. Optional `"reset":false` keeps the current odometry frame (useful for hole recording). |
| Stop recording | `{"cmd":"rec_stop"}` | Finalises and saves the perimeter polygon |
| Reset recorder | `{"cmd":"rec_reset"}` | Discards recorded data, returns to idle |
| Get stats | `{"cmd":"rec_stats"}` | Returns waypoint count, distance, state in `"data"` |

### 3.7 System

| Command | JSON | Notes |
|---------|------|-------|
| Ping | `{"cmd":"ping"}` | Connectivity check, always succeeds |
| Get full status | `{"cmd":"status"}` | Returns comprehensive status in `"data"` |
| Heartbeat | `{"cmd":"hb"}` | **No response.** Silent keepalive (see section 6). |

---

## 4. Responses (Jetson -> ESP32)

### Success
```json
{"ack":"vel","ok":true}
{"ack":"vel","seq":42,"ok":true}
```

### Success with data
```json
{"ack":"rec_stats","ok":true,"data":{"state":"recording","waypoints":15,"cumulative_distance_m":4.5,"elapsed_s":32.1}}
```

### Error
```json
{"ack":"vel","ok":false,"err":"motion not enabled"}
{"ack":"auger","ok":false,"err":"'on' is required"}
```

The `"seq"` field is only present if the request included one.

---

## 5. Telemetry Push (Jetson -> ESP32)

Sent automatically at **~5 Hz** while connected.
Identified by `"t":1`.  Compact format, typically under 200 bytes.

### Example
```json
{"t":1,"ts":1234567890,"st":"MANUAL","v":[0.15,0.00],"vt":[0.40,0.00],"bat":46.50,"f":[],"e":0,"aug":0,"sal":0,"ch":90.0,"mot":1,"odo":[1.23,-0.45,1.57],"rec":"idle","rwp":0}
```

### Field Reference

| Key | Type | Description |
|-----|------|-------------|
| `t` | int | Always `1`. Identifies this as a telemetry packet. |
| `ts` | int | Jetson timestamp (ms since epoch, truncated to 32 bits) |
| `st` | string | Robot state (see section 7) |
| `v` | [float, float] | Current velocity `[linear_m/s, angular_rad/s]` |
| `vt` | [float, float] | Target velocity `[linear_m/s, angular_rad/s]` |
| `bat` | float or null | Battery voltage (V). `null` if unavailable. |
| `f` | int[] | Active fault codes. Empty array = no faults. |
| `e` | int | E-stop active: `0` or `1` |
| `aug` | int | Auger on: `0` or `1` |
| `sal` | int | Salt spreader on: `0` or `1` |
| `ch` | float | Chute angle (degrees) |
| `mot` | int | Motion enabled: `0` or `1` |
| `odo` | [float, float, float] or null | Odometry `[x_m, y_m, theta_rad]`. `null` if disabled. |
| `rec` | string | Perimeter recorder state: `"idle"`, `"recording"`, `"done"`, `"n/a"` |
| `rwp` | int | Recorder waypoint count |

### Parsing Example (ArduinoJson)

```cpp
#include <ArduinoJson.h>

void parseTelemetry(const String& line) {
    StaticJsonDocument<384> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) return;

    // Identify packet type
    if (!doc.containsKey("t")) return;  // Not telemetry

    // Robot state
    const char* state = doc["st"];            // "MANUAL", "AUTO", "FAULT", ...

    // Velocity
    float curV = doc["v"][0];                 // Current linear velocity (m/s)
    float curW = doc["v"][1];                 // Current angular velocity (rad/s)
    float tgtV = doc["vt"][0];               // Target linear velocity
    float tgtW = doc["vt"][1];               // Target angular velocity

    // Battery
    float battery = doc["bat"] | 0.0;        // Volts (0 if null)

    // Safety flags
    int estop    = doc["e"];                  // 0 or 1
    int motionOk = doc["mot"];               // 0 or 1

    // Actuators
    int auger = doc["aug"];                   // 0 or 1
    int salt  = doc["sal"];                   // 0 or 1
    float chute = doc["ch"];                  // Degrees

    // Fault codes
    JsonArray faults = doc["f"];
    bool hasFaults = faults.size() > 0;

    // Odometry (may be null)
    float odoX = 0, odoY = 0, odoTheta = 0;
    if (!doc["odo"].isNull()) {
        odoX     = doc["odo"][0];             // Meters
        odoY     = doc["odo"][1];
        odoTheta = doc["odo"][2];             // Radians
    }

    // Perimeter recording
    const char* recState = doc["rec"];        // "idle", "recording", "done"
    int recWaypoints     = doc["rwp"];

    updateDisplay(state, battery, estop, motionOk,
                  curV, auger, salt, chute, recState, recWaypoints);
}
```

---

## 6. Heartbeat & Watchdog

The Jetson runs a **3-second watchdog**.  If no data is received from the
ESP32 for 3 seconds, the connection is dropped and the robot **immediately
stops all motion and disables actuators**.

**Rule:** If the ESP32 has not sent any command in the last ~1 second,
send a heartbeat:

```cpp
unsigned long lastSendTime = 0;

void loop() {
    // ... handle joystick, buttons, etc ...

    // Heartbeat if idle
    if (millis() - lastSendTime > 1000) {
        bt.println("{\"cmd\":\"hb\"}");
        lastSendTime = millis();
    }
}
```

The heartbeat command (`hb`) produces **no response** from the Jetson.
All other commands produce a response.

---

## 7. Robot States

The `"st"` field in telemetry will be one of these strings:

| State | Motion Allowed | Description |
|-------|:--------------:|-------------|
| `BOOT` | No | Startup, initialising |
| `IDLE` | No | Ready but not in an operational mode |
| `MANUAL` | **Yes** | Manual control via this controller |
| `AUTO` | **Yes** | Autonomous operation |
| `STOPPED` | No | Temporarily halted (send `resume` to continue) |
| `DEGRADED_COMM` | **Yes** | Communication issues but still operational |
| `FAULT` | No | Hardware/software fault (see fault codes in `f`) |
| `ESTOPPED` | No | Emergency stop latched (send `resume` to clear) |

**Key indicator for the display:** if `mot` is `0`, the joystick will have
no effect.  Show a "DRIVE LOCKED" warning.

---

## 8. Recommended Display Layout

```
+----------------------------------+
|  STATE: MANUAL     BAT: 46.5V   |
|  [DRIVE READY]     [NO FAULTS]  |
+----------------------------------+
|  Speed: 0.40 m/s                 |
|  Auger: ON   Salt: OFF          |
|  Chute: 45 deg                   |
+----------------------------------+
|  Recording: 42 pts  12.6m       |
|  Pos: (3.2, -1.1)               |
+----------------------------------+
```

Priority indicators (show prominently when active):
- **E-STOP** (red) — when `e` is `1`
- **FAULT** (orange) — when `f` is non-empty
- **DRIVE LOCKED** — when `mot` is `0`
- **RECORDING** — when `rec` is `"recording"`

---

## 9. Typical Command Flow

### Startup
```
ESP32 -> Jetson:  (Bluetooth SPP connect)
Jetson -> ESP32:  {"t":1,"st":"MANUAL","v":[0,0],...}     (telemetry begins)
ESP32 -> Jetson:  {"cmd":"ping"}
Jetson -> ESP32:  {"ack":"ping","ok":true}
```

### Joystick Driving (10-20 Hz)
```
ESP32 -> Jetson:  {"cmd":"vel","v":0.5,"w":0.0}
Jetson -> ESP32:  {"ack":"vel","ok":true}
ESP32 -> Jetson:  {"cmd":"vel","v":0.5,"w":0.1}
Jetson -> ESP32:  {"ack":"vel","ok":true}
...
ESP32 -> Jetson:  {"cmd":"stop"}
Jetson -> ESP32:  {"ack":"stop","ok":true}
```

### Toggle Auger
```
ESP32 -> Jetson:  {"cmd":"auger","on":true}
Jetson -> ESP32:  {"ack":"auger","ok":true}
```

### Perimeter Recording
```
ESP32 -> Jetson:  {"cmd":"rec_start"}
Jetson -> ESP32:  {"ack":"rec_start","ok":true}
                  (drive around boundary...)
                  (telemetry shows "rec":"recording", "rwp" incrementing)
ESP32 -> Jetson:  {"cmd":"rec_stop"}
Jetson -> ESP32:  {"ack":"rec_stop","ok":true}
                  (or loop closure auto-stops: telemetry shows "rec":"done")
```

### Emergency Stop
```
ESP32 -> Jetson:  {"cmd":"estop"}
Jetson -> ESP32:  {"ack":"estop","ok":true}
                  (telemetry shows "st":"ESTOPPED", "e":1, "mot":0)
                  (robot is fully stopped, all actuators off)
ESP32 -> Jetson:  {"cmd":"resume"}
Jetson -> ESP32:  {"ack":"resume","ok":true}
```

### Idle Keepalive
```
ESP32 -> Jetson:  {"cmd":"hb"}
                  (no response)
                  (1 second later...)
ESP32 -> Jetson:  {"cmd":"hb"}
                  (no response)
```

---

## 10. Error Handling

Commands that fail return `"ok":false` with an `"err"` string:

```json
{"ack":"vel","ok":false,"err":"motion not enabled"}
{"ack":"auger","ok":false,"err":"'on' is required"}
{"ack":"rec_start","ok":false,"err":"cannot start in state done"}
{"ack":"xyz","ok":false,"err":"unknown command: xyz"}
```

The ESP32 should check `ok` before trusting the action succeeded.
For joystick velocity commands, a motion-not-enabled error means the
robot is in a non-driving state — show the user a warning rather than
retrying.

---

## 11. Safety Summary

| Layer | What Happens |
|-------|-------------|
| **ESP32 heartbeat** | Must send data every 3s or Jetson drops connection and stops robot |
| **Jetson intent expiry** | Motion intents auto-expire after 250ms if not refreshed |
| **Jetson disconnect handler** | On BT disconnect: `stop_motion()`, auger off, salt off |
| **Arduino watchdog** | MCU-level 2s timeout — if Jetson stops sending, Arduino stops motors |
| **Hardware E-stop** | Physical button on the robot, overrides everything |

The ESP32 does **not** need to implement safety logic beyond the heartbeat.
All safety enforcement is handled by the Jetson and Arduino.

---

## 12. Quick Reference Card

```
COMMAND              FIELDS                    RESPONSE
─────────────────────────────────────────────────────────
vel                  v (float), w (float)      ack
stop                 (none)                    ack
estop                (none)                    ack
resume               (none)                    ack
mode                 mode (string)             ack
auger                on (bool)                 ack
salt                 on (bool)                 ack
chute                angle (float)             ack
light                on (bool)                 ack
slight               state (string)            ack
reset_faults         codes (int[], optional)   ack
rec_start            (none)                    ack
rec_stop             (none)                    ack
rec_reset            (none)                    ack
rec_stats            (none)                    ack + data
status               (none)                    ack + data
ping                 (none)                    ack
hb                   (none)                    (none)
```
