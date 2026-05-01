/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified main sketch implementing a hardware safety layer.
*
* Jetson is the source of truth; MCU enforces safety and applies intents.
*/

// Increase serial buffers to prevent overflow (must be before Arduino.h).
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 512
#endif

#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 512
#endif

#include <Arduino.h>
#include <ArduinoJson.h>

#include "config.h"
#include "types.h"
#include "errors.h"
#include "state_machine.h"
#include "watchdog.h"
#include "drive_control.h"
#include "actuator_control.h"
#include "light_control.h"
#include "imu_reader.h"
#include "encoder_reader.h"

// Memory monitoring (approximate for Arduino R4)
int freeMemory() {
    // Arduino R4 has 32KB RAM (0x20000000 - 0x20008000)
    // This is an approximation - we track stack pointer position
    // and estimate heap usage based on known memory layout
    char stack_var;
    unsigned long stack_ptr = (unsigned long)&stack_var;

    // RAM starts at 0x20000000, size is 32768 bytes on Arduino R4
    // Estimate: stack grows down from top, static data from bottom
    // We'll estimate roughly 8KB used by static data + heap
    const unsigned long ESTIMATED_STATIC_USAGE = 8192;
    const unsigned long RAM_SIZE = 32768;
    const unsigned long RAM_START = 0x20000000;

    // Free memory is approximately: stack position - estimated base
    unsigned long free_approx = stack_ptr - (RAM_START + ESTIMATED_STATIC_USAGE);

    return (int)free_approx;
}

// Creating global objects.
ErrorManager errors;
StateMachine stateMachine(&errors);
Watchdog watchdog;
DriveControl drive;
ActuatorControl actuators;
LightControl lights;

// Static JSON documents to avoid stack allocation.
static StaticJsonDocument<JSON_RESPONSE_SIZE> statDoc;
static StaticJsonDocument<JSON_RESPONSE_SIZE> ackDoc;

// Track if handshake has been completed.
bool helloDone = false;

// Timing.
uint32_t lastSensorStreamTime = 0;
uint32_t lastLoopTime = 0;
uint32_t lastBlinkTime = 0;
uint32_t statSeq = 0;
uint8_t slowStatCounter = 0;
uint32_t loopMaxTimeUs = 0;  // Track max loop execution time
uint32_t loopCount = 0;       // Count total loops
bool statForcePending = false;

// Memory monitoring
uint32_t lastMemCheck = 0;
int minFreeRam = 32000;
int lastReportedFreeRam = 0;

// Serial buffers.
char serialInputBuffer[JSON_BUFFER_SIZE];
uint16_t serialInputIndex = 0;
uint32_t rxBytes = 0;
uint32_t rxMsgs = 0;
uint32_t lastRxMs = 0;
uint32_t rxBufferOverflows = 0;  // Track buffer overflows

// Safety things.
volatile bool estopTriggered = false;
volatile bool estopState = false;
volatile bool estopSoftActive = false;
bool estopLastReading = false;
uint32_t estopLastChangeMs = 0;

// Intent cache (for telemetry).
float intentV = 0.0f;
float intentW = 0.0f;
bool statusBlink = false;
bool heartbeatOn = false;
uint32_t lastHeartbeatMs = 0;
bool estopPinReading = false;

// Sensor reading functions.
float readBatteryVoltage() {
    return 48.0f;
}

float readMotorCurrent(uint8_t motor) {
    return 0.0f;
}

float readTemperature() {
    float temp = imuReader.readTemp();
    if (!imuReader.isAvailable()) {
        return 25.0f;
    }
    return temp;
}

void readIMU(float& roll, float& pitch, float& yaw,
    float& accelX, float& accelY, float& accelZ,
    float& gyroX, float& gyroY, float& gyroZ,
    bool& valid) {
    valid = imuReader.read(roll, pitch, yaw,
        accelX, accelY, accelZ,
        gyroX, gyroY, gyroZ);
}

// Envelope helpers.
void sendAck(bool ok, uint32_t seq, const char* error, JsonObject data) {
    ackDoc.clear();  // Reuse static document
    ackDoc["type"] = "ACK";
    ackDoc["seq"] = seq;
    ackDoc["ts"] = millis();

    JsonObject payload = ackDoc.createNestedObject("data");
    payload["ok"] = ok;
    if (!ok && error) {
        payload["error"] = error;
    }
    if (!data.isNull() && data.size() > 0) {
        JsonObject out = payload.createNestedObject("data");
        for (JsonPair kv : data) {
            out[kv.key().c_str()] = kv.value();
        }
    }

    // Check for overflow before sending.
    if (ackDoc.overflowed()) {
        Serial.println("{\"type\":\"ERROR\",\"data\":{\"msg\":\"JSON_OVERFLOW_ACK\"}}");
        return;
    }

    serializeJson(ackDoc, Serial);
    Serial.println();
}

void sendAck(bool ok, uint32_t seq, const char* error = nullptr) {
    StaticJsonDocument<JSON_RESPONSE_SIZE> empty;
    JsonObject nullObj = empty.to<JsonObject>();
    sendAck(ok, seq, error, nullObj);
}

// Sensor streaming.
// Compact STAT schema to keep messages <= 256 bytes:
// {"type":"S","ts":<ms>,"data":{"s":<seq>,"st":<state>,"e":<estop>,
//  "enc":[L,R],"imu":[r,p,y,ax,ay,az,gx,gy,gz],"v":<imu_valid>,
//  "b":<mV>,"tp":<cC>,"f":[...]} }
// IMU scaling: angles deg*100, accel m/s^2*1000, gyro deg/s*100.
static inline int32_t scaleFloat(float value, float scale) {
    return (int32_t)(value * scale);
}

bool sendStat(bool force) {
    (void)force;  // Force is kept for compatibility; compact STAT is always used.

    statDoc.clear();  // Reuse static document
    statDoc["type"] = "S";
    statDoc["ts"] = millis();

    JsonObject data = statDoc.createNestedObject("data");
    data["s"] = statSeq++;
    data["st"] = (uint8_t)stateMachine.getCurrentState();
    data["e"] = stateMachine.isEstopActive() ? 1 : 0;

    // Encoder counts (atomic read).
    int32_t encL = 0;
    int32_t encR = 0;
    noInterrupts();
    encL = encoders.getLeftCount();
    encR = encoders.getRightCount();
    interrupts();
    JsonArray enc = data.createNestedArray("enc");
    enc.add(encL);
    enc.add(encR);

    // IMU (scaled ints).
    float roll, pitch, yaw;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    bool imuValid = false;
    readIMU(roll, pitch, yaw, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imuValid);
    JsonArray imu = data.createNestedArray("imu");
    imu.add(scaleFloat(roll, 100.0f));
    imu.add(scaleFloat(pitch, 100.0f));
    imu.add(scaleFloat(yaw, 100.0f));
    imu.add(scaleFloat(accelX, 1000.0f));
    imu.add(scaleFloat(accelY, 1000.0f));
    imu.add(scaleFloat(accelZ, 1000.0f));
    imu.add(scaleFloat(gyroX, 100.0f));
    imu.add(scaleFloat(gyroY, 100.0f));
    imu.add(scaleFloat(gyroZ, 100.0f));
    data["v"] = imuValid ? 1 : 0;

    // Active errors (if any).
    uint8_t errorCodes[10];
    uint8_t errorCount = errors.getActiveErrors(errorCodes, 10);
    if (errorCount > 0) {
        JsonArray faults = data.createNestedArray("f");
        for (uint8_t i = 0; i < errorCount; i++) {
            faults.add(errorCodes[i]);
        }
    }

    // Slow-rate fields to reduce bandwidth and message size.
    slowStatCounter++;
    if (slowStatCounter >= 10) {
        slowStatCounter = 0;
        data["b"] = (int32_t)(readBatteryVoltage() * 100.0f);  // centivolts (V * 100)
        data["tp"] = (int32_t)(readTemperature() * 10.0f);     // deci-degC (C * 10)

        uint8_t imuStatus = 0;
        uint8_t imuSelfTest = 0;
        uint8_t imuError = 0;
        uint8_t imuCalSys = 0;
        uint8_t imuCalGyro = 0;
        uint8_t imuCalAccel = 0;
        uint8_t imuCalMag = 0;

        if (imuReader.readSystemStatus(imuStatus, imuSelfTest, imuError)) {
            JsonArray imuStat = data.createNestedArray("is");
            imuStat.add(imuStatus);
            imuStat.add(imuSelfTest);
            imuStat.add(imuError);
        }
        if (imuReader.readCalibration(imuCalSys, imuCalGyro, imuCalAccel, imuCalMag)) {
            JsonArray imuCal = data.createNestedArray("ic");
            imuCal.add(imuCalSys);
            imuCal.add(imuCalGyro);
            imuCal.add(imuCalAccel);
            imuCal.add(imuCalMag);
        }
    }

    // Check for overflow before sending.
    if (statDoc.overflowed()) {
        Serial.println("{\"type\":\"ERROR\",\"data\":{\"msg\":\"JSON_OVERFLOW_STAT\"}}");
        return false;
    }

    serializeJson(statDoc, Serial);
    Serial.println();
    return true;
}

bool parseMode(const char* modeStr, RobotState& out) {
    if (!modeStr) return false;
    if (strcmp(modeStr, "IDLE") == 0) { out = RobotState::IDLE; return true; }
    if (strcmp(modeStr, "MANUAL") == 0) { out = RobotState::MANUAL; return true; }
    if (strcmp(modeStr, "AUTO") == 0) { out = RobotState::AUTO; return true; }
    if (strcmp(modeStr, "DEGRADED_COMM") == 0) { out = RobotState::DEGRADED_COMM; return true; }
    if (strcmp(modeStr, "STOPPED") == 0) { out = RobotState::STOPPED; return true; }
    return false;
}

// Command handlers.
void handleHello(JsonObject payload, uint32_t seq) {
    if (!payload.containsKey("args") || !payload["args"].containsKey("version")) {
        sendAck(false, seq, "MISSING_ARG:version");
        return;
    }

    const char* version = payload["args"]["version"];
    bool compatible = (strncmp(version, "0.3.", 4) == 0);

    // Keep HELLO ACK minimal to reduce TX size and avoid partial-line stalls.
    sendAck(compatible, seq, compatible ? nullptr : "INCOMPATIBLE_VERSION");

    if (compatible) {
        helloDone = true;
        watchdog.enable();
        statForcePending = true;
    }
}

void handleStatus(uint32_t seq) {
    // Send STAT instead of ACK (poll-based telemetry)
    // This prevents autonomous streaming that can overwhelm serial
    sendStat(false);
}

void handlePing(uint32_t seq) {
    StaticJsonDocument<JSON_RESPONSE_SIZE> tmp;
    JsonObject data = tmp.createNestedObject("data");
    data["uptime_ms"] = millis();
    sendAck(true, seq, nullptr, data);
}

void handleMode(JsonObject payload, uint32_t seq) {
    if (!payload.containsKey("args") || !payload["args"].containsKey("mode")) {
        sendAck(false, seq, "MISSING_ARG:mode");
        return;
    }

    const char* modeStr = payload["args"]["mode"];
    RobotState mode;
    if (!parseMode(modeStr, mode)) {
        sendAck(false, seq, "BAD_MODE");
        return;
    }

    if (mode == RobotState::STOPPED) {
        stateMachine.handleStop();
    } else {
        stateMachine.setMode(mode);
    }
    statForcePending = true;
    sendAck(true, seq);
}

void handleStop(uint32_t seq) {
    stateMachine.handleStop();
    statForcePending = true;
    sendAck(true, seq);
}

void handleResume(uint32_t seq) {
    stateMachine.handleResume();
    statForcePending = true;
    sendAck(true, seq);
}

void handleResetFaults(uint32_t seq) {
    errors.clearAll();

    // Clear estop if switch is released or soft estop was requested.
    if (!ESTOP_USE_PIN || digitalRead(PIN_ESTOP) == HIGH || estopSoftActive) {
        errors.removeError(ErrorCode::ESTOP);
        stateMachine.setEstopActive(false);
        estopSoftActive = false;
    }

    if (!stateMachine.isEstopActive()) {
        stateMachine.clearFault();
    }
    statForcePending = true;
    sendAck(true, seq);
}

void handleShutdown(uint32_t seq) {
    // Disable outputs and telemetry.
    helloDone = false;
    watchdog.disable();
    drive.disable();
    actuators.disable();
    lights.allOff();
    lights.update();
    sendAck(true, seq);
}


void processCommand(JsonObject payload, uint32_t seq) {
    if (!payload.containsKey("cmd")) {
        sendAck(false, seq, "MISSING_CMD");
        return;
    }

    const char* cmd = payload["cmd"];

    if (strcmp(cmd, "HELLO") == 0) {
        handleHello(payload, seq);
    } else if (strcmp(cmd, "STATUS") == 0) {
        handleStatus(seq);
    } else if (strcmp(cmd, "PING") == 0) {
        handlePing(seq);
    } else if (strcmp(cmd, "MODE") == 0) {
        handleMode(payload, seq);
    } else if (strcmp(cmd, "STOP") == 0) {
        handleStop(seq);
    } else if (strcmp(cmd, "RESUME") == 0) {
        handleResume(seq);
    } else if (strcmp(cmd, "RESET_FAULTS") == 0) {
        handleResetFaults(seq);
    } else if (strcmp(cmd, "SHUTDOWN") == 0) {
        handleShutdown(seq);
    } else {
        sendAck(false, seq, "UNKNOWN_COMMAND");
    }
}

void handleIntent(JsonObject data) {
    if (data.containsKey("v")) {
        intentV = data["v"];
    }
    if (data.containsKey("w")) {
        intentW = data["w"];
    }

    if (data.containsKey("auger_en")) {
        actuators.setAuger(data["auger_en"]);
    }
    if (data.containsKey("salt_en")) {
        actuators.setSalt(data["salt_en"]);
    }
    if (data.containsKey("chute_angle")) {
        uint8_t angle = data["chute_angle"];
        actuators.setChuteAngle(angle);
    }
    if (data.containsKey("headlight")) {
        lights.setHeadlight(data["headlight"]);
    }
    if (data.containsKey("status_light")) {
        const char* val = data["status_light"] | "";
        if (strcmp(val, "BLINK") == 0) {
            statusBlink = true;
        } else if (strcmp(val, "ON") == 0) {
            statusBlink = false;
            lights.setStatusLED(true);
        } else if (strcmp(val, "OFF") == 0) {
            statusBlink = false;
            lights.setStatusLED(false);
        }
    }

    if (data.containsKey("mode")) {
        const char* modeStr = data["mode"];
        RobotState mode;
        if (parseMode(modeStr, mode)) {
            if (mode == RobotState::STOPPED) stateMachine.handleStop();
            else stateMachine.setMode(mode);
        }
    }
    if (data.containsKey("stop") && data["stop"].as<bool>()) {
        stateMachine.handleStop();
    }
    if (data.containsKey("resume") && data["resume"].as<bool>()) {
        stateMachine.handleResume();
    }
    if (data.containsKey("reset_faults")) {
        errors.clearAll();
        if (!ESTOP_USE_PIN || !stateMachine.isEstopActive()) {
            stateMachine.clearFault();
        }
    }
    if (data.containsKey("estop") && data["estop"].as<bool>()) {
        errors.addError(ErrorCode::ESTOP);
        stateMachine.setEstopActive(true);
        estopSoftActive = true;
    }
}

void processMessage(const char* json) {
    lastRxMs = millis();
    rxMsgs++;
    watchdog.feed();  // Feed watchdog ONLY when message received

    StaticJsonDocument<JSON_RESPONSE_SIZE> cmdDoc;
    DeserializationError error = deserializeJson(cmdDoc, json);

    if (error) {
        // Debug ACK for parse failures.
        sendAck(false, 0, "PARSE_ERROR");
        return;
    }

    if (!cmdDoc.containsKey("type") || !cmdDoc.containsKey("data")) {
        sendAck(false, 0, "BAD_ENVELOPE");
        return;
    }

    const char* msgType = cmdDoc["type"];
    JsonObject data = cmdDoc["data"].as<JsonObject>();
    uint32_t seq = cmdDoc["seq"] | 0;

    if (strcmp(msgType, "CMD") == 0) {
        processCommand(data, seq);
    } else if (strcmp(msgType, "INTENT") == 0) {
        handleIntent(data);
    }
}

void onWatchdogTimeout() {
    drive.disable();
    actuators.disable();
    errors.addError(ErrorCode::WATCHDOG_TIMEOUT);
    stateMachine.enterFault();
}

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Set RX buffer size for platforms that support it (Teensy, some STM32)
    #if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(CORE_TEENSY)
        Serial.setRxBufferSize(1024);
    #endif

    delay(100);
    Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"setup_start\"}}");

    // Report memory at boot
    int bootFreeRam = freeMemory();
    minFreeRam = bootFreeRam;
    Serial.print("{\"type\":\"BOOT\",\"data\":{\"msg\":\"free_ram_boot\",\"bytes\":");
    Serial.print(bootFreeRam);
    Serial.println("}}");

    // Report buffer size for verification
    #ifdef SERIAL_RX_BUFFER_SIZE
    Serial.print("{\"type\":\"BOOT\",\"data\":{\"msg\":\"rx_buffer_size\",\"size\":");
    Serial.print(SERIAL_RX_BUFFER_SIZE);
    Serial.println("}}");
    #else
        Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"rx_buffer_size\",\"size\":64}}");
    #endif

    #ifdef SERIAL_TX_BUFFER_SIZE
    Serial.print("{\"type\":\"BOOT\",\"data\":{\"msg\":\"tx_buffer_size\",\"size\":");
    Serial.print(SERIAL_TX_BUFFER_SIZE);
    Serial.println("}}");
    #else
        Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"tx_buffer_size\",\"size\":64}}");
    #endif

    Serial.flush();

    uint32_t serialStartTime = millis();
    while (!Serial && (millis() - serialStartTime < 3000)) {
    }

    Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"serial_ready\"}}");

    imuReader.begin();

    Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"wire_ready\"}}");

    pinMode(PIN_ESTOP, INPUT_PULLUP);
    estopLastReading = (digitalRead(PIN_ESTOP) == LOW);
    estopLastChangeMs = millis();

    stateMachine.begin();
    watchdog.begin();
    watchdog.setTimeoutCallback(onWatchdogTimeout);
    drive.begin();
    actuators.begin();
    lights.begin();
    encoders.begin();
    lights.enable();
    actuators.enable();
    lights.setStatusLED(true);

    lastSensorStreamTime = millis();
    lastLoopTime = millis();
    lastBlinkTime = millis();

    serialInputIndex = 0;

    Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"setup_complete\"}}");
}

void loop() {
    uint32_t loopStartUs = micros();  // Start loop timing
    uint32_t now = millis();

    // Memory monitoring and emergency reset
    if (now - lastMemCheck > 5000) {  // Every 5 seconds
        int freeRam = freeMemory();
        if (freeRam < minFreeRam) {
            minFreeRam = freeRam;
        }

        // Alert if memory is critically low
        if (freeRam < 2000 && freeRam != lastReportedFreeRam) {
            // Set error code instead of printing (reported in next STAT)
            errors.addError(ErrorCode::BATTERY_CRITICAL);  // Reuse an error code or add LOW_MEMORY
            lastReportedFreeRam = freeRam;
        }

        // Emergency reset if memory exhausted (no print - just reset)
        if (freeRam < 500) {
            NVIC_SystemReset();
        }

        lastMemCheck = now;
    }

    // Auto-reset if no serial activity for 30 seconds (hard crash recovery)
    if (helloDone && (now - lastRxMs) > 30000) {
        // Just reset - don't try to print (serial is likely dead anyway)
        NVIC_SystemReset();
    }

    if (ESTOP_USE_PIN) {
        estopPinReading = (digitalRead(PIN_ESTOP) == LOW);
        if (estopPinReading != estopLastReading) {
            estopLastReading = estopPinReading;
            estopLastChangeMs = now;
        }
        if (estopPinReading && !estopState && (now - estopLastChangeMs) >= 50) {
            estopTriggered = true;
            estopState = true;
            estopSoftActive = false;
            digitalWrite(PIN_MOTOR_DRIVE_ENABLE, LOW);
        }
        if (!estopPinReading && estopState && (now - estopLastChangeMs) >= 50) {
            estopState = false;
        }
    } else {
        estopPinReading = false;
    }

    if (estopTriggered) {
        estopTriggered = false;
        errors.addError(ErrorCode::ESTOP);
        stateMachine.setEstopActive(true);
        drive.disable();
        actuators.disable();
        lights.setErrorLED(true);
        lights.setStatusLED(false);
        lights.update();
    }

    const uint16_t maxRxBytesPerLoop = 256;
    uint16_t rxBytesProcessed = 0;
    while (Serial.available() && rxBytesProcessed < maxRxBytesPerLoop) {
        char c = Serial.read();
        rxBytesProcessed++;
        rxBytes++;

        if (c == '\n' || c == '\r') {
            if (serialInputIndex > 0) {
                serialInputBuffer[serialInputIndex] = '\0';
                processMessage(serialInputBuffer);
                serialInputIndex = 0;
            }
        } else if (serialInputIndex < JSON_BUFFER_SIZE - 1) {
            serialInputBuffer[serialInputIndex++] = c;
        } else {
            // Buffer overflow - reset and wait for next message delimiter
            // This prevents accumulating garbage when messages are too large
            rxBufferOverflows++;
            serialInputIndex = 0;
            // Note: We're now discarding this oversized message
            // Consider increasing JSON_BUFFER_SIZE if this happens frequently
        }
    }

    // DISABLED: Autonomous telemetry streaming (now poll-based via STATUS command)
    // The Jetson will request telemetry at a controlled rate, preventing serial backlog
    /*
    if (helloDone) {
        bool shouldSend = statForcePending || (now - lastSensorStreamTime >= SENSOR_STREAM_PERIOD_MS);
        if (shouldSend) {
            if (sendStat(statForcePending)) {
                lastSensorStreamTime = now;
                statForcePending = false;
            }
        }
    }
    */

    stateMachine.update();

    float batteryVoltage = readBatteryVoltage();
    if (batteryVoltage < BATTERY_CRITICAL_V && batteryVoltage > 10.0f) {
        errors.addError(ErrorCode::BATTERY_CRITICAL);
    }

    float temp = readTemperature();
    if (temp > TEMP_CRITICAL_C) {
        errors.addError(ErrorCode::MOTOR_TEMP);
    }

    float motorLeftCurrent = readMotorCurrent(0);
    float motorRightCurrent = readMotorCurrent(1);
    if (motorLeftCurrent > DRIVE_MOTOR_CURRENT_MAX_A || motorRightCurrent > DRIVE_MOTOR_CURRENT_MAX_A) {
        errors.addError(ErrorCode::MOTOR_OVERLOAD);
    }

    bool stopped = stateMachine.getCurrentState() == RobotState::STOPPED ||
        stateMachine.getCurrentState() == RobotState::FAULT ||
        stateMachine.getCurrentState() == RobotState::ESTOPPED;

    if (stateMachine.isMotionEnabled()) {
        if (!drive.isEnabled()) {
            drive.enable();
        }
        drive.setVelocity(intentV, intentW);
        drive.update();
    } else {
        drive.disable();
        intentV = 0.0f;
        intentW = 0.0f;
    }

    if (stopped) {
        actuators.disable();
        actuators.setAuger(false);
        actuators.setSalt(false);
    } else {
        actuators.enable();
    }
    actuators.update();

    if (statusBlink) {
        if (now - lastBlinkTime >= 500) {
            lastBlinkTime = now;
            lights.setStatusLED(!lights.isStatusLEDOn());
        }
    }
    if (HEARTBEAT_LED_ENABLE && !statusBlink && stateMachine.getCurrentState() != RobotState::FAULT
        && stateMachine.getCurrentState() != RobotState::ESTOPPED) {
        if (now - lastHeartbeatMs >= HEARTBEAT_PERIOD_MS) {
            lastHeartbeatMs = now;
            heartbeatOn = !heartbeatOn;
            lights.setStatusLED(heartbeatOn);
        }
    }

    switch (stateMachine.getCurrentState()) {
        case RobotState::FAULT:
        case RobotState::ESTOPPED:
            lights.setErrorLED(true);
            lights.setStatusLED(false);
            break;
        default:
            lights.setErrorLED(false);
            if (!statusBlink && !HEARTBEAT_LED_ENABLE) {
                lights.setStatusLED(true);
            }
            break;
    }
    lights.update();

    if (helloDone && watchdog.isEnabled()) {
        watchdog.update();
    }

    lastLoopTime = now;

    // Track loop execution time
    uint32_t loopTimeUs = micros() - loopStartUs;
    if (loopTimeUs > loopMaxTimeUs) {
        loopMaxTimeUs = loopTimeUs;
    }
    loopCount++;

    // Only delay if no serial data pending (improves RX processing)
    if (Serial.available() == 0) {
        delay(1);  // Reduced from 2ms for faster serial processing
    }
}
