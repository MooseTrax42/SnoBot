/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified main sketch implementing a hardware safety layer.
*
* Jetson is the source of truth; MCU enforces safety and applies intents.
*/

// Increase serial RX buffer to prevent overflow. 
#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 1024
#endif

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Servo.h>

#include "config.h"
#include "types.h"
#include "errors.h"
#include "state_machine.h"
#include "watchdog.h"
#include "drive_control.h"
#include "actuator_control.h"
#include "light_control.h"
#include "encoder_reader.h"

// Creating global objects.
ErrorManager errors;
StateMachine stateMachine(&errors);
Watchdog watchdog;
DriveControl drive;
ActuatorControl actuators;
LightControl lights;

// Track if handshake has been completed.
bool helloDone = false;

// Timing.
uint32_t lastSensorStreamTime = 0;
uint32_t lastLoopTime = 0;
uint32_t lastBlinkTime = 0;
uint32_t statSeq = 0;
uint8_t slowStatCounter = 0;
uint32_t lastStatAttemptMs = 0;
uint32_t lastStatSuccessMs = 0;
uint32_t statAttempts = 0;
uint32_t statFailures = 0;
uint32_t statConsecutiveFailures = 0;
uint32_t loopMaxTimeUs = 0;  // Track max loop execution time
uint32_t loopCount = 0;       // Count total loops
bool statForcePending = false;

// Serial buffers.
char serialInputBuffer[JSON_BUFFER_SIZE];
uint16_t serialInputIndex = 0;
uint32_t rxBytes = 0;
uint32_t rxMsgs = 0;
uint32_t lastRxMs = 0;
uint32_t rxBufferOverflows = 0;  // Track buffer overflows
uint32_t txBytes = 0;
uint32_t txMsgs = 0;
int txAvailLast = 0;

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

// ====================================
volatile uint32_t lastBreadcrumb = 0;
volatile uint32_t breadcrumbTimestamp = 0;
volatile uint32_t loopIterations = 0;
volatile uint32_t sendStatCalls = 0;
volatile uint32_t sendStatSuccess = 0;
volatile uint32_t sendStatBlocked = 0;
volatile uint32_t serialWriteEntered = 0;
volatile uint32_t serialWriteExited = 0;
volatile uint32_t statTxBytesPlanned = 0;
volatile uint32_t statTxBytesWritten = 0;
volatile uint32_t statTxWriteCalls = 0;
volatile uint32_t statWriteTimeouts = 0;

// Timing tracking
uint32_t maxLoopTime = 0;
uint32_t loopsOver100ms = 0;
uint32_t maxSerialWriteTime = 0;
uint32_t maxPrintlnTime = 0;
uint32_t lastStatLen = 0;

// STAT TX buffer to reduce write call count.
static char statTxBuffer[1024];

uint8_t statRotation = 0;

class StatCountingStream : public Print {
public:
    size_t write(uint8_t b) override {
        statTxBytesWritten++;
        statTxWriteCalls++;
        return Serial.write(b);
    }
    size_t write(const uint8_t* buffer, size_t size) override {
        statTxBytesWritten += size;
        statTxWriteCalls++;
        return Serial.write(buffer, size);
    }
};

static StatCountingStream statCountingStream;

// Breadcrumb codes
#define BREADCRUMB_LOOP_START           100
#define BREADCRUMB_LOOP_SERIAL_CHECK    110
#define BREADCRUMB_LOOP_SENDSTAT_START  190
#define BREADCRUMB_LOOP_SENDSTAT_END    191
#define BREADCRUMB_LOOP_END             210
#define BREADCRUMB_SENDSTAT_ENTER       300
#define BREADCRUMB_SENDSTAT_BUILD_JSON  310
#define BREADCRUMB_SENDSTAT_CHECK_SIZE  320
#define BREADCRUMB_SENDSTAT_SERIAL_WRITE_START 330
#define BREADCRUMB_SENDSTAT_SERIAL_WRITE_END   331
#define BREADCRUMB_SENDSTAT_PRINTLN_START 340
#define BREADCRUMB_SENDSTAT_PRINTLN_END   341
#define BREADCRUMB_SENDSTAT_EXIT        350
#define BREADCRUMB_SENDSTAT_OVERFLOW    360
#define BREADCRUMB_SENDSTAT_FALLBACK    361
#define BREADCRUMB_SENDSTAT_BLOCKED     370
#define BREADCRUMB_SENDSTAT_MINI        371

// Macro to set breadcrumb
#define SET_BREADCRUMB(x) do { \
    noInterrupts(); \
    lastBreadcrumb = (x); \
    breadcrumbTimestamp = millis(); \
    interrupts(); \
} while(0)
// ==========================================

// Sensor reading functions.
float readBatteryVoltage() {
    return 48.0f;
}

float readMotorCurrent(uint8_t motor) {
    return 0.0f;
}

float readTemperature() {
    return 25.0f;
}

void readIMU(float& roll, float& pitch, float& yaw,
    float& accelX, float& accelY, float& accelZ,
    float& gyroX, float& gyroY, float& gyroZ,
    bool& valid) {
    roll = pitch = yaw = 0.0f;
    accelX = accelY = accelZ = 0.0f;
    gyroX = gyroY = gyroZ = 0.0f;
    valid = false;
}

// Envelope helpers.
void sendAck(bool ok, uint32_t seq, const char* error, JsonObject data) {
    StaticJsonDocument<JSON_RESPONSE_SIZE> doc;
    doc["type"] = "ACK";
    doc["seq"] = seq;
    doc["ts"] = millis();

    JsonObject payload = doc.createNestedObject("data");
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

    if (doc.overflowed()) {
        Serial.println("{\"type\":\"ERROR\",\"data\":{\"msg\":\"JSON_OVERFLOW_ACK\"}}");
        return;
    }

    size_t txLen = measureJson(doc) + 1;
    txBytes += txLen;
    txMsgs += 1;

    serializeJson(doc, Serial);
    Serial.println();
}

void sendAck(bool ok, uint32_t seq, const char* error = nullptr) {
    StaticJsonDocument<JSON_RESPONSE_SIZE> empty;
    JsonObject nullObj = empty.to<JsonObject>();
    sendAck(ok, seq, error, nullObj);
}

// Sensor streaming.
bool sendStat(bool force) {
    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_ENTER);
    sendStatCalls++;

    if (force) {
        // Force a minimal STAT immediately for init handshake
        StaticJsonDocument<96> mini;
        mini["type"] = "STAT";
        mini["ts"] = millis();
        JsonObject mdata = mini.createNestedObject("data");
        mdata["seq"] = statSeq++;
        mdata["state"] = getStateName(stateMachine.getCurrentState());
        mdata["uptime_ms"] = millis();

        if (mini.overflowed()) {
            return false;
        }

        size_t jsonLen = measureJson(mini);
        size_t txLen = jsonLen + 1;
        txBytes += txLen;
        txMsgs += 1;

        serializeJson(mini, Serial);
        Serial.write('\n');
        return true;
    }

    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_BUILD_JSON);

    StaticJsonDocument<JSON_RESPONSE_SIZE> doc;
    doc["type"] = "STAT";
    doc["ver"] = SBCP_VERSION;
    doc["ts"] = millis();

    JsonObject data = doc.createNestedObject("data");
    data["seq"] = statSeq++;
    data["state"] = getStateName(stateMachine.getCurrentState());
    
    // ALWAYS include critical data
    data["estop"] = stateMachine.isEstopActive();
    data["encoder_left"] = encoders.getLeftCount();
    data["encoder_right"] = encoders.getRightCount();
    data["uptime_ms"] = millis();
    data["hello_done"] = helloDone;
    
    // Rotate through different data (10 cycles × 200ms = 2 sec for full update)
    statRotation = (statRotation + 1) % 10;
    
    switch(statRotation) {
        case 0:  // Actuators
            data["auger_en"] = actuators.isAugerOn();
            data["salt_en"] = actuators.isSaltOn();
            data["chute_angle"] = actuators.getChuteAngle();
            break;
            
        case 1:  // IMU
            {
                float imuRoll, imuPitch, imuYaw, imuAccelX, imuAccelY, imuAccelZ;
                float imuGyroX, imuGyroY, imuGyroZ;
                bool imuValid;
                readIMU(imuRoll, imuPitch, imuYaw, imuAccelX, imuAccelY, imuAccelZ, 
                        imuGyroX, imuGyroY, imuGyroZ, imuValid);
                data["imu_yaw"] = imuYaw;
                data["imu_valid"] = imuValid;
            }
            break;
            
        case 2:  // Power & thermal
            data["battery_v"] = readBatteryVoltage();
            data["temp"] = readTemperature();
            data["estop_soft"] = estopSoftActive;
            data["estop_state"] = estopState;
            data["estop_pin"] = estopPinReading;
            data["estop_use_pin"] = ESTOP_USE_PIN;
            break;
            
        case 3:  // RX stats
            data["rx_msgs"] = rxMsgs;
            data["rx_bytes"] = rxBytes;
            data["rx_last_ms"] = lastRxMs;
            data["rx_overflows"] = rxBufferOverflows;
            break;
            
        case 4:  // TX stats
            data["tx_msgs"] = txMsgs;
            data["tx_bytes"] = txBytes;
            data["tx_avail"] = Serial.availableForWrite();
            data["stream_period_ms"] = SENSOR_STREAM_PERIOD_MS;
            break;
            
        case 5:  // Loop performance
            data["loop_max_us"] = loopMaxTimeUs;
            data["loop_count"] = loopCount;
            data["loops_over_100ms"] = loopsOver100ms;
            data["max_loop_time"] = maxLoopTime;
            loopMaxTimeUs = 0;
            break;
            
        case 6:  // SendStat performance
            data["sendstat_calls"] = sendStatCalls;
            data["sendstat_success"] = sendStatSuccess;
            data["sendstat_blocked"] = sendStatBlocked;
            data["sendstat_timeouts"] = statWriteTimeouts;
            data["stat_attempts"] = statAttempts;
            data["stat_failures"] = statFailures;
            data["stat_consecutive_failures"] = statConsecutiveFailures;
            break;
            
        case 7:  // Serial write performance
            data["serial_write_entered"] = serialWriteEntered;
            data["serial_write_exited"] = serialWriteExited;
            data["max_serial_write_time"] = maxSerialWriteTime;
            data["max_println_time"] = maxPrintlnTime;
            data["stat_len_last"] = lastStatLen;
            break;
            
        case 8:  // More debug counters
            data["loop_iterations"] = loopIterations;
            data["stat_last_attempt_ms"] = lastStatAttemptMs;
            data["stat_last_success_ms"] = lastStatSuccessMs;
            data["stat_tx_planned"] = statTxBytesPlanned;
            data["stat_tx_written"] = statTxBytesWritten;
            data["stat_tx_write_calls"] = statTxWriteCalls;
            break;
            
        case 9:  // Breadcrumb & misc
            data["breadcrumb"] = lastBreadcrumb;
            data["breadcrumb_age"] = millis() - breadcrumbTimestamp;
            break;
    }
    
    // Always include errors if present
    uint8_t errorCodes[10];
    uint8_t errorCount = errors.getActiveErrors(errorCodes, 10);
    if (errorCount > 0) {
        JsonArray faults = data.createNestedArray("faults");
        for (uint8_t i = 0; i < errorCount; i++) {
            faults.add(errorCodes[i]);
        }
    }

    // Check for overflow before sending
    if (doc.overflowed()) {
        SET_BREADCRUMB(BREADCRUMB_SENDSTAT_OVERFLOW);
        // Send error as separate message
        Serial.println("{\"type\":\"ERROR\",\"data\":{\"msg\":\"JSON_OVERFLOW_STAT\"}}");
        
        // Send minimal fallback STAT
        SET_BREADCRUMB(BREADCRUMB_SENDSTAT_FALLBACK);
        StaticJsonDocument<128> mini;
        mini["type"] = "STAT";
        JsonObject mdata = mini.createNestedObject("data");
        mdata["seq"] = statSeq++;
        mdata["state"] = getStateName(stateMachine.getCurrentState());
        mdata["uptime_ms"] = millis();
        mdata["hello_done"] = helloDone;
        mdata["breadcrumb"] = lastBreadcrumb;
        mdata["breadcrumb_age"] = millis() - breadcrumbTimestamp;

        if (!mini.overflowed()) {
            size_t jsonLen = measureJson(mini);
            size_t miniLen = jsonLen + 1;
            txBytes += miniLen;
            txMsgs += 1;
            serializeJson(mini, Serial);
            Serial.write('\n');
            sendStatSuccess++;
            return true;
        }
        return false;
    }

    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_CHECK_SIZE);

    size_t jsonLen = measureJson(doc);
    size_t fullLen = jsonLen + 1;
    lastStatLen = fullLen;
    txBytes += fullLen;
    txMsgs += 1;
    statTxBytesPlanned = fullLen;
    statTxBytesWritten = 0;
    statTxWriteCalls = 0;

    // If the platform reports TX buffer space, skip or downgrade STAT when space is low.
    int txAvail = Serial.availableForWrite();
    if (txAvail > 0 && (int)(fullLen + 1) > txAvail) {
        sendStatBlocked++;
        SET_BREADCRUMB(BREADCRUMB_SENDSTAT_BLOCKED);

        // Send minimal fallback STAT to avoid blocking.
        StaticJsonDocument<128> mini;
        mini["type"] = "STAT";
        JsonObject mdata = mini.createNestedObject("data");
        mdata["seq"] = statSeq++;
        mdata["state"] = getStateName(stateMachine.getCurrentState());
        mdata["uptime_ms"] = millis();
        mdata["hello_done"] = helloDone;
        mdata["breadcrumb"] = lastBreadcrumb;
        mdata["breadcrumb_age"] = millis() - breadcrumbTimestamp;

        if (!mini.overflowed()) {
            size_t jsonLen = measureJson(mini);
            size_t miniLen = jsonLen + 1;
            txBytes += miniLen;
            txMsgs += 1;
            SET_BREADCRUMB(BREADCRUMB_SENDSTAT_MINI);
            serializeJson(mini, Serial);
            Serial.write('\n');
            sendStatSuccess++;
            return true;
        }
        return false;
    }

    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_SERIAL_WRITE_START);
    uint32_t writeStart = micros();
    serialWriteEntered++;

    // Refresh breadcrumb values just before serialization.
    data["breadcrumb"] = lastBreadcrumb;
    data["breadcrumb_age"] = millis() - breadcrumbTimestamp;

    size_t actualLen = serializeJson(doc, statTxBuffer, sizeof(statTxBuffer));
    if (actualLen == 0 || actualLen >= sizeof(statTxBuffer)) {
        serializeJson(doc, statCountingStream);
    } else {
        statTxBytesWritten = actualLen;
        statTxWriteCalls = 1;
        Serial.write((const uint8_t*)statTxBuffer, actualLen);
    }
    serialWriteExited++;
    uint32_t writeTime = micros() - writeStart;
    if (writeTime > maxSerialWriteTime) {
        maxSerialWriteTime = writeTime;
    }
    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_SERIAL_WRITE_END);
        
    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_PRINTLN_START);
    uint32_t printlnStart = micros();
    Serial.write('\n');
    uint32_t printlnTime = micros() - printlnStart;
    if (printlnTime > maxPrintlnTime) {
        maxPrintlnTime = printlnTime;
    }
    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_PRINTLN_END);
        
    SET_BREADCRUMB(BREADCRUMB_SENDSTAT_EXIT);
    sendStatSuccess++;
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
    sendAck(true, seq);
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
    StaticJsonDocument<JSON_RESPONSE_SIZE> doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
        // Debug ACK for parse failures.
        sendAck(false, 0, "PARSE_ERROR");
        return;
    }

    if (!doc.containsKey("type") || !doc.containsKey("data")) {
        sendAck(false, 0, "BAD_ENVELOPE");
        return;
    }

    const char* msgType = doc["type"];
    JsonObject data = doc["data"].as<JsonObject>();
    uint32_t seq = doc["seq"] | 0;

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

    // Report buffer size for verification
    #ifdef SERIAL_RX_BUFFER_SIZE
    Serial.print("{\"type\":\"BOOT\",\"data\":{\"msg\":\"rx_buffer_size\",\"size\":");
    Serial.print(SERIAL_RX_BUFFER_SIZE);
    Serial.println("}}");
    #else
    Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"rx_buffer_size\",\"size\":64}}");
    #endif

    Serial.flush();

    #if defined(ESP32)
    // Increase task stack size
    xTaskCreatePinnedToCore(
        loop,
        "MainTask",
        8192,  // ← Increase from default 40960
        NULL,
        1,
        NULL,
        1
    );
    #endif

    uint32_t serialStartTime = millis();
    while (!Serial && (millis() - serialStartTime < 3000)) {
    }

    Serial.println("{\"type\":\"BOOT\",\"data\":{\"msg\":\"serial_ready\"}}");

    Wire.begin();

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
    SET_BREADCRUMB(BREADCRUMB_LOOP_START);
    loopIterations++;

    uint32_t loopStartUs = micros();  // Start loop timing
    uint32_t loopStart = millis();
    uint32_t now = millis();

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

    SET_BREADCRUMB(BREADCRUMB_LOOP_SERIAL_CHECK);

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

    SET_BREADCRUMB(BREADCRUMB_LOOP_SENDSTAT_START); 

    if (helloDone) {
        bool shouldSend = statForcePending || (now - lastStatAttemptMs >= SENSOR_STREAM_PERIOD_MS);
        if (shouldSend) {
            lastStatAttemptMs = now;
            statAttempts++;
            if (sendStat(statForcePending)) {
                lastSensorStreamTime = now;
                lastStatSuccessMs = now;
                statConsecutiveFailures = 0;
                statForcePending = false;
            } else {
                statFailures++;
                statConsecutiveFailures++;
            }
        }
    }
    SET_BREADCRUMB(BREADCRUMB_LOOP_SENDSTAT_END);

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

    uint32_t loopTime = millis() - loopStart;
    if (loopTime > maxLoopTime) {
        maxLoopTime = loopTime;
    }
    if (loopTime > 100) {
        loopsOver100ms++;
    }
    SET_BREADCRUMB(BREADCRUMB_LOOP_END);

    // Only delay if no serial data pending (improves RX processing)
    if (Serial.available() == 0) {
        delay(1);  // Reduced from 2ms for faster serial processing
    }
}
