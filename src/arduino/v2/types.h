/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified type definitions.
*/

#ifndef SBCP_TYPES_H
#define SBCP_TYPES_H

#include <Arduino.h>

// Robot states aligned with Jetson.
enum class RobotState : uint8_t {
    BOOT = 0,
    IDLE = 1,
    MANUAL = 2,
    AUTO = 3,
    STOPPED = 4,
    DEGRADED_COMM = 5,
    FAULT = 6,
    ESTOPPED = 7
};

// Hardware safety errors (use SBCP fault codes where possible).
enum class ErrorCode : uint8_t {
    OK = 0,
    ESTOP = 1,
    INTERLOCK = 2,
    BATTERY_CRITICAL = 5,
    WATCHDOG_TIMEOUT = 9,
    MOTOR_TEMP = 20,
    MOTOR_OVERLOAD = 21
};

// Assigning severity levels.
enum class ErrorSeverity : uint8_t {
    INFO = 0,
    WARNING = 1,
    ERROR = 2,
    CRITICAL = 3
};

// Sensor data structure.
struct SensorData {
    float battVoltage;
    float motorLeftCurrent;
    float motorRightCurrent;
    float imuRoll;
    float imuPitch;
    float imuYaw;
    float imuAccelX;
    float imuAccelY;
    float imuAccelZ;
    float imuGyroX;
    float imuGyroY;
    float imuGyroZ;
    bool imuValid;
    int32_t encoderLeft;
    int32_t encoderRight;
    float temperature;
    uint32_t timestamp;

    SensorData() :
        battVoltage(0.0f),
        motorLeftCurrent(0.0f),
        motorRightCurrent(0.0f),
        imuRoll(0.0f),
        imuPitch(0.0f),
        imuYaw(0.0f),
        imuAccelX(0.0f),
        imuAccelY(0.0f),
        imuAccelZ(0.0f),
        imuGyroX(0.0f),
        imuGyroY(0.0f),
        imuGyroZ(0.0f),
        imuValid(false),
        encoderLeft(0),
        encoderRight(0),
        temperature(0.0f),
        timestamp(0)
    {}
};

// Motor state structure.
struct MotorState {
    float commandedLeft;
    float commandedRight;
    int16_t pwmLeft;
    int16_t pwmRight;
    bool enabled;

    MotorState() :
        commandedLeft(0.0f),
        commandedRight(0.0f),
        pwmLeft(0),
        pwmRight(0),
        enabled(false)
    {}
};

// Actuator state structure.
struct ActuatorState {
    bool augerOn;
    bool saltOn;
    uint8_t chuteAngle;

    ActuatorState() :
        augerOn(false),
        saltOn(false),
        chuteAngle(90)
    {}
};

// Get state name as a string.
inline const char* getStateName(RobotState state) {
    switch (state) {
        case RobotState::BOOT: return "BOOT";
        case RobotState::IDLE: return "IDLE";
        case RobotState::MANUAL: return "MANUAL";
        case RobotState::AUTO: return "AUTO";
        case RobotState::STOPPED: return "STOPPED";
        case RobotState::DEGRADED_COMM: return "DEGRADED_COMM";
        case RobotState::FAULT: return "FAULT";
        case RobotState::ESTOPPED: return "ESTOPPED";
        default: return "UNKNOWN";
    }
}

// Get error name as a string.
inline const char* getErrorName(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK: return "OK";
        case ErrorCode::ESTOP: return "ESTOP";
        case ErrorCode::INTERLOCK: return "INTERLOCK";
        case ErrorCode::BATTERY_CRITICAL: return "BATT_CRITICAL";
        case ErrorCode::WATCHDOG_TIMEOUT: return "WATCHDOG_TIMEOUT";
        case ErrorCode::MOTOR_TEMP: return "MOTOR_TEMP";
        case ErrorCode::MOTOR_OVERLOAD: return "MOTOR_OVERLOAD";
        default: return "UNKNOWN";
    }
}

// Check if error is critical.
inline bool isErrorCritical(ErrorCode code) {
    return code == ErrorCode::ESTOP ||
        code == ErrorCode::INTERLOCK ||
        code == ErrorCode::BATTERY_CRITICAL ||
        code == ErrorCode::MOTOR_TEMP ||
        code == ErrorCode::MOTOR_OVERLOAD ||
        code == ErrorCode::WATCHDOG_TIMEOUT;
}

// Get error severity by code.
inline ErrorSeverity getErrorSeverity(ErrorCode code) {
    if (code == ErrorCode::OK) return ErrorSeverity::INFO;

    if (code == ErrorCode::WATCHDOG_TIMEOUT) return ErrorSeverity::ERROR;

    if (isErrorCritical(code)) return ErrorSeverity::CRITICAL;

    return ErrorSeverity::WARNING;
}

// Clamp float value to range.
inline float clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Clamp integer value to range.
inline int clampi(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Map value from one range to another.
inline float mapf(float value, float inMin, float inMax, float outMin, float outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

#endif // SBCP_TYPES_H
