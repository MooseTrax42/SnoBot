/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified drive control implementation.
*/

#include "drive_control.h"

// Constructor.
DriveControl::DriveControl() :
    commandedLeft(0.0f),
    commandedRight(0.0f),
    pwmLeft(0),
    pwmRight(0),
    dirLeft(true),
    dirRight(true),
    enabled(false),
    ditherAccumLeft(0.0f),
    ditherAccumRight(0.0f)
{}

// Initialize.
void DriveControl::begin() {
    // Set motor pins as outputs.
    pinMode(PIN_MOTOR_LEFT_PWM_L, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_PWM_R, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_PWM_L, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_PWM_R, OUTPUT);
    pinMode(PIN_MOTOR_DRIVE_ENABLE, OUTPUT);

    // Start with motors disabled.
    commandedLeft = 0.0f;
    commandedRight = 0.0f;
    pwmLeft = 0;
    pwmRight = 0;
    dirLeft = true;
    dirRight = true;
    enabled = false;
    ditherAccumLeft = 0.0f;
    ditherAccumRight = 0.0f;

    // Write zero RPM.
    writePWM();

    // Disable motor drivers.
    digitalWrite(PIN_MOTOR_DRIVE_ENABLE, LOW);
}

// Update motors.
void DriveControl::update() {
    // If disabled, ensure motors are stopped.
    if (!enabled) {
        pwmLeft = 0;
        pwmRight = 0;
        writePWM();
        return;
    }

    // Convert commanded speeds to PWM and direction.
    speedToPWM(commandedLeft, pwmLeft, dirLeft, ditherAccumLeft);
    speedToPWM(commandedRight, pwmRight, dirRight, ditherAccumRight);

    // Write to motors.
    writePWM();
}

// Set velocity from (v, w).
void DriveControl::setVelocity(float v, float w) {
    // Convert to wheel speds.
    float leftSpeed, rightSpeed;
    inverseKinematics(v, w, leftSpeed, rightSpeed);

    // Set motor commands.
    setMotors(leftSpeed, rightSpeed);
}

// Set motor speeds directly.
void DriveControl::setMotors(float left, float right) {
    // Clamp to valid range.
    commandedLeft = constrain(left, -1.0f, 1.0f);
    commandedRight = constrain(right, -1.0f, 1.0f);
}

// Get current commanded speeds.
void DriveControl::getMotors(float &left, float &right) const {
    left = commandedLeft;
    right = commandedRight;
}

// Enable motors.
void DriveControl::enable() {
    enabled = true;
    digitalWrite(PIN_MOTOR_DRIVE_ENABLE, HIGH);
}

// Disable motors. Same function as emergency stop.
void DriveControl::disable() {
    enabled = false;
    commandedLeft = 0.0f;
    commandedRight = 0.0f;
    pwmLeft = 0;
    pwmRight =0;
    dirLeft = true;
    dirRight = true;
    ditherAccumLeft = 0.0f;
    ditherAccumRight = 0.0f;
    writePWM();
    digitalWrite(PIN_MOTOR_DRIVE_ENABLE, LOW);
}

// Convert speed to PWM and direction.
void DriveControl::speedToPWM(float speed, uint8_t& pwm, bool& direction, float& ditherAccum) {
    // Clamp to valid range.
    speed = constrain(speed, -1.0f, 1.0f);

    // Determine direction.
    direction = (speed >= 0.0f);

    // Convert to desired PWM magnitude.
    float magnitude = abs(speed);
    float desiredPwm = magnitude * MOTOR_PWM_MAX;

    if (desiredPwm <= 0.0f) {
        pwm = 0;
        ditherAccum = 0.0f;
        return;
    }

    // Dither in the deadband region so low-speed commands still produce motion over time.
    if (MOTOR_PWM_DEADBAND > 0 && desiredPwm < MOTOR_PWM_DEADBAND) {
        ditherAccum += desiredPwm / (float)MOTOR_PWM_DEADBAND;
        if (ditherAccum >= 1.0f) {
            ditherAccum -= 1.0f;
            pwm = (uint8_t)MOTOR_PWM_DEADBAND;
        } else {
            pwm = 0;
        }
        return;
    }

    // Sigma-delta PWM quantization for sub-step average duty control.
    int whole = (int)desiredPwm;
    float frac = desiredPwm - (float)whole;
    ditherAccum += frac;
    if (ditherAccum >= 1.0f) {
        whole += 1;
        ditherAccum -= 1.0f;
    }
    whole = constrain(whole, 0, MOTOR_PWM_MAX);
    pwm = (uint8_t)whole;
}

// Inverse kinematics (v, w) -> (left, right) wheel speeds.
void DriveControl::inverseKinematics(float v, float w, float& leftSpeed, float& rightSpeed) {
    // Calculate wheel linear velocities.
    float leftVel = v - (w * WHEELBASE / 2.0f);
    float rightVel = v + (w * WHEELBASE / 2.0f);

    // Convert to normalized motor command.
    leftSpeed = leftVel / DRIVE_MAX_LIN_VEL;
    rightSpeed = rightVel / DRIVE_MAX_LIN_VEL;

    // Clamp to valid command range.
    float maxMag = max(abs(leftSpeed), abs(rightSpeed));
    if (maxMag > 1.0f) {
    leftSpeed  /= maxMag;
    rightSpeed /= maxMag;
}
}

// Write PWM to motor pins.
void DriveControl::writePWM() {
    // Left motor.
    if (dirLeft) {
        // Forward.
        analogWrite(PIN_MOTOR_LEFT_PWM_L, pwmLeft);
        analogWrite(PIN_MOTOR_LEFT_PWM_R, 0);
    } else {
        // Reverse direction.
        analogWrite(PIN_MOTOR_LEFT_PWM_L, 0);
        analogWrite(PIN_MOTOR_LEFT_PWM_R, pwmLeft);
    }

    // Right motor.
    if (dirRight) {
        // Forward.
        analogWrite(PIN_MOTOR_RIGHT_PWM_L, pwmRight);
        analogWrite(PIN_MOTOR_RIGHT_PWM_R, 0);
    } else {
        // Reverse.
        analogWrite(PIN_MOTOR_RIGHT_PWM_L, 0);
        analogWrite(PIN_MOTOR_RIGHT_PWM_R, pwmRight);
    }
}
