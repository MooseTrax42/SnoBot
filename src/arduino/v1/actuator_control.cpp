/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Actuator control implementation.
*/

#include "actuator_control.h"

// Constructor.
ActuatorControl::ActuatorControl() :
    enabled(false),
    augerOn(false),
    saltOn(false),
    chuteAngle(CHUTE_CENTER_ANGLE),
    lastChuteAngle(CHUTE_CENTER_ANGLE)
{}

// Initialize.
void ActuatorControl::begin() {
    // Set actuator pins as outputs.
    pinMode(PIN_AUGER_ENABLE, OUTPUT);
    pinMode(PIN_SALT_ENABLE, OUTPUT);

    // Initialize chute servo.
    chuteServo.attach(PIN_CHUTE_PWM);

    // Start with all actuators disabled.
    enabled = false;
    augerOn = false;
    saltOn = false;
    chuteAngle = CHUTE_CENTER_ANGLE;
    lastChuteAngle = chuteAngle;

    // Write initial states.
    digitalWrite(PIN_AUGER_ENABLE, LOW);
    digitalWrite(PIN_SALT_ENABLE, LOW);
    chuteServo.write(chuteAngle);
}

// Update actuators.
void ActuatorControl::update() {
    // Ensure pins are low if module is disabled.
    if (!enabled) {
        digitalWrite(PIN_AUGER_ENABLE, LOW);
        digitalWrite(PIN_SALT_ENABLE, LOW);
        return;
    }

    // Write current states to hardware.
    digitalWrite(PIN_AUGER_ENABLE, augerOn ? HIGH : LOW);
    digitalWrite(PIN_SALT_ENABLE, saltOn ? HIGH : LOW);
    if (chuteAngle != lastChuteAngle) {
        chuteServo.write(chuteAngle);
        lastChuteAngle = chuteAngle;
    }
}

// Set auger state.
void ActuatorControl::setAuger(bool enable) {
    augerOn = enable;
}

// Set salt dispenser state.
void ActuatorControl::setSalt(bool enable) {
    saltOn = enable;
}

// Set chute angle.
void ActuatorControl::setChuteAngle(uint8_t angle) {
    // Clamp to valid range.
    chuteAngle = constrain(angle, CHUTE_MIN_ANGLE, CHUTE_MAX_ANGLE);
}

// Enable all actuators.
void ActuatorControl::enable() {
    enabled = true;
}

// Disable all actuators.
void ActuatorControl::disable() {
    enabled = false;
    setAuger(false);
    setSalt(false);
    // Keep chute at current position.
}