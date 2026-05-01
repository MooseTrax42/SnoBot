/*
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified, direct actuator control.
*/

#ifndef SBCP_ACTUATOR_CONTROL_H
#define SBCP_ACTUATOR_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

// Simple actuator controller.
class ActuatorControl {
public:
    // Constructor.
    ActuatorControl();

    // Initialize actuators.
    void begin();

    // Update actuators.
    void update();

    // Auger control.
    void setAuger(bool enable);
    bool isAugerOn() const { return augerOn; }

    // Salt dispenser control.
    void setSalt(bool enable);
    bool isSaltOn() const { return saltOn; }

    // Chute servo control (angle in degrees, 0-180).
    void setChuteAngle(uint8_t angle);
    uint8_t getChuteAngle() const { return chuteAngle; }

    // Enable/disable all actuators.
    void enable();
    void disable();

private: 
    // States.
    bool enabled;
    bool augerOn;
    bool saltOn;
    Servo chuteServo;
    uint8_t chuteAngle;
    uint8_t lastChuteAngle;
};

#endif //SBCP_ACTUATOR_CONTROL_H