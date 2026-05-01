/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified drive control system.
*/

#ifndef SBCP_DRIVE_CONTROL_H
#define SBCP_DRIVE_CONTROL_H

#include <Arduino.h>
#include "config.h"
#include "types.h"

// Simple drive motor controller class.
class DriveControl {
public:
    // Constructor.
    DriveControl();

    // Intialize system and call in setup.
    void begin();

    // Update motors and call on iteration.
    void update();

    // Convert given velocity to motor commands.
    void setVelocity(float v, float w);

    // Set motor speeds directly.
    void setMotors(float left, float right);

    // Get current commanded motor speeds.
    void getMotors(float& left, float& right) const;

    // Enable/disable motor drivers.
    void enable();
    void disable();
    bool isEnabled() const {return enabled;}

private:
    // Motor state.
    float commandedLeft;
    float commandedRight;
    uint8_t pwmLeft;
    uint8_t pwmRight;
    bool dirLeft;
    bool dirRight;
    bool enabled;
    float ditherAccumLeft;
    float ditherAccumRight;

    // Convert speed to PWM.
    void speedToPWM(float speed, uint8_t& pwm, bool& dir, float& ditherAccum);

    // Write to motor pins.
    void writePWM();

    // Apply math to do motion things.
    void inverseKinematics(float v, float w, float& leftSpeed, float& rightSpeed);
};

#endif // SBCP_DRIVE_CONTROL_H
