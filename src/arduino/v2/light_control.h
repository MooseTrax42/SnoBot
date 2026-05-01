/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified light controls.
*/

#ifndef SBCP_LIGHT_CONTROL_H
#define SBCP_LIGHT_CONTROL_H

#include <Arduino.h>
#include "config.h"

// Simple light controller.
class LightControl {
public:
    // Constructor.
    LightControl();

    // Initialize lights.
    void begin();

    // Update lights.
    void update();

    // Headlight control.
    void setHeadlight(bool on);
    bool isHeadlightOn() const {return headlightOn;}

    // Status LED control.
    void setStatusLED(bool on);
    bool isStatusLEDOn() const {return statusLEDOn;}

    // Error LED control.
    void setErrorLED(bool on);
    bool isErrorLEDOn() const {return errorLEDOn;}

    // Turn all lights off.
    void allOff();

    // Enable/disable lights.
    void enable();
    void disable();

private:
    bool enabled;
    bool headlightOn;
    bool statusLEDOn;
    bool errorLEDOn;
};

#endif // SBCP_LIGHT_CONTROL_H