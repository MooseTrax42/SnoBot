/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified Light Control Implementation
*/

#include "light_control.h"

// Constructor.
LightControl::LightControl() :
    enabled(false),
    headlightOn(false),
    statusLEDOn(false),
    errorLEDOn(false)
{}

// Initialize.
void LightControl::begin() {
    // Set light pins as outputs.
    pinMode(PIN_LED_HEAD, OUTPUT);
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);

    // Start with all lights off.
    enabled = false;
    headlightOn = false;
    statusLEDOn = false;
    errorLEDOn = false;

    // Write initial states.
    digitalWrite(PIN_LED_HEAD, LOW);
    digitalWrite(PIN_LED_STATUS, LOW);
    digitalWrite(PIN_LED_ERROR, LOW);
}

// Update lights.
void LightControl::update() {
    if (!enabled) {
        digitalWrite(PIN_LED_HEAD, LOW);
        digitalWrite(PIN_LED_STATUS, LOW);
        digitalWrite(PIN_LED_ERROR, LOW);
        return;
    }

    // Write current states to hardware.
    digitalWrite(PIN_LED_HEAD, headlightOn ? HIGH : LOW);
    digitalWrite(PIN_LED_STATUS, statusLEDOn ? HIGH : LOW);
    digitalWrite(PIN_LED_ERROR, errorLEDOn ? HIGH : LOW);
}

// Set headlight.
void LightControl::setHeadlight(bool on) {
    headlightOn = on;
}

// Set status LED.
void LightControl::setStatusLED(bool on) {
    statusLEDOn = on;
}

// Set error LED.
void LightControl::setErrorLED(bool on) {
    errorLEDOn = on;
}

// Turn all lights off.
void LightControl::allOff() {
    setHeadlight(false);
    setStatusLED(false);
    setErrorLED(false);
}

// Enable light control.
void LightControl::enable() {
    enabled = true;
}

// Disable light control.
void LightControl::disable() {
    enabled = false;
}