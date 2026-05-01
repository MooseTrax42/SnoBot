/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified watchdog system without all the fancy stuff.
*/

#ifndef SBCP_WATCHDOG_H
#define SBCP_WATCHDOG_H

#include <Arduino.h>
#include "config.h"

// Simple safety watchdog.
class Watchdog {
public:
    // Constructor.
    Watchdog();

    // Initialize in setup.
    void begin();

    // Update in main loop.
    void update();

    // Feed the watchdog when any command is recieved.
    void feed();

    // Check if it's expired.
    bool isExpired() const;

    // Get time since last feed.
    uint32_t getTimeSinceFeed() const;

    // Enable/disable watchdog.
    void enable();
    void disable();
    bool isEnabled() const {return enabled;}

    // Set timeout callback (called when watchdog expires).
    void setTimeoutCallback(void (*callback)());

    // Get timeout stats.
    uint32_t getTimeoutCount() const {return timeoutCount;}

private:
    bool enabled;
    uint32_t lastFeedTime;
    uint32_t timeoutCount;
    bool timeoutActive;
    void (*timeoutCallback)();

    // Check for timeout and trigger callback.
    void checkTimeout();
};

#endif // SBCP_WATCHDOG_H