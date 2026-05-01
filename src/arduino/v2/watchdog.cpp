/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified watchdog implementation.
*/

#include "watchdog.h"

// Constructor.
Watchdog::Watchdog() :
    enabled(false),
    lastFeedTime(0),
    timeoutCount(0),
    timeoutActive(false),
    timeoutCallback(nullptr)
{}

// Initialize.
void Watchdog::begin() {
    lastFeedTime = millis();
    timeoutCount = 0;
    timeoutActive = false;
    enabled = false; // Don't enable until after HELLO handshake.
}

// Update and check for timeout.
void Watchdog::update() {
    if (!enabled) return;

    checkTimeout();
}

// Feed the watchdog.
void Watchdog::feed() {
    lastFeedTime = millis();

    // Clear timeout flag if it was active.
    if (timeoutActive) {
        timeoutActive = false;
    }
}

// Check if watchdog is expired.
bool Watchdog::isExpired() const {
    if (!enabled) return false;

    uint32_t elapsed = millis() - lastFeedTime;
    return elapsed >= WATCHDOG_TIMEOUT_MS;
}

// Get time since last feed.
uint32_t Watchdog::getTimeSinceFeed() const {
    return millis() - lastFeedTime;
}

// Enable the watchdog.
void Watchdog::enable() {
    if (!enabled) {
        enabled = true;
        lastFeedTime = millis(); // Reset timer when enabling.
        timeoutActive = false;
    }
}

// Disable the watchdog.
void Watchdog::disable() {
    enabled = false;
    timeoutActive = false;
}

// Set timeout callback.
void Watchdog::setTimeoutCallback(void (*callback)()) {
    timeoutCallback = callback;
}

// Check for timeout.
void Watchdog::checkTimeout() {
    // If already timed out, don't re-trigger.
    if (timeoutActive) return;

    // Check if timeout occurred.
    if (isExpired()) {
        timeoutActive = true;
        timeoutCount++;

        // Call callback if set.
        if (timeoutCallback) {
            timeoutCallback();
        }
    }
}