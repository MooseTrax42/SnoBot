/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified error management implementation.
*/

#include "errors.h"

// Constructor.
ErrorManager::ErrorManager() : activeErrors(0) {
    // Start with no errors.
}

// Add an error.
void ErrorManager::addError(ErrorCode code) {
    uint8_t c = static_cast<uint8_t>(code);

    // Ignore OK and only track hardware errors.
    if (c == 0 || c > 32) return;

    // Set the bit.
    uint8_t bitPos = getBitPosition(code);
    activeErrors |= (1UL << bitPos);
}

// Remove an error.
void ErrorManager::removeError(ErrorCode code) {
    uint8_t c = static_cast<uint8_t>(code);

    // Only clear hardware errors.
    if (c == 0 || c > 32) return;

    // Clear the bit.
    uint8_t bitPos = getBitPosition(code);
    activeErrors &= ~(1UL << bitPos);
}

// Check if specific error is active.
bool ErrorManager::hasError(ErrorCode code) const {
    uint8_t c = static_cast<uint8_t>(code);
    
    // Check range.
    if (c == 0 || c > 32) return false;
    
    // Check the bit.
    uint8_t bitPos = getBitPosition(code);
    return (activeErrors & (1UL << bitPos)) != 0;
}

// Check if any error is active.
bool ErrorManager::hasAnyError() const {
    return activeErrors != 0;
}

// Check if any critical error is active.
bool ErrorManager::hasCriticalError() const {
    // Check each active error for critical severity.
    for (uint8_t i = 1; i <= 32; i++) {
        ErrorCode code = static_cast<ErrorCode>(i);
        if (hasError(code) && isErrorCritical(code)) {
            return true;
        }
    }
    return false;
}

// Clear all errors.
void ErrorManager::clearAll() {
    activeErrors = 0;
}

// Get count of active errors.
uint8_t ErrorManager::getActiveErrorCount() const {
    uint8_t count = 0;
    
    // Count set bits.
    for (uint8_t i = 1; i <= 32; i++) {
        if (hasError(static_cast<ErrorCode>(i))) {
            count++;
        }
    }
    
    return count;
}

// Get list of active errors.
uint8_t ErrorManager::getActiveErrors(uint8_t* buffer, uint8_t bufferSize) const {
    uint8_t count = 0;
    
    // Fill buffer with active error codes.
    for (uint8_t i = 1; i <= 32 && count < bufferSize; i++) {
        ErrorCode code = static_cast<ErrorCode>(i);
        if (hasError(code)) {
            buffer[count++] = i;
        }
    }
    
    return count;
}

// Get highest severity of active errors.
ErrorSeverity ErrorManager::getHighestSeverity() const {
    if (!hasAnyError()) {
        return ErrorSeverity::INFO;
    }
    
    ErrorSeverity highest = ErrorSeverity::INFO;
    
    // Check each active error.
    for (uint8_t i = 1; i <= 32; i++) {
        ErrorCode code = static_cast<ErrorCode>(i);
        if (hasError(code)) {
            ErrorSeverity severity = getErrorSeverity(code);
            if (static_cast<uint8_t>(severity) > static_cast<uint8_t>(highest)) {
                highest = severity;
            }
        }
    }
    
    return highest;
}

// Helper to convert error code to bit position.
uint8_t ErrorManager::getBitPosition(ErrorCode code) const {
    uint8_t c = static_cast<uint8_t>(code);

    // Codes map to one-less bit position.
    return c - 1;
}