/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified error system for hardware safety only.
*/

#ifndef SBCP_ERRORS_H
#define SBCP_ERRORS_H

#include <Arduino.h>
#include "types.h"

// Hardware safety error manager. All errors are latched.
class ErrorManager {
public:
    // Constructor.
    ErrorManager();

    // Add an error to the active set.
    void addError(ErrorCode code);

    // Remove an error from the active set.
    void removeError(ErrorCode code);

    // Check if a specific error is active.
    bool hasError (ErrorCode code) const;

    // Check if any error is active.
    bool hasAnyError() const;

    // Check if any critical error is active.
    bool hasCriticalError() const;

    // Clear all errors (intended for RESET_FAULT command).
    void clearAll();

    // Get count of active errors.
    uint8_t getActiveErrorCount() const;

    // Get list of active error codes (fills buffer, returns count).
    uint8_t getActiveErrors(uint8_t* buffer, uint8_t bufferSize) const;

    // Get highest severity of active errors.
    ErrorSeverity getHighestSeverity() const;

private:
    // Active error bit flags.
    uint32_t activeErrors;

    // Convert ErrorCode to bit position.
    uint8_t getBitPosition(ErrorCode code) const;
};

#endif // SBCP_ERRORS_H