/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified state machine focused on hardware safety and Jetson alignment.
*/

#ifndef SBCP_STATE_MACHINE_H
#define SBCP_STATE_MACHINE_H

#include <Arduino.h>
#include "types.h"
#include "errors.h"

class StateMachine {
public:
    StateMachine(ErrorManager* errorManager);

    void begin();
    void update();

    RobotState getCurrentState() const { return currentState; }
    RobotState getPreviousState() const { return previousState; }
    uint32_t getTimeInState() const;

    bool isMotionEnabled() const;

    // Mode/state requests from Jetson.
    void setMode(RobotState mode);
    void handleStop();
    void handleResume();

    // Fault/E-stop handling.
    void enterFault();
    void clearFault();
    bool isEstopActive() const { return estopActive; }
    void setEstopActive(bool active);

private:
    ErrorManager* errors;

    RobotState currentState;
    RobotState previousState;
    RobotState lastMode;
    uint32_t stateEntryTime;
    bool estopActive;

    bool transitionTo(RobotState newState);
};

#endif // SBCP_STATE_MACHINE_H
