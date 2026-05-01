/**
* SBCP (SnoBot Communication Protocol) v0.3.0
* Simplified state machine implementation.
*/

#include "state_machine.h"

StateMachine::StateMachine(ErrorManager* errorManager) :
    errors(errorManager),
    currentState(RobotState::BOOT),
    previousState(RobotState::BOOT),
    lastMode(RobotState::IDLE),
    stateEntryTime(0),
    estopActive(false)
{}

void StateMachine::begin() {
    currentState = RobotState::BOOT;
    previousState = RobotState::BOOT;
    lastMode = RobotState::IDLE;
    stateEntryTime = millis();
    estopActive = false;
}

void StateMachine::update() {
    if (estopActive) {
        if (currentState != RobotState::ESTOPPED) {
            transitionTo(RobotState::ESTOPPED);
        }
        return;
    }

    if (errors->hasCriticalError()) {
        if (currentState != RobotState::FAULT) {
            transitionTo(RobotState::FAULT);
        }
    }
}

uint32_t StateMachine::getTimeInState() const {
    return millis() - stateEntryTime;
}

bool StateMachine::isMotionEnabled() const {
    if (estopActive) return false;
    if (errors->hasCriticalError()) return false;

    return currentState == RobotState::MANUAL ||
        currentState == RobotState::AUTO ||
        currentState == RobotState::DEGRADED_COMM;
}

void StateMachine::setMode(RobotState mode) {
    if (estopActive) return;
    if (errors->hasCriticalError()) return;

    if (mode == RobotState::IDLE || mode == RobotState::MANUAL || mode == RobotState::AUTO || mode == RobotState::DEGRADED_COMM) {
        lastMode = mode;
        transitionTo(mode);
    }
}

void StateMachine::handleStop() {
    if (estopActive) return;
    transitionTo(RobotState::STOPPED);
}

void StateMachine::handleResume() {
    if (estopActive) return;
    if (errors->hasCriticalError()) return;
    transitionTo(lastMode);
}

void StateMachine::enterFault() {
    transitionTo(RobotState::FAULT);
}

void StateMachine::clearFault() {
    if (estopActive) return;
    if (errors->hasCriticalError()) return;
    transitionTo(RobotState::IDLE);
}

void StateMachine::setEstopActive(bool active) {
    estopActive = active;
    if (active) {
        transitionTo(RobotState::ESTOPPED);
    }
}

bool StateMachine::transitionTo(RobotState newState) {
    if (currentState == newState) return true;

    previousState = currentState;
    currentState = newState;
    stateEntryTime = millis();
    return true;
}
