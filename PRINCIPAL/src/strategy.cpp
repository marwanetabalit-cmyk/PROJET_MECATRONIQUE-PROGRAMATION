#include "strategy.h"
#include "config.h"

constexpr unsigned long MATCH_DURATION_MS = 100000;

void StrategyManager::init() {
    state = RobotState::WAIT_START;
    stateStartMs = millis();
    matchStartMs = 0;
}

void StrategyManager::changeState(RobotState newState) {
    if (state != newState) {
        state = newState;
        stateStartMs = millis();
    }
}

RobotState StrategyManager::getState() const {
    return state;
}

void StrategyManager::update(const SafetySystem& safety,
                             const DistanceReadings& distances,
                             DriveBase& drive,
                             ServoController& servos,
                             ActionManager& actions) {
    coreUpdate(
        safety.isStartPressed(),
        safety.isEStopPressed(),
        distances,
        drive,
        servos,
        actions
    );
}

void StrategyManager::updateSimulation(const SimInputs& sim,
                                       DriveBase& drive,
                                       ServoController& servos,
                                       ActionManager& actions) {
    coreUpdate(
        sim.startPressed,
        sim.eStopPressed,
        sim.distances,
        drive,
        servos,
        actions
    );
}

void StrategyManager::coreUpdate(bool startPressed,
                                 bool eStopPressed,
                                 const DistanceReadings& distances,
                                 DriveBase& drive,
                                 ServoController& servos,
                                 ActionManager& actions) {
    if (eStopPressed) {
        drive.stop();
        changeState(RobotState::EMERGENCY_STOP);
        return;
    }

    if (matchStartMs != 0 && (millis() - matchStartMs >= MATCH_DURATION_MS)) {
        drive.stop();
        changeState(RobotState::END_MATCH);
        return;
    }

    if (state != RobotState::AVOID_OBSTACLE &&
        state != RobotState::EMERGENCY_STOP &&
        state != RobotState::END_MATCH &&
        distances.obstacle) {
        drive.stop();
        changeState(RobotState::AVOID_OBSTACLE);
        return;
    }

    switch (state) {
        case RobotState::WAIT_START:
            drive.stop();
            if (startPressed) {
                matchStartMs = millis();
                changeState(RobotState::GO_TO_BOX_ZONE);
            }
            break;

        case RobotState::GO_TO_BOX_ZONE: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 2000) {
                drive.forward(DRIVE_FORWARD_RPM);
            } else {
                drive.stop();
                changeState(RobotState::PICK_BOX);
            }
            break;
        }

        case RobotState::PICK_BOX:
            drive.stop();
            actions.pickBox(servos);
            changeState(RobotState::GO_TO_DROP_ZONE);
            break;

        case RobotState::GO_TO_DROP_ZONE: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 2000) {
                drive.forward(DRIVE_FORWARD_RPM);
            } else {
                drive.stop();
                changeState(RobotState::DROP_BOX);
            }
            break;
        }

        case RobotState::DROP_BOX:
            drive.stop();
            actions.dropBox(servos);
            changeState(RobotState::GO_TO_THERMOMETER);
            break;

        case RobotState::GO_TO_THERMOMETER: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 1500) {
                drive.forward(DRIVE_FORWARD_RPM);
            } else {
                drive.stop();
                changeState(RobotState::PUSH_CURSOR);
            }
            break;
        }

        case RobotState::PUSH_CURSOR:
            drive.stop();
            actions.pushCursor(servos);
            changeState(RobotState::RETURN_TO_NEST);
            break;

        case RobotState::RETURN_TO_NEST: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 2000) {
                drive.backward(DRIVE_FORWARD_RPM);
            } else {
                drive.stop();
                actions.returnHome();
                changeState(RobotState::END_MATCH);
            }
            break;
        }

        case RobotState::AVOID_OBSTACLE: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < AVOID_STOP_MS) {
                drive.stop();
            } else if (elapsed < (AVOID_STOP_MS + AVOID_TURN_MS)) {
                drive.rotateRight(DRIVE_TURN_RPM);
            } else {
                changeState(RobotState::GO_TO_BOX_ZONE);
            }
            break;
        }

        case RobotState::EMERGENCY_STOP:
            drive.stop();
            break;

        case RobotState::END_MATCH:
            drive.stop();
            break;
    }
}