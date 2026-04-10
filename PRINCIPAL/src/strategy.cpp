#include "strategy.h"
#include "config.h"

constexpr unsigned long MATCH_DURATION_MS = 100000;

void StrategyManager::init() {
    state = RobotState::WAIT_START;
    stateStartMs = millis();
    matchStartMs = 0;
}

void StrategyManager::changeState(RobotState newState) {
    state = newState;
    stateStartMs = millis();
}

RobotState StrategyManager::getState() const {
    return state;
}

void StrategyManager::update(const SafetySystem& safety,
                             const DistanceReadings& distances,
                             DriveBase& drive,
                             ServoController& servos) {
    coreUpdate(
        safety.isStartPressed(),
        safety.isEStopPressed(),
        distances,
        drive,
        servos
    );
}

void StrategyManager::updateSimulation(const SimInputs& sim,
                                       DriveBase& drive,
                                       ServoController& servos) {
    coreUpdate(
        sim.startPressed,
        sim.eStopPressed,
        sim.distances,
        drive,
        servos
    );
}

void StrategyManager::coreUpdate(bool startPressed,
                                 bool eStopPressed,
                                 const DistanceReadings& distances,
                                 DriveBase& drive,
                                 ServoController& servos) {
    (void)servos;

    if (eStopPressed) {
        drive.stop();
        changeState(RobotState::EMERGENCY_STOP);
        return;
    }

    if (matchStartMs != 0 && millis() - matchStartMs >= MATCH_DURATION_MS) {
        drive.stop();
        changeState(RobotState::END_MATCH);
        return;
    }

    switch (state) {
        case RobotState::WAIT_START:
            drive.stop();
            if (startPressed) {
                matchStartMs = millis();
                changeState(RobotState::RUN_FORWARD);
            }
            break;

        case RobotState::RUN_FORWARD:
            if (distances.obstacle) {
                drive.stop();
                changeState(RobotState::AVOID_OBSTACLE);
            } else {
                drive.forward(DRIVE_FORWARD_RPM);
            }
            break;

        case RobotState::AVOID_OBSTACLE: {
            unsigned long elapsed = millis() - stateStartMs;

            if (elapsed < AVOID_STOP_MS) {
                drive.stop();
            } else if (elapsed < (AVOID_STOP_MS + AVOID_TURN_MS)) {
                drive.rotateRight(DRIVE_TURN_RPM);
            } else {
                changeState(RobotState::RUN_FORWARD);
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