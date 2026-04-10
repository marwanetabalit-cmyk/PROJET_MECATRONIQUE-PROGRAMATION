#include "strategy.h"
#include "config.h"

void StrategyManager::init() {
    state = RobotState::WAIT_START;
    stateStartMs = millis();
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
    (void)servos; // pas encore utilisé dans cette première stratégie

    if (safety.isEStopPressed()) {
        drive.stop();
        changeState(RobotState::EMERGENCY_STOP);
        return;
    }

    switch (state) {
        case RobotState::WAIT_START:
            drive.stop();
            if (safety.isStartPressed()) {
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
                // exemple simple : tourne à droite
                drive.rotateRight(DRIVE_TURN_RPM);
            } else {
                changeState(RobotState::RUN_FORWARD);
            }
            break;
        }

        case RobotState::EMERGENCY_STOP:
            drive.stop();
            break;
    }
}