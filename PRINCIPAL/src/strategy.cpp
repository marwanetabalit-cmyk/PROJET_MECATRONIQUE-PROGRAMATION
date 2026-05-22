#include "strategy.h"
#include "config.h"

// ============================================================================
// CYCLE DE VIE ET GESTION D'ETATS
// ============================================================================

void StrategyManager::init() {
    state = RobotState::WAIT_START;
    stateBeforeAvoidance = RobotState::WAIT_START;
    odometryReferenceState = RobotState::WAIT_START;
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

void StrategyManager::resetOdometryOnStateEntry(RobotState trackedState, DriveBase& drive) {
    if (odometryReferenceState != trackedState) {
        drive.resetOdometry();
        odometryReferenceState = trackedState;
    }
}

bool StrategyManager::driveForwardForDistance(DriveBase& drive,
                                              float targetCm,
                                              unsigned long timeoutMs) {
    resetOdometryOnStateEntry(state, drive);

    unsigned long elapsed = millis() - stateStartMs;
    bool reached = drive.getTravelDistanceCm() >= targetCm;
    bool timedOut = elapsed >= timeoutMs;

    if (!reached && !timedOut) {
        drive.forward(DRIVE_FORWARD_RPM);
        return false;
    }

    drive.stop();
    return true;
}

bool StrategyManager::driveBackwardForDistance(DriveBase& drive,
                                               float targetCm,
                                               unsigned long timeoutMs) {
    resetOdometryOnStateEntry(state, drive);

    unsigned long elapsed = millis() - stateStartMs;
    bool reached = drive.getTravelDistanceCm() >= targetCm;
    bool timedOut = elapsed >= timeoutMs;

    if (!reached && !timedOut) {
        drive.backward(DRIVE_FORWARD_RPM);
        return false;
    }

    drive.stop();
    return true;
}

bool StrategyManager::rotateRightForAngle(DriveBase& drive,
                                          float targetDeg,
                                          unsigned long timeoutMs) {
    resetOdometryOnStateEntry(RobotState::AVOID_OBSTACLE, drive);

    unsigned long elapsed = millis() - stateStartMs;
    bool reached = drive.getTurnAngleDeg() >= targetDeg;
    bool timedOut = elapsed >= timeoutMs;

    if (!reached && !timedOut) {
        drive.rotateRight(DRIVE_TURN_RPM);
        return false;
    }

    drive.stop();
    return true;
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

// ============================================================================
// MACHINE A ETATS DE STRATEGIE
// ============================================================================

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

    bool canAvoidObstacle =
        state == RobotState::GO_TO_BOX_ZONE ||
        state == RobotState::GO_TO_DROP_ZONE ||
        state == RobotState::GO_TO_THERMOMETER ||
        state == RobotState::RETURN_TO_NEST;

    if (canAvoidObstacle && distances.obstacle) {
        drive.stop();
        stateBeforeAvoidance = state;
        changeState(RobotState::AVOID_OBSTACLE);
        return;
    }

    switch (state) {
        case RobotState::WAIT_START:
            drive.stop();
            if (startPressed || AUTO_START_WITHOUT_BUTTON) {
                matchStartMs = millis();
                changeState(RobotState::GO_TO_BOX_ZONE);
            }
            break;

        case RobotState::GO_TO_BOX_ZONE:
            if (driveForwardForDistance(drive, DIST_TO_BOX_ZONE_CM, MOVE_TO_BOX_TIMEOUT_MS)) {
                changeState(RobotState::PICK_BOX);
            }
            break;

        case RobotState::PICK_BOX: {
            ActionResult result = actions.pickBox(servos);

            if (result == ActionResult::DONE) {
                actions.resetAction();
                changeState(RobotState::GO_TO_DROP_ZONE);
            } else if (result == ActionResult::FAILED) {
                Serial.println("[STRATEGY] Timeout prelevement - continuant");
                actions.resetAction();
                changeState(RobotState::GO_TO_DROP_ZONE);
            }
            break;
        }

        case RobotState::GO_TO_DROP_ZONE:
            if (driveForwardForDistance(drive, DIST_TO_DROP_ZONE_CM, MOVE_TO_DROP_TIMEOUT_MS)) {
                changeState(RobotState::DROP_BOX);
            }
            break;

        case RobotState::DROP_BOX: {
            ActionResult result = actions.dropBox(servos);

            if (result == ActionResult::DONE) {
                actions.resetAction();
                changeState(RobotState::GO_TO_THERMOMETER);
            } else if (result == ActionResult::FAILED) {
                Serial.println("[STRATEGY] Timeout depot - continuant");
                actions.resetAction();
                changeState(RobotState::GO_TO_THERMOMETER);
            }
            break;
        }

        case RobotState::GO_TO_THERMOMETER:
            if (driveForwardForDistance(drive,
                                        DIST_TO_THERMOMETER_CM,
                                        MOVE_TO_THERMOMETER_TIMEOUT_MS)) {
                changeState(RobotState::PUSH_CURSOR);
            }
            break;

        case RobotState::PUSH_CURSOR: {
            ActionResult result = actions.pushCursor(servos);

            if (result == ActionResult::DONE) {
                actions.resetAction();
                changeState(RobotState::RETURN_TO_NEST);
            } else if (result == ActionResult::FAILED) {
                Serial.println("[STRATEGY] Timeout curseur - continuant");
                actions.resetAction();
                changeState(RobotState::RETURN_TO_NEST);
            }
            break;
        }

        case RobotState::RETURN_TO_NEST:
            if (driveBackwardForDistance(drive,
                                         DIST_RETURN_TO_NEST_CM,
                                         RETURN_TO_NEST_TIMEOUT_MS)) {
                actions.returnHome();
                changeState(RobotState::END_MATCH);
            }
            break;

        case RobotState::AVOID_OBSTACLE: {
            unsigned long elapsed = millis() - stateStartMs;

            if (elapsed < AVOID_STOP_MS) {
                drive.stop();
                odometryReferenceState = RobotState::WAIT_START;
            } else if (rotateRightForAngle(drive,
                                           AVOID_TURN_DEG,
                                           AVOID_STOP_MS + AVOID_TURN_MS)) {
                changeState(stateBeforeAvoidance);
            }
            break;
        }

        case RobotState::EMERGENCY_STOP:
            drive.stop();
            Serial.println("[STRATEGY] ARRET D'URGENCE");
            break;

        case RobotState::END_MATCH:
            drive.stop();
            Serial.println("[STRATEGY] FIN DU MATCH");
            break;

        default:
            drive.stop();
            break;
    }
}
