#include "strategy.h"
#include "config.h"
#include <math.h>

enum class ScenarioManip {
    NONE,
    PICK,
    DROP
};

struct ScenarioStep {
    const char* name;
    float distanceCm;
    float rotationDeg;
    ScenarioManip manip;
};

static const ScenarioStep SCENARIO[] = {
    {"Aller chercher 1re boite",          25.0f,   0.0f, ScenarioManip::PICK},
    {"Poser 1re boite",                   40.0f,   0.0f, ScenarioManip::DROP},
    {"Prendre 2e boite",                  40.0f,   0.0f, ScenarioManip::PICK},
    {"Se deplacer + rotation",            40.0f,  90.0f, ScenarioManip::NONE},
    {"Avancer + boite",                   35.0f,   0.0f, ScenarioManip::DROP},
    {"Prendre 3e boite",                  40.0f,   0.0f, ScenarioManip::PICK},
    {"Poser 3e boite + rotation",         40.0f,  90.0f, ScenarioManip::DROP},
    {"Se deplacer + rotation",            85.0f,  90.0f, ScenarioManip::NONE},
    {"Se deplacer + rotation",            55.0f,  90.0f, ScenarioManip::NONE},
    {"Se deplacer + rotation",            25.0f,  90.0f, ScenarioManip::NONE},
    {"Prendre derniere boite",            10.0f,   0.0f, ScenarioManip::PICK},
    {"Reculer + poser derniere boite",   -35.0f,   0.0f, ScenarioManip::DROP},
    {"Reculer + rotation",               -20.0f,  90.0f, ScenarioManip::NONE},
    {"Retour a la base",                  85.0f,   0.0f, ScenarioManip::NONE},
};

static constexpr uint8_t SCENARIO_STEP_COUNT =
    static_cast<uint8_t>(sizeof(SCENARIO) / sizeof(SCENARIO[0]));

void StrategyManager::init() {
    state = RobotState::WAIT_START;
    stateBeforeAvoidance = RobotState::WAIT_START;
    odometryReferenceState = RobotState::WAIT_START;
    stateStartMs = millis();
    matchStartMs = 0;
    currentStep = 0;
}

void StrategyManager::changeState(RobotState newState) {
    if (state != newState) {
        state = newState;
        stateStartMs = millis();
        odometryReferenceState = RobotState::WAIT_START;
    }
}

RobotState StrategyManager::getState() const {
    return state;
}

const char* StrategyManager::getStateName() const {
    switch (state) {
        case RobotState::WAIT_START:
            return "WAIT_START";
        case RobotState::SCENARIO_TRANSLATE:
            return "SCENARIO_TRANSLATE";
        case RobotState::SCENARIO_MANIP:
            return "SCENARIO_MANIP";
        case RobotState::SCENARIO_ROTATION_PAUSE_BEFORE:
            return "ROTATION_PAUSE_BEFORE";
        case RobotState::SCENARIO_ROTATE:
            return "SCENARIO_ROTATE";
        case RobotState::SCENARIO_ROTATION_PAUSE_AFTER:
            return "ROTATION_PAUSE_AFTER";
        case RobotState::AVOID_OBSTACLE:
            return "AVOID_OBSTACLE";
        case RobotState::EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        case RobotState::END_MATCH:
            return "END_MATCH";
        default:
            return "UNKNOWN";
    }
}

uint8_t StrategyManager::getCurrentStep() const {
    return currentStep;
}

void StrategyManager::resetOdometryOnStateEntry(RobotState trackedState, DriveBase& drive) {
    if (odometryReferenceState != trackedState) {
        drive.resetTravelCounters();
        odometryReferenceState = trackedState;
    }
}

void StrategyManager::startScenario(DriveBase& drive) {
    currentStep = 0;
    matchStartMs = millis();
    drive.resetOdometry(
        STRATEGY_START_X_CM,
        STRATEGY_START_Y_CM,
        STRATEGY_START_THETA_RAD
    );
    changeState(RobotState::SCENARIO_TRANSLATE);
}

void StrategyManager::advanceToNextStep() {
    if (currentStep + 1 >= SCENARIO_STEP_COUNT) {
        changeState(RobotState::END_MATCH);
        return;
    }

    ++currentStep;
    changeState(RobotState::SCENARIO_TRANSLATE);
}

void StrategyManager::goToRotationOrNextStep() {
    if (currentStep >= SCENARIO_STEP_COUNT) {
        changeState(RobotState::END_MATCH);
        return;
    }

    if (fabsf(SCENARIO[currentStep].rotationDeg) > 0.01f) {
        changeState(RobotState::SCENARIO_ROTATION_PAUSE_BEFORE);
    } else {
        advanceToNextStep();
    }
}

unsigned long StrategyManager::moveTimeoutForDistance(float distanceCm) const {
    float durationMs = (fabsf(distanceCm) / STRATEGY_LINEAR_SPEED_CM_S) * 1000.0f;
    return static_cast<unsigned long>(durationMs) + STRATEGY_MOVE_TIMEOUT_MARGIN_MS;
}

unsigned long StrategyManager::rotationTimeoutForAngle(float angleDeg) const {
    float durationMs = (fabsf(angleDeg) / 90.0f) * static_cast<float>(STRATEGY_ROTATION_90_MS);
    return static_cast<unsigned long>(durationMs) + STRATEGY_ROTATION_TIMEOUT_MARGIN_MS;
}

bool StrategyManager::driveForSignedDistance(DriveBase& drive, float targetCm) {
    resetOdometryOnStateEntry(RobotState::SCENARIO_TRANSLATE, drive);

    float targetAbsCm = fabsf(targetCm);
    unsigned long elapsed = millis() - stateStartMs;
    bool reached = drive.getTravelDistanceCm() >= targetAbsCm;
    bool timedOut = elapsed >= moveTimeoutForDistance(targetCm);

    if (targetAbsCm <= 0.01f || reached || timedOut) {
        drive.stop();
        return true;
    }

    if (targetCm >= 0.0f) {
        drive.forward(DRIVE_FORWARD_RPM);
    } else {
        drive.backward(DRIVE_FORWARD_RPM);
    }

    return false;
}

bool StrategyManager::rotateForAngle(DriveBase& drive, float targetDeg) {
    resetOdometryOnStateEntry(RobotState::SCENARIO_ROTATE, drive);

    float targetAbsDeg = fabsf(targetDeg);
    unsigned long elapsed = millis() - stateStartMs;
    bool reached = drive.getTurnAngleDeg() >= targetAbsDeg;
    bool timedOut = elapsed >= rotationTimeoutForAngle(targetDeg);

    if (targetAbsDeg <= 0.01f || reached || timedOut) {
        drive.stop();
        return true;
    }

    if (targetDeg >= 0.0f) {
        drive.rotateLeft(DRIVE_TURN_RPM);
    } else {
        drive.rotateRight(DRIVE_TURN_RPM);
    }

    return false;
}

bool StrategyManager::rotateRightForAvoidance(DriveBase& drive) {
    resetOdometryOnStateEntry(RobotState::AVOID_OBSTACLE, drive);

    unsigned long elapsed = millis() - stateStartMs;
    bool reached = drive.getTurnAngleDeg() >= AVOID_TURN_DEG;
    bool timedOut = elapsed >= (AVOID_STOP_MS + AVOID_TURN_MS);

    if (reached || timedOut) {
        drive.stop();
        return true;
    }

    drive.rotateRight(DRIVE_TURN_RPM);
    return false;
}

bool StrategyManager::waitInState(unsigned long durationMs, DriveBase& drive) {
    drive.stop();
    return (millis() - stateStartMs) >= durationMs;
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

    if (state == RobotState::SCENARIO_TRANSLATE && distances.obstacle) {
        drive.stop();
        stateBeforeAvoidance = state;
        changeState(RobotState::AVOID_OBSTACLE);
        return;
    }

    switch (state) {
        case RobotState::WAIT_START:
            drive.stop();
            if (startPressed || AUTO_START_WITHOUT_BUTTON) {
                actions.resetAction();
                startScenario(drive);
            }
            break;

        case RobotState::SCENARIO_TRANSLATE:
            if (currentStep >= SCENARIO_STEP_COUNT) {
                changeState(RobotState::END_MATCH);
            } else if (driveForSignedDistance(drive, SCENARIO[currentStep].distanceCm)) {
                changeState(RobotState::SCENARIO_MANIP);
            }
            break;

        case RobotState::SCENARIO_MANIP:
            if (currentStep >= SCENARIO_STEP_COUNT) {
                changeState(RobotState::END_MATCH);
                break;
            }

            if (SCENARIO[currentStep].manip == ScenarioManip::NONE) {
                goToRotationOrNextStep();
                break;
            }

            {
                ActionResult result = ActionResult::IN_PROGRESS;

                if (SCENARIO[currentStep].manip == ScenarioManip::PICK) {
                    result = actions.pickBox(servos);
                } else if (SCENARIO[currentStep].manip == ScenarioManip::DROP) {
                    result = actions.dropBox(servos);
                }

                if (result == ActionResult::DONE || result == ActionResult::FAILED) {
                    if (result == ActionResult::FAILED) {
                        Serial.print("[STRATEGY] Manip timeout etape ");
                        Serial.println(currentStep + 1);
                    }
                    actions.resetAction();
                    goToRotationOrNextStep();
                }
            }
            break;

        case RobotState::SCENARIO_ROTATION_PAUSE_BEFORE:
            if (waitInState(STRATEGY_ROTATION_PAUSE_MS, drive)) {
                changeState(RobotState::SCENARIO_ROTATE);
            }
            break;

        case RobotState::SCENARIO_ROTATE:
            if (currentStep >= SCENARIO_STEP_COUNT) {
                changeState(RobotState::END_MATCH);
            } else if (rotateForAngle(drive, SCENARIO[currentStep].rotationDeg)) {
                changeState(RobotState::SCENARIO_ROTATION_PAUSE_AFTER);
            }
            break;

        case RobotState::SCENARIO_ROTATION_PAUSE_AFTER:
            if (waitInState(STRATEGY_ROTATION_PAUSE_MS, drive)) {
                advanceToNextStep();
            }
            break;

        case RobotState::AVOID_OBSTACLE: {
            unsigned long elapsed = millis() - stateStartMs;

            if (elapsed < AVOID_STOP_MS) {
                drive.stop();
            } else if (rotateRightForAvoidance(drive)) {
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
            break;

        default:
            drive.stop();
            changeState(RobotState::END_MATCH);
            break;
    }
}
