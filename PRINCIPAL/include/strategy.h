#pragma once
#include <Arduino.h>
#include "safety.h"
#include "ultrasons.h"
#include "drive.h"
#include "servos_ctrl.h"
#include "actions.h"

enum class RobotState {
    WAIT_START,
    SCENARIO_TRANSLATE,
    SCENARIO_MANIP,
    SCENARIO_ROTATION_PAUSE_BEFORE,
    SCENARIO_ROTATE,
    SCENARIO_ROTATION_PAUSE_AFTER,
    AVOID_OBSTACLE,
    EMERGENCY_STOP,
    END_MATCH
};

struct SimInputs {
    bool startPressed = false;
    bool eStopPressed = false;
    DistanceReadings distances;
};

class StrategyManager {
public:
    void init();

    void update(const SafetySystem& safety,
                const DistanceReadings& distances,
                DriveBase& drive,
                ServoController& servos,
                ActionManager& actions);

    void updateSimulation(const SimInputs& sim,
                          DriveBase& drive,
                          ServoController& servos,
                          ActionManager& actions);

    RobotState getState() const;
    const char* getStateName() const;
    uint8_t getCurrentStep() const;

private:
    RobotState state = RobotState::WAIT_START;
    RobotState stateBeforeAvoidance = RobotState::WAIT_START;
    RobotState odometryReferenceState = RobotState::WAIT_START;
    unsigned long stateStartMs = 0;
    unsigned long matchStartMs = 0;
    uint8_t currentStep = 0;

    void changeState(RobotState newState);
    void resetOdometryOnStateEntry(RobotState trackedState, DriveBase& drive);
    void startScenario(DriveBase& drive);
    void advanceToNextStep();
    void goToRotationOrNextStep();

    bool driveForSignedDistance(DriveBase& drive, float targetCm);
    bool rotateForAngle(DriveBase& drive, float targetDeg);
    bool rotateRightForAvoidance(DriveBase& drive);
    bool waitInState(unsigned long durationMs, DriveBase& drive);

    unsigned long moveTimeoutForDistance(float distanceCm) const;
    unsigned long rotationTimeoutForAngle(float angleDeg) const;

    void coreUpdate(bool startPressed,
                    bool eStopPressed,
                    const DistanceReadings& distances,
                    DriveBase& drive,
                    ServoController& servos,
                    ActionManager& actions);
};
