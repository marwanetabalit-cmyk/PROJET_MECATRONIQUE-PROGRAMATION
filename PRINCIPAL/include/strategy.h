#pragma once
#include <Arduino.h>
#include "safety.h"
#include "ultrasons.h"
#include "drive.h"
#include "servos_ctrl.h"
#include "actions.h"

enum class RobotState {
    WAIT_START,
    GO_TO_BOX_ZONE,
    PICK_BOX,
    GO_TO_DROP_ZONE,
    DROP_BOX,
    GO_TO_THERMOMETER,
    PUSH_CURSOR,
    RETURN_TO_NEST,
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

private:
    RobotState state = RobotState::WAIT_START;
    RobotState stateBeforeAvoidance = RobotState::WAIT_START;
    unsigned long stateStartMs = 0;
    unsigned long matchStartMs = 0;

    void changeState(RobotState newState);

    void coreUpdate(bool startPressed,
                    bool eStopPressed,
                    const DistanceReadings& distances,
                    DriveBase& drive,
                    ServoController& servos,
                    ActionManager& actions);
};
