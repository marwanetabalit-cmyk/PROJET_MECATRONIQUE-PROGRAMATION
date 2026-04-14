#pragma once
#include <Arduino.h>
#include "servos_ctrl.h"

enum class ActionResult {
    DONE,
    FAILED
};

class ActionManager {
public:
    void init();

    ActionResult splitBoxes(ServoController& servos);
    ActionResult gripBox(ServoController& servos);
    ActionResult liftBox(ServoController& servos);

    ActionResult pickBox(ServoController& servos);
    ActionResult dropBox(ServoController& servos);

    ActionResult pushCursor(ServoController& servos);
    ActionResult returnHome();

private:
    void logAction(const char* message);
};