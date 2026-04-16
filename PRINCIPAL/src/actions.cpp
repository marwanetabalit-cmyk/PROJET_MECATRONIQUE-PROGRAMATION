#include "actions.h"

void ActionManager::init() {
    Serial.println("[ACTION] ActionManager initialise");
}

void ActionManager::logAction(const char* message) {
    Serial.print("[ACTION] ");
    Serial.println(message);
}

ActionResult ActionManager::splitBoxes(ServoController& servos) {
    logAction("Debut splitBoxes");

    servos.splitOpen();
    delay(500);

    servos.splitClose();
    delay(500);

    logAction("Fin splitBoxes");
    return ActionResult::DONE;
}

ActionResult ActionManager::gripBox(ServoController& servos) {
    logAction("Debut gripBox");

    servos.gripOpen();
    delay(400);

    servos.gripClose();
    delay(600);

    logAction("Fin gripBox");
    return ActionResult::DONE;
}

ActionResult ActionManager::liftBox(ServoController& servos) {
    logAction("Debut liftBox");

    servos.liftUp();
    delay(700);

    logAction("Fin liftBox");
    return ActionResult::DONE;
}

ActionResult ActionManager::pickBox(ServoController& servos) {
    logAction("Debut pickBox");

    servos.splitOpen();
    delay(400);

    servos.gripOpen();
    delay(400);

    servos.liftDown();
    delay(600);

    servos.gripClose();
    delay(700);

    servos.liftUp();
    delay(700);

    servos.splitClose();
    delay(400);

    logAction("Fin pickBox");
    return ActionResult::DONE;
}

ActionResult ActionManager::dropBox(ServoController& servos) {
    logAction("Debut dropBox");

    servos.liftDown();
    delay(700);

    servos.gripOpen();
    delay(600);

    servos.liftUp();
    delay(500);

    logAction("Fin dropBox");
    return ActionResult::DONE;
}

ActionResult ActionManager::pushCursor(ServoController& servos) {
    logAction("Debut pushCursor");

    servos.cursorPush();
    delay(700);

    servos.cursorHome();
    delay(700);

    logAction("Fin pushCursor");
    return ActionResult::DONE;
}

ActionResult ActionManager::returnHome() {
    logAction("Retour au nid demande");
    delay(500);

    logAction("Retour au nid termine");
    return ActionResult::DONE;
}