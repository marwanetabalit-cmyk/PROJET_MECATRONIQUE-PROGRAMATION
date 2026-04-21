#include "actions.h"

// ============================================================================
// INITIALISATION ET UTILITAIRES
// ============================================================================

void ActionManager::init() {
    Serial.println("[ACTION] ActionManager initialise");
    currentAction = ActionState::IDLE;
    stateStartMs = 0;
}

void ActionManager::logAction(const char* message) {
    Serial.print("[ACTION] ");
    Serial.println(message);
}

bool ActionManager::isActionInProgress() const {
    return currentAction != ActionState::IDLE;
}

void ActionManager::resetAction() {
    currentAction = ActionState::IDLE;
    stateStartMs = 0;
}

// ============================================================================
// MACHINE A ETATS NON-BLOQUANTE
// ============================================================================

ActionResult ActionManager::executeStepByStep(ServoController& servos,
                                               const ActionState* steps,
                                               const unsigned long* stepDurations,
                                               uint8_t stepCount) {
    if (stepCount == 0) {
        return ActionResult::DONE;
    }

    // Première invocation: initialiser l'action
    if (currentAction == ActionState::IDLE) {
        currentAction = steps[0];
        stateStartMs = millis();
        logAction("Action demarree");
        return ActionResult::IN_PROGRESS;
    }

    unsigned long elapsed = millis() - stateStartMs;

    // Vérifier le timeout
    if (elapsed > ACTION_STEP_TIMEOUT_MS) {
        logAction("Action timeout - ECHEC");
        currentAction = ActionState::IDLE;
        return ActionResult::FAILED;
    }

    // Déterminer l'étape actuelle et exécuter la commande servo
    uint8_t stepIndex = stepCount;
    for (uint8_t i = 0; i < stepCount; ++i) {
        if (currentAction == steps[i]) {
            stepIndex = i;
            break;
        }
    }

    if (stepIndex >= stepCount) {
        logAction("Action invalide - ECHEC");
        currentAction = ActionState::IDLE;
        return ActionResult::FAILED;
    }

    if (stepIndex < stepCount) {
        // Commander le servo correspondant à l'étape actuelle
        switch (currentAction) {
            case ActionState::SPLIT_OPEN:   servos.splitOpen(); break;
            case ActionState::SPLIT_CLOSE:  servos.splitClose(); break;
            case ActionState::GRIP_OPEN:    servos.gripOpen(); break;
            case ActionState::GRIP_CLOSE:   servos.gripClose(); break;
            case ActionState::LIFT_UP:      servos.liftUp(); break;
            case ActionState::LIFT_DOWN:    servos.liftDown(); break;
            case ActionState::CURSOR_PUSH:  servos.cursorPush(); break;
            case ActionState::CURSOR_HOME:  servos.cursorHome(); break;
            default: break;
        }

        // Vérifier si suffisamment de temps s'est écoulé pour passer à l'étape suivante
        if (elapsed >= stepDurations[stepIndex]) {
            // Vérifier si c'était la dernière étape
            if (stepIndex + 1 >= stepCount) {
                logAction("Action terminee");
                currentAction = ActionState::IDLE;
                return ActionResult::DONE;
            }

            // Passer à l'étape suivante
            currentAction = steps[stepIndex + 1];
            stateStartMs = millis();
        }
    }

    return ActionResult::IN_PROGRESS;
}

// ============================================================================
// ACTIONS COMPOSITES NON-BLOQUANTES
// ============================================================================

ActionResult ActionManager::splitBoxes(ServoController& servos) {
    // Étapes: SPLIT_OPEN -> SPLIT_CLOSE
    // Durées: 500ms pour ouvrir, 500ms pour fermer
    static const ActionState steps[] = {ActionState::SPLIT_OPEN, ActionState::SPLIT_CLOSE};
    static const unsigned long stepDurations[] = {500, 500};
    return executeStepByStep(servos,
                              steps,
                              stepDurations,
                              2);
}

ActionResult ActionManager::gripBox(ServoController& servos) {
    // Étapes: GRIP_OPEN -> GRIP_CLOSE
    // Durées: 400ms pour ouvrir, 600ms pour fermer
    static const ActionState steps[] = {ActionState::GRIP_OPEN, ActionState::GRIP_CLOSE};
    static const unsigned long stepDurations[] = {400, 600};
    return executeStepByStep(servos,
                              steps,
                              stepDurations,
                              2);
}

ActionResult ActionManager::liftBox(ServoController& servos) {
    // Étape unique: LIFT_UP
    // Durée: 700ms pour lever
    static const ActionState steps[] = {ActionState::LIFT_UP};
    static const unsigned long stepDurations[] = {700};
    return executeStepByStep(servos,
                              steps,
                              stepDurations,
                              1);
}

ActionResult ActionManager::pickBox(ServoController& servos) {
    // Séquence complète de prélèvement:
    // 1. Ouvrir séparateur (500ms)
    // 2. Ouvrir pince (400ms)
    // 3. Baisser bras (600ms)
    // 4. Fermer pince (700ms)
    // 5. Lever bras (700ms)
    // 6. Fermer séparateur (400ms)
    static const ActionState steps[] = {
        ActionState::SPLIT_OPEN,
        ActionState::GRIP_OPEN,
        ActionState::LIFT_DOWN,
        ActionState::GRIP_CLOSE,
        ActionState::LIFT_UP,
        ActionState::SPLIT_CLOSE
    };
    static const unsigned long stepDurations[] = {500, 400, 600, 700, 700, 400};
    return executeStepByStep(servos,
                              steps,
                              stepDurations,
                              6);
}

ActionResult ActionManager::dropBox(ServoController& servos) {
    // Séquence de déposition:
    // 1. Baisser bras (700ms)
    // 2. Ouvrir pince (600ms)
    // 3. Lever bras (500ms)
    static const ActionState steps[] = {
        ActionState::LIFT_DOWN,
        ActionState::GRIP_OPEN,
        ActionState::LIFT_UP
    };
    static const unsigned long stepDurations[] = {700, 600, 500};
    return executeStepByStep(servos,
                              steps,
                              stepDurations,
                              3);
}

ActionResult ActionManager::pushCursor(ServoController& servos) {
    // Séquence du curseur thermomètre:
    // 1. Pousser curseur (700ms)
    // 2. Ramener home (700ms)
    static const ActionState steps[] = {ActionState::CURSOR_PUSH, ActionState::CURSOR_HOME};
    static const unsigned long stepDurations[] = {700, 700};
    return executeStepByStep(servos,
                              steps,
                              stepDurations,
                              2);
}

ActionResult ActionManager::returnHome() {
    // Ramener tous les servos à la position initiale (instantané dans servo)
    logAction("Retour position initiale");
    // Cette opération ne bloque pas puisque les servos gèrent eux-mêmes le positionnement
    return ActionResult::DONE;
}

