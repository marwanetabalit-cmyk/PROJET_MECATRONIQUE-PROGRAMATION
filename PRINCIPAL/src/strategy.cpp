#include "strategy.h"
#include "config.h"

// ============================================================================
// CYCLE DE VIE ET GESTION D'ÉTATS
// ============================================================================

void StrategyManager::init() {
    // Initialiser la machine à états: attendre le signal de départ
    state = RobotState::WAIT_START;
    stateBeforeAvoidance = RobotState::WAIT_START;
    stateStartMs = millis();
    matchStartMs = 0;
}

void StrategyManager::changeState(RobotState newState) {
    // Éviter les changements inutiles et mémoriser le timestamp
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
    // Wrapper pour l'API sécurité réelle: transmettre au coreUpdate
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
    // Wrapper pour le mode simulation: transmettre au coreUpdate
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
// MACHINE À ÉTATS DE STRATÉGIE (NON-BLOQUANTE)
// ============================================================================

void StrategyManager::coreUpdate(bool startPressed,
                                 bool eStopPressed,
                                 const DistanceReadings& distances,
                                 DriveBase& drive,
                                 ServoController& servos,
                                 ActionManager& actions) {
    // ========================================================================
    // SÉCURITÉ CRITIQUE: E-STOP et fin de match
    // ========================================================================

    // Arrêt d'urgence: priorité absolue
    if (eStopPressed) {
        drive.stop();
        changeState(RobotState::EMERGENCY_STOP);
        return;
    }

    // Fin du match Eurobot: 90 secondes
    // Le match commence au démarrage (WAIT_START -> GO_TO_BOX_ZONE)
    if (matchStartMs != 0 && (millis() - matchStartMs >= MATCH_DURATION_MS)) {
        drive.stop();
        changeState(RobotState::END_MATCH);
        return;
    }

    // Évitement d'obstacle: priorité haute (sauf si déjà en cours)
    // Détecte un obstacle devant, sur les côtés
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

    // ========================================================================
    // MACHINE À ÉTATS: Logique de stratégie du match
    // ========================================================================

    switch (state) {
        // ====================================================================
        // État initial: attendre le signal de départ
        // ====================================================================
        case RobotState::WAIT_START:
            drive.stop();
            if (startPressed || AUTO_START_WITHOUT_BUTTON) {
                // Signal de départ reçu: lancer le chronomètre du match
                matchStartMs = millis();
                changeState(RobotState::GO_TO_BOX_ZONE);
            }
            break;

        // ====================================================================
        // Aller à la zone de boîtes (mouvement de 2 secondes)
        // ====================================================================
        case RobotState::GO_TO_BOX_ZONE: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 2000) {
                // Avancer à vitesse nominale
                drive.forward(DRIVE_FORWARD_RPM);
            } else {
                // Temps écoulé: arrêter et initier le prélèvement
                drive.stop();
                changeState(RobotState::PICK_BOX);
            }
            break;
        }

        // ====================================================================
        // Prélever une boîte (action non-bloquante multi-étapes)
        // ====================================================================
        case RobotState::PICK_BOX: {
            // Lancer/continuer l'action de prélèvement (non-bloquante)
            ActionResult result = actions.pickBox(servos);

            if (result == ActionResult::DONE) {
                // Action terminée: réinitialiser et passer à l'état suivant
                actions.resetAction();
                changeState(RobotState::GO_TO_DROP_ZONE);
            } else if (result == ActionResult::FAILED) {
                // Timeout: journaliser et continuer malgré tout
                Serial.println("[STRATEGY] Timeout prélèvement - continuant");
                actions.resetAction();
                changeState(RobotState::GO_TO_DROP_ZONE);
            }
            // IN_PROGRESS: rester dans cet état, boucler
            break;
        }

        // ====================================================================
        // Aller à la zone de dépôt (mouvement de 2 secondes)
        // ====================================================================
        case RobotState::GO_TO_DROP_ZONE: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 2000) {
                // Avancer à vitesse nominale
                drive.forward(DRIVE_FORWARD_RPM);
            } else {
                // Temps écoulé: arrêter et initier le dépôt
                drive.stop();
                changeState(RobotState::DROP_BOX);
            }
            break;
        }

        // ====================================================================
        // Déposer la boîte (action non-bloquante multi-étapes)
        // ====================================================================
        case RobotState::DROP_BOX: {
            // Lancer/continuer l'action de dépôt (non-bloquante)
            ActionResult result = actions.dropBox(servos);

            if (result == ActionResult::DONE) {
                // Action terminée: réinitialiser et passer à l'état suivant
                actions.resetAction();
                changeState(RobotState::GO_TO_THERMOMETER);
            } else if (result == ActionResult::FAILED) {
                // Timeout: journaliser et continuer malgré tout
                Serial.println("[STRATEGY] Timeout dépôt - continuant");
                actions.resetAction();
                changeState(RobotState::GO_TO_THERMOMETER);
            }
            // IN_PROGRESS: rester dans cet état, boucler
            break;
        }

        // ====================================================================
        // Aller au thermomètre (mouvement de 1.5 secondes)
        // ====================================================================
        case RobotState::GO_TO_THERMOMETER: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 1500) {
                // Avancer à vitesse nominale
                drive.forward(DRIVE_FORWARD_RPM);
            } else {
                // Temps écoulé: arrêter et initier la poussée du curseur
                drive.stop();
                changeState(RobotState::PUSH_CURSOR);
            }
            break;
        }

        // ====================================================================
        // Pousser le curseur thermomètre (action non-bloquante)
        // ====================================================================
        case RobotState::PUSH_CURSOR: {
            // Lancer/continuer l'action de poussée du curseur
            ActionResult result = actions.pushCursor(servos);

            if (result == ActionResult::DONE) {
                // Action terminée: réinitialiser et retourner au nid
                actions.resetAction();
                changeState(RobotState::RETURN_TO_NEST);
            } else if (result == ActionResult::FAILED) {
                // Timeout: journaliser et continuer
                Serial.println("[STRATEGY] Timeout curseur - continuant");
                actions.resetAction();
                changeState(RobotState::RETURN_TO_NEST);
            }
            // IN_PROGRESS: rester dans cet état, boucler
            break;
        }

        // ====================================================================
        // Retourner au nid (mouvement de ~ 5 secondes, arrière)
        // ====================================================================
        case RobotState::RETURN_TO_NEST: {
            unsigned long elapsed = millis() - stateStartMs;
            if (elapsed < 5000) {
                // Reculer à vitesse nominale pour retourner au nid
                drive.backward(DRIVE_FORWARD_RPM);
            } else {
                // Temps écoulé: arrêter, ramener le robot à domicile
                drive.stop();
                actions.returnHome();
                changeState(RobotState::END_MATCH);
            }
            break;
        }

        // ====================================================================
        // Évitement d'obstacle (rotation + pause)
        // ====================================================================
        case RobotState::AVOID_OBSTACLE: {
            unsigned long elapsed = millis() - stateStartMs;

            if (elapsed < AVOID_STOP_MS) {
                // Phase 1: Arrêter complètement (500ms)
                drive.stop();
            } else if (elapsed < (AVOID_STOP_MS + AVOID_TURN_MS)) {
                // Phase 2: Tourner à droite pour contourner (700ms)
                drive.rotateRight(DRIVE_TURN_RPM);
            } else {
                // Temps écoulé: reprendre la navigation vers la zone de boîtes
                changeState(stateBeforeAvoidance);
            }
            break;
        }

        // ====================================================================
        // Arrêt d'urgence: moteurs coupés
        // ====================================================================
        case RobotState::EMERGENCY_STOP:
            drive.stop();
            Serial.println("[STRATEGY] ARRÊT D'URGENCE");
            break;

        // ====================================================================
        // Fin du match: moteurs coupés, fin
        // ====================================================================
        case RobotState::END_MATCH:
            drive.stop();
            Serial.println("[STRATEGY] FIN DU MATCH");
            break;

        default:
            // État inconnu: sécurité d'abord
            drive.stop();
            break;
    }
}
