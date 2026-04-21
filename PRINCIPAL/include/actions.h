#pragma once
#include <Arduino.h>
#include "servos_ctrl.h"

/// Résultat d'une action
enum class ActionResult {
    IN_PROGRESS,  ///< Action en cours d'exécution (non-bloquante)
    DONE,         ///< Action terminée avec succès
    FAILED        ///< Action échouée (timeout)
};

/// État interne d'une action multi-étape
enum class ActionState {
    IDLE,           ///< Aucune action en cours
    SPLIT_OPEN,     ///< Étape: ouvrir le séparateur
    SPLIT_CLOSE,    ///< Étape: fermer le séparateur
    GRIP_OPEN,      ///< Étape: ouvrir la pince
    GRIP_CLOSE,     ///< Étape: fermer la pince
    LIFT_UP,        ///< Étape: lever le bras
    LIFT_DOWN,      ///< Étape: baisser le bras
    CURSOR_PUSH,    ///< Étape: pousser le curseur
    CURSOR_HOME     ///< Étape: ramener le curseur à domicile
};

class ActionManager {
public:
    /// Initialise le gestionnaire d'actions
    void init();

    /// Lance une action de séparation de boîtes (non-bloquante)
    /// @return IN_PROGRESS si en cours, DONE si terminée
    ActionResult splitBoxes(ServoController& servos);

    /// Lance une action de saisie de boîte (non-bloquante)
    /// @return IN_PROGRESS si en cours, DONE si terminée
    ActionResult gripBox(ServoController& servos);

    /// Lance une action de levage de boîte (non-bloquante)
    /// @return IN_PROGRESS si en cours, DONE si terminée
    ActionResult liftBox(ServoController& servos);

    /// Lance l'action complète de prélèvement de boîte (non-bloquante)
    /// @return IN_PROGRESS si en cours, DONE si terminée
    ActionResult pickBox(ServoController& servos);

    /// Lance l'action complète de dépôt de boîte (non-bloquante)
    /// @return IN_PROGRESS si en cours, DONE si terminée
    ActionResult dropBox(ServoController& servos);

    /// Lance l'action de poussée du curseur (non-bloquante)
    /// @return IN_PROGRESS si en cours, DONE si terminée
    ActionResult pushCursor(ServoController& servos);

    /// Ramène le robot à l'état initial (homing)
    /// @return Toujours DONE (opération instantanée sur servos)
    ActionResult returnHome();

    /// Teste si une action est en cours
    bool isActionInProgress() const;

    /// Réinitialise l'état interne
    void resetAction();

private:
    /// État courant de l'action multi-étape
    ActionState currentAction = ActionState::IDLE;
    
    /// Timestamp du début de l'étape actuelle
    unsigned long stateStartMs = 0;
    
    /// Durée maximale d'une étape (timeout)
    static constexpr unsigned long ACTION_STEP_TIMEOUT_MS = 1500;

    /// Dirige l'exécution d'une action étape par étape (machine à états interne)
    /// @param servos Contrôleur de servos
    /// @param steps Sequence ordonnee des etapes a executer
    /// @param stepDurations Tableau des durées de chaque étape (en ms)
    /// @param stepCount Nombre d'étapes
    /// @return IN_PROGRESS si en cours, DONE si terminée, FAILED si timeout
    ActionResult executeStepByStep(ServoController& servos,
                                    const ActionState* steps,
                                    const unsigned long* stepDurations,
                                    uint8_t stepCount);

    /// Enregistre un message de log formaté
    void logAction(const char* message);
};
