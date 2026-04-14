#include "strategy.h"  // Inclusion du header pour la classe StrategyManager
#include "config.h"    // Inclusion du header pour les configurations

// Durée maximale du match en millisecondes (100 secondes)
constexpr unsigned long MATCH_DURATION_MS = 100000;

// Initialisation du gestionnaire de stratégie : met l'état initial à WAIT_START
void StrategyManager::init() {
    state = RobotState::WAIT_START;  // État initial : attendre le départ
    stateStartMs = millis();         // Timestamp du début de l'état actuel
    matchStartMs = 0;                // Timestamp du début du match, initialisé à 0
}

// Change l'état du robot si différent du courant, et met à jour le timestamp
void StrategyManager::changeState(RobotState newState) {
    if (state != newState) {          // Vérifie si l'état change
        state = newState;             // Met à jour l'état
        stateStartMs = millis();      // Met à jour le timestamp de l'état
    }
}

// Retourne l'état actuel du robot
RobotState StrategyManager::getState() const {
    return state;  // Retourne l'état courant
}

// Met à jour la stratégie en utilisant les données réelles du système de sécurité, distances, etc.
void StrategyManager::update(const SafetySystem& safety,
                             const DistanceReadings& distances,
                             DriveBase& drive,
                             ServoController& servos) {
    coreUpdate(  // Appelle la fonction core commune
        safety.isStartPressed(),     // Bouton de départ pressé ?
        safety.isEStopPressed(),     // Bouton d'arrêt d'urgence pressé ?
        distances,                   // Lectures des distances
        drive,                       // Contrôleur de conduite
        servos                       // Contrôleur des servos
    );
}

// Met à jour la stratégie en mode simulation avec des entrées simulées
void StrategyManager::updateSimulation(const SimInputs& sim,
                                       DriveBase& drive,
                                       ServoController& servos) {
    coreUpdate(  // Appelle la fonction core commune
        sim.startPressed,            // Simulation du bouton de départ
        sim.eStopPressed,            // Simulation du bouton d'arrêt d'urgence
        sim.distances,               // Distances simulées
        drive,                       // Contrôleur de conduite
        servos                       // Contrôleur des servos
    );
}

// Fonction centrale de mise à jour de la stratégie, commune aux modes réel et simulation
void StrategyManager::coreUpdate(bool startPressed,
                                 bool eStopPressed,
                                 const DistanceReadings& distances,
                                 DriveBase& drive,
                                 ServoController& servos) {
    (void)servos;  // Supprime l'avertissement de paramètre non utilisé

    // Vérifie l'arrêt d'urgence en priorité
    if (eStopPressed) {
        drive.stop();                           // Arrête le robot
        changeState(RobotState::EMERGENCY_STOP); // Passe à l'état d'urgence
        return;                                 // Sort de la fonction
    }

    // Vérifie si le match est terminé (durée dépassée)
    if (matchStartMs != 0 && (millis() - matchStartMs >= MATCH_DURATION_MS)) {
        drive.stop();                      // Arrête le robot
        changeState(RobotState::END_MATCH); // Passe à l'état de fin de match
        return;                            // Sort de la fonction
    }

    // Machine à états pour gérer le comportement du robot
    switch (state) {
        case RobotState::WAIT_START:  // État : attendre le départ
            drive.stop();             // Le robot est arrêté
            if (startPressed) {       // Si le bouton de départ est pressé
                matchStartMs = millis();             // Enregistre le début du match
                changeState(RobotState::RUN_FORWARD); // Passe à l'état d'avancement
            }
            break;

        case RobotState::RUN_FORWARD:  // État : avancer
            if (distances.obstacle) {  // Si un obstacle est détecté
                drive.stop();                           // Arrête le robot
                changeState(RobotState::AVOID_OBSTACLE); // Passe à l'état d'évitement
            } else {
                drive.forward(DRIVE_FORWARD_RPM);  // Avance à vitesse normale
            }
            break;

        case RobotState::AVOID_OBSTACLE: {  // État : éviter l'obstacle
            unsigned long elapsed = millis() - stateStartMs;  // Temps écoulé depuis le début de l'état

            if (elapsed < AVOID_STOP_MS) {           // Phase d'arrêt initial
                drive.stop();                        // Le robot s'arrête
            } else if (elapsed < (AVOID_STOP_MS + AVOID_TURN_MS)) {  // Phase de rotation
                drive.rotateRight(DRIVE_TURN_RPM);   // Tourne à droite
            } else {                                 // Fin de l'évitement
                changeState(RobotState::RUN_FORWARD); // Retourne à l'avancement
            }
            break;
        }

        case RobotState::EMERGENCY_STOP:  // État : arrêt d'urgence
            drive.stop();                 // Le robot reste arrêté
            break;

        case RobotState::END_MATCH:  // État : fin du match
            drive.stop();            // Le robot reste arrêté
            break;
    }
}