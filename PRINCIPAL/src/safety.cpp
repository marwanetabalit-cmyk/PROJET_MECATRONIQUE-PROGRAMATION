#include "safety.h"
#include "config.h"

// ============================================================================
// INITIALISATION DES ENTRÉES DE SÉCURITÉ
// ============================================================================

void SafetySystem::init() {
    // Configurer le bouton de départ
    // INPUT_PULLUP signifie: pullup interne activé, LOW = pressé, HIGH = non-pressé
    pinMode(PIN_START_SWITCH, INPUT_PULLUP);

    // Configurer l'arrêt d'urgence (E-Stop)
    // INPUT_PULLUP signifie: pullup interne activé, LOW = pressé, HIGH = non-pressé
    pinMode(PIN_ESTOP, INPUT_PULLUP);
}

// ============================================================================
// LECTURE DES SIGNAUX DE SÉCURITÉ
// ============================================================================

/// Teste si le bouton de départ a été pressé
/// @return true si le bouton est enfoncé (logique inversée: active bas)
bool SafetySystem::isStartPressed() const {
    return digitalRead(PIN_START_SWITCH) == LOW;  // Actif bas (logique inversée)
}

/// Teste si l'arrêt d'urgence a été activé
/// @return true si l'E-Stop est enfoncé (logique inversée: active bas)
bool SafetySystem::isEStopPressed() const {
    return digitalRead(PIN_ESTOP) == LOW;  // Actif bas (logique inversée)
}