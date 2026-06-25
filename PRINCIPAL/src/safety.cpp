#include "safety.h"
#include "config.h"

// ============================================================================
// INITIALISATION DES ENTRÉES DE SÉCURITÉ
// ============================================================================

void SafetySystem::init() {
    // Configurer la tirette de depart
    // Tirette entre GPIO26 et GND:
    // - tirette en place/contact ouvert -> HIGH grace au pullup interne
    // - tirette retiree/contact ferme vers GND -> LOW
    pinMode(PIN_START_SWITCH, INPUT_PULLUP);

    // Configurer l'arrêt d'urgence (E-Stop)
    // INPUT_PULLUP signifie: pullup interne activé, LOW = pressé, HIGH = non-pressé
    pinMode(PIN_ESTOP, INPUT_PULLUP);
}

// ============================================================================
// LECTURE DES SIGNAUX DE SÉCURITÉ
// ============================================================================

/// Teste si la tirette de depart a ete activee
/// @return true si la tirette est retiree
bool SafetySystem::isStartPressed() const {
    return digitalRead(PIN_START_SWITCH) == LOW;  // Actif bas quand le contact GND se ferme
}

/// Teste si l'arrêt d'urgence a été activé
/// @return true si l'E-Stop est enfoncé (logique inversée: active bas)
bool SafetySystem::isEStopPressed() const {
    return digitalRead(PIN_ESTOP) == LOW;  // Actif bas (logique inversée)
}
