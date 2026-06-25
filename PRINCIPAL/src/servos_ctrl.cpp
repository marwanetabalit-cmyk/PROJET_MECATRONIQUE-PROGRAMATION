#include "servos_ctrl.h"
#include "config.h"

// ============================================================================
// INITIALISATION DES SERVOS
// ============================================================================

void ServoController::init() {
    // Configurer la fréquence PWM à 50 Hz (standard pour servos)
    servoLift.setPeriodHertz(50);    // Servo de levage
    servoGrip.setPeriodHertz(50);    // Servo de pince
    servoSplit.setPeriodHertz(50);   // Servo de séparateur
    servoCursor.setPeriodHertz(50);  // Servo du curseur

    // Attacher chaque servo à sa pin GPIO avec plage PWM configurée
    // Plage: 500µs (position min) à 2500µs (position max)
    servoLift.attach(SERVO_LIFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoGrip.attach(SERVO_GRIP_PIN, GRIP_SERVO_MIN_US, GRIP_SERVO_MAX_US);
    servoSplit.attach(SERVO_SPLIT_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoCursor.attach(SERVO_CURSOR_PIN, SERVO_MIN_US, SERVO_MAX_US);

    // Positionner tous les servos à leur position initiale
    liftDown();      // Bras baissé
    gripOpen();      // Pince ouverte
    splitOpen();     // Séparateur ouvert
    cursorHome();    // Curseur à domicile
}

// ============================================================================
// CONTRÔLE DU SERVO DE LEVAGE (Bras)
// ============================================================================

/// Baisser le bras (position fermée, prêt à saisir)
void ServoController::liftDown() {
    servoLift.write(LIFT_DOWN_ANGLE);   // Angle de baisse
}

/// Lever le bras (position haute, boîte soulevée)
void ServoController::liftUp() {
    servoLift.write(LIFT_UP_ANGLE);     // Angle de levée
}

// ============================================================================
// CONTRÔLE DU SERVO DE PINCE (Saisie)
// ============================================================================

/// Ouvrir la pince (relâcher la boîte)
void ServoController::gripOpen() {
    servoGrip.writeMicroseconds(GRIP_OPEN_US);
}

/// Fermer la pince (saisir la boîte)
void ServoController::gripClose() {
    servoGrip.writeMicroseconds(GRIP_CLOSE_US);
}

// ============================================================================
// CONTRÔLE DU SERVO DE SÉPARATEUR
// ============================================================================

/// Ouvrir le séparateur (écarter deux boîtes)
void ServoController::splitOpen() {
    servoSplit.write(SPLIT_OPEN_ANGLE);   // Angle écartement
}

/// Fermer le séparateur (rabattre)
void ServoController::splitClose() {
    servoSplit.write(SPLIT_CLOSE_ANGLE);  // Angle repli
}

// ============================================================================
// CONTRÔLE DU SERVO DU CURSEUR (Thermomètre)
// ============================================================================

/// Ramener le curseur à la position de repos
void ServoController::cursorHome() {
    servoCursor.write(CURSOR_HOME_ANGLE);  // Position neutre
}

/// Pousser le curseur thermomètre
void ServoController::cursorPush() {
    servoCursor.write(CURSOR_PUSH_ANGLE);  // Position poussée avant
}

// ============================================================================
// SÉQUENCE DE DÉMONSTRATION (TEST)
// ============================================================================

/// Exécute une démonstration complète de tous les servos
/// ATTENTION: Cette fonction est BLOQUANTE (utilise delay)
void ServoController::demoSequence() {
    Serial.println("[SERVO] Test pince commence");

    for (int cycle = 1; cycle <= 4; ++cycle) {
        Serial.print("[SERVO] Cycle pince ");
        Serial.print(cycle);
        Serial.println("/4 - ouverture");
        gripOpen();
        delay(1200);

        Serial.print("[SERVO] Cycle pince ");
        Serial.print(cycle);
        Serial.println("/4 - fermeture");
        gripClose();
        delay(1200);
    }

    Serial.println("[SERVO] Pince ouverte finale");
    gripOpen();
    delay(1200);

    Serial.println("[SERVO] Test pince termine");
}
