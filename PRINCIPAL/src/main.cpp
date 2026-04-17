#include <Arduino.h>
#include "config.h"
#include "drive.h"

// ============================================================================
// DÉCLARATIONS GLOBALES
// ============================================================================

HardwareSerial DXLSerial(2);
DriveBase drive(DXLSerial);

unsigned long lastDebugMs = 0;

// ============================================================================
// SETUP - Initialisation
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ROBOT EUROBOT 2026 - CODE PRINCIPAL ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // Initialiser les moteurs
    Serial.println("[INIT] Initialisation des moteurs Dynamixel...");
    Serial.println("  Moteur 1 (gauche) - ID: 1");
    Serial.println("  Moteur 2 (droite) - ID: 2");
    Serial.println("  Baudrate: 57600 bps");
    Serial.println("  Protocol: 2.0\n");

    drive.init();

    Serial.println("[READY] Moteurs prêts!\n");
    Serial.println("Tests des mouvements dans 2 secondes...\n");

    delay(2000);
}

// ============================================================================
// LOOP - Boucle principale
// ============================================================================

void loop() {
    unsigned long now = millis();

    // ========================================================================
    // Cycle de test des mouvements (répète toutes les 40 secondes)
    // ========================================================================

    unsigned long cycle = (now / 40000) % 1;  // Cycle de 40 secondes
    unsigned long timeInCycle = now % 40000;

    // Phase 1 : Avancer (0-8s)
    if (timeInCycle < 8000) {
        drive.forward(DRIVE_FORWARD_RPM);

        if (now - lastDebugMs >= 500) {
            lastDebugMs = now;
            Serial.print("[");
            Serial.print(timeInCycle / 1000);
            Serial.println("s] AVANCER");
        }
    }
    // Phase 2 : Tourner droite (8-16s)
    else if (timeInCycle < 16000) {
        drive.rotateRight(DRIVE_TURN_RPM);

        if (now - lastDebugMs >= 500) {
            lastDebugMs = now;
            Serial.print("[");
            Serial.print(timeInCycle / 1000);
            Serial.println("s] TOURNER DROITE");
        }
    }
    // Phase 3 : Reculer (16-24s)
    else if (timeInCycle < 24000) {
        drive.backward(DRIVE_FORWARD_RPM);

        if (now - lastDebugMs >= 500) {
            lastDebugMs = now;
            Serial.print("[");
            Serial.print(timeInCycle / 1000);
            Serial.println("s] RECULER");
        }
    }
    // Phase 4 : Tourner gauche (24-32s)
    else if (timeInCycle < 32000) {
        drive.rotateLeft(DRIVE_TURN_RPM);

        if (now - lastDebugMs >= 500) {
            lastDebugMs = now;
            Serial.print("[");
            Serial.print(timeInCycle / 1000);
            Serial.println("s] TOURNER GAUCHE");
        }
    }
    // Phase 5 : Arrêt (32-40s)
    else {
        drive.stop();

        if (now - lastDebugMs >= 500) {
            lastDebugMs = now;
            Serial.print("[");
            Serial.print(timeInCycle / 1000);
            Serial.println("s] ARRÊT");
        }
    }

    delay(1);
}