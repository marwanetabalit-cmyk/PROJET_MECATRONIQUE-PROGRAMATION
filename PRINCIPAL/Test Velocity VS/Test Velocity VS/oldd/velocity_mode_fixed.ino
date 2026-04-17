// ============================================================================
// TEST SIMPLIFIÉ DYNAMIXEL - Pour passer ton code
// ============================================================================
// À mettre dans: Test Velocity VS/Test Velocity VS/src/velocity_mode.ino

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial2
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 23;

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialise les ports séries
    DEBUG_SERIAL.begin(115200);
    delay(500);
    DXL_SERIAL.begin(57600, SERIAL_8N1, 16, 17);
    delay(500);

    DEBUG_SERIAL.println("\n╔════════════════════════════════════════╗");
    DEBUG_SERIAL.println("║  TEST DYNAMIXEL SIMPLIFIÉ             ║");
    DEBUG_SERIAL.println("╚════════════════════════════════════════╝\n");

    // Initialise Dynamixel
    DEBUG_SERIAL.println("[INIT] Démarrage communication Dynamixel...");
    dxl.begin(57600);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // ========================================================================
    // TEST 1 : Scan des IDs
    // ========================================================================
    DEBUG_SERIAL.println("\n[TEST 1] Scan des IDs Dynamixel");
    bool foundAny = false;
    for (uint8_t id = 1; id <= 20; id++) {
        DEBUG_SERIAL.print("  Test ID ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print("... ");
        if (dxl.ping(id)) {
            DEBUG_SERIAL.println("✓ TROUVÉ");
            foundAny = true;
        } else {
            DEBUG_SERIAL.println("✗");
        }
        delay(50);
    }
    if (!foundAny) {
        DEBUG_SERIAL.println("  Aucun Dynamixel trouvé entre ID 1 et 20.");
        DEBUG_SERIAL.println("  Si besoin, augmente la plage ou vérifie l'ID matériel.");
    }

    // ========================================================================
    // TEST 2 : Configuration des moteurs
    // ========================================================================
    DEBUG_SERIAL.println("\n[TEST 2] Configuration des moteurs");

    for (uint8_t id : {DXL_ID_1, DXL_ID_2}) {
        DEBUG_SERIAL.print("  Moteur ID ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(": ");

        // Désactiver le couple
        dxl.torqueOff(id);
        delay(50);

        // Passer en mode vitesse
        dxl.setOperatingMode(id, OP_VELOCITY);
        delay(50);

        // Réactiver le couple
        dxl.torqueOn(id);
        delay(50);

        DEBUG_SERIAL.println("✓ Configuré en mode VELOCITY");
    }

    DEBUG_SERIAL.println("\n[PRÊT] Tests commençant dans 2 secondes...");
    delay(2000);
}

// ============================================================================
// LOOP - Teste les 3 modes de vitesse
// ============================================================================

void loop() {
    // Test sur le moteur ID 1
    testMotor(DXL_ID_2);
    
    delay(3000); // Pause entre les cycles
}

// ============================================================================
// FONCTION DE TEST POUR UN MOTEUR
// ============================================================================

void testMotor(uint8_t motorID) {
    DEBUG_SERIAL.print("\n═══ MOTEUR ID ");
    DEBUG_SERIAL.print(motorID);
    DEBUG_SERIAL.println(" ═══\n");

    // ========================================================================
    // Test 1: Mode RAW (unité native Dynamixel)
    // ========================================================================
    DEBUG_SERIAL.println("[A] Mode RAW - setGoalVelocity(motorID, 200)");
    dxl.setGoalVelocity(motorID, 200);
    delay(500);

    int32_t vel_raw = dxl.getPresentVelocity(motorID);
    DEBUG_SERIAL.print("    → Présent Velocity (raw): ");
    DEBUG_SERIAL.println(vel_raw);

    if (vel_raw == 0) {
        DEBUG_SERIAL.println("    ⚠ PROBLÈME: Vitesse = 0!");
        DEBUG_SERIAL.println("    Vérifiez:");
        DEBUG_SERIAL.println("    - Le couple est-il activé?");
        DEBUG_SERIAL.println("    - Le mode OP_VELOCITY est-il bien appliqué?");
        DEBUG_SERIAL.println("    - L'alimentation 12V est-elle active?");
    }

    delay(1000);

    // ========================================================================
    // Test 2: Mode RPM (tours par minute)
    // ========================================================================
    DEBUG_SERIAL.println("\n[B] Mode RPM - setGoalVelocity(motorID, 30, UNIT_RPM)");
    dxl.setGoalVelocity(motorID, 30, UNIT_RPM);
    delay(500);

    float vel_rpm = dxl.getPresentVelocity(motorID, UNIT_RPM);
    DEBUG_SERIAL.print("    → Présent Velocity (RPM): ");
    DEBUG_SERIAL.println(vel_rpm, 2);

    if (vel_rpm == 0.0) {
        DEBUG_SERIAL.println("    ⚠ PROBLÈME: Vitesse RPM = 0!");
    }

    delay(1000);

    // ========================================================================
    // Test 3: Mode Pourcentage
    // ========================================================================
    DEBUG_SERIAL.println("\n[C] Mode Pourcentage - setGoalVelocity(motorID, 50, UNIT_PERCENT)");
    dxl.setGoalVelocity(motorID, 50, UNIT_PERCENT);
    delay(500);

    float vel_percent = dxl.getPresentVelocity(motorID, UNIT_PERCENT);
    DEBUG_SERIAL.print("    → Présent Velocity (%): ");
    DEBUG_SERIAL.println(vel_percent, 2);

    if (vel_percent == 0.0) {
        DEBUG_SERIAL.println("    ⚠ PROBLÈME: Vitesse % = 0!");
    }

    delay(1000);

    // ========================================================================
    // Test 4: Arrêt
    // ========================================================================
    DEBUG_SERIAL.println("\n[D] ARRÊT - setGoalVelocity(motorID, 0)");
    dxl.setGoalVelocity(motorID, 0);
    delay(500);

    vel_raw = dxl.getPresentVelocity(motorID);
    DEBUG_SERIAL.print("    → Présent Velocity: ");
    DEBUG_SERIAL.println(vel_raw);

    DEBUG_SERIAL.println();
}
