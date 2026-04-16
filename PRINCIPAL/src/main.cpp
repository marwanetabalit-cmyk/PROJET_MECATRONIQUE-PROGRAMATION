#include <Arduino.h>
#include "config.h"
#include "safety.h"
#include "ultrasons.h"
#include "drive.h"
#include "servos_ctrl.h"
#include "strategy.h"
#include "actions.h"

// ============================================================================
// DÉCLARATIONS GLOBALES
// ============================================================================

HardwareSerial DXLSerial(2);

SafetySystem safety;
UltrasonicArray ultrasons;
DriveBase drive(DXLSerial);
ServoController servos;
ActionManager actions;
StrategyManager strategy;

DistanceReadings distances;
SimInputs simInputs;

unsigned long lastSensorReadMs = 0;
unsigned long lastDebugMs = 0;

// ============================================================================
// MODE TEST: État de test actuel
// ============================================================================

enum class TestMode {
    SAFETY_TEST,        // Tester boutons start/estop
    ULTRASONIC_TEST,    // Tester les ultrasons
    SERVO_TEST,         // Tester les servos
    MOTOR_TEST,         // Tester les moteurs Dynamixel
    STRATEGY_TEST,      // Tester la stratégie complète
    COMPLETE_TEST       // Tester tout ensemble
};

// Mode de test actuel (à changer pour différents tests)
constexpr TestMode CURRENT_TEST = TestMode::COMPLETE_TEST;

static const char* stateToString(RobotState s) {
    switch (s) {
        case RobotState::WAIT_START:        return "WAIT_START";
        case RobotState::GO_TO_BOX_ZONE:    return "GO_TO_BOX_ZONE";
        case RobotState::PICK_BOX:          return "PICK_BOX";
        case RobotState::GO_TO_DROP_ZONE:   return "GO_TO_DROP_ZONE";
        case RobotState::DROP_BOX:          return "DROP_BOX";
        case RobotState::GO_TO_THERMOMETER: return "GO_TO_THERMOMETER";
        case RobotState::PUSH_CURSOR:       return "PUSH_CURSOR";
        case RobotState::RETURN_TO_NEST:    return "RETURN_TO_NEST";
        case RobotState::AVOID_OBSTACLE:    return "AVOID_OBSTACLE";
        case RobotState::EMERGENCY_STOP:    return "EMERGENCY_STOP";
        case RobotState::END_MATCH:         return "END_MATCH";
        default:                            return "UNKNOWN";
    }
}

// ============================================================================
// TESTS UNITAIRES
// ============================================================================

SimInputs generateSimulationScenario() {
    SimInputs sim;
    unsigned long t = millis();

    sim.startPressed = false;
    sim.eStopPressed = false;
    sim.distances.front = 100.0f;
    sim.distances.left  = 100.0f;
    sim.distances.right = 100.0f;
    sim.distances.obstacle = false;

    if (t >= 3000) {
        sim.startPressed = true;
    }

    if (t >= 9000 && t < 11000) {
        sim.distances.front = 10.0f;
        sim.distances.obstacle = true;
    }

    if (t >= 30000) {
        sim.eStopPressed = true;
    }

    return sim;
}

void test_safety() {
    Serial.println("\n=== TEST SÉCURITÉ ===");
    Serial.print("  Start button: ");
    Serial.println(safety.isStartPressed() ? "PRESSÉ" : "Libre");
    Serial.print("  E-Stop: ");
    Serial.println(safety.isEStopPressed() ? "PRESSÉ" : "Libre");
}

void test_ultrasonic() {
    Serial.println("\n=== TEST ULTRASONS ===");
    distances = ultrasons.readAll();
    Serial.print("  Front: ");
    Serial.print(distances.front);
    Serial.println(" cm");
    Serial.print("  Left: ");
    Serial.print(distances.left);
    Serial.println(" cm");
    Serial.print("  Right: ");
    Serial.print(distances.right);
    Serial.println(" cm");
    Serial.print("  Obstacle: ");
    Serial.println(distances.obstacle ? "OUI" : "NON");
}

void test_servo() {
    Serial.println("\n=== TEST SERVOS (séquence complète en ~6.5s) ===");
    
    Serial.println("  >> Ouverture séparateur");
    servos.splitOpen();
    delay(500);
    
    Serial.println("  >> Ouverture pince");
    servos.gripOpen();
    delay(400);
    
    Serial.println("  >> Baisse bras");
    servos.liftDown();
    delay(600);
    
    Serial.println("  >> Fermeture pince");
    servos.gripClose();
    delay(700);
    
    Serial.println("  >> Levée bras");
    servos.liftUp();
    delay(700);
    
    Serial.println("  >> Fermeture séparateur");
    servos.splitClose();
    delay(400);
    
    Serial.println("  >> Poussée curseur");
    servos.cursorPush();
    delay(600);
    
    Serial.println("  >> Retour curseur");
    servos.cursorHome();
    delay(600);
    
    Serial.println("  ✓ Séquence servos terminée");
}

void test_motor() {
    Serial.println("\n=== TEST MOTEURS (Dynamixel) ===");
    
    Serial.println("  >> Avancer (2 sec)");
    drive.forward(DRIVE_FORWARD_RPM);
    delay(2000);
    
    Serial.println("  >> Tourner droite (2 sec)");
    drive.rotateRight(DRIVE_TURN_RPM);
    delay(2000);
    
    Serial.println("  >> Reculer (2 sec)");
    drive.backward(DRIVE_FORWARD_RPM);
    delay(2000);
    
    Serial.println("  >> Tourner gauche (2 sec)");
    drive.rotateLeft(DRIVE_TURN_RPM);
    delay(2000);
    
    Serial.println("  >> Arrêt");
    drive.stop();
    
    Serial.println("  ✓ Test moteurs terminé");
}

void test_strategy() {
    Serial.println("\n=== TEST STRATÉGIE (mode simulation) ===");
    Serial.println("  En attente du signal de départ (simulé à t=3s)");
    Serial.println("  Obstacle simulé à t=9-11s");
    Serial.println("  E-Stop simulé à t=30s\n");
}

void test_complete() {
    unsigned long now = millis();

    // Lecture capteurs
    if (now - lastSensorReadMs >= SENSOR_PERIOD_MS) {
        lastSensorReadMs = now;
        simInputs = generateSimulationScenario();
        distances = simInputs.distances;
    }

    // Mise à jour stratégie
    strategy.updateSimulation(simInputs, drive, servos, actions);

    // Debug
    if (now - lastDebugMs >= DEBUG_PERIOD_MS) {
        lastDebugMs = now;

        Serial.print("[");
        Serial.print(now / 1000);
        Serial.print("s] État=");
        Serial.print(stateToString(strategy.getState()));

        Serial.print(" | Start=");
        Serial.print(simInputs.startPressed ? "ON" : "OFF");

        Serial.print(" | EStop=");
        Serial.print(simInputs.eStopPressed ? "ON" : "OFF");

        Serial.print(" | Front=");
        Serial.print(distances.front, 1);
        Serial.print("cm");

        Serial.print(" | Obstacle=");
        Serial.println(distances.obstacle ? "YES" : "NO");
    }

    delay(1);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ROBOT EUROBOT 2026 - MODE TEST      ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // Initialiser tous les systèmes
    Serial.println("[SETUP] Initialisation des systèmes...\n");

    Serial.print("  • Sécurité... ");
    safety.init();
    Serial.println("✓");

    Serial.print("  • Ultrasons... ");
    ultrasons.init();
    Serial.println("✓");

    Serial.print("  • Servos... ");
    servos.init();
    Serial.println("✓");

    Serial.print("  • Actions... ");
    actions.init();
    Serial.println("✓");

    Serial.print("  • Moteurs Dynamixel... ");
    drive.init();
    Serial.println("✓");

    Serial.print("  • Stratégie... ");
    strategy.init();
    Serial.println("✓");

    Serial.println("\n[SETUP] Tous les systèmes initialisés ✓\n");

    // Afficher le test qui va être exécuté
    Serial.println("═══════════════════════════════════════");
    switch (CURRENT_TEST) {
        case TestMode::SAFETY_TEST:
            Serial.println("Mode de test: SÉCURITÉ (boutons)");
            break;
        case TestMode::ULTRASONIC_TEST:
            Serial.println("Mode de test: ULTRASONS");
            break;
        case TestMode::SERVO_TEST:
            Serial.println("Mode de test: SERVOS");
            break;
        case TestMode::MOTOR_TEST:
            Serial.println("Mode de test: MOTEURS");
            break;
        case TestMode::STRATEGY_TEST:
            Serial.println("Mode de test: STRATÉGIE");
            break;
        case TestMode::COMPLETE_TEST:
            Serial.println("Mode de test: COMPLET (stratégie + composants)");
            break;
    }
    Serial.println("═══════════════════════════════════════\n");

    // Exécuter les tests unitaires (une seule fois)
    if (CURRENT_TEST == TestMode::SAFETY_TEST) {
        test_safety();
    }
    else if (CURRENT_TEST == TestMode::ULTRASONIC_TEST) {
        test_ultrasonic();
    }
    else if (CURRENT_TEST == TestMode::SERVO_TEST) {
        test_servo();
    }
    else if (CURRENT_TEST == TestMode::MOTOR_TEST) {
        test_motor();
    }
    else if (CURRENT_TEST == TestMode::STRATEGY_TEST) {
        test_strategy();
    }
    else if (CURRENT_TEST == TestMode::COMPLETE_TEST) {
        Serial.println("Lancement du test COMPLET...\n");
    }

    lastSensorReadMs = millis();
    lastDebugMs = millis();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    if (CURRENT_TEST == TestMode::SAFETY_TEST) {
        test_safety();
        delay(500);
    }
    else if (CURRENT_TEST == TestMode::ULTRASONIC_TEST) {
        test_ultrasonic();
        delay(500);
    }
    else if (CURRENT_TEST == TestMode::COMPLETE_TEST) {
        test_complete();
    }
    else {
        // Les autres tests s'exécutent en setup() et terminé
        delay(1000);
    }
}