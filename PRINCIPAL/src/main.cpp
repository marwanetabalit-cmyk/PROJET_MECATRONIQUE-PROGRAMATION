#include <Arduino.h>
#include "config.h"
#include "safety.h"
#include "ultrasons.h"
#include "drive.h"
#include "servos_ctrl.h"
#include "strategy.h"

HardwareSerial DXLSerial(2);

SafetySystem safety;
UltrasonicArray ultrasons;
DriveBase drive(DXLSerial);
ServoController servos;
StrategyManager strategy;

DistanceReadings distances;
SimInputs simInputs;

unsigned long lastSensorReadMs = 0;
unsigned long lastDebugMs = 0;

static const char* stateToString(RobotState s) {
    switch (s) {
        case RobotState::WAIT_START:     return "WAIT_START";
        case RobotState::RUN_FORWARD:    return "RUN_FORWARD";
        case RobotState::AVOID_OBSTACLE: return "AVOID_OBSTACLE";
        case RobotState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case RobotState::END_MATCH:      return "END_MATCH";
        default:                         return "UNKNOWN";
    }
}

SimInputs generateSimulationScenario() {
    SimInputs sim;
    unsigned long t = millis();

    // Valeurs par défaut : aucun obstacle, pas de départ, pas d'arrêt d'urgence
    sim.startPressed = false;
    sim.eStopPressed = false;
    sim.distances.front = 100.0f;
    sim.distances.left  = 100.0f;
    sim.distances.right = 100.0f;
    sim.distances.obstacle = false;

    // 0 à 3 s : attente
    if (t >= 3000) {
        sim.startPressed = true;
    }

    // 8 à 12 s : obstacle en face
    if (t >= 8000 && t < 12000) {
        sim.distances.front = 10.0f;
        sim.distances.obstacle = true;
    }

    // 16 à 18 s : obstacle à gauche
    if (t >= 16000 && t < 18000) {
        sim.distances.left = 12.0f;
        sim.distances.obstacle = true;
    }

    // 25 s et plus : arrêt d'urgence
    if (t >= 25000) {
        sim.eStopPressed = true;
    }

    return sim;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== ROBOT PRINCIPAL - INITIALISATION ===");

    safety.init();
    ultrasons.init();
    servos.init();
    drive.init();
    strategy.init();

    if (ROBOT_MODE == MODE_SIMULATION) {
        Serial.println("Mode simulation actif.");
    } else {
        Serial.println("Mode reel actif.");
    }

    Serial.println("Initialisation terminee.");
}

void loop() {
    unsigned long now = millis();

    if (ROBOT_MODE == MODE_REAL) {
        if (now - lastSensorReadMs >= SENSOR_PERIOD_MS) {
            lastSensorReadMs = now;
            distances = ultrasons.readAll();
        }

        strategy.update(safety, distances, drive, servos);
    }
    else if (ROBOT_MODE == MODE_SIMULATION) {
        simInputs = generateSimulationScenario();
        strategy.updateSimulation(simInputs, drive, servos);
        distances = simInputs.distances;
    }

    if (now - lastDebugMs >= DEBUG_PERIOD_MS) {
        lastDebugMs = now;

        Serial.print("Etat=");
        Serial.print(stateToString(strategy.getState()));

        if (ROBOT_MODE == MODE_REAL) {
            Serial.print(" | Start=");
            Serial.print(safety.isStartPressed() ? "ON" : "OFF");

            Serial.print(" | EStop=");
            Serial.print(safety.isEStopPressed() ? "ON" : "OFF");
        } else {
            Serial.print(" | Start=");
            Serial.print(simInputs.startPressed ? "ON" : "OFF");

            Serial.print(" | EStop=");
            Serial.print(simInputs.eStopPressed ? "ON" : "OFF");
        }

        Serial.print(" | Front=");
        Serial.print(distances.front);

        Serial.print(" cm | Left=");
        Serial.print(distances.left);

        Serial.print(" cm | Right=");
        Serial.print(distances.right);

        Serial.print(" cm | Obstacle=");
        Serial.println(distances.obstacle ? "YES" : "NO");
    }
}