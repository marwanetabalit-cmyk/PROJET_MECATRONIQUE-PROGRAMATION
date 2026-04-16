#include <Arduino.h>
#include "config.h"
#include "safety.h"
#include "ultrasons.h"
#include "drive.h"
#include "servos_ctrl.h"
#include "strategy.h"
#include "actions.h"

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

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== ROBOT PRINCIPAL - INITIALISATION ===");

    safety.init();
    ultrasons.init();
    servos.init();
    actions.init();
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

        strategy.update(safety, distances, drive, servos, actions);
    }
    else if (ROBOT_MODE == MODE_SIMULATION) {
        simInputs = generateSimulationScenario();
        strategy.updateSimulation(simInputs, drive, servos, actions);
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