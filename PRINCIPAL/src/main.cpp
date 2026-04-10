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

unsigned long lastSensorReadMs = 0;
unsigned long lastDebugMs = 0;

static const char* stateToString(RobotState s) {
    switch (s) {
        case RobotState::WAIT_START:     return "WAIT_START";
        case RobotState::RUN_FORWARD:    return "RUN_FORWARD";
        case RobotState::AVOID_OBSTACLE: return "AVOID_OBSTACLE";
        case RobotState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                         return "UNKNOWN";
    }
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

    Serial.println("Initialisation terminee.");
    Serial.println("Attente du switch de depart...");
}

void loop() {
    unsigned long now = millis();

    if (now - lastSensorReadMs >= SENSOR_PERIOD_MS) {
        lastSensorReadMs = now;
        distances = ultrasons.readAll();
    }

    strategy.update(safety, distances, drive, servos);

    if (now - lastDebugMs >= DEBUG_PERIOD_MS) {
        lastDebugMs = now;

        Serial.print("Etat=");
        Serial.print(stateToString(strategy.getState()));

        Serial.print(" | Start=");
        Serial.print(safety.isStartPressed() ? "ON" : "OFF");

        Serial.print(" | EStop=");
        Serial.print(safety.isEStopPressed() ? "ON" : "OFF");

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