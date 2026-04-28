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

enum class TestMode {
    SAFETY_TEST,
    ULTRASONIC_TEST,
    SERVO_TEST,
    MOTOR_TEST,
    STRATEGY_TEST,
    COMPLETE_TEST
};

constexpr TestMode CURRENT_TEST = TestMode::COMPLETE_TEST;

SimInputs generateSimulationScenario();

static const char* stateToString(RobotState state) {
    switch (state) {
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

void testSafety() {
    Serial.print("[SAFETY] Start=");
    Serial.print(safety.isStartPressed() ? "ON" : "OFF");
    Serial.print(" | EStop=");
    Serial.println(safety.isEStopPressed() ? "ON" : "OFF");
}

void testUltrasonic() {
    distances = ultrasons.readAll();
    Serial.print("[ULTRASON] Front=");
    if (distances.front > 0.0f) {
        Serial.print(distances.front, 1);
        Serial.print("cm");
    } else {
        Serial.print("--");
    }
    Serial.print(" | Left=");
    if (distances.left > 0.0f) {
        Serial.print(distances.left, 1);
        Serial.print("cm");
    } else {
        Serial.print("--");
    }
    Serial.print(" | Right=");
    if (distances.right > 0.0f) {
        Serial.print(distances.right, 1);
        Serial.print("cm");
    } else {
        Serial.print("--");
    }
    Serial.print(" | Obstacle=");
    Serial.println(distances.obstacle ? "YES" : "NO");
}

void testServo() {
    Serial.println("[SERVO] Sequence complete");
    servos.splitOpen();
    delay(500);
    servos.gripOpen();
    delay(400);
    servos.liftDown();
    delay(600);
    servos.gripClose();
    delay(700);
    servos.liftUp();
    delay(700);
    servos.splitClose();
    delay(400);
    servos.cursorPush();
    delay(600);
    servos.cursorHome();
    delay(600);
    Serial.println("[SERVO] Termine");
}

void testMotor() {
    Serial.println("[MOTOR] Sequence complete");
    drive.forward(DRIVE_FORWARD_RPM);
    delay(2000);
    drive.rotateRight(DRIVE_TURN_RPM);
    delay(2000);
    drive.backward(DRIVE_FORWARD_RPM);
    delay(2000);
    drive.rotateLeft(DRIVE_TURN_RPM);
    delay(2000);
    drive.stop();
    Serial.println("[MOTOR] Termine");
}

void runCompleteStrategy() {
    unsigned long now = millis();

    if (now - lastSensorReadMs >= SENSOR_PERIOD_MS) {
        lastSensorReadMs = now;

        if (ROBOT_MODE == MODE_REAL) {
            distances = ultrasons.readAll();
            simInputs.startPressed = safety.isStartPressed();
            simInputs.eStopPressed = safety.isEStopPressed();
            simInputs.distances = distances;
        } else {
            simInputs = generateSimulationScenario();
            distances = simInputs.distances;
        }
    }

    if (ROBOT_MODE == MODE_REAL) {
        strategy.update(safety, distances, drive, servos, actions);
    } else {
        strategy.updateSimulation(simInputs, drive, servos, actions);
    }

    if (now - lastDebugMs >= DEBUG_PERIOD_MS) {
        lastDebugMs = now;

        Serial.print("[");
        Serial.print(now / 1000);
        Serial.print("s] Etat=");
        Serial.print(stateToString(strategy.getState()));
        Serial.print(" | Start=");
        Serial.print(simInputs.startPressed ? "ON" : "OFF");
        Serial.print(" | EStop=");
        Serial.print(simInputs.eStopPressed ? "ON" : "OFF");
        Serial.print(" | Front=");
        if (distances.front > 0.0f) {
            Serial.print(distances.front, 1);
            Serial.print("cm");
        } else {
            Serial.print("--");
        }
        Serial.print(" | Obstacle=");
        Serial.println(distances.obstacle ? "YES" : "NO");
    }
}

SimInputs generateSimulationScenario() {
    SimInputs sim;
    unsigned long t = millis();

    sim.startPressed = (t >= 3000);
    sim.eStopPressed = (t >= 30000);
    sim.distances.front = 100.0f;
    sim.distances.left = 100.0f;
    sim.distances.right = 100.0f;
    sim.distances.obstacle = false;

    if (t >= 9000 && t < 11000) {
        sim.distances.front = 10.0f;
        sim.distances.obstacle = true;
    }

    return sim;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("ROBOT EUROBOT 2026 - CODE PRINCIPAL");
    Serial.println("[SETUP] Initialisation...");

    safety.init();
    ultrasons.init();
    servos.init();
    actions.init();
    drive.init();
    strategy.init();

    Serial.println("[SETUP] Systeme pret");

    switch (CURRENT_TEST) {
        case TestMode::SAFETY_TEST:
            Serial.println("Mode: SAFETY_TEST");
            break;
        case TestMode::ULTRASONIC_TEST:
            Serial.println("Mode: ULTRASONIC_TEST");
            break;
        case TestMode::SERVO_TEST:
            Serial.println("Mode: SERVO_TEST");
            testServo();
            break;
        case TestMode::MOTOR_TEST:
            Serial.println("Mode: MOTOR_TEST");
            testMotor();
            break;
        case TestMode::STRATEGY_TEST:
            Serial.println("Mode: STRATEGY_TEST");
            break;
        case TestMode::COMPLETE_TEST:
            Serial.println("Mode: COMPLETE_TEST");
            break;
    }

    lastSensorReadMs = millis();
    lastDebugMs = millis();
}

void loop() {
    switch (CURRENT_TEST) {
        case TestMode::SAFETY_TEST:
            testSafety();
            delay(500);
            break;
        case TestMode::ULTRASONIC_TEST:
            testUltrasonic();
            delay(500);
            break;
        case TestMode::SERVO_TEST:
        case TestMode::MOTOR_TEST:
        case TestMode::STRATEGY_TEST:
            delay(1000);
            break;
        case TestMode::COMPLETE_TEST:
            runCompleteStrategy();
            delay(20);
            break;
    }
}