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
unsigned long lastOdometryUpdateMs = 0;
unsigned long lastDebugMs = 0;

enum class TestMode {
    SAFETY_TEST,
    ULTRASONIC_TEST,
    SERVO_TEST,
    MOTOR_TEST,
    ODOMETRY_TEST,
    STRATEGY_TEST,
    COMPLETE_TEST,
    DAY_J_TEST
};

constexpr TestMode CURRENT_TEST = TestMode::DAY_J_TEST;

enum class OdometryTestState {
    WAIT_START,
    FORWARD_50_CM,
    PAUSE_AFTER_FORWARD,
    TURN_90_DEG,
    PAUSE_AFTER_TURN,
    BACKWARD_50_CM,
    DONE,
    EMERGENCY_STOP
};

constexpr float ODOMETRY_TEST_DISTANCE_CM = 50.0f;
constexpr float ODOMETRY_TEST_TURN_DEG = 90.0f;
constexpr float ODOMETRY_TEST_MOVE_RPM = 30.0f;
constexpr float ODOMETRY_TEST_TURN_RPM = 20.0f;
constexpr unsigned long ODOMETRY_TEST_PAUSE_MS = 1000;
constexpr unsigned long ODOMETRY_TEST_MOVE_TIMEOUT_MS = 15000;
constexpr unsigned long ODOMETRY_TEST_TURN_TIMEOUT_MS = 12000;

OdometryTestState odometryTestState = OdometryTestState::WAIT_START;
unsigned long odometryTestStateStartMs = 0;

enum class DayJState {
    WAIT_START,
    PREPARE_PUSH,
    PUSH_BOXES,
    PAUSE_AFTER_PUSH,
    BACK_UP,
    PAUSE_BEFORE_RETURN,
    RETURN_BASE,
    DONE,
    EMERGENCY_STOP
};

constexpr float DAY_J_PUSH_DISTANCE_CM = 65.0f;
constexpr float DAY_J_BACK_DISTANCE_CM = 25.0f;
constexpr float DAY_J_RETURN_DISTANCE_CM = 35.0f;
constexpr float DAY_J_PUSH_RPM = 35.0f;
constexpr float DAY_J_BACK_RPM = 30.0f;
constexpr float DAY_J_RETURN_RPM = 35.0f;
constexpr unsigned long DAY_J_PREPARE_MS = 1200;
constexpr unsigned long DAY_J_PAUSE_MS = 600;
constexpr unsigned long DAY_J_PUSH_MIN_MS = 6000;
constexpr unsigned long DAY_J_BACK_MIN_MS = 6000;
constexpr unsigned long DAY_J_PUSH_TIMEOUT_MS = 20000;
constexpr unsigned long DAY_J_BACK_TIMEOUT_MS = 16000;
constexpr unsigned long DAY_J_RETURN_TIMEOUT_MS = 10000;

DayJState dayJState = DayJState::WAIT_START;
unsigned long dayJStateStartMs = 0;
bool dayJObstacleBlocked = false;
unsigned long dayJObstacleBlockStartMs = 0;

SimInputs generateSimulationScenario();
void runOdometryTest();
void runDayJTest();
void updateOdometryPeriodic(unsigned long now);
void printOdometryStatus(const char* label);

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
    constexpr unsigned long gripMoveDelayMs = 2500;
    constexpr unsigned long splitMoveDelayMs = 1500;

    Serial.println("[SERVO] Test reel fendage, prise, depose");

    Serial.println("[SERVO] Preparation: pince ouverte, fendage au repos");
    servos.gripOpen();
    servos.splitOpen();
    delay(splitMoveDelayMs);

    Serial.println("[SERVO] Fendage des caisses");
    servos.splitClose();
    delay(splitMoveDelayMs);

    Serial.println("[SERVO] Remontee / retour du fendage");
    servos.splitOpen();
    delay(splitMoveDelayMs);

    Serial.println("[SERVO] Fermeture pince pour prise");
    servos.gripClose();
    delay(gripMoveDelayMs);

    Serial.println("[SERVO] Depose des caisses");
    servos.gripOpen();
    delay(gripMoveDelayMs);

    Serial.println("[SERVO] Test reel termine");
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

void updateOdometryPeriodic(unsigned long now) {
    if (now - lastOdometryUpdateMs >= ODOMETRY_PERIOD_MS) {
        lastOdometryUpdateMs = now;
        drive.updateOdometry();
    }
}

void printOdometryStatus(const char* label) {
    const auto& pose = drive.getPose();

    Serial.print("[ODO] ");
    Serial.print(label);
    Serial.print(" | Dist=");
    Serial.print(drive.getTravelDistanceCm(), 1);
    Serial.print("cm");
    Serial.print(" | Signed=");
    Serial.print(drive.getSignedDistanceCm(), 1);
    Serial.print("cm");
    Serial.print(" | Turn=");
    Serial.print(drive.getTurnAngleDeg(), 1);
    Serial.print("deg");
    Serial.print(" | X=");
    Serial.print(pose.xCm, 1);
    Serial.print(" Y=");
    Serial.print(pose.yCm, 1);
    Serial.print(" Th=");
    Serial.print(pose.thetaRad * 57.2957795f, 1);
    Serial.print("deg");
    Serial.print(" | L=");
    Serial.print(pose.leftTrackCm, 1);
    Serial.print(" R=");
    Serial.print(pose.rightTrackCm, 1);
    Serial.println();
}

void changeOdometryTestState(OdometryTestState newState, const char* label) {
    odometryTestState = newState;
    odometryTestStateStartMs = millis();
    drive.resetTravelCounters();
    printOdometryStatus(label);
}

void runOdometryTest() {
    unsigned long now = millis();
    updateOdometryPeriodic(now);

    if (safety.isEStopPressed()) {
        drive.stop();
        odometryTestState = OdometryTestState::EMERGENCY_STOP;
    }

    if (now - lastDebugMs >= DEBUG_PERIOD_MS) {
        lastDebugMs = now;

        switch (odometryTestState) {
            case OdometryTestState::WAIT_START:
                printOdometryStatus("WAIT_START");
                break;
            case OdometryTestState::FORWARD_50_CM:
                printOdometryStatus("FORWARD_50_CM");
                break;
            case OdometryTestState::PAUSE_AFTER_FORWARD:
                printOdometryStatus("PAUSE_AFTER_FORWARD");
                break;
            case OdometryTestState::TURN_90_DEG:
                printOdometryStatus("TURN_90_DEG");
                break;
            case OdometryTestState::PAUSE_AFTER_TURN:
                printOdometryStatus("PAUSE_AFTER_TURN");
                break;
            case OdometryTestState::BACKWARD_50_CM:
                printOdometryStatus("BACKWARD_50_CM");
                break;
            case OdometryTestState::DONE:
                printOdometryStatus("DONE");
                break;
            case OdometryTestState::EMERGENCY_STOP:
                printOdometryStatus("EMERGENCY_STOP");
                break;
        }
    }

    unsigned long elapsed = now - odometryTestStateStartMs;

    switch (odometryTestState) {
        case OdometryTestState::WAIT_START:
            drive.stop();
            if (safety.isStartPressed() || AUTO_START_WITHOUT_BUTTON) {
                drive.resetOdometry(0.0f, 0.0f, 0.0f);
                changeOdometryTestState(OdometryTestState::FORWARD_50_CM,
                                        "START -> FORWARD_50_CM");
            }
            break;

        case OdometryTestState::FORWARD_50_CM:
            if (drive.getTravelDistanceCm() >= ODOMETRY_TEST_DISTANCE_CM ||
                elapsed >= ODOMETRY_TEST_MOVE_TIMEOUT_MS) {
                drive.stop();
                changeOdometryTestState(OdometryTestState::PAUSE_AFTER_FORWARD,
                                        "FORWARD_DONE");
            } else {
                drive.forward(ODOMETRY_TEST_MOVE_RPM);
            }
            break;

        case OdometryTestState::PAUSE_AFTER_FORWARD:
            drive.stop();
            if (elapsed >= ODOMETRY_TEST_PAUSE_MS) {
                changeOdometryTestState(OdometryTestState::TURN_90_DEG,
                                        "PAUSE_DONE -> TURN_90_DEG");
            }
            break;

        case OdometryTestState::TURN_90_DEG:
            if (drive.getTurnAngleDeg() >= ODOMETRY_TEST_TURN_DEG ||
                elapsed >= ODOMETRY_TEST_TURN_TIMEOUT_MS) {
                drive.stop();
                changeOdometryTestState(OdometryTestState::PAUSE_AFTER_TURN,
                                        "TURN_DONE");
            } else {
                drive.rotateLeft(ODOMETRY_TEST_TURN_RPM);
            }
            break;

        case OdometryTestState::PAUSE_AFTER_TURN:
            drive.stop();
            if (elapsed >= ODOMETRY_TEST_PAUSE_MS) {
                changeOdometryTestState(OdometryTestState::BACKWARD_50_CM,
                                        "PAUSE_DONE -> BACKWARD_50_CM");
            }
            break;

        case OdometryTestState::BACKWARD_50_CM:
            if (drive.getTravelDistanceCm() >= ODOMETRY_TEST_DISTANCE_CM ||
                elapsed >= ODOMETRY_TEST_MOVE_TIMEOUT_MS) {
                drive.stop();
                changeOdometryTestState(OdometryTestState::DONE,
                                        "BACKWARD_DONE");
            } else {
                drive.backward(ODOMETRY_TEST_MOVE_RPM);
            }
            break;

        case OdometryTestState::DONE:
        case OdometryTestState::EMERGENCY_STOP:
            drive.stop();
            break;
    }
}

static const char* dayJStateName(DayJState state) {
    switch (state) {
        case DayJState::WAIT_START:         return "WAIT_START";
        case DayJState::PREPARE_PUSH:       return "PREPARE_PUSH";
        case DayJState::PUSH_BOXES:         return "PUSH_BOXES";
        case DayJState::PAUSE_AFTER_PUSH:   return "PAUSE_AFTER_PUSH";
        case DayJState::BACK_UP:            return "BACK_UP";
        case DayJState::PAUSE_BEFORE_RETURN:return "PAUSE_BEFORE_RETURN";
        case DayJState::RETURN_BASE:        return "RETURN_BASE";
        case DayJState::DONE:               return "DONE";
        case DayJState::EMERGENCY_STOP:     return "EMERGENCY_STOP";
        default:                            return "UNKNOWN";
    }
}

void changeDayJState(DayJState newState, const char* label) {
    dayJState = newState;
    dayJStateStartMs = millis();
    drive.resetTravelCounters();
    dayJObstacleBlocked = false;
    dayJObstacleBlockStartMs = 0;
    Serial.print("[DAYJ] ");
    Serial.println(label);
}

bool dayJStateUsesMotors(DayJState state) {
    return state == DayJState::PUSH_BOXES ||
           state == DayJState::BACK_UP ||
           state == DayJState::RETURN_BASE;
}

void runDayJTest() {
    unsigned long now = millis();
    updateOdometryPeriodic(now);

    if (now - lastSensorReadMs >= SENSOR_PERIOD_MS) {
        lastSensorReadMs = now;
        distances = ultrasons.readAll();
    }

    if (safety.isEStopPressed()) {
        drive.stop();
        dayJState = DayJState::EMERGENCY_STOP;
    }

    if (dayJStateUsesMotors(dayJState) && distances.obstacle) {
        drive.stop();

        if (!dayJObstacleBlocked) {
            dayJObstacleBlocked = true;
            dayJObstacleBlockStartMs = now;
            Serial.print("[DAYJ] Obstacle ultrason -> moteurs bloques");
            if (distances.front > 0.0f) {
                Serial.print(" | front=");
                Serial.print(distances.front, 1);
                Serial.print("cm");
            }
            Serial.println();
        }
        return;
    }

    if (dayJObstacleBlocked) {
        unsigned long blockedMs = now - dayJObstacleBlockStartMs;
        dayJStateStartMs += blockedMs;
        dayJObstacleBlocked = false;
        dayJObstacleBlockStartMs = 0;
        Serial.println("[DAYJ] Obstacle disparu -> reprise");
    }

    if (now - lastDebugMs >= DEBUG_PERIOD_MS) {
        lastDebugMs = now;
        Serial.print("[DAYJ] Etat=");
        Serial.print(dayJStateName(dayJState));
        Serial.print(" | Dist=");
        Serial.print(drive.getTravelDistanceCm(), 1);
        Serial.print("cm | Turn=");
        Serial.print(drive.getTurnAngleDeg(), 1);
        Serial.print("deg | Front=");
        if (distances.front > 0.0f) {
            Serial.print(distances.front, 1);
            Serial.print("cm");
        } else {
            Serial.print("--");
        }
        Serial.print(" | Obstacle=");
        Serial.print(distances.obstacle ? "YES" : "NO");
        Serial.println();
    }

    unsigned long elapsed = now - dayJStateStartMs;

    switch (dayJState) {
        case DayJState::WAIT_START:
            drive.stop();
            if (safety.isStartPressed() || AUTO_START_WITHOUT_BUTTON) {
                servos.liftDown();
                servos.gripClose();
                servos.splitOpen();
                drive.resetOdometry(0.0f, 0.0f, 0.0f);
                changeDayJState(DayJState::PREPARE_PUSH, "START -> pinces basses");
            }
            break;

        case DayJState::PREPARE_PUSH:
            drive.stop();
            servos.liftDown();
            servos.gripClose();
            if (elapsed >= DAY_J_PREPARE_MS) {
                changeDayJState(DayJState::PUSH_BOXES, "poussee des premieres caisses");
            }
            break;

        case DayJState::PUSH_BOXES:
            servos.liftDown();
            servos.gripClose();
            if ((elapsed >= DAY_J_PUSH_MIN_MS &&
                 drive.getTravelDistanceCm() >= DAY_J_PUSH_DISTANCE_CM) ||
                elapsed >= DAY_J_PUSH_TIMEOUT_MS) {
                drive.stop();
                changeDayJState(DayJState::PAUSE_AFTER_PUSH, "poussee terminee");
            } else {
                drive.forward(DAY_J_PUSH_RPM);
            }
            break;

        case DayJState::PAUSE_AFTER_PUSH:
            drive.stop();
            if (elapsed >= DAY_J_PAUSE_MS) {
                changeDayJState(DayJState::BACK_UP, "recul");
            }
            break;

        case DayJState::BACK_UP:
            servos.liftDown();
            servos.gripClose();
            if ((elapsed >= DAY_J_BACK_MIN_MS &&
                 drive.getTravelDistanceCm() >= DAY_J_BACK_DISTANCE_CM) ||
                elapsed >= DAY_J_BACK_TIMEOUT_MS) {
                drive.stop();
                changeDayJState(DayJState::PAUSE_BEFORE_RETURN, "recul termine");
            } else {
                drive.backward(DAY_J_BACK_RPM);
            }
            break;

        case DayJState::PAUSE_BEFORE_RETURN:
            drive.stop();
            if (elapsed >= DAY_J_PAUSE_MS) {
                servos.gripOpen();
                changeDayJState(DayJState::RETURN_BASE, "retour base");
            }
            break;

        case DayJState::RETURN_BASE:
            servos.liftDown();
            servos.gripOpen();
            if (drive.getTravelDistanceCm() >= DAY_J_RETURN_DISTANCE_CM ||
                elapsed >= DAY_J_RETURN_TIMEOUT_MS) {
                drive.stop();
                changeDayJState(DayJState::DONE, "sequence terminee");
            } else {
                drive.backward(DAY_J_RETURN_RPM);
            }
            break;

        case DayJState::DONE:
        case DayJState::EMERGENCY_STOP:
            drive.stop();
            break;
    }
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
        updateOdometryPeriodic(now);
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
        Serial.print(strategy.getStateName());
        Serial.print(" | Step=");
        Serial.print(strategy.getCurrentStep() + 1);
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
        Serial.print(distances.obstacle ? "YES" : "NO");
        if (ROBOT_MODE == MODE_REAL) {
            const auto& pose = drive.getPose();
            Serial.print(" | Odo=");
            Serial.print(drive.getTravelDistanceCm(), 1);
            Serial.print("cm");
            Serial.print(" | X=");
            Serial.print(pose.xCm, 1);
            Serial.print(" Y=");
            Serial.print(pose.yCm, 1);
            Serial.print(" Th=");
            Serial.print(pose.thetaRad * 57.2957795f, 1);
            Serial.print("deg");
        }
        Serial.println();
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
        case TestMode::ODOMETRY_TEST:
            Serial.println("Mode: ODOMETRY_TEST");
            Serial.println("[ODO] Sequence: forward 50cm, turn left 90deg, backward 50cm");
            Serial.println("[ODO] Garder le robot sur cales pour le premier essai.");
            odometryTestState = OdometryTestState::WAIT_START;
            odometryTestStateStartMs = millis();
            drive.resetOdometry(0.0f, 0.0f, 0.0f);
            break;
        case TestMode::STRATEGY_TEST:
            Serial.println("Mode: STRATEGY_TEST");
            break;
        case TestMode::COMPLETE_TEST:
            Serial.println("Mode: COMPLETE_TEST");
            break;
        case TestMode::DAY_J_TEST:
            Serial.println("Mode: DAY_J_TEST");
            Serial.println("[DAYJ] Sequence: pousser, reculer, retour base");
            dayJState = DayJState::WAIT_START;
            dayJStateStartMs = millis();
            drive.resetOdometry(0.0f, 0.0f, 0.0f);
            break;
    }

    lastSensorReadMs = millis();
    lastOdometryUpdateMs = millis();
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
        case TestMode::ODOMETRY_TEST:
            runOdometryTest();
            delay(20);
            break;
        case TestMode::COMPLETE_TEST:
            runCompleteStrategy();
            delay(20);
            break;
        case TestMode::DAY_J_TEST:
            runDayJTest();
            delay(20);
            break;
    }
}
