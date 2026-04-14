#include "servos_ctrl.h"
#include "config.h"

void ServoController::init() {
    servoLift.setPeriodHertz(50);
    servoGrip.setPeriodHertz(50);
    servoSplit.setPeriodHertz(50);
    servoCursor.setPeriodHertz(50);

    servoLift.attach(SERVO_LIFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoGrip.attach(SERVO_GRIP_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoSplit.attach(SERVO_SPLIT_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoCursor.attach(SERVO_CURSOR_PIN, SERVO_MIN_US, SERVO_MAX_US);

    liftDown();
    gripOpen();
    splitOpen();
    cursorHome();
}

void ServoController::liftDown() {
    servoLift.write(LIFT_DOWN_ANGLE);
}

void ServoController::liftUp() {
    servoLift.write(LIFT_UP_ANGLE);
}

void ServoController::gripOpen() {
    servoGrip.write(GRIP_OPEN_ANGLE);
}

void ServoController::gripClose() {
    servoGrip.write(GRIP_CLOSE_ANGLE);
}

void ServoController::splitOpen() {
    servoSplit.write(SPLIT_OPEN_ANGLE);
}

void ServoController::splitClose() {
    servoSplit.write(SPLIT_CLOSE_ANGLE);
}

void ServoController::cursorHome() {
    servoCursor.write(CURSOR_HOME_ANGLE);
}

void ServoController::cursorPush() {
    servoCursor.write(CURSOR_PUSH_ANGLE);
}

void ServoController::demoSequence() {
    splitOpen();
    delay(500);

    gripOpen();
    delay(500);

    liftDown();
    delay(500);

    gripClose();
    delay(700);

    liftUp();
    delay(700);

    splitClose();
    delay(500);

    cursorPush();
    delay(600);

    cursorHome();
    delay(600);
}