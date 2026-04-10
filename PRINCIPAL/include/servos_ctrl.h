#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class ServoController {
public:
    void init();

    void liftDown();
    void liftUp();

    void gripOpen();
    void gripClose();

    void splitOpen();
    void splitClose();

    void cursorHome();
    void cursorPush();

    void demoSequence();

private:
    Servo servoLift;
    Servo servoGrip;
    Servo servoSplit;
    Servo servoCursor;
};