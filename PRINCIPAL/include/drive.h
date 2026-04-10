#pragma once
#include <Arduino.h>
#include <Dynamixel2Arduino.h>

class DriveBase {
public:
    explicit DriveBase(HardwareSerial& serialPort);

    void init();
    void stop();
    void forward(float rpm);
    void backward(float rpm);
    void rotateLeft(float rpm);
    void rotateRight(float rpm);
    void setVelocity(float leftRpm, float rightRpm);

private:
    HardwareSerial& serial;
    Dynamixel2Arduino dxl;

    void configureMotor(uint8_t id);
    float applyMotorSign(float rpm, bool inverted);
};