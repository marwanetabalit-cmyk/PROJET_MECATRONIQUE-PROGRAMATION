#pragma once
#include <Arduino.h>
#include <Dynamixel2Arduino.h>

struct DriveOdometryPose {
    float xCm = 0.0f;
    float yCm = 0.0f;
    float thetaRad = 0.0f;
    float signedDistanceCm = 0.0f;
    float totalDistanceCm = 0.0f;
    float leftTrackCm = 0.0f;
    float rightTrackCm = 0.0f;
};

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

    void updateOdometry();
    void resetOdometry(float xCm = 0.0f, float yCm = 0.0f, float thetaRad = 0.0f);
    const DriveOdometryPose& getPose() const;
    float getTravelDistanceCm() const;
    float getSignedDistanceCm() const;
    float getTurnAngleDeg() const;

private:
    HardwareSerial& serial;
    Dynamixel2Arduino dxl;
    DriveOdometryPose pose;
    float lastLeftDeg = 0.0f;
    float lastRightDeg = 0.0f;
    bool odometryReady = false;

    void configureMotor(uint8_t id);
    float applyMotorSign(float rpm, bool inverted);
    float readTrackPositionDeg(uint8_t id, bool inverted);
    float degreesToTrackCm(float deltaDeg) const;
};
