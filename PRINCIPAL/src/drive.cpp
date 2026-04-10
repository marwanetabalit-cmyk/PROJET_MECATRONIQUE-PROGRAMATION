#include "drive.h"
#include "config.h"

DriveBase::DriveBase(HardwareSerial& serialPort)
    : serial(serialPort), dxl(serialPort, DXL_DIR_PIN) {}

float DriveBase::applyMotorSign(float rpm, bool inverted) {
    return inverted ? -rpm : rpm;
}

void DriveBase::configureMotor(uint8_t id) {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_VELOCITY);
    dxl.torqueOn(id);
}

void DriveBase::init() {
    serial.begin(DXL_BAUDRATE, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    configureMotor(DXL_LEFT_ID);
    configureMotor(DXL_RIGHT_ID);

    stop();
}

void DriveBase::setVelocity(float leftRpm, float rightRpm) {
    float leftCmd  = applyMotorSign(leftRpm, LEFT_MOTOR_INVERTED);
    float rightCmd = applyMotorSign(rightRpm, RIGHT_MOTOR_INVERTED);

    dxl.setGoalVelocity(DXL_LEFT_ID, leftCmd, UNIT_RPM);
    dxl.setGoalVelocity(DXL_RIGHT_ID, rightCmd, UNIT_RPM);
}

void DriveBase::stop() {
    setVelocity(0.0f, 0.0f);
}

void DriveBase::forward(float rpm) {
    setVelocity(rpm, rpm);
}

void DriveBase::backward(float rpm) {
    setVelocity(-rpm, -rpm);
}

void DriveBase::rotateLeft(float rpm) {
    setVelocity(-rpm, rpm);
}

void DriveBase::rotateRight(float rpm) {
    setVelocity(rpm, -rpm);
}