#include "drive.h"
#include "config.h"
#include <math.h>

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kTwoPi = 2.0f * kPi;
static constexpr uint8_t kDxlUnitDegree = static_cast<uint8_t>(UNIT_DEGREE);
static constexpr uint8_t kDxlUnitRpm = static_cast<uint8_t>(UNIT_RPM);

static float normalizeRadians(float angle) {
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

DriveBase::DriveBase(HardwareSerial& serialPort)
    : serial(serialPort), dxl(serialPort, DXL_DIR_PIN) {}

float DriveBase::applyMotorSign(float rpm, bool inverted) {
    return inverted ? -rpm : rpm;
}

void DriveBase::configureMotor(uint8_t id) {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, static_cast<uint8_t>(OP_VELOCITY));
    dxl.torqueOn(id);
}

float DriveBase::readTrackPositionDeg(uint8_t id, bool inverted) {
    float positionDeg = dxl.getPresentPosition(id, kDxlUnitDegree);
    return applyMotorSign(positionDeg, inverted);
}

float DriveBase::degreesToTrackCm(float deltaDeg) const {
    return (deltaDeg / 360.0f) * TRACK_CM_PER_REV;
}

void DriveBase::init() {
    serial.begin(DXL_BAUDRATE, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);

    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    configureMotor(DXL_LEFT_ID);
    configureMotor(DXL_RIGHT_ID);

    stop();
    resetOdometry();
}

void DriveBase::resetOdometry(float xCm, float yCm, float thetaRad) {
    pose.xCm = xCm;
    pose.yCm = yCm;
    pose.thetaRad = thetaRad;
    resetTravelCounters();
}

void DriveBase::resetTravelCounters() {
    pose.signedDistanceCm = 0.0f;
    pose.totalDistanceCm = 0.0f;
    pose.turnAngleRad = 0.0f;
    pose.leftTrackCm = 0.0f;
    pose.rightTrackCm = 0.0f;

    lastLeftDeg = readTrackPositionDeg(DXL_LEFT_ID, LEFT_MOTOR_INVERTED);
    lastRightDeg = readTrackPositionDeg(DXL_RIGHT_ID, RIGHT_MOTOR_INVERTED);
    odometryReady = true;
}

void DriveBase::updateOdometry() {
    float leftDeg = readTrackPositionDeg(DXL_LEFT_ID, LEFT_MOTOR_INVERTED);
    float rightDeg = readTrackPositionDeg(DXL_RIGHT_ID, RIGHT_MOTOR_INVERTED);

    if (!odometryReady) {
        lastLeftDeg = leftDeg;
        lastRightDeg = rightDeg;
        odometryReady = true;
        return;
    }

    float leftDeltaCm = degreesToTrackCm(leftDeg - lastLeftDeg);
    float rightDeltaCm = degreesToTrackCm(rightDeg - lastRightDeg);

    lastLeftDeg = leftDeg;
    lastRightDeg = rightDeg;

    float centerDeltaCm = (leftDeltaCm + rightDeltaCm) * 0.5f;
    float thetaDeltaRad = ((rightDeltaCm - leftDeltaCm) / TRACK_BASE_CM) * TRACK_TURN_CORRECTION;
    float midThetaRad = pose.thetaRad + (thetaDeltaRad * 0.5f);

    pose.xCm += centerDeltaCm * cosf(midThetaRad);
    pose.yCm += centerDeltaCm * sinf(midThetaRad);
    pose.thetaRad = normalizeRadians(pose.thetaRad + thetaDeltaRad);
    pose.signedDistanceCm += centerDeltaCm;
    pose.totalDistanceCm += fabsf(centerDeltaCm);
    pose.turnAngleRad += thetaDeltaRad;
    pose.leftTrackCm += leftDeltaCm;
    pose.rightTrackCm += rightDeltaCm;
}

const DriveOdometryPose& DriveBase::getPose() const {
    return pose;
}

float DriveBase::getTravelDistanceCm() const {
    return pose.totalDistanceCm;
}

float DriveBase::getSignedDistanceCm() const {
    return pose.signedDistanceCm;
}

float DriveBase::getTurnAngleDeg() const {
    return fabsf(pose.turnAngleRad) * 180.0f / kPi;
}

void DriveBase::setVelocity(float leftRpm, float rightRpm) {
    float leftCmd = applyMotorSign(leftRpm * LEFT_MOTOR_SPEED_SCALE, LEFT_MOTOR_INVERTED);
    float rightCmd = applyMotorSign(rightRpm * RIGHT_MOTOR_SPEED_SCALE, RIGHT_MOTOR_INVERTED);

    dxl.setGoalVelocity(DXL_LEFT_ID, leftCmd, kDxlUnitRpm);
    dxl.setGoalVelocity(DXL_RIGHT_ID, rightCmd, kDxlUnitRpm);
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
