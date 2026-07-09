#include "Robot.h"

Robot::Robot()
    : leftMotor(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true),
      rightMotor(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true),
      driveBase(leftMotor, rightMotor),
      colorSensor(EPort::PORT_E),
      ultrasonicSensor(EPort::PORT_F),
      forceSensor(EPort::PORT_D),
      speaker(),
      display(),
      button() {
    colorSensor.lightOn();
    speaker.setVolume(50);
}

void Robot::driveStraight(int distanceMm, int speed) {
    // TODO: エンコーダーを使って指定距離で停止する
}

void Robot::turn(float degrees, int speed) {
    // TODO: エンコーダーを使って指定角度で旋回して停止する
}

void Robot::setMotorPower(int left, int right) {
    driveBase.setPower(left, right);
}

void Robot::stop() {
    driveBase.stop();
}

int Robot::getUltrasonicDistance() const {
    return ultrasonicSensor.getDistance();
}

int Robot::getReflection() const {
    return colorSensor.getReflection();
}

bool Robot::isOnBlue() const {
    ColorSensor::HSV hsv;
    colorSensor.getColor(hsv, true);
    return (hsv.h == BLUE_HUE);
}

bool Robot::isForceSensorPressed() const {
    return forceSensor.isTouched();
}

bool Robot::isLeftButtonPressed() {
    return button.isLeftPressed();
}

bool Robot::isRightButtonPressed() {
    return button.isRightPressed();
}

bool Robot::isCenterButtonPressed() {
    return button.isCenterPressed();
}

void Robot::showChar(char c) {
    display.showChar(c);
}

void Robot::off() {
    display.off();
}

void Robot::beep(int ms) {
    speaker.playTone(NOTE_A4, ms);
}

void Robot::resetMotorCounts() {
    leftMotor.resetCount();
    rightMotor.resetCount();
}

int Robot::getLeftMotorCount() const {
    return leftMotor.getCount();
}

int Robot::getRightMotorCount() const {
    return rightMotor.getCount();
}
