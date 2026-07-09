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
    // getColor()は6色(赤/黄/緑/青/白/黒)の中から最も近いものを選び、その基準値を返す実装のため、
    // 僅差でも大差でも同じ値(青なら常にh=240,s=100,v=100)が返ってきて確信度が分からない。
    // ここではgetHSV()で丸め込み前の生の値を取得し、Hueの近さと彩度の高さの両方で判定する
    // （黒・白はほぼ無彩色=彩度が低いはずなので、境界付近のノイズでの誤判定を減らせる）。
    ColorSensor::HSV hsv;
    colorSensor.getHSV(hsv, true);
    int hueDiff = static_cast<int>(hsv.h) - static_cast<int>(BLUE_HUE);
    if(hueDiff < 0) {
        hueDiff = -hueDiff;
    }
    return (hsv.s >= BLUE_MIN_SATURATION && hueDiff <= BLUE_HUE_TOLERANCE);
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
