#include "Robot.h"
#include "kernel.h"
#include <cstdlib>
#include <t_syslog.h>  // タイムアウト時の警告ログ出力に使用

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
    speaker.setVolume(Config::SPEAKER_VOLUME);
}

void Robot::driveStraight(int distanceMm, int speedDegPerSec) {
    // distanceMmが負の場合は後退する
    int direction = (distanceMm >= 0) ? 1 : -1;
    int targetDistanceMm = std::abs(distanceMm);

    resetMotorCounts();

    // setPower(パワー制御)は低出力域にデッドゾーンがあり左右差も出やすいため、
    // モーター内蔵のサーボ制御で速度を保ってくれるsetSpeedを使う
    leftMotor.setSpeed(speedDegPerSec * direction);
    rightMotor.setSpeed(speedDegPerSec * direction);

    float traveledMm = 0.0f;
    int loopCount = 0;
    while(traveledMm < targetDistanceMm && loopCount < Config::DRIVE_TIMEOUT_LOOP_COUNT) {
        dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
        int count = (getLeftMotorCount() + getRightMotorCount()) / 2;
        traveledMm = (std::abs(count) / 360.0f) * 2 * Config::PI * Config::WHEEL_RADIUS_MM;
        loopCount++;
    }

    stop();
}

void Robot::turn(float degrees, int speedDegPerSec) {
    // + = 右旋回（左を正転、右を逆転）、- = 左旋回（その逆）
    int direction = (degrees >= 0) ? 1 : -1;
    // 旋回角度 → 必要なホイール回転量[°]（このロボットではdegrees×2に一致する）
    float targetWheelDeg = std::abs(degrees) * (Config::TREAD_MM / (2.0f * Config::WHEEL_RADIUS_MM));

    resetMotorCounts();

    leftMotor.setSpeed(speedDegPerSec * direction);
    rightMotor.setSpeed(-speedDegPerSec * direction);

    int loopCount = 0;
    float wheelDeg = 0.0f;
    while(wheelDeg < targetWheelDeg && loopCount < Config::TURN_TIMEOUT_LOOP_COUNT) {
        dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
        wheelDeg = (std::abs(getLeftMotorCount()) + std::abs(getRightMotorCount())) / 2.0f;
        loopCount++;
    }
    if(loopCount >= Config::TURN_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_NOTICE, "TURN,TIMEOUT");
    }

    stop();
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

ColorJudge::Reading Robot::getColorReading() const {
    ColorJudge::Reading reading;
    colorSensor.getRGB(reading.rgb);
    colorSensor.getHSV(reading.hsv, true);
    reading.reflection = colorSensor.getReflection();
    return reading;
}

ColorJudge::Color Robot::getColor() const {
    return ColorJudge::judge(getColorReading());
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
