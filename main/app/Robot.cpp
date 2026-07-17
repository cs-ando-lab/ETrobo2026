#include "Robot.h"
#include "kernel.h"
#include <cstdlib>
#include <t_syslog.h>  // タイムアウト時の警告ログ出力に使用

Robot::Robot()
    : leftMotor(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true),
      rightMotor(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true),
      armMotor(EPort::PORT_C, Motor::EDirection::CLOCKWISE, true),
      colorSensor(EPort::PORT_E),
      ultrasonicSensor(EPort::PORT_F),
      forceSensor(EPort::PORT_D),
      imu(),
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
        if(isCenterButtonPressed()) {  // センターボタンで安全停止
            break;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
        int count = (getLeftMotorCount() + getRightMotorCount()) / 2;
        traveledMm = (std::abs(count) / 360.0f) * 2 * Config::PI * Config::WHEEL_RADIUS_MM;
        loopCount++;
    }
    if(loopCount >= Config::DRIVE_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_NOTICE, "DRIVE,TIMEOUT");
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
        if(isCenterButtonPressed()) {  // センターボタンで安全停止
            break;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
        wheelDeg = (std::abs(getLeftMotorCount()) + std::abs(getRightMotorCount())) / 2.0f;
        loopCount++;
    }
    if(loopCount >= Config::TURN_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_NOTICE, "TURN,TIMEOUT");
    }

    stop();
}

void Robot::turnByImu(float degrees, int speedDegPerSec) {
    int loopCount = 0;
    // imuが使える状態かチェック
    while(!imu.isReady() && loopCount < Config::TURN_TIMEOUT_LOOP_COUNT) {
        if(isCenterButtonPressed()) {
            return;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
    }
    if(loopCount >= Config::TURN_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_ERROR, "TURN_IMU,READY_TIMEOUT");
        return;
    }

    imu.resetHeading();

    // 簡易的なP制御で回転速度を制御
    const float targetDeg = degrees;
    const int maxSpeed = std::abs(speedDegPerSec);

    loopCount = 0;
    while(loopCount < Config::TURN_TIMEOUT_LOOP_COUNT) {
        if(isCenterButtonPressed()) {
            break;
        }

        float errorDeg = targetDeg - imu.getHeading();
        float absErrorDeg = std::abs(errorDeg);
        if(absErrorDeg <= Config::TURN_IMU_STOP_TOLERANCE_DEG) {
            break;
        }

        int direction = (errorDeg >= 0.0f) ? 1 : -1;
        int turnSpeed = static_cast<int>(absErrorDeg * Config::TURN_IMU_KP);
        if(turnSpeed < Config::TURN_IMU_MIN_SPEED_DEG_PER_SEC) {
            turnSpeed = Config::TURN_IMU_MIN_SPEED_DEG_PER_SEC;
        }
        if(turnSpeed > maxSpeed) {
            turnSpeed = maxSpeed;
        }

        leftMotor.setSpeed(turnSpeed * direction);
        rightMotor.setSpeed(-turnSpeed * direction);

        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
    }
    if(loopCount >= Config::TURN_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_ERROR, "TURN_IMU,TIMEOUT");
    }

    stop();
}

void Robot::runStraightUntilColor(ColorJudge::Color color, int speedDegPerSec, int stableCount, bool forward) {
    runStraightUntilColors(&color, 1, speedDegPerSec, stableCount, forward);
}

void Robot::runStraightUntilColors(const ColorJudge::Color* colors, int colorCount, int speedDegPerSec, int stableCount, bool forward) {
    if(colors == nullptr || colorCount <= 0) {
        syslog(LOG_ERROR, "invalid colors or colorCount");
        return;
    }

    int direction;
    if(forward) {
        direction = 1;
    } else {
        direction = -1;
    }

    leftMotor.setSpeed(speedDegPerSec * direction);
    rightMotor.setSpeed(speedDegPerSec * direction);

    int loopCount = 0;
    int colorDetectedCount = 0;
    while(loopCount < Config::DRIVE_TIMEOUT_LOOP_COUNT) {
        if(isCenterButtonPressed() || isOnColors(colors, colorCount, colorDetectedCount, stableCount)) {  // センターボタンで安全停止
            break;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
        loopCount++;
    }
    if(loopCount >= Config::DRIVE_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_NOTICE, "STOP[runStraightUntilColor]: DRIVE,TIMEOUT");
    }

    stop();
}

void Robot::runWavingUntilColor(ColorJudge::Color color, int speedDegPerSec, int stableCount, float swingDeg) {
    runWavingUntilColors(&color, 1, speedDegPerSec, stableCount, swingDeg);
}

void Robot::runWavingUntilColors(const ColorJudge::Color* colors, int colorCount, int speedDegPerSec, int stableCount, float swingDeg) {
    if(colors == nullptr || colorCount <= 0) {
        syslog(LOG_ERROR, "invalid colors or colorCount");
        return;
    }
    // 0° < swingDeg <= 90°
    if(swingDeg <= 0) {
        syslog(LOG_ERROR, "ERROR[runWavingUntilColors]: invalid swingDeg");
        return;
    }
    if(swingDeg > 90) {
        swingDeg = 90;
    }

    // 一回の旋回あたりの角度 → 必要なホイール回転量[°]
    float targetWheelDeg = swingDeg * (Config::TREAD_MM / Config::WHEEL_RADIUS_MM);
    // 旋回回数のカウント
    int swingCnt = 0;
    int colorDetectedCount = 0;

    while(swingCnt < Config::RUC_SWING_MAX_COUNT) {
        // 一回の旋回あたりのループカウンタ
        int loopCount = 0;
        float wheelDeg = 0.0f;

        if(swingCnt == 0) {  // 最初の旋回は半分の旋回角度で左旋回
            float firstTargetWheelDeg = targetWheelDeg / 2.0f;

            resetMotorCounts();
            leftMotor.stop();
            rightMotor.setSpeed(speedDegPerSec);
            while(wheelDeg < firstTargetWheelDeg && loopCount < Config::RUC_SWING_TIMEOUT_LOOP_COUNT) {
                if(isCenterButtonPressed() || isOnColors(colors, colorCount, colorDetectedCount, stableCount)) {  // センターボタンもしくは停止条件で停止。
                    stop();
                    return;
                }
                dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
                wheelDeg = std::abs(getRightMotorCount());
                loopCount++;
            }
        } else if(swingCnt % 2 == 1) {  // 右旋回
            resetMotorCounts();
            leftMotor.setSpeed(speedDegPerSec);
            rightMotor.stop();
            while(wheelDeg < targetWheelDeg && loopCount < Config::RUC_SWING_TIMEOUT_LOOP_COUNT) {
                if(isCenterButtonPressed() || isOnColors(colors, colorCount, colorDetectedCount, stableCount)) {  // センターボタンもしくは停止条件で停止。
                    stop();
                    return;
                }
                dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
                wheelDeg = std::abs(getLeftMotorCount());
                loopCount++;
            }
        } else {  // 左旋回
            resetMotorCounts();
            leftMotor.stop();
            rightMotor.setSpeed(speedDegPerSec);
            while(wheelDeg < targetWheelDeg && loopCount < Config::RUC_SWING_TIMEOUT_LOOP_COUNT) {
                if(isCenterButtonPressed() || isOnColors(colors, colorCount, colorDetectedCount, stableCount)) {  // センターボタンもしくは停止条件で停止。
                    stop();
                    return;
                }
                dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
                wheelDeg = std::abs(getRightMotorCount());
                loopCount++;
            }
        }
        if(loopCount >= Config::RUC_SWING_TIMEOUT_LOOP_COUNT) {
            syslog(LOG_NOTICE, "STOP[runWavingUntilColor]: SWING,TIMEOUT");
        }
        swingCnt++;
    }

    stop();
}

void Robot::raiseArm() {
    armMotor.resetCount();
    armMotor.setSpeed(Config::ARM_RAISE_PWM);

    while(std::abs(armMotor.getCount()) < Config::ARM_RAISE_DEG) {
        if(isCenterButtonPressed())
            break;
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    armMotor.stop();
}

void Robot::lowerArm() {
    armMotor.resetCount();
    armMotor.setSpeed(Config::ARM_LOWER_PWM);

    while(std::abs(armMotor.getCount()) < Config::ARM_LOWER_DEG) {
        if(isCenterButtonPressed())
            break;
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    armMotor.stop();
}

bool Robot::isOnColor(ColorJudge::Color color, int& matchedCount, int stableCount) const {
    return isOnColors(&color, 1, matchedCount, stableCount);
}

bool Robot::isOnColors(const ColorJudge::Color* colors, int colorCount, int& matchedCount, int stableCount) const {
    if(colors == nullptr || colorCount < 1) {
        syslog(LOG_ERROR, "invalid colors or colorCount");
        matchedCount = 0;
        return false;
    }
    if(stableCount < 1) {
        stableCount = 1;
    }

    ColorJudge::Color detectedColor = getColor();
    bool matched = false;
    for(int i = 0; i < colorCount; i++) {
        if(detectedColor == colors[i]) {
            matched = true;
            break;
        }
    }

    if(matched) {
        matchedCount++;
    } else {
        matchedCount = 0;
    }

    return (matchedCount >= stableCount);
}

void Robot::setMotorPower(int left, int right) {
    leftMotor.setPower(left);
    rightMotor.setPower(right);
}

void Robot::stop() {
    leftMotor.stop();
    rightMotor.stop();
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
