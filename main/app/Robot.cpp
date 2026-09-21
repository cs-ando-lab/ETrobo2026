#include "Robot.h"
#include "kernel.h"
#include <cstdlib>
#include <cmath>
#include <t_syslog.h>  // タイムアウト時の警告ログ出力に使用
extern "C" {
#include <pbio/control.h>  // アームの位置制御に使用（spikeapi::Motorに位置制御が無いため、pbioを直接呼ぶ）
}

namespace {
    // アームの上げの間だけモーターの加減速度を変えるので、元の設定を退避しておく
    struct ArmLimits {
        int32_t speed;
        int32_t acceleration;
        int32_t deceleration;
        int32_t actuation;
    };

    // 位置制御でアームを上げ始める（完了は待たない）。失敗したらfalseを返し、設定は元に戻っている。
    // 成功した場合は、加減速度の変更がこのモーターの以降の全動作に効くので、必ずfinishArmRaise()で戻すこと
    bool startArmRaise(const Motor& arm, pbio_servo_t*& srv, ArmLimits& saved, int targetDeg) {
        // pup_motor_get_device()は呼ぶたびに初期化し直すため使わず、初期化なしで取得できるこちらを使う
        srv = nullptr;
        if(pbio_servo_get_servo(PBIO_PORT_ID_C, &srv) != PBIO_SUCCESS || srv == nullptr) {
            return false;
        }

        arm.resetCount();
        pbio_control_settings_get_limits(&srv->control.settings, &saved.speed, &saved.acceleration, &saved.deceleration, &saved.actuation);
        if(pbio_control_settings_set_limits(&srv->control.settings, saved.speed, Config::ARM_RAISE_ACCELERATION_DEG_PER_SEC2, Config::ARM_RAISE_DECELERATION_DEG_PER_SEC2, saved.actuation) != PBIO_SUCCESS) {
            return false;
        }

        const int direction = (Config::ARM_RAISE_SPEED_DEG_PER_SEC >= 0) ? 1 : -1;
        if(pbio_servo_run_target(srv, std::abs(Config::ARM_RAISE_SPEED_DEG_PER_SEC), direction * targetDeg, PBIO_CONTROL_ON_COMPLETION_HOLD) != PBIO_SUCCESS) {
            pbio_control_settings_set_limits(&srv->control.settings, saved.speed, saved.acceleration, saved.deceleration, saved.actuation);
            return false;
        }
        return true;
    }

    // 上げ切るのを待ち、加減速度を元に戻す。止まった角度をログに出す
    void finishArmRaise(Robot& robot, const Motor& arm, pbio_servo_t* srv, const ArmLimits& saved, int targetDeg) {
        const int timeoutLoopCount = (Config::ARM_TIMEOUT_MS * 1000) / Config::MOTION_POLL_INTERVAL_US;
        bool reached = false;
        const char* exitReason = "timeout";
        for(int i = 0; i < timeoutLoopCount; i++) {
            if(pbio_control_is_done(&srv->control)) {
                reached = true;
                exitReason = "reached";
                break;
            }
            if(robot.isCenterButtonPressed()) {
                exitReason = "button";
                break;
            }
            dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        }
        if(!reached) {
            // タイムアウトやボタンで打ち切ったときも、自重で落ちないようその場で保持する
            arm.stop();
            arm.hold();
        }
        pbio_control_settings_set_limits(&srv->control.settings, saved.speed, saved.acceleration, saved.deceleration, saved.actuation);
        syslog(LOG_NOTICE, "Arm raise: settled at %d deg (target %d, %s)", std::abs(arm.getCount()), targetDeg, exitReason);
    }

    // setSpeed()で回し、残り角度に比例して減速しながら目標角で止める。
    // 下げと、位置制御が使えなかったときの上げで使う。holdAtEndがfalseなら止めた後は保持しない
    void moveArm(const Motor& arm, Robot& robot, int speedDegPerSec, int targetDeg, bool holdAtEnd, const char* label) {
        const int direction = (speedDegPerSec >= 0) ? 1 : -1;
        const int maxSpeed = std::abs(speedDegPerSec);
        const int timeoutLoopCount = (Config::ARM_TIMEOUT_MS * 1000) / Config::MOTION_POLL_INTERVAL_US;

        arm.resetCount();
        int commandedSpeed = 0;
        const char* exitReason = "timeout";
        for(int i = 0; i < timeoutLoopCount; i++) {
            if(robot.isCenterButtonPressed()) {
                exitReason = "button";
                break;
            }
            int remainingDeg = targetDeg - std::abs(arm.getCount());
            if(remainingDeg <= 0) {
                exitReason = "reached";
                break;
            }

            // 終点で低速になっていれば、止めた後の惰性による行き過ぎが小さい
            int speed = maxSpeed;
            if(remainingDeg < Config::ARM_DECEL_ANGLE_DEG) {
                speed = maxSpeed * remainingDeg / Config::ARM_DECEL_ANGLE_DEG;
                if(speed < Config::ARM_MIN_SPEED_DEG_PER_SEC) {
                    speed = Config::ARM_MIN_SPEED_DEG_PER_SEC;
                }
            }
            // 同じ値を毎周期指令し直すと軌道が作り直されるので、変わったときだけ送る
            if(speed != commandedSpeed) {
                arm.setSpeed(speed * direction);
                commandedSpeed = speed;
            }
            dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        }

        // ログ用に、止めると決めた瞬間の角度を取っておく（その後の惰性によるずれはgetArmCount()で見る）
        int stoppedDeg = std::abs(arm.getCount());
        if(holdAtEnd) {
            // 回している最中にhold()を呼ぶと、実際の位置より先（制御の目標位置）で保持してしまう（pbio servo.c）。
            // 先にstop()で制御を切ると、hold()は実際の位置で保持する
            arm.stop();
            arm.hold();
        } else {
            arm.stop();
        }
        syslog(LOG_NOTICE, "Arm %s: stopped at %d deg (target %d, %s)", label, stoppedDeg, targetDeg, exitReason);
    }
}  // namespace

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

int Robot::driveStraight(int distanceMm, int speedDegPerSec) {
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

    return static_cast<int>(traveledMm) * direction;
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

bool Robot::waitForImuReady() {
    int loopCount = 0;
    while(!imu.isReady() && loopCount < Config::TURN_TIMEOUT_LOOP_COUNT) {
        if(isCenterButtonPressed()) {
            return false;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
    }
    if(loopCount >= Config::TURN_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_ERROR, "TURN_IMU,READY_TIMEOUT");
        return false;
    }
    return true;
}

int Robot::driveStraightByImu(int distanceMm, float directionDeg, int speedDegPerSec) {
    if(distanceMm == 0 || !std::isfinite(directionDeg) || !waitForImuReady()) {
        return 0;
    }

    // distanceMmが負の場合は後退する
    int direction = (distanceMm >= 0) ? 1 : -1;
    int targetDistanceMm = std::abs(distanceMm);
    int maxSpeed = std::abs(speedDegPerSec);
    if(maxSpeed == 0) {
        return 0;
    }

    int initialLeftCounts = getLeftMotorCount();
    int initialRightCounts = getRightMotorCount();

    float traveledMm = 0.0f;
    int loopCount = 0;
    while(traveledMm < targetDistanceMm && loopCount < Config::DRIVE_TIMEOUT_LOOP_COUNT) {
        if(isCenterButtonPressed()) {  // センターボタンで安全停止
            break;
        }

        float remainingMm = targetDistanceMm - traveledMm;
        float speedRatio = 1.0f;
        if(Config::DRIVE_IMU_DECEL_DISTANCE_MM > 0.0f && remainingMm < Config::DRIVE_IMU_DECEL_DISTANCE_MM) {
            speedRatio = remainingMm / Config::DRIVE_IMU_DECEL_DISTANCE_MM;
        }

        int baseSpeed = static_cast<int>(maxSpeed * speedRatio);
        int minSpeed = (maxSpeed < Config::DRIVE_IMU_MIN_SPEED_DEG_PER_SEC)
                           ? maxSpeed
                           : Config::DRIVE_IMU_MIN_SPEED_DEG_PER_SEC;
        if(baseSpeed < minSpeed) {
            baseSpeed = minSpeed;
        }

        // headingは複数回転分を含むことがあるため、目標との差を最短方向の[-180, 180]°にする。
        float headingErrorDeg = std::fmod(directionDeg - imu.getHeading(), 360.0f);
        if(headingErrorDeg > 180.0f) {
            headingErrorDeg -= 360.0f;
        } else if(headingErrorDeg < -180.0f) {
            headingErrorDeg += 360.0f;
        }

        int correction = static_cast<int>(headingErrorDeg * Config::DRIVE_IMU_HEADING_KP);
        int maxCorrection = static_cast<int>(baseSpeed * Config::DRIVE_IMU_MAX_CORRECTION_RATIO);
        if(correction > maxCorrection) {
            correction = maxCorrection;
        } else if(correction < -maxCorrection) {
            correction = -maxCorrection;
        }

        // 補正の符号は車体固定。後退時も「左を速く、右を遅く」でheadingの正方向へ旋回する。
        int signedBaseSpeed = baseSpeed * direction;
        leftMotor.setSpeed(signedBaseSpeed + correction);
        rightMotor.setSpeed(signedBaseSpeed - correction);

        dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
        float averageCount = (std::abs(getLeftMotorCount() - initialLeftCounts) + std::abs(getRightMotorCount() - initialRightCounts)) / 2.0f;
        traveledMm = (averageCount / 360.0f) * 2 * Config::PI * Config::WHEEL_RADIUS_MM;
        loopCount++;
    }
    if(loopCount >= Config::DRIVE_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_NOTICE, "DRIVE_IMU,TIMEOUT");
    }

    brake();

    return static_cast<int>(traveledMm) * direction;
}

float Robot::turnByImu(float degrees, int speedDegPerSec) {
    if(!std::isfinite(degrees) || !waitForImuReady()) {
        return 0.0f;
    }

    degrees = std::fmod(degrees, 360.0f);
    if(degrees > 180.0f) {
        degrees -= 360.0f;
    } else if(degrees < -180.0f) {
        degrees += 360.0f;
    }

    float initialAngle = imu.getHeading();  // 最初の角度

    const float targetDeg = degrees;  // -180°～180°に正規化している
    const int maxSpeed = std::abs(speedDegPerSec);

    int loopCount = 0;
    while(loopCount < Config::TURN_TIMEOUT_LOOP_COUNT) {
        if(isCenterButtonPressed()) {
            break;
        }

        float errorDeg = initialAngle + targetDeg - imu.getHeading();  // 目標との差 = 最初の角度 ＋ 回転角度 － 現在の角度
        float absErrorDeg = std::abs(errorDeg);
        if(absErrorDeg <= Config::TURN_IMU_STOP_TOLERANCE_DEG) {
            break;
        }

        int direction = (errorDeg >= 0.0f) ? 1 : -1;
        int turnSpeed = static_cast<int>(absErrorDeg * Config::TURN_IMU_KP);  // 簡易的なP制御で回転速度を制御
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

    brake();

    return imu.getHeading() - initialAngle;
}

Robot::SearchResult Robot::turnByImuUntilUltrasonic(float degrees, int detectDistanceMm, int speedDegPerSec) {
    if(!waitForImuReady()) {
        return { false, 0.0f, 0.0f, getUltrasonicDistance() };
    }

    imu.resetHeading();

    const float targetDeg = degrees;
    const int maxSpeed = std::abs(speedDegPerSec);
    bool everDetected = false;
    float bestHeadingDeg = 0.0f;
    // detectDistanceMmを初期値(=未検知状態)にする。センサーが範囲外/読み取り失敗で負値を返すことがあるため、
    // 生の読み取り値をそのまま初期値にすると以降の正常な検知が「distance < bestDistanceMm」を満たせなくなる。
    int bestDistanceMm = detectDistanceMm;

    int loopCount = 0;
    while(loopCount < Config::TURN_ULTRASONIC_TIMEOUT_LOOP_COUNT) {
        if(isCenterButtonPressed()) {
            break;
        }

        int distance = getUltrasonicDistance();
        float currentHeading = imu.getHeading();

        if(distance > 0 && distance < detectDistanceMm) {
            everDetected = true;
            if(distance < bestDistanceMm) {
                bestDistanceMm = distance;
                bestHeadingDeg = currentHeading;
            }
        } else if(everDetected) {
            break;
        }
        if(loopCount % Config::TURN_ULTRASONIC_LOG_INTERVAL_LOOPS == 0) {  // 走行中の距離・角度の推移を診断ログに残す
            syslog(LOG_NOTICE, "TURN_ULTRASONIC,STEP,heading=%d,dist=%d", (int)currentHeading, distance);
        }

        float errorDeg = targetDeg - currentHeading;
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

        dly_tsk(Config::TURN_ULTRASONIC_POLL_INTERVAL_US);
        loopCount++;
    }
    if(loopCount >= Config::TURN_ULTRASONIC_TIMEOUT_LOOP_COUNT) {
        syslog(LOG_NOTICE, "TURN_ULTRASONIC,TIMEOUT");
    }

    stop();

    return { everDetected, imu.getHeading(), bestHeadingDeg, bestDistanceMm };
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

void Robot::runWavingUntilColor(ColorJudge::Color color, int speedDegPerSec, int stableCount, float swingDeg, bool firstSwingRight) {
    runWavingUntilColors(&color, 1, speedDegPerSec, stableCount, swingDeg, firstSwingRight);
}

void Robot::runWavingUntilColors(const ColorJudge::Color* colors, int colorCount, int speedDegPerSec, int stableCount, float swingDeg, bool firstSwingRight) {
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

        // swingCnt==0は半分の旋回角度、それ以降は左右交互(奇数=右, 偶数=左)。
        // firstSwingRightが立っていれば全体の左右を反転させる。
        float target = (swingCnt == 0) ? (targetWheelDeg / 2.0f) : targetWheelDeg;
        bool isLeftTurn = (swingCnt == 0) ? true : (swingCnt % 2 == 0);
        if(firstSwingRight) {
            isLeftTurn = !isLeftTurn;
        }

        resetMotorCounts();
        if(isLeftTurn) {
            leftMotor.stop();
            rightMotor.setSpeed(speedDegPerSec);
        } else {
            leftMotor.setSpeed(speedDegPerSec);
            rightMotor.stop();
        }
        while(wheelDeg < target && loopCount < Config::RUC_SWING_TIMEOUT_LOOP_COUNT) {
            if(isCenterButtonPressed() || isOnColors(colors, colorCount, colorDetectedCount, stableCount)) {  // センターボタンもしくは停止条件で停止。
                stop();
                return;
            }
            dly_tsk(Config::MOTION_POLL_INTERVAL_US); /* エンコーダーを確認する周期 */
            wheelDeg = std::abs(isLeftTurn ? getRightMotorCount() : getLeftMotorCount());
            loopCount++;
        }
        if(loopCount >= Config::RUC_SWING_TIMEOUT_LOOP_COUNT) {
            syslog(LOG_NOTICE, "STOP[runWavingUntilColor]: SWING,TIMEOUT");
        }
        swingCnt++;
    }

    stop();
}

bool Robot::positionDeliveryArm(int targetCount, int speedDegPerSec, const char* label) {
    pbio_servo_t* srv = nullptr;
    if(pbio_servo_get_servo(PBIO_PORT_ID_C, &srv) != PBIO_SUCCESS || !srv) return false;
    ArmLimits saved{};
    pbio_control_settings_get_limits(&srv->control.settings, &saved.speed,
                                    &saved.acceleration, &saved.deceleration, &saved.actuation);
    if(pbio_control_settings_set_limits(&srv->control.settings, saved.speed,
        Config::ARM_RAISE_ACCELERATION_DEG_PER_SEC2, Config::ARM_RAISE_DECELERATION_DEG_PER_SEC2,
        saved.actuation) != PBIO_SUCCESS) return false;
    const int startCount = armMotor.getCount();
    const auto err = pbio_servo_run_target(srv, std::abs(speedDegPerSec), targetCount,
                                         PBIO_CONTROL_ON_COMPLETION_HOLD);
    if(err != PBIO_SUCCESS) {
        pbio_control_settings_set_limits(&srv->control.settings, saved.speed,
                                        saved.acceleration, saved.deceleration, saved.actuation);
        return false;
    }
    struct Sample { int ms, angle, speed, power; };
    Sample samples[51]{};
    int count = 0, stable = 0, stalledRun = 0, peakPower = 0;
    bool success = false;
    const char* reason = "timeout";
    SYSTIM start, now;
    get_tim(&start);
    int lastSampleMs = -40;
    for(;;) {
        get_tim(&now);
        const int ms = static_cast<int>((now - start) / 1000);
        const int angle = armMotor.getCount(), speed = armMotor.getSpeed(), power = armMotor.getPower();
        if(std::abs(power) > peakPower) peakPower = std::abs(power);
        if(ms - lastSampleMs >= 40 && count < 51) {
            samples[count++] = { ms, angle, speed, power };
            lastSampleMs = ms;
        }
        if(isCenterButtonPressed()) { reason = "button"; break; }
        // is_doneは許容幅内を示すだけなので、位置誤差と低速が連続して成立することを確認。
        stable = std::abs(angle-targetCount) <= 2 && std::abs(speed) <= 20 ? stable+1 : 0;
        if(stable >= 5) { success = true; reason = "settled"; break; }
        stalledRun = armMotor.isStalled() && std::abs(angle-targetCount) > 2 ? stalledRun+1 : 0;
        if(stalledRun >= 5) { reason = "stalled"; break; }
        if(ms >= Config::ARM_TIMEOUT_MS) break;
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    if(!success) { armMotor.stop(); armMotor.hold(); }
    pbio_control_settings_set_limits(&srv->control.settings, saved.speed,
                                    saved.acceleration, saved.deceleration, saved.actuation);
    syslog(LOG_NOTICE,"[ArmMotion] %s %s start %d target %d end %d",
           label,reason,startCount,targetCount,armMotor.getCount());
    syslog(LOG_NOTICE,"[ArmMotion] peakAbsPower %d samples %d",peakPower,count);
    // 移動中にsyslogしない。4KiBスタック内に収まる固定最大51点。
    for(int i=0;i<count;++i)
        syslog(LOG_NOTICE,"[ArmSample] %s ms %d count %d speed %d power %d",
               label,samples[i].ms,samples[i].angle,samples[i].speed,samples[i].power);
    return success;
}

void Robot::raiseArm(int extraDeg) {
    // 上げた角度でボトルの色を読むので、止まる位置の精度が要る。setSpeed()で回して角度を見て止める方式は、
    // 10ms周期の止め遅れと惰性のぶん目標を越える。位置制御ならモーター側が減速を計画して目標角で止まり、そのまま保持する
    pbio_servo_t* srv = nullptr;
    ArmLimits saved{};
    const int targetDeg = Config::ARM_RAISE_DEG + extraDeg;
    if(!startArmRaise(armMotor, srv, saved, targetDeg)) {
        syslog(LOG_NOTICE, "Arm raise: position control unavailable, falling back to speed control");
        moveArm(armMotor, *this, Config::ARM_RAISE_SPEED_DEG_PER_SEC, targetDeg, true, "raise");
        return;
    }
    finishArmRaise(*this, armMotor, srv, saved, targetDeg);
}

void Robot::raiseArmWhileDriving(int distanceMm, int speedDegPerSec, int extraDeg) {
    pbio_servo_t* srv = nullptr;
    ArmLimits saved{};
    const int targetDeg = Config::ARM_RAISE_DEG + extraDeg;
    if(!startArmRaise(armMotor, srv, saved, targetDeg)) {
        // 位置制御が使えないと同時に動かせないので、進んでから上げる
        syslog(LOG_NOTICE, "Arm raise: position control unavailable, driving then raising");
        driveStraight(distanceMm, speedDegPerSec);
        moveArm(armMotor, *this, Config::ARM_RAISE_SPEED_DEG_PER_SEC, targetDeg, true, "raise");
        return;
    }
    // アームはモーター側の位置制御で上がっていくので、その間に走行できる
    driveStraight(distanceMm, speedDegPerSec);
    finishArmRaise(*this, armMotor, srv, saved, targetDeg);
}

void Robot::lowerArm(int extraDeg) {
    // 下げた後は保持しない（従来どおり。走行中にアームを保持し続けないため）
    moveArm(armMotor, *this, Config::ARM_LOWER_SPEED_DEG_PER_SEC, Config::ARM_LOWER_DEG + extraDeg, false, "lower");
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

void Robot::brake() {
    leftMotor.brake();
    rightMotor.brake();
    // 両モーターが完全に停止するまで待機。
    int stableCount = 0;

    while(stableCount < Config::BRAKE_STABLE_COUNT) {
        dly_tsk(10 * 1000);  // 10ms待つ

        if(std::abs(leftMotor.getSpeed()) <= Config::BRAKE_STOP_SPEED && std::abs(rightMotor.getSpeed()) <= Config::BRAKE_STOP_SPEED) {
            stableCount++;
        } else {
            stableCount = 0;
        }
    }
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

float Robot::getImuHeading() const {
    return imu.getHeading();
}

IMU::Acceleration Robot::getImuAcceleration() {
    IMU::Acceleration accel{};
    imu.getAcceleration(accel);
    return accel;
}

IMU::AngularVelocity Robot::getImuAngularVelocity() {
    IMU::AngularVelocity ang{};
    imu.getAngularVelocity(ang);
    return ang;
}

bool Robot::isImuStationary() const {
    return imu.isStationary();
}

bool Robot::isForceSensorPressed() const {
    return forceSensor.isTouched();
}

float Robot::getAngularVelocityZ() {
    IMU::AngularVelocity angv;
    imu.getAngularVelocity(angv);
    return -angv.z;
}

float Robot::getHeading() const {
    return imu.getHeading();
}

void Robot::resetHeading() {
    imu.resetHeading();
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

void Robot::showImage(const uint8_t image[25]) {
    // Display::setImageはuint8_t*を要求するため、渡された配列をコピーして使う
    uint8_t buffer[25];
    for(int i = 0; i < 25; i++) {
        buffer[i] = image[i];
    }
    display.setImage(buffer);
}

void Robot::off() {
    display.off();
}

void Robot::beep(int ms) {
    speaker.playTone(NOTE_A4, ms);
}

void Robot::startBeepNonBlocking() {
    // SOUND_MANUAL_STOPを渡すと待たずに戻る（spike-rtのhub_speaker_play_tone）
    speaker.playTone(NOTE_A4, SOUND_MANUAL_STOP);
}

void Robot::stopBeep() {
    speaker.stop();
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

int Robot::getArmCount() const {
    return armMotor.getCount();
}

int Robot::getArmPower() const {
    return armMotor.getPower();
}

bool Robot::isArmStalled() const {
    return armMotor.isStalled();
}
