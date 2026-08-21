#include "test.h"
#include "Tracer.h"
#include "Config.h"
#include <kernel.h>
#include <t_syslog.h>
#include <cmath>

namespace {
constexpr int kTracePwm = 65;
constexpr int kBlueEntryConfirmMs = 300;

// エリア配置：左90・右0の斜め移動で80度回頭 → 前進180mm → 後退130mm
// （前進・後退の距離はchore/practice1ブランチのDeliveryTask.cppを参照）
constexpr int kDiagonalPwmLeft = 90;
constexpr int kDiagonalPwmRight = 0;
constexpr float kDiagonalTurnDeg = 80.0f;
constexpr int kForwardMm = 180;
constexpr int kBackwardMm = -140;
constexpr int kBackwardSpeedDegPerSec = 700;  // 最速で後退させる（モーターの物理上限で自動的にクランプされる）

constexpr float kHeadingStabilityThresholdDeg = 1.0f;
constexpr int kHeadingStabilityRequiredSamples = 30;

// 帰り: 蛇行(右優先)で黒線を探した後、一定時間の角度平均方向(=線と平行)に0.8秒直進 → 右折の斜め移動
// （斜め移動のパワーは本編DeliveryTask.cppの青1本目通過後の左折斜め移動と同じ90/30、左右を入れ替えて右折にする）
constexpr float kReturnHeadingAverageSec = 1.5f;  // 角度を時間平均する区間の長さ（振動1周期以上を想定）
constexpr int kReturnHeadingAverageLoopCount = static_cast<int>(kReturnHeadingAverageSec * 1000 * 1000 / Config::LINE_TRACE_POLL_INTERVAL_US);
constexpr int kReturnHeadingTrimNumerator = 1;
constexpr int kReturnHeadingTrimDenominator = 6;  // 左側(値が小さい方)からサンプルの2/11を除外する
constexpr int kReturnHeadingRightTrimNumerator = 1;
constexpr int kReturnHeadingRightTrimDenominator = 11;  // 右側(値が大きい方)からサンプルの1/11を除外する（左の半分程度）
constexpr float kReturnStraightSec = 1.65f;
constexpr int kReturnStraightPwm = 100;
constexpr int kReturnDiagonalPwmHigh = 90;
constexpr int kReturnDiagonalPwmLow = 30;
constexpr float kReturnDiagonalTurnDeg = 70.0f;

// 蛇行探索：反射率がこの値未満なら「線に乗った」と判定する（TRACER_TARGET_REFLECTIONの60より
// 低めに余裕を持たせ、黒白境界のブレを避ける）
constexpr int kWaveReflectionThreshold = 55;
constexpr int kWavePwm = 35;
constexpr int kWaveNonWhiteReflectionThreshold = 90;  // 白の実測値(約99)より少し低め。診断用ビープのしきい値

// 蛇行でラインを見つけた瞬間に、左60・右90で0.2秒動かす
constexpr int kWaveFoundPwmLeft = 30;
constexpr int kWaveFoundPwmRight = 90;
constexpr float kWaveFoundSec = 0.2f;
}  // namespace

Test::Test(Robot& robot)
    : robot(robot) {
}

void Test::run() {
    syslog(LOG_NOTICE, "--- Test Started ---");
    traceUntilBlueThenStraight();
}

namespace {
// <algorithm>のstd::sortはRTOSのt_stddef.hと衝突してビルドできないため、自前の挿入ソートを使う。
// サンプル数は高々100程度なのでO(n^2)でも問題にならない。
void insertionSort(float* values, int count) {
    for(int i = 1; i < count; i++) {
        float key = values[i];
        int j = i - 1;
        while(j >= 0 && values[j] > key) {
            values[j + 1] = values[j];
            j--;
        }
        values[j + 1] = key;
    }
}
}  // namespace

void Test::diagonalMoveUntilImuTurn(int leftPwm, int rightPwm, float startHeading, float turnDeg) {
    while(std::fabs(robot.getImuHeading() - startHeading) < turnDeg) {
        if(robot.isCenterButtonPressed()) {
            break;
        }
        robot.setMotorPower(leftPwm, rightPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    robot.stop();
}

// runWavingUntilColors相当を、色判定ではなく反射率のしきい値判定で自前実装したもの。
// 右優先（半分右→左→右→左…）で、片輪停止のピボット旋回を繰り返す
void Test::waveUntilReflectionBelow(int reflectionThreshold, float swingDeg, int pwm) {
    float baseHeading = robot.getImuHeading();
    float cumulativeTurnDeg = 0.0f;

    for(int swingCnt = 0; swingCnt < Config::RUC_SWING_MAX_COUNT; swingCnt++) {
        float swingTargetDeg = (swingCnt == 0) ? (swingDeg / 2.0f) : swingDeg;
        bool isRightTurn = (swingCnt % 2 == 0);  // 右優先: 0回目(半分)=右, 以降 左→右→左…
        float targetCumulative = cumulativeTurnDeg + (isRightTurn ? swingTargetDeg : -swingTargetDeg);

        int loopCount = 0;
        while(isRightTurn ? ((robot.getImuHeading() - baseHeading) < targetCumulative)
                           : ((robot.getImuHeading() - baseHeading) > targetCumulative)) {
            if(robot.isCenterButtonPressed()) {
                robot.stop();
                return;
            }
            int reflection = robot.getReflection();
            if(reflection < kWaveNonWhiteReflectionThreshold) {
                robot.beep(30);  // 診断用: 白以外を判定したら短く鳴らす（制御には影響しない）
            }
            if(reflection < reflectionThreshold) {
                robot.stop();
                syslog(LOG_NOTICE, "Line found by reflection: %d", reflection);

                int foundLoopCount = static_cast<int>(kWaveFoundSec * 1000 * 1000 / Config::MOTION_POLL_INTERVAL_US);
                for(int i = 0; i < foundLoopCount; i++) {
                    if(robot.isCenterButtonPressed()) {
                        break;
                    }
                    robot.setMotorPower(kWaveFoundPwmLeft, kWaveFoundPwmRight);
                    dly_tsk(Config::MOTION_POLL_INTERVAL_US);
                }
                robot.stop();
                return;
            }
            if(loopCount >= Config::RUC_SWING_TIMEOUT_LOOP_COUNT) {
                syslog(LOG_NOTICE, "STOP[waveUntilReflectionBelow]: SWING,TIMEOUT");
                break;
            }
            robot.setMotorPower(isRightTurn ? pwm : 0, isRightTurn ? 0 : pwm);
            dly_tsk(Config::MOTION_POLL_INTERVAL_US);
            loopCount++;
        }
        cumulativeTurnDeg = targetCumulative;
    }
    robot.stop();
}

// DeliveryTask.cppのステップ9後半〜11を切り出したテスト。
// 「青1本目を通過して右エッジに復帰した直後」から始まる想定で、
// 右エッジでライントレースしながら角度安定を監視 → 次の青ラインを検知したら即エリアに運ぶ、まで
void Test::traceUntilBlueThenStraight() {
    Tracer tracer(robot);
    tracer.setEdge(Tracer::Edge::RIGHT);
    tracer.setPwm(kTracePwm);

    bool isCurrentlyOnBlue = false;
    int matchedBlueCount = 0;
    ColorJudge::Color targetColor = ColorJudge::Color::BLUE;

    int blueEntryConfirmCount = (kBlueEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    float lastBlueHeading = 0.0f;
    float prevHeadingForStability = robot.getImuHeading();
    int stableSampleCount = 0;
    bool headingStabilized = false;
    float stabilizedHeadingSum = 0.0f;
    int stabilizedHeadingSampleCount = 0;

    while(true) {
        if(robot.isCenterButtonPressed()) {
            tracer.terminate();
            return;
        }

        if(!isCurrentlyOnBlue) {
            bool isOnBlueNow = robot.isOnColors(&targetColor, 1, matchedBlueCount, blueEntryConfirmCount);
            if(isOnBlueNow) {
                isCurrentlyOnBlue = true;
                robot.beep(100);
                lastBlueHeading = robot.getImuHeading();
                syslog(LOG_NOTICE, "Target count reached! Stopping immediately.");
                break;
            }
        }

        float currentHeadingForStability = robot.getImuHeading();
        float headingDelta = std::fabs(currentHeadingForStability - prevHeadingForStability);
        prevHeadingForStability = currentHeadingForStability;

        if(!headingStabilized) {
            if(headingDelta < kHeadingStabilityThresholdDeg) {
                stableSampleCount++;
                if(stableSampleCount >= kHeadingStabilityRequiredSamples) {
                    headingStabilized = true;
                    syslog(LOG_NOTICE, "Heading stabilized at %d deg", (int)currentHeadingForStability);
                }
            } else {
                stableSampleCount = 0;
            }
        }

        if(headingStabilized) {
            stabilizedHeadingSum += currentHeadingForStability;
            stabilizedHeadingSampleCount++;
        }

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "Reached target zone.");

    // 安定判定が取れたかどうかに関わらず、平均値そのものを常にログへ出す（安定判定の妥当性を後で確認するため）
    float averageStabilizedHeading = (stabilizedHeadingSampleCount > 0) ? (stabilizedHeadingSum / stabilizedHeadingSampleCount) : 0.0f;
    syslog(LOG_NOTICE, "Average stabilized heading (samples=%d): %d deg", stabilizedHeadingSampleCount, (int)averageStabilizedHeading);

    float diagonalStartHeading;
    if(headingStabilized) {
        diagonalStartHeading = averageStabilizedHeading;
        robot.beep(50);  // 安定角度を採用すると決めた瞬間に短く鳴らす
        syslog(LOG_NOTICE, "Using stabilized heading: %d deg", (int)diagonalStartHeading);
    } else {
        diagonalStartHeading = lastBlueHeading;
        syslog(LOG_NOTICE, "Heading never stabilized. Falling back to last-blue heading: %d deg", (int)diagonalStartHeading);
    }

    syslog(LOG_NOTICE, "Diagonal move into area");
    diagonalMoveUntilImuTurn(kDiagonalPwmLeft, kDiagonalPwmRight, diagonalStartHeading, kDiagonalTurnDeg);

    syslog(LOG_NOTICE, "Driving forward 180mm");
    robot.driveStraight(kForwardMm, Config::DRIVE_DEFAULT_SPEED_DEG_PER_SEC);

    syslog(LOG_NOTICE, "Driving backward 130mm");
    robot.driveStraight(kBackwardMm, kBackwardSpeedDegPerSec);

    syslog(LOG_NOTICE, "Turning right 95 degrees");
    robot.turnByImu(85.0f, Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 蛇行して黒/青どちらかの線を探す（右優先、反射率ベースで判定）
    syslog(LOG_NOTICE, "Waving to find line by reflection (right first)");
    waveUntilReflectionBelow(kWaveReflectionThreshold, Config::RUC_SWING_DEFAULT_DEG, kWavePwm);

    // ラインを見つけた後、一定時間ライントレースしながら角度をサンプリングする。
    // 単純な時間平均だと、ライントレースの左右振動が左右非対称な場合に平均が偏るため、
    // ソートして左側(値が小さい方)の1/4を切り捨ててから残りを平均する（トリム平均）
    tracer.setEdge(Tracer::Edge::LEFT);
    tracer.setPwm(kTracePwm);

    float headingSamples[kReturnHeadingAverageLoopCount];
    int headingSampleCount = 0;

    for(int i = 0; i < kReturnHeadingAverageLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            tracer.terminate();
            return;
        }

        headingSamples[headingSampleCount++] = robot.getImuHeading();

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    tracer.terminate();

    insertionSort(headingSamples, headingSampleCount);
    int leftTrimCount = headingSampleCount * kReturnHeadingTrimNumerator / kReturnHeadingTrimDenominator;
    int rightTrimCount = headingSampleCount * kReturnHeadingRightTrimNumerator / kReturnHeadingRightTrimDenominator;
    float headingAverageSum = 0.0f;
    int headingAverageSampleCount = 0;
    for(int i = leftTrimCount; i < headingSampleCount - rightTrimCount; i++) {
        headingAverageSum += headingSamples[i];
        headingAverageSampleCount++;
    }

    float returnDiagonalStartHeading = headingAverageSum / headingAverageSampleCount;
    robot.beep(50);  // 平均角度が取れた瞬間に短く鳴らす
    syslog(LOG_NOTICE, "Trimmed average heading (%d/%d samples used): %d deg", headingAverageSampleCount, headingSampleCount, (int)returnDiagonalStartHeading);

    // その平均角度の方向(=線と平行)に0.8秒直進
    robot.turnByImu(returnDiagonalStartHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    int returnStraightLoopCount = static_cast<int>(kReturnStraightSec * 1000 * 1000 / Config::LINE_TRACE_POLL_INTERVAL_US);
    for(int i = 0; i < returnStraightLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            break;
        }
        robot.setMotorPower(kReturnStraightPwm, kReturnStraightPwm);
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    robot.stop();

    // 右折の斜め移動（直前のturnByImuでIMU角度がリセットされているため、基準角度を取り直す）
    syslog(LOG_NOTICE, "Diagonal move (right turn)");
    float returnDiagonalMoveStartHeading = robot.getImuHeading();
    diagonalMoveUntilImuTurn(kReturnDiagonalPwmHigh, kReturnDiagonalPwmLow, returnDiagonalMoveStartHeading, kReturnDiagonalTurnDeg);

    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge");
    tracer.setEdge(Tracer::Edge::LEFT);
    tracer.setPwm(Config::TRACER_PWM);
    while(!robot.isCenterButtonPressed()) {
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    tracer.terminate();

    syslog(LOG_NOTICE, "--- Test Finished ---");
}
