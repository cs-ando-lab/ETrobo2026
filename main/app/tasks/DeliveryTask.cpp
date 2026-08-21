#include "DeliveryTask.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"
#include <kernel.h>
#include <t_syslog.h>
#include <cmath>

namespace {
// Config::DELIVERY_TRACER_PWM(30)のままだと、Tracerのカーブ減速機能（basePwmに対する絶対量で減速し、
// basePwm×20%まで落ち込む）でほぼ動けなくなるため、接近フェーズだけこちらのローカル値を使う
constexpr int kApproachPwm = 40;
// 蛇行探索直後はラインに対してズレて乗っている分turnが大きくなりやすく、
// カーブ減速機能でkApproachPwmよりさらに底(basePwm×20%)に張り付きやすいため、こちらは高めにする
constexpr int kReacquireLinePwm = 50;
constexpr int kPostSlowTracePwm = 65;    // ステップ8以降の速度。Config::TRACER_PWM(80)は実機で試すと速すぎたため
constexpr int kOnFirstBlueLinePwm = 50;  // 仮実装（計測用）: 青1本目に乗っている間だけ落とす速度
constexpr int kStraightAfterFirstBluePwm = 40;         // 青1本目通過後、直進する際のパワー
constexpr int kStraightAfterFirstBlueUs = 500 * 1000;  // 青1本目通過後、直進する時間[us]

// 青ライン判定の確定時間。サンプル回数の固定値ではなくms基準にすることで、
// LINE_TRACE_POLL_INTERVAL_US（制御周期）が変わっても意図した時間幅を保つ
constexpr int kBlueEntryConfirmMs = 300;   // 青に乗ったと確定するまでの時間
constexpr int kBluePassedConfirmMs = 400;  // 青を通過した（完全に降りた）と確定するまでの時間

// 斜め移動のパワー差とIMU回転量。実機調整で左右90/30・80度が良さそうだったのでこれを基準値にする
constexpr int kDiagonalPwmHigh = 90;
constexpr int kDiagonalPwmLow = 30;

// エリア配置：左90・右0の斜め移動で80度回頭（安定/フォールバックどちらも同じ回転量にする）→ 前進180mm → 後退140mm → 右85度旋回
constexpr int kAreaDiagonalPwmRight = 0;
constexpr float kDiagonalTurnDeg = 80.0f;
constexpr int kAreaForwardMm = 180;
constexpr int kAreaBackwardMm = -140;
constexpr int kAreaBackwardSpeedDegPerSec = 700;  // 最速で後退させる（モーターの物理上限で自動的にクランプされる）
constexpr float kAreaTurnDeg = 85.0f;

// 青1本目通過後の直進の後に行う、左折の斜め移動の回転量（パワーはkDiagonalPwmHigh/Lowを左右逆にして流用）
constexpr float kPostFirstBlueDiagonalTurnDeg = 70.0f;

// 右エッジ復帰後、ライントレースが安定したかの判定用（要実測調整）
constexpr float kHeadingStabilityThresholdDeg = 1.0f;  // 直前サンプルとの差がこの角度未満なら安定とみなす
constexpr int kHeadingStabilityRequiredSamples = 10;    // 安定判定に必要な連続サンプル数

// 帰り: 蛇行(右優先)で黒/青の線を探す。色判定ではなく反射率のしきい値判定にすることで、
// ライントレースが境界を追う際の黒白判定のブレ(TRACER_TARGET_REFLECTIONとCOLOR_ACHROMATIC_REFLECTION_THRESHOLDが同値)を避ける
constexpr int kWaveReflectionThreshold = 55;
constexpr int kWavePwm = 35;
constexpr int kWaveNonWhiteReflectionThreshold = 90;  // 白の実測値(約99)より少し低め。診断用ビープのしきい値

// 蛇行でラインを見つけた瞬間に、左30・右90で0.2秒動かす
constexpr int kWaveFoundPwmLeft = 30;
constexpr int kWaveFoundPwmRight = 90;
constexpr float kWaveFoundSec = 0.2f;

// 帰り: ラインを見つけた後、一定時間ライントレースしながら角度をサンプリングし、
// ソートして左右の外側（黄色以外・青/赤で共通のロジック、時間は色ごとに変える）を除いてから平均する
// （黄色はこの区間をスキップし、蛇行後すぐ左エッジ再開する）
constexpr float kReturnHeadingAverageSecBlue = 1.0f;
constexpr float kReturnHeadingAverageSecRed = 1.5f;
constexpr float kReturnHeadingAverageSecMax = kReturnHeadingAverageSecRed;  // サンプル配列のサイズ確保用（大きい方に合わせる）
constexpr int kReturnHeadingAverageLoopCountMax = static_cast<int>(kReturnHeadingAverageSecMax * 1000 * 1000 / Config::LINE_TRACE_POLL_INTERVAL_US);
constexpr int kReturnHeadingTrimNumerator = 1;
constexpr int kReturnHeadingTrimDenominator = 6;  // 左側(値が小さい方)からサンプルの1/6を除外する
constexpr int kReturnHeadingRightTrimNumerator = 1;
constexpr int kReturnHeadingRightTrimDenominator = 11;  // 右側(値が大きい方)からサンプルの1/11を除外する（左の半分程度）

// 平均角度の方向(=線と平行)に直進する時間。色によって変える
constexpr float kReturnStraightSecBlue = 1.1f;
constexpr float kReturnStraightSecRed = 1.65f;
constexpr int kReturnStraightPwm = 100;

// 帰りの右折斜め移動（パワーはkDiagonalPwmHigh/Lowを流用）
constexpr float kReturnDiagonalTurnDeg = 70.0f;

// <algorithm>のstd::sortはRTOSのt_stddef.hと衝突してビルドできないため、自前の挿入ソートを使う。
// サンプル数は高々150程度なのでO(n^2)でも問題にならない。
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

DeliveryTask::DeliveryTask(Robot& robot)
    : robot(robot) {
}

void DeliveryTask::diagonalMoveUntilImuTurn(int leftPwm, int rightPwm, float startHeading, float turnDeg) {
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
// firstSwingRightで指定した側優先（半分優先側→逆→優先側→逆…）で、片輪停止のピボット旋回を繰り返す
void DeliveryTask::waveUntilReflectionBelow(int reflectionThreshold, float swingDeg, int pwm, bool firstSwingRight) {
    float baseHeading = robot.getImuHeading();
    float cumulativeTurnDeg = 0.0f;

    for(int swingCnt = 0; swingCnt < Config::RUC_SWING_MAX_COUNT; swingCnt++) {
        float swingTargetDeg = (swingCnt == 0) ? (swingDeg / 2.0f) : swingDeg;
        bool isRightTurn = firstSwingRight ? (swingCnt % 2 == 0) : (swingCnt % 2 != 0);
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
                int foundMoveLeftPwm = firstSwingRight ? kWaveFoundPwmLeft : kWaveFoundPwmRight;
                int foundMoveRightPwm = firstSwingRight ? kWaveFoundPwmRight : kWaveFoundPwmLeft;
                for(int i = 0; i < foundLoopCount; i++) {
                    if(robot.isCenterButtonPressed()) {
                        break;
                    }
                    robot.setMotorPower(foundMoveLeftPwm, foundMoveRightPwm);
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

void DeliveryTask::run() {
    syslog(LOG_NOTICE, "--- DeliveryTask Started ---");

    // このタスクの数値・エッジ・回頭方向はすべてLコースで実測調整したもの。Rコースはその鏡像になるので、
    // isLeftCourseがfalseのときは進行方向に関わる箇所（エッジ・旋回角度・斜め移動の左右パワー）を反転させる。
    // 前進・後退の距離やパワーは向きに依存しないため反転不要
    bool isLeftCourse = CourseConfig::isLeftCourse();
    float courseSign = isLeftCourse ? 1.0f : -1.0f;

    Tracer tracer(robot);
    tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
    tracer.setPwm(kApproachPwm);
    bool hasBeeped = false;

    // 1. ライントレースしながらボトルに近づく（この間、ラインに正対した基準角度をIMUで平均して求めておく）
    float headingSum = 0.0f;
    int headingSampleCount = 0;
    while(true) {
        int currentDistance = robot.getUltrasonicDistance();

        if(!hasBeeped && currentDistance > 0 && currentDistance < 150) {
            robot.beep(100);
            hasBeeped = true;
        }

        if(currentDistance > 0 && currentDistance <= Config::DELIVERY_TARGET_DISTANCE_MM) {
            tracer.terminate();
            break;
        }

        headingSum += robot.getImuHeading();
        headingSampleCount++;

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    float baselineHeading = (headingSampleCount > 0) ? (headingSum / headingSampleCount) : robot.getImuHeading();

    // 2. ボトルの前でアームを上げる（Robotクラスに移譲）
    robot.raiseArm();

    // センサーの値を安定させるための待機
    dly_tsk(500 * 1000);

    // 3. ボトルの色を判定
    ColorJudge::Color bottleColor = robot.getColor();
    int targetBlueLineCount = 0;  // 目標の青ライン通過回数

    // 判定結果に応じてビープ音を鳴らし、目標通過回数を設定
    switch(bottleColor) {
        case ColorJudge::Color::YELLOW:
            syslog(LOG_NOTICE, "Bottle Color: YELLOW");
            robot.beep(100);
            targetBlueLineCount = 2;
            break;

        case ColorJudge::Color::BLUE:
            syslog(LOG_NOTICE, "Bottle Color: BLUE");
            robot.beep(100);
            dly_tsk(100 * 1000);
            robot.beep(100);
            targetBlueLineCount = 3;
            break;

        case ColorJudge::Color::RED:
            syslog(LOG_NOTICE, "Bottle Color: RED");
            robot.beep(100);
            dly_tsk(100 * 1000);
            robot.beep(100);
            dly_tsk(100 * 1000);
            robot.beep(100);
            targetBlueLineCount = 4;
            break;

        default:
            syslog(LOG_NOTICE, "Bottle Color: UNKNOWN");
            robot.beep(500);
            targetBlueLineCount = 0;  // 不明な場合はとりあえず0にしておく
            break;
    }

    // 4. アームを下げる（Robotクラスに移譲）
    robot.lowerArm();

    // 5. 蛇行して黒い線を探す
    syslog(LOG_NOTICE, "Waving to find BLACK line");
    robot.runWavingUntilColor(ColorJudge::Color::BLACK, 200);

    // 6. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge (Slow Speed)");
    tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    tracer.setPwm(kReacquireLinePwm);    // 蛇行直後はズレが大きくカーブ減速で止まりやすいため、kApproachPwmより高めに

    // 7. 1秒間、遅い速度でライントレース
    // LINE_TRACE_POLL_INTERVAL_USを使って1秒間に必要なループ回数を計算
    int slowTraceLoopCount = (1000 * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    for(int i = 0; i < slowTraceLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            tracer.terminate();
            return;
        }
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    // 8. 1秒経過したら、通常速度に戻してライントレースを継続
    syslog(LOG_NOTICE, "1 second passed. Switching to TRACER_PWM.");
    tracer.setPwm(kPostSlowTracePwm);

    // 9. 指定回数青ラインを検知するまでライントレース
    syslog(LOG_NOTICE, "Tracing until blue line count: %d", targetBlueLineCount);

    int detectedBlueCount = 0;       // 青ラインを検知した回数
    bool isCurrentlyOnBlue = false;  // 現在青ライン上にいるかのフラグ

    int matchedBlueCount = 0;     // 青を連続で読んだ回数
    int matchedNonBlueCount = 0;  // 青以外を連続で読んだ回数
    ColorJudge::Color targetColor = ColorJudge::Color::BLUE;

    // ms指定の確定時間を、現在のLINE_TRACE_POLL_INTERVAL_US(制御周期)でのサンプル回数に換算
    int blueEntryConfirmCount = (kBlueEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int bluePassedConfirmCount = (kBluePassedConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    // 右エッジ復帰後の安定角度の測定用。安定を検出できなかった場合のフォールバックとして、
    // エリアに運ぶ直前の青ラインを読んだ瞬間の角度(lastBlueHeading)も保持しておく
    float lastBlueHeading = 0.0f;
    bool trackingHeadingStability = false;
    float prevHeadingForStability = 0.0f;
    int stableSampleCount = 0;
    bool headingStabilized = false;
    float stabilizedHeadingSum = 0.0f;
    int stabilizedHeadingSampleCount = 0;

    while(true) {
        if(robot.isCenterButtonPressed()) {
            break;
        }

        if(!isCurrentlyOnBlue) {
            // まだ青ラインに乗っていない状態：青を探す
            bool isOnBlueNow = robot.isOnColors(&targetColor, 1, matchedBlueCount, blueEntryConfirmCount);
            if(isOnBlueNow) {
                isCurrentlyOnBlue = true;
                matchedNonBlueCount = 0;  // 青以外カウントをリセット

                // 青を読んだ瞬間にカウントアップ！
                detectedBlueCount++;
                robot.beep(100);
                syslog(LOG_NOTICE, "Entered blue line. Count: %d / %d", detectedBlueCount, targetBlueLineCount);
                if(detectedBlueCount == 1) {
                    // 仮実装（計測用）: 青1本目の上に乗っている間（判定しなくなるまで）は速度を落とす
                    tracer.setPwm(kOnFirstBlueLinePwm);
                }
                // 指定回数に達したら、その場ですぐに終了する
                if(detectedBlueCount >= targetBlueLineCount) {
                    // 安定角度が取れなかった場合のフォールバック用に、エリアに運ぶ直前の青ラインを読んだ瞬間の角度を残しておく
                    lastBlueHeading = robot.getImuHeading();
                    syslog(LOG_NOTICE, "Target count reached! Stopping immediately.");
                    break;
                }
            }
        } else {
            // すでに青ラインに乗っている状態：青から降りる（青以外）のを探す
            ColorJudge::Color currentColor = robot.getColor();

            if(currentColor != ColorJudge::Color::BLUE) {
                matchedNonBlueCount++;
            } else {
                matchedNonBlueCount = 0;  // もし途中で青を読んだらリセット
            }

            // 青以外を一定時間連続で読んだら「完全にラインを通り過ぎた」と判定して次のラインを探せるようにする
            if(matchedNonBlueCount >= bluePassedConfirmCount) {
                isCurrentlyOnBlue = false;
                matchedBlueCount = 0;  // 次の青ラインを探すためにリセット

                syslog(LOG_NOTICE, "Passed blue line!");

                if(detectedBlueCount == 1) {
                    // 青1本目を通過し終えたら、ステップ1で求めた基準角度から左に90度回転した状態にしてから、最速で2.8秒直進する
                    float targetHeading = baselineHeading - 90.0f * courseSign;
                    robot.turnByImu(targetHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

                    constexpr int kStraightAfterPassPwm = 100;
                    constexpr float kStraightAfterPassSec = 2.3f;
                    int straightAfterPassLoopCount = static_cast<int>(kStraightAfterPassSec * 1000 * 1000 / Config::LINE_TRACE_POLL_INTERVAL_US);
                    for(int i = 0; i < straightAfterPassLoopCount; i++) {
                        if(robot.isCenterButtonPressed()) {
                            break;
                        }
                        robot.setMotorPower(kStraightAfterPassPwm, kStraightAfterPassPwm);
                        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
                    }
                    robot.stop();

                    // 左折の斜め移動（エリア配置のdiagonalMoveUntilImuTurnと同じ左右パワーを逆にして左折させる）
                    float postFirstBlueDiagonalStartHeading = robot.getImuHeading();
                    int postFirstBlueLeftPwm = isLeftCourse ? kDiagonalPwmLow : kDiagonalPwmHigh;
                    int postFirstBlueRightPwm = isLeftCourse ? kDiagonalPwmHigh : kDiagonalPwmLow;
                    diagonalMoveUntilImuTurn(postFirstBlueLeftPwm, postFirstBlueRightPwm, postFirstBlueDiagonalStartHeading, kPostFirstBlueDiagonalTurnDeg);

                    tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
                    tracer.setPwm(kPostSlowTracePwm);

                    // ここから右エッジでのライントレースが安定するまでの角度変化を追跡開始
                    trackingHeadingStability = true;
                    prevHeadingForStability = robot.getImuHeading();
                }
            }
        }

        if(trackingHeadingStability) {
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
        }

        // 青ライン上でも関係なく通常のライントレースを継続
        tracer.run();

        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "Reached target zone.");

    // 安定判定の成否に関わらず、平均値そのものを常にログへ出す（安定判定の妥当性を後で確認するため）
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

    // 10. エリアへの配置（斜め移動 → 前進180mm → 後退140mm → 右85度旋回）。色に関わらず共通
    syslog(LOG_NOTICE, "Diagonal move into area");
    int areaDiagonalLeftPwm = isLeftCourse ? kDiagonalPwmHigh : kAreaDiagonalPwmRight;
    int areaDiagonalRightPwm = isLeftCourse ? kAreaDiagonalPwmRight : kDiagonalPwmHigh;
    diagonalMoveUntilImuTurn(areaDiagonalLeftPwm, areaDiagonalRightPwm, diagonalStartHeading, kDiagonalTurnDeg);

    syslog(LOG_NOTICE, "Driving forward 180mm");
    robot.driveStraight(kAreaForwardMm, Config::DRIVE_DEFAULT_SPEED_DEG_PER_SEC);

    syslog(LOG_NOTICE, "Driving backward 140mm");
    robot.driveStraight(kAreaBackwardMm, kAreaBackwardSpeedDegPerSec);

    syslog(LOG_NOTICE, "Turning right 85 degrees");
    robot.turnByImu(kAreaTurnDeg * courseSign, Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 帰りの線探し（優先側はコース依存、反射率ベース）。色に関わらず共通。見つけた瞬間の0.2秒移動もこの中で行う
    syslog(LOG_NOTICE, "Waving to find line by reflection");
    waveUntilReflectionBelow(kWaveReflectionThreshold, Config::RUC_SWING_DEFAULT_DEG, kWavePwm, isLeftCourse);

    // 黄色だけは、ここから先の角度サンプリング→直進→右折斜め移動をスキップしてそのまま左エッジ再開する
    if(bottleColor != ColorJudge::Color::YELLOW) {
        // ラインを見つけた後、一定時間ライントレースしながら角度をサンプリングする。
        // ソートして片側(Lコースなら左側=値が小さい方)を多め、反対側を少なめに切り捨ててから残りを平均する（トリム平均）。
        // 青は短め(1.0秒)、赤は長め(1.5秒)の区間を使う
        tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
        tracer.setPwm(kPostSlowTracePwm);

        float headingAverageSec = (bottleColor == ColorJudge::Color::BLUE) ? kReturnHeadingAverageSecBlue : kReturnHeadingAverageSecRed;
        int headingAverageLoopCount = static_cast<int>(headingAverageSec * 1000 * 1000 / Config::LINE_TRACE_POLL_INTERVAL_US);

        float headingSamples[kReturnHeadingAverageLoopCountMax];
        int headingSampleCount = 0;

        for(int i = 0; i < headingAverageLoopCount; i++) {
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
        // トリム比率はLコースで実測した左偏りを補正するために非対称にしたもの。Rコースでは鏡像の偏りになるはずなので、
        // 大きく削る側(kReturnHeadingTrimNumerator/Denominator)と少なく削る側を左右入れ替える
        int leftTrimCount = (isLeftCourse ? headingSampleCount * kReturnHeadingTrimNumerator / kReturnHeadingTrimDenominator
                                           : headingSampleCount * kReturnHeadingRightTrimNumerator / kReturnHeadingRightTrimDenominator);
        int rightTrimCount = (isLeftCourse ? headingSampleCount * kReturnHeadingRightTrimNumerator / kReturnHeadingRightTrimDenominator
                                            : headingSampleCount * kReturnHeadingTrimNumerator / kReturnHeadingTrimDenominator);
        float headingAverageSum = 0.0f;
        int headingAverageSampleCount = 0;
        for(int i = leftTrimCount; i < headingSampleCount - rightTrimCount; i++) {
            headingAverageSum += headingSamples[i];
            headingAverageSampleCount++;
        }

        float returnDiagonalStartHeading = headingAverageSum / headingAverageSampleCount;
        robot.beep(50);  // 平均角度が取れた瞬間に短く鳴らす
        syslog(LOG_NOTICE, "Trimmed average heading (%d/%d samples used): %d deg", headingAverageSampleCount, headingSampleCount, (int)returnDiagonalStartHeading);

        // その平均角度の方向(=線と平行)に直進（青1.1秒、赤1.65秒）
        robot.turnByImu(returnDiagonalStartHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

        float straightSec = (bottleColor == ColorJudge::Color::BLUE) ? kReturnStraightSecBlue : kReturnStraightSecRed;
        int returnStraightLoopCount = static_cast<int>(straightSec * 1000 * 1000 / Config::LINE_TRACE_POLL_INTERVAL_US);
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
        int returnDiagonalLeftPwm = isLeftCourse ? kDiagonalPwmHigh : kDiagonalPwmLow;
        int returnDiagonalRightPwm = isLeftCourse ? kDiagonalPwmLow : kDiagonalPwmHigh;
        diagonalMoveUntilImuTurn(returnDiagonalLeftPwm, returnDiagonalRightPwm, returnDiagonalMoveStartHeading, kReturnDiagonalTurnDeg);
    }

    // 11. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge");
    tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    tracer.setPwm(Config::TRACER_PWM);  // センターボタンが押されるまでの無限ループなので、暗黙の値継承に頼らず明示する

    // 終了条件（ここではセンターボタンが押されるまで）
    while(!robot.isCenterButtonPressed()) {
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "--- DeliveryTask Finished ---");
}