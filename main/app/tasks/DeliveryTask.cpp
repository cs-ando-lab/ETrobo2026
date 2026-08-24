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
    constexpr int kPostSlowTracePwm = 65;                  // ステップ8以降の速度。Config::TRACER_PWM(80)は実機で試すと速すぎたため
    constexpr int kOnFirstBlueLinePwm = 50;                // 仮実装（計測用）: 青1本目に乗っている間だけ落とす速度
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

    // 右エッジ復帰後、ライントレースが安定したかの判定用（要実測調整）
    constexpr float kHeadingStabilityThresholdDeg = 1.0f;  // 直前サンプルとの差がこの角度未満なら安定とみなす
    constexpr int kHeadingStabilityRequiredSamples = 10;   // 安定判定に必要な連続サンプル数

    // 帰り: 蛇行(コース依存で優先側を変える)で黒/青の線を探す。色判定ではなく反射率のしきい値判定にすることで、
    // ライントレースが境界を追う際の黒白判定のブレ(TRACER_TARGET_REFLECTIONとCOLOR_ACHROMATIC_REFLECTION_THRESHOLDが同値)を避ける
    constexpr int kWaveReflectionThreshold = 55;
    constexpr int kWavePwm = 35;
    constexpr int kWaveNonWhiteReflectionThreshold = 90;  // 白の実測値(約99)より少し低め。診断用ビープのしきい値

    // 蛇行でラインを見つけた瞬間に、左30・右90で0.2秒動かす
    constexpr int kWaveFoundPwmLeft = 30;
    constexpr int kWaveFoundPwmRight = 90;
    constexpr float kWaveFoundSec = 0.2f;

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

    // 5. 蛇行して黒い線を探す（firstSwingRightにisLeftCourseを渡し、コースに応じて最初の振り方向を反転させる）
    syslog(LOG_NOTICE, "Waving to find BLACK line");
    robot.runWavingUntilColor(ColorJudge::Color::BLACK, 200, Config::COLOR_DETECTED_STABLE_COUNT, Config::RUC_SWING_DEFAULT_DEG, isLeftCourse);

    // 6. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge (Slow Speed)");
    tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    tracer.setPwm(kReacquireLinePwm);  // 蛇行直後はズレが大きくカーブ減速で止まりやすいため、kApproachPwmより高めに

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

    // 右エッジ復帰後の安定角度の測定用（ログ確認のみに使用。エリア配置の起点角度には採用しない）
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
                    // 青1本目を通過し終えたら、ステップ1で求めた基準角度から80度回転した状態にする
                    // （試走会で直進・左斜め移動が不要と判断したため削除。旋回後そのまま右エッジでライントレースを再開する）
                    float targetHeading = baselineHeading - 80.0f * courseSign;
                    robot.turnByImu(targetHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

                    tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
                    tracer.setPwm(kPostSlowTracePwm);

                    // ここから右エッジでのライントレースが安定するまでの角度変化を追跡開始（ログ確認用）
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

    // 安定判定の成否に関わらず、平均値そのものを常にログへ出す（採用はせず、判定の妥当性を後で確認するため）
    float averageStabilizedHeading = (stabilizedHeadingSampleCount > 0) ? (stabilizedHeadingSum / stabilizedHeadingSampleCount) : 0.0f;
    syslog(LOG_NOTICE, "Average stabilized heading (samples=%d): %d deg", stabilizedHeadingSampleCount, (int)averageStabilizedHeading);
    syslog(LOG_NOTICE, "Last-blue heading (unused): %d deg", (int)lastBlueHeading);

    // 10. エリアへの配置（斜め移動 → 前進180mm → 後退140mm → 右85度旋回）。色に関わらず共通
    // 安定角度・フォールバック角度は採用せず、この時点の現在角度を起点に80度斜め移動する
    syslog(LOG_NOTICE, "Diagonal move into area");
    int areaDiagonalLeftPwm = isLeftCourse ? kDiagonalPwmHigh : kAreaDiagonalPwmRight;
    int areaDiagonalRightPwm = isLeftCourse ? kAreaDiagonalPwmRight : kDiagonalPwmHigh;
    diagonalMoveUntilImuTurn(areaDiagonalLeftPwm, areaDiagonalRightPwm, robot.getImuHeading(), kDiagonalTurnDeg);

    syslog(LOG_NOTICE, "Driving forward 180mm");
    robot.driveStraight(kAreaForwardMm, Config::DRIVE_DEFAULT_SPEED_DEG_PER_SEC);

    syslog(LOG_NOTICE, "Driving backward 140mm");
    robot.driveStraight(kAreaBackwardMm, kAreaBackwardSpeedDegPerSec);

    syslog(LOG_NOTICE, "Turning right 85 degrees");
    robot.turnByImu(kAreaTurnDeg * courseSign, Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 帰りの線探し（優先側はコース依存、反射率ベース）。色に関わらず共通。見つけた瞬間の0.2秒移動もこの中で行う
    syslog(LOG_NOTICE, "Waving to find line by reflection");
    waveUntilReflectionBelow(kWaveReflectionThreshold, Config::RUC_SWING_DEFAULT_DEG, kWavePwm, isLeftCourse);

    // 11. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge");
    tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    tracer.setPwm(Config::TRACER_PWM);  // 暗黙の値継承に頼らず明示する

    // 終了条件（ボトル色によって、何本目の青(LAPゲート)で終了するかが変わる：黄=1本, 青=2本, 赤=3本）
    // 「乗った→完全に降りた」次を探す、の確認ロジックは9.の行きの青ライン検知と同じもの（blueEntryConfirmCount/bluePassedConfirmCountを流用）
    int finalTargetBlueLineCount = (bottleColor == ColorJudge::Color::BLUE)  ? 2
                                    : (bottleColor == ColorJudge::Color::RED) ? 3
                                                                               : 1;
    syslog(LOG_NOTICE, "Finishing after blue line count: %d", finalTargetBlueLineCount);

    int finalDetectedBlueCount = 0;
    bool isCurrentlyOnFinalBlue = false;
    int finalMatchedBlueCount = 0;
    int finalMatchedNonBlueCount = 0;

    while(!robot.isCenterButtonPressed()) {
        if(!isCurrentlyOnFinalBlue) {
            // まだ青ラインに乗っていない状態：青を探す
            bool isOnBlueNow = robot.isOnColors(&targetColor, 1, finalMatchedBlueCount, blueEntryConfirmCount);
            if(isOnBlueNow) {
                isCurrentlyOnFinalBlue = true;
                finalMatchedNonBlueCount = 0;  // 青以外カウントをリセット

                finalDetectedBlueCount++;
                robot.beep(100);
                syslog(LOG_NOTICE, "Entered blue line. Count: %d / %d", finalDetectedBlueCount, finalTargetBlueLineCount);

                if(finalDetectedBlueCount >= finalTargetBlueLineCount) {
                    syslog(LOG_NOTICE, "Target count reached! Finishing DeliveryTask.");
                    break;
                }
            }
        } else {
            // すでに青ラインに乗っている状態：青から降りる（青以外）のを探す
            ColorJudge::Color currentColor = robot.getColor();

            if(currentColor != ColorJudge::Color::BLUE) {
                finalMatchedNonBlueCount++;
            } else {
                finalMatchedNonBlueCount = 0;  // もし途中で青を読んだらリセット
            }

            // 青以外を一定時間連続で読んだら「完全にラインを通り過ぎた」と判定して次のラインを探せるようにする
            if(finalMatchedNonBlueCount >= bluePassedConfirmCount) {
                isCurrentlyOnFinalBlue = false;
                finalMatchedBlueCount = 0;  // 次の青ラインを探すためにリセット
                syslog(LOG_NOTICE, "Passed blue line!");
            }
        }

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "--- DeliveryTask Finished ---");
}
