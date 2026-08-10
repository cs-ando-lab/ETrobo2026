#include "DeliveryTask.h"
#include "Tracer.h"
#include "Config.h"
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
constexpr float kDiagonalTurnDeg = 80.0f;          // 安定角度を基準にできた場合の回転量
constexpr float kFallbackDiagonalTurnDeg = 90.0f;  // 安定角度が取れず、直前の青検知角度を基準にする場合の回転量

// 右エッジ復帰後、ライントレースが安定したかの判定用（要実測調整）
constexpr float kHeadingStabilityThresholdDeg = 1.0f;  // 直前サンプルとの差がこの角度未満なら安定とみなす
constexpr int kHeadingStabilityRequiredSamples = 10;    // 安定判定に必要な連続サンプル数
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

void DeliveryTask::run() {
    syslog(LOG_NOTICE, "--- DeliveryTask Started ---");

    Tracer tracer(robot);
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
    tracer.setEdge(Tracer::Edge::LEFT);  // エッジを左に設定
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
                    float targetHeading = baselineHeading - 90.0f;
                    robot.turnByImu(targetHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

                    constexpr int kStraightAfterPassPwm = 100;
                    constexpr float kStraightAfterPassSec = 2.6f;
                    int straightAfterPassLoopCount = static_cast<int>(kStraightAfterPassSec * 1000 * 1000 / Config::LINE_TRACE_POLL_INTERVAL_US);
                    for(int i = 0; i < straightAfterPassLoopCount; i++) {
                        if(robot.isCenterButtonPressed()) {
                            break;
                        }
                        robot.setMotorPower(kStraightAfterPassPwm, kStraightAfterPassPwm);
                        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
                    }
                    robot.stop();

                    tracer.setEdge(Tracer::Edge::RIGHT);
                    tracer.setPwm(kPostSlowTracePwm);

                    // ここから右エッジでのライントレースが安定するまでの角度変化を追跡開始
                    trackingHeadingStability = true;
                    prevHeadingForStability = robot.getImuHeading();
                }
                if(detectedBlueCount == 2) {
                    // 青1本目と同様、ラインに乗っている最中ではなく完全に通過し終えてからエッジを切り替える
                    // （乗ったまま切り替えると急な進路変化で誤検知・二重カウントが起きやすいため）
                    tracer.setEdge(Tracer::Edge::LEFT);
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

    // 安定角度が取れていればそれを基準に80度、取れていなければエリアに運ぶ直前の青検知角度を基準に90度回す
    float diagonalStartHeading;
    float diagonalTurnDeg;
    if(headingStabilized) {
        diagonalStartHeading = stabilizedHeadingSum / stabilizedHeadingSampleCount;
        diagonalTurnDeg = kDiagonalTurnDeg;
        syslog(LOG_NOTICE, "Using stabilized heading: %d deg", (int)diagonalStartHeading);
    } else {
        diagonalStartHeading = lastBlueHeading;
        diagonalTurnDeg = kFallbackDiagonalTurnDeg;
        syslog(LOG_NOTICE, "Heading never stabilized. Falling back to last-blue heading: %d deg", (int)diagonalStartHeading);
    }

    // 10. エリアへの配置（斜め移動 → 150mm後退 → 回転）
    syslog(LOG_NOTICE, "Diagonal move into area");
    diagonalMoveUntilImuTurn(kDiagonalPwmHigh, kDiagonalPwmLow, diagonalStartHeading, diagonalTurnDeg);

    syslog(LOG_NOTICE, "Driving backward 150mm");
    robot.driveStraight(-130, Config::DRIVE_DEFAULT_SPEED_DEG_PER_SEC);

    syslog(LOG_NOTICE, "Turning right 90 degrees");
    robot.turnByImu(90.0f, Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 11. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge");
    tracer.setEdge(Tracer::Edge::LEFT);
    tracer.setPwm(Config::TRACER_PWM);  // センターボタンが押されるまでの無限ループなので、暗黙の値継承に頼らず明示する

    // 終了条件（ここではセンターボタンが押されるまで）
    while(!robot.isCenterButtonPressed()) {
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "--- DeliveryTask Finished ---");
}