#include "test.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"
#include <kernel.h>
#include <t_syslog.h>
#include <cmath>

namespace {
    // ── 青ライン判定 ──────────────────────────────────
    // 確定はサンプル数ではなくms基準にする。制御周期が変わっても意図した時間幅を保つため
    constexpr int kBlueEntryConfirmMs = 300;   // 青に乗ったと確定するまでの時間
    constexpr int kBluePassedConfirmMs = 400;  // 青を通過した（完全に降りた）と確定するまでの時間
    // エリアへ向かう最後の1本だけ短くする。300msだと確定までに約60mm進み、入口ではなく出口で抜けてしまうため
    constexpr int kBlueFinalEntryConfirmMs = 50;

    // ── エリア配置 ────────────────────────────────────
    constexpr int kDiagonalPwmHigh = 85;       // 斜め移動の外輪PWM
    // 内輪PWM。旋回半径 R = TREAD/2 * (外輪+内輪)/(外輪-内輪) なので、上げるほど弧が大きくなり
    // 同じ回頭量でも進む距離が伸びる。回頭量を3等分して段階的に切り替える
    constexpr int kAreaDiagonalInnerPwms[] = { 5, 10, 55 };
    constexpr float kDiagonalTurnDeg = 80.0f;  // 斜め移動での回頭量[度]
    // 斜め移動の出だし。いきなりkDiagonalPwmHighで動くとボトルが倒れかけるため、立ち上がりだけ段階的に上げる
    constexpr int kDiagonalRampPwms[] = { 50, 70 };
    constexpr int kDiagonalRampStageMs = 90;

    constexpr int kAreaBackwardMm = -60;                // ボトルから抜ける後退量
    constexpr int kAreaBackwardSpeedDegPerSec = 10000;  // 常に飽和させて最速で後退（pbio側でモーターの上限にクランプされる）
    // 車体が浮くのは速度ではなく立ち上がりのトルクが原因なので、必要ならデューティ上限でトルクの頭を押さえる（100で無効）
    constexpr int kAreaBackwardDutyLimit = 100;

    // 惰性を残したまま逆を指令すると初速が出ないため、後退前に止め切る
    constexpr int kSettleSpeedDegPerSec = 60;  // これ未満なら止まったとみなす
    constexpr int kSettleTimeoutMs = 400;
    // brake()は二択で全掛けだと車体が煽られるため、周期の一部だけ掛けて平均の制動力を落とす
    constexpr int kSettleBrakeCycleLoops = 3;  // 制動の1周期（30ms）
    constexpr int kSettleBrakeOnLoops = 1;     // そのうちブレーキを掛ける回数。0で完全なcoast、周期数と同じで全掛け

    constexpr float kAreaTurnDeg = 70.0f;  // 後退後、帰りの線を探す向きへの旋回量
    constexpr int kAreaTurnPwm = 70;       // 両輪逆転のその場旋回PWM（ボトル配置済みなのでジャークは気にしない）

    // ── 帰りの線探し ──────────────────────────────────
    // 色判定ではなく反射率で判定する（TRACER_TARGET_REFLECTIONとCOLOR_ACHROMATIC_REFLECTION_THRESHOLDが
    // 同値のため、ライントレース中の白黒判定が最も不安定になるため）
    constexpr int kSearchReflectionThreshold = 55;
    constexpr int kSearchPwm = 65;
    constexpr float kSearchMaxTurnDeg = 180.0f;   // これ以上回っても線から離れるだけなので打ち切る
    constexpr int kSearchTimeoutLoopCount = 500;  // 保険（10ms周期なので5秒）
    constexpr int kSearchFoundPwmLeft = 30;       // 線を見つけた瞬間に踏み込む左右PWM
    constexpr int kSearchFoundPwmRight = 90;
    constexpr float kSearchFoundSec = 0.10f;

}  // namespace

Test::Test(Robot& robot)
    : robot(robot) {
}

void Test::diagonalMoveUntilImuTurn(bool isOuterLeft, int outerPwm, float startHeading, float turnDeg) {
    const int rampStageLoopCount = (kDiagonalRampStageMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
    const int rampStageCount = static_cast<int>(sizeof(kDiagonalRampPwms) / sizeof(kDiagonalRampPwms[0]));
    const int innerStageCount = static_cast<int>(sizeof(kAreaDiagonalInnerPwms) / sizeof(kAreaDiagonalInnerPwms[0]));

    int loopCount = 0;
    float turnedDeg = 0.0f;
    while(turnedDeg < turnDeg) {
        if(robot.isCenterButtonPressed()) {
            break;
        }

        // 回頭がどこまで進んだかで内輪を決める。進むほど弧を広げて距離を稼ぐ
        int innerStage = static_cast<int>(turnedDeg / turnDeg * innerStageCount);
        if(innerStage >= innerStageCount) {
            innerStage = innerStageCount - 1;
        }
        int appliedOuterPwm = outerPwm;
        int appliedInnerPwm = kAreaDiagonalInnerPwms[innerStage];

        // 出だしだけ段階的に出力を上げる。左右の比を保ったまま縮めるので、描く弧は変わらない
        int rampStage = loopCount / rampStageLoopCount;
        if(rampStage < rampStageCount) {
            appliedOuterPwm = appliedOuterPwm * kDiagonalRampPwms[rampStage] / kDiagonalPwmHigh;
            appliedInnerPwm = appliedInnerPwm * kDiagonalRampPwms[rampStage] / kDiagonalPwmHigh;
        }

        robot.setMotorPower(isOuterLeft ? appliedOuterPwm : appliedInnerPwm,
                            isOuterLeft ? appliedInnerPwm : appliedOuterPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
        turnedDeg = std::fabs(robot.getImuHeading() - startHeading);
    }
    robot.stop();
    syslog(LOG_NOTICE, "Diagonal done: %d deg in %dms", (int)turnedDeg, loopCount * Config::MOTION_POLL_INTERVAL_US / 1000);
}

// 進行方向を変える前に、ブレーキで速度を落とし切る。惰性が残ったまま逆を指令すると
// 減速と加速が同じ制御に混ざり、初速が出ない（サーボの場合）か電流が跳ねる（PWM直叩きの場合）。
// 前進からのブレーキは機首が下がる向きなので、後退時のように車体が浮くことはない
void Test::brakeUntilStopped(int speedThresholdDegPerSec, int timeoutMs) {
    const int timeoutLoopCount = (timeoutMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
    for(int i = 0; i < timeoutLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            return;
        }

        // 掛けっぱなしにせず間引くことで、制動力を平均で下げる
        if((i % kSettleBrakeCycleLoops) < kSettleBrakeOnLoops) {
            robot.brake();
        } else {
            robot.stop();
        }
        // Robotは速度を公開していないため、モーターのgetterから直接読む
        int leftSpeed = robot.getLeftMotor().getSpeed();
        int rightSpeed = robot.getRightMotor().getSpeed();
        if(std::abs(leftSpeed) < speedThresholdDegPerSec && std::abs(rightSpeed) < speedThresholdDegPerSec) {
            syslog(LOG_NOTICE, "Settled in %dms", i * Config::MOTION_POLL_INTERVAL_US / 1000);
            return;
        }
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    syslog(LOG_NOTICE, "STOP[brakeUntilStopped]: TIMEOUT");
}

// トルク上限を落として直進/後退する。setSpeed()のサーボ制御は加速度を抑えてくれるが、
// それでも短距離では立ち上がりのトルクで車体が浮くため、デューティ上限でトルクの頭を押さえる。
// 戻し忘れると以降のライントレースまで非力になるので、必ず元の値に復帰させること
void Test::driveStraightWithDutyLimit(int distanceMm, int speedDegPerSec, int dutyLimit) {
    int oldLeftLimit = robot.getLeftMotor().setDutyLimit(dutyLimit);
    int oldRightLimit = robot.getRightMotor().setDutyLimit(dutyLimit);

    robot.driveStraight(distanceMm, speedDegPerSec);

    robot.getLeftMotor().restoreDutyLimit(oldLeftLimit);
    robot.getRightMotor().restoreDutyLimit(oldRightLimit);
}

// 両輪を逆向きに回してその場で旋回する。turnByImu(閉ループ)より速く回せるので、
// ボトル配置後のように精度より速度が欲しい場面で使う
void Test::turnInPlaceByImu(int leftPwm, int rightPwm, float turnDeg) {
    float startHeading = robot.getImuHeading();
    while(std::fabs(robot.getImuHeading() - startHeading) < turnDeg) {
        if(robot.isCenterButtonPressed()) {
            break;
        }
        robot.setMotorPower(leftPwm, rightPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    robot.stop();
}

// 帰りの線探し。色判定ではなく反射率のしきい値で線を判定する。
// 以前は左右交互の蛇行だったが、振り戻しをやめて、優先側への片輪ピボットのまま線を見つけるまで回し続ける
void Test::pivotUntilReflectionBelow(int reflectionThreshold, int pwm, bool isRightTurn) {
    float startHeading = robot.getImuHeading();
    int loopCount = 0;

    while(std::fabs(robot.getImuHeading() - startHeading) < kSearchMaxTurnDeg) {
        if(robot.isCenterButtonPressed()) {
            robot.stop();
            return;
        }

        int reflection = robot.getReflection();
        // ここに診断用のbeep(30)を入れていたが、beep()は内部がdly_tsk(duration*1000)でブロッキングする。
        // 白以外を読むたびに30ms停止し、10ms周期の制御が実質40ms周期になっていたため削除した
        if(reflection < reflectionThreshold) {
            robot.stop();
            syslog(LOG_NOTICE, "Line found by reflection: %d", reflection);

            int foundLoopCount = static_cast<int>(kSearchFoundSec * 1000 * 1000 / Config::MOTION_POLL_INTERVAL_US);
            int foundMoveLeftPwm = isRightTurn ? kSearchFoundPwmLeft : kSearchFoundPwmRight;
            int foundMoveRightPwm = isRightTurn ? kSearchFoundPwmRight : kSearchFoundPwmLeft;
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

        if(loopCount >= kSearchTimeoutLoopCount) {
            syslog(LOG_NOTICE, "STOP[pivotUntilReflectionBelow]: TIMEOUT");
            break;
        }

        robot.setMotorPower(isRightTurn ? pwm : 0, isRightTurn ? 0 : pwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
    }
    robot.stop();
    syslog(LOG_NOTICE, "STOP[pivotUntilReflectionBelow]: line not found");
}

void Test::run() {
    syslog(LOG_NOTICE, "--- Test (after corner) Started ---");

    // 直角コーナーは本番(DeliveryTask)へ移したので、このテストは「90度カーブを曲がりきってラインに復帰した直後」
    // の状態から始まると仮定する。右エッジ・通常速度で、コーナー判定は最初から無効。
    // 走行体を手でその位置・向きに置いてからスタートさせること。
    bool isLeftCourse = CourseConfig::isLeftCourse();

    // 色判定は行わず、青ボトルを読んだ場合と同じ扱いで固定する。
    // カーブまでに青1本目を通過済みなので、ここから数えると青は残り2本でエリアへ向かう
    // （黄なら残り1本、赤なら残り3本）
    ColorJudge::Color bottleColor = ColorJudge::Color::BLUE;
    int targetBlueLineCount = 3;
    syslog(LOG_NOTICE, "Assuming Bottle Color: BLUE (1 blue already passed)");

    Tracer tracer(robot);
    tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
    tracer.setPwm(Config::TRACER_PWM);

    // 9. 指定回数青ラインを検知するまでライントレース
    syslog(LOG_NOTICE, "Tracing until blue line count: %d", targetBlueLineCount);

    int detectedBlueCount = 1;       // 青1本目は通過済みとして1から数え始める
    bool isCurrentlyOnBlue = false;  // 現在青ライン上にいるかのフラグ

    int matchedBlueCount = 0;     // 青を連続で読んだ回数
    int matchedNonBlueCount = 0;  // 青以外を連続で読んだ回数
    ColorJudge::Color targetColor = ColorJudge::Color::BLUE;

    // ms指定の確定時間を、現在のLINE_TRACE_POLL_INTERVAL_US(制御周期)でのサンプル回数に換算
    int blueEntryConfirmCount = (kBlueEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int blueFinalEntryConfirmCount = (kBlueFinalEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int bluePassedConfirmCount = (kBluePassedConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    while(true) {
        if(robot.isCenterButtonPressed()) {
            break;
        }

        if(!isCurrentlyOnBlue) {
            // まだ青ラインに乗っていない状態：青を探す。
            // エリアへ向かう最後の1本だけは、青の入口で抜けられるよう確定を早める
            bool isFinalBlue = (detectedBlueCount + 1 >= targetBlueLineCount);
            bool isOnBlueNow = robot.isOnColors(&targetColor, 1, matchedBlueCount, isFinalBlue ? blueFinalEntryConfirmCount : blueEntryConfirmCount);
            if(isOnBlueNow) {
                isCurrentlyOnBlue = true;
                matchedNonBlueCount = 0;  // 青以外カウントをリセット

                // 青を読んだ瞬間にカウントアップ！
                detectedBlueCount++;
                robot.beep(100);
                syslog(LOG_NOTICE, "Entered blue line. Count: %d / %d", detectedBlueCount, targetBlueLineCount);
                // 指定回数に達したら、その場ですぐに終了する
                if(detectedBlueCount >= targetBlueLineCount) {
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
            }
        }

        // 青ライン上でも関係なく通常のライントレースを継続
        tracer.run();

        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "Reached target zone.");

    // 10. エリアへの配置（斜め移動 → 後退 → 右旋回）。色に関わらず共通。
    // 前進は開始までの立ち上がりが遅くボトルネックだったため廃止し、斜め移動だけでエリアまで運ぶ
    // 安定角度・フォールバック角度は採用せず、この時点の現在角度を起点に80度斜め移動する
    syslog(LOG_NOTICE, "Diagonal move into area");
    diagonalMoveUntilImuTurn(isLeftCourse, kDiagonalPwmHigh, robot.getImuHeading(), kDiagonalTurnDeg);

    // 後退に移る前に斜め移動の惰性を殺す。これをしないと後退の初速が出ない（実測で後退に785ms掛かっていた）
    brakeUntilStopped(kSettleSpeedDegPerSec, kSettleTimeoutMs);

    syslog(LOG_NOTICE, "Driving backward %dmm (duty limit %d)", kAreaBackwardMm, kAreaBackwardDutyLimit);
    driveStraightWithDutyLimit(kAreaBackwardMm, kAreaBackwardSpeedDegPerSec, kAreaBackwardDutyLimit);

    syslog(LOG_NOTICE, "Turning %d degrees in place", (int)kAreaTurnDeg);
    turnInPlaceByImu(isLeftCourse ? kAreaTurnPwm : -kAreaTurnPwm, isLeftCourse ? -kAreaTurnPwm : kAreaTurnPwm, kAreaTurnDeg);

    // 帰りの線探し（旋回方向はコース依存、反射率ベース）。色に関わらず共通。見つけた瞬間の移動もこの中で行う
    syslog(LOG_NOTICE, "Pivoting to find line by reflection");
    pivotUntilReflectionBelow(kSearchReflectionThreshold, kSearchPwm, isLeftCourse);

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
