#include "DeliveryTask.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"
#include <kernel.h>
#include <t_syslog.h>
#include <cmath>

namespace {
    // ── ライントレース速度 ──────────────────────────────
    constexpr int kApproachPwm = 40;         // ボトル接近中。Config::DELIVERY_TRACER_PWM(30)だとカーブ減速でほぼ動けなくなる
    constexpr int kReacquireLinePwm = 50;    // ライン復帰直後。ラインに対するズレが大きくカーブ減速が効きやすいので高め
    constexpr int kPostSlowTracePwm = 65;    // ステップ8以降。Config::TRACER_PWM(80)は実機では速すぎた
    constexpr int kOnFirstBlueLinePwm = 50;  // 青1本目に乗っている間だけ落とす速度

    // ── アームを下げた直後の直進（これだけでラインへ復帰させる）──
    constexpr int kAfterArmStraightLeftPwm = 35;
    constexpr int kAfterArmStraightRightPwm = 40;
    constexpr float kAfterArmStraightSec = 1.0f;

    // ── 青ライン判定 ──────────────────────────────────
    // 確定はサンプル数ではなくms基準にする。制御周期が変わっても意図した時間幅を保つため
    constexpr int kBlueEntryConfirmMs = 300;   // 青に乗ったと確定するまでの時間
    constexpr int kBluePassedConfirmMs = 400;  // 青を通過した（完全に降りた）と確定するまでの時間
    // エリアへ向かう最後の1本だけ短くする。300msだと確定までに約60mm進み、入口ではなく出口で抜けてしまうため
    constexpr int kBlueFinalEntryConfirmMs = 50;

    // ── 直角コーナー対策 ───────────────────────────────
    // Tracerのカーブ減速はEMA(時定数約290ms)でステップ状の変化に間に合わないため、
    // 白の連続で「線を見失った」を検知し、ピボット旋回で曲がり直す
    constexpr int kCornerWhiteReflection = 85;       // これ以上を白とみなす（実測の白は約99）
    constexpr int kCornerBlackReflection = 35;       // これ以下を黒（実測 黒15/青37）。45では境目のグラデーション(43〜55)を誤検知した
    constexpr int kCornerWhiteRunCount = 12;         // 直線部でも白は6〜7回連続する。12ならマージン5サンプル
    constexpr int kCornerDetectStartDelayMs = 1000;  // 80度旋回直後は姿勢が乱れて誤検知するため判定を止める時間

    constexpr int kCornerPivotOuterPwm = 57;      // 両輪逆転は回転ジャークでボトルを落とすため片輪駆動。強すぎると線を踏み抜く
    constexpr int kCornerPivotInnerPwm = 0;       // 0で片輪旋回。負にすると半径は縮むがボトルへの負荷が増える
    constexpr int kCornerPivotRampLoopCount = 15;  // 立ち上がりでボトルを振らないよう150msかけて上げる
    constexpr int kCornerPivotBlackRunCount = 3;   // 1サンプルのノイズで抜けないための連続回数
    // 行き過ぎた直後は元の線の方が近く先に当たる。元の線だと18度で終わるが正解時は72〜86度なので40度で分離できる
    constexpr float kCornerPivotMinTurnDeg = 40.0f;
    // 線がどこにあっても減速済みで到達するよう、ゲートが開く角度から線形に出力を落とす
    constexpr float kCornerPivotTaperStartDeg = kCornerPivotMinTurnDeg;
    constexpr float kCornerPivotTaperEndDeg = 100.0f;
    constexpr float kCornerPivotTaperMinRatio = 0.6f;  // 57×0.6≒34。低すぎると動かなくなる
    constexpr float kCornerDoneTurnDeg = 70.0f;        // これだけ回って復帰できたら曲がりきったとみなし、以降の判定を止める
    constexpr float kCornerPivotMaxTurnDeg = 120.0f;   // 暴走を止める保険
    constexpr int kCornerPivotTimeoutLoopCount = 300;  // 保険その2（10ms周期なので3秒）
    constexpr int kCornerSuppressAfterBlueMs = 600;   // 青を跨ぐとき脇の白を踏むため、青の上と直後は判定を止める
    constexpr int kCornerRetryAfterFailMs = 1500;     // 線を見つけられなかったときに次の検知まで置く間隔
    constexpr int kCornerConfirmTimeoutMs = 1500;     // 線に復帰した後、Tracerが曲がりきるのを待つ上限
    constexpr int kReturnCornerDetectStartDelayMs = 500;  // 帰りの判定開始前に置く無効期間
    constexpr int kCornerSuppressAfterBlueCount = (kCornerSuppressAfterBlueMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    constexpr int kCornerRetryAfterFailCount = (kCornerRetryAfterFailMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    constexpr int kCornerConfirmTimeoutCount = (kCornerConfirmTimeoutMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

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
    // IMUが動かなくなった場合に回り続けるのを防ぐ保険。実測は斜め移動600ms・その場旋回308ms
    constexpr int kDiagonalTimeoutMs = 3000;
    constexpr int kAreaTurnTimeoutMs = 2000;

    // ── 帰りの線探し ──────────────────────────────────
    // 色判定ではなく反射率で判定する（TRACER_TARGET_REFLECTIONとCOLOR_ACHROMATIC_REFLECTION_THRESHOLDが
    // 同値のため、ライントレース中の白黒判定が最も不安定になるため）
    constexpr int kSearchReflectionThreshold = 55;
    constexpr int kSearchPwm = 65;
    constexpr float kSearchMaxTurnDeg = 180.0f;      // これ以上回っても線から離れるだけなので打ち切る
    constexpr int kSearchTimeoutLoopCount = 500;     // 保険（10ms周期なので5秒）
    constexpr int kSearchFoundPwmLeft = 30;          // 線を見つけた瞬間に踏み込む左右PWM
    constexpr int kSearchFoundPwmRight = 90;
    constexpr float kSearchFoundSec = 0.10f;

}  // namespace

DeliveryTask::DeliveryTask(Robot& robot)
    : robot(robot) {
}

// 左右のパワー差で弧を描きながら、startHeadingからturnDeg回頭したら停止する。
// 内輪は回頭の進み具合で段階的に上げる（時間ではなく角度基準なので、速度がばらついても弧の形が変わらない）
void DeliveryTask::diagonalMoveUntilImuTurn(bool isOuterLeft, int outerPwm, float startHeading, float turnDeg) {
    const int rampStageLoopCount = (kDiagonalRampStageMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
    const int rampStageCount = static_cast<int>(sizeof(kDiagonalRampPwms) / sizeof(kDiagonalRampPwms[0]));
    const int innerStageCount = static_cast<int>(sizeof(kAreaDiagonalInnerPwms) / sizeof(kAreaDiagonalInnerPwms[0]));

    const int timeoutLoopCount = (kDiagonalTimeoutMs * 1000) / Config::MOTION_POLL_INTERVAL_US;

    int loopCount = 0;
    float turnedDeg = 0.0f;
    while(turnedDeg < turnDeg) {
        if(robot.isCenterButtonPressed()) {
            break;
        }
        if(loopCount >= timeoutLoopCount) {
            syslog(LOG_NOTICE, "STOP[diagonalMoveUntilImuTurn]: TIMEOUT");
            break;
        }

        int innerStage = static_cast<int>(turnedDeg / turnDeg * innerStageCount);
        if(innerStage >= innerStageCount) {
            innerStage = innerStageCount - 1;
        }
        int appliedOuterPwm = outerPwm;
        int appliedInnerPwm = kAreaDiagonalInnerPwms[innerStage];

        // 出だしだけ出力を抑える。左右の比を保ったまま縮めるので、描く弧は変わらない
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
// 減速と加速が同じ制御に混ざり、初速が出ない（実測で後退に785ms掛かっていた）
void DeliveryTask::brakeUntilStopped(int speedThresholdDegPerSec, int timeoutMs) {
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

// デューティ上限（＝トルク上限）を落として直進/後退する。
// 戻し忘れると以降のライントレースまで非力になるため、必ずこの関数内で復帰させる
void DeliveryTask::driveStraightWithDutyLimit(int distanceMm, int speedDegPerSec, int dutyLimit) {
    int oldLeftLimit = robot.getLeftMotor().setDutyLimit(dutyLimit);
    int oldRightLimit = robot.getRightMotor().setDutyLimit(dutyLimit);

    robot.driveStraight(distanceMm, speedDegPerSec);

    robot.getLeftMotor().restoreDutyLimit(oldLeftLimit);
    robot.getRightMotor().restoreDutyLimit(oldRightLimit);
}

// 両輪を逆向きに回してその場で旋回する。turnByImu(閉ループ)より速く回せる
void DeliveryTask::turnInPlaceByImu(int leftPwm, int rightPwm, float turnDeg) {
    const int timeoutLoopCount = (kAreaTurnTimeoutMs * 1000) / Config::MOTION_POLL_INTERVAL_US;

    float startHeading = robot.getImuHeading();
    int loopCount = 0;
    while(std::fabs(robot.getImuHeading() - startHeading) < turnDeg) {
        if(robot.isCenterButtonPressed()) {
            break;
        }
        if(loopCount >= timeoutLoopCount) {
            syslog(LOG_NOTICE, "STOP[turnInPlaceByImu]: TIMEOUT");
            break;
        }
        robot.setMotorPower(leftPwm, rightPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
        loopCount++;
    }
    robot.stop();
}

// 帰りの線探し。片輪ピボットのまま反射率で線を見つけるまで回し続ける。
// 左右交互の蛇行だと、振り戻しで一度ライン上に来ても行き過ぎて見失うため一方向にした
bool DeliveryTask::pivotUntilReflectionBelow(int reflectionThreshold, int pwm, bool isRightTurn) {
    float startHeading = robot.getImuHeading();
    int loopCount = 0;

    while(std::fabs(robot.getImuHeading() - startHeading) < kSearchMaxTurnDeg) {
        if(robot.isCenterButtonPressed()) {
            robot.stop();
            return false;
        }

        int reflection = robot.getReflection();
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
            return true;
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
    return false;
}

// 直角コーナー用。内輪を落として旋回し、線を見つけた時点で止まる。
// 主の終了条件は反射率（線が正解）。どれだけ行き過ぎたかは毎回変わるので、IMUで固定角度を狙うのではなく
// 「ここまで回っても見つからなければ何かおかしい」の保険としてだけIMU角度とループ回数を使う
bool DeliveryTask::pivotUntilLineFound(bool isLeftTurn, int outerPwm, int innerPwm, float minTurnDeg, float maxTurnDeg, float& turnedDegOut) {
    float startHeading = robot.getImuHeading();
    int blackRun = 0;
    // ゲートが開いた後に「白を踏んでから黒に入った」ことを要求する。
    // ゲート中に線の近くへ居座ったまま開いた瞬間に抜けるのを防ぐ
    // （実測: ゲート40度に対し turned 40 ちょうどや、turned 47・反射率33=グラデーション帯で終了していた。
    //  正しく曲がれたときは白の上を通過してから反射率17〜18の真っ黒で終了している）
    bool seenWhiteAfterGate = false;
    turnedDegOut = 0.0f;

    // 以下は診断専用。制御には一切使わず、旋回が終わったときに一度だけまとめて出す
    // （周期ごとにログを出すと制御周期が伸びるうえ、以前カウンタを制御と共用して挙動を壊したことがある）
    int firstWhiteAfterGateDeg = -1;  // ゲート後に初めて白を読んだ角度
    int minReflectionAfterGate = 101;
    int minReflectionAtDeg = -1;
    int maxBlackRun = 0;

    for(int loopCount = 0; loopCount < kCornerPivotTimeoutLoopCount; loopCount++) {
        if(robot.isCenterButtonPressed()) {
            robot.stop();
            return false;
        }

        float turnedDeg = std::fabs(robot.getImuHeading() - startHeading);
        turnedDegOut = turnedDeg;

        int reflection = robot.getReflection();
        if(reflection <= kCornerBlackReflection) {
            blackRun++;
        } else {
            blackRun = 0;
        }
        if(turnedDeg >= minTurnDeg && reflection >= kCornerWhiteReflection) {
            seenWhiteAfterGate = true;
        }

        if(turnedDeg >= minTurnDeg) {
            if(firstWhiteAfterGateDeg < 0 && reflection >= kCornerWhiteReflection) {
                firstWhiteAfterGateDeg = static_cast<int>(turnedDeg);
            }
            if(reflection < minReflectionAfterGate) {
                minReflectionAfterGate = reflection;
                minReflectionAtDeg = static_cast<int>(turnedDeg);
            }
        }
        if(blackRun > maxBlackRun) {
            maxBlackRun = blackRun;
        }

        if(seenWhiteAfterGate && blackRun >= kCornerPivotBlackRunCount) {
            robot.stop();
            syslog(LOG_NOTICE, "Pivot done: line found (reflection %d, turned %d deg)", reflection, (int)turnedDeg);
        syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
            return true;
        }

        if(turnedDeg >= maxTurnDeg) {
            robot.stop();
            syslog(LOG_NOTICE, "STOP[pivotUntilLineFound]: MAX_TURN (reflection %d)", reflection);
        syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
            return false;
        }

        // 立ち上がりを緩やかにしてボトルを振らないようにする
        float powerRatio = 1.0f;
        if(loopCount < kCornerPivotRampLoopCount) {
            powerRatio = static_cast<float>(loopCount + 1) / kCornerPivotRampLoopCount;
        }

        // 線を探す区間は線形に落とす。ランプ中と重なる場合は小さい方を採用する。
        // 終端はmaxTurnDegではなく専用の定数にする（逆方向探索でmaxTurnが240度になっても傾きを変えないため）
        if(turnedDeg >= kCornerPivotTaperStartDeg) {
            float taperRatio = 1.0f - (1.0f - kCornerPivotTaperMinRatio) * (turnedDeg - kCornerPivotTaperStartDeg) / (kCornerPivotTaperEndDeg - kCornerPivotTaperStartDeg);
            if(taperRatio < kCornerPivotTaperMinRatio) {
                taperRatio = kCornerPivotTaperMinRatio;
            }
            if(taperRatio < powerRatio) {
                powerRatio = taperRatio;
            }
        }

        int scaledOuterPwm = static_cast<int>(outerPwm * powerRatio);
        int scaledInnerPwm = static_cast<int>(innerPwm * powerRatio);

        // 左旋回なら左が内輪、右旋回なら右が内輪
        robot.setMotorPower(isLeftTurn ? scaledInnerPwm : scaledOuterPwm, isLeftTurn ? scaledOuterPwm : scaledInnerPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }

    robot.stop();
    syslog(LOG_NOTICE, "STOP[pivotUntilLineFound]: TIMEOUT");
    syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
    return false;
}

// コーナーを検知した後の旋回。線を捕まえ直し、失敗したら逆へ振り戻す。行き（左折）と帰り（右折）で共用する。
// 完了判定は「正方向の旋回量 - 振り戻し量」ではなく、startHeadingからの実方位差で行う。
// 正方向と振り戻しでは軸にする車輪が変わって旋回中心が別物になるうえ、停止後の惰性も引き算には入らないため
DeliveryTask::CornerResult DeliveryTask::turnAtCorner(bool isLeftTurn, float startHeading) {
    float pivotTurnedDeg = 0.0f;
    bool lineFound = pivotUntilLineFound(isLeftTurn, kCornerPivotOuterPwm, kCornerPivotInnerPwm, kCornerPivotMinTurnDeg, kCornerPivotMaxTurnDeg, pivotTurnedDeg);

    bool sweepBackFoundLine = false;
    if(!lineFound) {
        // 回りすぎて通り過ぎたか、そもそも線が無い方向だった。
        // 逆向きに、開始角度を跨いで反対側まで振り戻して探す（線の位置が不明なので最低旋回角はかけない）
        syslog(LOG_NOTICE, "Pivot failed. Sweeping back the other way.");
        float sweepBackTurnedDeg = 0.0f;
        sweepBackFoundLine = pivotUntilLineFound(!isLeftTurn, kCornerPivotOuterPwm, kCornerPivotInnerPwm, 0.0f, kCornerPivotMaxTurnDeg * 2.0f, sweepBackTurnedDeg);
        syslog(LOG_NOTICE, "Sweep back %s (turned %d deg)", sweepBackFoundLine ? "found line" : "FAILED", (int)sweepBackTurnedDeg);
    }

    if(!lineFound && !sweepBackFoundLine) {
        return CornerResult::FAILED;
    }

    float turnedDeg = std::fabs(robot.getImuHeading() - startHeading);
    return (turnedDeg >= kCornerDoneTurnDeg) ? CornerResult::CLEARED : CornerResult::REACQUIRED;
}

// コーナー検知の1周期分。ライントレースのループから毎周期呼ぶ。
// 行き・帰りで状態を別に持つだけで、判定そのものは共通
void DeliveryTask::updateCornerDetection(CornerState& state, bool isLeftTurn, Tracer& tracer, bool isOnBlue, const char* label) {
    if(state.done) {
        return;
    }

    state.loopCount++;
    if(state.suppressCount > 0) {
        state.suppressCount--;
    }
    // 青ラインを跨ぐときは脇の白を踏んで誤検知するため抑制する
    if(isOnBlue) {
        state.suppressCount = kCornerSuppressAfterBlueCount;
    }

    if(state.pending && state.loopCount >= state.enableLoop) {
        state.pending = false;
        state.enabled = true;
        syslog(LOG_NOTICE, "%s detection ENABLED.", label);
    }

    // 線には復帰したが曲がりきってはいない状態。残りはTracerに任せ、方位が届いたら完了とする
    if(state.confirming) {
        float turnedDeg = std::fabs(robot.getImuHeading() - state.startHeading);
        if(turnedDeg >= kCornerDoneTurnDeg) {
            state.confirming = false;
            state.done = true;
            tracer.setPwm(Config::TRACER_PWM);
            syslog(LOG_NOTICE, "%s cleared by tracer (%d deg). Detection DISABLED.", label, (int)turnedDeg);
        } else if(state.loopCount >= state.confirmDeadlineLoop) {
            // 曲がりきれていない。検知を戻してやり直させる
            state.confirming = false;
            state.enabled = true;
            state.whiteRun = 0;
            syslog(LOG_NOTICE, "%s NOT cleared after wait (%d deg). Re-arming.", label, (int)turnedDeg);
        }
        return;
    }

    if(!state.enabled || state.suppressCount > 0) {
        return;
    }

    int reflection = robot.getReflection();
    if(reflection >= kCornerWhiteReflection) {
        state.whiteRun++;
    } else {
        state.whiteRun = 0;
    }
    if(state.whiteRun < kCornerWhiteRunCount) {
        return;
    }

    syslog(LOG_NOTICE, "%s detected (reflection %d). Pivoting.", label, reflection);
    state.whiteRun = 0;
    state.startHeading = robot.getImuHeading();
    state.suppressCount = kCornerSuppressAfterBlueCount;

    CornerResult result = turnAtCorner(isLeftTurn, state.startHeading);
    float turnedDeg = std::fabs(robot.getImuHeading() - state.startHeading);

    if(result == CornerResult::CLEARED) {
        // カーブは1周に1つしかないので、通過後の検知は誤検知のリスクにしかならない
        state.enabled = false;
        state.done = true;
        tracer.setPwm(Config::TRACER_PWM);
        syslog(LOG_NOTICE, "%s cleared (%d deg). Detection DISABLED.", label, (int)turnedDeg);
    } else if(result == CornerResult::REACQUIRED) {
        state.enabled = false;
        state.confirming = true;
        state.confirmDeadlineLoop = state.loopCount + kCornerConfirmTimeoutCount;
        syslog(LOG_NOTICE, "%s reacquired line (%d deg). Waiting for tracer.", label, (int)turnedDeg);
    } else {
        // 線を見つけられなかった。間隔を置いてから再挑戦させる
        state.suppressCount = kCornerRetryAfterFailCount;
        syslog(LOG_NOTICE, "%s pivot FAILED (%d deg). Retrying later.", label, (int)turnedDeg);
    }
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

    // 1. ライントレースしながらボトルに近づく（この間、ラインに正対した基準角度をIMUで平均して求めておく）
    float headingSum = 0.0f;
    int headingSampleCount = 0;
    while(true) {
        if(robot.isCenterButtonPressed()) {  // センターボタンで安全停止
            tracer.terminate();
            return;
        }

        int currentDistance = robot.getUltrasonicDistance();

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

    // 5. 左35・右40のパワーで0.5秒直進してラインに復帰する（蛇行探索より速く、実機ではこれで十分だった）
    int afterArmStraightLoopCount = static_cast<int>(kAfterArmStraightSec * 1000 * 1000 / Config::MOTION_POLL_INTERVAL_US);
    for(int i = 0; i < afterArmStraightLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            return;
        }
        robot.setMotorPower(kAfterArmStraightLeftPwm, kAfterArmStraightRightPwm);
        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }
    robot.stop();

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
    int blueFinalEntryConfirmCount = (kBlueFinalEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int bluePassedConfirmCount = (kBluePassedConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    // 直角コーナー対策の状態。青1本目通過後の80度旋回から1秒経ったら有効化する
    CornerState outboundCorner;
    const int cornerDetectStartDelayCount = (kCornerDetectStartDelayMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

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
                syslog(LOG_NOTICE, "Entered blue line. Count: %d / %d", detectedBlueCount, targetBlueLineCount);
                if(detectedBlueCount == 1) {
                    // 仮実装（計測用）: 青1本目の上に乗っている間（判定しなくなるまで）は速度を落とす
                    tracer.setPwm(kOnFirstBlueLinePwm);
                }
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

                if(detectedBlueCount == 1) {
                    // 青1本目を通過し終えたら、ステップ1で求めた基準角度から80度回転した状態にする
                    // （試走会で直進・左斜め移動が不要と判断したため削除。旋回後そのまま右エッジでライントレースを再開する）
                    float targetHeading = baselineHeading - 80.0f * courseSign;
                    robot.turnByImu(targetHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

                    tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
                    tracer.setPwm(Config::TRACER_PWM);

                    // この先に90度カーブがある。旋回直後は姿勢が乱れていて誤検知するので、
                    // 1秒トレースしてから判定を有効化する
                    outboundCorner.pending = true;
                    outboundCorner.enableLoop = outboundCorner.loopCount + cornerDetectStartDelayCount;
                }
            }
        }

        // Lコースでは左90度コーナーなので左へ回す
        updateCornerDetection(outboundCorner, isLeftCourse, tracer, isCurrentlyOnBlue, "Corner");

        // 青ライン上でも関係なく通常のライントレースを継続
        tracer.run();

        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "Reached target zone.");

    // 10. エリアへの配置（斜め移動 → 惰性を殺す → 後退 → その場旋回）。色に関わらず共通。
    // 前進は立ち上がりが遅くボトルネックだったため廃止し、斜め移動の弧だけでエリアまで運ぶ
    syslog(LOG_NOTICE, "Diagonal move into area");
    diagonalMoveUntilImuTurn(isLeftCourse, kDiagonalPwmHigh, robot.getImuHeading(), kDiagonalTurnDeg);

    brakeUntilStopped(kSettleSpeedDegPerSec, kSettleTimeoutMs);

    syslog(LOG_NOTICE, "Driving backward %dmm (duty limit %d)", kAreaBackwardMm, kAreaBackwardDutyLimit);
    driveStraightWithDutyLimit(kAreaBackwardMm, kAreaBackwardSpeedDegPerSec, kAreaBackwardDutyLimit);

    syslog(LOG_NOTICE, "Turning %d degrees in place", (int)kAreaTurnDeg);
    turnInPlaceByImu(isLeftCourse ? kAreaTurnPwm : -kAreaTurnPwm, isLeftCourse ? -kAreaTurnPwm : kAreaTurnPwm, kAreaTurnDeg);

    // 帰りの線探し（旋回方向はコース依存、反射率ベース）。見つけた瞬間の踏み込みもこの中で行う
    syslog(LOG_NOTICE, "Pivoting to find line by reflection");
    if(!pivotUntilReflectionBelow(kSearchReflectionThreshold, kSearchPwm, isLeftCourse)) {
        // 線が無い場所でTracerを起動すると白の上を暴走するだけなので、ここで打ち切る
        syslog(LOG_NOTICE, "Line search FAILED. Aborting return trip.");
        robot.stop();
        return;
    }

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

    // 帰りにも直角コーナーが1つある。行きは左折だったが帰りは右折になる（Lコース基準）。
    // 判定を始める時点はボトル色で変わるが、いずれも「終了する青ラインの1本手前を通過し終えた直後」なので
    // finalTargetBlueLineCount - 1 本目の通過で揃う（黄=0本＝トレース開始直後 / 青=1本目 / 赤=2本目）
    const int returnCornerAfterBlueCount = finalTargetBlueLineCount - 1;
    const int returnCornerStartDelayCount = (kReturnCornerDetectStartDelayMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    CornerState returnCorner;
    returnCorner.pending = (returnCornerAfterBlueCount <= 0);  // 黄はトレース開始直後から数え始める
    returnCorner.enableLoop = returnCornerStartDelayCount;

    while(!robot.isCenterButtonPressed()) {
        if(!isCurrentlyOnFinalBlue) {
            // まだ青ラインに乗っていない状態：青を探す
            bool isOnBlueNow = robot.isOnColors(&targetColor, 1, finalMatchedBlueCount, blueEntryConfirmCount);
            if(isOnBlueNow) {
                isCurrentlyOnFinalBlue = true;
                finalMatchedNonBlueCount = 0;  // 青以外カウントをリセット

                finalDetectedBlueCount++;
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

                // 目的の本数を通過し終えたら、無効期間を置いてからコーナー判定を始める
                if(!returnCorner.done && returnCornerAfterBlueCount > 0 && finalDetectedBlueCount == returnCornerAfterBlueCount) {
                    returnCorner.pending = true;
                    returnCorner.enableLoop = returnCorner.loopCount + returnCornerStartDelayCount;
                }
            }
        }

        // 帰りはLコースで右90度コーナーなので、行きとは逆の右へ回す
        updateCornerDetection(returnCorner, !isLeftCourse, tracer, isCurrentlyOnFinalBlue, "Return corner");

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "--- DeliveryTask Finished ---");
}
