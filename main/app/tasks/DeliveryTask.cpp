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
    constexpr int kCornerSuppressAfterBlueMs = 600;      // 青を跨ぐとき脇の白を踏むため、青の上と直後は判定を止める
    constexpr int kCornerSuppressAfterRecoveryMs = 1500;  // 復帰したが曲がりきってはいない場合、Tracerが掴み直す時間
    constexpr int kReturnCornerDetectStartDelayMs = 500;  // 帰りの判定開始前に置く無効期間

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

    int loopCount = 0;
    float turnedDeg = 0.0f;
    while(turnedDeg < turnDeg) {
        if(robot.isCenterButtonPressed()) {
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

// 帰りの線探し。片輪ピボットのまま反射率で線を見つけるまで回し続ける。
// 左右交互の蛇行だと、振り戻しで一度ライン上に来ても行き過ぎて見失うため一方向にした
void DeliveryTask::pivotUntilReflectionBelow(int reflectionThreshold, int pwm, bool isRightTurn) {
    float startHeading = robot.getImuHeading();
    int loopCount = 0;

    while(std::fabs(robot.getImuHeading() - startHeading) < kSearchMaxTurnDeg) {
        if(robot.isCenterButtonPressed()) {
            robot.stop();
            return;
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

        if(seenWhiteAfterGate && blackRun >= kCornerPivotBlackRunCount) {
            robot.stop();
            syslog(LOG_NOTICE, "Pivot done: line found (reflection %d, turned %d deg)", reflection, (int)turnedDeg);
            return true;
        }

        if(turnedDeg >= maxTurnDeg) {
            robot.stop();
            syslog(LOG_NOTICE, "STOP[pivotUntilLineFound]: MAX_TURN (reflection %d)", reflection);
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
    return false;
}

// コーナーを検知した後の旋回。線を捕まえ直し、失敗したら逆へ振り戻す。行き（左折）と帰り（右折）で共用する。
// 戻り値: 曲がりきったか。recoveredOutは、曲がりきってはいないが線には復帰したか
bool DeliveryTask::turnAtCorner(bool isLeftTurn, bool& recoveredOut) {
    float pivotTurnedDeg = 0.0f;
    bool lineFound = pivotUntilLineFound(isLeftTurn, kCornerPivotOuterPwm, kCornerPivotInnerPwm, kCornerPivotMinTurnDeg, kCornerPivotMaxTurnDeg, pivotTurnedDeg);

    // 正味の回転量。振り戻した場合は逆向きなので差し引く
    float netTurnedDeg = pivotTurnedDeg;
    bool sweepBackFoundLine = false;
    if(!lineFound) {
        // 回りすぎて通り過ぎたか、そもそも線が無い方向だった。
        // 逆向きに、開始角度を跨いで反対側まで振り戻して探す（線の位置が不明なので最低旋回角はかけない）
        syslog(LOG_NOTICE, "Pivot failed. Sweeping back the other way.");
        float sweepBackTurnedDeg = 0.0f;
        sweepBackFoundLine = pivotUntilLineFound(!isLeftTurn, kCornerPivotOuterPwm, kCornerPivotInnerPwm, 0.0f, kCornerPivotMaxTurnDeg * 2.0f, sweepBackTurnedDeg);
        netTurnedDeg = pivotTurnedDeg - sweepBackTurnedDeg;
        syslog(LOG_NOTICE, "Sweep back %s (turned %d deg, net %d deg)", sweepBackFoundLine ? "found line" : "FAILED", (int)sweepBackTurnedDeg, (int)netTurnedDeg);
    }
    recoveredOut = sweepBackFoundLine;

    // 正味で大きく回ってラインに復帰できた＝本物の90度カーブを曲がりきった。
    // 振り戻しで復帰した場合も正味の回転量で判断する（振り戻しには最低旋回角のゲートが無く、
    // 元の線を掴んでいる可能性があるため、「線を見つけた」だけでは曲がりきった証拠にならない）
    if((lineFound || sweepBackFoundLine) && netTurnedDeg >= kCornerDoneTurnDeg) {
        syslog(LOG_NOTICE, "Corner cleared (net %d deg).", (int)netTurnedDeg);
        return true;
    }
    if(sweepBackFoundLine) {
        // 線には乗ったが曲がりきってはいない
        syslog(LOG_NOTICE, "Recovered onto line but corner not cleared (net %d deg).", (int)netTurnedDeg);
    }
    return false;
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
    robot.beep(50);  // 速度切り替わりの合図
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
    bool cornerDetectPending = false;
    bool cornerDetectEnabled = false;
    bool cornerDone = false;  // 曲がりきったら立てる。以降は判定を一切行わない
    int cornerWhiteRun = 0;
    int cornerSuppressCount = 0;  // 0より大きい間はコーナー検知を止める
    int traceLoopCount = 0;
    int cornerDetectEnableLoop = 0;
    const int cornerDetectStartDelayCount = (kCornerDetectStartDelayMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    const int cornerSuppressAfterBlueCount = (kCornerSuppressAfterBlueMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    const int cornerSuppressAfterRecoveryCount = (kCornerSuppressAfterRecoveryMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

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
                    cornerDetectPending = true;
                    cornerDetectEnableLoop = traceLoopCount + cornerDetectStartDelayCount;
                }
            }
        }

        // 直角コーナーで線を見失ったら、その場で旋回して線を捕まえ直す。
        // 青ライン上とその直後は、脇の白を踏んで誤検知するため抑制する
        // （誤検知したまま旋回すると致命的なので必須）
        traceLoopCount++;
        if(cornerSuppressCount > 0) {
            cornerSuppressCount--;
        }
        if(isCurrentlyOnBlue) {
            cornerSuppressCount = cornerSuppressAfterBlueCount;
        }
        if(cornerDetectPending && !cornerDone && traceLoopCount >= cornerDetectEnableLoop) {
            cornerDetectPending = false;
            cornerDetectEnabled = true;
            syslog(LOG_NOTICE, "Corner detection ENABLED.");
        }

        if(cornerDetectEnabled && cornerSuppressCount == 0) {
            int cornerReflection = robot.getReflection();
            if(cornerReflection >= kCornerWhiteReflection) {
                cornerWhiteRun++;
            } else {
                cornerWhiteRun = 0;
            }

            if(cornerWhiteRun >= kCornerWhiteRunCount) {
                syslog(LOG_NOTICE, "Corner detected (reflection %d). Pivoting.", cornerReflection);

                // Lコースでは左90度コーナーなので左へ回す
                bool recoveredOntoLine = false;
                bool cornerCleared = turnAtCorner(isLeftCourse, recoveredOntoLine);

                cornerWhiteRun = 0;
                cornerSuppressCount = cornerSuppressAfterBlueCount;

                if(cornerCleared) {
                    // カーブは1つしかないので、通過後の検知は誤検知のリスクにしかならない
                    cornerDone = true;
                    cornerDetectEnabled = false;
                    tracer.setPwm(Config::TRACER_PWM);
                    syslog(LOG_NOTICE, "Corner detection DISABLED, back to normal trace.");
                } else if(recoveredOntoLine) {
                    // すぐ再検知すると連鎖するので、Tracerに掴み直す時間を与える
                    cornerSuppressCount = cornerSuppressAfterRecoveryCount;
                }
            }
        }

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

    // 帰りにも直角コーナーが1つある。行きは左折だったが帰りは右折になる（Lコース基準）。
    // 判定を始める時点はボトル色で変わるが、いずれも「終了する青ラインの1本手前を通過し終えた直後」なので
    // finalTargetBlueLineCount - 1 本目の通過で揃う（黄=0本＝トレース開始直後 / 青=1本目 / 赤=2本目）
    const int returnCornerAfterBlueCount = finalTargetBlueLineCount - 1;
    const int returnCornerStartDelayCount = (kReturnCornerDetectStartDelayMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    bool returnCornerPending = (returnCornerAfterBlueCount <= 0);
    bool returnCornerEnabled = false;
    bool returnCornerDone = false;  // 帰りもカーブは1つだけなので、曲がりきったら二度と判定しない
    int returnCornerWhiteRun = 0;
    int returnCornerSuppressCount = 0;
    int returnTraceLoopCount = 0;
    int returnCornerEnableLoop = returnCornerStartDelayCount;

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

                // 目的の本数を通過し終えたら、無効期間を置いてからコーナー判定を始める
                if(!returnCornerDone && returnCornerAfterBlueCount > 0 && finalDetectedBlueCount == returnCornerAfterBlueCount) {
                    returnCornerPending = true;
                    returnCornerEnableLoop = returnTraceLoopCount + returnCornerStartDelayCount;
                }
            }
        }

        // 帰りの直角コーナー判定。青ライン上とその直後は脇の白を踏んで誤検知するため抑制する
        returnTraceLoopCount++;
        if(returnCornerSuppressCount > 0) {
            returnCornerSuppressCount--;
        }
        if(isCurrentlyOnFinalBlue) {
            returnCornerSuppressCount = cornerSuppressAfterBlueCount;
        }
        if(returnCornerPending && !returnCornerDone && returnTraceLoopCount >= returnCornerEnableLoop) {
            returnCornerPending = false;
            returnCornerEnabled = true;
            syslog(LOG_NOTICE, "Return corner detection ENABLED.");
        }

        if(returnCornerEnabled && returnCornerSuppressCount == 0) {
            int cornerReflection = robot.getReflection();
            if(cornerReflection >= kCornerWhiteReflection) {
                returnCornerWhiteRun++;
            } else {
                returnCornerWhiteRun = 0;
            }

            if(returnCornerWhiteRun >= kCornerWhiteRunCount) {
                syslog(LOG_NOTICE, "Return corner detected (reflection %d). Pivoting.", cornerReflection);

                // 帰りはLコースで右90度コーナーなので、行きとは逆の右へ回す
                bool recoveredOntoLine = false;
                bool cornerCleared = turnAtCorner(!isLeftCourse, recoveredOntoLine);

                returnCornerWhiteRun = 0;
                returnCornerSuppressCount = cornerSuppressAfterBlueCount;

                if(cornerCleared) {
                    returnCornerDone = true;
                    returnCornerEnabled = false;
                    tracer.setPwm(Config::TRACER_PWM);
                    syslog(LOG_NOTICE, "Return corner detection DISABLED, back to normal trace.");
                } else if(recoveredOntoLine) {
                    returnCornerSuppressCount = cornerSuppressAfterRecoveryCount;
                }
            }
        }

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    syslog(LOG_NOTICE, "--- DeliveryTask Finished ---");
}
