#include "DeliveryTask.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"
#include <kernel.h>
#include <t_syslog.h>
#include <cmath>

namespace {
    // ── ライントレース速度 ──────────────────────────────
    constexpr int kApproachPwm = 50;         // ボトル接近中。Config::DELIVERY_TRACER_PWM(30)だとカーブ減速でほぼ動けなくなる
    constexpr int kReacquireLinePwm = 50;    // ライン復帰直後。ラインに対するズレが大きくカーブ減速が効きやすいので高め
    constexpr int kPostSlowTracePwm = 80;    // ステップ8以降。Config::TRACER_PWM(現在90)は実機では速すぎた
    constexpr int kOnFirstBlueLinePwm = 75;  // 青1本目に乗っている間だけ落とす速度

    // ── ボトルの色判定 ────────────────────────────────
    // 離れた位置から読むため反射光が弱く、実測で反射率2〜3・明度7〜9しかない。それでも彩度88〜91・
    // 色相210〜218と色味は安定して出るので、彩度がこれ以上あればHueを信用する
    constexpr int kBottleMinSaturation = 40;
    // 走行全体の目標本数を決める判定なので、1サンプルでは決めない。停止・整定した状態で読むため
    // 連続一致で確定できるはず。決まらないまま走り出すより止まる方が安全
    // アームを上げた直後の整定待ち。confirmBottleColor()が5回連続一致を要求し、決まらなければ
    // 1秒まで読み直すので、長い固定待ちは不要。車体の揺れが収まる分だけ置く
    constexpr int kArmSettleBeforeColorMs = 200;
    constexpr int kBottleColorStableCount = 5;
    // 色が読めなかったとき、アームを下げて少し前に出ながら上げ直し、読めるまで繰り返す。
    // 角度と距離の組み合わせでセンサーの当たる位置が変わるので、前に出るたびに読める可能性がある
    constexpr int kBottleRetryAdvanceMm = 5;
    constexpr int kBottleRetryAdvanceSpeedDegPerSec = 200;  // ボトルの目の前なのでゆっくり
    constexpr int kBottleRetryExtraArmDeg = 2;              // 角度不足で読めないことがあるので、再試行ごとに上げ角を足す
    // 保険。前に出続けるとボトルに当たるため、この回数で打ち切る（5mm×5回＝25mm）
    constexpr int kBottleRetryMaxCount = 5;
    constexpr int kBottleColorSampleIntervalMs = 20;
    constexpr int kBottleColorTimeoutMs = 1000;

    // ── アームを下げた直後の直進（これだけでラインへ復帰させる）──
    constexpr int kAfterArmStraightLeftPwm = 35;
    constexpr int kAfterArmStraightRightPwm = 40;
    constexpr float kAfterArmStraightSec = 1.0f;

    // ── 青ライン判定 ──────────────────────────────────
    // 白黒のグラデーション帯は色相が青寄り(実測195〜215度)で、彩度がときどき40〜56まで跳ねるため
    // ColorJudgeでは青と判定される。実測の本物の青ラインは彩度70〜88あるので、彩度で切り分ける
    constexpr int kBlueLineMinSaturation = 65;
    // 確定はサンプル数ではなくms基準にする。制御周期が変わっても意図した時間幅を保つため
    // 実測: コーナー後の青は条件を満たす最大連続が12〜20サンプルしかない。100ms(10サンプル)だと
    // マージンが2〜3しか残らないため70msまで詰める。誤検知は彩度ゲート(kBlueLineMinSaturation)で切る
    constexpr int kBlueEntryConfirmMs = 70;    // 青に乗ったと確定するまでの時間
    constexpr int kBluePassedConfirmMs = 400;  // 青を通過した（完全に降りた）と確定するまでの時間
    // エリアへ向かう最後の1本だけ短くする。300msだと確定までに約60mm進み、入口ではなく出口で抜けてしまうため
    constexpr int kBlueFinalEntryConfirmMs = 50;

    // ── 青1本目の後の移動（右エッジへの持ち替え） ──────────────
    // 基準角度から80度へのturnByImuは約1秒かかり、青の上で膨らんだ分も戻せなかった。弧を描いて線を横切り、
    // 黒を踏んだ時点で止める。時間だけで止めると、青を降りた位置のばらつきがそのまま残る。Lコース基準
    constexpr int kAfterBlue1OuterPwm = 50;
    constexpr int kAfterBlue1InnerPwm = 30;
    constexpr int kAfterBlue1MoveMs = 450;          // 上限。実測では143〜187msで黒を踏む
    constexpr int kAfterBlue1MinMoveMs = 100;       // 動き出しで左エッジの黒を拾って即終了しないための下限
    constexpr int kAfterBlue1BlackReflection = 35;  // 黒15/青37/グラデーション43〜55
    constexpr int kAfterBlue1BlackRunCount = 2;

    // ── 青1本目での線の踏み越え ─────────────────────────────
    // 青の手前の曲線は成功時でも白が最大91回続くので判定に使えない。青1本目の上なら成功時の白の連続は
    // 11〜15回、踏み越えた3回は35回以上と分かれる。踏み越えると左エッジのTracerは白を見て右へ切り、線から離れ続ける
    constexpr int kBlue1OvershootWhiteRunCount = 20;
    // 踏み越えたら元の方式（基準角度から80度へ旋回→右エッジ）に切り替える。
    // 内側へピボットして線を探す方式は、線を見つけても向きが行き過ぎて見失った
    constexpr float kBlue1OvershootTargetHeadingDeg = 80.0f;
    // 踏み越えた位置によっては、旋回後に青1本目をもう一度跨ぐ。2本目と数えるとエリアの本数がずれるので、
    // この間は青を数えず、明けたら1本目を通過済みとして揃える。コーナー後に青1本目は無いので、曲がりきったら早めに明ける
    constexpr int kBlue1OvershootBlueIgnoreMs = 5000;

    // ── 行きの90度コーナー手前の減速 ─────────────────────────
    // TRACER_PWM(90)のままピボットに入るとボトルを落とす。トレース開始からコーナー検知までは
    // 1194〜1226mm（3回）と安定しているので、距離で手前から落とす。検知は線の終わりの約40mm先
    constexpr int kCornerTracePwm = 100;         // 手前で減速する前提なので、そこまでは速く走る
    constexpr int kCornerSlowdownStartMm = 950;  // 実機調整。線の終わりの約210mm手前
    // 踏み越え時は80度旋回の後から測るので、起点が通常時とずれる。通常時より早めに落とす
    constexpr int kCornerSlowdownStartMmAfterOvershoot = 800;
    constexpr int kCornerApproachPwm = 65;  // 行き・帰りのコーナー手前の減速後の速度

    // ── 帰りの90度コーナーの後 ────────────────────────────
    constexpr int kReturnAfterCornerFastPwm = 100;
    constexpr int kReturnFinishAfterCornerMm = 900;  // 曲がりきってからこの距離を走ったら終了し、ラリーへ引き渡す

    // ── 帰りの90度コーナーの手前 ──────────────────────────
    // 帰りのトレース開始からコーナー検知までは 黄467〜473 / 青777 / 赤1064〜1089mm で、色が1つ変わるごとに約305mm。
    // 時間で判定を始めるとエリア付近のトレースの揺れで白が9回続き、黄では356mmで誤検知したので、距離で始める
    constexpr int kReturnCornerDetectStartMmYellow = 400;    // 誤検知した356mmより後ろ、線の終わり(約430mm)より手前
    constexpr int kReturnCornerSlowdownStartMmYellow = 270;  // 実機調整。線の終わりの約160mm手前
    constexpr int kReturnCornerColorStepMm = 305;            // 黄→青→赤で1段ずつ遠くなる
    constexpr int kReturnTracePwm = 100;                     // 減速位置まで。手前で減速する前提なので速く走る

    // ── エリアに入る青の手前 ─────────────────────────────
    // 速いまま斜め移動に入るとボトルを落とす。行きのコーナーを曲がりきってからエリアに入る青までは
    // 黄450〜452 / 赤1023〜1026mm（各3回）で、青2〜4本目の間隔は約287mm（青は青3本目の約735mm）
    constexpr int kAreaSlowdownStartMmYellow = 240;  // 実機調整。青の約210mm手前
    constexpr int kAreaBlueColorStepMm = 287;
    constexpr int kAreaApproachPwm = 65;

    // ── 直角コーナー対策 ───────────────────────────────
    // Tracerのカーブ減速はEMA(TRACER_CURVE_TURN_FILTER_ALPHA=0.05 → 時定数約200ms)で
    // ステップ状の変化には間に合わないため、白の連続で「線を見失った」を検知しピボット旋回で曲がり直す
    constexpr int kCornerWhiteReflection = 85;  // これ以上を白とみなす（実測の白は約99）
    constexpr int kCornerBlackReflection = 35;  // これ以下を黒（実測 黒15/青37）。45では境目のグラデーション(43〜55)を誤検知した
    // 直線部でも白は6〜7回連続する（TRACER_PWM=80・旧PIDでの実測）。10では帰りの手前で9まで来て誤検知もしたので12。
    // コーナー手前で減速するので、検知が2サンプル遅れても行き過ぎは小さい。Tracerの設定を触ったら測り直すこと
    constexpr int kCornerWhiteRunCount = 12;

    constexpr int kCornerPivotOuterPwm = 57;       // 両輪逆転は回転ジャークでボトルを落とすため片輪駆動。強すぎると線を踏み抜く
    constexpr int kCornerPivotInnerPwm = 0;        // 0で片輪旋回。負にすると半径は縮むがボトルへの負荷が増える
    constexpr int kCornerPivotRampLoopCount = 15;  // 立ち上がりでボトルを振らないよう150msかけて上げる
    constexpr int kCornerPivotBlackRunCount = 3;   // 1サンプルのノイズで抜けないための連続回数
    // 行き過ぎた直後は元の線の方が近く先に当たる。元の線だと18度で終わるが正解時は72〜86度なので40度で分離できる
    constexpr float kCornerPivotMinTurnDeg = 40.0f;
    // 復路は事情が違う。実測では旋回開始から40度に達する前に線を横切っており（黒17〜21サンプル）、
    // 40度のゲートがその線を握りつぶして120度まで空回りしていた。ゲートを外して最初の交差で拾う。
    // 掴んだのがコーナー手前の線でも正味70度に届かずREACQUIREDになるので、Tracerが進んでから再挑戦になる
    constexpr float kCornerPivotReturnMinTurnDeg = 0.0f;
    // 線がどこにあっても減速済みで到達するよう、ゲートが開く角度から線形に出力を落とす
    constexpr float kCornerPivotTaperStartDeg = 40.0f;  // ゲートとは独立。ゲートを下げても減速の形は変えない
    constexpr float kCornerPivotTaperEndDeg = 100.0f;
    constexpr float kCornerPivotTaperMinRatio = 0.6f;  // 57×0.6≒34。低すぎると動かなくなる
    constexpr float kCornerDoneTurnDeg = 70.0f;        // これだけ回って復帰できたら曲がりきったとみなし、以降の判定を止める
    constexpr float kCornerPivotMaxTurnDeg = 120.0f;   // 暴走を止める保険
    constexpr int kCornerPivotTimeoutLoopCount = 300;  // 保険その2（10ms周期なので3秒）
    constexpr int kCornerSuppressAfterBlueMs = 600;    // 青を跨ぐとき脇の白を踏むため、青の上と直後は判定を止める
    constexpr int kCornerRetryAfterFailMs = 1500;      // 線を見つけられなかったときに次の検知まで置く間隔
    constexpr int kCornerConfirmTimeoutMs = 1500;      // 線に復帰した後、Tracerが曲がりきるのを待つ上限
    constexpr int kCornerSuppressAfterBlueCount = (kCornerSuppressAfterBlueMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    constexpr int kCornerRetryAfterFailCount = (kCornerRetryAfterFailMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    constexpr int kCornerConfirmTimeoutCount = (kCornerConfirmTimeoutMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    // ── エリア配置 ────────────────────────────────────
    constexpr int kDiagonalPwmHigh = 85;  // 斜め移動の外輪PWM
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
    constexpr float kSearchMaxTurnDeg = 180.0f;   // これ以上回っても線から離れるだけなので打ち切る
    constexpr int kSearchTimeoutLoopCount = 500;  // 保険（10ms周期なので5秒）
    constexpr int kSearchFoundPwmLeft = 30;       // 線を見つけた瞬間に踏み込む左右PWM
    constexpr int kSearchFoundPwmRight = 90;
    constexpr float kSearchFoundSec = 0.10f;

    // 診断ログ用。区間ごとの所要時間を出すため
    int nowMs() {
        SYSTIM now;
        get_tim(&now);
        return static_cast<int>(now / 1000);
    }

}  // namespace

DeliveryTask::DeliveryTask(Robot& robot)
    : robot(robot) {
}

// ボトルの色判定。ColorJudge::judge()は反射率20未満または明度30未満だとHueを信用せず無彩色に倒すが、
// これは黒ラインを青と誤検知しないための対策で、離れたボトルを読むこの場面には合わない
// （実測の反射率2〜3・明度7〜9は常にこの条件に掛かり、必ず黒＝UNKNOWNになってしまう）。
// ここは彩度とHueだけで判定し、候補もデリバリーで使う赤・黄・青の3色に絞る
ColorJudge::Color DeliveryTask::judgeBottleColor() const {
    ColorJudge::Reading reading = robot.getColorReading();

    const ColorJudge::Color candidates[] = { ColorJudge::Color::RED, ColorJudge::Color::YELLOW, ColorJudge::Color::BLUE };
    const int candidateHues[] = { Config::COLOR_RED_HUE, Config::COLOR_YELLOW_HUE, Config::COLOR_BLUE_HUE };
    const int candidateCount = static_cast<int>(sizeof(candidateHues) / sizeof(candidateHues[0]));

    int minDist = 360;
    ColorJudge::Color nearest = ColorJudge::Color::UNKNOWN;
    for(int i = 0; i < candidateCount; i++) {
        // Hueは円環なので最短距離で比べる
        int diff = std::abs(static_cast<int>(reading.hsv.h) - candidateHues[i]);
        if(diff > 180) {
            diff = 360 - diff;
        }
        if(diff < minDist) {
            minDist = diff;
            nearest = candidates[i];
        }
    }

    lastBottleReading = reading;  // 確定後にまとめてログへ出すために持っておく

    if(reading.hsv.s < kBottleMinSaturation || minDist > Config::COLOR_HUE_TOLERANCE) {
        return ColorJudge::Color::UNKNOWN;
    }
    return nearest;
}

// アームを上げた直後に色を読む一式。整定待ち→角度と生値のログ→確定
ColorJudge::Color DeliveryTask::readBottleColorAfterRaise() {
    dly_tsk(kArmSettleBeforeColorMs * 1000);
    // 診断: 色を読む瞬間のアーム角度。raiseArm()が止めた角度と比べ、保持中に動いていないかを見る
    syslog(LOG_NOTICE, "Arm angle at color reading: %d deg", std::abs(robot.getArmCount()));

    ColorJudge::Color color = confirmBottleColor();
    syslog(LOG_NOTICE, "Bottle reading: h=%d s=%d v=%d refl=%d", (int)lastBottleReading.hsv.h, (int)lastBottleReading.hsv.s, (int)lastBottleReading.hsv.v, lastBottleReading.reflection);
    return color;
}

// ボトル色を連続一致で確定させる。決まらなければUNKNOWNを返す
ColorJudge::Color DeliveryTask::confirmBottleColor() {
    const int intervalUs = kBottleColorSampleIntervalMs * 1000;
    const int maxSamples = kBottleColorTimeoutMs / kBottleColorSampleIntervalMs;

    ColorJudge::Color stableColor = ColorJudge::Color::UNKNOWN;
    int matchedCount = 0;

    for(int i = 0; i < maxSamples; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            return ColorJudge::Color::UNKNOWN;
        }

        ColorJudge::Color sampled = judgeBottleColor();
        if(sampled != ColorJudge::Color::UNKNOWN && sampled == stableColor) {
            matchedCount++;
            if(matchedCount >= kBottleColorStableCount) {
                return stableColor;
            }
        } else {
            stableColor = sampled;
            matchedCount = 1;
        }
        dly_tsk(intervalUs);
    }
    return ColorJudge::Color::UNKNOWN;
}

// 青ラインかどうかの1サンプル判定。ColorJudge::judge()をそのまま使うと、ライントレースが追う
// 白黒の境界（グラデーション帯）を青と誤判定する。明度95〜100・反射率37〜58と明るいため、
// ColorJudgeの暗所ガード（黒を青と誤検知しない対策）には掛からない
bool DeliveryTask::isBlueReading(BlueStats& stats) const {
    ColorJudge::Reading reading = robot.getColorReading();

    int hueDiff = std::abs(static_cast<int>(reading.hsv.h) - Config::COLOR_BLUE_HUE);
    if(hueDiff > 180) {
        hueDiff = 360 - hueDiff;
    }
    bool inBlueHue = (hueDiff <= Config::COLOR_HUE_TOLERANCE);
    bool isBlue = inBlueHue && (reading.hsv.s >= kBlueLineMinSaturation);

    // 診断。彩度が惜しかったのか、そもそも青の色相すら見えていないのかを後から切り分ける
    if(inBlueHue && reading.hsv.s > stats.maxSaturationInHue) {
        stats.maxSaturationInHue = reading.hsv.s;
    }
    if(isBlue) {
        stats.satisfiedCount++;
        stats.currentRun++;
        if(stats.currentRun > stats.maxConsecutive) {
            stats.maxConsecutive = stats.currentRun;
        }
    } else {
        stats.currentRun = 0;
    }
    return isBlue;
}

// Robot::isOnColors()と同じ「連続一致回数」方式。カウンタは呼び出し側で持つ
bool DeliveryTask::isOnBlueLine(int& matchedCount, int stableCount, BlueStats& stats) const {
    if(isBlueReading(stats)) {
        matchedCount++;
    } else {
        matchedCount = 0;
    }
    return (matchedCount >= stableCount);
}

void DeliveryTask::logBlueStats(const char* label, const BlueStats& stats) const {
    syslog(LOG_NOTICE, "Blue stats[%s]: maxSat %d, satisfied %d, maxRun %d", label, stats.maxSaturationInHue, stats.satisfiedCount, stats.maxConsecutive);
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
            aborted = true;
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
            aborted = true;
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
            aborted = true;
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
            aborted = true;
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
                    aborted = true;
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
    int firstBlackAfterGateDeg = -1;  // ゲート後に初めて黒判定になった角度（線に触れた場所）
    int minReflectionAfterGate = 101;
    int minReflectionAtDeg = -1;
    int maxBlackRun = 0;  // 成功して抜けた分は含めない（途中で線に触れて連続条件に届かなかったかを見るため）

    for(int loopCount = 0; loopCount < kCornerPivotTimeoutLoopCount; loopCount++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            robot.stop();
            return false;
        }

        float turnedDeg = std::fabs(robot.getImuHeading() - startHeading);
        turnedDegOut = turnedDeg;

        int reflection = robot.getReflection();
        if(reflection <= kCornerBlackReflection) {
            blackRun++;
        } else {
            // 黒が途切れた時点で記録する。成功時はこの分岐を通らずに抜けるので、最大値に混ざらない
            if(blackRun > maxBlackRun) {
                maxBlackRun = blackRun;
            }
            blackRun = 0;
        }
        if(turnedDeg >= minTurnDeg && reflection >= kCornerWhiteReflection) {
            seenWhiteAfterGate = true;
        }

        if(turnedDeg >= minTurnDeg) {
            if(firstWhiteAfterGateDeg < 0 && reflection >= kCornerWhiteReflection) {
                firstWhiteAfterGateDeg = static_cast<int>(turnedDeg);
            }
            if(firstBlackAfterGateDeg < 0 && reflection <= kCornerBlackReflection) {
                firstBlackAfterGateDeg = static_cast<int>(turnedDeg);
            }
            if(reflection < minReflectionAfterGate) {
                minReflectionAfterGate = reflection;
                minReflectionAtDeg = static_cast<int>(turnedDeg);
            }
        }

        if(seenWhiteAfterGate && blackRun >= kCornerPivotBlackRunCount) {
            robot.stop();
            syslog(LOG_NOTICE, "Pivot done: line found (reflection %d, turned %d deg)", reflection, (int)turnedDeg);
            syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, firstBlack %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, firstBlackAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
            return true;
        }

        if(turnedDeg >= maxTurnDeg) {
            robot.stop();
            syslog(LOG_NOTICE, "STOP[pivotUntilLineFound]: MAX_TURN (reflection %d)", reflection);
            syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, firstBlack %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, firstBlackAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
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
    syslog(LOG_NOTICE, "Pivot stats: firstWhite %d deg, firstBlack %d deg, minRefl %d @ %d deg, maxBlackRun %d", firstWhiteAfterGateDeg, firstBlackAfterGateDeg, minReflectionAfterGate, minReflectionAtDeg, maxBlackRun);
    return false;
}

int DeliveryTask::wheelDistanceMm() const {
    const int avgDeg = (robot.getLeftMotorCount() + robot.getRightMotorCount()) / 2;
    return static_cast<int>(avgDeg * M_PI * Config::WHEEL_RADIUS_MM / 180.0f);
}

// 曲がるべき向きを正とした、startHeadingからの回頭量。getImuHeading()は右回りが正なので、
// 左折なら符号を反転する。逆方向へ回った分はマイナスになる
float DeliveryTask::signedTurnFrom(float startHeading, bool isLeftTurn) const {
    float delta = robot.getImuHeading() - startHeading;
    return isLeftTurn ? -delta : delta;
}

// コーナーを検知した後の旋回。線を捕まえ直し、失敗したら逆へ振り戻す。行き（左折）と帰り（右折）で共用する。
// 完了判定は「正方向の旋回量 - 振り戻し量」ではなく、startHeadingからの実方位差で行う。
// 正方向と振り戻しでは軸にする車輪が変わって旋回中心が別物になるうえ、停止後の惰性も引き算には入らないため
DeliveryTask::CornerResult DeliveryTask::turnAtCorner(bool isLeftTurn, float startHeading, float minTurnDeg) {
    float pivotTurnedDeg = 0.0f;
    bool lineFound = pivotUntilLineFound(isLeftTurn, kCornerPivotOuterPwm, kCornerPivotInnerPwm, minTurnDeg, kCornerPivotMaxTurnDeg, pivotTurnedDeg);

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

    // fabsで測ると、振り戻しが開始角度を越えて逆方向へ大きく回った場合も完了扱いになってしまう。
    // 曲がるべき向きを正とした符号付きの角度で判定する
    float turnedDeg = signedTurnFrom(startHeading, isLeftTurn);
    return (turnedDeg >= kCornerDoneTurnDeg) ? CornerResult::CLEARED : CornerResult::REACQUIRED;
}

// コーナー検知の1周期分。ライントレースのループから毎周期呼ぶ。
// 行き・帰りで状態を別に持つだけで、判定そのものは共通
void DeliveryTask::updateCornerDetection(CornerState& state, bool isLeftTurn, float minTurnDeg, Tracer& tracer, bool isOnBlue, const char* label) {
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
        syslog(LOG_NOTICE, "%s detection ENABLED. t=%d ms (+%d ms / +%d mm since trace start)", label, nowMs(), nowMs() - state.armedMs, wheelDistanceMm() - state.armedMm);
    }

    if(!state.enabled || state.suppressCount > 0) {
        return;
    }

    int reflection = robot.getReflection();

    // 線には復帰したが曲がりきってはいない状態。残りはTracerに任せ、方位が届いたら完了とする。
    // ただし線を見失ったまま旋回しているだけでも方位は増えるので、ライン上にいることを条件にする
    // （実測: 反射率99のまま方位だけ70度回り、コーナー未通過なのに完了扱いになっていた）。
    // 検知は止めずに続ける。ここで止めると、この後に本物のコーナーが来ても何も働かない
    if(state.confirming) {
        float turnedDeg = signedTurnFrom(state.startHeading, isLeftTurn);
        if(turnedDeg >= kCornerDoneTurnDeg && reflection < kCornerWhiteReflection) {
            state.confirming = false;
            state.enabled = false;
            state.done = true;
            tracer.setPwm(Config::TRACER_PWM);
            syslog(LOG_NOTICE, "%s cleared by tracer t=%d ms (%d deg, reflection %d). Detection DISABLED.", label, nowMs(), (int)turnedDeg, reflection);
            return;
        }
        if(state.loopCount >= state.confirmDeadlineLoop) {
            state.confirming = false;
            syslog(LOG_NOTICE, "%s NOT cleared after wait (%d deg). Keep detecting.", label, (int)turnedDeg);
        }
    }

    if(reflection >= kCornerWhiteReflection) {
        state.whiteRun++;
        state.measureWhiteRun++;
        if(reflection < state.minReflectionInWhiteRun) {
            state.minReflectionInWhiteRun = reflection;
        }
    } else {
        // 白が途切れた時点で記録する。しきい値に達して検知に使われた分は下で0に戻すので混ざらない
        if(state.measureWhiteRun > state.maxWhiteRunBeforeTrigger) {
            state.maxWhiteRunBeforeTrigger = state.measureWhiteRun;
        }
        state.measureWhiteRun = 0;
        state.whiteRun = 0;
        state.minReflectionInWhiteRun = 101;
    }
    if(state.whiteRun < kCornerWhiteRunCount) {
        return;
    }
    state.measureWhiteRun = 0;

    syslog(LOG_NOTICE, "%s detected t=%d ms (reflection %d, darkest in run %d). Pivoting.", label, nowMs(), reflection, state.minReflectionInWhiteRun);
    // 検知は線の終わりから白kCornerWhiteRunCount回分だけ遅れる。減速位置はこの値から手前に取る
    syslog(LOG_NOTICE, "%s detected: +%d ms / +%d mm since trace start", label, nowMs() - state.armedMs, wheelDistanceMm() - state.armedMm);
    state.whiteRun = 0;
    state.minReflectionInWhiteRun = 101;
    state.startHeading = robot.getImuHeading();
    state.suppressCount = kCornerSuppressAfterBlueCount;

    CornerResult result = turnAtCorner(isLeftTurn, state.startHeading, minTurnDeg);
    float turnedDeg = signedTurnFrom(state.startHeading, isLeftTurn);

    if(result == CornerResult::CLEARED) {
        // カーブは1周に1つしかないので、通過後の検知は誤検知のリスクにしかならない
        state.enabled = false;
        state.done = true;
        tracer.setPwm(Config::TRACER_PWM);
        syslog(LOG_NOTICE, "%s cleared t=%d ms (%d deg). Detection DISABLED.", label, nowMs(), (int)turnedDeg);
    } else if(result == CornerResult::REACQUIRED) {
        // enabledは落とさない。Tracerが曲がりきるのを待ちつつ、再び線を見失ったら検知し直す
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
    aborted = false;

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
            aborted = true;
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
    // 診断: 以降の角度はすべてこの基準からのズレで出す。接近中に曲がっていると基準自体がずれるので、停止時点との差も見る
    syslog(LOG_NOTICE, "[Heading] baseline %d (0.1deg, %d samples), at bottle stop %d from baseline", (int)(baselineHeading * 10.0f), headingSampleCount, (int)((robot.getImuHeading() - baselineHeading) * 10.0f));

    // 2. ボトルの前でアームを上げる（Robotクラスに移譲）
    int armExtraDeg = 0;  // 上げ角の上乗せ分。下げるときも同じだけ下げる
    robot.raiseArm(armExtraDeg);

    // 3. ボトルの色を判定。読めなければアームを下げ、少し前に出ながら上げ直して読み直す
    ColorJudge::Color bottleColor = readBottleColorAfterRaise();
    for(int retry = 1; bottleColor == ColorJudge::Color::UNKNOWN && !aborted && retry <= kBottleRetryMaxCount; retry++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }
        robot.lowerArm(armExtraDeg);
        armExtraDeg += kBottleRetryExtraArmDeg;
        syslog(LOG_NOTICE, "Bottle color unknown. Retry %d/%d: advance %dmm while raising to %d deg", retry, kBottleRetryMaxCount, kBottleRetryAdvanceMm, Config::ARM_RAISE_DEG + armExtraDeg);
        robot.raiseArmWhileDriving(kBottleRetryAdvanceMm, kBottleRetryAdvanceSpeedDegPerSec, armExtraDeg);
        bottleColor = readBottleColorAfterRaise();
    }
    if(aborted) {
        robot.stop();
        syslog(LOG_NOTICE, "ABORTED during bottle color judgement. Stopping.");
        return;
    }
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
            // 再試行しても読めなかった。目標本数が決まらないまま走ると、青1本目でエリアへ向かうなど
            // 的外れな動きになるので、走らずに止めて人が気づけるようにする
            syslog(LOG_NOTICE, "Bottle Color: UNKNOWN after %d retries. Stopping.", kBottleRetryMaxCount);
            robot.beep(500);
            robot.stop();
            return;
    }

    // 4. アームを下げる（Robotクラスに移譲）
    robot.lowerArm(armExtraDeg);

    // 5. 左35・右40のパワーでkAfterArmStraightSec秒直進してラインに復帰する（蛇行探索より速く、実機ではこれで十分だった）
    int afterArmStraightLoopCount = static_cast<int>(kAfterArmStraightSec * 1000 * 1000 / Config::MOTION_POLL_INTERVAL_US);
    for(int i = 0; i < afterArmStraightLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
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

    // 7. 0.5秒間、遅い速度でライントレース
    int slowTraceLoopCount = (1000 * 500) / Config::LINE_TRACE_POLL_INTERVAL_US;
    for(int i = 0; i < slowTraceLoopCount; i++) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            tracer.terminate();
            return;
        }
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    // 8. 通常速度に戻してライントレースを継続
    syslog(LOG_NOTICE, "Slow trace done. Switching to pwm %d.", kPostSlowTracePwm);
    tracer.setPwm(kPostSlowTracePwm);

    // 9. 指定回数青ラインを検知するまでライントレース
    syslog(LOG_NOTICE, "Tracing until blue line count: %d", targetBlueLineCount);

    int detectedBlueCount = 0;       // 青ラインを検知した回数
    bool isCurrentlyOnBlue = false;  // 現在青ライン上にいるかのフラグ

    int matchedBlueCount = 0;     // 青を連続で読んだ回数
    int matchedNonBlueCount = 0;  // 青以外を連続で読んだ回数

    // ms指定の確定時間を、現在のLINE_TRACE_POLL_INTERVAL_US(制御周期)でのサンプル回数に換算
    int blueEntryConfirmCount = (kBlueEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int blueFinalEntryConfirmCount = (kBlueFinalEntryConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;
    int bluePassedConfirmCount = (kBluePassedConfirmMs * 1000) / Config::LINE_TRACE_POLL_INTERVAL_US;

    // 青判定の診断。コーナー前の高彩度がコーナー後の情報を隠さないよう、区間を分けて集計する
    BlueStats blueStatsBeforeCorner;
    BlueStats blueStatsAfterCorner;

    // 診断: 向きは基準角度からのズレ[度]。Lコースの左カーブは負
    auto headingFromBaseline = [this, baselineHeading]() {
        return static_cast<int>(robot.getImuHeading() - baselineHeading);
    };

    // 直角コーナー対策の状態。手前での減速と同時に有効化する（下のisCornerSlowdownPendingの処理）
    CornerState outboundCorner;

    // 青1本目の上での踏み越え検知。診断用の集計とはカウンタを共用しない（共用して挙動を壊したことがあるため）
    int blue1OvershootWhiteRun = 0;
    bool isBlueIgnored = false;  // 踏み越え後、青1本目を二重に数えないための無視期間
    int blueIgnoreEndMs = 0;

    // コーナーを曲がりきった地点。エリアに入る青の手前での減速の起点。-1はまだ曲がりきっていない
    int outboundCornerClearedMs = 0;
    int outboundCornerClearedMm = -1;
    const int areaColorSteps = targetBlueLineCount - 2;  // 黄0・青1・赤2（エリアに入る青の本数から）
    const int areaSlowdownStartMm = kAreaSlowdownStartMmYellow + kAreaBlueColorStepMm * areaColorSteps;
    bool isAreaSlowdownPending = true;

    // コーナー手前の減速。トレース開始からの距離で落とす
    bool isCornerSlowdownPending = false;
    int cornerSlowdownStartMm = kCornerSlowdownStartMm;

    // 右エッジのトレースに切り替え、手前での減速を予約する（通常時・踏み越え時で共通）。
    // コーナー判定は減速するまで始めない（姿勢の乱れによる誤検知を避けるため）
    auto startRightEdgeTrace = [&](int slowdownStartMm) {
        tracer.setEdge(isLeftCourse ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);
        tracer.setPwm(kCornerTracePwm);
        isCornerSlowdownPending = true;
        cornerSlowdownStartMm = slowdownStartMm;
        outboundCorner.armedMs = nowMs();
        outboundCorner.armedMm = wheelDistanceMm();
        syslog(LOG_NOTICE, "Corner trace start t=%d ms", outboundCorner.armedMs);
    };

    while(true) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }

        const int reflection = robot.getReflection();

        // 青1本目の上で線を踏み越えたら、通過を待たずに基準角度から80度へ旋回して右エッジで進む
        if(detectedBlueCount == 1 && isCurrentlyOnBlue && !isBlueIgnored) {
            blue1OvershootWhiteRun = (reflection >= kCornerWhiteReflection) ? blue1OvershootWhiteRun + 1 : 0;
            if(blue1OvershootWhiteRun >= kBlue1OvershootWhiteRunCount) {
                const int detectedMs = nowMs();
                syslog(LOG_NOTICE, "[Overshoot] blue1: white %d samples t=%d ms, heading %d. Turning to baseline-80.", blue1OvershootWhiteRun, detectedMs, headingFromBaseline());

                isCurrentlyOnBlue = false;
                matchedBlueCount = 0;
                matchedNonBlueCount = 0;
                isBlueIgnored = true;
                blueIgnoreEndMs = detectedMs + kBlue1OvershootBlueIgnoreMs;

                float targetHeading = baselineHeading - kBlue1OvershootTargetHeadingDeg * courseSign;
                const float turnedDeg = robot.turnByImu(targetHeading - robot.getImuHeading(), Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);
                syslog(LOG_NOTICE, "[Overshoot] blue1: turned %d deg in %d ms, heading %d, reflection %d", (int)turnedDeg, nowMs() - detectedMs, headingFromBaseline(), robot.getReflection());
                if(robot.isCenterButtonPressed()) {
                    aborted = true;
                    break;
                }
                // 弧を描く移動は線の左側から始める前提なので行わない
                startRightEdgeTrace(kCornerSlowdownStartMmAfterOvershoot);
            }
        }

        if(isBlueIgnored) {
            if(nowMs() >= blueIgnoreEndMs || outboundCorner.done) {
                isBlueIgnored = false;
                detectedBlueCount = 1;
                isCurrentlyOnBlue = false;
                matchedBlueCount = 0;
                matchedNonBlueCount = 0;
                syslog(LOG_NOTICE, "[Overshoot] blue ignore end t=%d ms (%s). Blue count set to 1.", nowMs(), outboundCorner.done ? "corner cleared" : "timeout");
            }
        }

        // 無視期間中も、コーナー判定の抑制（青を跨ぐときの脇の白）には生の青判定を使う
        bool isOnBlueForCorner = isCurrentlyOnBlue;
        if(isBlueIgnored) {
            BlueStats& blueStats = outboundCorner.done ? blueStatsAfterCorner : blueStatsBeforeCorner;
            isOnBlueForCorner = isBlueReading(blueStats);
        } else if(!isCurrentlyOnBlue) {
            // まだ青ラインに乗っていない状態：青を探す。
            // エリアへ向かう最後の1本だけは、青の入口で抜けられるよう確定を早める
            bool isFinalBlue = (detectedBlueCount + 1 >= targetBlueLineCount);
            BlueStats& blueStats = outboundCorner.done ? blueStatsAfterCorner : blueStatsBeforeCorner;
            bool isOnBlueNow = isOnBlueLine(matchedBlueCount, isFinalBlue ? blueFinalEntryConfirmCount : blueEntryConfirmCount, blueStats);
            if(isOnBlueNow) {
                isCurrentlyOnBlue = true;
                matchedNonBlueCount = 0;  // 青以外カウントをリセット

                // 青を読んだ瞬間にカウントアップ！
                detectedBlueCount++;
                ColorJudge::Reading entryReading = robot.getColorReading();
                syslog(LOG_NOTICE, "Entered blue line. Count: %d / %d (h=%d s=%d v=%d)", detectedBlueCount, targetBlueLineCount, (int)entryReading.hsv.h, (int)entryReading.hsv.s, (int)entryReading.hsv.v);
                if(outboundCornerClearedMm >= 0) {
                    // 診断: エリアに入る青の手前で減速する位置を決めるため
                    syslog(LOG_NOTICE, "Blue %d entry: +%d ms / +%d mm since corner cleared", detectedBlueCount, nowMs() - outboundCornerClearedMs, wheelDistanceMm() - outboundCornerClearedMm);
                }
                if(detectedBlueCount == 1) {
                    // 青1本目の上に乗っている間（判定しなくなるまで）は速度を落とす
                    tracer.setPwm(kOnFirstBlueLinePwm);
                    blue1OvershootWhiteRun = 0;
                    syslog(LOG_NOTICE, "[Blue1] entry t=%d ms, heading %d", nowMs(), headingFromBaseline());
                }
                // 指定回数に達したら、その場ですぐに終了する
                if(detectedBlueCount >= targetBlueLineCount) {
                    syslog(LOG_NOTICE, "Target count reached! Stopping immediately.");
                    break;
                }
            }
            isOnBlueForCorner = isCurrentlyOnBlue;
        } else {
            // すでに青ラインに乗っている状態：青から降りる（青以外）のを探す
            BlueStats& blueStats = outboundCorner.done ? blueStatsAfterCorner : blueStatsBeforeCorner;
            if(!isBlueReading(blueStats)) {
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
                    syslog(LOG_NOTICE, "[Blue1] passed t=%d ms, heading %d", nowMs(), headingFromBaseline());

                    // 弧を描いて線を横切り、右エッジ側へ移る。黒を踏んだら止める
                    const float headingBefore = robot.getImuHeading();
                    const int leftPwm = isLeftCourse ? kAfterBlue1OuterPwm : kAfterBlue1InnerPwm;
                    const int rightPwm = isLeftCourse ? kAfterBlue1InnerPwm : kAfterBlue1OuterPwm;
                    const int moveLoopCount = (kAfterBlue1MoveMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
                    const int minMoveLoopCount = (kAfterBlue1MinMoveMs * 1000) / Config::MOTION_POLL_INTERVAL_US;
                    const int moveStartMs = nowMs();
                    const int startReflection = robot.getReflection();
                    int blackRun = 0;
                    const char* stopReason = "timeout";
                    for(int k = 0; k < moveLoopCount; k++) {
                        if(robot.isCenterButtonPressed()) {
                            aborted = true;
                            break;
                        }
                        blackRun = (robot.getReflection() <= kAfterBlue1BlackReflection) ? blackRun + 1 : 0;
                        if(k >= minMoveLoopCount && blackRun >= kAfterBlue1BlackRunCount) {
                            stopReason = "black";
                            break;
                        }
                        robot.setMotorPower(leftPwm, rightPwm);
                        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
                    }
                    if(aborted) {
                        break;
                    }
                    syslog(LOG_NOTICE, "[Blue1] move stop by %s at %d ms, reflection start %d / end %d, heading change %d", stopReason, nowMs() - moveStartMs, startReflection, robot.getReflection(), (int)(robot.getImuHeading() - headingBefore));

                    startRightEdgeTrace(kCornerSlowdownStartMm);
                }
            }
            isOnBlueForCorner = isCurrentlyOnBlue;
        }

        if(isCornerSlowdownPending) {
            if(outboundCorner.done) {
                isCornerSlowdownPending = false;  // 減速位置より手前で曲がりきった
            } else if(wheelDistanceMm() - outboundCorner.armedMm >= cornerSlowdownStartMm) {
                isCornerSlowdownPending = false;
                tracer.setPwm(kCornerApproachPwm);  // 曲がりきるとupdateCornerDetection()がTRACER_PWMに戻す
                // 減速したここから白の連続を数え始める
                outboundCorner.pending = true;
                outboundCorner.enableLoop = outboundCorner.loopCount;  // 次の周期から有効
                syslog(LOG_NOTICE, "Corner slowdown to pwm %d t=%d ms (+%d mm since trace start)", kCornerApproachPwm, nowMs(), wheelDistanceMm() - outboundCorner.armedMm);
            }
        }

        // Lコースでは左90度コーナーなので左へ回す
        updateCornerDetection(outboundCorner, isLeftCourse, kCornerPivotMinTurnDeg, tracer, isOnBlueForCorner, "Corner");
        if(outboundCorner.done && outboundCornerClearedMm < 0) {
            outboundCornerClearedMs = nowMs();
            outboundCornerClearedMm = wheelDistanceMm();
            syslog(LOG_NOTICE, "Corner cleared: distance origin for blue entries t=%d ms, area slowdown at %d mm", outboundCornerClearedMs, areaSlowdownStartMm);
        }
        if(outboundCornerClearedMm >= 0 && isAreaSlowdownPending && wheelDistanceMm() - outboundCornerClearedMm >= areaSlowdownStartMm) {
            isAreaSlowdownPending = false;
            tracer.setPwm(kAreaApproachPwm);
            syslog(LOG_NOTICE, "Area slowdown to pwm %d t=%d ms (+%d mm since corner cleared)", kAreaApproachPwm, nowMs(), wheelDistanceMm() - outboundCornerClearedMm);
        }

        // 青ライン上でも関係なく通常のライントレースを継続
        tracer.run();

        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    logBlueStats("before corner", blueStatsBeforeCorner);
    logBlueStats("after corner", blueStatsAfterCorner);
    syslog(LOG_NOTICE, "Outbound end: blueCount %d / %d, onBlue %d, corner done %d, reason %s", detectedBlueCount, targetBlueLineCount, isCurrentlyOnBlue ? 1 : 0, outboundCorner.done ? 1 : 0, aborted ? "ABORTED" : "reached");
    syslog(LOG_NOTICE, "Corner stats: maxWhiteRun(before trigger) %d / threshold %d", outboundCorner.maxWhiteRunBeforeTrigger, kCornerWhiteRunCount);
    if(aborted) {
        // 中断で抜けた場合にエリア配置へ進むと、各動作が即座に空振りして「到達した」ように見えてしまう
        robot.stop();
        syslog(LOG_NOTICE, "ABORTED during blue search. Stopping.");
        return;
    }
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

    if(aborted) {
        robot.stop();
        syslog(LOG_NOTICE, "ABORTED during area placement. Stopping.");
        return;
    }

    // 帰りの線探し（旋回方向はコース依存、反射率ベース）。見つけた瞬間の踏み込みもこの中で行う
    syslog(LOG_NOTICE, "Pivoting to find line by reflection");
    if(!pivotUntilReflectionBelow(kSearchReflectionThreshold, kSearchPwm, isLeftCourse)) {
        // 線が無い場所でTracerを起動すると白の上を暴走するだけなので、ここで打ち切る
        robot.stop();
        syslog(LOG_NOTICE, aborted ? "ABORTED during line search. Stopping." : "Line search FAILED. Aborting return trip.");
        return;
    }

    // 11. 左エッジでライントレースを再開
    syslog(LOG_NOTICE, "Resuming line trace on LEFT edge");
    tracer.setEdge(isLeftCourse ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    tracer.setPwm(kReturnTracePwm);  // 暗黙の値継承に頼らず明示する

    // 帰りは青で止めない（青の本数はエリアの青の上から走り出すとずれるため）。
    // 90度コーナーを曲がりきってから一定距離を走った時点で終了し、ラリーへ引き渡す
    syslog(LOG_NOTICE, "Return: finish %d mm after the corner is cleared", kReturnFinishAfterCornerMm);

    BlueStats returnBlueStats;
    CornerState returnCorner;
    // 帰りにも直角コーナーが1つある。行きは左折だったが帰りは右折になる（Lコース基準）。
    // 判定の開始と減速は、トレース開始からの距離で行う（位置はボトル色で決まる）
    returnCorner.armedMs = nowMs();
    returnCorner.armedMm = wheelDistanceMm();
    const int returnColorSteps = (bottleColor == ColorJudge::Color::RED) ? 2 : (bottleColor == ColorJudge::Color::BLUE) ? 1
                                                                                                                        : 0;
    const int returnCornerDetectStartMm = kReturnCornerDetectStartMmYellow + kReturnCornerColorStepMm * returnColorSteps;
    const int returnCornerSlowdownStartMm = kReturnCornerSlowdownStartMmYellow + kReturnCornerColorStepMm * returnColorSteps;
    bool isReturnCornerDetectStartPending = true;
    bool isReturnCornerSlowdownPending = true;
    syslog(LOG_NOTICE, "Return trace start t=%d ms: slowdown at %d mm, corner detection at %d mm", returnCorner.armedMs, returnCornerSlowdownStartMm, returnCornerDetectStartMm);

    // 曲がりきった地点。終了までの距離の起点。-1はまだ曲がりきっていない
    int returnCornerClearedMm = -1;

    while(true) {
        if(robot.isCenterButtonPressed()) {
            aborted = true;
            break;
        }

        const int returnTracedMm = wheelDistanceMm() - returnCorner.armedMm;
        if(isReturnCornerSlowdownPending && returnTracedMm >= returnCornerSlowdownStartMm) {
            isReturnCornerSlowdownPending = false;
            if(!returnCorner.done) {
                tracer.setPwm(kCornerApproachPwm);  // 曲がりきると下でkReturnAfterCornerFastPwmに上書きする
                syslog(LOG_NOTICE, "Return corner slowdown to pwm %d t=%d ms (+%d mm since trace start)", kCornerApproachPwm, nowMs(), returnTracedMm);
            }
        }
        if(isReturnCornerDetectStartPending && returnTracedMm >= returnCornerDetectStartMm) {
            isReturnCornerDetectStartPending = false;
            returnCorner.pending = true;
            returnCorner.enableLoop = returnCorner.loopCount;  // 次の周期から有効
        }

        // コーナー判定の抑制（青を跨ぐときの脇の白）にだけ青判定を使う。終了条件には使わない
        bool isOnBlueForCorner = false;
        if(!returnCorner.done) {
            isOnBlueForCorner = isBlueReading(returnBlueStats);
        }

        // 帰りはLコースで右90度コーナーなので、行きとは逆の右へ回す
        updateCornerDetection(returnCorner, !isLeftCourse, kCornerPivotReturnMinTurnDeg, tracer, isOnBlueForCorner, "Return corner");

        // updateCornerDetection()は曲がりきるとTRACER_PWMに戻すので、その直後に上書きする
        if(returnCorner.done && returnCornerClearedMm < 0) {
            returnCornerClearedMm = wheelDistanceMm();
            tracer.setPwm(kReturnAfterCornerFastPwm);
            syslog(LOG_NOTICE, "Return after corner: pwm %d for %d mm, then finish", kReturnAfterCornerFastPwm, kReturnFinishAfterCornerMm);
        }
        if(returnCornerClearedMm >= 0 && wheelDistanceMm() - returnCornerClearedMm >= kReturnFinishAfterCornerMm) {
            syslog(LOG_NOTICE, "Return finished: +%d mm since corner cleared, +%d mm since trace start. Finishing DeliveryTask.", wheelDistanceMm() - returnCornerClearedMm, wheelDistanceMm() - returnCorner.armedMm);
            break;
        }

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    robot.stop();
    logBlueStats("return", returnBlueStats);
    syslog(LOG_NOTICE, "Return end: corner done %d", returnCorner.done ? 1 : 0);
    syslog(LOG_NOTICE, "Corner stats: maxWhiteRun(before trigger) %d / threshold %d", returnCorner.maxWhiteRunBeforeTrigger, kCornerWhiteRunCount);
    syslog(LOG_NOTICE, aborted ? "--- DeliveryTask ABORTED ---" : "--- DeliveryTask Finished ---");
}
