#ifndef EARLY_EDGE_HANDOFF_H_
#define EARLY_EDGE_HANDOFF_H_

/**
 * 青1本目の通過を待たずに、曲線終盤で反対エッジへ持ち替えるための状態機械。
 * ハードウェアにも時計にも触らず、値を渡されて判断するだけなのでホストで試験できる。
 *
 * 角度は「ボトル接近中に求めた基準方位からの進み」を0.1度単位で渡すこと
 * （progressDeg10 = -(heading - baselineHeading) * courseSign * 10）。
 * IMUの生の絶対角度でも、曲線開始からの角度でもない。
 *
 * 距離は曲線開始からの車輪距離[mm]。
 *
 * 受付は距離の下限ではなく「曲線後半へ減速済み」で開く（curveSlowed）。上限だけ距離で持つ。
 *
 * LEGACY: 何もしない。OBSERVE_ONLY: 候補の成立だけを記録し、制御は変えない。
 * EARLY: 実際に持ち替えへ進む。
 */
class EarlyEdgeHandoff {
public:
    enum class Mode { LEGACY,
                      OBSERVE_ONLY,
                      EARLY };

    enum class Phase { CURVE,      // 曲線追従中。候補を探している
                       TRANSFER,   // 横切り移動中
                       ACQUIRE,    // 反対エッジで追従の成立を確認中
                       FAST,       // 高速へ移った
                       FAILED };   // 上限に当たった・証拠が取れなかった

    enum class Step { NONE,
                      CANDIDATE_READY,  // 候補成立（OBSERVE_ONLY。制御は変えない）
                      START_TRANSFER,   // 横切り移動を始める
                      CROSSED,          // 横断候補が成立した
                      ABORT };          // 上限・証拠不足で打ち切る

    enum class Missed { NONE,
                        ALREADY_PAST_AT_OPEN,  // 受付を開いた時点で既に角度を越えていた
                        WINDOW_PASSED };       // 受付範囲を出るまで成立しなかった

    enum class AbortReason { NONE,
                             DISTANCE,
                             TIME,
                             HEADING,
                             NO_EVIDENCE };

    // すべて初期比較値。実機で確認するまで確定値として扱わないこと
    struct Limits {
        int startProgressDeg10 = 700;  // 持ち替えを始める進み角[0.1度]
        int candidateRunCount = 5;     // 角度がその値以上で連続成立する回数（一瞬の越えで始めない）
        // 受付の下限は距離ではなく、呼び出し側が渡す「曲線後半へ減速済み」。
        // 固定600mmを下限にしていた頃は、その手前で角度に達した走行を毎回取りこぼしていた
        int acceptEndMm = 1100;  // 曲線開始からの受付上限
        int crossMaxMm = 60;           // 横切り移動の上限
        int crossMaxMs = 400;
        int crossHeadingLimitDeg10 = 100;  // 移動中の方位逸脱の上限[0.1度]
        int darkReflection = 35;           // これ以下を黒帯とみなす
        int brightReflection = 60;         // 黒帯を抜けたとみなす明るさ
        int darkRunCount = 3;              // それぞれの連続回数
        int brightRunCount = 3;
        int minCrossMm = 10;            // 横断成立に必要な最低前進量
        int minCrossMmStartedDark = 25;  // 開始時に黒だった場合の最低前進量（黒1点を証拠にしない）
    };

    EarlyEdgeHandoff(Mode mode, const Limits& limits) : mode(mode), limits(limits) {}

    // 曲線追従中の1周期。候補の成立だけを判断する。
    // curveSlowed: 呼び出し側の「曲線後半へ減速済み」。これが受付を開く条件になる。
    // 受付の前でも毎周期渡すこと（角度の履歴と、前半での角度超過をここで残す）
    Step updateCurve(int progressDeg10, int curveMm, bool curveSlowed) {
        if(mode == Mode::LEGACY || phase != Phase::CURVE || decided) return Step::NONE;

        const bool inWindow = curveSlowed && curveMm <= limits.acceptEndMm;
        if(!windowOpened) {
            if(progressDeg10 > preWindowMaxDeg10) preWindowMaxDeg10 = progressDeg10;
        }
        if(!windowOpened && inWindow) {
            windowOpened = true;
            windowOpenDeg10 = progressDeg10;
            windowOpenMm = curveMm;
            // 受付を開いた時点で既に越えていた角度は採らない。
            // 古い角度越えを後から拾って、狙いより遅い位置で突然始めないため
            if(progressDeg10 >= limits.startProgressDeg10) {
                decided = true;
                missed = Missed::ALREADY_PAST_AT_OPEN;
                return Step::NONE;
            }
        }
        if(!inWindow) {
            candidateRun = 0;  // 受付前の連続数を開始判定へ持ち越さない
            if(windowOpened && curveMm > limits.acceptEndMm) {
                decided = true;
                missed = Missed::WINDOW_PASSED;
            }
            return Step::NONE;
        }

        if(progressDeg10 >= limits.startProgressDeg10) {
            // 連続を数え始めた最初の周期。確定まで数周期ぶん進むので、開始角は閾値ちょうどにならない
            if(candidateRun == 0) {
                firstOverDeg10 = progressDeg10;
                firstOverMm = curveMm;
            }
            candidateRun++;
        } else {
            candidateRun = 0;
        }
        if(candidateRun > maxCandidateRun) maxCandidateRun = candidateRun;
        if(candidateRun < limits.candidateRunCount) return Step::NONE;

        decided = true;
        candidateMm = curveMm;
        candidateDeg10 = progressDeg10;
        if(mode != Mode::EARLY) return Step::CANDIDATE_READY;  // 観測だけ。制御は変えない
        phase = Phase::TRANSFER;
        return Step::START_TRANSFER;
    }

    // 横切り移動を始めるときに、開始時の状況を渡す
    void beginTransfer(int reflection, bool rawBlue) {
        startReflection = reflection;
        startedBlue = rawBlue;
        startedDark = (!rawBlue && reflection <= limits.darkReflection);
    }

    // 横切り移動中の1周期。上限の判定と、横断の証拠集めを行う
    Step updateTransfer(int movedMm, int movedMs, int headingChangeDeg10, int reflection, bool rawBlue) {
        if(phase != Phase::TRANSFER) return Step::NONE;

        // 上限を先に見る。証拠が集まる前でも越えたら止める
        if(movedMs > limits.crossMaxMs) return abort(AbortReason::TIME);
        if(movedMm > limits.crossMaxMm) return abort(AbortReason::DISTANCE);
        const int headingMag = headingChangeDeg10 < 0 ? -headingChangeDeg10 : headingChangeDeg10;
        if(headingMag > limits.crossHeadingLimitDeg10) return abort(AbortReason::HEADING);

        if(rawBlue) {
            // 青の上の暗い値は黒帯の証拠にしない。曖昧になった履歴は取り直す
            blueSamples++;
            darkRun = 0;
            brightRun = 0;
            sawDark = false;
            return Step::NONE;
        }

        if(reflection <= limits.darkReflection) {
            darkRun++;
            brightRun = 0;
            if(darkRun >= limits.darkRunCount) sawDark = true;
            return Step::NONE;
        }

        darkRun = 0;
        if(!sawDark) {
            brightRun = 0;
            return Step::NONE;
        }
        if(reflection < limits.brightReflection) {
            brightRun = 0;
            return Step::NONE;
        }
        brightRun++;
        if(brightRun < limits.brightRunCount) return Step::NONE;

        // 黒帯へ入り、反対の境界へ抜けた。開始時が黒だった場合は前進量を厚く要求する
        const int requiredMm = startedDark ? limits.minCrossMmStartedDark : limits.minCrossMm;
        if(movedMm < requiredMm) return Step::NONE;

        phase = Phase::ACQUIRE;
        crossedMm = movedMm;
        crossedMs = movedMs;
        return Step::CROSSED;
    }

    // 追従の成立を確認できた／できなかった
    void markFast() { phase = Phase::FAST; }
    void markFailed(AbortReason reason) {
        phase = Phase::FAILED;
        if(abortReason == AbortReason::NONE) abortReason = reason;
    }

    Phase getPhase() const { return phase; }
    Mode getMode() const { return mode; }
    Missed getMissed() const { return missed; }
    AbortReason getAbortReason() const { return abortReason; }
    bool isCommitted() const { return phase != Phase::CURVE; }  // 旧青1経路を止めてよいか
    bool isDecided() const { return decided; }
    int getCandidateMm() const { return candidateMm; }
    int getCandidateDeg10() const { return candidateDeg10; }
    // 受付を開いた瞬間の記録。見送った走行でも呼び出し側が回収できるよう、候補の成立とは別に持つ
    bool isWindowOpened() const { return windowOpened; }
    int getWindowOpenDeg10() const { return windowOpenDeg10; }
    int getWindowOpenMm() const { return windowOpenMm; }
    int getPreWindowMaxDeg10() const { return preWindowMaxDeg10; }
    int getFirstOverDeg10() const { return firstOverDeg10; }
    int getFirstOverMm() const { return firstOverMm; }
    int getMaxCandidateRun() const { return maxCandidateRun; }
    int getCrossedMm() const { return crossedMm; }
    int getCrossedMs() const { return crossedMs; }
    int getStartReflection() const { return startReflection; }
    int getBlueSamples() const { return blueSamples; }
    bool wasStartedDark() const { return startedDark; }
    bool wasStartedBlue() const { return startedBlue; }
    bool hasSeenDark() const { return sawDark; }

private:
    Step abort(AbortReason reason) {
        phase = Phase::FAILED;
        abortReason = reason;
        return Step::ABORT;
    }

    Mode mode;
    Limits limits;
    Phase phase = Phase::CURVE;
    Missed missed = Missed::NONE;
    AbortReason abortReason = AbortReason::NONE;

    bool decided = false;  // 候補を1回判断したか（1走行につき一度だけ）
    bool windowOpened = false;
    int windowOpenDeg10 = 0, windowOpenMm = -1;
    int preWindowMaxDeg10 = -3600;  // 受付前の最大進み角。前半で越えていたかを後から見る
    int firstOverDeg10 = 0, firstOverMm = -1;
    int candidateRun = 0, maxCandidateRun = 0;
    int candidateMm = -1, candidateDeg10 = 0;

    int startReflection = -1;
    bool startedDark = false, startedBlue = false;
    bool sawDark = false;
    int darkRun = 0, brightRun = 0, blueSamples = 0;
    int crossedMm = -1, crossedMs = -1;
};

#endif  // !EARLY_EDGE_HANDOFF_H_
