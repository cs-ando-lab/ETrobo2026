#include "../main/app/tasks/EarlyEdgeHandoff.h"
#include <cassert>

namespace {
using Mode = EarlyEdgeHandoff::Mode;
using Step = EarlyEdgeHandoff::Step;
using Phase = EarlyEdgeHandoff::Phase;
using Missed = EarlyEdgeHandoff::Missed;
using AbortReason = EarlyEdgeHandoff::AbortReason;

EarlyEdgeHandoff::Limits defaultLimits() { return EarlyEdgeHandoff::Limits{}; }

// 曲線前半（まだ減速していない）を走らせる。受付は開かない
void driveFirstHalf(EarlyEdgeHandoff& handoff, int deg10, int fromMm, int toMm) {
    for(int mm = fromMm; mm <= toMm; ++mm) handoff.updateCurve(deg10, mm, false);
}

// 受付の中で角度を越えさせ、候補を成立させる
Step driveToCandidate(EarlyEdgeHandoff& handoff, int startMm = 650) {
    Step last = Step::NONE;
    for(int i = 0; i < 3; ++i) last = handoff.updateCurve(600, startMm + i, true);  // まだ60度
    for(int i = 0; i < 10; ++i) {
        last = handoff.updateCurve(720, startMm + 3 + i, true);
        if(last != Step::NONE) return last;
    }
    return last;
}
}  // namespace

int main() {
    // LEGACYは何も起こさない
    {
        EarlyEdgeHandoff handoff(Mode::LEGACY, defaultLimits());
        assert(driveToCandidate(handoff) == Step::NONE);
        assert(!handoff.isCommitted());
        assert(handoff.getPhase() == Phase::CURVE);
    }

    // OBSERVE_ONLYは候補を記録するが、持ち替えへは進まない（制御を変えない）
    {
        EarlyEdgeHandoff handoff(Mode::OBSERVE_ONLY, defaultLimits());
        assert(driveToCandidate(handoff) == Step::CANDIDATE_READY);
        assert(handoff.getPhase() == Phase::CURVE);
        assert(!handoff.isCommitted());
        assert(handoff.getCandidateMm() >= 650);
        // 一度だけ。以降は何も返さない
        for(int i = 0; i < 20; ++i) assert(handoff.updateCurve(750, 800 + i, true) == Step::NONE);
    }

    // EARLYは候補で持ち替えへ入る。旧経路はここから止めてよい
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        assert(driveToCandidate(handoff) == Step::START_TRANSFER);
        assert(handoff.getPhase() == Phase::TRANSFER);
        assert(handoff.isCommitted());
        // 二度目は開始しない
        assert(handoff.updateCurve(750, 900, true) == Step::NONE);
    }

    // 角度が一瞬だけ越えても始めない（連続成立が要る）
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        for(int i = 0; i < 20; ++i) {
            const int deg = (i % 2 == 0) ? 720 : 690;  // 交互に越える
            assert(handoff.updateCurve(deg, 650 + i, true) == Step::NONE);
        }
        assert(handoff.getPhase() == Phase::CURVE);
    }

    // 受付を開いた時点で既に越えていたら採らない（古い角度越えを拾わない）。
    // その場で遅れて強制開始もしない。旧経路がそのまま続く
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        for(int i = 0; i < 30; ++i) assert(handoff.updateCurve(800, 600 + i, true) == Step::NONE);
        assert(handoff.getMissed() == Missed::ALREADY_PAST_AT_OPEN);
        assert(handoff.getPhase() == Phase::CURVE);
        assert(!handoff.isCommitted());
        assert(handoff.isWindowOpened());
        assert(handoff.getWindowOpenDeg10() == 800 && handoff.getWindowOpenMm() == 600);
    }

    // 曲線前半で角度を越えても始まらない。連続数も受付へ持ち越さない
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveFirstHalf(handoff, 750, 200, 400);  // 前半でずっと超過
        assert(handoff.getMissed() == Missed::NONE);
        assert(!handoff.isWindowOpened());
        assert(handoff.getPreWindowMaxDeg10() == 750);
        // 受付が開いた時点でまだ超過していれば「既に越えていた」扱い
        assert(handoff.updateCurve(750, 401, true) == Step::NONE);
        assert(handoff.getMissed() == Missed::ALREADY_PAST_AT_OPEN);
    }
    // 前半で一度越えても、受付が開く時点で下がっていれば普通に始められる
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveFirstHalf(handoff, 720, 200, 300);
        driveFirstHalf(handoff, 600, 301, 400);
        assert(handoff.getPreWindowMaxDeg10() == 720);
        assert(handoff.updateCurve(600, 401, true) == Step::NONE);
        assert(handoff.getMissed() == Missed::NONE);
        // 受付前の連続数は持ち越されないので、ここから5周期ぶん必要になる
        assert(handoff.updateCurve(720, 402, true) == Step::NONE);
        assert(handoff.updateCurve(720, 403, true) == Step::NONE);
        assert(handoff.updateCurve(720, 404, true) == Step::NONE);
        assert(handoff.updateCurve(720, 405, true) == Step::NONE);
        assert(handoff.updateCurve(720, 406, true) == Step::START_TRANSFER);
    }
    // 600mmより手前で後半に入り、そこで70度へ達する走行でも始められる（固定600mmでは逃していた）
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveFirstHalf(handoff, 500, 300, 409);
        assert(handoff.updateCurve(600, 410, true) == Step::NONE);  // 410mmで後半へ
        assert(handoff.isWindowOpened() && handoff.getWindowOpenMm() == 410);
        Step step = Step::NONE;
        for(int i = 0; i < 6 && step == Step::NONE; ++i) step = handoff.updateCurve(705 + i, 411 + i, true);
        assert(step == Step::START_TRANSFER);
        assert(handoff.getCandidateMm() < 600);
        // 連続成立に数周期かかるので、最初に越えた角度と候補確定の角度は一致しない
        assert(handoff.getFirstOverDeg10() == 705);
        assert(handoff.getCandidateDeg10() > handoff.getFirstOverDeg10());
    }
    // 受付が開いたまま上限を過ぎたら見送る（角度が届かない場合）
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        for(int mm = 410; mm <= 1150; ++mm) assert(handoff.updateCurve(600, mm, true) == Step::NONE);
        assert(handoff.getMissed() == Missed::WINDOW_PASSED);
        assert(!handoff.isCommitted());
    }
    // 後半へ入らないまま上限を過ぎた場合は、受付自体が開かない
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveFirstHalf(handoff, 750, 410, 1150);
        assert(!handoff.isWindowOpened());
        assert(handoff.getMissed() == Missed::NONE);
        assert(!handoff.isCommitted());
    }

    // 横断：明→黒→明で成立する
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        int mm = 0, ms = 0;
        Step step = Step::NONE;
        for(int i = 0; i < 3; ++i) step = handoff.updateTransfer(++mm, ms += 10, 5, 70, false);
        for(int i = 0; i < 3; ++i) step = handoff.updateTransfer(++mm, ms += 10, 5, 20, false);  // 黒帯
        assert(handoff.hasSeenDark());
        for(int i = 0; i < 3; ++i) step = handoff.updateTransfer(mm += 3, ms += 10, 5, 75, false);
        assert(step == Step::CROSSED);
        assert(handoff.getPhase() == Phase::ACQUIRE);
    }

    // 黒を踏まずに明るいだけでは成立しない
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        for(int i = 0; i < 20; ++i) {
            assert(handoff.updateTransfer(i, i * 10, 0, 90, false) == Step::NONE);
        }
        assert(!handoff.hasSeenDark());
    }

    // 青の上の暗い値を黒帯の証拠にしない。履歴は取り直す
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        int mm = 0, ms = 0;
        for(int i = 0; i < 5; ++i) handoff.updateTransfer(++mm, ms += 10, 0, 25, true);  // 青で暗い
        assert(!handoff.hasSeenDark());
        assert(handoff.getBlueSamples() == 5);
        for(int i = 0; i < 3; ++i) handoff.updateTransfer(++mm, ms += 10, 0, 80, false);
        assert(handoff.updateTransfer(++mm, ms += 10, 0, 80, false) == Step::NONE);  // 黒無しでは渡さない
    }

    // 黒帯を確認した後に青が入ったら、黒帯の確認から取り直す
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        int mm = 0, ms = 0;
        for(int i = 0; i < 3; ++i) handoff.updateTransfer(++mm, ms += 10, 0, 20, false);
        assert(handoff.hasSeenDark());
        handoff.updateTransfer(++mm, ms += 10, 0, 20, true);  // 青
        assert(!handoff.hasSeenDark());
        for(int i = 0; i < 5; ++i) assert(handoff.updateTransfer(mm += 2, ms += 10, 0, 80, false) == Step::NONE);
    }

    // 開始時に黒だった場合は、前進量を厚く要求する（黒1点を横断の証拠にしない）
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(20, false);
        assert(handoff.wasStartedDark());
        int mm = 0, ms = 0;
        for(int i = 0; i < 3; ++i) handoff.updateTransfer(++mm, ms += 10, 0, 20, false);
        for(int i = 0; i < 3; ++i) {
            // 明るい側へ抜けても、距離が足りないうちは成立させない
            assert(handoff.updateTransfer(mm, ms += 10, 0, 80, false) == Step::NONE);
        }
        assert(handoff.updateTransfer(30, ms += 10, 0, 80, false) == Step::CROSSED);
    }

    // 上限：距離・時間・方位
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        assert(handoff.updateTransfer(61, 100, 0, 70, false) == Step::ABORT);
        assert(handoff.getAbortReason() == AbortReason::DISTANCE);
        assert(handoff.getPhase() == Phase::FAILED);
        // 打ち切った後は何も返さない
        assert(handoff.updateTransfer(10, 100, 0, 20, false) == Step::NONE);
    }
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        assert(handoff.updateTransfer(10, 401, 0, 70, false) == Step::ABORT);
        assert(handoff.getAbortReason() == AbortReason::TIME);
    }
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        assert(handoff.updateTransfer(10, 100, -101, 70, false) == Step::ABORT);
        assert(handoff.getAbortReason() == AbortReason::HEADING);
    }

    // 高速へ移るのは一度だけ。失敗を上書きしない
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        driveToCandidate(handoff);
        handoff.beginTransfer(70, false);
        handoff.markFailed(AbortReason::NO_EVIDENCE);
        assert(handoff.getPhase() == Phase::FAILED);
        assert(handoff.updateTransfer(1, 1, 0, 20, false) == Step::NONE);
    }

    // 旧経路と二重に動かさない。持ち替えへ入った後も、打ち切った後も受付へ戻らない
    {
        EarlyEdgeHandoff handoff(Mode::EARLY, defaultLimits());
        assert(driveToCandidate(handoff) == Step::START_TRANSFER);
        handoff.beginTransfer(70, false);
        assert(handoff.updateTransfer(61, 100, 0, 70, false) == Step::ABORT);
        assert(handoff.getPhase() == Phase::FAILED);
        // 打ち切っても受付は再開しない（旧青1経路と早期経路が同じ走行で二度動かない）
        for(int i = 0; i < 20; ++i) assert(handoff.updateCurve(800, 700 + i, true) == Step::NONE);
        assert(handoff.getPhase() == Phase::FAILED);
        assert(handoff.isCommitted());
    }

    // Rコースの鏡像。符号を反転した進み角を渡せば同じ判断になる
    {
        EarlyEdgeHandoff left(Mode::EARLY, defaultLimits());
        EarlyEdgeHandoff right(Mode::EARLY, defaultLimits());
        for(int i = 0; i < 3; ++i) {
            left.updateCurve(600, 650 + i, true);
            right.updateCurve(600, 650 + i, true);  // 呼び出し側で -(heading-baseline)*courseSign に揃える
        }
        Step leftStep = Step::NONE, rightStep = Step::NONE;
        for(int i = 0; i < 10 && leftStep == Step::NONE; ++i) leftStep = left.updateCurve(720, 653 + i, true);
        for(int i = 0; i < 10 && rightStep == Step::NONE; ++i) rightStep = right.updateCurve(720, 653 + i, true);
        assert(leftStep == rightStep && leftStep == Step::START_TRANSFER);
    }
}
