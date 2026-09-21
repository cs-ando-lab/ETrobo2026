#include "../main/app/tasks/ColorNotifier.h"
#include <cassert>

namespace {
using Action = ColorNotifier::Action;
constexpr int kTone = 100, kGap = 100, kTick = 10;

// 走行ループを模して、10msごとにupdateを回す。鳴っている区間の長さを数える
struct Run {
    int started = 0, stopped = 0, maxToneMs = 0, totalMs = 0;
};

Run runLoop(ColorNotifier& notifier, int loopMs, int startAtMs, int count) {
    Run result;
    bool playing = false;
    int toneMs = 0;
    notifier.start(count, startAtMs);
    for(int t = startAtMs; t < startAtMs + loopMs; t += kTick) {
        const Action action = notifier.update(t);
        if(action == Action::START_TONE) {
            result.started++;
            playing = true;
            toneMs = 0;
        } else if(action == Action::STOP_TONE) {
            result.stopped++;
            playing = false;
            if(toneMs > result.maxToneMs) result.maxToneMs = toneMs;
        }
        if(playing) toneMs += kTick;
        result.totalMs += kTick;
    }
    if(notifier.cancel() == Action::STOP_TONE) result.stopped++;
    return result;
}
}  // namespace

int main() {
    // 黄1・青2・赤3。1秒の直進区間に収まり、開始と停止が必ず同じ回数になる
    for(int count = 1; count <= 3; ++count) {
        ColorNotifier notifier(kTone, kGap);
        const Run result = runLoop(notifier, 1000, 0, count);
        assert(result.started == count);
        assert(result.stopped == count);  // 鳴らしっぱなしにしない
        assert(result.maxToneMs <= kTone + kTick);
        assert(notifier.isFinished() && !notifier.isPlaying());
    }

    // OFF相当（0回）。何も鳴らさず、すぐ終わる
    {
        ColorNotifier notifier(kTone, kGap);
        const Run result = runLoop(notifier, 1000, 0, 0);
        assert(result.started == 0 && result.stopped == 0);
        assert(notifier.isFinished());
    }

    // 周期が遅れても、遅れた分をまとめて鳴らさない（1回のupdateで遷移は1回だけ）
    {
        ColorNotifier notifier(kTone, kGap);
        notifier.start(3, 0);
        assert(notifier.update(0) == Action::START_TONE);
        assert(notifier.update(5000) == Action::STOP_TONE);  // 大きく遅れて気づいた
        assert(notifier.update(5000) == Action::NONE);       // 遅れた分をまとめて鳴らさない
        assert(notifier.update(5099) == Action::NONE);       // 間隔は気づいた時点から数え直す
        assert(notifier.update(5100) == Action::START_TONE);
        assert(notifier.getStartedCount() == 2);
    }

    // 区間が短くて鳴らし終わらなくても、必ず止めてから終わる（走行を延ばさない）
    {
        ColorNotifier notifier(kTone, kGap);
        const Run result = runLoop(notifier, 150, 0, 3);
        assert(result.started >= 1);
        assert(result.started == result.stopped);  // 開始した音は全部止まっている
        assert(notifier.isFinished() && !notifier.isPlaying());
        assert(result.totalMs == 150);  // 音のためにループを延ばしていない
    }

    // 鳴っている最中の中断では止める指示が出る。鳴っていなければ何もしない
    {
        ColorNotifier notifier(kTone, kGap);
        notifier.start(2, 0);
        assert(notifier.update(0) == Action::START_TONE);
        assert(notifier.cancel() == Action::STOP_TONE);
        assert(notifier.cancel() == Action::NONE);  // 二重に止めない
        assert(notifier.update(1000) == Action::NONE);
    }
    {
        ColorNotifier notifier(kTone, kGap);
        notifier.start(2, 0);
        notifier.update(0);
        notifier.update(100);  // GAPへ
        assert(!notifier.isPlaying());
        assert(notifier.cancel() == Action::NONE);
    }

    // 開始していなければ何も起きない
    {
        ColorNotifier notifier(kTone, kGap);
        for(int t = 0; t < 1000; t += kTick) assert(notifier.update(t) == Action::NONE);
        assert(notifier.cancel() == Action::NONE);
    }
}
