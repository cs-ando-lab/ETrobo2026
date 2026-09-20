#ifndef BOTTLE_COLOR_BEEP_H_
#define BOTTLE_COLOR_BEEP_H_

// ボトル色の通知ビープの鳴らし方。ハードウェアに触らないのでホストでも試験できる。
// playTone()は鳴り終わるまで戻らないため、鳴らしている間は走行が止まる。
// その時間を数値として外から見えるようにし、「通知を消したら本当に0になるか」を試験で固定する。
struct BottleColorBeepPlan {
    int count;       // 鳴らす回数。0なら鳴らさない
    int toneMs;      // 1回の長さ
    int gapMs;       // 2回目以降の前に空ける時間
    int blockingMs;  // 鳴らし終わるまでに走行が止まっている時間
};

inline BottleColorBeepPlan bottleColorBeepPlan(int beepCount, bool enabled, int toneMs, int gapMs) {
    if(!enabled || beepCount <= 0) {
        return BottleColorBeepPlan{ 0, toneMs, gapMs, 0 };
    }
    return BottleColorBeepPlan{ beepCount, toneMs, gapMs, beepCount * toneMs + (beepCount - 1) * gapMs };
}

#endif  // !BOTTLE_COLOR_BEEP_H_
