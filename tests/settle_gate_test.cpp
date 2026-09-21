#include "../main/app/tasks/SettleGate.h"
#include <cassert>

namespace {
constexpr int kThreshold = 5;
constexpr int kTimeoutLoops = 60;  // 600ms / 10ms

// 速度の列を与えて、実機のループと同じ順序で判定を回す
SettleStep runSettle(const int* leftSpeeds, const int* rightSpeeds, int count, int buttonAtLoop, int& endLoop) {
    const SettleGate gate{ kThreshold, kTimeoutLoops, 5, 2 };
    for(int i = 0;; ++i) {
        const bool button = (buttonAtLoop >= 0 && i >= buttonAtLoop);
        const int left = i < count ? leftSpeeds[i] : leftSpeeds[count - 1];
        const int right = i < count ? rightSpeeds[i] : rightSpeeds[count - 1];
        const SettleStep decision = gate.step(i, button, left, right);
        if(decision != SettleStep::CONTINUE) {
            endLoop = i;
            return decision;
        }
    }
}
}  // namespace

int main() {
    const SettleGate gate{ kThreshold, kTimeoutLoops, 5, 2 };

    // 間引きの形。5周期のうち2回だけ掛ける
    assert(gate.shouldBrake(0) && gate.shouldBrake(1));
    assert(!gate.shouldBrake(2) && !gate.shouldBrake(3) && !gate.shouldBrake(4));
    assert(gate.shouldBrake(5) && !gate.shouldBrake(7));

    // 閾値は「両輪とも」。片輪だけ落ちても止まったとみなさない
    assert(gate.isStopped(4, -4));
    assert(!gate.isStopped(4, 5));
    assert(!gate.isStopped(-9, 0));
    assert(!gate.isStopped(5, 5));  // 閾値ちょうどは未達

    // 落ちていけば止まる
    {
        const int left[] = { 200, 120, 60, 20, 4 };
        const int right[] = { 190, 110, 55, 18, 3 };
        int endLoop = -1;
        assert(runSettle(left, right, 5, -1, endLoop) == SettleStep::STOPPED);
        assert(endLoop == 4);
    }

    // 落ちなければTIMEOUT。止まったことにはしない
    {
        const int left[] = { 200 };
        const int right[] = { 200 };
        int endLoop = -1;
        assert(runSettle(left, right, 1, -1, endLoop) == SettleStep::TIMEOUT);
        assert(endLoop == kTimeoutLoops);
    }

    // 途中でボタンを押したら中断。速度が閾値未満でも中断が優先
    {
        const int left[] = { 200 };
        const int right[] = { 200 };
        int endLoop = -1;
        assert(runSettle(left, right, 1, 7, endLoop) == SettleStep::CANCELLED);
        assert(endLoop == 7);
    }
    assert(gate.step(0, true, 0, 0) == SettleStep::CANCELLED);
    // 上限に達した周期でも、中断が最優先
    assert(gate.step(kTimeoutLoops, true, 0, 0) == SettleStep::CANCELLED);
    // 上限に達していれば、速度が落ちていてもTIMEOUT（ループは回り切っている）
    assert(gate.step(kTimeoutLoops, false, 0, 0) == SettleStep::TIMEOUT);
}
