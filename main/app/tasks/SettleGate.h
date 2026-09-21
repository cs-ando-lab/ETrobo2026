#ifndef SETTLE_GATE_H_
#define SETTLE_GATE_H_

// 「止め切る」処理の判定だけを取り出したもの。モーターにも時計にも触らないのでホストで試験できる。
// ブレーキを掛けっぱなしにせず間引くのは、平均の制動力を下げてボトルを振らせないため。
enum class SettleStep { CONTINUE,
                        STOPPED,
                        TIMEOUT,
                        CANCELLED };

struct SettleGate {
    int thresholdDegPerSec = 0;  // 両輪がこれ未満なら止まったとみなす（完全静止ではない）
    int timeoutLoops = 0;        // これだけ回っても届かなければTIMEOUT
    int brakeCycleLoops = 1;     // 間引きの周期
    int brakeOnLoops = 1;        // そのうちブレーキを掛ける回数

    bool shouldBrake(int loopIndex) const {
        return (loopIndex % brakeCycleLoops) < brakeOnLoops;
    }

    bool isStopped(int leftSpeed, int rightSpeed) const {
        const int left = leftSpeed < 0 ? -leftSpeed : leftSpeed;
        const int right = rightSpeed < 0 ? -rightSpeed : rightSpeed;
        return left < thresholdDegPerSec && right < thresholdDegPerSec;
    }

    // 1周期ぶんの判定。中断が最優先で、TIMEOUTを停止成功として返さない
    SettleStep step(int loopIndex, bool buttonPressed, int leftSpeed, int rightSpeed) const {
        if(buttonPressed)
            return SettleStep::CANCELLED;
        if(loopIndex >= timeoutLoops)
            return SettleStep::TIMEOUT;
        if(isStopped(leftSpeed, rightSpeed))
            return SettleStep::STOPPED;
        return SettleStep::CONTINUE;
    }
};

#endif  // !SETTLE_GATE_H_
