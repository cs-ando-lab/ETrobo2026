#ifndef COLOR_NOTIFIER_H_
#define COLOR_NOTIFIER_H_

/**
 * ボトル色の通知音を、走行ループの中で鳴らすための状態機械。
 * 時刻を引数で受け取るだけでハードウェアに触らないので、ホストでも試験できる。
 *
 * 呼び出し側の責任:
 *  - 毎周期、安全確認とモーター出力を済ませてからupdate()を呼ぶ。音のために待たない。
 *  - 区間の終了・中断では必ずcancel()を呼ぶ。音が鳴り終わらなくても区間は延ばさない。
 *  - 返ってきたActionだけを実行する（自分が鳴らした音だけを止める）。
 */
class ColorNotifier {
public:
    enum class Action { NONE,
                        START_TONE,
                        STOP_TONE };
    enum class Phase { IDLE,
                       PENDING,
                       TONE,
                       GAP,
                       DONE };

    ColorNotifier(int toneMs, int gapMs)
        : toneMs(toneMs), gapMs(gapMs) {}

    // 鳴らす回数を決める。0以下なら何も鳴らさずDONEになる
    void start(int count, int nowMs) {
        remaining = count;
        deadlineMs = nowMs;
        phase = count > 0 ? Phase::PENDING : Phase::DONE;
    }

    // 1周期ぶん。期限を過ぎていても遷移は1回だけで、遅れを取り戻すために連続では鳴らさない
    Action update(int nowMs) {
        switch(phase) {
            case Phase::PENDING:
                phase = Phase::TONE;
                deadlineMs = nowMs + toneMs;
                remaining--;
                startedCount++;
                return Action::START_TONE;
            case Phase::TONE:
                if(nowMs < deadlineMs)
                    return Action::NONE;
                phase = remaining > 0 ? Phase::GAP : Phase::DONE;
                deadlineMs = nowMs + gapMs;
                return Action::STOP_TONE;
            case Phase::GAP:
                if(nowMs < deadlineMs)
                    return Action::NONE;
                phase = Phase::TONE;
                deadlineMs = nowMs + toneMs;
                remaining--;
                startedCount++;
                return Action::START_TONE;
            default:
                return Action::NONE;
        }
    }

    // 区間の終了・中断。鳴っている最中なら止める指示を返す
    Action cancel() {
        const bool playing = (phase == Phase::TONE);
        remaining = 0;
        phase = Phase::DONE;
        return playing ? Action::STOP_TONE : Action::NONE;
    }

    bool isPlaying() const { return phase == Phase::TONE; }
    bool isFinished() const { return phase == Phase::DONE || phase == Phase::IDLE; }
    Phase getPhase() const { return phase; }
    int getStartedCount() const { return startedCount; }
    int getRemaining() const { return remaining; }

private:
    int toneMs;
    int gapMs;
    Phase phase = Phase::IDLE;
    int deadlineMs = 0;
    int remaining = 0;
    int startedCount = 0;
};

#endif  // !COLOR_NOTIFIER_H_
