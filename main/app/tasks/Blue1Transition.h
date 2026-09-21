#ifndef BLUE1_TRANSITION_H_
#define BLUE1_TRANSITION_H_

/**
 * 青1本目の「次の青を数え直してよいか（再受付）」と「右エッジへの持ち替えを始めてよいか」を
 * 別々に持つ。旧実装は1本の非青カウンタが両方を決めていたため、持ち替えの開始だけを
 * 早めようとすると、同じ青を2本目として数えてしまう危険と抱き合わせになっていた。
 *
 * moveExitRunとrearmExitRunを同じ値にすると旧実装と同じ動きになる。
 * 呼び出し側は1周期につき、白の判定→（青に乗っている間だけ）青の判定、の順で呼ぶこと。
 */
class Blue1Transition {
public:
    enum class Action { NONE,
                        START_MOVE };

    // moveExitRun: 持ち替えを始める非青の連続数、rearmExitRun: 次の青を数え直す非青の連続数、
    // whiteRunLimit: 青の上で線を踏み越えたとみなす白の連続数
    Blue1Transition(int moveExitRun, int rearmExitRun, int whiteRunLimit)
        : moveExitRun(moveExitRun), rearmExitRun(rearmExitRun), whiteRunLimit(whiteRunLimit) {}

    // 青1本目を確定した
    void acceptBlue() {
        accepted = true;
        whiteRunCount = 0;
        moveExitCount = 0;
        rearmExitCount = 0;
    }

    // 青の上での白の連続。踏み越え回復に入るならtrue。
    // 持ち替えを始めた後・再受付後は、弧の中とトレースで見るので数えない
    bool updateWhite(bool white) {
        if(!isGuarding()) {
            return false;
        }
        whiteRunCount = white ? whiteRunCount + 1 : 0;
        return whiteRunCount >= whiteRunLimit;
    }

    // 非青の連続。持ち替えの開始と再受付を別々に判定する
    Action updateBlue(bool rawBlue) {
        if(!accepted || moveStarted || recovered) {
            return Action::NONE;
        }
        if(rawBlue) {
            moveExitCount = 0;
            rearmExitCount = 0;
            return Action::NONE;
        }
        moveExitCount++;
        rearmExitCount++;
        if(!rearmed && rearmExitCount >= rearmExitRun) {
            rearmed = true;
            rearmPending = true;
        }
        if(moveExitCount >= moveExitRun) {
            moveStarted = true;
            return Action::START_MOVE;
        }
        return Action::NONE;
    }

    // 再受付が成立した最初の1回だけtrue。呼び出し側で青の本数の探索を再開する
    bool takeRearm() {
        if(!rearmPending) {
            return false;
        }
        rearmPending = false;
        return true;
    }

    // 通常の持ち替えが終わった。線を横切った後なので、まだ再受付していなければここで成立させる
    void markCrossed() {
        moveStarted = true;
        if(!rearmed) {
            rearmed = true;
            rearmPending = true;
        }
    }

    // 別の経路（早期持ち替えなど）が持ち替えを引き受けた。以降この状態から動作を起こさない
    void markSuperseded() { markRecovered(); }

    // 踏み越え回復に入った。通常の持ち替えは行わない。青の無視は呼び出し側が持つ
    void markRecovered() {
        recovered = true;
        moveStarted = true;
        rearmed = true;
        rearmPending = false;
    }

    bool isAccepted() const { return accepted; }
    int getWhiteRun() const { return whiteRunCount; }
    int getExitRun() const { return moveExitCount; }

private:
    // 白の連続で踏み越えを見るのは、青1を確定してから持ち替えを始めるまでの間だけ
    bool isGuarding() const { return accepted && !moveStarted && !rearmed && !recovered; }

    int moveExitRun;
    int rearmExitRun;
    int whiteRunLimit;

    bool accepted = false;
    bool moveStarted = false;
    bool rearmed = false;
    bool rearmPending = false;
    bool recovered = false;
    int whiteRunCount = 0;
    int moveExitCount = 0;
    int rearmExitCount = 0;
};

#endif  // !BLUE1_TRANSITION_H_
