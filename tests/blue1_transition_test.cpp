#include "../main/app/tasks/Blue1Transition.h"
#include <cassert>

namespace {

constexpr int kPassRun = 40;   // 400ms / 10ms
constexpr int kWhiteRun = 20;  // 踏み越えとみなす白の連続数

// 旧実装（1本の非青カウンタが再受付と持ち替え開始の両方を決めていた）の模型。
// 分離した実装が、同じ入力列に対して同じ時点で同じことをするかを比べるために置く。
struct OldModel {
    bool onBlue = false;
    bool finished = false;
    int whiteRun = 0;
    int nonBlueRun = 0;

    void acceptBlue() {
        onBlue = true;
        whiteRun = 0;
        nonBlueRun = 0;
    }

    // 1周期ぶん。旧実装のループ順どおり、白の判定が先で青の判定が後
    void step(bool white, bool rawBlue, bool& recovery, bool& move, bool& rearm) {
        recovery = move = rearm = false;
        if(finished || !onBlue) return;
        whiteRun = white ? whiteRun + 1 : 0;
        if(whiteRun >= kWhiteRun) {
            recovery = true;
            onBlue = false;
            finished = true;
            return;
        }
        nonBlueRun = rawBlue ? 0 : nonBlueRun + 1;
        if(nonBlueRun >= kPassRun) {
            rearm = true;   // isCurrentlyOnBlue = false（次の青を数え直す）
            move = true;    // 同じ条件で弧移動も始めていた
            onBlue = false;
            finished = true;
        }
    }
};

// 決まった種から作る疑似乱数。実機のログではなく、青・非青・白の並びを広く当てるため
unsigned nextRandom(unsigned& state) {
    state = state * 1664525u + 1013904223u;
    return state >> 16;
}

}  // namespace

int main() {
    // 同じ閾値なら旧実装と同じ。回復・持ち替え・再受付が同じ周期で起きる。
    for(unsigned seed = 1; seed <= 200; ++seed) {
        unsigned state = seed;
        OldModel old;
        Blue1Transition blue1(kPassRun, kPassRun, kWhiteRun);
        old.acceptBlue();
        blue1.acceptBlue();
        bool newOnBlue = true;

        for(int cycle = 0; cycle < 400; ++cycle) {
            const unsigned r = nextRandom(state);
            // 青の上では青が優勢。白は時々続く
            const bool rawBlue = (r % 100) < 70;
            const bool white = !rawBlue && ((r / 100) % 100) < 60;

            bool oldRecovery = false, oldMove = false, oldRearm = false;
            old.step(white, rawBlue, oldRecovery, oldMove, oldRearm);

            const bool newRecovery = blue1.updateWhite(white);
            bool newMove = false, newRearm = false;
            if(newRecovery) {
                blue1.markRecovered();
                newOnBlue = false;
            } else if(newOnBlue) {
                newMove = blue1.updateBlue(rawBlue) == Blue1Transition::Action::START_MOVE;
                if(blue1.takeRearm()) {
                    newRearm = true;
                    newOnBlue = false;
                }
                if(newMove) {
                    blue1.markCrossed();
                    if(blue1.takeRearm()) {
                        newRearm = true;
                        newOnBlue = false;
                    }
                }
            }
            assert(oldRecovery == newRecovery);
            assert(oldMove == newMove);
            assert(oldRearm == newRearm);  // 再受付が成立する周期も一致する
            assert(old.onBlue == newOnBlue);
        }
    }

    // 持ち替えだけを早める設定。再受付は旧来の位置のまま動かない。
    {
        Blue1Transition blue1(10, kPassRun, kWhiteRun);
        blue1.acceptBlue();
        for(int i = 0; i < 9; ++i) assert(blue1.updateBlue(false) == Blue1Transition::Action::NONE);
        assert(!blue1.takeRearm());  // まだ再受付しない
        assert(blue1.updateBlue(false) == Blue1Transition::Action::START_MOVE);
        assert(!blue1.takeRearm());  // 10回目でも再受付は成立しない
        // 弧を渡り切ったら、そこで再受付する（線を横切った後なので同じ青は数えない）
        blue1.markCrossed();
        assert(blue1.takeRearm());
        assert(!blue1.takeRearm());  // 成立は1回だけ
    }

    // 早い設定でも、持ち替えを始めるまでは白の連続で回復に入れる。
    {
        Blue1Transition blue1(10, kPassRun, kWhiteRun);
        blue1.acceptBlue();
        for(int i = 0; i < kWhiteRun - 1; ++i) assert(!blue1.updateWhite(true));
        assert(blue1.updateWhite(true));
        blue1.markRecovered();
        // 回復後は通常の持ち替えを行わない。再受付の通知も出さない（呼び出し側が無視期間で持つ）
        for(int i = 0; i < kPassRun + 5; ++i) assert(blue1.updateBlue(false) == Blue1Transition::Action::NONE);
        assert(!blue1.takeRearm());
        assert(!blue1.updateWhite(true));
    }

    // 青の中に非青が短く混ざっても、連続が切れればやり直し。出口候補の後で青に戻っても数え直す。
    {
        Blue1Transition blue1(kPassRun, kPassRun, kWhiteRun);
        blue1.acceptBlue();
        for(int i = 0; i < kPassRun - 1; ++i) blue1.updateBlue(false);
        assert(blue1.updateBlue(true) == Blue1Transition::Action::NONE);
        assert(blue1.getExitRun() == 0);
        for(int i = 0; i < kPassRun - 1; ++i) assert(blue1.updateBlue(false) == Blue1Transition::Action::NONE);
        assert(blue1.updateBlue(false) == Blue1Transition::Action::START_MOVE);
    }

    // 青1を確定する前は何も起きない。白も数えない。
    {
        Blue1Transition blue1(kPassRun, kPassRun, kWhiteRun);
        for(int i = 0; i < 100; ++i) {
            assert(!blue1.updateWhite(true));
            assert(blue1.updateBlue(false) == Blue1Transition::Action::NONE);
        }
        assert(!blue1.takeRearm());
        assert(!blue1.isAccepted());
    }

    // 持ち替えは一度だけ。渡り切った後はもう始めない。
    {
        Blue1Transition blue1(kPassRun, kPassRun, kWhiteRun);
        blue1.acceptBlue();
        for(int i = 0; i < kPassRun; ++i) blue1.updateBlue(false);
        blue1.markCrossed();
        for(int i = 0; i < kPassRun * 3; ++i) assert(blue1.updateBlue(false) == Blue1Transition::Action::NONE);
    }
}
