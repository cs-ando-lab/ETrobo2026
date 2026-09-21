#include "../main/app/Pid.h"
#include <cassert>
#include <cmath>
#include <initializer_list>

namespace {
constexpr float kDt = 0.01f;
constexpr float kKp = 0.30f, kKi = 0.01f, kKd = 0.02f, kTarget = 60.0f;
bool near(float a, float b) { return std::fabs(a - b) < 1e-4f; }
}  // namespace

int main() {
    // 再開の初回は、偏差がいくつでもDとIが0でPだけが出る。黒・白・目標値ちょうどで確認する。
    for(float first : { 15.0f, 99.0f, kTarget }) {
        Pid pid(kKp, kKi, kKd, kTarget);
        pid.restartFromNextSample();
        const float out = pid.calculate(first, kDt);
        assert(near(pid.getLastD(), 0.0f));
        assert(near(pid.getLastI(), 0.0f));
        assert(near(pid.getLastP(), kKp * (kTarget - first)));
        assert(near(out, pid.getLastP()));

        // 2回目が同じ値なら、変化していないのでDは0のまま。Iは1周期ぶんだけ入る。
        const float same = pid.calculate(first, kDt);
        assert(near(pid.getLastD(), 0.0f));
        assert(near(pid.getLastI(), kKi * (kTarget - first) * kDt));
        assert(near(same, pid.getLastP() + pid.getLastI()));
    }

    // 2回目が変化していれば、通常どおり符号つきで応答する（黒へ寄る＝偏差が増える）。
    {
        Pid pid(kKp, kKi, kKd, kTarget);
        pid.restartFromNextSample();
        pid.calculate(50.0f, kDt);
        pid.calculate(40.0f, kDt);
        assert(pid.getLastD() > 0.0f);  // 偏差が10増えた
    }
    {
        Pid pid(kKp, kKi, kKd, kTarget);
        pid.restartFromNextSample();
        pid.calculate(50.0f, kDt);
        pid.calculate(70.0f, kDt);
        assert(pid.getLastD() < 0.0f);
    }

    // 旧reset()は変わらない。再開直後の1回目に「現在偏差 - 0」ぶんのDが出る（これが今回避けたい挙動）。
    {
        Pid pid(kKp, kKi, kKd, kTarget);
        pid.reset();
        pid.calculate(15.0f, kDt);
        assert(pid.getLastD() > 0.0f);
    }

    // 予約は1回だけ消費される。2回目以降は通常の計算に戻る。
    {
        Pid pid(kKp, kKi, kKd, kTarget);
        pid.restartFromNextSample();
        pid.calculate(15.0f, kDt);
        pid.calculate(99.0f, kDt);
        assert(!near(pid.getLastD(), 0.0f));
    }

    // 再開は積分の持ち越しも捨てる。上限まで溜めてから再開しても初回のIは0。
    {
        Pid pid(kKp, kKi, kKd, kTarget);
        for(int i = 0; i < 2000; ++i) pid.calculate(0.0f, kDt);
        assert(near(pid.getLastI(), kKi * Config::PID_INTEGRAL_LIMIT));  // 既存のクランプは不変
        pid.restartFromNextSample();
        pid.calculate(0.0f, kDt);
        assert(near(pid.getLastI(), 0.0f));
    }

    // reset()は再開の予約を取り消す。初期化の方法が二重に掛からない。
    {
        Pid pid(kKp, kKi, kKd, kTarget);
        pid.restartFromNextSample();
        assert(pid.hasPendingRestart());
        pid.reset();
        assert(!pid.hasPendingRestart());
        pid.calculate(15.0f, kDt);
        assert(pid.getLastD() > 0.0f);  // 予約は消えているので旧reset()どおりの初回D
    }

    // 逆順（reset→予約）なら予約が残る。
    {
        Pid pid(kKp, kKi, kKd, kTarget);
        pid.reset();
        pid.restartFromNextSample();
        assert(pid.hasPendingRestart());
        pid.calculate(15.0f, kDt);
        assert(near(pid.getLastD(), 0.0f));
        assert(!pid.hasPendingRestart());
    }

    // 追従を続けたままゲインを変える経路（Tracer::setConfigKeepingTrackingStateが行う操作）。
    // 前回偏差と微分フィルタは残り、積分だけが0に戻る。
    {
        Pid keep(kKp, kKi, kKd, kTarget), reference(kKp, kKi, kKd, kTarget);
        for(int i = 0; i < 50; ++i) {
            keep.calculate(50.0f, kDt);
            reference.calculate(50.0f, kDt);
        }
        assert(keep.getLastI() > 0.0f);

        keep.setGain(0.30f, 0.01f, 0.02f);
        keep.setTarget(kTarget);
        keep.resetIntegral();
        const float after = keep.calculate(45.0f, kDt);

        // 同じ履歴のまま積分だけ捨てたのと一致する（前回偏差・微分フィルタが残っている証拠）
        reference.setGain(0.30f, 0.01f, 0.02f);
        reference.resetIntegral();
        const float expected = reference.calculate(45.0f, kDt);
        assert(near(after, expected));
        assert(keep.getLastD() > 0.0f);           // 実際に5だけ動いたぶんのDが出る
        // 積分はこの周期ぶんだけ。台形なので前回偏差(60-50)が残っていることもここで効く
        const float trapezoid = ((kTarget - 45.0f) + (kTarget - 50.0f)) * kDt / 2.0f;
        assert(near(keep.getLastI(), kKi * trapezoid));
    }

    // dtが0以下のときの既存の保護（0.01秒として扱う）は変わらない。
    {
        Pid a(kKp, kKi, kKd, kTarget), b(kKp, kKi, kKd, kTarget);
        a.restartFromNextSample();
        b.restartFromNextSample();
        a.calculate(50.0f, 0.0f);
        b.calculate(50.0f, kDt);
        assert(near(a.calculate(40.0f, 0.0f), b.calculate(40.0f, kDt)));
    }
}
