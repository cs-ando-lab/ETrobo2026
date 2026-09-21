#include "../main/app/tasks/ApproachEnd.h"
#include <cassert>

namespace {
using Reason = ApproachEnd::Reason;

constexpr int kTarget = 110;    // Config::DELIVERY_TARGET_DISTANCE_MM
constexpr int kFallback = 200;  // kApproachFallbackDistanceMm

Reason decide(int ultrasonicMm, int movedMm) {
    return ApproachEnd::decide(ultrasonicMm, kTarget, movedMm, kFallback);
}
}  // namespace

int main() {
    // 距離の境目。199mmでは終わらず、200mmちょうどで終わる
    assert(decide(0, 199) == Reason::NONE);
    assert(decide(0, 200) == Reason::DISTANCE);
    assert(decide(0, 201) == Reason::DISTANCE);

    // 超音波が先に正常検知したら、200mmまで待たない
    assert(decide(110, 10) == Reason::ULTRASONIC);
    assert(decide(50, 0) == Reason::ULTRASONIC);
    assert(decide(111, 10) == Reason::NONE);  // まだ遠い

    // 超音波が無効（0・負）でも距離条件は独立に働く
    assert(decide(0, 250) == Reason::DISTANCE);
    assert(decide(-1, 250) == Reason::DISTANCE);
    assert(decide(-1, 100) == Reason::NONE);
    // 遠方値を返し続けても同じ
    assert(decide(2000, 199) == Reason::NONE);
    assert(decide(2000, 200) == Reason::DISTANCE);

    // 後退した場合。絶対値では到達扱いにしない
    assert(decide(0, -200) == Reason::NONE);
    assert(decide(0, -1) == Reason::NONE);
    assert(decide(-5, -300) == Reason::NONE);

    // 同じ周期に両方成立したら、記録は超音波優先（動作はどちらでも同じ）
    assert(decide(100, 250) == Reason::ULTRASONIC);
    assert(decide(kTarget, kFallback) == Reason::ULTRASONIC);

    // 代替距離を0以下にしても、開始直後にいきなり成立させない用途では使わない前提だが、
    // 判定そのものは渡された値どおりに動く
    assert(ApproachEnd::decide(0, kTarget, 0, 0) == Reason::DISTANCE);
    assert(ApproachEnd::decide(0, kTarget, 0, 1) == Reason::NONE);
}
