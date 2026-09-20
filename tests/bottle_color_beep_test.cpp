#include "../main/app/tasks/BottleColorBeep.h"
#include <cassert>

int main() {
    // 旧通知での「鳴り終わるまで走行が止まる時間」。黄100 / 青300 / 赤500ms。
    assert(bottleColorBeepPlan(1, true, 100, 100).blockingMs == 100);
    assert(bottleColorBeepPlan(2, true, 100, 100).blockingMs == 300);
    assert(bottleColorBeepPlan(3, true, 100, 100).blockingMs == 500);
    assert(bottleColorBeepPlan(3, true, 100, 100).count == 3);

    // 無効化したら回数も待ち時間も0。通知のぶんの空待ちを残さない
    for(int n = 0; n <= 3; ++n) {
        assert(bottleColorBeepPlan(n, false, 100, 100).count == 0);
        assert(bottleColorBeepPlan(n, false, 100, 100).blockingMs == 0);
    }
    // 回数0や負でも鳴らさない
    assert(bottleColorBeepPlan(0, true, 100, 100).blockingMs == 0);
    assert(bottleColorBeepPlan(-1, true, 100, 100).count == 0);
}
