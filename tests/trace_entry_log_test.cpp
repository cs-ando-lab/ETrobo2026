#include "../main/app/tasks/TraceEntryLog.h"
#include <cassert>

int main() {
    TraceEntryLogBuffer<2> buffer;
    assert(buffer.count() == 0 && buffer.dropped() == 0);

    TraceEntryRecord record;
    record.label = "blue1";
    record.result = "stable";
    record.ms = 630;
    assert(buffer.add(record) == 0);
    record.label = "area-return";
    assert(buffer.add(record) == 1);

    // 上限を越えた分は捨てて、件数だけ数える。既にある記録は壊さない。
    record.label = "overflow";
    assert(buffer.add(record) == -1);
    assert(buffer.count() == 2 && buffer.dropped() == 1);
    assert(buffer.at(0).label[0] == 'b' && buffer.at(0).ms == 630);
    assert(buffer.at(1).label[0] == 'a');

    // 追加後に書き戻せる（高速側の最初の制御までの時間は、戻った後で埋める）。
    buffer.at(1).gapToFastUs = 9200;
    assert(buffer.at(1).gapToFastUs == 9200);

    // 走行ごとに初期化する。欠落数も戻る。
    buffer.clear();
    assert(buffer.count() == 0 && buffer.dropped() == 0);
    assert(buffer.add(record) == 0);
    assert(buffer.at(0).gapToFastUs == -1);  // 既定値は「未計測」
}
