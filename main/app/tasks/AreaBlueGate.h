#ifndef AREA_BLUE_GATE_H_
#define AREA_BLUE_GATE_H_

// 距離はコーナー完了からの相対値。観測カウンタとは独立した最終青専用ゲート。
class AreaBlueGate {
public:
    enum class Result { OBSERVE,
                        WAIT_CLEAR,
                        SEARCH,
                        FOUND,
                        MISSING };
    AreaBlueGate(int startMm, int limitMm, int required)
        : startMm(startMm), limitMm(limitMm), required(required) {}
    Result update(int distanceMm, bool blue, bool slowed) {
        // 上限外の青を配置の根拠にしない。
        if(distanceMm > limitMm)
            return Result::MISSING;
        if(distanceMm < startMm || !slowed) {
            clearRun = matchRun = 0;
            cleared = false;
            return Result::OBSERVE;
        }
        // 開始地点ですでに踏んでいる青（前の青かもしれない）は採用しない。
        if(!cleared) {
            clearRun = blue ? 0 : clearRun + 1;
            if(clearRun >= 3)
                cleared = true;
            return Result::WAIT_CLEAR;
        }
        matchRun = blue ? matchRun + 1 : 0;
        return matchRun >= required ? Result::FOUND : Result::SEARCH;
    }
    bool armed() const { return cleared; }

private:
    int startMm, limitMm, required;
    int clearRun = 0, matchRun = 0;
    bool cleared = false;
};

#endif
