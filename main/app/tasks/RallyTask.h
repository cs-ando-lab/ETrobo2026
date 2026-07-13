#ifndef RALLYTASK_H_
#define RALLYTASK_H_

#include "Robot.h"

/**
 * ETラリーの処理を行うクラス。
 * run()を1回呼べば、赤→青→黄のゲートを順番に通過する。
 */
class RallyTask {
public:
    RallyTask(Robot& robot);
    void run();

    // ゲートの座標を保持する構造体
    struct GridPoint {  // ゲートの足がどのグリッド上にあるか
        int row;        // 1〜5
        int col;        // 1〜5
    };
    struct Gate {  // ゲートの両足の座標
        GridPoint leftLeg;
        GridPoint rightLeg;
    };
    struct GatePositions {  // 3色ゲートの座標
        Gate red;
        Gate blue;
        Gate yellow;
    };

private:
    Robot& robot;
    static constexpr GatePositions RALLY_GATE_POSITIONS = {
        { { 5, 2 }, { 5, 3 } },  // red
        { { 3, 5 }, { 4, 5 } },  // blue
        { { 2, 1 }, { 2, 2 } },  // yellow
    };
};

#endif  // !RALLYTASK_H_
