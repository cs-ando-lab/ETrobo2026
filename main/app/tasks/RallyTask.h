#ifndef RALLYTASK_H_
#define RALLYTASK_H_

#include "Robot.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"

/**
 * ETラリーの処理を行うクラス。
 * run()を1回呼べば、赤→青→黄のゲートを順番に通過する。
 */
class RallyTask {
public:
    RallyTask(Robot& robot);
    void run();
    void test();

    // ゲートの色
    enum struct GateColor {
        RED,
        BLUE,
        YELLOW
    };

    // ゲートの足がどのグリッド上にあるかを保持する構造体
    struct GridPoint {
        int row;  // 1〜5
        int col;  // 1〜5
    };

    // ゲートの両足の座標
    struct Gate {
        GateColor color;
        GridPoint leftLeg;
        GridPoint rightLeg;
    };

private:
    Robot& robot;
    // ゲートを通る順番を保持する配列
    static const Gate gatesSequence[3];

    void turn(float degree, int adjustmentDistance = 0, int delayTime = 0);
};

#endif  // !RALLYTASK_H_
