#ifndef RALLYTASK_H_
#define RALLYTASK_H_

#include "Robot.h"
#include "Tracer.h"
#include "Config.h"

/**
 * ETラリーの処理を行うクラス。
 * run()を1回呼べば、赤→青→黄のゲートを順番に通過する。
 */
class RallyTask {
public:
    RallyTask(Robot& robot);
    void run();
    void test();

    enum struct GateColor {  // ゲートの色
        RED,
        BLUE,
        YELLOW
    };

    // ゲートの座標を保持する構造体
    struct GridPoint {  // ゲートの足がどのグリッド上にあるか
        int row;        // 1〜5
        int col;        // 1〜5
    };
    struct Gate {  // ゲートの両足の座標
        GateColor color;
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
        // { ゲートの色, 左足の座標, 右足の座標}
        { GateColor::RED, { 5, 2 }, { 5, 3 } },     // red
        { GateColor::BLUE, { 3, 5 }, { 4, 5 } },    // blue
        { GateColor::YELLOW, { 2, 1 }, { 2, 2 } },  // yellow
    };
    // ゲートを通る順番を保持する配列
    static constexpr Gate gatesSequence[3] = {
        RALLY_GATE_POSITIONS.red,
        RALLY_GATE_POSITIONS.blue,
        RALLY_GATE_POSITIONS.yellow
    };

    void goToGateRow(Gate gate, Tracer& tracer);         // ラリーフィールド左（右）のライン上において、目標のゲート位置に対応する色の上で停止して90°右に回転する。
    void runThroughGate(Gate gate);                      // 赤、黄ゲートの場合はゲート列へ行ってから90°右に回転し、ラリーフィールドの端まで直進。
    void returnToRallyStart(Gate gate, Tracer& tracer);  // ゲート通過後のラインからスタート位置に戻る。
    ColorJudge::Color getTargetRowColor(Gate gate);      // ゲートに対応するグリッドの行の色を返す。
    int toXmm(Gate gate);                                // ガイドラインからゲートのある列までの距離を返す。
    int toYmm(Gate gate);                                // ゲート上部からラリーフィールドの端（下）までの距離を返す。
    void turn(Robot robot, float degree, int adjustmentDistance = 0);
};

#endif  // !RALLYTASK_H_
