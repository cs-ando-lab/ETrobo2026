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

    // // ゲートの足がどのグリッド上にあるかを保持する構造体
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

    void goToGateRow(Gate gate, Tracer& tracer);         // ラリーフィールド左（右）のライン上において、目標のゲート位置に対応する色の上で停止して90°右に回転する。
    void runThroughGate(Gate gate);                      // 赤、黄ゲートの場合はゲート列へ行ってから90°右に回転し、ラリーフィールドの端まで直進。
    void returnToRallyStart(Gate gate, Tracer& tracer);  // ゲート通過後のラインからスタート位置に戻る。
    ColorJudge::Color getTargetRowColor(Gate gate);      // ゲートに対応するグリッドの行の色を返す。
    int toXmm(Gate gate);                                // ガイドラインからゲートのある列までの距離を返す。
    int toYmm(Gate gate);                                // ゲート上部からラリーフィールドの端（下）までの距離を返す。
    void turn(Robot robot, float degree, int adjustmentDistance = 0, int delayTime = 0);
};

#endif  // !RALLYTASK_H_
