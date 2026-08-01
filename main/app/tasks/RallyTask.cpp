#include "RallyTask.h"

#include "kernel.h" /* dly_tskのため */
#include "t_syslog.h"

// 各色ゲートの座標
const RallyTask::Gate RallyTask::gatesSequence[3] = {
    { GateColor::RED,
      { Config::ETRALLY_RED_GATE_LEFT_ROW, Config::ETRALLY_RED_GATE_LEFT_COL },
      { Config::ETRALLY_RED_GATE_RIGHT_ROW, Config::ETRALLY_RED_GATE_RIGHT_COL } },
    { GateColor::BLUE,
      { Config::ETRALLY_BLUE_GATE_LEFT_ROW, Config::ETRALLY_BLUE_GATE_LEFT_COL },
      { Config::ETRALLY_BLUE_GATE_RIGHT_ROW, Config::ETRALLY_BLUE_GATE_RIGHT_COL } },
    { GateColor::YELLOW,
      { Config::ETRALLY_YELLOW_GATE_LEFT_ROW, Config::ETRALLY_YELLOW_GATE_LEFT_COL },
      { Config::ETRALLY_YELLOW_GATE_RIGHT_ROW, Config::ETRALLY_YELLOW_GATE_RIGHT_COL } }
};

RallyTask::RallyTask(Robot& robot)
    : robot(robot) {
}

// testはテスト用runが本番用
void RallyTask::test() {
    return;
}

void RallyTask::run() {
    /* 一定距離後ろ向きにライントレースした後、停止。 */
    int BACK_LINE_TRACE_DISTANCE_MM = 100;  // 後ろ向きにライントレースする基本距離
    return;
}

void RallyTask::turn(float degree, int adjustmentDistance, int delayTime) {
    dly_tsk(delayTime);                                                          // 直前のモータの動きによって正確性に影響が出ないようにdelayを挟む
    robot.driveStraight((int)(adjustmentDistance), Config::ETRALLY_SLOW_SPEED);  // 回転軸の位置を調整
    robot.turnByImu(degree);
    dly_tsk(delayTime);  // モータの動きによって直後の動きの正確性に影響が出ないようにdelayを挟む
}
