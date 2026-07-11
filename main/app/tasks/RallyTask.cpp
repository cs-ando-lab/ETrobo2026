#include "RallyTask.h"

#include "kernel.h" /* dly_tskのため */
#include "Tracer.h"
#include "Config.h"

RallyTask::RallyTask(Robot& robot)
    : robot(robot) {
}

void RallyTask::run() {
    // TODO: 担当者が実装する（赤ゲート → 青ゲート → 黄ゲートの順に通過 → ラインに戻る）
    Tracer tracer(robot);

    // ガレージ直前のy字路から青ラインまでライントレース
    while(!tracer.isOnBlue()) {
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();

    // 青ライン到達後90°左折
    robot.turn(-90.0f, Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 蛇行走行で黒ラインの捜索、黒ライン発見後ライントレースに移行
    robot.driveWaving(40.0f, Config::WAVE_DEFAULT_SPEED_DEG_PER_SEC, ColorJudge::Color::BLACK);
}
