#include "GameRunner.h"
#include "Calibrator.h"
#include "Tracer.h"
#include "tasks/SumoTask.h"
#include "tasks/DeliveryTask.h"
#include "tasks/RallyTask.h"
#include "Config.h"
#include "kernel.h"

GameRunner::GameRunner(Robot& robot)
    : robot(robot) {
}

void GameRunner::run() {
    // 1. キャリブレーション（L/R選択、スタート待ち）
    Calibrator calib(robot);
    calib.run();

    // 2. LAPゲートまでライントレース → ET相撲
    if(!lineTraceUntilLap()) {
        return;
    }
    // SumoTask sumo(robot);
    // sumo.run();

    // 3. LAPゲートまでライントレース → ボトルデリバリー
    // if(!lineTraceUntilLap()) {
    //  return;
    //}
    DeliveryTask delivery(robot);
    delivery.run();

    // 4. LAPゲートまでライントレース → ETラリー
    // if(!lineTraceUntilLap()) {
    //  return;
    //}
    // RallyTask rally(robot);
    // rally.run();

    // 5. ゴール
    // if(!lineTraceUntilLap()) {
    //     return;
    // }
    robot.stop();
}

bool GameRunner::lineTraceUntilLap() {
    Tracer tracer(robot);

    const char* const TRACER_LABEL = "Tracer";
    const int TRACER_LABEL_LEN = 6; /* strlen(TRACER_LABEL) */
    int prevLabelIndex = -1;
    int blueCount = 0;          // 青ラインを検知した回数
    int displayCycleCount = 0;  // "Tracer"の文字循環表示の周期カウント

    while(1) {
        /* センターボタンで安全停止 */
        if(robot.isCenterButtonPressed()) {
            tracer.terminate();
            return false;
        }

        /* BLUEを一定回数検知したら停止 */
        if(robot.isOnColor(ColorJudge::Color::BLUE, blueCount)) {
            break;
        }

        /* ライントレース */
        tracer.run();

        /* ライントレース中は"Tracer"の文字を1文字ずつ循環表示 */
        int labelIndex = (displayCycleCount / Config::LABEL_CHANGE_CYCLES) % TRACER_LABEL_LEN;
        if(labelIndex != prevLabelIndex) {
            robot.showChar(TRACER_LABEL[labelIndex]);
            prevLabelIndex = labelIndex;
        }

        displayCycleCount++;
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    tracer.terminate();
    return true;
}
