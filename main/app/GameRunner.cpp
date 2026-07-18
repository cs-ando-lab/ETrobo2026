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
    while(true) {
        if(!robot.isForceSensorPressed())
            break;
    }

    // モード管理用の配列と変数
    const char modeChars[] = { 'S', 'D', 'R' };
    const int MODE_MAX = 2;
    int startMode = 0;  // 0:相撲(S), 1:デリバリー(D), 2:ラリー(R)

    // 試走会用のモード切替関数、汚いので今後捨てます
    robot.showChar(modeChars[startMode]);
    while(true) {
        // 右ボタン：+1して次のモードへ
        if(robot.isRightButtonPressed()) {
            startMode++;
            if(startMode > MODE_MAX) {
                startMode = 0;
            }
            robot.showChar(modeChars[startMode]);
            robot.beep(100);
            dly_tsk(300 * 1000);
        }

        else if(robot.isLeftButtonPressed()) {
            startMode--;
            if(startMode < 0) {
                startMode = MODE_MAX;
            }
            robot.showChar(modeChars[startMode]);
            robot.beep(100);
            dly_tsk(300 * 1000);
        }

        else if(robot.isForceSensorPressed()) {
            robot.beep(500);
            robot.off();
            break;
        }

        dly_tsk(Config::MOTION_POLL_INTERVAL_US);
    }

    // 2. LAPゲートまでライントレース → ET相撲
    if(startMode <= 0) {
        if(!lineTraceUntilLap()) {
            return;
        }
        SumoTask sumo(robot);
        sumo.run();
    }

    // 3. LAPゲートまでライントレース → ボトルデリバリー
    if(startMode <= 1) {
        if(!lineTraceUntilLap()) {
            return;
        }
        DeliveryTask delivery(robot);
        delivery.run();
    }

    // 4. LAPゲートまでライントレース → ETラリー
    if(startMode <= 2) {
        RallyTask rally(robot);
        rally.test();
        return;
    }

    // 5. ゴール
    if(!lineTraceUntilLap()) {
        return;
    }
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
