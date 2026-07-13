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
    robot.runUntilColor(Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, Robot::DriveMode::WAVING, ColorJudge::Color::BLACK);

    /*  ↓↓↓ ここの機能はまとめて関数にした方がよさそう ↓↓↓ */
    // struct GatePositions = {Gate red, Gate blue, Gate yellow}
    // GateLoop()

    // findRow()
    // 黒ライン上ではライントレースを行い、ゲートのある行に対応する色の場合は停止
    // [1] 黒ライン上をライントレース
    ColorJudge::Color currentColor;
    ColorJudge::Color targetColor = ColorJudge::Color::RED;
    while(1) {
        while(1) {
            currentColor = robot.getColor();
            if(currentColor == ColorJudge::Color::GREEN || currentColor == ColorJudge::Color::YELLOW || currentColor == ColorJudge::Color::RED || currentColor == ColorJudge::Color::BLUE)
                // [2] 色を検知したらストップ
                break;
            tracer.run();
            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
        }
        if(currentColor == targetColor)
            break;
        robot.runUntilColor(Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, Robot::DriveMode::WAVING, ColorJudge::Color::BLACK);
    }

    // [3] 目的の色でなければ黒を見つけるまで蛇行走行して[1]に戻る。

    // ゲート通過後
    // 赤or黄の場合はコース下のラインまで直進
    // 青の場合はコース右のラインまで直進

    /* ↑↑↑ ここの機能はまとめて関数にした方がよさそう ↑↑↑ */
}
