#include "RallyTask.h"

#include "kernel.h" /* dly_tskのため */
#include "t_syslog.h"

RallyTask::RallyTask(Robot& robot)
    : robot(robot) {
}

void RallyTask::run() {
    // TODO: 担当者が実装する（赤ゲート → 青ゲート → 黄ゲートの順に通過 → ラインに戻る）
    Tracer tracer(robot);

    // ガレージ直前のy字路から青ラインまでライントレース
    while(1) {
        static int blueCount = 0;
        if(robot.getColor() == ColorJudge::Color::BLUE)
            blueCount++;
        if(blueCount > 3)
            break;
        tracer.setPwm(35);
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    tracer.terminate();

    // ゲートを通る順番を保持する配列
    const Gate gatesSequence[] = {
        RALLY_GATE_POSITIONS.red,
        RALLY_GATE_POSITIONS.blue,
        RALLY_GATE_POSITIONS.yellow
    };

    // ETラリー開始
    for(const Gate& currentGate : gatesSequence) {
        // ラリーの最初だけ、方向が違うため
        static bool isFirstSequence = true;
        if(isFirstSequence) {
            // 青ライン到達後90°左折
            robot.driveStraight(50, 100);  // 青ライン中心に調整
            robot.turn(-90.0f, Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);
            isFirstSequence = false;
        } else {
            // 青ライン到達後90°右折
            robot.driveStraight(45, 100);  // 青ライン中心に調整
            robot.turn(90.0f, Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);
        }

        // 蛇行走行で黒ラインの捜索
        robot.runUntilColor(ColorJudge::Color::BLACK, 150, Robot::DriveMode::WAVING);

        // ゲートのある行に対応する色まで行って、90°右回転
        goToGateRow(currentGate, tracer);

        // ゲートを通過する
        runThroughGate(currentGate);

        // ゲートを通過したあと、スタート位置に戻る。
        returnRallyStart(currentGate, tracer);
    }
}

// TODO: ターゲットカラーがwhiteの時の処理も考える。
void RallyTask::goToGateRow(Gate gate, Tracer& tracer) {
    // 黒ライン上ではライントレースを行い、ゲートのある行に対応する色の場合は停止
    ColorJudge::Color currentColor = ColorJudge::Color::UNKNOWN;
    ColorJudge::Color targetColor = getTargetRowColor(gate);

    tracer.setConfig(Config::TRACER_KP, 0.0, 0.0, Config::TRACER_TARGET_REFLECTION, 30);
    while(1) {
        // [1] 黒ライン上をライントレース
        while(1) {
            // [2] 色を検知したらストップ
            currentColor = robot.getColor();
            bool stopCondition = currentColor == ColorJudge::Color::BLUE
                                 || currentColor == ColorJudge::Color::RED
                                 || currentColor == ColorJudge::Color::YELLOW
                                 || currentColor == ColorJudge::Color::GREEN;
            if(stopCondition)
                break;

            tracer.run();
            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
        }
        if(currentColor == targetColor)
            break;
        // [3] 目的の色でなければ黒を見つけるまで蛇行走行して[1]に戻る。
        robot.runUntilColor(ColorJudge::Color::BLACK, Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, Robot::DriveMode::STRAIGHT);
    }
    // 目的の色であれば停止して90°右に回転
    tracer.terminate();
    robot.driveStraight(80, 100);
    robot.turn(88, 100);
}

void RallyTask::runThroughGate(Gate gate) {
    if(gate.color == GateColor::RED || gate.color == GateColor::YELLOW) {  // 赤ゲートor黄ゲートの場合
        // [1] ゲートの列まで直進
        robot.driveStraight(toXmm(gate), 150);
        // [2] ゲートの列に到着したら90°右に回転
        robot.turn(90, 100);
        // [3] ETラリーフィールドの端まで下に直進（距離指定（変化））
        robot.driveStraight(toYmm(gate), 150);
        // [4] 黒を探知するまで直進
        robot.runUntilColor();
        // [5] 探知したら右に90°曲がってライントレース
        robot.turn(90, 100);
        // 終了
    } else if(gate.color == GateColor::BLUE) {  // 青ゲートの場合
        // [1] ETラリーフィールドの端まで直進（距離指定（固定））
        robot.driveStraight(240 * 5);  // あとでこの引数の値も定数として定義（ラリーフィールドディスタンス とか？）
        // [2] 黒もしくは青を探知するまで直進
        int colorCount = 2;
        ColorJudge::Color colors[colorCount] = { ColorJudge::Color::BLACK, ColorJudge::Color::BLUE };
        robot.runUntilColors(colors, colorCount);
        // [3] 探知したら右に90°曲がってライントレース
        robot.turn(90, 100);
        // 終了
    } else {
        // 受け取ったゲートの情報が不正
        syslog(LOG_ERROR, "ERROR[runThroughGate]: invalid gate color");
    }
}

void RallyTask::returnRallyStart(Gate gate, Tracer& tracer) {  // ゲート通過後のラインからスタート位置に戻る
    // tracerのパラメータをデフォルトに戻す。
    tracer.setConfig(Config::TRACER_KP, Config::TRACER_KI, Config::TRACER_KD, Config::TRACER_TARGET_REFLECTION, 35);

    if(gate.color == GateColor::RED || gate.color == GateColor::YELLOW) {
        // ライントレースでスタート位置へ
        while(1) {
            if(robot.getColor() == ColorJudge::Color::BLUE)
                break;

            tracer.run();

            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
        }
    } else if(gate.color == GateColor::BLUE) {  // 青ゲートの場合
        // カーブ前の青ラインは全部無視する。（青ラインの通過をカウントして判定）
        int blueIgnoreCount;  // 無視する青ラインの回数
        int blueCount;        // 通った青ラインの数
        ColorJudge::Color currentCollor = ColorJudge::Color::UNKNOWN;
        ColorJudge::Color previousCollor = ColorJudge::Color::UNKNOWN;

        // ゲートの色によってblueIngnoreCountの数値を調整
        ColorJudge::Color rowColor = getTargetRowColor(gate);
        switch(rowColor) {
            case ColorJudge::Color::BLUE:
                blueIgnoreCount = 3;
                break;
            case ColorJudge::Color::RED:
                blueIgnoreCount = 2;
                break;
            case ColorJudge::Color::YELLOW:
                blueIgnoreCount = 1;
                break;
            default:
                blueIgnoreCount = 0;
                break;
        }

        // ライントレースでスタート位置へ
        while(1) {
            currentCollor = robot.getColor();

            // 青ラインから黒ラインへ移ったら、blueCountを増やす
            if(previousCollor == ColorJudge::Color::BLUE && currentCollor == ColorJudge::Color::BLACK) {
                blueCount++;
            }

            // ライントレース終了条件
            if(currentCollor == ColorJudge::Color::BLUE) {
                if(blueCount >= blueIgnoreCount)
                    break;
            }

            previousCollor = currentCollor;

            tracer.run();

            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
        }
        // 終了
    }

    tracer.terminate();
}

ColorJudge::Color RallyTask::getTargetRowColor(Gate gate) {
    int gateRowMax = gate.leftLeg.row;
    if(gateRowMax < gate.rightLeg.row)
        gateRowMax = gate.rightLeg.row;
    switch(gateRowMax) {
        case 1:
            return ColorJudge::Color::WHITE;
        case 2:
            return ColorJudge::Color::BLUE;
        case 3:
            return ColorJudge::Color::RED;
        case 4:
            return ColorJudge::Color::YELLOW;
        case 5:
            return ColorJudge::Color::GREEN;
        default:
            syslog(LOG_NOTICE, "ERROR[getRowColor]: invalid gate position");
            return ColorJudge::Color::UNKNOWN;
            break;
    }
}

int RallyTask::toXmm(Gate gate) {  // 左(右)のラインからゲートの位置までの距離[mm]を返す
    int gridDistance = 240;        //[mm] TODO:後でコンフィグに移行
    return (gate.leftLeg.col * gridDistance);
}

int RallyTask::toYmm(Gate gate) {  // ゲートの上側からラリーフィールドの端（下）までの距離[mm]を返す
    int gridDistance = 240;        //[mm] 後でコンフィグに移行
    return ((6 - gate.leftLeg.row) * gridDistance);
}
