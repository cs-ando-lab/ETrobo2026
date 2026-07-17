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
    Tracer tracer(robot);
    // ガレージ直前のy字路から青ラインまでライントレース
    tracer.setPwm(Config::ETRALLY_LINE_TRACE_DEFAULT_POWER);
    while(1) {
        static int blueCount = 0;
        if(robot.isOnColor(ColorJudge::Color::BLUE, blueCount))
            break;

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    tracer.terminate();

    // ETラリー開始
    for(const Gate& currentGate : gatesSequence) {
        static bool isFirstSequence = true;  // ラリーの最初だけ、方向が違うため最初だけ動きを変える。
        if(isFirstSequence) {
            // 青ライン到達後90°左折(Lコース)
            turn(robot, 90.0f * CourseConfig::sign(), Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL);
            isFirstSequence = false;
        } else {
            // 青ライン到達後90°右折(Lコース)
            turn(robot, -90.0f * CourseConfig::sign(), Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL);
        }

        // 蛇行走行で黒ラインの捜索
        robot.runWavingUntilColor(ColorJudge::Color::BLACK, Config::ETRALLY_WAVING_SPEED);

        // ゲートのある行に対応する色まで行って、90°右回転
        goToGateRow(currentGate, tracer);

        // ゲートを通過する
        runThroughGate(currentGate);

        // ゲートを通過したあと、スタート位置に戻る。
        returnToRallyStart(currentGate, tracer);
    }

    // フォースボタン押すと次のゲートに進む
    while(robot.isForceSensorPressed()) {
        ;
    }
    while(!robot.isForceSensorPressed()) {
        ;
    }
}

void RallyTask::run() {
    // TODO: 担当者が実装する（赤ゲート → 青ゲート → 黄ゲートの順に通過 → ラインに戻る）
    Tracer tracer(robot);

    // ガレージ直前のy字路から青ラインまでライントレース
    tracer.setPwm(Config::ETRALLY_LINE_TRACE_DEFAULT_POWER);
    while(1) {
        static int blueCount = 0;
        if(robot.isOnColor(ColorJudge::Color::BLUE, blueCount))
            break;

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    tracer.terminate();

    // ETラリー開始
    for(const Gate& currentGate : gatesSequence) {
        static bool isFirstSequence = true;  // ラリーの最初だけ、方向が違うため最初だけ動きを変える。
        if(isFirstSequence) {
            // 青ライン到達後90°左折(Lコース)
            turn(robot, 90.0f * CourseConfig::sign(), Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL);
            isFirstSequence = false;
        } else {
            // 青ライン到達後90°右折(Lコース)
            turn(robot, -90.0f * CourseConfig::sign(), Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL);
        }

        // 蛇行走行で黒ラインの捜索
        robot.runWavingUntilColor(ColorJudge::Color::BLACK, Config::ETRALLY_WAVING_SPEED);

        // ゲートのある行に対応する色まで行って、90°右回転
        goToGateRow(currentGate, tracer);

        // ゲートを通過する
        runThroughGate(currentGate);

        // ゲートを通過したあと、スタート位置に戻る。
        returnToRallyStart(currentGate, tracer);
    }
}

// TODO: ターゲットカラーがwhiteの時の処理も考える。
void RallyTask::goToGateRow(Gate gate, Tracer& tracer) {
    // 黒ライン上ではライントレースを行い、色付き円の場所では停止
    // 円の色がターゲットカラーの場合停止して右回転
    // [1] 黒ライン上をライントレース
    // [2] 色を検知したらストップ
    // [3] 目的の色でなければ黒を見つけるまで蛇行走行して[1]に戻る。
    ColorJudge::Color currentColor = ColorJudge::Color::UNKNOWN;
    ColorJudge::Color targetColor = getTargetRowColor(gate);
    int targetColorCount = 0;

    tracer.setConfig(Config::TRACER_KP, 0.0, 0.0, Config::TRACER_TARGET_REFLECTION, Config::ETRALLY_LINE_TRACE_DEFAULT_POWER);
    tracer.setEdge(CourseConfig::isLeftCourse() ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);
    while(1) {
        while(1) {  // [1]
            int colorNum = 4;
            ColorJudge::Color stopColors[] = { ColorJudge::Color::GREEN,
                                               ColorJudge::Color::YELLOW,
                                               ColorJudge::Color::RED,
                                               ColorJudge::Color::BLUE };
            if(robot.isOnColors(stopColors, colorNum, targetColorCount, 1)) {  // stableCountが3だと、色付き円に到着した際にすぐ止まらないため走行体の向きが斜めになる可能性があるため、1にしている。
                currentColor = robot.getColor();
                break;  // [2]
            }

            tracer.run();
            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
        }
        if(currentColor == targetColor)
            break;

        robot.runWavingUntilColor(ColorJudge::Color::BLACK, Config::ETRALLY_WAVING_SPEED);  // [3]
    }
    // 目的の色であれば停止して90°右に回転(Lコース)
    tracer.terminate();
    turn(robot, -90.0f * CourseConfig::sign(), Config::COLOR_CIRCLE_RADIUS + Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL);
    dly_tsk(Config::ETRALLY_DELAY);  // すぐに動き出すとずれが生じる可能性が高くなるので、少しディレイを入れている。
}

void RallyTask::runThroughGate(Gate gate) {
    if(gate.color == GateColor::RED || gate.color == GateColor::YELLOW) {  // 赤ゲートor黄ゲートの場合
        // [1] ゲートの列まで直進
        syslog(LOG_NOTICE, "RED or YELLOW - Xmm: [%d]", toXmm(gate));  // TODO: debug終われば消す
        robot.driveStraight(toXmm(gate) + Config::ETRALLY_THROUGH_GATE_ADJUSTMENT_DISTANCE, Config::ETRALLY_DEFAULT_SPEED);
        // [2] ゲートの列に到着したら90°右に回転
        turn(robot, -90 * CourseConfig::sign(), 0, Config::ETRALLY_DELAY);
        // [3] ETラリーフィールドの端まで下に直進（距離指定（変化））
        syslog(LOG_NOTICE, "RED or YELLOW - Ymm: [%d]", toYmm(gate));  // TODO: debug終われば消す
        robot.driveStraight(toYmm(gate), Config::ETRALLY_DEFAULT_SPEED);
        // [4] 黒を探知するまで直進
        robot.runStraightUntilColor(ColorJudge::Color::BLACK);
        // [5] 探知したら右に90°曲がってライントレース
        turn(robot, -90 * CourseConfig::sign(), Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL);
        // 終了
    } else if(gate.color == GateColor::BLUE) {                                        // 青ゲートの場合
                                                                                      // [1] ETラリーフィールドの端まで直進（距離指定（固定））
        syslog(LOG_NOTICE, "BLUE - Xmm: [%d]", (Config::ETRALLY_UNIT_DISTANCE * 5));  // TODO: debug終われば消す

        robot.driveStraight((Config::ETRALLY_UNIT_DISTANCE * 5), Config::ETRALLY_DEFAULT_SPEED);
        // [2] 黒もしくは青を探知するまで直進
        int colorCount = 2;
        ColorJudge::Color colors[colorCount] = { ColorJudge::Color::BLACK, ColorJudge::Color::BLUE };
        robot.runStraightUntilColors(colors, colorCount);
        // [3] 探知したら右に90°曲がってライントレース
        turn(robot, -90 * CourseConfig::sign(), Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL);
        // 終了
    } else {
        // 受け取ったゲートの情報が不正
        syslog(LOG_ERROR, "ERROR[runThroughGate]: invalid gate color");
    }
}

void RallyTask::returnToRallyStart(Gate gate, Tracer& tracer) {  // ゲート通過後のラインからスタート位置に戻る
    // tracerのパラメータをデフォルトに戻す。
    tracer.setConfig(Config::TRACER_KP, Config::TRACER_KI, Config::TRACER_KD, Config::TRACER_TARGET_REFLECTION, Config::ETRALLY_LINE_TRACE_FAST_POWER);
    tracer.setEdge(CourseConfig::isLeftCourse() ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT);

    if(gate.color == GateColor::RED || gate.color == GateColor::YELLOW) {
        int onBlueCount = 0;

        // ライントレースでスタート位置へ
        while(1) {
            if(robot.isOnColor(ColorJudge::Color::BLUE, onBlueCount))
                break;

            tracer.run();

            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
        }
    } else if(gate.color == GateColor::BLUE) {  // 青ゲートの場合
        // カーブ前の青ラインは全部無視する。（青ラインの通過をカウントして判定）
        int blueIgnoreCount;       // 無視する青ラインの回数
        int throughBlueCount = 0;  // 通った青ラインの数
        int onBlueCount = 0;       // 青ライン上にいると判定するためのカウンタ
        int onBlackCount = 0;      // 黒ライン上にいると判定するためのカウンタ
        bool blueFlag = false;     // 青を通り過ぎて黒を検知待ちであるかどうかの状態

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
            if(robot.isOnColor(ColorJudge::Color::BLUE, onBlueCount)) {
                // ライントレース終了条件
                if(throughBlueCount >= blueIgnoreCount)
                    break;

                // 青フラグが false → true になれば青に侵入したとみなす。
                if(!blueFlag) {
                    blueFlag = true;
                }
            } else if(robot.getColor() == ColorJudge::Color::BLACK) {
                // 青フラグが true → false になれば青を通過したとみなす。
                if(blueFlag) {
                    throughBlueCount++;
                    blueFlag = false;
                }
            }

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
    return (gate.leftLeg.col * Config::ETRALLY_UNIT_DISTANCE);
}

int RallyTask::toYmm(Gate gate) {  // ゲートの上側からラリーフィールドの端（下）までの距離[mm]を返す
    return ((6 - gate.leftLeg.row) * Config::ETRALLY_UNIT_DISTANCE);
}

void RallyTask::turn(Robot robot, float degree, int adjustmentDistance, int delayTime) {
    dly_tsk(delayTime);                                                          // 直前のモータの動きによって正確性に影響が出ないようにdelayを挟む
    robot.driveStraight((int)(adjustmentDistance), Config::ETRALLY_SLOW_SPEED);  // 回転軸の位置を調整
    robot.turnByImu(degree);
    dly_tsk(delayTime);  // モータの動きによって直後の動きの正確性に影響が出ないようにdelayを挟む
}
