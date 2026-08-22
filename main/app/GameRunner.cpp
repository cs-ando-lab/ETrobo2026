#include "GameRunner.h"
#include "Calibrator.h"
#include "Tracer.h"
#include "tasks/DeliveryTask.h"
#include "tasks/RallyTask.h"
#include "tasks/test.h"
#include "Config.h"
#include "kernel.h"

GameRunner::GameRunner(Robot& robot)
    : robot(robot) {
}

void GameRunner::run() {
    // キャリブレーション（L/R選択、スタート待ち）
    Calibrator calib(robot);
    calib.run();
    while(true) {
        if(!robot.isForceSensorPressed())
            break;
    }

    // モード管理用の配列と変数
    const char modeChars[] = { 'O', 'D', 'R', 'T' };
    const int MODE_MAX = 3;
    int startMode = 0;  // 0:本番(O), 1:デリバリー(D), 2:ラリー(R), 3:テスト(T)

    // 試走会用のモード切替
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

    // 0. 本番走行
    if(startMode <= 0) {
        // LAPゲートまでライントレース
        if(!lineTraceUntilLap()) {
            return;
        }

        // ボトルデリバリー
        DeliveryTask delivery(robot);
        delivery.run();

        // ETラリー
        RallyTask rally(robot);
        rally.run();

        // ガレージ入庫
        if(!moveTowardGarageArea()) {
            return;
        }
    }

    // 1. LAPゲートまでライントレース → ボトルデリバリー
    if(startMode <= 1) {
        if(!lineTraceUntilLap()) {
            return;
        }
        DeliveryTask delivery(robot);
        delivery.run();
    }

    // 2. ETラリー
    if(startMode <= 2) {
        RallyTask rally(robot);
        rally.run();
    }

    // 3. テスト用（関数などを試す）
    if(startMode <= 3) {
        Test test(robot);
        test.run();
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
    robot.off();
    return true;
}

bool GameRunner::moveTowardGarageArea() {
    Tracer tracer(robot);

    /* ↓↓ 仮案 ↓↓ */
    bool straight = true;        // ガレージまで直接行く
    bool traceLine = !straight;  // ラインをトレースしてガレージまで行く

    if(straight) {  // ガレージまでそのまま直進
        // Todo：後でConfigに移す
        float BLUE_LINE_LEFT_TO_GARAGE = 5.364415f * Config::BLUE_LINE_LENGTH_MM;

        robot.driveStraightByImu(BLUE_LINE_LEFT_TO_GARAGE, robot.getHeading());
        robot.turnByImu(90 * CourseConfig::sign());
    }

    if(traceLine) {  // ライントレースでガレージまで向かう
        // Todo：後でConfigに移す
        float BLUE_LINE_LEFT_TO_JUNCTION_MM = 7.587079f * Config::BLUE_LINE_LENGTH_MM;
        float JUNCTION_TO_GARAGE_MM = 3.5f * Config::BLUE_LINE_LENGTH_MM;
        float GARAGE_ENTRANCE_TO_CENTER = 1.5f * Config::BLUE_LINE_LENGTH_MM;

        int initialLeftCount;
        int initialRightCount;
        float traveledMm;
        tracer.setEdge(CourseConfig::isLeftCourse() ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT);

        // 三叉路上までライントレース
        initialLeftCount = robot.getLeftMotorCount();
        initialRightCount = robot.getRightMotorCount();
        traveledMm = 0.0f;

        while(1) {
            /* センターボタンで安全停止 */
            if(robot.isCenterButtonPressed()) {
                tracer.terminate();
                return false;
            }

            /* 三叉路までの距離をライントレースしたら停止 */
            if(traveledMm >= BLUE_LINE_LEFT_TO_JUNCTION_MM) {
                break;
            }

            tracer.run();

            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);

            int leftCount = robot.getLeftMotorCount() - initialLeftCount;
            int rightCount = robot.getRightMotorCount() - initialRightCount;
            int count = (leftCount + rightCount) / 2;
            traveledMm = (count / 360.0f) * 2 * Config::PI * Config::WHEEL_RADIUS_MM;
        }
        tracer.terminate();

        // コース外側に90°回転
        robot.turnByImu(90 * CourseConfig::sign());
        // ラインの太さだけ直進
        robot.driveStraight(Config::BLUE_LINE_WIDTH_MM, 150);
        // ガレージ側に90°回転
        robot.turnByImu(90 * CourseConfig::sign());

        // 三叉路からガレージ手前までライントレース
        initialLeftCount = robot.getLeftMotorCount();
        initialRightCount = robot.getRightMotorCount();
        traveledMm = 0.0f;

        while(1) {
            /* センターボタンで安全停止 */
            if(robot.isCenterButtonPressed()) {
                tracer.terminate();
                return false;
            }

            /* 三叉路までの距離をライントレースしたら停止 */
            if(traveledMm >= JUNCTION_TO_GARAGE_MM) {
                break;
            }

            tracer.run();

            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);

            int leftCount = robot.getLeftMotorCount() - initialLeftCount;
            int rightCount = robot.getRightMotorCount() - initialRightCount;
            int count = (leftCount + rightCount) / 2;
            traveledMm = (count / 360.0f) * 2 * Config::PI * Config::WHEEL_RADIUS_MM;
        }
        tracer.terminate();

        // ガレージ中央まで進む
        robot.driveStraight(GARAGE_ENTRANCE_TO_CENTER);
    }

    return true;
}