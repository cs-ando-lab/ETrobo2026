#include "RallyTask.h"
#include <cstdlib>
#include <cmath>

#include "kernel.h" /* dly_tskのため */
#include "t_syslog.h"

// 各色ゲートの座標
const RallyTypes::Gate RallyTask::gatesSequence[3] = {
    { RallyTypes::GateColor::RED,
      { Config::ETRALLY_RED_GATE_LEFT_ROW, Config::ETRALLY_RED_GATE_LEFT_COL },
      { Config::ETRALLY_RED_GATE_RIGHT_ROW, Config::ETRALLY_RED_GATE_RIGHT_COL } },
    { RallyTypes::GateColor::BLUE,
      { Config::ETRALLY_BLUE_GATE_LEFT_ROW, Config::ETRALLY_BLUE_GATE_LEFT_COL },
      { Config::ETRALLY_BLUE_GATE_RIGHT_ROW, Config::ETRALLY_BLUE_GATE_RIGHT_COL } },
    { RallyTypes::GateColor::YELLOW,
      { Config::ETRALLY_YELLOW_GATE_LEFT_ROW, Config::ETRALLY_YELLOW_GATE_LEFT_COL },
      { Config::ETRALLY_YELLOW_GATE_RIGHT_ROW, Config::ETRALLY_YELLOW_GATE_RIGHT_COL } }
};

RallyTask::RallyTask(Robot& robot)
    : robot(robot) {
}

// testはテスト用、runが本番用
void RallyTask::test() {
    Tracer tracer(robot);
    referenceGyroYaw = robot.getHeading();
    syslog(LOG_NOTICE, "referenceGyroYaw : %d [°]", static_cast<int>(referenceGyroYaw));

    // [3-1] - ルート算出フェーズ
    /* 格子上のルートを求める */
    std::vector<RallyTypes::Node> path = { { 0, 5 }, { 1, 5 }, { 2, 5 }, { 2, 4 }, { 3, 4 }, { 3, 3 }, { 3, 2 }, { 2, 2 }, { 2, 3 }, { 1, 3 }, { 1, 2 }, { 0, 2 }, { 0, 3 }, { 0, 4 }, { 1, 4 }, { 1, 5 }, { 0, 5 } };
    RallyRoute rallyRoute;
    std::vector<RallyTypes::Segment> segments = rallyRoute.groupStraightSegments(path);
    // [3-2] - ゲート通過フェーズ
    /* 格子上を移動し、ゲート通過する */
    followNodeSegments(segments);

    return;
}

void RallyTask::run() {
    Tracer tracer(robot);

    // [1] - 基準角設定フェーズ
    /* 180°転回 */
    turn(180.0f * CourseConfig::sign());
    /* [a] 一定距離ライントレースを行う */
    traceLineforDistance(Config::ETRALLY_TRACE_BACK_DISTANCE, tracer);
    /* 180°転回 */
    turn(-180.0f * CourseConfig::sign());
    /** 直線上で正確性の高いライントレースを行う
     *  青ラインを探知するまで行う
     *  このライントレース中にIMUの方向をリセット(条件あり)
     *  IMUの方向をリセットできなかった場合は、距離を増やして[a]からやり直す(検討中)
     */
    calibrateHeadingByLineTrace(tracer);

    // [2] - 格子点移動フェーズ
    /* ジャイロ角が基準角に合うまで回転してラインに対して平行の向きに合わせる */
    turn(calculateTurnAngle(0.0f));
    /* 青ラインの右端から1/4の地点まで行く : runStraight(青ライン1/4[mm] - (秒速[mm/s] * 2 * 0.01[s])[mm]) */
    robot.driveStraightByImu(Config::DISTANCE_FROM_COLORCENSOR_TO_WHEEL + (Config::BLUE_LINE_LENGTH_MM / 4.0f), robot.getHeading(), Config::ETRALLY_SLOW_DRIVE_SPEED);
    /* 開始格子点まで行く : 90°右転回 → runStraight(開始格子点までの距離[mm]) */
    turn(calculateTurnAngle(90.0f));
    robot.driveStraightByImu(Config::BLUE_LINE_WIDTH_MM + Config::START_GRID_POINT_TO_START_LINE_MM, robot.getHeading(), Config::ETRALLY_SLOW_DRIVE_SPEED);

    // [3-1] - ルート算出フェーズ
    /* 格子上のルートを求める */
    std::vector<RallyTypes::Node> path = { { 0, 5 }, { 1, 5 }, { 2, 5 }, { 2, 4 }, { 3, 4 }, { 3, 3 }, { 3, 2 }, { 2, 2 }, { 2, 3 }, { 1, 3 }, { 1, 2 }, { 0, 2 }, { 0, 3 }, { 0, 4 }, { 1, 4 }, { 1, 5 }, { 0, 5 } };
    RallyRoute rallyRoute;
    std::vector<RallyTypes::Segment> segments = rallyRoute.groupStraightSegments(path);
    {  // debug
        syslog(LOG_NOTICE, "=== segments ===");
        for(RallyTypes::Segment segment : segments) {
            syslog(LOG_NOTICE, "start: (%d, %d)", segment.start.x, segment.start.y);
            syslog(LOG_NOTICE, "end: (%d, %d)", segment.end.x, segment.end.y);
            switch(segment.direction) {
                case RallyTypes::Direction::NORTH:
                    syslog(LOG_NOTICE, "direction: NORTH");
                    break;
                case RallyTypes::Direction::EAST:
                    syslog(LOG_NOTICE, "direction: EAST");
                    break;
                case RallyTypes::Direction::WEST:
                    syslog(LOG_NOTICE, "direction: SOUTH");
                    break;
                case RallyTypes::Direction::SOUTH:
                    syslog(LOG_NOTICE, "direction: WEST");
                    break;
                default:
                    break;
            }
            dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US * 10);
        }
        syslog(LOG_NOTICE, "=== segments ===");
    }
    // [3-2] - ゲート通過フェーズ
    /* 格子上を移動し、ゲート通過する */
    followNodeSegments(segments);
    // [4] 終了フェーズ
    /* 開始格子点まで戻る */

    /*  */

    return;
}

void RallyTask::turn(float degree, int delayTimeUs) {
    dly_tsk(delayTimeUs);  // 直前のモータの動きによって正確性に影響が出ないようにdelayを挟む
    robot.turnByImu(degree, 150);
    dly_tsk(delayTimeUs);  // モータの動きによって直後の動きの正確性に影響が出ないようにdelayを挟む
}

float RallyTask::calculateTurnAngle(float degree) {
    return referenceGyroYaw + degree - robot.getHeading();
}

void RallyTask::traceLineforDistance(float distance, Tracer tracer) {
    Tracer::Edge edge = CourseConfig::isLeftCourse() ? Tracer::Edge::RIGHT : Tracer::Edge::LEFT;
    tracer.setEdge(edge);
    robot.resetMotorCounts();
    float traveledMm = 0.0f;

    while(1) {
        if(robot.isForceSensorPressed() || traveledMm > distance)
            break;
        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);

        int count = (robot.getLeftMotorCount() + robot.getRightMotorCount()) / 2;
        traveledMm = (std::abs(count) / 360.0f) * 2 * Config::PI * Config::WHEEL_RADIUS_MM;
    }

    tracer.terminate();
};

void RallyTask::calibrateHeadingByLineTrace(Tracer tracer) {
    Tracer::Edge edge = CourseConfig::isLeftCourse() ? Tracer::Edge::LEFT : Tracer::Edge::RIGHT;
    tracer.setEdge(edge);
    tracer.setLeftMotorOffset(2);
    tracer.setConfig(Config::ETRALLY_HEADING_CALIBRATION_KP, Config::ETRALLY_HEADING_CALIBRATION_KI, Config::ETRALLY_HEADING_CALIBRATION_KD, Config::TRACER_TARGET_REFLECTION, Config::ETRALLY_HEADING_CALIBRATION_PWM);

    int blueMatched = 0;
    HeadingCalibration headingCalib;

    while(1) {
        // 停止条件
        if(robot.isForceSensorPressed() || robot.isOnColor(ColorJudge::Color::BLUE, blueMatched)) {
            tracer.terminate();
            if(headingCalib.isSampleEnough()) {
                referenceGyroYaw = headingCalib.getReferenceGyroYaw();
            } else {
                referenceGyroYaw = robot.getHeading();
            }

            {  // debug用ブロック
                int gyroYaw100 = referenceGyroYaw * 100;
                syslog(LOG_NOTICE, "Reference Gyro Yaw : %d.%02d [°]", gyroYaw100 / 100, gyroYaw100 < 0 ? -gyroYaw100 % 100 : gyroYaw100 % 100);
            }
            break;
        }

        tracer.run();

        // サンプルをリングバッファに保存
        headingCalib.updateSample(robot.getHeading());

        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }
    return;
}

void RallyTask::followNodeSegments(std::vector<RallyTypes::Segment> segments, int speed) {
    for(RallyTypes::Segment segment : segments) {
        switch(segment.direction) {
            case RallyTypes::Direction::NORTH: {
                turn(calculateTurnAngle(90.0f));
                int edgeCount = segment.start.y - segment.end.y;
                logAngle("(NORTH)");
                syslog(LOG_NOTICE, "%d [mm]",
                       robot.driveStraightByImu(Config::RALLY_UNIT_DISTANCE_MM * edgeCount, referenceGyroYaw + (90.0f), speed));
                logAngle("(NORTH)");
                break;
            }
            case RallyTypes::Direction::EAST: {
                turn(calculateTurnAngle(180.0f));
                int edgeCount = segment.end.x - segment.start.x;
                logAngle("(EAST)");
                syslog(LOG_NOTICE, "%d [mm]",
                       robot.driveStraightByImu(Config::RALLY_UNIT_DISTANCE_MM * edgeCount, referenceGyroYaw + (180.0f), speed));
                logAngle("(EAST)");
                break;
            }
            case RallyTypes::Direction::SOUTH: {
                turn(calculateTurnAngle(-90.0f));
                int edgeCount = segment.end.y - segment.start.y;
                logAngle("(SOUTH)");
                syslog(LOG_NOTICE, "%d [mm]",
                       robot.driveStraightByImu(Config::RALLY_UNIT_DISTANCE_MM * edgeCount, referenceGyroYaw + (-90.0f), speed));
                logAngle("(SOUTH)");
                break;
            }
            case RallyTypes::Direction::WEST: {
                turn(calculateTurnAngle(0.0f));
                int edgeCount = segment.start.x - segment.end.x;
                logAngle("(WEST)");
                syslog(LOG_NOTICE, "%d [mm]",
                       robot.driveStraightByImu(Config::RALLY_UNIT_DISTANCE_MM * edgeCount, referenceGyroYaw + (0.0f), speed));
                logAngle("(WEST)");
                break;
            }
            default:
                syslog(LOG_ERROR, "ERROR: segment invalid direction");
                break;
        }
    }
}

void RallyTask::logAngle(const char* s) {  // debug用
    int gyroYaw100 = (robot.getHeading() - referenceGyroYaw) * 100;
    syslog(LOG_NOTICE, "%s Gyro Yaw : %d.%02d [°]", s, gyroYaw100 / 100, gyroYaw100 < 0 ? -gyroYaw100 % 100 : gyroYaw100 % 100);
}
