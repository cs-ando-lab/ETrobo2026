#ifndef RALLYTASK_H_
#define RALLYTASK_H_

#include "Robot.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"
#include "RallyTask/RallyTypes.h"
#include "RallyTask/RallyRoute.h"
#include "RallyTask/HeadingCalibration.h"
#include <vector>

#include <libcpp/spike/Clock.h>  // debug用

/**
 * ETラリーの処理を行うクラス。
 * run()を1回呼べば、赤→青→黄のゲートを順番に通過する。
 */
class RallyTask {
public:
    RallyTask(Robot& robot);
    void run();
    void test();

private:
    Robot& robot;
    bool debug = false;

    float referenceGyroYaw = 0.0f;  // 基準角

    float turnToDirection(RallyTypes::Direction direction, int speedDegPerSec = Config::ETRALLY_DEFAULT_TURN_SPEED);  // 基準角を北としてDirectionの方角へ回転する。（turnのラッパー関数）
    int moveToDirection(int distanceMm,
                        RallyTypes::Direction direction,
                        int speedDegPerSec = Config::ETRALLY_DEFAULT_DRIVE_SPEED);  // 基準角を北としてDirectionの方角へ直進する（robot.driveStraightのラッパー関数）
    float calculateTurnAngle(float degree);                                         // 基準角度に対する角度を受け取り、現時点のgetHeading角度との差を求める
    float getDirectionDegrees(RallyTypes::Direction direction) const;               // 方角から基準角に対する相対角度を取得

    void traceLineforDistance(float distance, Tracer tracer);  // 指定した距離までライントレースを実施
    void calibrateHeadingByLineTrace(Tracer tracer);           // 低速ライントレースを行い、ライントレース終盤のジャイロ角を基準角として設定する。
    void followNodeSegments(std::vector<RallyTypes::Segment> segments,
                            int turnSpeed = Config::ETRALLY_DEFAULT_TURN_SPEED,
                            int driveSpeed = Config::ETRALLY_DEFAULT_DRIVE_SPEED);  // vector<RallyTypes::Segment>で指定された通りのルートを走行する。
};

#endif  // !RALLYTASK_H_
