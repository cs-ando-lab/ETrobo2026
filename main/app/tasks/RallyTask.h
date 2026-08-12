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

    float referenceGyroYaw = 0.0f;

    // ゲートを通る順番を保持する配列
    static const RallyTypes::Gate gatesSequence[3];

    void turn(float degree, int delayTimeUs = 100 * 1000);
    float calculateTurnAngle(float degree);                                                                               // 基準角度に対する角度を受け取り、現時点のgetHeading角度との差を求める
    void traceLineforDistance(float distance, Tracer tracer);                                                             // 指定した距離までライントレースを実施
    void calibrateHeadingByLineTrace(Tracer tracer);                                                                      // 低速ライントレースを行い、ライントレース終盤のジャイロ角を基準角として設定する。
    void followNodeSegments(std::vector<RallyTypes::Segment> segments, int speed = Config::ETRALLY_DEFAULT_DRIVE_SPEED);  // vector<RallyTypes::Segment>で指定された通りのルートを走行する。

    void logAngle(const char* s);
};

#endif  // !RALLYTASK_H_
