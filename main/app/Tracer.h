#ifndef TRACER_H_
#define TRACER_H_

#include "Robot.h"
#include "ColorJudge.h"
#include "Pid.h"
#include "Config.h"

/**
 * ライントレースを行うクラス
 */
class Tracer {
public:
    Tracer(Robot& robot);
    void run();
    void terminate();
    bool isOnBlue();  // 青色ライン上にいるかを判定。

private:
    Robot& robot;
    Pid pid;  // 反射率がConfig::TRACER_TARGET_REFLECTIONに近づくよう左右パワー差を計算する

    int8_t blueCount = 0;  // カラーセンサが青色と判定した回数
};

#endif  // !TRACER_H_
