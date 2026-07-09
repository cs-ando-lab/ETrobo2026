#ifndef TRACER_H_
#define TRACER_H_

#include "Robot.h"

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

    static constexpr float KP = 0.2f;      // 曲がりすぎ、曲がり切れないということがない丁度いい値が大体この辺り
    static constexpr int32_t TARGET = 60;  // 白と黒の反射率の中間値(（19+99）/ 2)
    static constexpr int32_t BIAS = 0;
    static constexpr int8_t PWM = 30;

    int8_t blueCount = 0;                              // カラーセンサが青色と判定した回数
    static constexpr int8_t BLUE_DETECTION_COUNT = 3;  // 青判定に必要な連続検出回数

    float calcPropValue() const;
};

#endif  // !TRACER_H_
