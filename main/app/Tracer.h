#ifndef TRACER_H_
#define TRACER_H_

#include "Motor.h"
#include "ColorSensor.h"

using namespace spikeapi;

/**
 * ライントレースを行うクラス
 */
class Tracer {
public:
    Tracer();
    void init();
    void run();
    void terminate();
    bool isOnBlue();  // 青色ライン上にいるかを判定。

    const Motor& getLeftMotor() const { return leftWheel; }
    const Motor& getRightMotor() const { return rightWheel; }
    const ColorSensor& getColorSensor() const { return colorSensor; }

private:
    Motor leftWheel;
    Motor rightWheel;
    ColorSensor colorSensor;

    static constexpr float KP = 0.2f;      // 曲がりすぎ、曲がり切れないということがない丁度いい値が大体この辺り
    static constexpr int32_t TARGET = 60;  // 白と黒の反射率の中間値(（19+99）/ 2)
    static constexpr int32_t BIAS = 0;
    static constexpr int8_t PWM = 30;

    int8_t blueCount = 0;                              // カラーセンサが青色と判定した回数
    static constexpr int8_t BLUE_DETECTION_COUNT = 3;  // 青判定に必要な連続検出回数
    static constexpr uint16_t TARGET_HUE = 240;        // 近似したHUEの判定用基準値

    float calcPropValue() const;
};

#endif  // !TRACER_H_
