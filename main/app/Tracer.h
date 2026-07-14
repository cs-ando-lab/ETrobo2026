#ifndef TRACER_H_
#define TRACER_H_

#include <cstdint>
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

    // Pidのパラメータを保持する構造体
    struct PidConfig {
        float kp;
        float ki;
        float kd;
        int32_t targetReflection;
        int basePwm;
    };

    enum struct Edge {
        LEFT = -1,
        RIGHT = 1
    };

    // Pidのパラメータを更新
    void setConfig(float newKp, float newKi, float newKd, int32_t newTarget, int newPwm);
    void setTarget(int32_t newTarget);
    void setPwm(int newPwm);

    //
    void setEdge(Edge newEdge);

private:
    Robot& robot;
    Pid pid;  // 反射率がConfig::TRACER_TARGET_REFLECTIONに近づくよう左右パワー差を計算する
    PidConfig pidConfig;
    Edge edge;

    void updateConfig(const PidConfig& newConfig);  // PidクラスのsetGain, setTargetを呼び出し、パラメータを更新。
    int8_t blueCount = 0;                           // カラーセンサが青色と判定した回数
};

#endif  // !TRACER_H_
