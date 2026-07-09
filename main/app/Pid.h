#ifndef PID_H_
#define PID_H_

#include "Config.h"

/**
 * 目標値と現在値の差（偏差）から、P/I/D制御の操作量を計算する汎用PID制御クラス。
 * ライントレースの左右パワー配分、直進時の左右差補正、旋回など、
 * 「目標値に近づけたい」場面ならどこでも使い回せる。
 */
class Pid {
public:
    // kp/ki/kd: 各制御項のゲイン, target: 目標値
    Pid(float kp, float ki, float kd, float target);

    // ゲインを変更する
    void setGain(float newKp, float newKi, float newKd);

    // 目標値を変更する
    void setTarget(float newTarget);

    // 現在値を渡し、操作量（偏差 = target - currentValue に基づく）を計算する
    // deltaSec: 前回計算してからの経過時間[秒]（省略時は10ms周期を想定）
    float calculate(float currentValue, float deltaSec = 0.01f);

    // 積分値・微分値の内部状態をリセットする（目標値を大きく変える時などに使う）
    void reset();

private:
    float kp;
    float ki;
    float kd;
    float target;

    float prevDeviation = 0.0f;       // 前回の偏差
    float integral = 0.0f;            // 偏差の累積(積分項)
    float filteredDerivative = 0.0f;  // ローパスフィルタをかけた微分項
};

#endif  // !PID_H_
