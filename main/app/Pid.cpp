#include "Pid.h"

Pid::Pid(float kp, float ki, float kd, float target)
    : kp(kp),
      ki(ki),
      kd(kd),
      target(target) {
}

void Pid::setGain(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void Pid::setTarget(float newTarget) {
    target = newTarget;
}

void Pid::restartFromNextSample() {
    pendingRestart = true;
}

void Pid::resetIntegral() {
    integral = 0.0f;
}

void Pid::reset() {
    prevDeviation = 0.0f;
    integral = 0.0f;
    filteredDerivative = 0.0f;
}

float Pid::calculate(float currentValue, float deltaSec) {
    if(deltaSec <= 0.0f) {
        deltaSec = 0.01f;
    }

    float deviation = target - currentValue;

    if(pendingRestart) {
        // 再開の初回。前回偏差を今回の偏差で埋めることで、存在しない変化からDを作らない。
        // 初回はPだけを出し、積分もこの周期からやり直す
        pendingRestart = false;
        prevDeviation = deviation;
        integral = 0.0f;
        filteredDerivative = 0.0f;
        lastP = kp * deviation;
        lastI = 0.0f;
        lastD = 0.0f;
        return lastP;
    }

    // 積分項（台形積分）。暴走を防ぐため上下限でクランプする
    integral += (deviation + prevDeviation) * deltaSec / 2.0f;
    if(integral > Config::PID_INTEGRAL_LIMIT) {
        integral = Config::PID_INTEGRAL_LIMIT;
    } else if(integral < -Config::PID_INTEGRAL_LIMIT) {
        integral = -Config::PID_INTEGRAL_LIMIT;
    }

    // 微分項。ノイズによる急激な変化を抑えるためローパスフィルタをかける
    float derivative = (deviation - prevDeviation) / deltaSec;
    filteredDerivative = Config::PID_DERIVATIVE_FILTER_ALPHA * derivative
                         + (1.0f - Config::PID_DERIVATIVE_FILTER_ALPHA) * filteredDerivative;

    prevDeviation = deviation;

    lastP = kp * deviation;
    lastI = ki * integral;
    lastD = kd * filteredDerivative;

    return lastP + lastI + lastD;
}
