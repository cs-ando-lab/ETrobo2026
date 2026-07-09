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

    return (kp * deviation) + (ki * integral) + (kd * filteredDerivative);
}
