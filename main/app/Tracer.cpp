#include "Tracer.h"

Tracer::Tracer(Robot& robot)
    : robot(robot) {
}

void Tracer::terminate() {
    robot.stop();
}

void Tracer::run() {
    float turn = calcPropValue();  // 比例制御の調整値を求める
    int pwm_l = PWM - turn;        // 基準値と調整値を使って操作量を求める
    int pwm_r = PWM + turn;
    robot.setMotorPower(pwm_l, pwm_r);
}

bool Tracer::isOnBlue() {
    if(robot.isOnBlue()) {
        blueCount++;
    } else {
        blueCount = 0;
    }
    return (blueCount >= BLUE_DETECTION_COUNT);  // カラーセンサの取得値が一定期回数青色か判定
}

float Tracer::calcPropValue() const {
    int diff = robot.getReflection() - TARGET;  // 偏差を求める
    return (KP * diff + BIAS);                  // 調整値を計算して返す
}
