#include "Tracer.h"

Tracer::Tracer()
    : leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true),
      rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true),
      colorSensor(EPort::PORT_E) {
}

void Tracer::init() {
    colorSensor.lightOn();
}

void Tracer::terminate() {
    leftWheel.stop();
    rightWheel.stop();
}

void Tracer::run() {
    float turn = calcPropValue();  // 比例制御の調整値を求める
    int pwm_l = PWM - turn;        // 基準値と調整値を使って操作量を求める
    int pwm_r = PWM + turn;
    leftWheel.setPower(pwm_l);
    rightWheel.setPower(pwm_r);
}

float Tracer::calcPropValue() const {
    int diff = colorSensor.getReflection() - TARGET;  // 偏差を求める
    return (KP * diff + BIAS);                        // 調整値を計算して返す
}
