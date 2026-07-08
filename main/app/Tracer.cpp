#include "Tracer.h"

Tracer::Tracer(DriveBase& driveBase, ColorSensor& colorSensor)
    : driveBase(driveBase),
      colorSensor(colorSensor) {
}

void Tracer::init() {
    colorSensor.lightOn();
}

void Tracer::terminate() {
    driveBase.stop();
}

void Tracer::run() {
    float turn = calcPropValue();  // 比例制御の調整値を求める
    int pwm_l = PWM - turn;        // 基準値と調整値を使って操作量を求める
    int pwm_r = PWM + turn;
    driveBase.setPower(pwm_l, pwm_r);
}

bool Tracer::isOnBlue() {
    ColorSensor::HSV hsv;
    colorSensor.getColor(hsv, true);

    // 友達のコード（hsv.h == TARGET_HUE）を、俺らのフィルターに差し替え！
    if(hsv.h >= 200 && hsv.h <= 280 && hsv.s > 60 && hsv.v > 20) {
        blueCount++;
    } else {
        blueCount = 0;
    }
    
    return (blueCount >= BLUE_DETECTION_COUNT);  
}

float Tracer::calcPropValue() const {
    int diff = colorSensor.getReflection() - TARGET;  // 偏差を求める
    return (KP * diff + BIAS);                        // 調整値を計算して返す
}
