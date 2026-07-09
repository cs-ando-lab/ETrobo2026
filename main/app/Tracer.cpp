#include "Tracer.h"

Tracer::Tracer(Robot& robot)
    : robot(robot),
      pid(Config::TRACER_KP, Config::TRACER_KI, Config::TRACER_KD, Config::TRACER_TARGET_REFLECTION) {
}

void Tracer::terminate() {
    robot.stop();
}

void Tracer::run() {
    // Pidは偏差を「目標値 - 現在値」で計算するため、符号を反転して使う
    // 実際の呼び出し周期(LINE_TRACE_POLL_INTERVAL_US)をdeltaSecとして渡す
    constexpr float DELTA_SEC = Config::LINE_TRACE_POLL_INTERVAL_US / 1000000.0f;
    float turn = -pid.calculate(robot.getReflection(), DELTA_SEC);  // 比例制御の調整値を求める
    int pwm_l = Config::TRACER_PWM - turn;                          // 基準値と調整値を使って操作量を求める
    int pwm_r = Config::TRACER_PWM + turn;
    robot.setMotorPower(pwm_l, pwm_r);
}

bool Tracer::isOnBlue() {
    if(robot.getColor() == ColorJudge::Color::BLUE) {
        blueCount++;
    } else {
        blueCount = 0;
    }
    return (blueCount >= Config::TRACER_BLUE_DETECTION_COUNT);  // カラーセンサの取得値が一定期回数青色か判定
}
