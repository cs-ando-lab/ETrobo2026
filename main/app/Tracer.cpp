#include "Tracer.h"
#include <cmath>

Tracer::Tracer(Robot& robot)
    : robot(robot),
      pid(Config::TRACER_KP, Config::TRACER_KI, Config::TRACER_KD, Config::TRACER_TARGET_REFLECTION),
      pidConfig{ Config::TRACER_KP, Config::TRACER_KI, Config::TRACER_KD,
                 Config::TRACER_TARGET_REFLECTION, Config::TRACER_PWM },
      edge(Edge::RIGHT) {
}

void Tracer::terminate() {
    robot.stop();
}

void Tracer::run() {
    // Pidは偏差を「目標値 - 現在値」で計算するため、符号を反転して使う
    // 実際の呼び出し周期(LINE_TRACE_POLL_INTERVAL_US)をdeltaSecとして渡す
    constexpr float DELTA_SEC = Config::LINE_TRACE_POLL_INTERVAL_US / 1000000.0f;
    float turn = -pid.calculate(robot.getReflection(), DELTA_SEC);  // 比例制御の調整値を求める

    // |turn|が大きい（＝急に曲がろうとしている）ほど基準パワーから減速する。
    // 操舵の遅延を避けるため、turn自体は生値のまま使い、減速量の算出にのみ平滑化した値を使う。
    float turnMag = std::fabs(turn);
    filteredTurnMag = Config::TRACER_CURVE_TURN_FILTER_ALPHA * turnMag
                      + (1.0f - Config::TRACER_CURVE_TURN_FILTER_ALPHA) * filteredTurnMag;
    float minPwm = pidConfig.basePwm * Config::TRACER_CURVE_MIN_PWM_RATIO;
    float curvePwm = pidConfig.basePwm - Config::TRACER_CURVE_DECEL_GAIN * filteredTurnMag;
    if(curvePwm < minPwm)
        curvePwm = minPwm;
    else if(curvePwm > pidConfig.basePwm)
        curvePwm = pidConfig.basePwm;

    int pwm_l = static_cast<int>(curvePwm - (static_cast<int>(edge) * turn));  // 基準値と調整値を使って操作量を求める
    int pwm_r = static_cast<int>(curvePwm + (static_cast<int>(edge) * turn));
    robot.setMotorPower(pwm_l, pwm_r);
}

void Tracer::setConfig(float newKp, float newKi, float newKd, int32_t newTarget, int newPwm) {
    PidConfig newConfig = { newKp, newKi, newKd, newTarget, newPwm };
    updateConfig(newConfig);
}

void Tracer::setTarget(int32_t newTarget) {
    PidConfig newConfig = { pidConfig.kp, pidConfig.ki, pidConfig.kd, newTarget, pidConfig.basePwm };
    updateConfig(newConfig);
}

void Tracer::setPwm(int newPwm) {
    PidConfig newConfig = { pidConfig.kp, pidConfig.ki, pidConfig.kd, pidConfig.targetReflection, newPwm };
    updateConfig(newConfig);
}

void Tracer::setEdge(Edge newEdge) {
    edge = newEdge;
}

void Tracer::updateConfig(const PidConfig& newConfig) {
    // ゲインや目標反射率を変更する場合、走行安定性のためpid.reset()を実行。
    bool shouldResetPid = pidConfig.kp != newConfig.kp
                          || pidConfig.ki != newConfig.ki
                          || pidConfig.kd != newConfig.kd
                          || pidConfig.targetReflection != newConfig.targetReflection;
    if(shouldResetPid) {
        pid.reset();
        filteredTurnMag = 0.0f;
    }
    pidConfig = newConfig;
    pid.setGain(pidConfig.kp, pidConfig.ki, pidConfig.kd);
    pid.setTarget(pidConfig.targetReflection);
}
