#include "Tracer.h"
#include <t_syslog.h>

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
    float turn = -pid.calculate(robot.getReflection(), DELTA_SEC);                      // 比例制御の調整値を求める
    int pwm_l = static_cast<int>(pidConfig.basePwm - (static_cast<int>(edge) * turn));  // 基準値と調整値を使って操作量を求める
    int pwm_r = static_cast<int>(pidConfig.basePwm + (static_cast<int>(edge) * turn));
    robot.setMotorPower(pwm_l, pwm_r);
}

void Tracer::setConfig(float newKp, float newKi, float newKd, int32_t newTarget, int newPwm) {
    if(newTarget < 0 || newTarget > 100) {
        syslog(LOG_ERROR, "invalid target value : target must be 0 ~ 100");
        return;
    }
    if(newPwm < 0 || newPwm > 100) {
        syslog(LOG_ERROR, "invalid pwm : pwm must be 0 ~ 100");
        return;
    }

    PidConfig newConfig = { newKp, newKi, newKd, newTarget, newPwm };
    updateConfig(newConfig);
}

void Tracer::setTarget(int32_t newTarget) {
    if(newTarget < 0 || newTarget > 100) {
        syslog(LOG_ERROR, "invalid target value : target must be 0 ~ 100");
        return;
    }

    PidConfig newConfig = { pidConfig.kp, pidConfig.ki, pidConfig.kd, newTarget, pidConfig.basePwm };
    updateConfig(newConfig);
}

void Tracer::setPwm(int newPwm) {
    if(newPwm < 0 || newPwm > 100) {
        syslog(LOG_ERROR, "invalid pwm : pwm must be 0 ~ 100");
        return;
    }

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
    if(shouldResetPid)
        pid.reset();
    pidConfig = newConfig;
    pid.setGain(pidConfig.kp, pidConfig.ki, pidConfig.kd);
    pid.setTarget(pidConfig.targetReflection);
}
