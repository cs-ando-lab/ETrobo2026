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
    void setPwm(int newPwm);  // newPwmは0～100の大きさのみ
    void setLeftMotorOffset(int newOffset);

    // カーブ減速の強さ（|操舵量|1あたりに基準パワーから引くPWM）を変更する。
    // 既定値はConfig::TRACER_CURVE_DECEL_GAINなので、呼ばなければ今までどおり。
    // 直線区間など、減速されると困る場面で一時的に弱める用
    void setCurveDecelGain(float newGain);

    // 次のrun()から制御を始め直す。ゲイン・目標・PWM・エッジを設定した後に呼ぶ。
    // 次のrun()が読んだ反射率でPidの前回偏差を初期化するので、再開直後だけのD項が出ない。
    // カーブ減速のEMAもここで0にし、初回の実際の操舵量から通常どおり積み上げ直す
    void restartFromNextSample();

    // 追従したままゲイン・PWMを切り替える。setConfig()はゲインが変わるとPIDと減速フィルタを
    // リセットするため、区間をまたいで実測の履歴を引き継ぎたい場面では使えない。
    // 前回偏差・微分フィルタ・カーブ減速のEMAを保持し、積分だけを0に戻す。
    // 目標反射率が今と違う場合、値が不正な場合、再開が予約されている場合は何も変えない。
    // エッジを変える切り替えには使わないこと（履歴の意味が変わる）
    void setConfigKeepingTrackingState(float newKp, float newKi, float newKd, int32_t newTarget, int newPwm);

    // PIDの内部状態（積分・微分）をリセットする。ゲインを途中で変えるときに、
    // 前の区間で溜まった積分がそのまま効いてしまうのを防ぐ。
    // 微分の履歴まで消えるので、積分だけを消したいときはresetPidIntegral()を使う
    void resetPid();
    void resetPidIntegral();

    // 直近のP/I/Dの符号つき内訳（診断用）
    float getLastP() const { return pid.getLastP(); }
    float getLastI() const { return pid.getLastI(); }
    float getLastD() const { return pid.getLastD(); }

    // 最後にrun()が計算した左右のPWM。モーター側で±100に頭打ちされる前の値なので、
    // 「操舵が上限に当たっていないか」を外から測れる（診断用。制御には影響しない）
    int getLastLeftPwm() const { return lastLeftPwm; }
    int getLastRightPwm() const { return lastRightPwm; }
    int getBasePwm() const { return pidConfig.basePwm; }  // 診断用。制御状態は変更しない。

    //
    void setEdge(Edge newEdge);

private:
    Robot& robot;
    Pid pid;  // 反射率がConfig::TRACER_TARGET_REFLECTIONに近づくよう左右パワー差を計算する
    PidConfig pidConfig;
    Edge edge;
    float curveDecelGain = Config::TRACER_CURVE_DECEL_GAIN;  // カーブ減速の強さ。setCurveDecelGain()で変えられる
    float filteredTurnMag = 0.0f;                            // カーブ減速量算出用、|turn|にEMAをかけた値
    int leftMotorOffset = 0;                                 // 左右モーターの出力差を均すための調整値
    int lastLeftPwm = 0;                                     // 診断用。クランプ前の計算値
    int lastRightPwm = 0;

    void updateConfig(const PidConfig& newConfig);  // PidクラスのsetGain, setTargetを呼び出し、パラメータを更新。
};

#endif  // !TRACER_H_
