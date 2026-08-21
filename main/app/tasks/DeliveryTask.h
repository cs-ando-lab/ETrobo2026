#ifndef DELIVERYTASK_H_
#define DELIVERYTASK_H_

#include "Robot.h"
#include "Motor.h"  // ▼ アームのモーターを引数で渡すために追加！

using namespace spikeapi;

/**
 * ボトルデリバリーの処理を行うクラス。
 */
class DeliveryTask {
public:
    DeliveryTask(Robot& robot);
    void run();

private:
    Robot& robot;

    // ▼ アーム操作用の関数（モーターを受け取るように変更） ▼
    void lowerArm(Motor& armMotor);
    void raiseArm(Motor& armMotor);

    // 左右のパワー差で進みつつ、startHeadingからturnDeg回転したら停止する（斜め移動）
    void diagonalMoveUntilImuTurn(int leftPwm, int rightPwm, float startHeading, float turnDeg);

    // 帰りの線探し。色判定(isOnColors)ではなく反射率のしきい値判定で、片輪停止のピボット旋回を繰り返す。
    // firstSwingRightがtrueなら右優先(半分右→左→右…)、falseなら左優先（Rコース用に反転）
    void waveUntilReflectionBelow(int reflectionThreshold, float swingDeg, int pwm, bool firstSwingRight);
};

#endif  // !DELIVERYTASK_H_