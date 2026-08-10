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
};

#endif  // !DELIVERYTASK_H_