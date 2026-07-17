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
};

#endif  // !DELIVERYTASK_H_