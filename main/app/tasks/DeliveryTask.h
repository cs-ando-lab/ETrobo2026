#ifndef DELIVERYTASK_H_
#define DELIVERYTASK_H_

#include "Robot.h"

/**
 * ボトルデリバリーの処理を行うクラス。
 * run()を1回呼べば、ボトルの色を判定して正しいゾーンに運ぶ。
 */
class DeliveryTask {
public:
    DeliveryTask(Robot& robot);
    void run();

private:
    Robot& robot;
};

#endif  // !DELIVERYTASK_H_
