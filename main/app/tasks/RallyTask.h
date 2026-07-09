#ifndef RALLYTASK_H_
#define RALLYTASK_H_

#include "Robot.h"

/**
 * ETラリーの処理を行うクラス。
 * run()を1回呼べば、赤→青→黄のゲートを順番に通過する。
 */
class RallyTask {
public:
    RallyTask(Robot& robot);
    void run();

private:
    Robot& robot;
};

#endif  // !RALLYTASK_H_
