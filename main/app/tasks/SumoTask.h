#ifndef SUMOTASK_H_
#define SUMOTASK_H_

#include "Robot.h"

/**
 * ET相撲の処理を行うクラス。
 * run()を1回呼べば、土俵に行ってボトルを押し出し、ラインに戻ってくる。
 */
class SumoTask {
public:
    SumoTask(Robot& robot);
    void run();

private:
    Robot& robot;
};

#endif  // !SUMOTASK_H_
