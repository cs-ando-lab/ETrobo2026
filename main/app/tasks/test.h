#ifndef TEST_H_
#define TEST_H_

#include "Robot.h"

/**
 * 関数などを試すためのテスト用クラス。
 */
class Test {
public:
    Test(Robot& robot);
    void run();

private:
    Robot& robot;
};

#endif  // !TEST_H_
