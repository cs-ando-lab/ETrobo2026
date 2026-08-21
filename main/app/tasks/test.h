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

    // DeliveryTask.cppのステップ9（177行目付近）から、
    // ライントレース→青検知→直進、までを切り出して単体で試すための関数
    void traceUntilBlueThenStraight();

    // DeliveryTask.cppのdiagonalMoveUntilImuTurnと同じもの（左右のパワーを
    // 渡す側で入れ替えれば左右どちらの斜め移動も試せる）
    void diagonalMoveUntilImuTurn(int leftPwm, int rightPwm, float startHeading, float turnDeg);

    // 蛇行探索（右優先）。isOnColorsの離散色判定だと黒/白境界の反射率(60)と
    // TRACER_TARGET_REFLECTION(60)が同値でブレるため、生の反射率でしきい値未満を判定する
    void waveUntilReflectionBelow(int reflectionThreshold, float swingDeg, int pwm);
};

#endif  // !TEST_H_
