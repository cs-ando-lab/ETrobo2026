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

    // 左右のパワー差で進みつつ、startHeadingからturnDeg回転したら停止する（斜め移動）。
    // isOuterLeftがtrueなら左輪が外側（＝右へ回る）。内輪は回頭の進みに応じて段階的に上げる
    void diagonalMoveUntilImuTurn(bool isOuterLeft, int outerPwm, float startHeading, float turnDeg);

    // 進行方向を変える前に、ブレーキで速度が落ちるまで待つ（惰性と逆向き指令の喧嘩を避ける）
    void brakeUntilStopped(int speedThresholdDegPerSec, int timeoutMs);

    // デューティ上限（＝トルク上限）を落として直進/後退する。distanceMmが負なら後退。上限は関数内で必ず元に戻す
    void driveStraightWithDutyLimit(int distanceMm, int speedDegPerSec, int dutyLimit);

    // 両輪を逆向きに回してその場でturnDeg旋回する（閉ループのturnByImuより速い）
    void turnInPlaceByImu(int leftPwm, int rightPwm, float turnDeg);

    // 帰りの線探し。色判定(isOnColors)ではなく反射率のしきい値判定で、片輪停止のピボット旋回を続ける。
    // isRightTurnがtrueなら右旋回、falseなら左旋回（Rコース用に反転）
    void pivotUntilReflectionBelow(int reflectionThreshold, int pwm, bool isRightTurn);
};

#endif  // !TEST_H_
