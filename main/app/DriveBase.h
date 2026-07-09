#ifndef DRIVEBASE_H_
#define DRIVEBASE_H_

#include "Motor.h"

using namespace spikeapi;

/**
 * 左右モーターの操作を簡易化するためのクラス。
 */
class DriveBase {
public:
    DriveBase(Motor& leftWheel, Motor& rightWheel);
    void setPower(int leftPower, int rightPower);  // 左右モーターのパワーをセット
    void stop();                                   // 左右のモーターを停止

private:
    Motor& leftWheel;
    Motor& rightWheel;
};

#endif  // !DRIVEBASE_H_
