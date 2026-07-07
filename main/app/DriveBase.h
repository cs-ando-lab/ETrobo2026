#ifndef DRIVEBASE_H_
#define DRIVEBASE_H_

#include "Motor.h"

using namespace spikeapi;

/**
 * 左右モーターを管理するクラス
 * 左右モーターの操作はこのクラスを通して行う。
 */
class DriveBase {
public:
    DriveBase();
    void run(int8_t leftPower, int8_t rightPower);  // 左右モーターのパワーをセット
    void stop();                                    // 左右のモーターを停止

    const Motor& getLeftMotor() const { return leftWheel; }
    const Motor& getRightMotor() const { return rightWheel; }

private:
    Motor leftWheel;
    Motor rightWheel;
};

#endif  // !DRIVEBASE_H_
