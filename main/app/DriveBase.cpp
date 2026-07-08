#include "DriveBase.h"

DriveBase::DriveBase(Motor& leftWheel, Motor& rightWheel)
    : leftWheel(leftWheel),
      rightWheel(rightWheel) {
}

void DriveBase::setPower(int leftPower, int rightPower) {
    leftWheel.setPower(leftPower);
    rightWheel.setPower(rightPower);
}

void DriveBase::stop() {
    leftWheel.stop();
    rightWheel.stop();
}