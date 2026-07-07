#include <DriveBase.h>

DriveBase::DriveBase()
    : leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true),
      rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true) {
}

void DriveBase::run(int8_t leftPower, int8_t rightPower) {
    leftWheel.setPower(leftPower);
    rightWheel.setPower(rightPower);
}

void DriveBase::stop() {
    leftWheel.stop();
    rightWheel.stop();
}