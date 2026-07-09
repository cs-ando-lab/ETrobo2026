#include "Calibrator.h"
#include "CourseConfig.h"
#include "Config.h"
#include "kernel.h"

Calibrator::Calibrator(Robot& robot)
    : robot(robot) {
}

void Calibrator::run() {
    robot.showChar('B');
    robot.beep(Config::CALIBRATOR_BEEP_MS);
    dly_tsk(Config::CALIBRATOR_BLE_WAIT_US); /* BLE接続待ち */

    selectCourseAndWaitForStart();

    robot.off();
    robot.resetMotorCounts();
}

void Calibrator::selectCourseAndWaitForStart() {
    robot.showChar('L');  // デフォルトはLコース
    while(!robot.isForceSensorPressed()) {
        if(robot.isLeftButtonPressed()) {
            CourseConfig::setCourse(CourseConfig::Course::L);
            robot.showChar('L');
        } else if(robot.isRightButtonPressed()) {
            CourseConfig::setCourse(CourseConfig::Course::R);
            robot.showChar('R');
        }
        dly_tsk(Config::CALIBRATOR_POLL_INTERVAL_US);
    }
}
