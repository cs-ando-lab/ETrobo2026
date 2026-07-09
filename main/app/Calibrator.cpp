#include "Calibrator.h"
#include "CourseConfig.h"
#include "kernel.h"

Calibrator::Calibrator(Robot& robot)
    : robot(robot) {
}

void Calibrator::run() {
    robot.showChar('B');
    robot.beep(300);
    dly_tsk(3 * 1000 * 1000); /* BLE接続待ち */

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
        dly_tsk(50 * 1000);
    }
}
