#include "Calibrator.h"
#include "CourseConfig.h"
#include "Config.h"
#include "kernel.h"

namespace {
// 追従エッジを示す矢印アイコン(5x5、輝度0~100)
constexpr uint8_t ARROW_RIGHT[25] = {
    0, 0, 100, 0, 0,
    0, 100, 0, 0, 0,
    100, 100, 100, 100, 100,
    0, 100, 0, 0, 0,
    0, 0, 100, 0, 0,
};

constexpr uint8_t ARROW_LEFT[25] = {
    0, 0, 100, 0, 0,
    0, 0, 0, 100, 0,
    100, 100, 100, 100, 100,
    0, 0, 0, 100, 0,
    0, 0, 100, 0, 0,
};
}  // namespace

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
    // デフォルトはLコース(CourseConfigの初期値と一致)
    int displayCycleCount = 0;
    bool showArrow = false;
    updateCourseDisplay(showArrow);

    while(!robot.isForceSensorPressed()) {
        bool courseChanged = false;
        if(robot.isLeftButtonPressed()) {
            CourseConfig::setCourse(CourseConfig::Course::L);
            courseChanged = true;
        } else if(robot.isRightButtonPressed()) {
            CourseConfig::setCourse(CourseConfig::Course::R);
            courseChanged = true;
        }

        if(courseChanged) {
            // ボタン操作直後はコース文字から表示し直す
            displayCycleCount = 0;
            showArrow = false;
            updateCourseDisplay(showArrow);
        } else if(++displayCycleCount >= Config::CALIBRATOR_DISPLAY_TOGGLE_CYCLES) {
            displayCycleCount = 0;
            showArrow = !showArrow;
            updateCourseDisplay(showArrow);
        }

        dly_tsk(Config::CALIBRATOR_POLL_INTERVAL_US);
    }
}

void Calibrator::updateCourseDisplay(bool showArrow) {
    bool isLeftCourse = CourseConfig::isLeftCourse();
    if(showArrow) {
        // Lコースは右エッジ、Rコースは左エッジを追従する(GameRunner::lineTraceUntilLapと同じ対応)
        robot.showImage(isLeftCourse ? ARROW_RIGHT : ARROW_LEFT);
    } else {
        robot.showChar(isLeftCourse ? 'L' : 'R');
    }
}
