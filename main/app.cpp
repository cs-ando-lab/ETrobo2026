#include "app.h"
#include "kernel_cfg.h"
#include <t_syslog.h>
#include "debug_log.h"
#include "Robot.h"
#include "Tracer.h"

#include <spike/hub/system.h>

/* ハードウェアの実体はすべてRobotが所有 */
Robot robot;
Tracer tracer(robot);

namespace {
    const char* const TRACER_LABEL = "Tracer";
    const int TRACER_LABEL_LEN = 6;    /* strlen(TRACER_LABEL) */
    const int LABEL_CHANGE_CYCLES = 3; /* 100ms * 3 = 300ms毎に1文字切替 */
}  // namespace

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {
    robot.showChar('B');
    robot.beep(300);
    dly_tsk(3 * 1000 * 1000); /* BLE接続待ち */

    /* フォースセンサーが押されるまでライントレース開始を待つ */
    robot.off();
    while(!robot.isForceSensorPressed()) {
        dly_tsk(50 * 1000);
    }

    const debug_sensors_t sensors = {
        .color = &robot.getColorSensor(),
        .left_motor = &robot.getLeftMotor(),
        .right_motor = &robot.getRightMotor(),
        .ultrasonic = &robot.getUltrasonicSensor(),
        .force = &robot.getForceSensor(),
    };
    debug_log_init(&sensors);

    int count = 0;
    int prevLabelIndex = -1;

    while(1) {
        /* センターボタンで停止 */
        if(robot.isCenterButtonPressed() || tracer.isOnBlue()) {
            break;
        }

        /* ライントレース */
        tracer.run();

        /* ライントレース中は"Tracer"の文字を1文字ずつ循環表示 */
        int labelIndex = (count / LABEL_CHANGE_CYCLES) % TRACER_LABEL_LEN;
        if(labelIndex != prevLabelIndex) {
            robot.showChar(TRACER_LABEL[labelIndex]);
            prevLabelIndex = labelIndex;
        }

        debug_log_all(&sensors, count);

        count++;
        dly_tsk(100 * 1000); /* 100ms周期 */
    }

    robot.off();
    tracer.terminate();

    syslog(LOG_NOTICE, "stopped");
    dly_tsk(500 * 1000);

    /* センターボタンでの停止後、ハブの電源も切る */
    hub_system_shutdown();
}
