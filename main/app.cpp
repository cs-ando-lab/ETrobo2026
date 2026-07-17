#include "app.h"
#include "kernel_cfg.h"
#include <t_syslog.h>
#include "Robot.h"
#include "GameRunner.h"
#include "debug_log.h"

#include <spike/hub/system.h>

/* ハードウェアの実体はすべてRobotが所有 */
Robot robot;
GameRunner runner(robot);

/* BLE Monitorへのセンサー値送信用（DEBUG_LOG_TASKが周期的に参照する） */
static const debug_sensors_t debugSensors{ &robot.getColorSensor(), &robot.getLeftMotor(), &robot.getRightMotor(),
                                           &robot.getUltrasonicSensor(), &robot.getForceSensor() };

/* センサー値をBLE Monitorへ送信する周期タスク。DEBUG_LOG_PERIODごとに起動される。
 * main_taskが何をしていようと（キャリブレーション中・ライントレース中・各課題の実行中）
 * 独立して動くため、電源投入後は常にセンサー値が流れ続ける。 */
void debug_log_task(intptr_t unused) {
    static bool initialized = false;
    static int count = 0;

    if(!initialized) {
        debug_log_init(&debugSensors);
        initialized = true;
    }
    debug_log_all(&debugSensors, count++);
}

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {
    runner.run();

    robot.off();
    syslog(LOG_NOTICE, "stopped");
    dly_tsk(500 * 1000);

    /* センターボタンでの停止後、ハブの電源も切る */
    hub_system_shutdown();
}
