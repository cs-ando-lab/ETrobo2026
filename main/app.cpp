#include "app.h"
#include "kernel_cfg.h"
#include <t_syslog.h>
#include "Robot.h"
#include "GameRunner.h"

#include <spike/hub/system.h>

/* ハードウェアの実体はすべてRobotが所有 */
Robot robot;
GameRunner runner(robot);

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {
    runner.run();

    robot.off();
    syslog(LOG_NOTICE, "stopped");
    dly_tsk(500 * 1000);

    /* センターボタンでの停止後、ハブの電源も切る */
    hub_system_shutdown();
}
