#include "app.h"
#include "kernel_cfg.h"
#include <t_syslog.h>
#include "debug_log.h"

#include "Tracer.h"
#include "Speaker.h"
#include "Display.h"
#include "Button.h"
#include "UltrasonicSensor.h"
#include "ForceSensor.h"
#include <spike/hub/system.h>

using namespace spikeapi;

Tracer tracer;

namespace {
    const char *const TRACER_LABEL     = "Tracer";
    const int         TRACER_LABEL_LEN = 6;  /* strlen(TRACER_LABEL) */
    const int         LABEL_CHANGE_CYCLES = 3; /* 100ms * 3 = 300ms毎に1文字切替 */
}

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {
    Speaker speaker;
    Display display;
    Button button;
    UltrasonicSensor ultrasonicSensor(EPort::PORT_F);
    ForceSensor forceSensor(EPort::PORT_D);

    speaker.setVolume(50);
    display.showChar('B');
    speaker.playTone(NOTE_A4, 300);
    dly_tsk(3 * 1000 * 1000); /* BLE接続待ち */

    /* フォースセンサーが押されるまでライントレース開始を待つ */
    display.off();
    while (!forceSensor.isTouched()) {
        dly_tsk(50 * 1000);
    }

    /* ライントレース初期化 */
    tracer.init();

    const debug_sensors_t sensors = {
        .color = &tracer.getColorSensor(),
        .left_motor = &tracer.getLeftMotor(),
        .right_motor = &tracer.getRightMotor(),
        .ultrasonic = &ultrasonicSensor,
        .force = &forceSensor,
    };
    debug_log_init(&sensors);

    int count = 0;
    int prevLabelIndex = -1;

    while(1) {
        /* センターボタンで停止 */
        if (button.isCenterPressed()) {
            break;
        }

        /* ライントレース */
        tracer.run();

        /* ライントレース中は"Tracer"の文字を1文字ずつ循環表示 */
        int labelIndex = (count / LABEL_CHANGE_CYCLES) % TRACER_LABEL_LEN;
        if (labelIndex != prevLabelIndex) {
            display.showChar(TRACER_LABEL[labelIndex]);
            prevLabelIndex = labelIndex;
        }

        debug_log_all(&sensors, count);

        count++;
        dly_tsk(100 * 1000); /* 100ms周期 */
    }

    display.off();
    tracer.terminate();

    syslog(LOG_NOTICE, "stopped");
    dly_tsk(500 * 1000);

    /* センターボタンでの停止後、ハブの電源も切る */
    hub_system_shutdown();
}
