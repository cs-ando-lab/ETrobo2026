#include "kernel_cfg.h"
#include "app.h"

#include <cstdint>

#include "Battery.h"
#include "Button.h"
#include "Display.h"
#include "Light.h"
#include "Speaker.h"

extern "C" {
#include <spike/hub/system.h>
#include <pbdrv/charger.h>
}

using namespace spikeapi;

namespace {

    // SPIKE Prime Hub内蔵バッテリーは常にLi-ion(2セル)として扱われる
    // (PBDRV_CONFIG_BATTERY_ADC_TYPE=2固定)。ここの閾値は
    // external/libpybricks/lib/pbio/sys/battery.cのLIION_*定数と同じ値。
    constexpr uint16_t kBatteryFullMv = 8300;
    constexpr uint16_t kBatteryLowMv = 6800;
    constexpr uint16_t kBatteryCriticalMv = 6000;

    // 誤ってセンターボタンに触れただけでシャットダウンしないための長押し時間
    constexpr uint32_t kShutdownHoldMs = 1000;
    constexpr uint32_t kButtonPollMs = 100;

    int8_t voltageToPercent(uint16_t mv) {
        if(mv <= kBatteryCriticalMv) {
            return 0;
        }
        if(mv >= kBatteryFullMv) {
            // Display::showNumber()は100以上を'>'表示にしてしまうため99を上限にする
            return 99;
        }
        return static_cast<int8_t>((static_cast<int32_t>(mv - kBatteryCriticalMv) * 100) / (kBatteryFullMv - kBatteryCriticalMv));
    }

    char chargerStatusChar(pbdrv_charger_status_t status) {
        switch(status) {
            case PBDRV_CHARGER_STATUS_CHARGE:
                return 'C';
            case PBDRV_CHARGER_STATUS_COMPLETE:
                return 'F';
            case PBDRV_CHARGER_STATUS_FAULT:
                return 'X';
            case PBDRV_CHARGER_STATUS_DISCHARGE:
            default:
                return 'B';
        }
    }

    void updateStatusLight(Light& light, pbdrv_charger_status_t status, uint16_t voltageMv) {
        switch(status) {
            case PBDRV_CHARGER_STATUS_CHARGE:
                light.turnOnColor(Light::EColor::ORANGE);
                break;
            case PBDRV_CHARGER_STATUS_COMPLETE:
                light.turnOnColor(Light::EColor::GREEN);
                break;
            case PBDRV_CHARGER_STATUS_FAULT:
                light.turnOnColor(Light::EColor::RED);
                break;
            case PBDRV_CHARGER_STATUS_DISCHARGE:
            default:
                // USB未接続でバッテリー駆動のまま放置されているケースに気づけるように、
                // 充電されていない状態でも電圧が低ければ警告色にする
                light.turnOnColor(voltageMv <= kBatteryLowMv ? Light::EColor::RED : Light::EColor::WHITE);
                break;
        }
    }

    // 状態が切り替わった瞬間だけ鳴らす。ポーリングのたびに鳴ると煩わしいため
    void notifyStatusChange(Speaker& speaker, pbdrv_charger_status_t status) {
        switch(status) {
            case PBDRV_CHARGER_STATUS_CHARGE:
                speaker.playTone(NOTE_C5, 150);
                break;
            case PBDRV_CHARGER_STATUS_COMPLETE:
                speaker.playTone(NOTE_C6, 150);
                dly_tsk(180 * 1000);
                speaker.playTone(NOTE_C6, 150);
                break;
            case PBDRV_CHARGER_STATUS_FAULT:
                speaker.playTone(NOTE_A4, 400);
                break;
            case PBDRV_CHARGER_STATUS_DISCHARGE:
            default:
                break;
        }
    }

    // センターボタンをkShutdownHoldMs以上押し続けたらtrueを返す
    bool isShutdownRequested(Button& button) {
        if(!button.isCenterPressed()) {
            return false;
        }

        for(uint32_t heldMs = 0; heldMs < kShutdownHoldMs; heldMs += kButtonPollMs) {
            dly_tsk(kButtonPollMs * 1000);
            if(!button.isCenterPressed()) {
                return false;
            }
        }
        return true;
    }

}  // namespace

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {
    Battery battery;
    Button button;
    Display display;
    Light light;
    Speaker speaker;

    speaker.setVolume(70);

    bool first = true;
    pbdrv_charger_status_t prevStatus = PBDRV_CHARGER_STATUS_DISCHARGE;

    for(;;) {
        if(isShutdownRequested(button)) {
            hub_system_shutdown();
        }

        uint16_t voltageMv = battery.getVoltage();
        pbdrv_charger_status_t status = pbdrv_charger_get_status();

        if(first || status != prevStatus) {
            notifyStatusChange(speaker, status);
            prevStatus = status;
            first = false;
        }

        updateStatusLight(light, status, voltageMv);

        // 「バッテリー残量(%)」と「充電状態(B/C/F/X)」を交互に表示する
        display.showNumber(voltageToPercent(voltageMv));
        dly_tsk(1500 * 1000);

        display.showChar(chargerStatusChar(status));
        dly_tsk(1000 * 1000);
    }

    /* タスク終了(実際にはhub_system_shutdown()から戻らないため到達しない) */
    ext_tsk();
}
