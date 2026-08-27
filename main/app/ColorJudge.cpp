#include "ColorJudge.h"
#include <t_syslog.h>

namespace {
const char* colorName(ColorJudge::Color color) {
    switch(color) {
        case ColorJudge::Color::BLACK:
            return "BLACK";
        case ColorJudge::Color::WHITE:
            return "WHITE";
        case ColorJudge::Color::RED:
            return "RED";
        case ColorJudge::Color::GREEN:
            return "GREEN";
        case ColorJudge::Color::BLUE:
            return "BLUE";
        case ColorJudge::Color::YELLOW:
            return "YELLOW";
        default:
            return "UNKNOWN";
    }
}
}  // namespace

int ColorJudge::hueDistance(uint16_t h1, uint16_t h2) {
    int diff = static_cast<int>(h1) - static_cast<int>(h2);
    if(diff < 0) {
        diff = -diff;
    }
    if(diff > 180) {
        diff = 360 - diff;
    }
    return diff;
}

ColorJudge::Color ColorJudge::judge(const Reading& reading) {
    Color result;

    // 暗すぎる場面(反射率or明度が低い)はHue/Saturationの計算がノイズで不安定になり、
    // 彩度がCOLOR_CHROMATIC_MIN_SATURATIONを超えて有彩色に誤判定することがあるため、無彩色扱いにする
    bool tooDarkToTrustHue = reading.reflection < Config::COLOR_DARK_REFLECTION_THRESHOLD
                             || reading.hsv.v < Config::COLOR_DARK_VALUE_THRESHOLD;

    if(tooDarkToTrustHue || reading.hsv.s < Config::COLOR_CHROMATIC_MIN_SATURATION) {
        // 無彩色: 反射率で黒/白を判定する
        result = (reading.reflection < Config::COLOR_ACHROMATIC_REFLECTION_THRESHOLD) ? Color::BLACK : Color::WHITE;
    } else {
        // 有彩色: Hueが一番近い色を選ぶ
        int redDist = hueDistance(reading.hsv.h, Config::COLOR_RED_HUE);
        int yellowDist = hueDistance(reading.hsv.h, Config::COLOR_YELLOW_HUE);
        int greenDist = hueDistance(reading.hsv.h, Config::COLOR_GREEN_HUE);
        int blueDist = hueDistance(reading.hsv.h, Config::COLOR_BLUE_HUE);

        int minDist = redDist;
        Color nearest = Color::RED;
        if(yellowDist < minDist) {
            minDist = yellowDist;
            nearest = Color::YELLOW;
        }
        if(greenDist < minDist) {
            minDist = greenDist;
            nearest = Color::GREEN;
        }
        if(blueDist < minDist) {
            minDist = blueDist;
            nearest = Color::BLUE;
        }

        result = (minDist <= Config::COLOR_HUE_TOLERANCE) ? nearest : Color::UNKNOWN;
    }

    // 判定結果が変わった時だけ送信(ライントレース中は高頻度で呼ばれるため、BLEを埋めないようデルタ送信にする)
    static Color previous = Color::UNKNOWN;
    if(result != previous) {
        syslog(LOG_NOTICE, "COLORJUDGE,%s", colorName(result));
        previous = result;
    }

    return result;
}
