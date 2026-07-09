#include "ColorJudge.h"

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
    if(reading.hsv.s < Config::COLOR_CHROMATIC_MIN_SATURATION) {
        // 無彩色: 反射率で黒/白を判定する
        return (reading.reflection < Config::COLOR_ACHROMATIC_REFLECTION_THRESHOLD) ? Color::BLACK : Color::WHITE;
    }

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

    return (minDist <= Config::COLOR_HUE_TOLERANCE) ? nearest : Color::UNKNOWN;
}
