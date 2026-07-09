#ifndef COLORJUDGE_H_
#define COLORJUDGE_H_

#include "ColorSensor.h"
#include "Config.h"

using namespace spikeapi;

/**
 * カラーセンサーの生の測定値(RGB/HSV/反射率)から、コース上の色を判定するクラス。
 * ライントレースの黒/白判定、LAPゲート(青)判定、各課題でのボトル・ゲートの色判定など、
 * 「色を見て判断する」処理はすべてこのクラスに集約する。
 * しきい値はすべてConfigクラスで一元管理している。
 */
class ColorJudge {
public:
    enum class Color { BLACK,
                       WHITE,
                       RED,
                       GREEN,
                       BLUE,
                       YELLOW,
                       UNKNOWN };

    // RGB/HSV/反射率をまとめて持ち運ぶための構造体
    struct Reading {
        ColorSensor::RGB rgb;
        ColorSensor::HSV hsv;
        int reflection;  // 反射率 [0〜100]
    };

    static Color judge(const Reading& reading);

private:
    // 円環になっているHue(0〜360度)の2点間の最短距離を求める
    static int hueDistance(uint16_t h1, uint16_t h2);
};

#endif  // !COLORJUDGE_H_
