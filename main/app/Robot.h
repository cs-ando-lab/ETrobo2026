#ifndef ROBOT_H_
#define ROBOT_H_

#include "Motor.h"
#include "ColorSensor.h"
#include "UltrasonicSensor.h"
#include "ForceSensor.h"
#include <IMU.h>
#include "Speaker.h"
#include "Display.h"
#include "Button.h"
#include "ColorJudge.h"
#include "Config.h"

using namespace spikeapi;

/**
 * 走行体のハードウェア（モーター・センサー・HMI）をまとめて管理するクラス。
 * 他のクラスはこのクラス経由でハードウェアを操作する。
 */
class Robot {
public:
    Robot();

    // ── 移動 ──────────────────────────────────────────
    // 指定した距離（mm）だけ直進する。distanceMmが負なら後退する
    // speedDegPerSec: 目標回転速度 [°/秒]（setPowerではなくsetSpeedによる速度制御を使う）
    void driveStraight(int distanceMm, int speedDegPerSec = Config::DRIVE_DEFAULT_SPEED_DEG_PER_SEC);

    // 指定した角度だけ超信地旋回する（+ = 右旋回、- = 左旋回）
    // speedDegPerSec: 目標回転速度 [°/秒]（setSpeedによる速度制御を使う）
    void turn(float degrees, int speedDegPerSec = Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // IMUの方位角を使って指定角度だけ超信地旋回する（+ = 右旋回、- = 左旋回）
    void turnByImu(float degrees, int speedDegPerSec = Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 指定された色（単色/複数色）を認識するまで直進する。
    // colors: Color配列 / colorCount: 配列数 / speedDegPerSec: 回転速度 [°/秒] / stableCount: 色を検知して止まるための連続検出回数 / forward: trueなら直進、falseなら後退
    void runStraightUntilColor(ColorJudge::Color color, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, bool forward = true);
    void runStraightUntilColors(const ColorJudge::Color* colors, int colorCount, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, bool forward = true);

    // 指定された色（単色/複数色）を認識するまで蛇行走行する。
    // colors: Color配列 / colorCount: 配列数 / speedDegPerSec: 回転速度 [°/秒] / stableCount: 色を検知して止まるための連続検出回数 / swingDeg: 蛇行運転の1旋回あたりの角度
    void runWavingUntilColor(ColorJudge::Color color, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, float swingDeg = Config::RUC_SWING_DEFAULT_DEG);
    void runWavingUntilColors(const ColorJudge::Color* colors, int colorCount, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, float swingDeg = Config::RUC_SWING_DEFAULT_DEG);

    // 指定された色（単色/複数色）とカラーセンサーの値が複数回一致するか調べる。
    // colors: Color配列 / colorCount: 配列数 / matchedCount: 指定された色とカラーセンサーの値が一致した回数を保持する変数
    // ※matchedCountは呼び出し元で管理する。
    bool isOnColor(const ColorJudge::Color color, int& matchedCount, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT) const;
    bool isOnColors(const ColorJudge::Color* colors, int colorCount, int& matchedCount, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT) const;

    // 左右のモーターパワーを直接指定する（Tracerが使う）
    void setMotorPower(int left, int right);

    // モーターを停止する
    void stop();

    // ── センサー ───────────────────────────────────────
    // 超音波センサーで前方の距離を取得する [mm]
    int getUltrasonicDistance() const;

    // カラーセンサーの反射光強度を取得する [0〜100]
    int getReflection() const;

    // カラーセンサーの生の測定値(RGB/HSV/反射率)をまとめて取得する
    ColorJudge::Reading getColorReading() const;

    // 現在の色を判定する（黒/白/赤/黄/緑/青）。LAPゲート検出や各課題での色判定に使う
    ColorJudge::Color getColor() const;

    // フォースセンサーが押されているかどうかを返す
    bool isForceSensorPressed() const;

    // ── ボタン ────────────────────────────────────────
    // Button::isXxxPressed() がconstメソッドではないため、このメソッドもconstにできない
    bool isLeftButtonPressed();
    bool isRightButtonPressed();
    bool isCenterButtonPressed();

    // ── HMI ──────────────────────────────────────────
    void showChar(char c);                        // ディスプレイに1文字表示
    void off();                                   // ディスプレイを消灯
    void beep(int ms = Config::BEEP_DEFAULT_MS);  // ビープ音

    // ── エンコーダー（距離計算に使う）───────────────────
    void resetMotorCounts();         // カウントをリセット
    int getLeftMotorCount() const;   // 左モーターの回転量 [degree]
    int getRightMotorCount() const;  // 右モーターの回転量 [degree]

    // ── デバッグログ用（BLE Monitorへのセンサー値送信にのみ使用）─────
    const ColorSensor& getColorSensor() const { return colorSensor; }
    const Motor& getLeftMotor() const { return leftMotor; }
    const Motor& getRightMotor() const { return rightMotor; }
    const UltrasonicSensor& getUltrasonicSensor() const { return ultrasonicSensor; }
    const ForceSensor& getForceSensor() const { return forceSensor; }

private:
    Motor leftMotor;                    // PORT_B, COUNTERCLOCKWISE
    Motor rightMotor;                   // PORT_A, CLOCKWISE
    ColorSensor colorSensor;            // PORT_E
    UltrasonicSensor ultrasonicSensor;  // PORT_F
    ForceSensor forceSensor;            // PORT_D
    IMU imu;
    Speaker speaker;
    Display display;
    Button button;
};

#endif  // !ROBOT_H_
