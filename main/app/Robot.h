#ifndef ROBOT_H_
#define ROBOT_H_

#include "Motor.h"
#include "ColorSensor.h"
#include "UltrasonicSensor.h"
#include "ForceSensor.h"
#include "Speaker.h"
#include "Display.h"
#include "Button.h"
#include "DriveBase.h"

using namespace spikeapi;

/**
 * 走行体のハードウェア（モーター・センサー・HMI）をまとめて管理するクラス。
 * 他のクラスはこのクラス経由でハードウェアを操作する。
 */
class Robot {
public:
    Robot();

    // ── 移動 ──────────────────────────────────────────
    // 指定した距離（mm）だけ直進する
    void driveStraight(int distanceMm, int speed = 30);

    // 指定した角度だけ超信地旋回する（+ = 右旋回、- = 左旋回）
    void turn(float degrees, int speed = 20);

    // 左右のモーターパワーを直接指定する（Tracerが使う）
    void setMotorPower(int left, int right);

    // モーターを停止する
    void stop();

    // ── センサー ───────────────────────────────────────
    // 超音波センサーで前方の距離を取得する [mm]
    int getUltrasonicDistance() const;

    // カラーセンサーの反射光強度を取得する [0〜100]
    int getReflection() const;

    // 青色の上にいるかどうかを返す（LAPゲート検出に使う）
    bool isOnBlue() const;

    // フォースセンサーが押されているかどうかを返す
    bool isForceSensorPressed() const;

    // ── ボタン ────────────────────────────────────────
    // Button::isXxxPressed() がconstメソッドではないため、このメソッドもconstにできない
    bool isLeftButtonPressed();
    bool isRightButtonPressed();

    // ── HMI ──────────────────────────────────────────
    void showChar(char c);    // ディスプレイに1文字表示
    void beep(int ms = 100);  // ビープ音

    // ── エンコーダー（距離計算に使う）───────────────────
    void resetMotorCounts();         // カウントをリセット
    int getLeftMotorCount() const;   // 左モーターの回転量 [degree]
    int getRightMotorCount() const;  // 右モーターの回転量 [degree]

private:
    Motor leftMotor;                    // PORT_B, COUNTERCLOCKWISE
    Motor rightMotor;                   // PORT_A, CLOCKWISE
    DriveBase driveBase;                // 左右モーターをまとめて操作するためのヘルパー
    ColorSensor colorSensor;            // PORT_E
    UltrasonicSensor ultrasonicSensor;  // PORT_F
    ForceSensor forceSensor;            // PORT_D
    Speaker speaker;
    Display display;
    Button button;

    static constexpr float WHEEL_RADIUS_MM = 28.0f;
    static constexpr float TREAD_MM = 112.0f;
    static constexpr float PI = 3.14159f;
    static constexpr uint16_t BLUE_HUE = 240;  // 青色と判定するHUE値
};

#endif  // !ROBOT_H_
