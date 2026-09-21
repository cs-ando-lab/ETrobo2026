#ifndef ROBOT_H_
#define ROBOT_H_

#include <cstdint>
#include "Motor.h"
#include "ColorSensor.h"
#include "UltrasonicSensor.h"
#include "ForceSensor.h"
#include "IMU.h"
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

    // 超音波を使った探索の結果（turnByImuUntilUltrasonicの戻り値）
    struct SearchResult {
        bool found;             // 一度でも距離を検知できればtrue
        float actualTurnedDeg;  // この呼び出しで実際に旋回できた角度（IMU実測）
        float bestHeadingDeg;   // 走行中、距離が最小だった時点のIMU角度（ターゲットに最も正対していたと推定される向き）
        int bestDistanceMm;     // 上記の時点での超音波距離
    };

    // ── 移動 ──────────────────────────────────────────
    // 指定した距離（mm）だけ直進する。distanceMmが負なら後退する
    // speedDegPerSec: 目標回転速度 [°/秒]（setPowerではなくsetSpeedによる速度制御を使う）
    // 戻り値: 実際に進んだ距離[mm]（後退なら負値。タイムアウトやセンターボタン中断は要求値より小さくなる）
    int driveStraight(int distanceMm, int speedDegPerSec = Config::DRIVE_DEFAULT_SPEED_DEG_PER_SEC);

    // 指定した角度だけ超信地旋回する（+ = 右旋回、- = 左旋回）
    // speedDegPerSec: 目標回転速度 [°/秒]（setSpeedによる速度制御を使う）
    void turn(float degrees, int speedDegPerSec = Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 指定した距離（mm）だけ、停止前に減速しながらIMUの目標方位を保って直進する。distanceMmが負なら後退する
    // speedDegPerSec: 目標回転速度 [°/秒]（setPowerではなくsetSpeedによる速度制御を使う）
    // directionDeg: resetHeading()で定めた0°を基準とする目標heading角[°]
    // 戻り値: 実際に進んだ距離[mm]（後退なら負値。タイムアウトやセンターボタン中断は要求値より小さくなる）
    int driveStraightByImu(int distanceMm, float directionDeg, int speedDegPerSec = Config::DRIVE_DEFAULT_SPEED_DEG_PER_SEC);

    // IMUの方位角を使って指定角度を正規化した最短角度分、超信地旋回する（+ = 右旋回、- = 左旋回）
    // 戻り値: 実際に旋回できた角度[°]（IMU実測。タイムアウトやセンターボタン中断は要求値と異なる）
    float turnByImu(float degrees, int speedDegPerSec = Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 超信地旋回しながら、指定角度に達するか超音波センサが指定距離未満を検知するまで走行する（+ = 右旋回、- = 左旋回）。
    // ビームの広がりで検知端がずれるため、走行中に距離が最小だった角度をbestHeadingDegとして返す。
    // 呼び出し側はそこまで旋回し直すことで対象物に最も正対した向きに戻せる。
    SearchResult turnByImuUntilUltrasonic(float degrees, int detectDistanceMm, int speedDegPerSec = Config::TURN_DEFAULT_SPEED_DEG_PER_SEC);

    // 指定された色（単色/複数色）を認識するまで直進する。
    // colors: Color配列 / colorCount: 配列数 / speedDegPerSec: 回転速度 [°/秒] / stableCount: 色を検知して止まるための連続検出回数 / forward: trueなら直進、falseなら後退
    void runStraightUntilColor(ColorJudge::Color color, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, bool forward = true);
    void runStraightUntilColors(const ColorJudge::Color* colors, int colorCount, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, bool forward = true);

    // 指定された色（単色/複数色）を認識するまで蛇行走行する。
    // colors: Color配列 / colorCount: 配列数 / speedDegPerSec: 回転速度 [°/秒] / stableCount: 色を検知して止まるための連続検出回数 / swingDeg: 蛇行運転の1旋回あたりの角度
    // firstSwingRight: 最初の半分旋回を右にするか（falseなら従来通り左）
    void runWavingUntilColor(ColorJudge::Color color, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, float swingDeg = Config::RUC_SWING_DEFAULT_DEG, bool firstSwingRight = false);
    void runWavingUntilColors(const ColorJudge::Color* colors, int colorCount, int speedDegPerSec = Config::RUC_DEFAULT_SPEED_DEG_PER_SEC, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT, float swingDeg = Config::RUC_SWING_DEFAULT_DEG, bool firstSwingRight = false);

    // 指定された色（単色/複数色）とカラーセンサーの値が複数回一致するか調べる。
    // colors: Color配列 / colorCount: 配列数 / matchedCount: 指定された色とカラーセンサーの値が一致した回数を保持する変数
    // ※matchedCountは呼び出し元で管理する。
    bool isOnColor(const ColorJudge::Color color, int& matchedCount, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT) const;
    bool isOnColors(const ColorJudge::Color* colors, int colorCount, int& matchedCount, int stableCount = Config::COLOR_DETECTED_STABLE_COUNT) const;

    // 左右のモーターパワーを直接指定する（Tracerが使う）
    void setMotorPower(int left, int right);

    // アームの上げ下げ。上げはモーターの位置制御で目標角に止めて保持し、下げは止めた後に保持しない。
    // extraDegはConfig::ARM_RAISE_DEG / ARM_LOWER_DEGへの上乗せ分[°]。上げで足した分は、下げでも同じだけ足すこと
    void raiseArm(int extraDeg = 0);
    // Delivery専用。エンコーダー原点を変えず、同じ目標位置へ移動する。
    // 出力上限は既存値のまま。未達・失速・中断はfalse。
    bool positionDeliveryArm(int targetCount, int speedDegPerSec, const char* label);
    void lowerArm(int extraDeg = 0);
    // アームを上げながら直進する（上げはモーター側で進むので、その間に走れる）。
    // distanceMmが負なら後退。走り終えても、上げ切るまでは戻らない
    void raiseArmWhileDriving(int distanceMm, int speedDegPerSec, int extraDeg = 0);

    // モーターを停止する
    void stop();
    void brake();

    // ── センサー ───────────────────────────────────────
    // 超音波センサーで前方の距離を取得する [mm]
    int getUltrasonicDistance() const;

    // カラーセンサーの反射光強度を取得する [0〜100]
    int getReflection() const;

    // カラーセンサーの生の測定値(RGB/HSV/反射率)をまとめて取得する
    ColorJudge::Reading getColorReading() const;

    // IMUの方位角を取得する [°]（+ = 右回り）。resetは行わない生値
    float getImuHeading() const;

    // 現在の色を判定する（黒/白/赤/黄/緑/青）。LAPゲート検出や各課題での色判定に使う
    ColorJudge::Color getColor() const;

    // フォースセンサーが押されているかどうかを返す
    bool isForceSensorPressed() const;

    // IMUの加速度[mm/s^2]。静止していれば重力の分解になるので、車体の傾き（ピッチ）が分かる。
    // アームを上げる瞬間に車体が煽られていないかを見る診断用
    IMU::Acceleration getImuAcceleration();
    // IMUの角速度[°/s]。上と同じ用途
    IMU::AngularVelocity getImuAngularVelocity();
    // IMUが静止していると判断しているか
    bool isImuStationary() const;

    // IMU関連
    float getAngularVelocityZ();  // IMUのz軸における角速度(符号反転)を取得する
    float getHeading() const;     // IMUの方位角を取得する
    void resetHeading();          // IMUの方位角をリセットする

    // ── ボタン ────────────────────────────────────────
    // Button::isXxxPressed() がconstメソッドではないため、このメソッドもconstにできない
    bool isLeftButtonPressed();
    bool isRightButtonPressed();
    bool isCenterButtonPressed();

    // ── HMI ──────────────────────────────────────────
    void showChar(char c);                        // ディスプレイに1文字表示
    void showImage(const uint8_t image[25]);      // ディスプレイに5x5の輝度パターン(0～100)を表示
    void off();                                   // ディスプレイを消灯
    void beep(int ms = Config::BEEP_DEFAULT_MS);  // ビープ音（鳴り終わるまで戻らない）
    // beep()と同じ音を、鳴り終わるのを待たずに開始する。走行ループの中から鳴らす用。
    // 自動では止まらないので、鳴らし始めた側が必ずstopBeep()で止めること
    void startBeepNonBlocking();
    void stopBeep();  // 再生中の音を止める（鳴っていなければ何も起きない）

    // ── エンコーダー（距離計算に使う）───────────────────
    void resetMotorCounts();         // カウントをリセット
    int getLeftMotorCount() const;   // 左モーターの回転量 [degree]
    int getRightMotorCount() const;  // 右モーターの回転量 [degree]
    int getArmCount() const;         // アームの回転量 [degree]（直前のraiseArm/lowerArmの開始時を0とする）

    // アームモーターの出力[%]。保持中は速度がほぼ0なので、そのままトルクの目安になる
    int getArmPower() const;
    // アームモーターが失速しているか（出力を出しているのに回っていない）
    bool isArmStalled() const;

    // ── デバッグログ用（BLE Monitorへのセンサー値送信にのみ使用）─────
    const ColorSensor& getColorSensor() const { return colorSensor; }
    const Motor& getLeftMotor() const { return leftMotor; }
    const Motor& getRightMotor() const { return rightMotor; }
    const UltrasonicSensor& getUltrasonicSensor() const { return ultrasonicSensor; }
    const ForceSensor& getForceSensor() const { return forceSensor; }

private:
    // IMUが使える状態になるまで待つ（turnByImu/turnByImuUntilUltrasonicの共通処理）
    // 戻り値: 使える状態になればtrue。タイムアウトまたはセンターボタン中断ならfalse
    bool waitForImuReady();

    Motor leftMotor;                    // PORT_B, COUNTERCLOCKWISE
    Motor rightMotor;                   // PORT_A, CLOCKWISE
    Motor armMotor;                     // PORT_C, CLOCKWISE
    ColorSensor colorSensor;            // PORT_E
    UltrasonicSensor ultrasonicSensor;  // PORT_F
    ForceSensor forceSensor;            // PORT_D
    IMU imu;
    Speaker speaker;
    Display display;
    Button button;
};

#endif  // !ROBOT_H_
