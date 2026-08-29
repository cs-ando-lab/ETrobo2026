#ifndef DELIVERYTASK_H_
#define DELIVERYTASK_H_

#include "Robot.h"
#include "Motor.h"  // ▼ アームのモーターを引数で渡すために追加！

class Tracer;

using namespace spikeapi;

/**
 * ボトルデリバリーの処理を行うクラス。
 */
class DeliveryTask {
public:
    DeliveryTask(Robot& robot);
    void run();

private:
    Robot& robot;

    // コーナー旋回の結果。「線を見つけたか」だけでは曲がりきったか判断できないため3状態にする
    enum class CornerResult {
        CLEARED,     // 線を発見し、検知時からの方位差が完了角度以上
        REACQUIRED,  // 線には復帰したが、まだ完了角度に届いていない
        FAILED       // 正方向・振り戻しとも線を発見できなかった
    };

    // 直角コーナー検知の状態。行きと帰りで1つずつ持ち、互いに干渉させない
    struct CornerState {
        bool pending = false;     // 有効化待ち
        bool enabled = false;     // 白の連続を数えている
        bool confirming = false;  // 線には復帰した。Tracerが残りを曲がりきるのを待っている
        bool done = false;        // 曲がりきった。以降は一切判定しない
        int whiteRun = 0;
        int suppressCount = 0;
        int loopCount = 0;
        int enableLoop = 0;
        int confirmDeadlineLoop = 0;
        float startHeading = 0.0f;  // コーナーを検知した時点の方位
    };

    // ▼ アーム操作用の関数（モーターを受け取るように変更） ▼
    void lowerArm(Motor& armMotor);
    void raiseArm(Motor& armMotor);

    // 左右のパワー差で弧を描きながら、startHeadingからturnDeg回頭したら停止する（斜め移動）。
    // isOuterLeftがtrueなら左輪が外側（＝右へ回る）。内輪は回頭の進み具合で段階的に上げる
    void diagonalMoveUntilImuTurn(bool isOuterLeft, int outerPwm, float startHeading, float turnDeg);

    // 進行方向を変える前に、ブレーキで速度が落ちるまで待つ（惰性と逆向き指令の喧嘩を避ける）
    void brakeUntilStopped(int speedThresholdDegPerSec, int timeoutMs);

    // デューティ上限（＝トルク上限）を落として直進/後退する。distanceMmが負なら後退。上限は関数内で必ず元に戻す
    void driveStraightWithDutyLimit(int distanceMm, int speedDegPerSec, int dutyLimit);

    // 両輪を逆向きに回してその場でturnDeg旋回する（閉ループのturnByImuより速い）
    void turnInPlaceByImu(int leftPwm, int rightPwm, float turnDeg);

    // 帰りの線探し。色判定(isOnColors)ではなく反射率のしきい値判定で、片輪ピボットのまま回し続ける。
    // isRightTurnがtrueなら右旋回、falseなら左旋回（Rコース用に反転）。戻り値: 線を見つけられたか
    bool pivotUntilReflectionBelow(int reflectionThreshold, int pwm, bool isRightTurn);

    // 直角コーナー用。内輪を落として旋回し、線を見つけたら止まる。
    // minTurnDegまでは線を見つけても無視する（元の線を掴むのを防ぐ。探索用途では0を渡す）。
    // turnedDegOutには、線を見つけた（もしくは打ち切った）時点での旋回量を返す
    bool pivotUntilLineFound(bool isLeftTurn, int outerPwm, int innerPwm, float minTurnDeg, float maxTurnDeg, float& turnedDegOut);

    // コーナー検知後の旋回。行き（左折）と帰り（右折）で共用する。
    // 完了判定にはstartHeading（検知時の方位）からの実方位差を使う
    CornerResult turnAtCorner(bool isLeftTurn, float startHeading);

    // コーナー検知の1周期分の更新。ライントレースのループから毎周期呼ぶ。
    // labelはログの接頭辞（行き/帰りの区別用）
    void updateCornerDetection(CornerState& state, bool isLeftTurn, Tracer& tracer, bool isOnBlue, const char* label);
};

#endif  // !DELIVERYTASK_H_