#ifndef DELIVERYTASK_H_
#define DELIVERYTASK_H_

#include "Robot.h"

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

    // センターボタンによる中断。各ヘルパが立て、run()が段階ごとに確認して打ち切る。
    // 「探索に失敗した」と「人が止めた」を区別しないと、中断がエリア到達として扱われてしまう
    bool aborted = false;

    // コーナー旋回の結果。「線を見つけたか」だけでは曲がりきったか判断できないため3状態にする
    enum class CornerResult {
        CLEARED,     // 線を発見し、検知時からの方位差が完了角度以上
        REACQUIRED,  // 線には復帰したが、まだ完了角度に届いていない
        FAILED       // 正方向・振り戻しとも線を発見できなかった
    };

    // 青判定の診断集計。制御で使ったのと同じ読み取り値だけを渡す（診断のためにセンサーを追加で読まない）
    struct BlueStats {
        int maxSaturationInHue = 0;  // 青の色相範囲内で見えた最大彩度
        int satisfiedCount = 0;      // 青条件を満たしたサンプルの総数
        int maxConsecutive = 0;      // 青条件を満たした最大連続回数（確定に必要な回数と直接比較できる）
        int currentRun = 0;
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

        // 診断専用。検知に使われた白連続（＝しきい値に達したもの）は除いて最大値を取るので、
        // 「直線部でどこまで白が続くか」＝しきい値までのマージンがそのまま読める
        int measureWhiteRun = 0;
        int maxWhiteRunBeforeTrigger = 0;
        // 検知を成立させた白連続の中で最も暗かった反射率。しきい値ぎりぎり(85〜92)の連続で
        // 発火したのか、完全に線を外した白(98〜99)だったのかを切り分ける
        int minReflectionInWhiteRun = 101;
    };

    // ボトルの色判定。ColorJudge::judge()の暗所ガードが離れたボトルには合わないため、彩度とHueだけで判定する
    ColorJudge::Color judgeBottleColor() const;

    // 上記を連続一致で確定させる。決まらなければUNKNOWN（呼び出し側で走行を打ち切る）
    ColorJudge::Color confirmBottleColor();

    // 確定後にまとめてログへ出すための、最後に読んだボトルの生値
    mutable ColorJudge::Reading lastBottleReading{};

    // 青ライン判定。ColorJudgeの青判定は白黒グラデーションを拾ってしまうため、彩度で絞った独自判定にする。
    // 読んだ値はそのままstatsへ集計する
    bool isBlueReading(BlueStats& stats) const;
    bool isOnBlueLine(int& matchedCount, int stableCount, BlueStats& stats) const;
    void logBlueStats(const char* label, const BlueStats& stats) const;

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

    // 曲がるべき向きを正とした、startHeadingからの回頭量（逆方向へ回った分はマイナス）
    float signedTurnFrom(float startHeading, bool isLeftTurn) const;

    // コーナー検知後の旋回。行き（左折）と帰り（右折）で共用する。
    // 完了判定にはstartHeading（検知時の方位）からの実方位差を使う
    CornerResult turnAtCorner(bool isLeftTurn, float startHeading, float minTurnDeg);

    // コーナー検知の1周期分の更新。ライントレースのループから毎周期呼ぶ。
    // labelはログの接頭辞（行き/帰りの区別用）
    void updateCornerDetection(CornerState& state, bool isLeftTurn, float minTurnDeg, Tracer& tracer, bool isOnBlue, const char* label);
};

#endif  // !DELIVERYTASK_H_