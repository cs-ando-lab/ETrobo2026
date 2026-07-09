#ifndef GAMERUNNER_H_
#define GAMERUNNER_H_

#include "Robot.h"
#include "debug_log.h"

/**
 * 走行の全体フローを統括するクラス。run()を読めば処理の流れがすべてわかる。
 */
class GameRunner {
public:
    GameRunner(Robot& robot);
    void run();  // ← ここを読めば全体の処理フローがわかる

private:
    Robot& robot;
    debug_sensors_t sensors;  // BLE Monitorへのセンサー値送信用
    int debugLogCount = 0;

    // 青ゲートを検出するまでライントレースする
    // センターボタンで中断された場合はfalseを返す（呼び出し側はそこで走行を打ち切る）
    bool lineTraceUntilLap();
};

#endif  // !GAMERUNNER_H_
