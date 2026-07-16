#ifndef GAMERUNNER_H_
#define GAMERUNNER_H_

#include "Robot.h"

/**
 * 走行の全体フローを統括するクラス。run()を読めば処理の流れがすべてわかる。
 *
 * センサー値のBLE Monitor送信は、電源投入時から常に動く独立タスク(app.cppのdebug_log_task)が
 * 行うため、GameRunnerはそれを意識しない。
 */
class GameRunner {
public:
    GameRunner(Robot& robot);
    void run();  // ← ここを読めば全体の処理フローがわかる

private:
    Robot& robot;

    // 青ゲートを検出するまでライントレースする
    // センターボタンで中断された場合はfalseを返す（呼び出し側はそこで走行を打ち切る）
    bool lineTraceUntilLap();
};

#endif  // !GAMERUNNER_H_
