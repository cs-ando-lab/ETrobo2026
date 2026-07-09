#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "Robot.h"

/**
 * 走行開始前の準備（L/Rコース選択、スタート合図待ち）を担当するクラス。
 */
class Calibrator {
public:
    Calibrator(Robot& robot);

    // ビープ音→BLE接続待ち→L/Rコース選択→スタート待ちを一通り行う
    void run();

private:
    Robot& robot;

    // 左右ボタンでコースを選びながら、フォースセンサーが押されるまで待機する
    void selectCourseAndWaitForStart();
};

#endif  // !CALIBRATOR_H_
