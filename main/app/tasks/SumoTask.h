#ifndef SUMOTASK_H_
#define SUMOTASK_H_

#include "Robot.h"
#include "Odometry.h"
#include "Config.h"

/**
 * ET相撲の処理を行うクラス。
 * run()を1回呼べば、土俵に行ってボトルを押し出し、ラインに戻ってくる。
 *
 * 復路はOdometryで自己位置(x, y, 向き)を継続的に追跡し、現在地からLAPゲート(原点)への
 * 経路をその場で計算して直接戻る方式にしている（往路の手順数によらず、復路の手順数は常に一定に保たれる）。
 */
class SumoTask {
public:
    SumoTask(Robot& robot);
    void run();

private:
    Robot& robot;
    Odometry odometry;

    void faceRingAndApproach();  // 土俵の方向へ旋回し、縁まで直進する
    bool searchBottle();         // 連続旋回しながら超音波センサでボトルを探索する
    void pushBottle();           // 土俵の直径分前進し、土俵外へ押し出す
    void returnToStart();        // Odometryの計算結果をもとにLAPゲートへ戻り、最後に黒ラインへ物理的に合流アンカーする
};

#endif  // !SUMOTASK_H_
