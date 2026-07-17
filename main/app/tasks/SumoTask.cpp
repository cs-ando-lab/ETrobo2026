#include "SumoTask.h"
#include "CourseConfig.h"
#include "kernel.h"
#include "t_syslog.h"

SumoTask::SumoTask(Robot& robot)
    : robot(robot) {
}

void SumoTask::run() {
    odometry.reset();
    syslog(LOG_NOTICE, "SUMO,RUN,START,course=%d", CourseConfig::isLeftCourse() ? 0 : 1);

    // 0. ライントレースによるLAPゲートへの到達はGameRunnerが行う（呼び出し時点で完了している前提）

    // 1. 土俵の方向へ旋回し、縁まで直進する
    faceRingAndApproach();

    // 2. 連続旋回しながら超音波センサでボトルを探索する
    bool found = searchBottle();
    syslog(LOG_NOTICE, "SUMO,RUN,SEARCH_DONE,found=%d", found ? 1 : 0);

    // 3. 土俵の直径分前進し、土俵外へ押し出す
    pushBottle();

    // TODO: 土俵外に確実に押し出せたか判定する処理を追加

    // TODO: 押し出し後に returnToStart() によって回転する際に、ボトルを倒してしまう可能性があるため、後退する処理を追加

    // 4. Odometryの計算結果をもとにLAPゲートへ戻り、最後に黒ラインへ物理的に合流アンカーする
    returnToStart();

    syslog(LOG_NOTICE, "SUMO,RUN,END");
}

void SumoTask::faceRingAndApproach() {
    // 土俵の方向へ旋回する（L/Rコースで旋回方向が反転するため、sign()で補正する）
    float reqTurnDeg = CourseConfig::sign() * Config::SUMO_TURN_TO_RING_DEG;
    float turnedDeg = robot.turnByImu(reqTurnDeg, Config::SUMO_APPROACH_TURN_SPEED_DEG_PER_SEC);
    odometry.applyTurn(turnedDeg);
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);
    syslog(LOG_NOTICE, "SUMO,TURN,reqDeg=%d,actualDeg=%d", (int)reqTurnDeg, (int)turnedDeg);

    int drivenMm = robot.driveStraight(Config::SUMO_DRIVE_TO_RING_MM, Config::SUMO_APPROACH_DRIVE_SPEED_DEG_PER_SEC);
    odometry.applyDrive(static_cast<float>(drivenMm));
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);
    syslog(LOG_NOTICE, "SUMO,DRIVE,reqMm=%d,actualMm=%d", Config::SUMO_DRIVE_TO_RING_MM, drivenMm);

    syslog(LOG_NOTICE, "SUMO,ODOM,x=%d,y=%d,heading=%d", (int)odometry.getX(), (int)odometry.getY(), (int)odometry.getHeadingDeg());
}

bool SumoTask::searchBottle() {
    syslog(LOG_NOTICE, "SUMO,SEARCH,START,maxAngle=%d,detectMm=%d", (int)Config::SUMO_SEARCH_MAX_ANGLE_DEG, Config::SUMO_BOTTLE_DETECT_DISTANCE_MM);

    // 走査範囲の左端(-x°)まで一度で旋回する（ここではまだ超音波センサーで確認する必要がないため、
    // 探索用のゆっくりした速度(SUMO_SEARCH_SPEED_DEG_PER_SEC)ではなく通常の旋回速度で素早く移動する）
    float turnedToStart = robot.turnByImu(-Config::SUMO_SEARCH_MAX_ANGLE_DEG);
    odometry.applyTurn(turnedToStart);
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);
    syslog(LOG_NOTICE, "SUMO,SEARCH,TO_START,actualDeg=%d", (int)turnedToStart);

    // 左端から右端(+x°)まで、超音波センサで確認しながら旋回する
    Robot::SearchResult result = robot.turnByImuUntilUltrasonic(
        2.0f * Config::SUMO_SEARCH_MAX_ANGLE_DEG,
        Config::SUMO_BOTTLE_DETECT_DISTANCE_MM,
        Config::SUMO_SEARCH_SPEED_DEG_PER_SEC);
    odometry.applyTurn(result.actualTurnedDeg);
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);
    syslog(LOG_NOTICE, "SUMO,SEARCH,SWEEP,actualDeg=%d,found=%d,bestHeading=%d,bestDist=%d",
           (int)result.actualTurnedDeg, result.found ? 1 : 0, (int)result.bestHeadingDeg, result.bestDistanceMm);

    if(result.found) {
        // 対象物に最も正対していたと推定される向きまで旋回し直す
        float correctionDeg = result.bestHeadingDeg - result.actualTurnedDeg;
        float turnedBack = robot.turnByImu(correctionDeg);
        odometry.applyTurn(turnedBack);
        dly_tsk(Config::SUMO_MOVE_SETTLE_US);
        syslog(LOG_NOTICE, "SUMO,SEARCH,CENTER,correctionDeg=%d,actualDeg=%d", (int)correctionDeg, (int)turnedBack);
    }

    if(!result.found) {
        // 走査範囲内でボトルを検知できなかった場合、土俵中央付近にいる想定でそのまま押し出しを試みる
        syslog(LOG_NOTICE, "SUMO,BOTTLE_NOT_FOUND,heading=%d", (int)odometry.getHeadingDeg());
    } else {
        syslog(LOG_NOTICE, "SUMO,BOTTLE_FOUND,heading=%d", (int)odometry.getHeadingDeg());
    }
    return result.found;
}

void SumoTask::pushBottle() {
    int drivenMm = robot.driveStraight(Config::SUMO_RING_DIAMETER_MM, Config::SUMO_PUSH_SPEED_DEG_PER_SEC);
    odometry.applyDrive(static_cast<float>(drivenMm));
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);
    syslog(LOG_NOTICE, "SUMO,PUSH,reqMm=%d,actualMm=%d", Config::SUMO_RING_DIAMETER_MM, drivenMm);
}

void SumoTask::returnToStart() {
    Odometry::ReturnPlan plan = odometry.computeReturnPlan();
    syslog(LOG_NOTICE, "SUMO,RETURN,PLAN,turnDeg=%d,distMm=%d,finalTurnDeg=%d",
           (int)plan.turnToFaceOriginDeg, (int)plan.distanceToOriginMm, (int)plan.finalHeadingCorrectionDeg);

    float turned1 = robot.turnByImu(plan.turnToFaceOriginDeg, Config::SUMO_RETURN_TURN_SPEED_DEG_PER_SEC);
    odometry.applyTurn(turned1);
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);

    int driven = robot.driveStraight(static_cast<int>(plan.distanceToOriginMm), Config::SUMO_RETURN_DRIVE_SPEED_DEG_PER_SEC);
    odometry.applyDrive(static_cast<float>(driven));
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);

    float turned2 = robot.turnByImu(plan.finalHeadingCorrectionDeg, Config::SUMO_RETURN_TURN_SPEED_DEG_PER_SEC);
    odometry.applyTurn(turned2);
    dly_tsk(Config::SUMO_MOVE_SETTLE_US);

    syslog(LOG_NOTICE, "SUMO,RETURN,ODOM,x=%d,y=%d,heading=%d", (int)odometry.getX(), (int)odometry.getY(), (int)odometry.getHeadingDeg());

    // 自己位置推定だけに頼らず、最後は蛇行走行で黒ラインへ物理的に合流アンカーする
    robot.runWavingUntilColor(ColorJudge::Color::BLACK);
    syslog(LOG_NOTICE, "SUMO,RETURN,DONE");
}
