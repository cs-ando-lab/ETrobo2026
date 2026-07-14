#include "DeliveryTask.h"
#include "Tracer.h"   
#include "Config.h"   
#include <kernel.h>
#include <t_syslog.h>
#include <cmath>

DeliveryTask::DeliveryTask(Robot& robot)
    : robot(robot) {
}

void DeliveryTask::run() {
    syslog(LOG_NOTICE, "--- DeliveryTask Started ---");

    Tracer tracer(robot);
    tracer.setPwm(Config::DELIVERY_TRACER_PWM);
    bool hasBeeped = false;

    // 1. ライントレースしながらボトルに近づく
    while (true) {
        int currentDistance = robot.getUltrasonicDistance();
        
        if (!hasBeeped && currentDistance > 0 && currentDistance < 150) {
            robot.beep(100); 
            hasBeeped = true; 
        }

        if (currentDistance > 0 && currentDistance <= Config::DELIVERY_TARGET_DISTANCE_MM) {
            tracer.terminate(); 
            break;
        }

        tracer.run();
        dly_tsk(Config::LINE_TRACE_POLL_INTERVAL_US);
    }

    Motor armMotor(EPort::PORT_C, Motor::EDirection::CLOCKWISE, true);

    // 2. ボトルの前でアームを上げる
    raiseArm(armMotor);

    // センサーの値を安定させるための待機
    dly_tsk(500 * 1000); 

    // 3. ボトルの色を判定
    ColorJudge::Color bottleColor = robot.getColor();

    // 判定結果に応じてビープ音を鳴らす
    switch (bottleColor) {
        case ColorJudge::Color::YELLOW:
            syslog(LOG_NOTICE, "Bottle Color: YELLOW");
            robot.beep(100); // 1回
            break;
            
        case ColorJudge::Color::BLUE:
            syslog(LOG_NOTICE, "Bottle Color: BLUE");
            robot.beep(100); // 1回目
            dly_tsk(100 * 1000);
            robot.beep(100); // 2回目
            break;
            
        case ColorJudge::Color::RED:
            syslog(LOG_NOTICE, "Bottle Color: RED");
            robot.beep(100); // 1回目
            dly_tsk(100 * 1000);
            robot.beep(100); // 2回目
            dly_tsk(100 * 1000);
            robot.beep(100); // 3回目
            break;
            
        default:
            syslog(LOG_NOTICE, "Bottle Color: UNKNOWN");
            break;
    }

    // ここから先で bottleColor を使って運搬先の分岐処理などを書く
    // ...

    // 4. アームを下げる
    lowerArm(armMotor);

    syslog(LOG_NOTICE, "--- DeliveryTask Finished ---");
}

void DeliveryTask::raiseArm(Motor& armMotor) {
    armMotor.resetCount(); 
    int targetMoveDeg = 135; 
    armMotor.setSpeed(-100); 

    while (std::abs(armMotor.getCount()) < targetMoveDeg) {
        if (robot.isCenterButtonPressed()) break;
        dly_tsk(Config::MOTION_POLL_INTERVAL_US); 
    }
    armMotor.stop(); 
}

void DeliveryTask::lowerArm(Motor& armMotor) {
    armMotor.resetCount(); 
    int targetMoveDeg = 150; 
    armMotor.setSpeed(100);  

    while (std::abs(armMotor.getCount()) < targetMoveDeg) {
        if (robot.isCenterButtonPressed()) break;
        dly_tsk(Config::MOTION_POLL_INTERVAL_US); 
    }
    armMotor.stop(); 
}