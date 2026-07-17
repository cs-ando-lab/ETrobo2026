#include "Odometry.h"
#include "Config.h"
#include <cmath>

void Odometry::reset() {
    x = 0.0f;
    y = 0.0f;
    headingDeg = 0.0f;
}

void Odometry::applyTurn(float actualTurnedDeg) {
    headingDeg = normalizeAngle(headingDeg + actualTurnedDeg);
}

void Odometry::applyDrive(float actualDrivenMm) {
    // headingDegは「+ = 右旋回」という向きの符号を採用しているため、
    // x-y平面上では時計回りに角度が増える向きになる(= yの符号を反転させて積算する)
    float headingRad = headingDeg * Config::PI / 180.0f;
    x += actualDrivenMm * std::cos(headingRad);
    y -= actualDrivenMm * std::sin(headingRad);
}

Odometry::ReturnPlan Odometry::computeReturnPlan() const {
    float dx = -x;
    float dy = -y;

    // applyDriveと同じ符号規約(yを反転)に合わせてbearingを求める
    float bearingRad = std::atan2(-dy, dx);
    float bearingDeg = bearingRad * 180.0f / Config::PI;

    ReturnPlan plan;
    plan.turnToFaceOriginDeg = normalizeAngle(bearingDeg - headingDeg);
    plan.distanceToOriginMm = std::sqrt(dx * dx + dy * dy);
    plan.finalHeadingCorrectionDeg = normalizeAngle(-bearingDeg);
    return plan;
}

float Odometry::normalizeAngle(float deg) {
    while(deg > 180.0f) {
        deg -= 360.0f;
    }
    while(deg <= -180.0f) {
        deg += 360.0f;
    }
    return deg;
}
