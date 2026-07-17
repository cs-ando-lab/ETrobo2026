#ifndef ODOMETRY_H_
#define ODOMETRY_H_

/**
 * 旋回・直進の実測値を積算し、自己位置(x, y, 向き)を追跡するクラス。
 * SumoTaskの復路で、現在地からLAPゲート(原点)までの経路をその都度直接計算するために使う。
 */
class Odometry {
public:
    // 現在地からLAPゲート(原点)へ戻るための3手順の計算結果
    struct ReturnPlan {
        float turnToFaceOriginDeg;        // 原点方向を向くための旋回角[°]
        float distanceToOriginMm;         // 原点までの直進距離[mm]
        float finalHeadingCorrectionDeg;  // 原点到着後、最初の向き(0°)に戻るための旋回角[°]
    };

    // 自己位置をLAPゲート地点(x=0, y=0, heading=0°)にリセットする
    void reset();

    // turnByImu/turnByImuUntilUltrasonicで実際に旋回できた角度を反映する
    void applyTurn(float actualTurnedDeg);

    // driveStraightで実際に走行できた距離を反映する(現在の向きに沿って移動したものとして積算する)
    void applyDrive(float actualDrivenMm);

    float getHeadingDeg() const { return headingDeg; }
    float getX() const { return x; }
    float getY() const { return y; }

    // 現在地からLAPゲート(原点)へ戻るための3手順を計算する
    ReturnPlan computeReturnPlan() const;

private:
    static float normalizeAngle(float deg);  // -180〜180の範囲に正規化する

    float x = 0.0f;           // [mm] 原点からの変位
    float y = 0.0f;           // [mm]
    float headingDeg = 0.0f;  // [°] 原点での向きを0とした相対角(turn系と同じ符号: + = 右旋回方向)
};

#endif  // !ODOMETRY_H_
