#ifndef RALLYTASK_H_
#define RALLYTASK_H_

#include "Robot.h"
#include "Tracer.h"
#include "Config.h"
#include "CourseConfig.h"
#include "RallyTask/RallyTypes.h"
#include "RallyTask/RallyRoute.h"
#include <vector>

#include <libcpp/spike/Clock.h>  // debug用

/**
 * ETラリーの処理を行うクラス。
 * run()を1回呼べば、赤→青→黄のゲートを順番に通過する。
 */
class RallyTask {
public:
    RallyTask(Robot& robot);
    void run();
    void test();

private:
    Robot& robot;

    float referenceGyroYaw = 0.0f;

    // ゲートを通る順番を保持する配列
    static const RallyTypes::Gate gatesSequence[3];

    void turn(float degree, int delayTimeUs = 100 * 1000);
    float calculateTurnAngle(float degree);                                                                               // 基準角度に対する角度を受け取り、現時点のgetHeading角度との差を求める
    void traceLineforDistance(float distance, Tracer tracer);                                                             // 指定した距離までライントレースを実施
    void calibrateHeadingByLineTrace(Tracer tracer);                                                                      // IMUの初期化ができなかった場合falseを返す
    void followNodeSegments(std::vector<RallyTypes::Segment> segments, int speed = Config::ETRALLY_DEFAULT_DRIVE_SPEED);  // vector<RallyTypes::Segment>で指定された通りのルートを走行する。

    void logAngle(const char* s);

    class AlignmentRingBuffer {  // ライントレース中、ウィンドウ時間分のセンサ値を保持し、安定直進区間を見つける。
    public:
        // 安定直進区間を判定するためのセンサー値を保持する構造体
        struct AlignmentSample {
            int reflectionError;  // 反射光偏差 (反射光目標値 - 反射光値)
            float gyroRate;       // ジャイロ角速度 (imu: x軸)
            float gyroYaw;        // ジャイロ角 (imu: x軸)
        };

        // 最も信頼できる安定直進区間を保持する構造体
        struct Candidate {
            float reflectionErrorMean;      // 区間の平均反射光偏差
            float reflectionErrorVariance;  // 区間の反射光偏差の分散
            float gyroRateMeanSquare;       // 区間のジャイロ角速度の分散
            float gyroYawAverage;           // 区間の平均ジャイロ角
            float score;                    // 区間の信頼スコア（スコアが低いほど信頼できる）
        };

        void push(AlignmentSample sample);  // 現在区間のセンサー値をリングバッファに登録し、区間の合計を更新する
        bool isBufferFull() const;          // ウィンドウ時間分のデータがリングバッファにたまっていればtrueを返す
        bool judgeSamples();                // 現在区間が安定直進区間のための条件を満たすか判定し、満たせばtrueを返す
        float getReferenceGyroYaw();        // 基準ジャイロ角を取得する。

    private:
        AlignmentSample samples[Config::ETRALLY_ALIGNMENT_BUFFER_SIZE]{};
        Candidate bestCandidate{};

        spikeapi::Clock clock;

        size_t wirteIndex = 0;
        size_t size = 0;
        bool hasBestCandidate = false;
        bool isFull = false;

        // 現在区間の各合計値
        int reflectionErrorSum = 0.0f;        // 反射光偏差の区間合計
        int reflectionErrorSquareSum = 0.0f;  // 反射光偏差二乗の区間合計
        double gyroRateSquareSum = 0.0f;      // ジャイロ角速度二乗の区間合計
        double gyroYawSum = 0.0f;             // ジャイロ角の区間合計

        double getScore(double reflectionErrorVariance,  // 区間のスコアを算出
                        double gyroRateVariance) const;
        bool compareCandidates(Candidate newcomer);  // 最良区間と候補区間を比較して最良区間が更新されればtrueを返す。

        // utility
        size_t getNextIndex(size_t currentIndex) const;
        size_t getPreviousIndex(size_t currentIndex) const;
        int square(int value) const;
        double dSquare(double value) const;
    };
};

#endif  // !RALLYTASK_H_
