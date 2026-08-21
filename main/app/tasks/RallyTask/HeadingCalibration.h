#ifndef HEADING_CALIBRATION_H_
#define HEADING_CALIBRATION_H_

#include "Config.h"
#include <cstddef>

// 直近のジャイロ角を保持しておき、停止直前の5サンプルを除外し、
// その直前の10サンプルの方位を平均したものを基準角とする。
class HeadingCalibration {
public:
    void updateSample(float gyroYaw);   // ジャイロ角をリングバッファに登録する。
    bool isSampleEnough() const;        // 必要なサンプル数がたまっていればtrueを返す
    float getReferenceGyroYaw() const;  // 基準ジャイロ角を取得する。

private:
    float samples[Config::ETRALLY_HEADING_CALIBRATION_BUFFER_SIZE]{};  // サンプルを保持するリングバッファ
    size_t writeIndex = 0;
    size_t size = 0;
    bool isFull = false;

    // utility
    size_t getNextIndex(size_t currentIndex) const;
    size_t getPreviousIndex(size_t currentIndex) const;
};

#endif  // HEADING_CALIBRATION_H_
