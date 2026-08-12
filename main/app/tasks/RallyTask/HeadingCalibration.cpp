#include "HeadingCalibration.h"

#include "t_syslog.h"

void HeadingCalibration::updateSample(float currentGyroYaw) {
    // リングバッファに新しいジャイロ角を登録
    samples[writeIndex] = currentGyroYaw;
    writeIndex = getNextIndex(writeIndex);

    {  // debug
        int gyroYaw100 = currentGyroYaw * 100;
        syslog(LOG_NOTICE, "%d.%02d", gyroYaw100 / 100, gyroYaw100 < 0 ? -gyroYaw100 % 100 : gyroYaw100 % 100);
    }

    // リングバッファが埋まるまでサイズをインクリメント
    if(!isFull) {
        size++;
        if(size >= Config::ETRALLY_HEADING_CALIBRATION_BUFFER_SIZE)
            isFull = true;
    }
}

bool HeadingCalibration::isSampleEnough() const {
    return isFull;
}

float HeadingCalibration::getReferenceGyroYaw() const {
    float gyroYawSum = 0.0f;
    size_t lastIndex = writeIndex;
    for(size_t i = 0; i < Config::ETRALLY_HEADING_CALIBRATION_EXCLUSION_COUNT; i++) {
        lastIndex = getPreviousIndex(lastIndex);
    }
    for(size_t i = 0; i < Config::ETRALLY_HEADING_CALIBRATION_SAMPLE_COUNT; i++) {
        lastIndex = getPreviousIndex(lastIndex);
        gyroYawSum += samples[lastIndex];
    }

    return (gyroYawSum / Config::ETRALLY_HEADING_CALIBRATION_SAMPLE_COUNT);
}

size_t HeadingCalibration::getNextIndex(size_t currentIndex) const {
    size_t nextIndex = currentIndex + 1;
    if(nextIndex >= Config::ETRALLY_HEADING_CALIBRATION_BUFFER_SIZE)
        nextIndex = 0;
    return nextIndex;
}

size_t HeadingCalibration::getPreviousIndex(size_t currentIndex) const {
    if(currentIndex == 0)
        return Config::ETRALLY_HEADING_CALIBRATION_BUFFER_SIZE - 1;
    return currentIndex - 1;
}
