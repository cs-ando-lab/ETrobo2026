#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include "ColorSensor.h"
#include "Motor.h"
#include "UltrasonicSensor.h"
#include "ForceSensor.h"

/* 0 にするとログ出力がすべて無効化される */
#define DEBUG_LOG 1

typedef struct {
    const spikeapi::ColorSensor       *color;
    const spikeapi::Motor             *left_motor;
    const spikeapi::Motor             *right_motor;
    const spikeapi::UltrasonicSensor  *ultrasonic;
    const spikeapi::ForceSensor       *force;
} debug_sensors_t;

#if DEBUG_LOG
void debug_log_init(const debug_sensors_t *sensors);
void debug_log_all(const debug_sensors_t *sensors, int count);
#else
#define debug_log_init(sensors)        ((void)0)
#define debug_log_all(sensors, count)  ((void)0)
#endif

#endif /* DEBUG_LOG_H */
