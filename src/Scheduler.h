#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "config.h"
#include <Arduino.h>

class Scheduler {
public:
    Scheduler();
    void begin();
    void run(uint8_t currentHour, uint8_t currentMinute);
    uint8_t getBrightness(uint8_t channel);

private:
    uint8_t peakBrightness[NUM_LED_CHANNELS];
    uint8_t currentBrightness[NUM_LED_CHANNELS];
    
    float cubicBezier(float t);
    void calculateAutoBrightness(int currentTimeMinutes);
};

#endif // SCHEDULER_H
