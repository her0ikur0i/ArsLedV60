#include "Scheduler.h"

Scheduler::Scheduler() {
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        currentBrightness[i] = 0;
    }
}

void Scheduler::begin() {
    // No log output (clean)
}

float Scheduler::cubicBezier(float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    float mt = 1 - t;
    float mt2 = mt * mt;
    // P0(0,0), P1(0.42, 0), P2(0.58, 1), P3(1,1) is a common smooth curve
    // The coefficients 0.42 and 0.58 create the S-curve shape for gradual start/end
    return 3 * mt2 * t * 0.42 + 3 * mt * t2 * 0.58 + t3;
}

void Scheduler::calculateAutoBrightness(int currentMinutes) {
    const int scheduleOn = SCHED_ON_HOUR * 60 + SCHED_ON_MINUTE;
    const int schedulePeakStart = SCHED_PEAK_START_HOUR * 60 + SCHED_PEAK_START_MINUTE;
    const int schedulePeakEnd = SCHED_PEAK_END_HOUR * 60 + SCHED_PEAK_END_MINUTE;
    const int scheduleOff = SCHED_OFF_HOUR * 60 + SCHED_OFF_MINUTE;
    
    SchedulePhase phase;
    if(currentMinutes < scheduleOn || currentMinutes >= scheduleOff) {
        phase = PHASE_OFF;
    } else if(currentMinutes < schedulePeakStart) {
        phase = PHASE_SUNRISE; // 08:00 - 10:00 (Dim Up)
    } else if(currentMinutes < schedulePeakEnd) {
        phase = PHASE_PEAK; // 10:00 - 16:00 (Peak)
    } else {
        phase = PHASE_SUNSET; // 16:00 - 20:00 (Dim Down)
    }
    
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        float brightness = 0.0f;
        
        switch(phase) {
            case PHASE_OFF:
                brightness = 0.0f;
                break;
            case PHASE_SUNRISE: {
                // Progress from scheduleOn to schedulePeakStart
                int duration = schedulePeakStart - scheduleOn;
                int elapsed = currentMinutes - scheduleOn;
                float progress = (float)elapsed / (float)duration;
                brightness = cubicBezier(progress) * peakBrightness[i];
                break;
            }
            case PHASE_PEAK:
                brightness = peakBrightness[i];
                break;
            case PHASE_SUNSET: {
                // Progress from schedulePeakEnd to scheduleOff
                int duration = scheduleOff - schedulePeakEnd;
                int elapsed = currentMinutes - schedulePeakEnd;
                float progress = (float)elapsed / (float)duration;
                // We want 1.0 at start of sunset, 0.0 at end, so use (1.0 - curve)
                brightness = peakBrightness[i] * (1.0f - cubicBezier(progress));
                break;
            }
        }
        currentBrightness[i] = (uint8_t)constrain(brightness, 0, 100);
    }
}

void Scheduler::run(int currentHour, int currentMinute) {
    int currentMinutes = currentHour * 60 + currentMinute;
    calculateAutoBrightness(currentMinutes);
}

uint8_t Scheduler::getBrightness(uint8_t channel) {
    if(channel >= NUM_LED_CHANNELS) return 0;
    return currentBrightness[channel];
}

bool Scheduler::isLEDsOn() {
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        if(currentBrightness[i] > 0) return true;
    }
    return false;
}
