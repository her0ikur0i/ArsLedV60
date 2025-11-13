#include "Scheduler.h"
#include <Arduino.h>

Scheduler::Scheduler() {
    // Inisialisasi Kecerahan Puncak
    peakBrightness[0] = PEAK_BRIGHTNESS_WHITE;
    peakBrightness[1] = PEAK_BRIGHTNESS_BLUE;
    peakBrightness[2] = PEAK_BRIGHTNESS_UV;
    
    // Inisialisasi Kecerahan Saat Ini
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        currentBrightness[i] = 0;
    }
}

void Scheduler::begin() {
    // Tidak ada yang dilakukan saat ini
}

// Fungsi Cubic Bezier untuk transisi yang mulus (smooth)
float Scheduler::cubicBezier(float t) {
    // Kontrol point: P0=(0,0), P1=(0.42, 0.0), P2=(0.58, 1.0), P3=(1,1)
    // Formula: (1-t)^3 * P0 + 3*(1-t)^2*t * P1 + 3*(1-t)*t^2 * P2 + t^3 * P3
    // Karena P0=(0,0) dan P3=(1,1):
    return 3.0f * pow(1.0f - t, 2) * t * 0.42f + 3.0f * (1.0f - t) * pow(t, 2) * 0.58f + pow(t, 3);
}

void Scheduler::calculateAutoBrightness(int currentTimeMinutes) {
    // Waktu dalam menit dari 00:00
    const int scheduleOn = SCHED_ON_HOUR * 60 + SCHED_ON_MINUTE;
    const int schedulePeakStart = SCHED_PEAK_START_HOUR * 60 + SCHED_PEAK_START_MINUTE;
    const int schedulePeakEnd = SCHED_PEAK_END_HOUR * 60 + SCHED_PEAK_END_MINUTE;
    const int scheduleOff = SCHED_OFF_HOUR * 60 + SCHED_OFF_MINUTE;

    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        float brightness = 0.0f;
        
        if (currentTimeMinutes < scheduleOn || currentTimeMinutes >= scheduleOff) {
            // OFF period
            brightness = 0.0f;
        } else if (currentTimeMinutes >= schedulePeakStart && currentTimeMinutes < schedulePeakEnd) {
            // PEAK period
            brightness = (float)peakBrightness[i];
        } else if (currentTimeMinutes >= scheduleOn && currentTimeMinutes < schedulePeakStart) {
            // RAMP UP (Sunrise)
            float duration = (float)(schedulePeakStart - scheduleOn);
            float progress = (float)(currentTimeMinutes - scheduleOn) / duration;
            brightness = cubicBezier(progress) * peakBrightness[i];
        } else if (currentTimeMinutes >= schedulePeakEnd && currentTimeMinutes < scheduleOff) {
            // RAMP DOWN (Sunset)
            float duration = (float)(scheduleOff - schedulePeakEnd);
            float progress = (float)(currentTimeMinutes - schedulePeakEnd) / duration;
            brightness = (1.0f - cubicBezier(progress)) * peakBrightness[i];
        }

        // Simpan nilai brightness (constrain 0-100)
        currentBrightness[i] = (uint8_t)constrain(brightness, 0, 100);
    }
}

void Scheduler::run(uint8_t currentHour, uint8_t currentMinute) {
    int currentTimeMinutes = currentHour * 60 + currentMinute;
    calculateAutoBrightness(currentTimeMinutes);
}

uint8_t Scheduler::getBrightness(uint8_t channel) {
    if(channel >= NUM_LED_CHANNELS) return 0;
    return currentBrightness[channel];
}
