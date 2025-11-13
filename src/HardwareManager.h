#ifndef HARDWAREMANAGER_H
#define HARDWAREMANAGER_H

#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_SH110X.h> // FIX V60: Ganti SSD1306/U8g2
#include <Adafruit_GFX.h>
#include "config.h"

// --- Public Display Object (SH1106G) ---
extern Adafruit_SH1106G display;

// --- HardwareManager Class ---
class HardwareManager {
public:
    HardwareManager();
    void begin();
    void run(const uint8_t currentBrightness[], bool isLEDsOn);
    void setChannelBrightness(uint8_t channel, uint8_t brightness);
    DateTime getRTCTime();
    void setRTCTime(const DateTime& dt);
    void printBootSummary();

private:
    bool isRTCReady;
    bool isOLEDReady;
    bool isINA226Ready;
    bool isAHT20Ready;
    bool isI2CPing(uint8_t address);
    void initOLED();
    void initPWM();
    void initFanPWM();
    void updateOLEDDisplay(const uint8_t currentBrightness[], bool isLEDsOn);
};

#endif // HARDWAREMANAGER_H
