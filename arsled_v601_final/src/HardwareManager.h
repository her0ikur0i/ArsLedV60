#ifndef HARDWAREMANAGER_H
#define HARDWAREMANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <INA226.h>
#include <U8g2lib.h>
#include "config.h"

class HardwareManager {
private:
    RTC_DS3231 rtc;
    INA226 ina226;
    U8G2_SH1106_128X64_NONAME_F_HW_I2C display;
    
    const uint8_t ledPins[NUM_LED_CHANNELS] = {LED_RB_PIN, LED_CW_PIN, LED_B_PIN, LED_FS_PIN};
    
    // NTC Temperature Readings
    float lastWaterTemp = 25.0f;
    float lastSinkTemp = 25.0f;
    float lastRoomTemp = 25.0f;
    
    // Power Monitoring
    float totalKwh = 0.0f;
    unsigned long lastPowerUpdate = 0;
    
    // Hardware Status
    bool isRTC_OK = false;
    bool isINA226_OK = false;
    bool isOLED_OK = false;
    bool isLED_OK = false;
    bool isFAN_OK = false;
    bool isTEMP_OK = false;
    
    // Time Sync Variables
    bool isPCTimeReceived = false;
    char pcTimeStr[9] = "";
    char pcDateStr[11] = "";
    
    // Internal functions
    float readThermistor(int pin); // NTC Steinhart-Hart implementation
    void setupPWMChannels();
    void updateTemperatures(); // Modified to poll NTCs
    void updatePowerMonitoring();
    void updateFanControl();
    void updateOLEDDisplay(uint8_t brightness[]);
    
    void scanI2CDevices();
    bool verifyRTC();
    bool verifyINA226();
    bool verifyOLED();
    
public:
    HardwareManager();
    void begin();
    void run(uint8_t brightness[]);
    void setChannelBrightness(uint8_t channel, uint8_t percent);
    
    // Getters for Live Dashboard
    float getCurrentWatts();
    float getTotalKwh() { return totalKwh; }
    float getCostRp();
    DateTime getRTCTime();
    uint8_t getCurrentFanSpeed();
    float getWaterTemp() { return lastWaterTemp; }
    float getRoomTemp() { return lastRoomTemp; }
    float getSinkTemp() { return lastSinkTemp; }
    
    // Time Adjustment
    void adjustRTCTime(int h, int m, int s);
    void adjustRTCDate(int y, int m, int d);
    
    // Summary Printing (Public for Fasa 1)
    void printBootSummary(unsigned long bootTime);
    void printLiveDashboard(uint8_t brightness[]); // New for Fasa 2
    
    bool isPWM_OK = false;
    // Status Getters for main.cpp
    bool isSystemTimeOK() { return isRTC_OK || isPCTimeReceived; }
    bool getStatus(const char* component);
};

#endif
