#!/bin/bash

# ==============================================================================
# ARSLED MARINE LIGHT V601 - FINAL STANDALONE
# Base: V59 (Stable U8g2)
# Fixes: Dual-Phase Serial Monitoring, Steinhart-Hart NTC 3950/10K
# Status: Final Stable Release
# ==============================================================================

PROJECT_DIR="arsled_v601_final"

echo "=== ArsLed Marine Light V601 Final Script Generator ==="
echo "Base: V59 Stable Logic + NTC Steinhart-Hart + Dual-Phase Monitoring"
echo "Membuat direktori proyek: $PROJECT_DIR"

# Force clean
[ -d "$PROJECT_DIR" ] && rm -rf "$PROJECT_DIR"
mkdir -p "$PROJECT_DIR/src"
cd "$PROJECT_DIR" || exit

# ------------------------------------------------------------------------------
# 1. platformio.ini (Minimal Deps + U8g2)
# ------------------------------------------------------------------------------
echo "1/9: Membuat platformio.ini..."
cat <<'EOT_INI' > platformio.ini
[env:lolin_s2_mini]
platform = espressif32
board = lolin_s2_mini
framework = arduino
monitor_speed = 115200

upload_protocol = esptool
upload_speed = 460800
upload_port = COM5

board_build.f_cpu = 240000000L
board_build.arduino.loop_stack_size = 32768

lib_deps = 
    adafruit/RTClib@^2.1.3
    robtillaart/INA226@^0.6.4
    olikraus/U8g2@^2.35.19
EOT_INI
echo "   -> platformio.ini - OK"

# ------------------------------------------------------------------------------
# 2. sync_time.py
# ------------------------------------------------------------------------------
echo "2/9: Membuat sync_time.py..."
cat <<'EOT_SYNC' > sync_time.py
#!/usr/bin/env python3
import serial
import time
from datetime import datetime
import sys

# Change PORT if necessary
PORT = "COM5" 
BAUD_RATE = 115200
WAIT_TIME = 8

def sync_time():
    now = datetime.now()
    time_str = now.strftime("%H:%M:%S")
    date_str = now.strftime("%y:%m:%d")
    
    print(f"\n=== ArsLed V601 Manual Time Sync ===")
    print(f"Port: {PORT}")
    print(f"Time: {time_str}")
    print(f"Date: {date_str}")
    print(f"\nWaiting {WAIT_TIME} seconds for ESP32 stability...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
        time.sleep(WAIT_TIME)
        
        print("\n[1/2] Sending SETTIME command...")
        cmd_time = f"SETTIME={time_str}\n"
        ser.write(cmd_time.encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Response: {response.strip()}")
        
        print("\n[2/2] Sending SETDATE command...")
        cmd_date = f"SETDATE={date_str}\n"
        ser.write(cmd_date.encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Response: {response.strip()}")
        
        ser.close()
        print("\n✓ Time synchronization completed!")
        
    except serial.SerialException as e:
        print(f"\n✗ ERROR: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        PORT = sys.argv[1] # Allow port override if passed as argument
        
    sync_time()
EOT_SYNC
chmod +x sync_time.py
echo "   -> sync_time.py - OK"

# ------------------------------------------------------------------------------
# 3. src/config.h (Thermistor Constants added)
# ------------------------------------------------------------------------------
echo "3/9: Membuat src/config.h..."
cat <<'EOT_CONFIG' > src/config.h
#ifndef CONFIG_H
#define CONFIG_H

#define NUM_LED_CHANNELS        4
#define LED_RB_PIN              7
#define LED_CW_PIN              8
#define LED_B_PIN               9
#define LED_FS_PIN              10
#define ONBOARD_LED_PIN         15
#define I2C_SDA_PIN             1
#define I2C_SCL_PIN             2
#define THERM_WATER_PIN         3   // ADC Pin for Water NTC
#define THERM_SINK_PIN          4   // ADC Pin for Heatsink NTC
#define THERM_ROOM_PIN          5   // ADC Pin for Room NTC
#define FAN_PIN                 6

#define PWM_FREQUENCY           25000UL
#define PWM_RESOLUTION_BITS     13
#define MAX_DUTY_VALUE          ((1 << PWM_RESOLUTION_BITS) - 1)
#define MAX_PERCENT_LIMIT       85

#define CH_RB                   0
#define CH_CW                   1
#define CH_B                    2
#define CH_FS                   3

// SCHEDULE & BRIGHTNESS
#define SCHED_ON_HOUR           8
#define SCHED_ON_MINUTE         0
#define SCHED_PEAK_START_HOUR   10
#define SCHED_PEAK_START_MINUTE 0
#define SCHED_PEAK_END_HOUR     16
#define SCHED_PEAK_END_MINUTE   0
#define SCHED_OFF_HOUR          20
#define SCHED_OFF_MINUTE        0

#define PEAK_BRIGHT_RB          70
#define PEAK_BRIGHT_CW          30
#define PEAK_BRIGHT_B           100
#define PEAK_BRIGHT_FS          20

#define DEFAULT_KWH_COST        1444
#define LIVE_DASHBOARD_INTERVAL 60000UL // 1 minute in milliseconds
#define BOOT_SUMMARY_DURATION   60000UL // 1 minute in milliseconds
#define HEARTBEAT_INTERVAL      60000UL // 1 minute in milliseconds

// NTC THERMISTOR CONSTANTS (3950 10K)
#define THERM_NOMINAL_RESISTANCE 10000.0f
#define THERM_NOMINAL_TEMP_K     298.15f // 25C in Kelvin
#define THERM_BETA_VALUE         3950.0f
#define THERM_SERIES_RESISTOR    10000.0f // Assuming a 10k resistor in series

#endif
EOT_CONFIG
echo "   -> src/config.h - OK"

# ------------------------------------------------------------------------------
# 4. src/Scheduler.h
# ------------------------------------------------------------------------------
echo "4/9: Membuat src/Scheduler.h..."
cat <<'EOT_SCHED_H' > src/Scheduler.h
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include "config.h"

enum SchedulePhase {
    PHASE_OFF,
    PHASE_SUNRISE,
    PHASE_PEAK,
    PHASE_SUNSET
};

class Scheduler {
private:
    uint8_t peakBrightness[NUM_LED_CHANNELS] = {
        PEAK_BRIGHT_RB, PEAK_BRIGHT_CW, PEAK_BRIGHT_B, PEAK_BRIGHT_FS
    };
    uint8_t currentBrightness[NUM_LED_CHANNELS];
    float cubicBezier(float t);
    void calculateAutoBrightness(int currentMinutes);
    
public:
    Scheduler();
    void begin();
    void run(int currentHour, int currentMinute);
    uint8_t getBrightness(uint8_t channel);
    bool isLEDsOn(); // New utility
};

#endif
EOT_SCHED_H
echo "   -> src/Scheduler.h - OK"

# ------------------------------------------------------------------------------
# 5. src/Scheduler.cpp
# ------------------------------------------------------------------------------
echo "5/9: Membuat src/Scheduler.cpp..."
cat <<'EOT_SCHED_CPP' > src/Scheduler.cpp
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
EOT_SCHED_CPP
echo "   -> src/Scheduler.cpp - OK"

# ------------------------------------------------------------------------------
# 6. src/HardwareManager.h
# ------------------------------------------------------------------------------
echo "6/9: Membuat src/HardwareManager.h..."
cat <<'EOT_HW_H' > src/HardwareManager.h
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
EOT_HW_H
echo "   -> src/HardwareManager.h - OK"

# ------------------------------------------------------------------------------
# 7. src/HardwareManager.cpp (V601 Logic - Steinhart-Hart + Dashboard)
# ------------------------------------------------------------------------------
echo "7/9: Membuat src/HardwareManager.cpp..."
cat <<'EOT_HW_CPP' > src/HardwareManager.cpp
#include "HardwareManager.h"

// Define PI for Steinhart-Hart
#define PI 3.14159265358979323846f

HardwareManager::HardwareManager() 
: ina226(0x40), display(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN) {}

bool HardwareManager::getStatus(const char* component) {
    if (strcmp(component, "OLED") == 0) return isOLED_OK;
    if (strcmp(component, "RTC") == 0) return isRTC_OK;
    if (strcmp(component, "INA226") == 0) return isINA226_OK;
    if (strcmp(component, "LED") == 0) return isLED_OK;
    if (strcmp(component, "FAN") == 0) return isFAN_OK;
    if (strcmp(component, "TEMP") == 0) return isTEMP_OK;
    return false;
}

// =============================================================================
// NTC STEINHART-HART IMPLEMENTATION
// =============================================================================
float HardwareManager::readThermistor(int pin) {
    // Read raw ADC value
    int raw = analogRead(pin);
    
    if (raw < 10) return -100.0f; // Return error value if almost 0
    
    // 1. Calculate Thermistor Resistance (RT)
    // Assumes Vout = R_T / (R_series + R_T)
    // R_T = R_series * (Vout / (Vcc - Vout))
    // ESP32 ADC is 12-bit (4095 max value)
    
    float vout = (float)raw;
    float resistance = THERM_SERIES_RESISTOR / ((4095.0f / vout) - 1.0f);
    
    // 2. Apply Steinhart-Hart Equation
    float steinhart;
    steinhart = resistance / THERM_NOMINAL_RESISTANCE; // (R / R0)
    steinhart = log(steinhart);                        // ln(R / R0)
    steinhart /= THERM_BETA_VALUE;                     // (1/B) * ln(R / R0)
    steinhart += (1.0f / THERM_NOMINAL_TEMP_K);        // + (1 / T0)
    steinhart = 1.0f / steinhart;                      // 1 / [...]
    
    // 3. Convert from Kelvin to Celsius
    return steinhart - 273.15f;
}

// =============================================================================
// HARDWARE VERIFICATION & INITIALIZATION
// =============================================================================

void HardwareManager::scanI2CDevices() {
    Serial.println("\nI2C Scanner: Scanning...");
    byte count = 0;
    
    for(byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        
        if(error == 0) {
            Serial.printf("  Device found at 0x%02X", addr);
            
            if(addr == 0x3C) Serial.print(" (OLED SH1106?)");
            else if(addr == 0x40) Serial.print(" (INA226?)");
            else if(addr == 0x68) Serial.print(" (RTC DS3231?)");
            
            Serial.println();
            count++;
        }
    }
    
    if(count == 0) {
        Serial.println("  No I2C devices found!");
    } else {
        Serial.printf("I2C Scanner: Found %d device(s)\n", count);
    }
    Serial.println();
}

bool HardwareManager::verifyRTC() {
    if(!rtc.begin()) return false;
    DateTime now = rtc.now();
    if(now.year() < 2000 || now.year() > 2099) return false;
    return true;
}

bool HardwareManager::verifyINA226() {
    if(!ina226.begin()) return false;
    delay(10);
    float voltage = ina226.getBusVoltage_mV();
    if(voltage < 0.0f || voltage > 36000.0f) return false;
    return true;
}

bool HardwareManager::verifyOLED() {
    display.begin();
    delay(10);
    // Simple verification by drawing and assuming success if no crash
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.drawStr(0, 10, "ArsLed V601");
    display.sendBuffer();
    return true;
}

void HardwareManager::begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(100);
    
    scanI2CDevices();
    
    Serial.println("HardwareManager: Initializing...");
    
    // OLED Verification
    if(verifyOLED()) { isOLED_OK = true; } 
    
    // RTC Verification
    if(verifyRTC()) { isRTC_OK = true; }
    
    // INA226 Verification
    if(verifyINA226()) { isINA226_OK = true; }
    
    // Temperature Sensors (NTC)
    pinMode(THERM_WATER_PIN, INPUT);
    pinMode(THERM_SINK_PIN, INPUT);
    pinMode(THERM_ROOM_PIN, INPUT);
    
    // Check if NTC pins return a valid reading (simulated quick check)
    if(readThermistor(THERM_WATER_PIN) > -50.0f && readThermistor(THERM_SINK_PIN) > -50.0f) {
        isTEMP_OK = true;
    }

    // PWM Channels (always OK if pins are initialized)
    setupPWMChannels();
    isPWM_OK = true;
    isLED_OK = true;
    isFAN_OK = true;
    
    Serial.println();
    if(isOLED_OK) delay(1000);
}

// =============================================================================
// SERIAL DASHBOARD IMPLEMENTATION (Fasa 1 & 2)
// =============================================================================

void HardwareManager::printBootSummary(unsigned long bootTime) {
    if(bootTime == 0) return; // Prevent printing if not initialized
    
    DateTime now = getRTCTime();
    int readyCount = 0;
    int totalCount = 6;
    
    // Note: isTEMP_OK only counts if at least one NTC reading is plausible
    if(isOLED_OK) readyCount++;
    if(isRTC_OK) readyCount++;
    if(isINA226_OK) readyCount++;
    if(isLED_OK) readyCount++;
    if(isFAN_OK) readyCount++;
    if(isTEMP_OK) readyCount++;
    
    int readyPercent = (readyCount * 100) / totalCount;
    
    Serial.println("\n=============================================");
    Serial.println("=== ArsLed Intelligent Marine Light V601 ===");
    
    // --- System Status ---
    Serial.println("System Status:");
    if(isPCTimeReceived) {
        Serial.printf("PC Time:  %s %s\n", pcTimeStr, pcDateStr);
    }
    
    if(isRTC_OK) {
        Serial.printf("RTC Time: %02d:%02d:%02d %02d/%02d/%04d\n",
                     now.hour(), now.minute(), now.second(),
                     now.day(), now.month(), now.year());
    } else {
        Serial.println("RTC Time: Failed (Using PC or Default Time)");
    }
    
    Serial.println("HardwareManager: Initializing...");
    Serial.printf("OLED: %s\n", isOLED_OK ? "Ready" : "Failed");
    Serial.printf("RTC: %s\n", isRTC_OK ? "Ready" : "Failed");
    Serial.printf("INA226: %s\n", isINA226_OK ? "Ready" : "Failed");
    Serial.printf("LED: %s\n", isLED_OK ? "Ready" : "Failed");
    Serial.printf("FAN: %s\n", isFAN_OK ? "Ready" : "Failed");
    Serial.printf("TEMP: %s\n", isTEMP_OK ? "Ready" : "Failed"); // NTC Summary
    
    Serial.println("---------------------------------------------");
    Serial.printf("Hardware Manager: %d%% Ready (%d/%d)\n", readyPercent, readyCount, totalCount);
    
    if(isRTC_OK && isINA226_OK && isTEMP_OK) { // Critical Check
        Serial.println("System: Ready !");
    } else {
        Serial.println("System: Limited Functionality.");
    }
    Serial.println("=============================================\n");
}

void HardwareManager::printLiveDashboard(uint8_t brightness[]) {
    DateTime now = getRTCTime();
    
    Serial.println("\n=============================================");
    Serial.println("=== ArsLed V601 - Live Monitoring ===");
    Serial.printf("Time: %02d:%02d:%02d %02d/%02d/%04d\n",
                 now.hour(), now.minute(), now.second(),
                 now.day(), now.month(), now.year());
    Serial.println("---------------------------------------------");
    
    // Temperature Readings
    Serial.printf("Water Temp: %.2fC | Room Temp: %.2fC\n", lastWaterTemp, lastRoomTemp);
    Serial.printf("Sink Temp:  %.2fC | Fan Speed: %d%%\n", lastSinkTemp, getCurrentFanSpeed());

    // Power Readings
    Serial.printf("Power: %.2fW | Total kWh: %.3f\n", getCurrentWatts(), totalKwh);
    Serial.printf("Cost (Rp): %.2f\n", getCostRp());
    
    // LED Brightness (Avg)
    float avgBright = 0;
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        avgBright += brightness[i];
    }
    avgBright /= NUM_LED_CHANNELS;
    
    Serial.printf("LED Brightness: RB:%d CW:%d B:%d FS:%d\n",
                  brightness[CH_RB], brightness[CH_CW], brightness[CH_B], brightness[CH_FS]);
    Serial.printf("Average Brightness: %.0f%%\n", avgBright);
    Serial.println("=============================================\n");
}

// =============================================================================
// RUNTIME FUNCTIONS
// =============================================================================

void HardwareManager::updateTemperatures() {
    // Read all NTCs regardless of LED status, but Sink Temp is most critical
    lastWaterTemp = readThermistor(THERM_WATER_PIN);
    lastRoomTemp = readThermistor(THERM_ROOM_PIN);
    lastSinkTemp = readThermistor(THERM_SINK_PIN);
}

void HardwareManager::updatePowerMonitoring() {
    if(!isINA226_OK) return;
    unsigned long currentTime = millis();
    if(currentTime - lastPowerUpdate >= 1000) {
        float watts = getCurrentWatts();
        float hours = (currentTime - lastPowerUpdate) / 3600000.0f;
        totalKwh += (watts / 1000.0f) * hours;
        lastPowerUpdate = currentTime;
    }
}

void HardwareManager::updateFanControl() {
    uint8_t fanSpeed = getCurrentFanSpeed();
    uint16_t duty = (uint16_t)(((float)fanSpeed / 100.0f) * MAX_DUTY_VALUE);
    ledcWrite(NUM_LED_CHANNELS, duty);
}

float HardwareManager::getCurrentWatts() {
    if(!isINA226_OK) return 0.0f;
    float voltage = ina226.getBusVoltage_mV();
    if(voltage < 0.0f) return 0.0f;
    return (voltage / 1000.0f) * ina226.getCurrent_mA() / 1000.0f;
}

float HardwareManager::getCostRp() {
    return totalKwh * DEFAULT_KWH_COST;
}

uint8_t HardwareManager::getCurrentFanSpeed() {
    // Fan Logic: 50% at 50C, max 90%
    if(lastSinkTemp < 50.0f) return 0;
    
    // Calculate fan speed based on linear increase (10% per 10C over 50C)
    int tempDiff = (int)lastSinkTemp - 50;
    int fanIncrease = (tempDiff / 10) * 10;
    int fanSpeed = 50 + fanIncrease;
    
    return (uint8_t)constrain(fanSpeed, 50, 90);
}

DateTime HardwareManager::getRTCTime() {
    if(!isRTC_OK) return DateTime(2025, 1, 1, 0, 0, 0);
    return rtc.now();
}

void HardwareManager::setupPWMChannels() {
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        ledcSetup(i, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
        ledcAttachPin(ledPins[i], i);
        ledcWrite(i, 0);
    }
    ledcSetup(NUM_LED_CHANNELS, PWM_FREQUENCY, PWM_RESOLUTION_BITS); // Fan Channel
    ledcAttachPin(FAN_PIN, NUM_LED_CHANNELS);
    ledcWrite(NUM_LED_CHANNELS, 0);
}

void HardwareManager::setChannelBrightness(uint8_t channel, uint8_t percent) {
    if(channel >= NUM_LED_CHANNELS) return;
    percent = constrain(percent, 0, 100);
    const float MAX_ALLOWED_DUTY = MAX_DUTY_VALUE * ((float)MAX_PERCENT_LIMIT / 100.0f);
    uint16_t duty = (uint16_t)(((float)percent / 100.0f) * MAX_DUTY_VALUE);
    if(duty > (uint16_t)MAX_ALLOWED_DUTY) duty = (uint16_t)MAX_ALLOWED_DUTY;
    ledcWrite(channel, duty);
}

void HardwareManager::adjustRTCTime(int h, int m, int s) {
    snprintf(pcTimeStr, sizeof(pcTimeStr), "%02d:%02d:%02d", h, m, s);
    isPCTimeReceived = true;
    
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    
    if(!isRTC_OK) {
        Serial.println("RTC not connected. Time stored from PC only.");
        digitalWrite(ONBOARD_LED_PIN, LOW);
        return;
    }
    
    DateTime rtcNow = rtc.now();
    DateTime newTime(rtcNow.year(), rtcNow.month(), rtcNow.day(), h, m, s);
    rtc.adjust(newTime);
    Serial.printf("✓ RTC Time adjusted to: %02d:%02d:%02d\n", h, m, s);
    
    digitalWrite(ONBOARD_LED_PIN, LOW);
}

void HardwareManager::adjustRTCDate(int y, int m, int d) {
    int fullYear = (y < 100) ? (y + 2000) : y;
    snprintf(pcDateStr, sizeof(pcDateStr), "%02d/%02d/%04d", d, m, fullYear);
    isPCTimeReceived = true;
    
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    
    if(!isRTC_OK) {
        Serial.println("RTC not connected. Date stored from PC only.");
        digitalWrite(ONBOARD_LED_PIN, LOW);
        return;
    }
    
    DateTime rtcNow = rtc.now();
    DateTime newDate(fullYear, m, d, rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    rtc.adjust(newDate);
    Serial.printf("✓ RTC Date adjusted to: %02d/%02d/%04d\n", d, m, fullYear);
    
    digitalWrite(ONBOARD_LED_PIN, LOW);
}

void HardwareManager::updateOLEDDisplay(uint8_t brightness[]) {
    if(!isOLED_OK) return;
    
    char line[32];
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tr);
    
    // Line 1: Time or PC Time
    if(isRTC_OK) {
        DateTime now = rtc.now();
        snprintf(line, sizeof(line), "%02d:%02d:%02d %02d/%02d/%04d", 
                 now.hour(), now.minute(), now.second(),
                 now.day(), now.month(), now.year());
        display.drawStr(0, 10, line);
    } else if(isPCTimeReceived) {
        snprintf(line, sizeof(line), "%s %s", pcTimeStr, pcDateStr);
        display.drawStr(0, 10, line);
    } else {
        display.drawStr(0, 10, "No Clock");
    }
    
    // Line 2: Temperatures
    snprintf(line, sizeof(line), "W:%.1f S:%.1f R:%.1f", 
             lastWaterTemp, lastSinkTemp, lastRoomTemp);
    display.drawStr(0, 22, line);
    
    // Line 3: LED Brightness
    snprintf(line, sizeof(line), "RB:%d CW:%d B:%d FS:%d", 
             brightness[CH_RB], brightness[CH_CW], 
             brightness[CH_B], brightness[CH_FS]);
    display.drawStr(0, 34, line);
    
    // Line 4: Power
    if(isINA226_OK) {
        snprintf(line, sizeof(line), "%.2fW %.3fkWh", getCurrentWatts(), totalKwh);
    } else {
        snprintf(line, sizeof(line), "Power: N/A");
    }
    display.drawStr(0, 46, line);
    
    // Line 5: Fan
    snprintf(line, sizeof(line), "Fan:%d%%", getCurrentFanSpeed());
    display.drawStr(0, 58, line);
    
    display.sendBuffer();
}

void HardwareManager::run(uint8_t brightness[]) {
    updateTemperatures();
    updatePowerMonitoring();
    updateFanControl();
    updateOLEDDisplay(brightness);
}
EOT_HW_CPP
echo "   -> src/HardwareManager.cpp - OK"

# ------------------------------------------------------------------------------
# 8. src/main.cpp (V601 Logic - Dual-Phase Monitoring)
# ------------------------------------------------------------------------------
echo "8/9: Membuat src/main.cpp..."
cat <<'EOT_MAIN' > src/main.cpp
#include <Arduino.h>
#include "HardwareManager.h"
#include "Scheduler.h"
#include "config.h"

HardwareManager hw;
Scheduler scheduler;

unsigned long bootTimeStart = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastSummaryPrintTime = 0;
unsigned long lastLiveDashboardPrintTime = 0;

void handleSerialCommands() {
    if(Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if(command.startsWith("SETTIME=")) {
            String timeStr = command.substring(8);
            int h, m, s;
            if(sscanf(timeStr.c_str(), "%d:%d:%d", &h, &m, &s) == 3) {
                hw.adjustRTCTime(h, m, s);
            } else {
                Serial.println("Format: SETTIME=HH:MM:SS");
            }
        } else if(command.startsWith("SETDATE=")) {
            String dateStr = command.substring(8);
            int y, m, d;
            if(sscanf(dateStr.c_str(), "%d:%d:%d", &y, &m, &d) == 3) {
                hw.adjustRTCDate(y, m, d);
            } else {
                Serial.println("Format: SETDATE=YY:MM:DD");
            }
        } else if(command == "HELP") {
            Serial.println("\n--- COMMANDS ---");
            Serial.println("SETTIME=HH:MM:SS  - Set time");
            Serial.println("SETDATE=YY:MM:DD  - Set date");
            Serial.println("HELP              - Show this help");
            Serial.println("----------------\n");
        } else if(command.length() > 0) {
            Serial.printf("Unknown: %s (type HELP)\n", command.c_str());
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LOW);
    
    // Boot sequence - 5 blinks (500ms duration)
    for(int i = 0; i < 5; i++) {
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(500); // 500ms
        digitalWrite(ONBOARD_LED_PIN, LOW);
        delay(500); // 500ms
    }
    
    hw.begin();
    scheduler.begin();
    bootTimeStart = millis(); // Start timing for dual-phase monitoring
    
    // Wait for serial commands (10 second window)
    Serial.println("\n[Waiting 10s for serial commands...]");
    unsigned long startWait = millis();
    while(millis() - startWait < 10000) {
        handleSerialCommands();
        delay(10);
    }
    
    // Print initial summary once
    hw.printBootSummary(bootTimeStart); 
    Serial.println("Type HELP for available commands\n");
}

void loop() {
    yield();
    handleSerialCommands();
    
    unsigned long currentTime = millis();
    
    // Heartbeat - 2 blinks every minute (Non-blocking)
    if(currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
        // Quick 2-blink sequence
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(50); 
        digitalWrite(ONBOARD_LED_PIN, LOW);
        delay(50);
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(50);
        digitalWrite(ONBOARD_LED_PIN, LOW);
        lastHeartbeatTime = currentTime;
    }
    
    // =========================================================================
    // DUAL-PHASE SERIAL MONITORING LOGIC
    // =========================================================================
    
    uint8_t currentBrightness[NUM_LED_CHANNELS];
    
    // Phase 1: Boot Summary (First 60 seconds)
    if(currentTime - bootTimeStart < BOOT_SUMMARY_DURATION) {
        // Print the aesthetic summary every 5 seconds (5000ms) for Fasa 1
        if(currentTime - lastSummaryPrintTime >= 5000) {
            hw.printBootSummary(bootTimeStart);
            lastSummaryPrintTime = currentTime;
        }
    } 
    // Phase 2: Live Dashboard (After 60 seconds)
    else {
        // Print Live Dashboard every 1 minute (60000ms) for Fasa 2
        if(currentTime - lastLiveDashboardPrintTime >= LIVE_DASHBOARD_INTERVAL) {
            // Re-run scheduler and hw.run to update data before printing
            DateTime now = hw.getRTCTime();
            scheduler.run(now.hour(), now.minute());
            
            for(int i = 0; i < NUM_LED_CHANNELS; i++) {
                currentBrightness[i] = scheduler.getBrightness(i);
                hw.setChannelBrightness(i, currentBrightness[i]);
            }
            
            hw.run(currentBrightness); // Updates temps, power, fan
            hw.printLiveDashboard(currentBrightness);
            lastLiveDashboardPrintTime = currentTime;
        }
    }
    // =========================================================================
    
    // Standard Runtime (PWM/OLED/Power/Temp/Fan control)
    if(currentTime - bootTimeStart >= BOOT_SUMMARY_DURATION) {
        // This block runs outside the 60s print interval to keep PWM/etc running continuously
        DateTime now = hw.getRTCTime();
        scheduler.run(now.hour(), now.minute());
        
        for(int i = 0; i < NUM_LED_CHANNELS; i++) {
            currentBrightness[i] = scheduler.getBrightness(i);
            hw.setChannelBrightness(i, currentBrightness[i]);
        }
        
        hw.run(currentBrightness); // Run Hw Manager (Temp/Power/Fan/OLED)
    } else {
        // Hw Manager still needs to run during the 60s summary period to get initial data
        DateTime now = hw.getRTCTime();
        scheduler.run(now.hour(), now.minute());
        
        for(int i = 0; i < NUM_LED_CHANNELS; i++) {
            currentBrightness[i] = scheduler.getBrightness(i);
            hw.setChannelBrightness(i, currentBrightness[i]);
        }
        
        hw.run(currentBrightness); // Run Hw Manager (Temp/Power/Fan/OLED)
    }
    
    // Small delay to prevent watchdog timer and excessive serial print load
    delay(100); 
}
EOT_MAIN
echo "   -> src/main.cpp - OK"

# ------------------------------------------------------------------------------
# 9. README.md
# ------------------------------------------------------------------------------
echo "9/9: Membuat README.md..."
cat <<'EOT_README' > README.md
# ArsLed Marine Light V601 - Final Standalone

## Base Version
**V59 Stable Logic** + **NTC Steinhart-Hart Integration** + **Dual-Phase Monitoring**

## Fit