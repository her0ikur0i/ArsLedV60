#!/bin/bash
# ==============================================================================
# ARSLED MARINE LIGHT V60 - STANDALONE GENERATOR (SH1106 OLED)
#
# FITUR:
# - Tidak membuat folder baru. File dibuat di direktori saat ini.
# - Base: V54 Logic (I2C Ping Fix)
# - FIX: Menggunakan Adafruit SH110X & GFX Library (untuk SH1106 OLED).
# - FIX: Menggunakan deklarasi objek SH110X yang benar.
#
# PENGGUNAAN:
# - Buat folder project, cd ke dalamnya, lalu jalankan script ini.
# - Contoh: bash v60_standalone.sh
# ==============================================================================

set -euo pipefail

echo "=== ArsLed Marine Light V60 Standalone Generator (SH1106) ==="

# --- 1. platformio.ini (SH110X Libraries) ---
echo "1/9: Membuat platformio.ini..."
cat <<'EOT_INI' > platformio.ini
[env:lolin_s2_mini]
platform = espressif32
board = lolin_s2_mini
framework = arduino
monitor_speed = 115200

upload_protocol = esptool
upload_speed = 460800
upload_port = COM5 ; <<<<<<<< HARAP GANTI DENGAN PORT SERIAL ANDA YANG BENAR

board_build.f_cpu = 240000000L
board_build.arduino.loop_stack_size = 32768

lib_deps =
    adafruit/RTClib@^2.1.3
    robtillaart/INA226@^0.2.0
    adafruit/Adafruit SH110X@^2.1.10
    adafruit/Adafruit GFX Library@^1.11.11
    adafruit/Adafruit BusIO@^1.14.5
    adafruit/Adafruit AHTX0@^2.0.2
    adafruit/Adafruit MAX31856 library@^1.2.0
    adafruit/Adafruit AS726x@^1.0.1
    adafruit/Adafruit MCP4728@^1.0.1
    adafruit/Adafruit NeoPixel@^1.12.0
EOT_INI
echo "   -> platformio.ini - OK"

# --- 2. src/config.h (Pin & Setup) ---
mkdir -p src
echo "2/9: Membuat src/config.h..."
cat <<'EOT_CONFIG' > src/config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- HARDWARE PIN DEFINITIONS ---
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ONBOARD_LED_PIN 33 // S2 Mini
#define FAN_PWM_PIN 1

// --- LED CHANNEL DEFINITIONS ---
// Jumlah saluran LED yang digunakan (default 3 untuk White, Blue, UV)
#define NUM_LED_CHANNELS 3
#define PWM_FREQUENCY 5000 // 5 kHz
#define PWM_RESOLUTION 8   // 8-bit (0-255)

// Channel ke ESP32 LEDC
#define LEDC_CHANNEL_WHITE 0
#define LEDC_CHANNEL_BLUE  1
#define LEDC_CHANNEL_UV    2

// PIN Output LEDC
#define LED_PIN_WHITE 3
#define LED_PIN_BLUE  4
#define LED_PIN_UV    5

// --- I2C ADDRESSES ---
#define OLED_ADDR   0x3C
#define RTC_ADDR    0x68
#define INA_ADDR    0x40
#define AHT_ADDR    0x38

// --- SCHEDULER DEFINITIONS (in minutes) ---
// Contoh: 08:00 (8*60 = 480)
#define SCHED_ON_HOUR   8
#define SCHED_ON_MINUTE 0

// Contoh: 10:00 (10*60 = 600)
#define SCHED_PEAK_START_HOUR 10
#define SCHED_PEAK_START_MINUTE 0

// Contoh: 16:00 (16*60 = 960)
#define SCHED_PEAK_END_HOUR 16
#define SCHED_PEAK_END_MINUTE 0

// Contoh: 20:00 (20*60 = 1200)
#define SCHED_OFF_HOUR  20
#define SCHED_OFF_MINUTE 0

// Kecerahan Puncak per Saluran (0-100%)
// Urutan: White, Blue, UV
#define PEAK_BRIGHTNESS_WHITE 100
#define PEAK_BRIGHTNESS_BLUE  100
#define PEAK_BRIGHTNESS_UV    100

// --- SYSTEM DEFINITIONS ---
#define HEARTBEAT_INTERVAL 60000 // 1 menit (dalam ms)
#define OLED_UPDATE_INTERVAL 5000 // 5 detik
#define RTC_SYNC_WINDOW 10000 // Jendela 10 detik setelah boot untuk sync waktu

#endif // CONFIG_H
EOT_CONFIG
echo "   -> src/config.h - OK"

# --- 3. src/HardwareManager.h ---
echo "3/9: Membuat src/HardwareManager.h..."
cat <<'EOT_HW_H' > src/HardwareManager.h
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
EOT_HW_H
echo "   -> src/HardwareManager.h - OK"

# --- 4. src/HardwareManager.cpp ---
echo "4/9: Membuat src/HardwareManager.cpp..."
cat <<'EOT_HW_CPP' > src/HardwareManager.cpp
#include "HardwareManager.h"

#include <esp32-hal-ledc.h>
#include <Adafruit_SH110X.h> // FIX V60: Ganti SSD1306/U8g2

// --- I2C/Driver Objects ---
// FIX V60: Ganti deklarasi SSD1306 ke SH1106G
// I2C OLED (128x64)
Adafruit_SH1106G display(128, 64, &Wire, -1); 

// RTC
RTC_DS3231 rtc; 

// --- Constructor ---
HardwareManager::HardwareManager() 
    : isRTCReady(false), isOLEDReady(false), isINA226Ready(false), isAHT20Ready(false) {}

// --- I2C Ping (Check for Device Existence) ---
bool HardwareManager::isI2CPing(uint8_t address) {
    // FIX V54: Menggunakan Wire.beginTransmission untuk verifikasi fisik
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

// --- Initialization ---
void HardwareManager::begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // 1. OLED (SH1106)
    isOLEDReady = isI2CPing(OLED_ADDR);
    if (isOLEDReady) {
        initOLED();
        display.print("SH1106 Ready!");
        display.display();
    }
    
    // 2. RTC
    isRTCReady = isI2CPing(RTC_ADDR) && rtc.begin();
    
    // 3. INA226 (Placeholder, cek ping saja)
    isINA226Ready = isI2CPing(INA_ADDR);
    
    // 4. AHT20 (Placeholder, cek ping saja)
    isAHT20Ready = isI2CPing(AHT_ADDR);
    
    initPWM();
    initFanPWM();
}

void HardwareManager::initOLED() {
    // FIX V60: Ubah begin() untuk SH110X. OLED_RESET = -1 (tidak ada pin reset)
    if (display.begin(OLED_ADDR, true)) { // begin(address, reset=true)
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE); // FIX V60: Ganti SSD1306_WHITE
        display.setCursor(0, 0);
    } else {
        isOLEDReady = false;
        Serial.println("OLED Init FAILED!");
    }
}

void HardwareManager::initPWM() {
    // PWM Channels (White, Blue, UV)
    ledcSetup(LEDC_CHANNEL_WHITE, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(LED_PIN_WHITE, LEDC_CHANNEL_WHITE);

    ledcSetup(LEDC_CHANNEL_BLUE, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(LED_PIN_BLUE, LEDC_CHANNEL_BLUE);

    ledcSetup(LEDC_CHANNEL_UV, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(LED_PIN_UV, LEDC_CHANNEL_UV);
}

void HardwareManager::initFanPWM() {
    // Fan PWM setup (placeholder - bisa disempurnakan)
    ledcSetup(3, 25000, 8); // Channel 3, 25kHz, 8-bit
    ledcAttachPin(FAN_PWM_PIN, 3);
    ledcWrite(3, 0); // Fan OFF
}

// --- Runtime Functions ---
void HardwareManager::run(const uint8_t currentBrightness[], bool isLEDsOn) {
    static unsigned long lastOLEDUpdate = 0;

    if (millis() - lastOLEDUpdate >= OLED_UPDATE_INTERVAL) {
        updateOLEDDisplay(currentBrightness, isLEDsOn);
        lastOLEDUpdate = millis();
    }
    
    // Fan Logic (Contoh: ON jika total brightness > 50%)
    uint16_t totalBrightness = 0;
    for (int i = 0; i < NUM_LED_CHANNELS; i++) {
        totalBrightness += currentBrightness[i];
    }

    // Set Fan PWM (Placeholder: 50% speed jika LED ON)
    if (isLEDsOn) {
        ledcWrite(3, 128); // 50% speed
    } else {
        ledcWrite(3, 0);   // OFF
    }
}

void HardwareManager::setChannelBrightness(uint8_t channel, uint8_t brightness) {
    // Brightness 0-100, convert to 0-255 (8-bit)
    uint8_t duty = map(brightness, 0, 100, 0, (1 << PWM_RESOLUTION) - 1); 

    if (channel == 0) ledcWrite(LEDC_CHANNEL_WHITE, duty);
    else if (channel == 1) ledcWrite(LEDC_CHANNEL_BLUE, duty);
    else if (channel == 2) ledcWrite(LEDC_CHANNEL_UV, duty);
}

DateTime HardwareManager::getRTCTime() {
    if (isRTCReady) {
        return rtc.now();
    }
    // Jika RTC Gagal, kembalikan waktu default (1 Jan 2000)
    return DateTime(2000, 1, 1, 0, 0, 0); 
}

void HardwareManager::setRTCTime(const DateTime& dt) {
    if (isRTCReady) {
        rtc.adjust(dt);
        Serial.printf("RTC Adjusted to: %02d:%02d:%02d %02d/%02d/%02d\n", 
            dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(), dt.year() % 100);
    }
}

void HardwareManager::updateOLEDDisplay(const uint8_t currentBrightness[], bool isLEDsOn) {
    if (!isOLEDReady) return;

    DateTime now = getRTCTime();

    display.clearDisplay();
    display.setCursor(0, 0);
    
    // Baris 1: Waktu
    display.setTextSize(2);
    display.printf("%02d:%02d", now.hour(), now.minute());
    
    // Baris 2: Tanggal & Status
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.printf("%02d/%02d/%02d", now.day(), now.month(), now.year() % 100);
    
    // Status RTC
    display.setCursor(70, 20);
    display.print(isRTCReady ? "RTC: OK" : "RTC: FAIL");

    // Baris 3: Status Brightness
    display.setCursor(0, 30);
    display.printf("W: %d%% B: %d%% U: %d%%", 
        currentBrightness[0], currentBrightness[1], currentBrightness[2]);
    
    // Status FAN dan INA/AHT
    display.setCursor(0, 40);
    display.printf("FAN: %s", isLEDsOn ? "ON" : "OFF");
    display.setCursor(70, 40);
    display.print(isINA226Ready ? "INA: OK" : "INA: FAIL");

    display.setCursor(0, 50);
    display.print(isAHT20Ready ? "AHT: OK" : "AHT: FAIL");

    display.display();
}

void HardwareManager::printBootSummary() {
    Serial.println("\n--- HARDWARE SUMMARY ---");
    Serial.printf("OLED (SH1106): %s\n", isOLEDReady ? "Ready" : "Failed");
    Serial.printf("RTC (DS3231): %s\n", isRTCReady ? "Ready" : "Failed");
    Serial.printf("Current Sensor (INA226): %s\n", isINA226Ready ? "Ready" : "Failed");
    Serial.printf("Temp/Humid (AHT20): %s\n", isAHT20Ready ? "Ready" : "Failed");
    Serial.println("------------------------");
}
EOT_HW_CPP
echo "   -> src/HardwareManager.cpp - OK"

# --- 5. src/Scheduler.h ---
echo "5/9: Membuat src/Scheduler.h..."
cat <<'EOT_SCHED_H' > src/Scheduler.h
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
EOT_SCHED_H
echo "   -> src/Scheduler.h - OK"

# --- 6. src/Scheduler.cpp ---
echo "6/9: Membuat src/Scheduler.cpp..."
cat <<'EOT_SCHED_CPP' > src/Scheduler.cpp
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
EOT_SCHED_CPP
echo "   -> src/Scheduler.cpp - OK"

# --- 7. src/main.cpp ---
echo "7/9: Membuat src/main.cpp..."
cat <<'EOT_MAIN' > src/main.cpp
#include <Arduino.h>
#include <RTClib.h>
#include "HardwareManager.h"
#include "Scheduler.h"
#include "config.h"

// --- Global Objects ---
HardwareManager hw;
Scheduler scheduler;
unsigned long lastHeartbeatTime = 0;

// --- Helper: Parsing Time Command ---
void setTimeFromSerial(const char* timeStr) {
    // Format yang diharapkan: SETTIME <HH> <MM> <SS> <DD> <MM> <YY>
    uint8_t H, M, S, D, Mo, Y;
    
    if (sscanf(timeStr, "%hhu %hhu %hhu %hhu %hhu %hhu", 
        &H, &M, &S, &D, &Mo, &Y) == 6) {
        
        DateTime newTime(Y + 2000, Mo, D, H, M, S);
        hw.setRTCTime(newTime);
    } else {
        Serial.println("ERROR: Format SETTIME salah. Gunakan: SETTIME HH MM SS DD MM YY");
    }
}

// --- Serial Command Handler ---
void handleSerialCommands() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase();

        if (input.startsWith("SETTIME")) {
            // SETTIME 10 30 00 25 12 24 (Jam 10:30:00, 25 Des 2024)
            setTimeFromSerial(input.substring(7).c_str());
        } else if (input.startsWith("HELP")) {
            Serial.println("--- COMMANDS ---");
            Serial.println("SETTIME HH MM SS DD MM YY: Set RTC Time & Date.");
            Serial.println("HELP: Tampilkan daftar perintah.");
        }
    }
}

// --- Arduino Core ---
void setup() {
    Serial.begin(115200);
    delay(3000); // Tunggu serial siap
    
    // Onboard LED as status indicator
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, LOW);
    
    // Boot Blink 5x
    for(int i = 0; i < 5; i++) {
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(200);
        digitalWrite(ONBOARD_LED_PIN, LOW);
        delay(200);
    }
    
    hw.begin();
    scheduler.begin();
    hw.printBootSummary();
}

void loop() {
    yield();
    handleSerialCommands();
    
    // Heartbeat Logic (2x Blink setiap 1 menit)
    if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(200); 
        digitalWrite(ONBOARD_LED_PIN, LOW);
        delay(200); 
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        delay(200); 
        digitalWrite(ONBOARD_LED_PIN, LOW);
        lastHeartbeatTime = millis();
    }
    
    DateTime now = hw.getRTCTime();
    scheduler.run(now.hour(), now.minute());
    
    uint8_t currentBrightness[NUM_LED_CHANNELS];
    bool isLEDsOn = false;
    
    for(int i = 0; i < NUM_LED_CHANNELS; i++) {
        uint8_t brightness = scheduler.getBrightness(i);
        hw.setChannelBrightness(i, brightness);
        currentBrightness[i] = brightness;
        
        if(brightness > 0) isLEDsOn = true;
    }
    
    hw.run(currentBrightness, isLEDsOn);
    delay(100);
}
EOT_MAIN
echo "   -> src/main.cpp - OK"

# --- 8. sync_time.py ---
echo "8/9: Membuat sync_time.py..."
cat <<'EOT_PYTHON' > sync_time.py
# Script Python untuk menyinkronkan waktu dari PC ke ESP32 melalui Serial
# Penggunaan: python sync_time.py <PORT> <HH> <MM> <SS> <DD> <MM> <YY>
# Contoh: python sync_time.py COM5 08 41 46 12 11 25

import sys
import serial
import time
import datetime

# --- Parameter (default to current time) ---
if len(sys.argv) < 2:
    print("Error: Argumen PORT tidak ditemukan.")
    print("Penggunaan: python sync_time.py <PORT> <HH> <MM> <SS> <DD> <MM> <YY>")
    sys.exit(1)

port = sys.argv[1]
try:
    if len(sys.argv) == 8:
        # Gunakan waktu dari argumen
        H, M, S = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
        D, Mo, Y = int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7])
        dt = datetime.datetime(2000 + Y, Mo, D, H, M, S)
    else:
        # Gunakan waktu PC saat ini (dengan Y sebagai 2 digit)
        now = datetime.datetime.now()
        H, M, S = now.hour, now.minute, now.second
        D, Mo, Y = now.day, now.month, now.year % 100
        dt = now

    time_command = f"SETTIME {H} {M} {S} {D} {Mo} {Y}"
    print(f"-> Perintah: {time_command}")
    print(f"-> Waktu yang disetel: {dt.strftime('%H:%M:%S %d/%m/%Y')}")
    
except ValueError as e:
    print(f"Error pada format waktu: {e}")
    sys.exit(1)

# --- Serial Communication ---
try:
    print(f"Mencoba menyambung ke port {port}...")
    ser = serial.Serial(port, 115200, timeout=5)
    time.sleep(2) # Tunggu agar port serial terbuka
    
    # Kirim perintah
    ser.write(f"{time_command}\n".encode('utf-8'))
    print("Perintah SETTIME terkirim. Menunggu respon...")
    time.sleep(1) 
    
    # Baca respon (opsional)
    response = ser.read_until(b'\n').decode('utf-8').strip()
    if "Adjusted" in response or "Adjusted" in ser.read_until(b'\n').decode('utf-8').strip():
        print(f"✅ Sinkronisasi SUKSES. Periksa Serial Monitor untuk konfirmasi RTC.")
    else:
        print("⚠️ Sinkronisasi mungkin GAGAL. Cek apakah perangkat sudah booting penuh.")
        
    ser.close()

except serial.SerialException as e:
    print(f"ERROR: Tidak dapat menyambung ke port {port}. Cek kabel atau port COM yang benar.")
    print(f"Detail: {e}")
    sys.exit(1)

EOT_PYTHON
echo "   -> sync_time.py - OK"


# --- 9. README.md ---
echo "9/9: Membuat README.md..."
cat <<'EOT_README' > README.md
# ArsLed Marine Light V60 (Standalone - SH1106 OLED)

Project ini dibuat dengan generator V60 yang menggunakan driver **Adafruit SH110X** untuk OLED.

## Struktur Proyek