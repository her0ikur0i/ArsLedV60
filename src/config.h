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
