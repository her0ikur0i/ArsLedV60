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
