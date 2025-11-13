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
