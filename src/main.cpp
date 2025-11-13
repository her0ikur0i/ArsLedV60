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
