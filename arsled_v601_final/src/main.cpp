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
