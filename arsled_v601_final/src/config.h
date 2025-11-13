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
