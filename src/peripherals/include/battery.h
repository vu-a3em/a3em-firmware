#ifndef __BATTERY_HEADER_H__
#define __BATTERY_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

// Number of conversions taken per measurement
#define BATTERY_NUM_SAMPLES              6
#define BATTERY_CONVERSION_MAX_WAITS     32

// Number of consecutive low readings that have to repeat before the device acts on it, and the voltage
// recovery margin before a low-battery condition is considered cleared
#define BATTERY_LOW_CONSECUTIVE_READINGS 3
#define BATTERY_LOW_HYSTERESIS_MV        100

typedef struct { uint32_t millivolts; float celcius; bool valid; } battery_result_t;


// Public API Functions ------------------------------------------------------------------------------------------------

void battery_monitor_init(void);
void battery_monitor_deinit(void);
battery_result_t battery_monitor_get_details(void);
bool battery_monitor_is_critically_low(uint32_t threshold_millivolts);
void battery_monitor_reset_low_state(void);

#endif  // #ifndef __BATTERY_HEADER_H__
