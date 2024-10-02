#ifndef __BATTERY_HEADER_H__
#define __BATTERY_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef enum { BATTERY_EMPTY = 3500, BATTERY_CRITICAL = 3680, BATTERY_LOW = 3750 } battery_status_t;
typedef struct { uint32_t millivolts; float celcius; } battery_result_t;


// Public API Functions ------------------------------------------------------------------------------------------------

void battery_monitor_init(void);
void battery_monitor_deinit(void);
battery_result_t battery_monitor_get_details(void);

#endif  // #ifndef __BATTERY_HEADER_H__
