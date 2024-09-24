#ifndef __BATTERY_HEADER_H__
#define __BATTERY_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef enum { BATTERY_EMPTY = 3500, BATTERY_CRITICAL = 3680, BATTERY_NOMINAL = 3750, BATTERY_FULL = 4200 } battery_status_t;


// Public API Functions ------------------------------------------------------------------------------------------------

void battery_monitor_init(void);
void battery_monitor_deinit(void);
uint32_t battery_monitor_get_level_mV(void);

#endif  // #ifndef __BATTERY_HEADER_H__
