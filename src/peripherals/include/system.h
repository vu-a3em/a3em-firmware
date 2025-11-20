#ifndef __SYSTEM_HEADER_H__
#define __SYSTEM_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void setup_hardware(void);
void system_reset(void);
void system_initialize_peripherals(void);
void system_deinitialize_peripherals(void);
void system_enable_interrupts(bool enabled);
void system_enter_deep_sleep_mode(void);
void system_enter_power_off_mode(uint32_t wake_on_magnet, uint32_t wake_on_timestamp, bool reinit_on_wakeup);
void system_read_ID(uint8_t *id, uint32_t id_length);
void system_delay(uint32_t delay_us);
void system_enable_watchdog(void);
void system_feed_watchdog(void);

#endif  // #ifndef __SYSTEM_HEADER_H__
