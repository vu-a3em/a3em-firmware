#ifndef __SYSTEM_HEADER_H__
#define __SYSTEM_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "mram.h"
#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef struct
{
   am_hal_reset_status_t hardware_status;
   uint32_t software_reason;
   uint32_t fault_address;
   uint32_t boot_epoch;         // Power-on count, from MRAM
   uint32_t resets_this_epoch;  // Resets since the last power-on, from a scratch register
   uint32_t scratch0_raw;       // Scratch registers exactly as the previous run left them, so that a
   uint32_t scratch1_raw;       //   disagreement with the decoded reason can be seen in the log
   bool was_power_on;
} system_boot_info_t;

typedef enum {
   SELF_TEST_PASS = 0,
   SELF_TEST_FAIL_MICROPHONE = 1,
   SELF_TEST_FAIL_STORAGE = 2,
   SELF_TEST_FAIL_IMU = 3,
   SELF_TEST_FAIL_POWER_OR_CLOCK = 4
} self_test_result_t;


// Public API Functions ------------------------------------------------------------------------------------------------

// Shared system-level functionality
void setup_hardware(void);
void system_initialize_peripherals(void);
void system_deinitialize_peripherals(void);
void system_enable_interrupts(bool enabled);
void system_enter_deep_sleep_mode(void);
void system_enter_power_off_mode(uint32_t wake_on_magnet, uint32_t wake_on_timestamp, bool reinit_on_wakeup);
void system_set_sram_active(bool active_during_deep_sleep);
void system_release_sram_retention(void);
void system_accumulate_cache_stats(void);
bool system_get_cache_stats(uint64_t *accesses, uint64_t *hits, float *hit_rate_percent);
void system_read_ID(uint8_t *id, uint32_t id_length);
void system_delay(uint32_t delay_us);

// Reset control functionality
void system_reset(void);
void system_reset_with_reason(uint32_t reason) __attribute__((noreturn));

// Boot diagnostics functionality
self_test_result_t system_run_self_test(void);
const system_boot_info_t* system_get_boot_info(void);
const char* reset_reason_name(uint32_t reason);
uint32_t system_get_hal_failure_count(void);
const char* system_get_first_hal_failure(uint32_t *line, uint32_t *status);

// Watchdog functionality
void system_enable_watchdog(void);
void system_disable_watchdog(void);
void system_feed_watchdog(void);

#endif  // #ifndef __SYSTEM_HEADER_H__
