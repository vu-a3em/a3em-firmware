#ifndef __MRAM_HEADER_H__
#define __MRAM_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

// Why the device last stopped recording. Persisted across the power cycle so that a
// returned card answers "why did it stop", which is the first question asked of any
// device that came back early. Values are stable: the dashboard stores them.
typedef enum {
   DEACTIVATION_UNKNOWN = 0,
   DEACTIVATION_MAGNET,
   DEACTIVATION_BATTERY_LOW,
   DEACTIVATION_PHASE_ENDED,
   DEACTIVATION_DEPLOYMENT_ENDED,
   DEACTIVATION_RTC_STOPPED,
   DEACTIVATION_STORAGE_ERROR
} deactivation_reason_t;


// Public API Functions ------------------------------------------------------------------------------------------------

void mram_init(void);
bool mram_set_deactivation_reason(deactivation_reason_t reason);
deactivation_reason_t mram_get_deactivation_reason(void);
const char* mram_deactivation_reason_name(deactivation_reason_t reason);
void mram_deinit(void);
bool mram_set_activated(bool activated, uint32_t deployment_time);
bool mram_is_activated(void);
bool mram_set_last_known_timestamp(uint32_t timestamp);
uint32_t mram_get_last_known_timestamp(void);
uint32_t mram_get_deployment_start_time(void);
bool mram_store_audadc_dc_offset(int32_t dc_offset);
int32_t mram_get_audadc_dc_offset(void);

#endif  // #ifndef __MRAM_HEADER_H__
