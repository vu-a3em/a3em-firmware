#ifndef __MRAM_HEADER_H__
#define __MRAM_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef struct
{
   uint32_t boot_epoch;          // Incremented once per power-on, never per reset
   uint32_t hardware_status;     // Raw RSTGEN status bits from am_hal_reset_status_get()
   uint32_t software_reason;     // One of the RESET_REASON_* codes
   uint32_t fault_address;       // Stacked PC at the time of a hard fault, otherwise 0
} mram_boot_record_t;


// Public API Functions ------------------------------------------------------------------------------------------------

void mram_init(void);
void mram_deinit(void);
bool mram_set_activated(bool activated, uint32_t deployment_time);
bool mram_is_activated(void);
bool mram_set_last_known_timestamp(uint32_t timestamp);
uint32_t mram_get_last_known_timestamp(void);
uint32_t mram_get_deployment_start_time(void);
bool mram_store_audadc_dc_offset(int32_t dc_offset);
int32_t mram_get_audadc_dc_offset(void);
bool mram_increment_boot_epoch(void);
uint32_t mram_get_boot_epoch(void);
void mram_get_boot_record(mram_boot_record_t *record);
bool mram_store_fault(uint32_t software_reason, uint32_t hardware_status, uint32_t fault_address);

#endif  // #ifndef __MRAM_HEADER_H__
