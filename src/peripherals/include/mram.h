#ifndef __MRAM_HEADER_H__
#define __MRAM_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


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

#endif  // #ifndef __MRAM_HEADER_H__
