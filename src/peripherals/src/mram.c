// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "mram.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

extern uint8_t _persistent_base_address[];

typedef struct __attribute__((packed, aligned(16))) {
   union {
      uint32_t is_activated_addr[4];
      struct {
         uint32_t is_activated;
         uint32_t deployment_start_time;
      };
   };
   union {
      uint32_t audadc_dc_offset_addr[4];
      int32_t audadc_dc_offset;
   };
   union {
      uint32_t last_known_timestamp_addr[4];
      uint32_t last_known_timestamp;
   };
} persistent_data_t;

static persistent_data_t persistent_data;


// Public API Functions ------------------------------------------------------------------------------------------------

void mram_init(void)
{
   // Initialize the persistent storage data structure
   memcpy(&persistent_data, _persistent_base_address, sizeof(persistent_data_t));
}

void mram_deinit(void)
{
   // No-op function for this peripheral
}

bool mram_set_activated(bool activated, uint32_t deployment_time)
{
   // Store the activation flag to persistent memory
   int result = -1;
   persistent_data.is_activated = activated ? 10 : 0;
   persistent_data.deployment_start_time = deployment_time;
   uint8_t *src = (uint8_t*)persistent_data.is_activated_addr;
   uint8_t *dst = (uint8_t*)_persistent_base_address + offsetof(persistent_data_t, is_activated_addr);
   AM_CRITICAL_BEGIN
   result = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, (uint32_t*)src, (uint32_t*)dst, sizeof(persistent_data.is_activated_addr) / sizeof(uint32_t));
   AM_CRITICAL_END
   return (result == 0);
}

bool mram_is_activated(void)
{
   // Return whether the activation flag is as expected
   return persistent_data.is_activated == 10;
}

bool mram_set_last_known_timestamp(uint32_t timestamp)
{
   // Store the current timestamp to persistent memory
   int result = -1;
   persistent_data.last_known_timestamp = timestamp;
   uint8_t *src = (uint8_t*)persistent_data.last_known_timestamp_addr;
   uint8_t *dst = (uint8_t*)_persistent_base_address + offsetof(persistent_data_t, last_known_timestamp_addr);
   AM_CRITICAL_BEGIN
   result = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, (uint32_t*)src, (uint32_t*)dst, sizeof(persistent_data.last_known_timestamp_addr) / sizeof(uint32_t));
   AM_CRITICAL_END
   return (result == 0);
}

uint32_t mram_get_last_known_timestamp(void)
{
   // Return the last timestamp stored in persistent memory
   return persistent_data.last_known_timestamp;
}

uint32_t mram_get_deployment_start_time(void)
{
   // Retrieve the last known deployment start time
   return persistent_data.deployment_start_time;
}

bool mram_store_audadc_dc_offset(int32_t dc_offset)
{
   // Store the AUDADC DC offset value to persistent memory
   int result = -1;
   persistent_data.audadc_dc_offset = dc_offset;
   uint8_t *src = (uint8_t*)persistent_data.audadc_dc_offset_addr;
   uint8_t *dst = (uint8_t*)_persistent_base_address + offsetof(persistent_data_t, audadc_dc_offset_addr);
   AM_CRITICAL_BEGIN
   result = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, (uint32_t*)src, (uint32_t*)dst, sizeof(persistent_data.audadc_dc_offset_addr) / sizeof(uint32_t));
   AM_CRITICAL_END
   return (result == 0);
}

int32_t mram_get_audadc_dc_offset(void)
{
   // Return the AUDADC DC offset value
   return persistent_data.audadc_dc_offset;
}
