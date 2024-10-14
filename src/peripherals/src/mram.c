// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "mram.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

typedef struct {
   uint32_t is_activated;
   uint32_t last_known_timestamp;
   uint32_t deployment_start_time;
   uint32_t reserved;
} persistent_data_t;

static persistent_data_t persistent_data;


// Public API Functions ------------------------------------------------------------------------------------------------

void mram_init(void)
{
   // Initialize persistent storage data structures
   memcpy(&persistent_data, (void*)MRAM_PERSISTENT_STORAGE_ADDRESS, sizeof(persistent_data));
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
   AM_CRITICAL_BEGIN
   result = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, (uint32_t*)&persistent_data, (uint32_t*)MRAM_PERSISTENT_STORAGE_ADDRESS, sizeof(persistent_data) / sizeof(uint32_t));
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
   AM_CRITICAL_BEGIN
   result = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, (uint32_t*)&persistent_data, (uint32_t*)MRAM_PERSISTENT_STORAGE_ADDRESS, sizeof(persistent_data) / sizeof(uint32_t));
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
