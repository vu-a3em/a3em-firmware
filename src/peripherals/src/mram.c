// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "mram.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define MRAM_MAX_PROGRAM_ATTEMPTS      4
#define MRAM_TIMESTAMP_WRITE_INTERVAL  3600
#define MRAM_DC_OFFSET_VALID_TAG       0xA3E30FF5

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
      struct {
         int32_t audadc_dc_offset;
         uint32_t audadc_dc_offset_tag;
      };
   };
   union {
      uint32_t last_known_timestamp_addr[4];
      uint32_t last_known_timestamp;
   };
   union {
      uint32_t boot_record_addr[4];
      struct {
         uint32_t boot_epoch;
         uint32_t hardware_status;
         uint32_t software_reason;
         uint32_t fault_address;
      };
   };
} persistent_data_t;

static persistent_data_t persistent_data;


// Private Helper Functions --------------------------------------------------------------------------------------------

static bool program_block(uint32_t block_offset, const uint32_t *source)
{
   // Program one 16-byte block of the persistent structure, verifying the result by reading it back
   uint32_t *destination = (uint32_t*)((uint8_t*)_persistent_base_address + block_offset);
   for (uint32_t attempt = 0; attempt < MRAM_MAX_PROGRAM_ATTEMPTS; ++attempt)
   {
      int result;
      AM_CRITICAL_BEGIN
      result = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, (uint32_t*)source, destination, 4);
      AM_CRITICAL_END
      if (result == 0)
      {
         // Verify by read-back so that a silently failed program is reported rather than assumed
         bool verified = true;
         for (uint32_t i = 0; i < 4; ++i)
            if (destination[i] != source[i])
               verified = false;
         if (verified)
            return true;
      }
   }
   return false;
}


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
   persistent_data.is_activated = activated ? 10 : 0;
   persistent_data.deployment_start_time = deployment_time;
   return program_block(offsetof(persistent_data_t, is_activated_addr), persistent_data.is_activated_addr);
}

bool mram_is_activated(void)
{
   // Return whether the activation flag is as expected
   return persistent_data.is_activated == 10;
}

uint32_t mram_get_deployment_start_time(void)
{
   // Retrieve the last known deployment start time
   return persistent_data.deployment_start_time;
}

bool mram_set_last_known_timestamp(uint32_t timestamp)
{
   // Rate limit writes so that a months-long deployment does not repeatedly program the same word
   const uint32_t stored = persistent_data.last_known_timestamp;
   if (!timestamp || ((timestamp >= stored) && ((timestamp - stored) < MRAM_TIMESTAMP_WRITE_INTERVAL)))
      return true;

   // Store the current timestamp to persistent memory
   persistent_data.last_known_timestamp = timestamp;
   return program_block(offsetof(persistent_data_t, last_known_timestamp_addr), persistent_data.last_known_timestamp_addr);
}

uint32_t mram_get_last_known_timestamp(void)
{
   // Return the last timestamp stored in persistent memory
   return persistent_data.last_known_timestamp;
}

bool mram_store_audadc_dc_offset(int32_t dc_offset)
{
   // Store the AUDADC DC offset value to persistent memory along with its validity marker
   persistent_data.audadc_dc_offset = dc_offset;
   persistent_data.audadc_dc_offset_tag = MRAM_DC_OFFSET_VALID_TAG;
   return program_block(offsetof(persistent_data_t, audadc_dc_offset_addr), persistent_data.audadc_dc_offset_addr);
}

int32_t mram_get_audadc_dc_offset(void)
{
   // Only report a cached offset if one has actually been written
   return (persistent_data.audadc_dc_offset_tag == MRAM_DC_OFFSET_VALID_TAG) ? persistent_data.audadc_dc_offset : 0;
}

bool mram_increment_boot_epoch(void)
{
   // Advance the power-on counter
   if (persistent_data.boot_epoch == 0xFFFFFFFF)
      persistent_data.boot_epoch = 0;
   ++persistent_data.boot_epoch;
   return program_block(offsetof(persistent_data_t, boot_record_addr), persistent_data.boot_record_addr);
}

uint32_t mram_get_boot_epoch(void)
{
   return (persistent_data.boot_epoch == 0xFFFFFFFF) ? 0 : persistent_data.boot_epoch;
}

void mram_get_boot_record(mram_boot_record_t *record)
{
   // Return the reset details recorded by the previous run
   record->boot_epoch = mram_get_boot_epoch();
   record->hardware_status = (persistent_data.hardware_status == 0xFFFFFFFF) ? 0 : persistent_data.hardware_status;
   record->software_reason = (persistent_data.software_reason == 0xFFFFFFFF) ? RESET_REASON_NONE : persistent_data.software_reason;
   record->fault_address = (persistent_data.fault_address == 0xFFFFFFFF) ? 0 : persistent_data.fault_address;
}

bool mram_store_fault(uint32_t software_reason, uint32_t hardware_status, uint32_t fault_address)
{
   // Record why the device is about to restart, in the same block as the boot epoch
   persistent_data.hardware_status = hardware_status;
   persistent_data.software_reason = software_reason;
   persistent_data.fault_address = fault_address;
   return program_block(offsetof(persistent_data_t, boot_record_addr), persistent_data.boot_record_addr);
}
