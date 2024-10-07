// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "henrik.h"


// Static Global Variables and Definitions -----------------------------------------------------------------------------

static henrik_data_callback_t data_callback;
static volatile henrik_data_t unread_data;
static bool interrupts_enabled;


// Private Helper Functions --------------------------------------------------------------------------------------------

void enable_data_interrupts(void)
{
   // Only enable interrupts if not already enabled
   if (!interrupts_enabled)
   {
      // TODO: Enable them
      interrupts_enabled = true;
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void henrik_init(void)
{
   // Initialize peripheral variables
   data_callback = NULL;
   interrupts_enabled = false;
   memset((void*)&unread_data, 0, sizeof(unread_data));

   // TODO: Create an I2C configuration structure
   /*const am_hal_ios_config_t i2c_config =
   {
      .ui32InterfaceSelect = AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(EXT_HW_I2C_ADDRESS),
      .ui32ROBase = 0,
      .ui32FIFOBase = 0,
      .ui32RAMBase = 0,
      .ui32FIFOThreshold = 0,
      .pui8SRAMBuffer = NULL,
      .ui32SRAMBufferCap = 0,
      .ui8WrapEnabled = false
   };*/

   // TODO: Initialize the I2C module and enable all relevant I2C pins

   // Enable incoming data interrupts
   enable_data_interrupts();
}

void henrik_deinit(void)
{
   // TODO: Disable all I2C communications and interrupts
   interrupts_enabled = false;
   data_callback = NULL;
}

void henrik_get_gps_reading(void)
{
   // TODO: Request GPS reading
}

henrik_data_t henrik_get_data(void)
{
   // TODO: Request GPS timestamp  (ensure in UTC and not GPS time)
   enable_data_interrupts();
   henrik_data_t data = unread_data;
   unread_data.new_data = false;
   return data;
}

void henrik_register_data_callback(henrik_data_callback_t callback)
{
   // TODO: Register interrupt handler for i2c incoming comms
   data_callback = callback;
}

void henrik_send_alert(void)
{
   // TODO: Send alert
}
