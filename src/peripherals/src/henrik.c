// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "henrik.h"


// Static Global Variables and Definitions -----------------------------------------------------------------------------



// Private Helper Functions --------------------------------------------------------------------------------------------



// Public API Functions ------------------------------------------------------------------------------------------------

void henrik_init(void)
{
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
}

void henrik_deinit(void)
{
   // TODO: Disable all I2C communications
}

void henrik_get_gps_reading(void)
{
   // TODO: Request GPS reading
}

uint32_t henrik_get_gps_timestamp(void)
{
   // TODO: Request GPS timestamp  (ensure in UTC and not GPS time)
   return 0;
}

void henrik_get_gps_location(float *lat, float *lon, float *height)
{
   // TODO: Request GPS reading
}

void henrik_send_alert(void)
{
   // TODO: Send alert
}
