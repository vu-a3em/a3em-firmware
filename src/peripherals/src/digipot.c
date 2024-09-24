// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "digipot.h"


// Static Global Variables and Definitions -----------------------------------------------------------------------------

#define REG_ADDRESS_ACR   0x10
#define REG_ADDRESS_WR    0x0

static void *i2c_handle = NULL;


// Private Helper Functions --------------------------------------------------------------------------------------------

static int32_t i2c_read(void *handle, uint8_t reg_number, uint8_t *read_buffer)
{
   am_hal_iom_transfer_t read_transaction = {
      .uPeerInfo.ui32I2CDevAddr     = DIGIPOT_I2C_ADDRESS,
      .ui32InstrLen                 = 1,
      .ui64Instr                    = reg_number,
      .eDirection                   = AM_HAL_IOM_RX,
      .ui32NumBytes                 = 1,
      .pui32TxBuffer                = NULL,
      .pui32RxBuffer                = (uint32_t*)read_buffer,
      .bContinue                    = false,
      .ui8RepeatCount               = 0,
      .ui8Priority                  = 1,
      .ui32PauseCondition           = 0,
      .ui32StatusSetClr             = 0
   };
   return am_hal_iom_blocking_transfer(i2c_handle, &read_transaction);
}

static int32_t i2c_write(void *handle, uint8_t reg_number, const uint8_t *write_buffer)
{
   am_hal_iom_transfer_t write_transaction = {
      .uPeerInfo.ui32I2CDevAddr     = DIGIPOT_I2C_ADDRESS,
      .ui32InstrLen                 = 1,
      .ui64Instr                    = reg_number,
      .eDirection                   = AM_HAL_IOM_TX,
      .ui32NumBytes                 = 1,
      .pui32TxBuffer                = (uint32_t*)write_buffer,
      .pui32RxBuffer                = NULL,
      .bContinue                    = false,
      .ui8RepeatCount               = 0,
      .ui8Priority                  = 1,
      .ui32PauseCondition           = 0,
      .ui32StatusSetClr             = 0
   };
   return am_hal_iom_blocking_transfer(i2c_handle, &write_transaction);
}


// Public API Functions ------------------------------------------------------------------------------------------------

void digipot_init(void)
{
   // Create an I2C configuration structure
   const am_hal_iom_config_t i2c_config =
   {
      .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
      .ui32ClockFreq = AM_HAL_IOM_400KHZ,
      .eSpiMode = 0,
      .pNBTxnBuf = NULL,
      .ui32NBTxnBufLength = 0
   };

   // Initialize the I2C module and enable all relevant I2C pins
   am_hal_gpio_pincfg_t scl_config = g_AM_BSP_GPIO_IOM0_SCL;
   am_hal_gpio_pincfg_t sda_config = g_AM_BSP_GPIO_IOM0_SDA;
   scl_config.GP.cfg_b.uFuncSel = PIN_DIGIPOT_I2C_SCL_FUNCTION;
   sda_config.GP.cfg_b.uFuncSel = PIN_DIGIPOT_I2C_SDA_FUNCTION;
   configASSERT0(am_hal_iom_initialize(DIGIPOT_I2C_NUMBER, &i2c_handle));
   configASSERT0(am_hal_gpio_pinconfig(PIN_DIGIPOT_I2C_SCL, scl_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_DIGIPOT_I2C_SDA, sda_config));
   am_hal_iom_power_ctrl(i2c_handle, AM_HAL_SYSCTRL_WAKE, false);
   am_hal_iom_configure(i2c_handle, &i2c_config);
   am_hal_iom_enable(i2c_handle);

   // Ensure that the chip is not in shutdown mode
   const uint8_t active_mode = 0x40;
   i2c_write(i2c_handle, REG_ADDRESS_ACR, &active_mode);
}

void digipot_deinit(void)
{
   // Only de-initialize if handle exists
   if (i2c_handle)
   {
      // Put the chip into shutdown mode
      const uint8_t shutdown_mode = 0x0;
      i2c_write(i2c_handle, REG_ADDRESS_ACR, &shutdown_mode);

      // Disable all I2C communications
      while (am_hal_iom_disable(i2c_handle) != AM_HAL_STATUS_SUCCESS);
      am_hal_iom_uninitialize(i2c_handle);
      i2c_handle = NULL;
   }
}

float digipot_get_percent_output(void)
{
   // Read the current output percent from the wiper register
   uint8_t wiper_value = 0;
   i2c_read(i2c_handle, REG_ADDRESS_WR, &wiper_value);
   return (float)wiper_value / 255.0f;
}

void digipot_set_percent_output(float percent)
{
   // Write the desired output percent into the wiper register
   uint8_t wiper_value = (uint8_t)(255 * percent);
   i2c_write(i2c_handle, REG_ADDRESS_WR, &wiper_value);
}
