// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "henrik.h"


// Static Global Variables and Definitions -----------------------------------------------------------------------------

#define TX_BUFFER_MAX_BYTES     1023

static henrik_status_data_t status_data;
static volatile henrik_gps_data_t gps_data;
static henrik_data_callback_t data_callback;
static uint8_t tx_buffer[TX_BUFFER_MAX_BYTES];
static void *i2c_handle = NULL;


// Private Helper Functions --------------------------------------------------------------------------------------------

void alert_host(void)
{
   // Wake up the host to initiate communications
   am_hal_gpio_output_set(PIN_EXT_HW_INTERRUPT);
   am_hal_ios_control(i2c_handle, AM_HAL_IOS_REQ_FIFO_UPDATE_CTR, NULL);
   am_hal_gpio_output_clear(PIN_EXT_HW_INTERRUPT);
}

void am_ioslave_ios_isr(void)
{
   // Clear the interrupt status and de-assert the HW interrupt line
   static uint32_t status;
   am_hal_ios_interrupt_status_get(i2c_handle, false, &status);
   am_hal_ios_interrupt_clear(i2c_handle, status);
   am_hal_gpio_output_set(PIN_EXT_HW_INTERRUPT);

   // Handle any incoming data
   if (status & AM_HAL_IOS_INT_FSIZE)
      am_hal_ios_interrupt_service(i2c_handle, status);
   if (status & AM_HAL_IOS_INT_XCMPWR)
   {
      // Handle incoming data based on its type
      const uint8_t *packet = (uint8_t*)am_hal_ios_pui8LRAM;
      const uint8_t data_length = packet[0], message_type = packet[1];
      if ((message_type == MSG_GPS) && (data_length == sizeof(henrik_gps_data_t)))
         gps_data = *(const henrik_gps_data_t*)(packet + 1);
      else if ((message_type == MSG_STATUS_REQUEST) && (data_length == 1))
         henrik_send_status_update();

      // Invoke a registered data listener for any received data
      if (data_callback)
         data_callback(message_type, packet + 1);
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void henrik_init(void)
{
   // Initialize peripheral variables
   data_callback = NULL;
   memset((void*)&status_data, 0, sizeof(status_data));
   memset((void*)&gps_data, 0, sizeof(gps_data));
   memset((void*)&tx_buffer, 0, sizeof(tx_buffer));
   status_data.msg_type = MSG_STATUS;

   // Create an I2C slave configuration structure
   am_hal_ios_config_t i2c_config =
   {
      .ui32InterfaceSelect = AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(EXT_HW_I2C_ADDRESS << 1),
      .ui32ROBase = 0x78,
      .ui32FIFOBase = 0x80,
      .ui32RAMBase = 0x100,
      .ui32FIFOThreshold = 0x40,
      .pui8SRAMBuffer = tx_buffer,
      .ui32SRAMBufferCap = sizeof(tx_buffer),
      .ui8WrapEnabled = false
   };

   // Configure the external interrupt request pin
   const am_hal_gpio_pincfg_t ext_hw_interrupt_pin_config = AM_HAL_GPIO_PINCFG_OPENDRAIN;
   configASSERT0(am_hal_gpio_pinconfig(PIN_EXT_HW_INTERRUPT, ext_hw_interrupt_pin_config));
   am_hal_gpio_output_set(PIN_EXT_HW_INTERRUPT);

   // Initialize the I2C module and enable all relevant I2C pins
   am_hal_gpio_pincfg_t scl_config = g_AM_BSP_GPIO_IOS_SCL;
   am_hal_gpio_pincfg_t sda_config = g_AM_BSP_GPIO_IOS_SDA;
   scl_config.GP.cfg_b.uFuncSel = PIN_EXT_HW_I2C_SCL_FUNCTION;
   sda_config.GP.cfg_b.uFuncSel = PIN_EXT_HW_I2C_SDA_FUNCTION;
   configASSERT0(am_hal_gpio_pinconfig(PIN_EXT_HW_I2C_SCL, scl_config));
   configASSERT0(am_hal_gpio_pinconfig(PIN_EXT_HW_I2C_SDA, sda_config));
   configASSERT0(am_hal_ios_initialize(EXT_HW_I2C_NUMBER, &i2c_handle));
   configASSERT0(am_hal_ios_power_ctrl(i2c_handle, AM_HAL_SYSCTRL_WAKE, false));
   configASSERT0(am_hal_ios_configure(i2c_handle, &i2c_config));

   // Enable data transfer interrupts
   am_hal_ios_interrupt_clear(i2c_handle, AM_HAL_IOS_INT_ALL);
   am_hal_ios_interrupt_enable(i2c_handle, AM_HAL_IOS_INT_FOVFL | AM_HAL_IOS_INT_FUNDFL | AM_HAL_IOS_INT_FRDERR | AM_HAL_IOS_INT_FSIZE |
                                           AM_HAL_IOS_INT_XCMPWR | AM_HAL_IOS_INT_XCMPWF | AM_HAL_IOS_INT_XCMPRR | AM_HAL_IOS_INT_XCMPRF);
   NVIC_SetPriority(IOSLAVE_IRQn, EXT_HW_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(IOSLAVE_IRQn);
}

void henrik_deinit(void)
{
   // Only de-initialize if handle exists
   if (i2c_handle)
   {
      // Disable all I2C communications
      am_hal_gpio_output_set(PIN_EXT_HW_INTERRUPT);
      while (am_hal_ios_disable(i2c_handle) != AM_HAL_STATUS_SUCCESS);
      am_hal_ios_uninitialize(i2c_handle);
      data_callback = NULL;
      i2c_handle = NULL;
   }
}

void henrik_register_data_callback(henrik_data_callback_t callback)
{
   // Register an interrupt handler for incoming communications
   data_callback = callback;
}

uint32_t henrik_get_current_time(void)
{
   // Check if a valid GPS timestamp has been received
   if (gps_data.utc_timestamp)
   {
      const uint32_t timestamp = gps_data.utc_timestamp;
      gps_data.utc_timestamp = 0;
      return timestamp;
   }
   else
   {
      // Request a new GPS timestamp (ensure in UTC and not GPS time)
      uint32_t num_written = 0;
      const uint8_t gps_request_msg = MSG_GPS_REQUEST;
      am_hal_ios_fifo_write(i2c_handle, (uint8_t*)&gps_request_msg, sizeof(gps_request_msg), &num_written);
      alert_host();
      return 0;
   }
}

void henrik_update_status_data(uint32_t timestamp)
{
   // Simply update the current status data structure
   status_data.utc_timestamp = timestamp;
}

void henrik_send_alert(const henrik_alert_data_t *alert)
{
   // Write alert data to the FIFO for host retrieval
   uint32_t num_written = 0;
   am_hal_ios_fifo_write(i2c_handle, (uint8_t*)alert, sizeof(*alert), &num_written);
   alert_host();
}

void henrik_send_status_update(void)
{
   // Write status data to the FIFO for host retrieval
   uint32_t num_written = 0;
   am_hal_ios_fifo_write(i2c_handle, (uint8_t*)&status_data, sizeof(status_data), &num_written);
   alert_host();
}
