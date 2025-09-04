#include "henrik.h"
#include "logging.h"
#include "system.h"

#define I2C_HOST

#ifndef I2C_HOST

static void henrik_data_available(henrik_msg_t message_type, const void *new_data)
{
   // Handle the incoming message based on its type
   switch (message_type)
   {
      case MSG_GPS:
      {
         const henrik_gps_data_t *gps_data = (const henrik_gps_data_t*)new_data;
         print("GPS data received: Timestamp = %u, Lat = %.2f, Lon = %.2f, Ht = %.2f\n",
               gps_data->utc_timestamp, gps_data->lat, gps_data->lon, gps_data->height);
         break;
      }
      case MSG_CONFIG:
         const henrik_config_data_t *config_data = (const henrik_config_data_t*)new_data;
         print("Config data received: Msg Type = %u, TODO\n", (uint32_t)config_data->msg_type);
         break;
      default:
         break;
   }
}

int main(void)
{
   // Set up system hardware
   setup_hardware();
   henrik_init();
   system_enable_interrupts(true);

   // Listen for incoming data from Henrik's board
   henrik_register_data_callback(henrik_data_available);

   // Loop forever
   uint32_t current_time = 0;
   while (true)
   {
      // Test GPS time requests
      print("Waiting for a GPS time update from Henrik...\n");
      am_hal_delay_us(1000000);
      while (!(current_time = henrik_get_current_time()))
         am_hal_delay_us(1000000);
      print("GPS time received!\n");

      // Test intermittent status updates
      for (int i = 0; i < 3; ++i)
      {
         for (int j = 0; j < 3; ++j)
         {
            print("Updating on-device status data (time = %u)...\n", ++current_time);
            henrik_update_status_data(current_time);
            am_hal_delay_us(1000000);
         }
         print("Sending status update to Henrik...\n");
         henrik_send_status_update();
         print("Status update sent!\n");
         am_hal_delay_us(1000000);
      }

      // Test sending alerts
      for (int i = 0; i < 3; ++i)
      {
         print("Sending alert data (time = %u)...\n", ++current_time);
         const henrik_alert_data_t alert_data = { .utc_timestamp = current_time };
         henrik_send_alert(&alert_data);
         am_hal_delay_us(1000000);
      }
   }

   // Should never reach this point
   return 0;
}

#else

#define REG_DATA_LENGTH     0x7C
#define REG_DATA            0x7F
#define REG_WRITE           0x80

static void *i2c_handle;
static uint8_t data[256];
static volatile bool new_data = false;
static am_hal_iom_config_t i2c_config = { .eInterfaceMode = AM_HAL_IOM_I2C_MODE, .ui32ClockFreq  = AM_HAL_IOM_250KHZ };
const am_hal_gpio_pincfg_t hw_int_pin_config =
{
   .GP.cfg_b.uFuncSel       = AM_HAL_PIN_35_GPIO,
   .GP.cfg_b.ePullup        = AM_HAL_GPIO_PIN_PULLUP_24K,
   .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
   .GP.cfg_b.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
   .GP.cfg_b.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_HI2LO
};

void iom_slave_read(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
   am_hal_iom_transfer_t transaction = {
      .ui32InstrLen = 1,
      .ui64Instr = offset,
      .eDirection = AM_HAL_IOM_RX,
      .ui32NumBytes = size,
      .pui32RxBuffer = pBuf,
      .bContinue = false,
      .ui8RepeatCount = 0,
      .ui32PauseCondition = 0,
      .ui32StatusSetClr = 0,
      .uPeerInfo.ui32I2CDevAddr = EXT_HW_I2C_ADDRESS
   };
   uint32_t s = am_hal_iom_blocking_transfer(i2c_handle, &transaction);
   print("R: %u\n", s);
}

void iom_slave_write(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
   am_hal_iom_transfer_t transaction = {
      .ui32InstrLen = 1,
      .ui64Instr = offset,
      .eDirection = AM_HAL_IOM_TX,
      .ui32NumBytes = size,
      .pui32TxBuffer = pBuf,
      .bContinue = false,
      .ui8RepeatCount = 0,
      .ui32PauseCondition = 0,
      .ui32StatusSetClr = 0,
      .uPeerInfo.ui32I2CDevAddr = EXT_HW_I2C_ADDRESS
   };
   uint32_t s = am_hal_iom_blocking_transfer(i2c_handle, &transaction);
   print("W: %u\n", s);
}

void i2c_interrupt_handler(void *arg)
{
   // Read the data length
   uint32_t data_length = 0;
   iom_slave_read(REG_DATA_LENGTH, &data_length, 2);
   if (data_length)
   {
      // Read the data and set the new_data flag
      iom_slave_read(REG_DATA, (uint32_t*)data, data_length);
      print("Read %u bytes of I2C data\n", data_length);
      new_data = true;
   }
}

int main(void)
{
   // Set up system hardware
   setup_hardware();

   // Set up the I2C master peripheral
   am_hal_iom_initialize(3, &i2c_handle);
   am_hal_iom_power_ctrl(i2c_handle, AM_HAL_SYSCTRL_WAKE, false);
   am_hal_iom_configure(i2c_handle, &i2c_config);
   am_bsp_iom_pins_enable(3, AM_HAL_IOM_I2C_MODE);
   am_hal_iom_enable(i2c_handle);

   // Set up the HW interrupt pin
   uint32_t int_num = PIN_EXT_HW_INTERRUPT;
   am_hal_gpio_pinconfig(PIN_EXT_HW_INTERRUPT, hw_int_pin_config);
   am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, (am_hal_gpio_mask_t*)&int_num);
   am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, PIN_EXT_HW_INTERRUPT, i2c_interrupt_handler, NULL);
   am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, &int_num);
   NVIC_SetPriority(GPIO0_001F_IRQn, AM_IRQ_PRIORITY_DEFAULT);
   NVIC_EnableIRQ(GPIO0_001F_IRQn);
   am_hal_interrupt_master_enable();

   // Loop forever
   uint32_t num_wakeups = 0;
   henrik_config_data_t config_data = { .msg_type = MSG_CONFIG };
   henrik_gps_data_t gps_data = { .msg_type = MSG_GPS, .utc_timestamp = 0, .lat = 1.0f, .lon = 2.0f, .height = 3.0f };
   while (true)
   {
      // Sleep until woken up by an interrupt
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

      // Handle newly received data
      if (new_data)
      {
         new_data = false;
         switch (data[0])
         {
            case MSG_GPS_REQUEST:
               gps_data.utc_timestamp++;
               print("Received MSG_GPS_REQUEST...responding with t = %u\n", gps_data.utc_timestamp);
               iom_slave_write(REG_WRITE, (uint32_t*)&gps_data, sizeof(gps_data));
               break;
            case MSG_ALERT:
               print("Received MSG_ALERT with t = %u\n", *(uint32_t*)(data+1));
               break;
            case MSG_STATUS:
               print("Received MSG_STATUS with t = %u\n", *(uint32_t*)(data+1));
               break;
            default:
               break;
         }

         // Send config message every 5 wake-ups
         if ((++num_wakeups % 5) == 0)
            iom_slave_write(REG_WRITE, (uint32_t*)&config_data, sizeof(config_data));
      }
   }

   // Should never reach this point
   return 0;
}

#endif
