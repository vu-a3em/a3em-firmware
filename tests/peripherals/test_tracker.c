#include "logging.h"
#include "system.h"
#include "tracker.h"

//#define I2C_HOST

#ifndef I2C_HOST

static void tracker_data_available(tracker_msg_t message_type, const void *new_data)
{
   // Handle the incoming message based on its type
   switch (message_type)
   {
      case MSG_GPS:
      {
         const tracker_gps_data_t *gps_data = (const tracker_gps_data_t*)new_data;
         print("GPS data received: Timestamp = %u, Lat = %.2f, Lon = %.2f, Ht = %.2f\n",
               gps_data->utc_timestamp, gps_data->lat, gps_data->lon, gps_data->height);
         break;
      }
      case MSG_CONFIG:
         const tracker_config_data_t *config_data = (const tracker_config_data_t*)new_data;
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
   tracker_init();
   system_enable_interrupts(true);

   // Listen for incoming data from the GPS Tracker board
   tracker_register_data_callback(tracker_data_available);

   // Loop forever
   uint32_t current_time = 0;
   while (true)
   {
      // Test GPS time requests
      print("Waiting for a GPS time update from Tracker...\n");
      am_hal_delay_us(1000000);
      while (!(current_time = tracker_get_current_time()))
         am_hal_delay_us(1000000);
      print("GPS time received!\n");

      // Test intermittent status updates
      for (int i = 0; i < 3; ++i)
      {
         for (int j = 0; j < 3; ++j)
         {
            print("Updating on-device status data (time = %u)...\n", ++current_time);
            tracker_update_status_data(current_time);
            am_hal_delay_us(1000000);
         }
         print("Sending status update to Tracker...\n");
         tracker_send_status_update();
         print("Status update sent!\n");
         am_hal_delay_us(1000000);
      }

      // Test sending alerts
      for (int i = 0; i < 3; ++i)
      {
         print("Sending alert data (time = %u)...\n", ++current_time);
         const tracker_alert_data_t alert_data = { .utc_timestamp = current_time };
         tracker_send_alert(&alert_data);
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
   tracker_config_data_t config_data = { .msg_type = MSG_CONFIG };
   tracker_gps_data_t gps_data = { .msg_type = MSG_GPS, .utc_timestamp = 0, .lat = 1.0f, .lon = 2.0f, .height = 3.0f };
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

/*
//#define I2C_HOST

#ifdef I2C_HOST

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define IOM_MODULE          3
#define USE_SPI             0
#define I2C_ADDR            0x10
#define MAX_SIZE            10000
#define XOR_BYTE            0
#define EMPTY_BYTE          0xEE

typedef enum
{
    AM_IOSTEST_CMD_START_DATA    = 0,
    AM_IOSTEST_CMD_STOP_DATA     = 1,
    AM_IOSTEST_CMD_ACK_DATA      = 2,
} AM_IOSTEST_CMD_E;

#define IOSOFFSET_WRITE_INTEN       0xF8
#define IOSOFFSET_WRITE_INTCLR      0xFA
#define IOSOFFSET_WRITE_CMD         0x80
#define IOSOFFSET_READ_INTSTAT      0x79
#define IOSOFFSET_READ_FIFO         0x7F
#define IOSOFFSET_READ_FIFOCTR      0x7C

#define AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK  1

#define HANDSHAKE_PIN                   3

#define AM_TEST_RCV_BUF_SIZE    1024 // Max Size we can receive is 1023
uint8_t g_pui8RcvBuf[AM_TEST_RCV_BUF_SIZE];
volatile uint32_t g_startIdx = 0;
volatile bool bIosInt = false;
void *g_IOMHandle;

static am_hal_iom_config_t g_sIOMI2cConfig =
{
    .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq  = AM_HAL_IOM_100KHZ,
};
#define MAX_I2C_SIZE   255

const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_HANDSHAKE =
{
    .GP.cfg_b.uFuncSel       = AM_HAL_PIN_3_GPIO,
    .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

void clear_rx_buf(void)
{
    uint32_t i;
    for ( i = 0; i < AM_TEST_RCV_BUF_SIZE; i++ )
    {
        g_pui8RcvBuf[i] = EMPTY_BYTE;
    }
}

uint32_t validate_rx_buf(uint32_t rxSize)
{
    uint32_t i;
    for ( i = 0; i < rxSize; i++ )
    {
        if ( g_pui8RcvBuf[i] != (((g_startIdx + i) & 0xFF) ^ XOR_BYTE) )
        {
            am_util_stdio_printf("Failed to compare buffers at index %d \n", i);
            break;
        }
    }

    //
    // Set the reference for next chunk
    //
    g_startIdx += rxSize;
    return (i == rxSize);
}

void hostint_handler(void)
{
    bIosInt = true;
}

void iom_slave_read(bool bSpi, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui64Instr = offset;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if ( bSpi )
    {
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
    }
    else
    {
        Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
    }
    uint32_t r = am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
    am_util_stdio_printf("R: %u\n", r);
}

void iom_slave_write(bool bSpi, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui64Instr = offset;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32TxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if ( bSpi )
    {
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
    }
    else
    {
        Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
    }
    uint32_t r = am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
    am_util_stdio_printf("W: %u\n", r);
}

static void iom_set_up(uint32_t iomModule, bool bSpi)
{
    uint32_t ioIntEnable = AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK;

    //
    // Initialize the IOM.
    //
    am_hal_iom_initialize(iomModule, &g_IOMHandle);

    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);

    if ( bSpi )
    {
        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_SPI_MODE);
    }
    else
    {
        //
        // Set the required configuration settings for the IOM.
        //
        am_hal_iom_configure(g_IOMHandle, &g_sIOMI2cConfig);

        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_I2C_MODE);
    }

    //
    // Enable the IOM.
    //
    am_hal_iom_enable(g_IOMHandle);
    am_hal_gpio_pinconfig(HANDSHAKE_PIN, g_AM_BSP_GPIO_HANDSHAKE);

    uint32_t IntNum = HANDSHAKE_PIN;
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

    //
    // Set up the host IO interrupt
    //
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, (am_hal_gpio_mask_t*)&IntNum);

    //
    // Register handler for IOS => IOM interrupt
    //
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, HANDSHAKE_PIN,
                                    (am_hal_gpio_handler_t)hostint_handler, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
    NVIC_SetPriority(GPIO0_001F_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(GPIO0_001F_IRQn);

    //
    // Set up IOCTL interrupts
    // IOS ==> IOM
    //
    iom_slave_write(bSpi, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);
}

uint32_t g_ui32LastUpdate = 0;

void update_progress(uint32_t ui32NumPackets)
{
    //
    // Print a dot every 10000 packets.
    //
    if ( (ui32NumPackets - g_ui32LastUpdate) > 1000 )
    {
        am_util_stdio_printf(".");
        g_ui32LastUpdate = ui32NumPackets;
    }
}

int main(void)
{
    uint32_t iom = IOM_MODULE;
    bool bSpi = USE_SPI;
    bool bReadIosData = false;
    bool bDone = false;
    uint32_t data;
    uint32_t maxSize = MAX_I2C_SIZE;

    //
    // Enable the ITM print interface.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("IOS Test Host: Waiting for at least %d bytes from the slave.\n", MAX_SIZE);

    //
    // Allow time for all printing to finish.
    //
    am_util_delay_ms(10);

    //
    // Enable Interrupts.
    //
    am_hal_interrupt_master_enable();

    //
    // Set up the IOM
    //
    iom_set_up(iom, bSpi);

    //
    // Make sure the print is complete
    //
    am_util_delay_ms(100);

    //
    // Send the START
    //
    data = AM_IOSTEST_CMD_START_DATA;
    iom_slave_write(bSpi, IOSOFFSET_WRITE_CMD, &data, 1);

    //
    // Loop forever.
    //
    while ( !bDone )
    {
        //
        // Disable interrupt while we decide whether we're going to sleep.
        //
        uint32_t ui32IntStatus = am_hal_interrupt_master_disable();

        if ( bIosInt == true )
        {
            //
            // Enable interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);
            bIosInt = false;

            //
            // Read & Clear the IOINT status
            //
            iom_slave_read(bSpi, IOSOFFSET_READ_INTSTAT, &data, 1);

            //
            // We need to clear the bit by writing to IOS
            //
            if ( data & AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK )
            {
                data = AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK;
                iom_slave_write(bSpi, IOSOFFSET_WRITE_INTCLR, &data, 1);

                //
                // Set bReadIosData
                //
                bReadIosData = true;
            }
            if ( bReadIosData )
            {
                uint32_t iosSize = 0;

                bReadIosData = false;

                //
                // Read the Data Size
                //
                iom_slave_read(bSpi, IOSOFFSET_READ_FIFOCTR, &iosSize, 2);
                iosSize = (iosSize > maxSize)? maxSize: iosSize;

                //
                // Initialize Rx Buffer for later comparison
                //
                clear_rx_buf();

                //
                // Read the data
                //
                iom_slave_read(bSpi, IOSOFFSET_READ_FIFO,
                    (uint32_t *)g_pui8RcvBuf, iosSize);

                //
                // Validate Content
                //
                if ( !validate_rx_buf(iosSize) )
                {
                    am_util_stdio_printf("\nData Verification failed Accum:%lu rx=%d\n",
                        g_startIdx, iosSize);
                }

                //
                // Send the ACK/STOP
                //
                data = AM_IOSTEST_CMD_ACK_DATA;

                update_progress(g_startIdx);

                if ( g_startIdx >= MAX_SIZE )
                {
                    bDone = true;
                    data = AM_IOSTEST_CMD_STOP_DATA;
                }
                iom_slave_write(bSpi, IOSOFFSET_WRITE_CMD, &data, 1);
            }
        }
        else
        {
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

            //
            // Enable interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);
        }
    }

    am_util_stdio_printf("\nTest Done - Total Received = =%d\n", g_startIdx);
    while (1);
}

#else

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define USE_SPI              0
#define I2C_ADDR             0x10
#define TEST_IOS_XCMP_INT    1
#define SENSOR0_DATA_SIZE    200
#define SENSOR1_DATA_SIZE    500

#define     SENSOR0_FREQ   12 //<! 12 times a second
#define     SENSOR1_FREQ    7 //<! 7 times a second
#define     XOR_BYTE            0
#define     AM_HAL_IOS_INT_ERR  (AM_HAL_IOS_INT_FOVFL | AM_HAL_IOS_INT_FUNDFL | AM_HAL_IOS_INT_FRDERR)
#define AM_HAL_IOS_XCMP_INT (AM_HAL_IOS_INT_XCMPWR | AM_HAL_IOS_INT_XCMPWF | AM_HAL_IOS_INT_XCMPRR | AM_HAL_IOS_INT_XCMPRF)

typedef enum
{
    AM_IOSTEST_CMD_START_DATA    = 0,
    AM_IOSTEST_CMD_STOP_DATA     = 1,
    AM_IOSTEST_CMD_ACK_DATA      = 2,
} AM_IOSTEST_CMD_E;

#define AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK  1

typedef enum
{
    AM_IOSTEST_SLAVE_STATE_NODATA   = 0,
    AM_IOSTEST_SLAVE_STATE_DATA     = 1,
} AM_IOSTEST_SLAVE_STATE_E;

volatile AM_IOSTEST_SLAVE_STATE_E g_iosState;
volatile uint32_t g_sendIdx = 0;
volatile bool g_bSensor0Data, g_bSensor1Data;
static void *g_pIOSHandle;

#define HANDSHAKE_PIN                   3


#define AM_TEST_REF_BUF_SIZE    512
uint8_t g_pui8TestBuf[AM_TEST_REF_BUF_SIZE];

#define AM_IOS_TX_BUFSIZE_MAX   1023
uint8_t g_pui8TxFifoBuffer[AM_IOS_TX_BUFSIZE_MAX];

const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_OUTPUT =
{
    .GP.cfg_b.uFuncSel            = AM_HAL_PIN_3_GPIO,
    .GP.cfg_b.eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .GP.cfg_b.eCEpol              = AM_HAL_GPIO_PIN_CEPOL_ACTIVEHIGH,
    .GP.cfg_b.eGPOutCfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .GP.cfg_b.ePullup             = AM_HAL_GPIO_PIN_PULLUP_24K,
};

am_hal_ios_config_t g_sIOSI2cConfig =
{
    //
    // Configure the IOS in I2C mode.
    //
    .ui32InterfaceSelect = AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(I2C_ADDR << 1),

    //
    // Eliminate the "read-only" section, so an external host can use the
    // entire "direct write" section.
    //
    .ui32ROBase = 0x78,

    //
    // Set the FIFO base to the maximum value, making the "direct write"
    // section as big as possible.
    //
    .ui32FIFOBase = 0x80,

    //
    // We don't need any RAM space, so extend the FIFO all the way to the end
    // of the LRAM.
    //
    .ui32RAMBase = 0x100,

    //
    // FIFO Threshold - set to half the size
    //
    .ui32FIFOThreshold = 0x40,

    .pui8SRAMBuffer = g_pui8TxFifoBuffer,
    .ui32SRAMBufferCap = AM_IOS_TX_BUFSIZE_MAX,
};

void timer0_handler(void)
{
    //
    // Inform main loop of sensor 0 Data availability
    //
    g_bSensor0Data = true;
}

void timer1_handler(void)
{
    // Inform main loop of sensor 1 Data availability
    g_bSensor1Data = true;
}

void am_timer00_isr(void)
{
    //
    // Clear Timer0 Interrupt (write to clear).
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE0));
    am_hal_timer_clear(0);

    timer0_handler();
}

void am_timer02_isr(void)
{
    //
    // Clear Timer2 Interrupt (write to clear).
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(2, AM_HAL_TIMER_COMPARE0));
    am_hal_timer_clear(2);

    timer1_handler();
}

void stop_sensors(void)
{
    //
    // Stop timer 0
    //
    am_hal_timer_stop(0);
    //
    // Stop timer 2
    //
    am_hal_timer_stop(2);
}

void start_sensors(void)
{
    //
    // Just in case host died without sending STOP last time
    //
    stop_sensors();

    //
    // Initialize Data Buffer Index
    //
    g_sendIdx = 0;

    //
    // Start timer 0
    //
    am_hal_timer_start(0);

    //
    // Start timer 2
    //
    am_hal_timer_start(2);
    g_iosState = AM_IOSTEST_SLAVE_STATE_NODATA;
}

void init_sensors(void)
{
    //
    // Set up timer 0.
    //
    am_hal_timer_config_t       Timer0Config;
    am_hal_timer_default_config_set(&Timer0Config);
    Timer0Config.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV4K;   //<! 96MHz / 4K = 24KHz
    Timer0Config.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    Timer0Config.ui32Compare0 = 24000 / SENSOR0_FREQ ;          //<! Sensor 0 Freq

    am_hal_timer_config(0, &Timer0Config);
    am_hal_timer_clear_stop(0);

    //
    // Set up timer 2.
    //
    am_hal_timer_config_t       Timer2Config;
    am_hal_timer_default_config_set(&Timer2Config);
    Timer2Config.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV4K;   //<! 96MHz / 4K = 24KHz
    Timer2Config.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    Timer2Config.ui32Compare0 = 24000 / SENSOR1_FREQ ;          //<! Sensor 1 Freq

    am_hal_timer_config(2, &Timer2Config);
    am_hal_timer_clear_stop(2);

    //
    // Clear the timer Interrupt
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE0));
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(2, AM_HAL_TIMER_COMPARE0));

    //
    // Enable the timer Interrupt.
    //
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE0));
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(2, AM_HAL_TIMER_COMPARE0));

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_SetPriority(TIMER0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_SetPriority(TIMER2_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(TIMER2_IRQn);
    am_hal_interrupt_master_enable();
}

static void ios_set_up(bool bSpi)
{
     //
     // Configure I2C interface
     //
     am_bsp_ios_pins_enable(0, AM_HAL_IOS_USE_I2C);

     //
     // Configure the IOS interface and LRAM structure.
     //
     am_hal_ios_initialize(0, &g_pIOSHandle);
     am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
     am_hal_ios_configure(g_pIOSHandle, &g_sIOSI2cConfig);

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
    am_hal_ios_interrupt_clear(g_pIOSHandle, AM_HAL_IOS_INT_ALL);
    am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_INT_ERR | AM_HAL_IOS_INT_FSIZE);
#ifdef TEST_IOS_XCMP_INT
    am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_XCMP_INT);
#endif

    //
    // Set the bit in the NVIC to accept access interrupts from the IO Slave.
    //
    NVIC_SetPriority(IOSLAVE_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(IOSLAVE_IRQn);

    //
    // Set up the IOSINT interrupt pin as generic output
    //
    am_hal_gpio_pinconfig(HANDSHAKE_PIN, g_AM_BSP_GPIO_OUTPUT);
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
}

void inform_host(void)
{
    uint32_t ui32Arg = AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK;

    //
    // Update FIFOCTR for host to read
    //
    am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_FIFO_UPDATE_CTR, NULL);

    //
    // Interrupt the host
    //
    am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_HOST_INTSET, &ui32Arg);
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_SET);
}

void am_ioslave_ios_isr(void)
{
    uint32_t ui32Status;
    uint8_t  *pui8Packet;
    uint32_t ui32UsedSpace = 0;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    am_hal_ios_interrupt_status_get(g_pIOSHandle, false, &ui32Status);

    am_hal_ios_interrupt_clear(g_pIOSHandle, ui32Status);
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

    if (ui32Status & AM_HAL_IOS_INT_FUNDFL)
    {
        am_util_stdio_printf("Hitting underflow for the requested IOS FIFO transfer\n");
        // We should never hit this case unless the threshold has beeen set
        // incorrect, or we are unable to handle the data rate
        // ERROR!
        //am_hal_debug_assert_msg(0,
        //    "Hitting underflow for the requested IOS FIFO transfer.");
    }

    if (ui32Status & AM_HAL_IOS_INT_ERR)
    {
        // We should never hit this case
        // ERROR!
        //am_hal_debug_assert_msg(0,
        //    "Hitting ERROR case.");
    }

    if (ui32Status & AM_HAL_IOS_INT_FSIZE)
    {
        //
        // Service the I2C slave FIFO if necessary.
        //
        am_hal_ios_interrupt_service(g_pIOSHandle, ui32Status);
    }

    if (ui32Status & AM_HAL_IOS_INT_XCMPWR)
    {
        //
        // Set up a pointer for writing 32-bit aligned packets through the IO slave
        // interface.
        //
        pui8Packet = (uint8_t *) am_hal_ios_pui8LRAM;
        switch ( pui8Packet[0] )
        {
            case AM_IOSTEST_CMD_START_DATA:
                //
                // Host wants to start data exchange
                // Start the Sensor Emulation
                //
                start_sensors();
                break;

            case AM_IOSTEST_CMD_STOP_DATA:
                //
                // Host no longer interested in data from us
                // Stop the Sensor emulation
                //
                stop_sensors();
                g_iosState = AM_IOSTEST_SLAVE_STATE_NODATA;
                break;

            case AM_IOSTEST_CMD_ACK_DATA:
                //
                // Host done reading the last block signalled
                // Check if any more data available
                //
                am_hal_ios_fifo_space_used(g_pIOSHandle, &ui32UsedSpace);
                if (ui32UsedSpace)
                {
                    g_iosState = AM_IOSTEST_SLAVE_STATE_DATA;
                    inform_host();
                }
                else
                {
                    g_iosState = AM_IOSTEST_SLAVE_STATE_NODATA;
                }
                break;

            default:
                break;
        }
    }
}

int main(void)
{
    int i;

    //
    // Enable the ITM print interface.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("IOS FIFO Example\n");

    //
    // Initialize Test Data
    //
    for (i = 0; i < AM_TEST_REF_BUF_SIZE; i++)
    {
        g_pui8TestBuf[i] = (i & 0xFF) ^ XOR_BYTE;
    }

    //
    // Initialize the Sensors
    //
    init_sensors();

    //
    // Enable the IOS. Choose the correct protocol based on USE_SPI
    //
    ios_set_up(USE_SPI);

    //
    // Enable interrupts so we can receive messages from the boot host.
    //
    am_hal_interrupt_master_enable();

    //
    // Loop forever.
    //
    while(1)
    {
        uint32_t numWritten = 0;
        uint32_t numWritten1 = 0;
        uint32_t chunk1;
        uint32_t ui32UsedSpace = 0;
        uint32_t ui32IntStatus = am_hal_interrupt_master_disable();
        if (g_bSensor0Data || g_bSensor1Data)
        {
            //
            // Enable the interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);

            if (g_bSensor0Data)
            {
                chunk1 = AM_TEST_REF_BUF_SIZE - g_sendIdx;
                if (chunk1 > SENSOR0_DATA_SIZE)
                {
                    am_hal_ios_fifo_write(g_pIOSHandle, &g_pui8TestBuf[g_sendIdx], SENSOR0_DATA_SIZE, &numWritten);
                }
                else
                {
                    am_hal_ios_fifo_write(g_pIOSHandle, &g_pui8TestBuf[g_sendIdx], chunk1, &numWritten);
                    if (numWritten == chunk1)
                    {
                        am_hal_ios_fifo_write(g_pIOSHandle, &g_pui8TestBuf[0], SENSOR0_DATA_SIZE - chunk1, &numWritten1);
                        numWritten += numWritten1;
                    }
                }

                g_sendIdx += numWritten;
                g_sendIdx %= AM_TEST_REF_BUF_SIZE;
                g_bSensor0Data = false;
            }
            if (g_bSensor1Data)
            {
                chunk1 = AM_TEST_REF_BUF_SIZE - g_sendIdx;
                if (chunk1 > SENSOR1_DATA_SIZE)
                {
                    am_hal_ios_fifo_write(g_pIOSHandle, &g_pui8TestBuf[g_sendIdx], SENSOR1_DATA_SIZE, &numWritten);
                }
                else
                {
                    am_hal_ios_fifo_write(g_pIOSHandle, &g_pui8TestBuf[g_sendIdx], chunk1, &numWritten);
                    if (numWritten == chunk1)
                    {
                        am_hal_ios_fifo_write(g_pIOSHandle, &g_pui8TestBuf[0], SENSOR1_DATA_SIZE - chunk1, &numWritten1);
                        numWritten += numWritten1;
                    }
                }

                g_sendIdx += numWritten;
                g_sendIdx %= AM_TEST_REF_BUF_SIZE;
                g_bSensor1Data = false;
            }

            //
            // If we were Idle - need to inform Host if there is new data
            //
            if (g_iosState == AM_IOSTEST_SLAVE_STATE_NODATA)
            {
                am_hal_ios_fifo_space_used(g_pIOSHandle, &ui32UsedSpace);
                if (ui32UsedSpace)
                {
                    g_iosState = AM_IOSTEST_SLAVE_STATE_DATA;
                    inform_host();
                }
            }
        }
        else
        {
            //
            // Go to Deep Sleep.
            //
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);

            //
            // Enable the interrupts
            //
            am_hal_interrupt_master_set(ui32IntStatus);
        }
    }
}

#endif
*/
