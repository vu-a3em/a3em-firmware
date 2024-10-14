// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "diskio.h"
#include "logging.h"
#include "storage.h"
#include "system.h"


// Private Peripheral Type Definitions ---------------------------------------------------------------------------------

typedef struct
{
   // Host Configuration Options
   am_hal_host_xfer_mode_e transfer_mode;
   am_hal_host_bus_width_e bus_width;
   am_hal_host_bus_voltage_e bus_voltage;
   am_hal_host_uhs_mode_e uhs_mode;
   uint32_t clock;

   // Device Configuration Options
   am_hal_card_type_e card_type;
   am_hal_card_pwr_ctrl_policy_e card_power_ctrl_policy;
   am_hal_host_event_cb_t callback;
   uint32_t sector_count;
} am_device_card_config_t;


// Static Global Variables ---------------------------------------------------------------------------------------------

static FATFS file_system;
static am_hal_card_t sd_card;
static am_hal_card_host_t *sd_card_host = NULL;
static am_device_card_config_t sd_card_config;
static volatile bool async_write_complete, async_read_complete, card_present;
static FIL current_file, log_file, timestamp_file, imu_file;
static volatile DSTATUS sd_disk_status;
static bool file_open, imu_file_open, is_wav_file;
static uint8_t work_buf[FF_MAX_SS];
static uint32_t data_size;
static char sd_card_path[4];


// Private Helper Functions --------------------------------------------------------------------------------------------

DSTATUS disk_initialize(BYTE)
{
   // Check if already initialized
   if (sd_disk_status != STA_NOINIT)
      return sd_disk_status;

   // Retrieve the SDHC card host instance
   sd_card_host = am_hal_get_card_host(AM_HAL_SDHC_CARD_HOST, true);
   if (!sd_card_host)
   {
      print("ERROR: No SD card host found!\n");
      return sd_disk_status;
   }

   // Verify that an SD card is present
   if (am_hal_card_host_find_card(sd_card_host, &sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: No SD card found!\n");
      return sd_disk_status;
   }

   // Initialize the SD card
   card_present = true;
   if (am_hal_card_init(&sd_card, sd_card_config.card_type, NULL, sd_card_config.card_power_ctrl_policy) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: SD card is not ready...\n");
      return sd_disk_status;
   }

   // Set the correct data transfer mode
   am_hal_card_host_set_xfer_mode(sd_card_host, sd_card_config.transfer_mode);

   // Register a callback function for asynchronous read, write, card-insert, and card-remove events
   if (sd_card_config.callback)
      am_hal_card_register_evt_callback(&sd_card, sd_card_config.callback);

   // Read the sector count from the SD card
   sd_card_config.sector_count = am_hal_sd_card_get_block_count(&sd_card);
   if (!sd_card_config.sector_count)
   {
      print("ERROR: Failed to read the SD card sector count\n");
      return sd_disk_status;
   }

   // Configure the SDIO host
   if (am_hal_card_cfg_set(&sd_card, sd_card_config.card_type, sd_card_config.bus_width, sd_card_config.clock, sd_card_config.bus_voltage, sd_card_config.uhs_mode) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: Failed to configure the SDIO host\n");
      return sd_disk_status;
   }

   // Power down the SDIO peripheral
   if (am_hal_card_pwrctrl_sleep(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: Failed to power down the SDIO peripheral\n");
      return sd_disk_status;
   }

   // Set the storage interrupt priority
   NVIC_SetPriority(SDIO_IRQn, STORAGE_INTERRUPT_PRIORITY);

   // Return an initialized disk status
   sd_disk_status &= ~STA_NOINIT;
   return sd_disk_status;
}

DSTATUS disk_status(BYTE)
{
   // Return the current disk status
   return sd_disk_status;
}

DRESULT disk_read(BYTE, BYTE *buff, LBA_t sector, UINT count)
{
   // Validate status and transfer parameters
   if (!count)
      return RES_PARERR;
   if (sd_disk_status & STA_NOINIT)
      return RES_NOTRDY;

   // Power on the SDIO peripheral
   if (am_hal_card_pwrctrl_wakeup(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: Failed to power on the SDIO peripheral\n");
      return RES_ERROR;
   }

   // Call the appropriate synchronous or asynchronous read API
   if (!sd_card_config.callback)
   {
      uint32_t status = am_hal_sd_card_block_read_sync(&sd_card, sector, count, (uint8_t*)buff);
      if ((status & 0xFFFF) != AM_HAL_STATUS_SUCCESS)
      {
         print("ERROR: Failed to call the synchronous read API...Number of bytes read = %d\n", status);
         return RES_ERROR;
      }
   }
   else
   {
      async_read_complete = false;
      uint32_t status = am_hal_sd_card_block_read_async(&sd_card, sector, count, (uint8_t *)buff);
      if (status != AM_HAL_STATUS_SUCCESS)
      {
         print("ERROR: Failed to call the asynchronous read API...Read Status = %d\n", status);
         NVIC_SetPriority(SDIO_IRQn, STORAGE_INTERRUPT_PRIORITY);
         return RES_ERROR;
      }

      // Wait until the asynchronous read is complete
      for (int i = 0; (i < 1001) && !async_read_complete; ++i)
      {
         am_util_delay_ms(1);
         if (i == 1000)
         {
            print("ERROR: Timed out reading from SD card\n");
            return RES_ERROR;
         }
      }
   }

   // Power down the SDIO peripheral
   if (am_hal_card_pwrctrl_sleep(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: Failed to power down the SDIO peripheral\n");
      return RES_ERROR;
   }
   return RES_OK;
}

DRESULT disk_write(BYTE, const BYTE *buff, LBA_t sector, UINT count)
{
   // Validate status and transfer parameters
   if (!count)
      return RES_PARERR;
   if (sd_disk_status & STA_NOINIT)
      return RES_NOTRDY;

   // Power on the SDIO peripheral
   if (am_hal_card_pwrctrl_wakeup(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: Failed to power on the SDIO peripheral\n");
      return RES_ERROR;
   }

   // Call the appropriate synchronous or asynchronous write API
   if (!sd_card_config.callback)
   {
      uint32_t status = am_hal_sd_card_block_write_sync(&sd_card, sector, count, (uint8_t*)buff);
      if ((status & 0xFFFF) != AM_HAL_STATUS_SUCCESS)
      {
         print("ERROR: Failed to call the synchronous write API...Number of bytes written = %d\n", status);
         return RES_ERROR;
      }
   }
   else
   {
      async_write_complete = false;
      uint32_t status = am_hal_sd_card_block_write_async(&sd_card, sector, count, (uint8_t*)buff);
      if (status != AM_HAL_STATUS_SUCCESS)
      {
         print("ERROR: Failed to call the asynchronous write API...Write Status = %d\n", status);
         NVIC_SetPriority(SDIO_IRQn, STORAGE_INTERRUPT_PRIORITY);
         return RES_ERROR;
      }

      // Wait until the asynchronous write is complete
      for (int i = 0; (i < 1001) && !async_write_complete; ++i)
      {
         am_util_delay_ms(1);
         if (i == 1000)
         {
            print("ERROR: Timed out writing to SD card\n");
            return RES_ERROR;
         }
      }
   }

   // Power down the SDIO peripheral
   if (am_hal_card_pwrctrl_sleep(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      print("ERROR: Failed to power down the SDIO peripheral\n");
      return RES_ERROR;
   }
   return RES_OK;
}

DRESULT disk_ioctl(BYTE, BYTE cmd, void *buff)
{
   // Verify that the disk has been initialized
   DRESULT res = RES_ERROR;
   if (sd_disk_status & STA_NOINIT)
      return RES_NOTRDY;

   // Carry out the requested disk ioctl function
   switch (cmd)
   {
      case CTRL_SYNC:           // Ensure there are no pending write processes
         res = RES_OK;
         break;
      case GET_SECTOR_COUNT:    // Return the number of sectors on the disk
         *(DWORD*)buff = sd_card_config.sector_count;
         res = RES_OK;
         break;
      case GET_BLOCK_SIZE:      // Return the block size in units of sectors
         *(DWORD*)buff = 1;
         res = RES_OK;
         break;
      default:                  // Unknown ioctl
         res = RES_PARERR;
   }
   return res;
}

void sd_card_event_callback(am_hal_host_evt_t *pEvt)
{
   // Handle the specific callback event
   am_hal_card_host_t *pHost = (am_hal_card_host_t*)pEvt->pCtx;
   switch (pEvt->eType)
   {
      case AM_HAL_EVT_XFER_COMPLETE:
         if (pHost->AsyncCmdData.dir == AM_HAL_DATA_DIR_READ)
            async_read_complete = true;
         else if (pHost->AsyncCmdData.dir == AM_HAL_DATA_DIR_WRITE)
            async_write_complete = true;
         break;
      case AM_HAL_EVT_CARD_NOT_PRESENT:
         card_present = false;
         break;
      case AM_HAL_EVT_CARD_PRESENT:
         card_present = true;
         break;
      default:
         break;
   }
}

void am_sdio_isr(void)
{
   // Service the SDIO interrupt
   static uint32_t status;
   am_hal_sdhc_intr_status_get(sd_card_host->pHandle, &status, false);
   am_hal_sdhc_intr_status_clear(sd_card_host->pHandle, status);
   am_hal_sdhc_interrupt_service(sd_card_host->pHandle, status);
}


// Public API Functions ------------------------------------------------------------------------------------------------

void storage_init(void)
{
   // Initialize all static local variables
   async_write_complete = async_read_complete = card_present = false;
   file_open = imu_file_open = is_wav_file = false;
   sd_disk_status = STA_NOINIT;

   // Set up the SD Card configuration structure
   sd_card_config = (am_device_card_config_t) {
      .transfer_mode = AM_HAL_HOST_XFER_ADMA,
      .clock = 48000000,
      .bus_width = AM_HAL_HOST_BUS_WIDTH_4,
      .bus_voltage = AM_HAL_HOST_BUS_VOLTAGE_3_3,
      .uhs_mode = AM_HAL_HOST_UHS_SDR50,
      .card_type = AM_HAL_CARD_TYPE_SDHC,
      .card_power_ctrl_policy = AM_HAL_CARD_PWR_CTRL_NONE, // TODO: Explore fully shutting off SD card using PIN_SD_CARD_ENABLE
      .callback = sd_card_event_callback,
      .sector_count = 0,
   };

   // Configure the relevant SDIO pins
   configASSERT0(am_hal_gpio_pinconfig(PIN_SD_CARD_CMD,  g_AM_BSP_GPIO_SDIO_CMD));
   configASSERT0(am_hal_gpio_pinconfig(PIN_SD_CARD_CLK,  g_AM_BSP_GPIO_SDIO_CLK));
   configASSERT0(am_hal_gpio_pinconfig(PIN_SD_CARD_DAT0, g_AM_BSP_GPIO_SDIO_DAT0));
   configASSERT0(am_hal_gpio_pinconfig(PIN_SD_CARD_DAT1, g_AM_BSP_GPIO_SDIO_DAT1));
   configASSERT0(am_hal_gpio_pinconfig(PIN_SD_CARD_DAT2, g_AM_BSP_GPIO_SDIO_DAT2));
   configASSERT0(am_hal_gpio_pinconfig(PIN_SD_CARD_DAT3, g_AM_BSP_GPIO_SDIO_DAT3));

   // Ensure that the SD card is initially enabled and powered on
   const am_hal_gpio_pincfg_t enable_pin_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   configASSERT0(am_hal_gpio_pinconfig(PIN_SD_CARD_ENABLE, enable_pin_config));
   am_hal_gpio_output_set(PIN_SD_CARD_ENABLE);

   // Mount and initialize the file system on the SD card
   FRESULT res = f_mount(&file_system, (TCHAR const*)sd_card_path, 0);
   if ((res == FR_NO_FILESYSTEM) && ((res = f_mkfs((TCHAR const*)sd_card_path, 0, work_buf, sizeof(work_buf))) != FR_OK))
      print("ERROR: Unable to create a file system on the SD card\n");
   else if (res != FR_OK)
      print("ERROR: Unable to mount the SD card file system\n");
}

void storage_deinit(void)
{
   // Close any open SD card files
   if (file_open)
      storage_close();
   if (imu_file_open)
      f_close(&imu_file);
   f_close(&timestamp_file);
   f_close(&log_file);
   file_open = imu_file_open = false;

   // De-initialize and power down the SD card host
   if (sd_card_host)
      sd_card_host->ops->deinit(sd_card_host->pHandle);
   am_hal_gpio_output_clear(PIN_SD_CARD_ENABLE);
   sd_card_host = NULL;

   // Disable the SDIO pins
   am_hal_gpio_pinconfig(PIN_SD_CARD_CMD,  am_hal_gpio_pincfg_default);
   am_hal_gpio_pinconfig(PIN_SD_CARD_CLK,  am_hal_gpio_pincfg_default);
   am_hal_gpio_pinconfig(PIN_SD_CARD_DAT0, am_hal_gpio_pincfg_default);
   am_hal_gpio_pinconfig(PIN_SD_CARD_DAT1, am_hal_gpio_pincfg_default);
   am_hal_gpio_pinconfig(PIN_SD_CARD_DAT2, am_hal_gpio_pincfg_default);
   am_hal_gpio_pinconfig(PIN_SD_CARD_DAT3, am_hal_gpio_pincfg_default);
}

void storage_setup_logs(void)
{
   // Ensure that a log file and timestamp file are present on the device
   if (f_open(&log_file, LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
      print("ERROR: Unable to open SD card log file for writing\n");
   if (f_open(&timestamp_file, LAST_TIMESTAMP_FILE_NAME, FA_OPEN_APPEND | FA_WRITE | FA_READ) != FR_OK)
      print("ERROR: Unable to open SD card timestamp file for writing\n");
   else
      f_lseek(&timestamp_file, 0);
}

bool storage_chdir(const char *directory)
{
   return (f_chdir(directory) == FR_OK);
}

bool storage_mkdir(const char *directory)
{
   // Attempt to create the specified directory if it does not exist
   static FILINFO file_info;
   return (f_stat(directory, &file_info) == FR_OK) ? true : (f_mkdir(directory) == FR_OK);
}

bool storage_open(const char *file_path, bool writeable)
{
   // Close an already-opened file
   if (file_open)
      storage_close();

   // Open the requested file
   data_size = 0;
   file_open = (f_open(&current_file, file_path, writeable ? (FA_CREATE_ALWAYS | FA_WRITE) : FA_READ) == FR_OK);
   return file_open;
}

bool storage_open_imu_file(void)
{
   // Open the IMU data file
   if (f_open(&imu_file, IMU_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
   {
      print("ERROR: Unable to open SD card IMU file for writing\n");
      return false;
   }
   imu_file_open = true;
   return true;
}

bool storage_write(const void *data, uint32_t data_len)
{
   // Write the requested data to the currently open file
   UINT data_written = 0;
   if (file_open && (f_write(&current_file, data, data_len, &data_written) == FR_OK) && (data_written == data_len))
   {
      data_size += data_len;
      return true;
   }
   return false;
}

bool storage_write_wav_header(uint32_t num_channels, uint32_t sample_rate_hz)
{
   // Write the WAV header segment
   uint32_t field = 36, bytes_per_sample = 2;
   is_wav_file = true;
   bool success = storage_write("RIFF", 4);
   success = success && storage_write(&field, 4);
   success = success && storage_write("WAVE", 4);
   success = success && storage_write("fmt ", 4);
   field = 16;
   success = success && storage_write(&field, 4);
   field = 1;
   success = success && storage_write(&field, 2);
   success = success && storage_write(&num_channels, 2);
   success = success && storage_write(&sample_rate_hz, 4);
   field = sample_rate_hz * num_channels * bytes_per_sample;
   success = success && storage_write(&field, 4);
   field = num_channels * bytes_per_sample;
   success = success && storage_write(&field, 2);
   field = 8 * bytes_per_sample;
   success = success && storage_write(&field, 2);
   success = success && storage_write("data", 4);
   success = success && storage_write(&field, 4);
   data_size = 0;
   return success;
}

void storage_write_log(const char *fmt, ...)
{
   // Write the requested data to the log file
   va_list args;
   va_start(args, fmt);
   f_vprintf(&log_file, fmt, args);
   va_end(args);
}

void storage_flush_log(void)
{
   // Flush the log file to ensure that contents are not lost upon power loss
   f_sync(&log_file);
}

bool storage_start_imu_data_stream(uint32_t timestamp, uint32_t sample_rate_hz)
{
   // Write an IMU data delimiter, sample rate, and timestamp
   UINT data_written = 0;
   static uint32_t imu_delimiter = IMU_DATA_DELIMITER;
   return (f_write(&imu_file, &imu_delimiter, sizeof(imu_delimiter), &data_written) == FR_OK) &&
          (f_write(&imu_file, &sample_rate_hz, sizeof(sample_rate_hz), &data_written) == FR_OK) &&
          (f_write(&imu_file, &timestamp, sizeof(timestamp), &data_written) == FR_OK);
}

bool storage_write_imu_data(const void *data, uint32_t data_len)
{
   // Store to the IMU data file
   UINT data_written = 0;
   return (f_write(&imu_file, data, data_len, &data_written) == FR_OK) && (data_written == data_len);
}

void storage_finish_imu_data_stream(void)
{
   // Flush the IMU data file to ensure contents are not lost upon power loss
   f_sync(&imu_file);
}

uint32_t storage_read(uint8_t *read_buffer, uint32_t buffer_len)
{
   // Read up to the requested number of bytes from the current file
   UINT data_read = 0;
   return (file_open && (f_read(&current_file, read_buffer, buffer_len, &data_read) == FR_OK)) ? data_read : 0;
}

int32_t storage_read_line(char *read_buffer, uint32_t buffer_len)
{
   // Read up to the requested number of bytes from the current file
   UINT data_read = 0;
   memset(read_buffer, 0, buffer_len);
   FSIZE_t read_start_location = file_open ? current_file.fptr : 0;
   if (file_open && (f_read(&current_file, read_buffer, buffer_len, &data_read) == FR_OK))
   {
      for (UINT i = 0; i < data_read; ++i)
         if (read_buffer[i] == '\n')
         {
            f_lseek(&current_file, read_start_location + i + 1);
            return (int32_t)(((i > 0) && (read_buffer[i-1] == '\r')) ? (i - 1) : i);
         }
   }
   return -1;
}

bool storage_set_last_known_timestamp(uint32_t timestamp)
{
   // Store to the persistent timestamp storage file
   UINT data_written = 0;
   bool success = (f_write(&timestamp_file, &timestamp, sizeof(timestamp), &data_written) == FR_OK) && (data_written == sizeof(timestamp));
   f_lseek(&timestamp_file, 0);
   return success;
}

uint32_t storage_get_last_known_timestamp(void)
{
   // Read from the timestamp file if it exists
   UINT data_read;
   uint32_t timestamp = 0;
   f_read(&timestamp_file, &timestamp, sizeof(timestamp), &data_read);
   f_lseek(&timestamp_file, 0);
   return timestamp;
}

void storage_delete(const char *file_path)
{
   // Attempt to unlink the existing file
   f_unlink(file_path);
}

void storage_close(void)
{
   // Close the currently open file
   if (file_open)
   {
      if (is_wav_file)
      {
         f_lseek(&current_file, 4);
         uint32_t field = 36 + data_size;
         storage_write(&field, 4);
         f_lseek(&current_file, 40);
         storage_write(&data_size, 4);

      }
      f_close(&current_file);
      file_open = is_wav_file = false;
   }
}
