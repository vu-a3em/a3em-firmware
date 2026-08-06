// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <time.h>
#include "diskio.h"
#include "imu.h"
#include "logging.h"
#include "ogg_writer.h"
#include "rtc.h"
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
static FIL current_file, log_file, imu_file, audio_file;
static char time_string[24], audio_directory[MAX_DEVICE_LABEL_LEN + 32];
static bool using_ogg, log_open, file_open, imu_file_open, audio_file_open;
static uint32_t audio_directory_timestamp, data_size, opus_audio_buffer_idx;
static uint8_t work_buf[FF_MAX_SS], opus_audio_buffer[AUDIO_BUFFER_MAX_SIZE];
static float imu_data_buffer[2*IMU_BUFFER_MAX_SAMPLES][3], (*imu_storage_buffer)[3];
static volatile bool async_write_complete, async_read_complete, card_present;
static volatile uint8_t *imu_data_awaiting_storage;
static volatile uint32_t imu_storage_index;
static volatile DSTATUS sd_disk_status;
static volatile uint32_t sd_read_errors, sd_write_errors, sd_timeout_errors;
static ogg_writer_t ogg_writer;
static ogg_data_packet_t ogg_packet;


// Private Helper Functions --------------------------------------------------------------------------------------------

// Mutex functions are only used if FF_FS_REENTRANT == 1 in ffconf.h

static volatile uint32_t ff_mutexes[2] = { 0 };

int ff_mutex_create(int _vol) { return 1; }
void ff_mutex_delete(int _vol) {}

int ff_mutex_take(int vol)
{
   int status, count = 0;
   do {
      while (__LDREXW(&ff_mutexes[vol]) != 0)
         __WFE();
      status = __STREXW(1, &ff_mutexes[vol]);
   } while (status != 0 && ++count < 100);
   __DMB();
   return 1;
}

void ff_mutex_give(int vol)
{
   __DMB();
   ff_mutexes[vol] = 0;
   __SEV();
}

DSTATUS disk_initialize(BYTE)
{
   // Check if already initialized
   if (sd_disk_status != STA_NOINIT)
      return sd_disk_status;

   // Retrieve the SDHC card host instance
   sd_card_host = am_hal_get_card_host(AM_HAL_SDHC_CARD_HOST, true);
   if (!sd_card_host)
   {
      printonly("ERROR: No SD card host found!\n");
      return sd_disk_status;
   }

   // Verify that an SD card is present
   if (am_hal_card_host_find_card(sd_card_host, &sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: No SD card found!\n");
      return sd_disk_status;
   }

   // Initialize the SD card
   if (am_hal_card_init(&sd_card, sd_card_config.card_type, NULL, sd_card_config.card_power_ctrl_policy) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: SD card is not ready...\n");
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
      printonly("ERROR: Failed to read the SD card sector count\n");
      return sd_disk_status;
   }

   // Configure the SDIO host
   if (am_hal_card_cfg_set(&sd_card, sd_card_config.card_type, sd_card_config.bus_width, sd_card_config.clock, sd_card_config.bus_voltage, sd_card_config.uhs_mode) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: Failed to configure the SDIO host\n");
      return sd_disk_status;
   }

   // Power down the SDIO peripheral
   if (am_hal_card_pwrctrl_sleep(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: Failed to power down the SDIO peripheral\n");
      return sd_disk_status;
   }

   // Set the storage interrupt priority
   NVIC_SetPriority(SDIO_IRQn, STORAGE_INTERRUPT_PRIORITY);

   // Return an initialized disk status
   card_present = true;
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
      printonly("ERROR: Failed to power on the SDIO peripheral\n");
      return RES_ERROR;
   }

   // Call the appropriate synchronous or asynchronous read API
   if (!sd_card_config.callback)
   {
      uint32_t status = am_hal_sd_card_block_read_sync(&sd_card, sector, count, (uint8_t*)buff);
      if ((status & 0xFFFF) != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the synchronous read API...Number of bytes read = %d\n", status);
         ++sd_read_errors;
         return RES_ERROR;
      }
   }
   else
   {
      async_read_complete = false;
      uint32_t status = am_hal_sd_card_block_read_async(&sd_card, sector, count, (uint8_t *)buff);
      if (status != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the asynchronous read API...Read Status = %d\n", status);
         NVIC_SetPriority(SDIO_IRQn, STORAGE_INTERRUPT_PRIORITY);
         ++sd_read_errors;
         return RES_ERROR;
      }

      // Wait until the asynchronous read is complete
      for (int i = 0; (i < 1001) && !async_read_complete; ++i)
      {
         am_util_delay_ms(1);
         if (i == 1000)
         {
            printonly("ERROR: Timed out reading from SD card\n");
            ++sd_timeout_errors;
            return RES_ERROR;
         }
      }
   }

   // Power down the SDIO peripheral
   if (am_hal_card_pwrctrl_sleep(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: Failed to power down the SDIO peripheral\n");
      return RES_ERROR;
   }
   return RES_OK;
}

DRESULT disk_write(BYTE, const BYTE *buff, LBA_t sector, UINT count)
{
   // Validate status and transfer parameters
   static uint8_t error_count = 0;
   if (!count)
      return RES_PARERR;
   if (sd_disk_status & STA_NOINIT)
      return RES_NOTRDY;
   if (++error_count > 2)
      system_reset();

   // Power on the SDIO peripheral
   if (am_hal_card_pwrctrl_wakeup(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: Failed to power on the SDIO peripheral\n");
      return RES_ERROR;
   }

   // Call the appropriate synchronous or asynchronous write API
   if (!sd_card_config.callback)
   {
      uint32_t status = am_hal_sd_card_block_write_sync(&sd_card, sector, count, (uint8_t*)buff);
      if ((status & 0xFFFF) != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the synchronous write API...Number of bytes written = %d\n", status);
         ++sd_write_errors;
         return RES_ERROR;
      }
   }
   else
   {
      async_write_complete = false;
      uint32_t status = am_hal_sd_card_block_write_async(&sd_card, sector, count, (uint8_t*)buff);
      if (status != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the asynchronous write API...Write Status = %d\n", status);
         NVIC_SetPriority(SDIO_IRQn, STORAGE_INTERRUPT_PRIORITY);
         ++sd_write_errors;
         return RES_ERROR;
      }

      // Wait until the asynchronous write is complete
      for (int i = 0; (i < 1001) && !async_write_complete; ++i)
      {
         am_util_delay_ms(1);
         if (i == 1000)
         {
            printonly("ERROR: Timed out writing to SD card\n");
            ++sd_timeout_errors;
            return RES_ERROR;
         }
      }
   }

   // Power down the SDIO peripheral
   if (am_hal_card_pwrctrl_sleep(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: Failed to power down the SDIO peripheral\n");
      return RES_ERROR;
   }
   error_count = 0;
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


// Private Helper Functions --------------------------------------------------------------------------------------------

static void storage_flush_imu_data(void)
{
   // Flush any unwritten IMU data in the storage buffer to the SD card
   __disable_irq();
   if (imu_storage_index)
   {
      UINT data_written = 0;
      const uint8_t* imu_data = (uint8_t*)imu_storage_buffer;
      const uint32_t imu_data_len = sizeof(float) * 3 * imu_storage_index;
      imu_storage_buffer = (imu_storage_buffer == imu_data_buffer) ? &imu_data_buffer[IMU_BUFFER_MAX_SAMPLES] : &imu_data_buffer[0];
      imu_storage_index = 0;
      __enable_irq();
      if (imu_file_open)
         f_write(&imu_file, imu_data, imu_data_len, &data_written);
   }
   else
      __enable_irq();
}

static bool storage_write_wav_audio(const void *data, uint32_t data_len)
{
   // Write any outstanding IMU data at the same time as the audio data
   storage_handle_imu_data();

   // Write the requested data to the currently open audio file
   UINT data_written = 0;
   if (audio_file_open && (f_write(&audio_file, data, data_len, &data_written) == FR_OK) && (data_written == data_len))
   {
      data_size += data_len;
      return true;
   }
   return false;
}

static bool storage_write_ogg_opus_audio(const void *data, uint32_t num_samples, bool is_last_packet)
{
   // Initialize function-local variables
   static const opus_frame_t *result_begin, *result_end;
   UINT data_written = 0;

   // Only continue with storage if an audio file is already open
   if (audio_file_open)
   {
      // Encode the audio data into Opus data frames
      opusenc_encode(data, num_samples, &result_begin, &result_end);

      // Encapsulate each Opus frame into an Ogg page and store
      for (const opus_frame_t *frame = result_begin; frame != result_end; frame = frame->next)
      {
         const uint8_t is_last = is_last_packet && (frame->next == result_end);
         ogg_add_packet(&ogg_writer, &ogg_packet, frame->encoded_data, frame->num_encoded_bytes, is_last);
         if (ogg_packet.data_len)
         {
            // Copy the Ogg packet to the storage buffer
            const uint32_t bytes_to_copy = MIN(sizeof(opus_audio_buffer) - opus_audio_buffer_idx, ogg_packet.data_len);
            const uint32_t bytes_remaining = ogg_packet.data_len - bytes_to_copy;
            memcpy(opus_audio_buffer + opus_audio_buffer_idx, ogg_packet.data, bytes_to_copy);
            opus_audio_buffer_idx += bytes_to_copy;

            // Write storage buffer to SD card if full
            if (bytes_remaining)
            {
               // Write any outstanding IMU data at the same time as the audio data
               storage_handle_imu_data();

               // Write the audio data
               if ((f_write(&audio_file, opus_audio_buffer, sizeof(opus_audio_buffer), &data_written) == FR_OK) && (data_written == sizeof(opus_audio_buffer)))
               {
                  memcpy(opus_audio_buffer, ogg_packet.data + bytes_to_copy, bytes_remaining);
                  opus_audio_buffer_idx = bytes_remaining;
                  data_size += sizeof(opus_audio_buffer);
               }
            }
         }
      }
      return true;
   }
   return false;
}

static void storage_close_wav_audio(void)
{
   // Finalize and close the currently open audio file
   if (audio_file_open)
   {
      // Capture the payload size BEFORE patching the header. storage_write_wav_audio()
      // adds every byte it writes to data_size, so routing the header patch through it
      // inflated the declared data chunk by the 4 bytes of the patch itself. Write the
      // header fields directly instead.
      UINT written = 0;
      const uint32_t payload_size = data_size;
      const uint32_t riff_size = 36 + payload_size;
      f_lseek(&audio_file, 4);
      f_write(&audio_file, &riff_size, 4, &written);
      f_lseek(&audio_file, 40);
      f_write(&audio_file, &payload_size, 4, &written);
      f_close(&audio_file);
      audio_file_open = false;
   }
}

static void storage_close_ogg_opus_audio(void)
{
   // Finalize and close the currently open audio file
   if (audio_file_open)
   {
      UINT data_written = 0;
      ogg_flush_page(&ogg_writer, &ogg_packet, 1);
      const uint32_t bytes_to_copy = MIN(sizeof(opus_audio_buffer) - opus_audio_buffer_idx, ogg_packet.data_len);
      const uint32_t bytes_remaining = ogg_packet.data_len - bytes_to_copy;
      memcpy(opus_audio_buffer + opus_audio_buffer_idx, ogg_packet.data, bytes_to_copy);
      opus_audio_buffer_idx += bytes_to_copy;
      if (bytes_remaining && (f_write(&audio_file, opus_audio_buffer, sizeof(opus_audio_buffer), &data_written) == FR_OK) && (data_written == sizeof(opus_audio_buffer)))
      {
         memcpy(opus_audio_buffer, ogg_packet.data + bytes_to_copy, bytes_remaining);
         opus_audio_buffer_idx = bytes_remaining;
         data_size += sizeof(opus_audio_buffer);
      }
      data_size += opus_audio_buffer_idx;
      f_write(&audio_file, opus_audio_buffer, opus_audio_buffer_idx, &data_written);
      f_close(&audio_file);
      audio_file_open = false;
   }
}

static void storage_rotate_log(void)
{
   // Move logging into the current audio directory. A single deployment-long log means
   // one bad cluster destroys the entire history; one log per 4-hour directory limits
   // the loss to that window. The dashboard stitches them back into one view.
   char log_path[sizeof(audio_directory) + sizeof(LOG_FILE_NAME) + 2];
   if (log_open)
   {
      f_sync(&log_file);
      f_close(&log_file);
      log_open = false;
   }
   snprintf(log_path, sizeof(log_path), "%s/%s", audio_directory, LOG_FILE_NAME);
   log_open = (f_open(&log_file, log_path, FA_OPEN_APPEND | FA_WRITE) == FR_OK);
   if (!log_open)
   {
      // Never lose logging outright: fall back to the log at the card root.
      log_open = (f_open(&log_file, LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) == FR_OK);
      if (log_open)
         print("ERROR: Unable to open log in %s, continuing in the root log\n", audio_directory);
   }
}

static void storage_update_audio_directory(uint32_t activation_number, const char *device_label, uint32_t current_time)
{
   // Build the timestamped file name used by every audio and IMU file
   const time_t timestamp = (time_t)current_time;
   struct tm *curr_time = gmtime(&timestamp);
   strftime(time_string, sizeof(time_string), "%F %H-%M-%S", curr_time);

   // Nothing further to do unless it is time to roll over to a new directory
   if ((current_time - audio_directory_timestamp) < NUM_SECONDS_PER_AUDIO_DIRECTORY)
      return;

   // Generate a new directory name from the current date and time
   static FILINFO file_info;
   curr_time->tm_min = curr_time->tm_sec = 0;
   curr_time->tm_hour = (curr_time->tm_hour / NUM_HOURS_PER_AUDIO_DIRECTORY) * NUM_HOURS_PER_AUDIO_DIRECTORY;
   size_t label_len = strlen(device_label);
   memset(audio_directory, 0, sizeof(audio_directory));
   strncpy(audio_directory, device_label, label_len + 1);
   snprintf(audio_directory + label_len, sizeof(audio_directory) - label_len, "/Activation_%04lu", activation_number);
   if ((f_stat(audio_directory, &file_info) != FR_OK) && (f_mkdir(audio_directory) != FR_OK))
      print("ERROR: Unable to create audio storage directory: %s\n", audio_directory);
   strftime(audio_directory + label_len + 16, sizeof(audio_directory) - label_len - 16, "/%F", curr_time);
   if ((f_stat(audio_directory, &file_info) != FR_OK) && (f_mkdir(audio_directory) != FR_OK))
      print("ERROR: Unable to create audio storage directory: %s\n", audio_directory);
   strftime(audio_directory + label_len + 16, sizeof(audio_directory) - label_len - 16, "/%F/%H", curr_time);
   if ((f_stat(audio_directory, &file_info) != FR_OK) && (f_mkdir(audio_directory) != FR_OK))
      print("ERROR: Unable to create audio storage directory: %s\n", audio_directory);
   audio_directory_timestamp = (uint32_t)mktime(curr_time);

   // Start a fresh log inside the directory just created
   storage_rotate_log();
}

static bool storage_open_wav_file(uint32_t activation_number, const char *device_label, uint32_t num_channels, uint32_t sample_rate_hz, uint32_t current_time)
{
   // Close an already-opened audio file
   if (audio_file_open)
      storage_close_audio();

   // Determine if time to create a new audio storage directory
   storage_update_audio_directory(activation_number, device_label, current_time);

   // Open the requested file
   data_size = 0;
   static char file_name[FF_MAX_LFN] = { 0 };
   snprintf(file_name, sizeof(file_name), "%s/%s.wav", audio_directory, time_string);
   audio_file_open = (f_open(&audio_file, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK);

   // Write the WAV header segment
   if (audio_file_open)
   {
      uint32_t field = 36, bytes_per_sample = 2;
      bool success = storage_write_wav_audio("RIFF", 4);
      success = success && storage_write_wav_audio(&field, 4);
      success = success && storage_write_wav_audio("WAVE", 4);
      success = success && storage_write_wav_audio("fmt ", 4);
      field = 16;
      success = success && storage_write_wav_audio(&field, 4);
      field = 1;
      success = success && storage_write_wav_audio(&field, 2);
      success = success && storage_write_wav_audio(&num_channels, 2);
      success = success && storage_write_wav_audio(&sample_rate_hz, 4);
      field = sample_rate_hz * num_channels * bytes_per_sample;
      success = success && storage_write_wav_audio(&field, 4);
      field = num_channels * bytes_per_sample;
      success = success && storage_write_wav_audio(&field, 2);
      field = 8 * bytes_per_sample;
      success = success && storage_write_wav_audio(&field, 2);
      success = success && storage_write_wav_audio("data", 4);
      success = success && storage_write_wav_audio(&field, 4);
      data_size = 0;
      if (!success)
         storage_close_audio();
   }
   return audio_file_open;
}

static bool storage_open_ogg_opus_file(uint32_t activation_number, const char *device_label, uint32_t current_time)
{
   // Close an already-opened audio file
   if (audio_file_open)
      storage_close_ogg_opus_audio();
   opus_audio_buffer_idx = 0;

   // Determine if time to create a new audio storage directory
   storage_update_audio_directory(activation_number, device_label, current_time);

   // Open the requested file
   data_size = 0;
   static char file_name[FF_MAX_LFN] = { 0 };
   snprintf(file_name, sizeof(file_name), "%s/%s.opus", audio_directory, time_string);
   audio_file_open = (f_open(&audio_file, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK);

   // Reset the Ogg writer and write the Ogg header
   if (audio_file_open)
   {
      ogg_reset_writer(&ogg_writer, &ogg_packet);
      if (!storage_write_wav_audio(ogg_packet.data, ogg_packet.data_len))
         storage_close_ogg_opus_audio();
   }
   return audio_file_open;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void storage_init(void)
{
   // Initialize all static local variables
   memset(time_string, 0, sizeof(time_string));
   memset(audio_directory, 0, sizeof(audio_directory));
   async_write_complete = async_read_complete = card_present = false;
   log_open = file_open = imu_file_open = audio_file_open = false;
   imu_storage_buffer = imu_data_buffer;
   imu_data_awaiting_storage = NULL;
   audio_directory_timestamp = 0;
   sd_disk_status = STA_NOINIT;

   // Set up the SD Card configuration structure
   sd_card_config = (am_device_card_config_t) {
      .transfer_mode = AM_HAL_HOST_XFER_ADMA,
      .clock = 48000000,
      .bus_width = AM_HAL_HOST_BUS_WIDTH_4,
      .bus_voltage = AM_HAL_HOST_BUS_VOLTAGE_3_3,
      .uhs_mode = AM_HAL_HOST_UHS_SDR50,
      .card_type = AM_HAL_CARD_TYPE_SDHC,
      .card_power_ctrl_policy = AM_HAL_CARD_PWR_CTRL_SDHC_OFF,
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
   char sd_card_path[4];
   const MKFS_PARM opts = { .fmt = FM_EXFAT, .n_fat = 0, .align = 0, .n_root = 0, .au_size = 4096 };
   FRESULT res = f_mount(&file_system, (TCHAR const*)sd_card_path, 1);
   if ((res == FR_NO_FILESYSTEM) && ((res = f_mkfs((TCHAR const*)sd_card_path, &opts, work_buf, sizeof(work_buf))) != FR_OK))
      print("ERROR: Unable to create a file system on the SD card\n");
   else if (res != FR_OK)
      print("ERROR: Unable to mount the SD card file system\n");
}

void storage_deinit(void)
{
   // Close any open SD card files
   storage_close_imu();
   storage_close_audio();
   storage_close();
   if (log_open)
      f_close(&log_file);
   log_open = file_open = imu_file_open = audio_file_open = false;
   audio_directory_timestamp = 0;

   // De-initialize and power down the SD card host
   if (sd_card_host)
   {
      am_hal_card_pwrctrl_wakeup(&sd_card);
      sd_card_host->ops->deinit(sd_card_host->pHandle);
   }
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
   // Ensure that a log file is present on the device
   if (card_present && !log_open)
      log_open = (f_open(&log_file, LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) == FR_OK);
   if (!log_open)
      print("ERROR: Unable to open SD card log file for writing\n");
}

bool storage_sd_card_error(void)
{
   return !card_present;
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
   file_open = (f_open(&current_file, file_path, writeable ? (FA_CREATE_ALWAYS | FA_WRITE) : FA_READ) == FR_OK);
   return file_open;
}

bool storage_open_audio_file(uint32_t activation_number, const char *device_label, uint32_t num_channels, uint32_t sample_rate_hz, uint32_t current_time, bool use_ogg)
{
   // Call the appropriate open-file function
   using_ogg = use_ogg;
   return using_ogg ?
         storage_open_ogg_opus_file(activation_number, device_label, current_time) :
         storage_open_wav_file(activation_number, device_label, num_channels, sample_rate_hz, current_time);
}

bool storage_open_imu_file(uint32_t activation_number, const char *device_label, uint32_t current_time, uint32_t sample_rate_hz)
{
   // Close an already-opened IMU file
   if (imu_file_open)
      storage_close_imu();

   // Reset the IMU storage buffering details
   imu_storage_index = 0;
   imu_data_awaiting_storage = NULL;

   // Determine if time to create a new audio storage directory
   storage_update_audio_directory(activation_number, device_label, current_time);

   // Open the requested file
   static char file_name[FF_MAX_LFN] = { 0 };
   snprintf(file_name, sizeof(file_name), "%s/%s.imu", audio_directory, time_string);
   imu_file_open = (f_open(&imu_file, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK);

   // Write the sample rate and starting timestamp.
   //
   // The timestamp is written as an explicit uint32 rather than a time_t. sizeof(time_t)
   // is 8 on this toolchain, which made the header 12 bytes with a always-zero high
   // word, and left every reader having to know that. A fixed 8-byte header of two
   // uint32s is unambiguous and matches what the file format was always meant to be.
   UINT data_written = 0;
   const uint32_t start_timestamp = current_time;
   return imu_file_open &&
          (f_write(&imu_file, &sample_rate_hz, sizeof(sample_rate_hz), &data_written) == FR_OK) &&
          (f_write(&imu_file, &start_timestamp, sizeof(start_timestamp), &data_written) == FR_OK);
}

uint32_t storage_get_current_activation_number(const char *device_label)
{
   // Search for the most recent activation directory
   FILINFO file_info;
   bool directory_found = true;
   uint32_t activation_number = 0;
   char audio_directory[MAX_DEVICE_LABEL_LEN + 24] = { 0 };
   while (directory_found)
   {
      snprintf(audio_directory, sizeof(audio_directory), "%s/Activation_%04lu", device_label, ++activation_number);
      directory_found = (f_stat(audio_directory, &file_info) == FR_OK);
   }
   return activation_number - 1;
}

bool storage_write(const void *data, uint32_t data_len)
{
   // Write the requested data to the currently open file
   UINT data_written = 0;
   return file_open && (f_write(&current_file, data, data_len, &data_written) == FR_OK) && (data_written == data_len);
}

bool storage_write_audio(const void *data, uint32_t data_len, bool is_last_packet)
{
   // Always sync the IMU data with an attempted audio write
   imu_drain_fifo();

   // Call the appropriate audio writing function
   return using_ogg ?
         storage_write_ogg_opus_audio(data, data_len / sizeof(int16_t), is_last_packet) :
         storage_write_wav_audio(data, data_len);
}

void storage_write_log(const char *fmt, ...)
{
   // Write the requested data to the log file, prefixing a UTC timestamp at the start
   // of every line. A single logical line is often built from several print() calls
   // (the device UID, for example), so the prefix is emitted only when the previous
   // write ended a line. Tracking that from the format string is exact here because
   // every newline in this codebase is a literal in the format, never an argument.
   if (log_open)
   {
      static bool at_line_start = true;
      const size_t fmt_len = strlen(fmt);

      if (at_line_start)
      {
         // Before the RTC holds a real time, mark the line rather than logging a
         // misleading epoch, so the dashboard can tell "no clock yet" from a real time.
         if (rtc_is_valid())
            f_printf(&log_file, "[%u] ", rtc_get_timestamp());
         else
            f_printf(&log_file, "[----------] ");
      }

      va_list args;
      va_start(args, fmt);
      f_vprintf(&log_file, fmt, args);
      va_end(args);

      at_line_start = fmt_len && (fmt[fmt_len - 1] == '\n');
   }
}

void storage_flush_log(void)
{
   // Flush the log file to ensure that contents are not lost upon power loss
   if (log_open)
      f_sync(&log_file);
}

void storage_get_error_counts(uint32_t *read_errors, uint32_t *write_errors, uint32_t *timeouts)
{
   // Report SD card failures accumulated at the disk layer.
   //
   // disk_read() and disk_write() sit BELOW FatFs, so they cannot write to the log
   // themselves without re-entering it. They count instead, and a caller above the
   // filesystem reports the totals. Corrupt files and partial data loss are the most
   // common field failure, and these counters are the only on-card evidence of them.
   if (read_errors)
      *read_errors = sd_read_errors;
   if (write_errors)
      *write_errors = sd_write_errors;
   if (timeouts)
      *timeouts = sd_timeout_errors;
}

uint32_t storage_get_free_space_mb(void)
{
   // Remaining space on the card, in megabytes.
   //
   // Logging this periodically is what turns "the card filled on 12 May" into a
   // statement of fact rather than an inference from file sizes, and it is the best
   // available calibration signal for the dashboard's storage forecast.
   FATFS *fs = NULL;
   DWORD free_clusters = 0;
   if (!card_present || (f_getfree("", &free_clusters, &fs) != FR_OK) || !fs)
      return 0;
   const uint64_t free_sectors = (uint64_t)free_clusters * fs->csize;
   return (uint32_t)((free_sectors * FF_MAX_SS) / (1024ULL * 1024ULL));
}

bool storage_write_device_info(const char *fw_version, const char *hw_revision, const char *build_datetime,
                               const char *device_uid, uint32_t activation_number, uint32_t timestamp,
                               uint32_t battery_mv, const char *last_deactivation_reason)
{
   // Write a small machine-readable device file at the card root, overwritten on every
   // boot. The dashboard reads it to identify the device by its hardware UID, select
   // the matching firmware capability profile, and show last-known state -- none of
   // which should require parsing a multi-megabyte log.
   //
   // Same KEY = "value" grammar as the configuration file, so it reuses the parser.
   FIL info_file;
   if (!card_present || (f_open(&info_file, DEVICE_INFO_FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK))
      return false;
   f_printf(&info_file, "FW_VERSION = \"%s\"\n", fw_version);
   f_printf(&info_file, "HW_REVISION = \"%s\"\n", hw_revision);
   f_printf(&info_file, "BUILD_DATETIME = \"%s\"\n", build_datetime);
   f_printf(&info_file, "DEVICE_UID = \"%s\"\n", device_uid);
   f_printf(&info_file, "ACTIVATION_NUMBER = \"%u\"\n", activation_number);
   f_printf(&info_file, "LAST_TIMESTAMP = \"%u\"\n", timestamp);
   f_printf(&info_file, "LAST_BATTERY_MV = \"%u\"\n", battery_mv);
   f_printf(&info_file, "LAST_DEACTIVATION_REASON = \"%s\"\n", last_deactivation_reason);
   f_close(&info_file);
   return true;
}

void storage_write_imu_data(float accel_x_mg, float accel_y_mg, float accel_z_mg)
{
   // Store IMU data into storage buffer
   imu_storage_buffer[imu_storage_index][0] = accel_x_mg;
   imu_storage_buffer[imu_storage_index][1] = accel_y_mg;
   imu_storage_buffer[imu_storage_index][2] = accel_z_mg;

   // Set storage flag for the main thread if buffer is full
   if (++imu_storage_index == IMU_BUFFER_MAX_SAMPLES)
   {
      imu_storage_index = 0;
      imu_data_awaiting_storage = (uint8_t*)imu_storage_buffer;
      imu_storage_buffer = (imu_storage_buffer == imu_data_buffer) ? &imu_data_buffer[IMU_BUFFER_MAX_SAMPLES] : &imu_data_buffer[0];
   }
}

void storage_handle_imu_data(void)
{
   // Check if the IMU data buffer is full and needs to be written to storage
   if (imu_data_awaiting_storage)
   {
      UINT data_written = 0;
      if (imu_file_open)
         f_write(&imu_file, (uint8_t*)imu_data_awaiting_storage, sizeof(float) * 3 * IMU_BUFFER_MAX_SAMPLES, &data_written);
      imu_data_awaiting_storage = NULL;
   }
}

uint32_t storage_read(uint8_t *read_buffer, uint32_t buffer_len)
{
   // Read up to the requested number of bytes from the current file
   UINT data_read = 0;
   return (file_open && (f_read(&current_file, read_buffer, buffer_len, &data_read) == FR_OK)) ? data_read : 0;
}

int32_t storage_read_line(char *read_buffer, uint32_t buffer_len)
{
   // Read one line from the current file.
   //
   // Returns the line length, 0 for a line that could not be represented (skipped),
   // or -1 only at genuine end of file.
   //
   // Previously ANY line longer than the buffer also returned -1, which ended the
   // caller's read loop and silently discarded the entire rest of the file. A single
   // over-long line therefore produced a partial configuration that looked complete.
   // Now such a line is reported and skipped, and parsing continues.
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

      // Nothing read at all means end of file
      if (!data_read)
         return -1;

      // A full buffer with no newline is an over-long line. Skip to the start of the
      // next line so the remainder of the file is still parsed.
      if (data_read == buffer_len)
      {
         print("ERROR: Configuration line exceeds %u bytes and was skipped\n", (unsigned)buffer_len);
         char skip_buffer[64];
         UINT skipped = 0;
         while (f_read(&current_file, skip_buffer, sizeof(skip_buffer), &skipped) == FR_OK && skipped)
            for (UINT i = 0; i < skipped; ++i)
               if (skip_buffer[i] == '\n')
               {
                  f_lseek(&current_file, current_file.fptr - (skipped - i - 1));
                  memset(read_buffer, 0, buffer_len);
                  return 0;
               }
         return -1;   // reached end of file while skipping
      }

      // Trailing content with no final newline. Return it rather than dropping it,
      // then let the next call report end of file.
      print("WARNING: Configuration file does not end with a newline\n");
      return (int32_t)data_read;
   }
   return -1;
}

void storage_delete(const char *file_path)
{
   // Attempt to unlink the existing file
   f_unlink(file_path);
}

void storage_close_audio(void)
{
   // Call the appropriate close-audio function
   if (using_ogg)
      storage_close_ogg_opus_audio();
   else
      storage_close_wav_audio();
}

void storage_close_imu(void)
{
   // Check if an IMU file is currently open
   if (imu_file_open)
   {
      // Drain and write any outstanding IMU data
      imu_drain_fifo();
      storage_handle_imu_data();
      storage_flush_imu_data();

      // Close the currently open IMU file
      f_close(&imu_file);
      imu_file_open = false;
   }
}

void storage_close(void)
{
   // Close the currently open file
   if (file_open)
   {
      f_close(&current_file);
      file_open = false;
   }
}
