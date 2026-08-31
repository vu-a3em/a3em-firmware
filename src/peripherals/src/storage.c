// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "datetime.h"
#include "diskio.h"
#include "imu.h"
#include "logging.h"
#include "ogg_writer.h"
#include "storage.h"
#include "system.h"


// Private Peripheral Type Definitions ---------------------------------------------------------------------------------

#define EARLY_LOG_MAX_BYTES     4096
#define EVENT_PAYLOAD_MAX_BYTES 384

_Static_assert(EVENT_PAYLOAD_MAX_BYTES < AM_PRINTF_BUFSIZE, "Event payload buffer must be smaller than the formatter's own buffer");

// Whether SD transfers wait on the SDIO completion interrupt instead of polling the controller
// The interrupt-driven path has been observed to lose completions on this hardware, which is unrecoverable
#define STORAGE_USE_ASYNC_TRANSFERS 0

#define ASYNC_POLL_INTERVAL_US    10     // How often the completion flag is re-read
#define ASYNC_POLLS_PER_MS        100    // 100 x 10 us = 1 ms between watchdog feeds
#define ASYNC_TRANSFER_TIMEOUT_MS 2000   // Well beyond the SD specification's worst-case block write

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
   uint32_t erase_block_sectors;
} am_device_card_config_t;


// Static Global Variables ---------------------------------------------------------------------------------------------

static FATFS file_system;
static am_hal_card_t sd_card;
static am_hal_card_host_t *sd_card_host = NULL;
static am_device_card_config_t sd_card_config;
static FIL current_file, log_file, imu_file, audio_file;
static char time_string[DATETIME_STAMP_LEN], audio_directory[MAX_DEVICE_LABEL_LEN + 40];
static char dev_fw_version[32], dev_hw_revision[8], dev_build_datetime[40], dev_uid[24], early_log[EARLY_LOG_MAX_BYTES];
static bool using_ogg, log_open, file_open, imu_file_open, audio_file_open, dev_info_captured, early_log_overflowed, sd_card_awake;
static uint32_t audio_directory_timestamp, data_size, opus_audio_buffer_idx, early_log_used;
static uint32_t measured_sample_rate_hz, wav_num_channels, sd_session_depth;
static volatile bool async_write_complete, async_read_complete, card_present;
static volatile uint8_t *imu_data_awaiting_storage;
static volatile uint32_t imu_storage_index;
static volatile DSTATUS sd_disk_status;
static uint8_t work_buf[FF_MAX_SS];
static ogg_data_packet_t ogg_packet;
static ogg_writer_t ogg_writer;
static storage_health_t health;

// The audio staging buffer and the IMU double buffer live in the shared SRAM group.
__attribute__((section(".shared"), aligned(32)))
static union { uint8_t opus[AUDIO_BUFFER_MAX_SIZE]; uint8_t wav[WAV_STAGING_BUFFER_SIZE]; } audio_staging;
#define opus_audio_buffer  (audio_staging.opus)
#define wav_staging_buffer (audio_staging.wav)
_Static_assert(sizeof(audio_staging) == WAV_STAGING_BUFFER_SIZE, "WAV staging is expected to be the larger overlay");
__attribute__((section(".shared"), aligned(32)))
static float imu_data_buffer[2*IMU_BUFFER_MAX_SAMPLES][3];
static uint32_t wav_staging_used;
static float (*imu_storage_buffer)[3];


// Private Helper Functions --------------------------------------------------------------------------------------------

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

   sd_card_config.erase_block_sectors = 1;

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
   sd_session_depth = 0;
   sd_card_awake = false;

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

static bool wait_for_async_transfer(volatile bool *complete_flag)
{
   for (uint32_t elapsed_ms = 0; elapsed_ms < ASYNC_TRANSFER_TIMEOUT_MS; ++elapsed_ms)
   {
      for (uint32_t i = 0; i < ASYNC_POLLS_PER_MS; ++i)
      {
         if (*complete_flag)
            return true;
         am_hal_delay_us(ASYNC_POLL_INTERVAL_US);
      }
      system_feed_watchdog();
   }
   return *complete_flag;
}

static bool sd_wake(void)
{
   if (sd_card_awake)
      return true;
   if (am_hal_card_pwrctrl_wakeup(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: Failed to power on the SDIO peripheral\n");
      return false;
   }
   sd_card_awake = true;
   return true;
}

static bool sd_sleep_if_idle(void)
{
   // Stay awake for as long as any session is open; the outermost session end powers the card down
   if (sd_session_depth || !sd_card_awake)
      return true;
   if (am_hal_card_pwrctrl_sleep(&sd_card) != AM_HAL_STATUS_SUCCESS)
   {
      printonly("ERROR: Failed to power down the SDIO peripheral\n");
      return false;
   }
   sd_card_awake = false;
   return true;
}

void storage_sd_session_begin(void)
{
   ++sd_session_depth;
}

void storage_sd_session_end(void)
{
   if (sd_session_depth && !--sd_session_depth)
      sd_sleep_if_idle();
}

DRESULT disk_read(BYTE, BYTE *buff, LBA_t sector, UINT count)
{
   // Validate status and transfer parameters
   if (!count)
      return RES_PARERR;
   if (sd_disk_status & STA_NOINIT)
      return RES_NOTRDY;

   // Power on the SDIO peripheral unless a session is already holding it awake
   if (!sd_wake())
      return RES_ERROR;

   // Call the appropriate synchronous or asynchronous read API
   DRESULT result = RES_OK;
   if (!STORAGE_USE_ASYNC_TRANSFERS)
   {
      uint32_t status = am_hal_sd_card_block_read_sync(&sd_card, sector, count, (uint8_t*)buff);
      if ((status & 0xFFFF) != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the synchronous read API...Number of bytes read = %d\n", status);
         result = RES_ERROR;
      }
   }
   else
   {
      async_read_complete = false;
      uint32_t status = am_hal_sd_card_block_read_async(&sd_card, sector, count, (uint8_t *)buff);
      if (status != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the asynchronous read API...Read Status = %d\n", status);
         result = RES_ERROR;
      }
      else if (!wait_for_async_transfer(&async_read_complete))
      {
         printonly("ERROR: Timed out reading from SD card\n");
         result = RES_ERROR;
      }
   }

   // Power down the SDIO peripheral unless a session is holding it awake for further transfers
   if (!sd_sleep_if_idle())
      result = RES_ERROR;
   return result;
}

DRESULT disk_write(BYTE, const BYTE *buff, LBA_t sector, UINT count)
{
   // Validate status and transfer parameters
   if (!count)
      return RES_PARERR;
   if (sd_disk_status & STA_NOINIT)
      return RES_NOTRDY;

   // Power on the SDIO peripheral unless a session is already holding it awake
   if (!sd_wake())
      return RES_ERROR;

   // Call the appropriate synchronous or asynchronous write API
   DRESULT result = RES_OK;
   if (!STORAGE_USE_ASYNC_TRANSFERS)
   {
      uint32_t status = am_hal_sd_card_block_write_sync(&sd_card, sector, count, (uint8_t*)buff);
      if ((status & 0xFFFF) != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the synchronous write API...Number of bytes written = %d\n", status);
         result = RES_ERROR;
      }
   }
   else
   {
      async_write_complete = false;
      uint32_t status = am_hal_sd_card_block_write_async(&sd_card, sector, count, (uint8_t*)buff);
      if (status != AM_HAL_STATUS_SUCCESS)
      {
         printonly("ERROR: Failed to call the asynchronous write API...Write Status = %d\n", status);
         result = RES_ERROR;
      }
      else if (!wait_for_async_transfer(&async_write_complete))
      {
         printonly("ERROR: Timed out writing to SD card\n");
         result = RES_ERROR;
      }
   }

   // Power down the SDIO peripheral unless a session is holding it awake for further transfers
   if (!sd_sleep_if_idle())
      result = RES_ERROR;
   return result;
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
      case GET_BLOCK_SIZE:      // Return the erase block size in units of sectors
         *(DWORD*)buff = sd_card_config.erase_block_sectors;
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

static void note_write_failure(void)
{
   ++health.write_failures;
   ++health.consecutive_failures;
}

static void note_write_success(void)
{
   health.consecutive_failures = 0;
}

static bool ensure_audio_directory(uint32_t activation_number, const char *device_label, uint32_t current_time)
{
   // File and directory names are UTC Unix timestamps rather than rendered dates
   snprintf(time_string, sizeof(time_string), "%010lu", (unsigned long)current_time);

   // Determine if it is time to create a new audio storage directory
   if (audio_directory_timestamp && ((current_time - audio_directory_timestamp) < NUM_SECONDS_PER_AUDIO_DIRECTORY) &&
       (current_time >= audio_directory_timestamp))
      return false;

   // Bucket boundaries fall on exact multiples of their period
   const uint32_t hour_bucket = (current_time / NUM_SECONDS_PER_AUDIO_DIRECTORY) * NUM_SECONDS_PER_AUDIO_DIRECTORY;
   const uint32_t day_bucket = (current_time / 86400u) * 86400u;

   // Create each level of the directory hierarchy in turn
   static FILINFO file_info;
   snprintf(audio_directory, sizeof(audio_directory), "%s", device_label);
   if ((f_stat(audio_directory, &file_info) != FR_OK) && (f_mkdir(audio_directory) != FR_OK))
      print("ERROR: Unable to create audio storage directory: %s\n", audio_directory);
   snprintf(audio_directory, sizeof(audio_directory), "%s/Activation_%04lu", device_label, (unsigned long)activation_number);
   if ((f_stat(audio_directory, &file_info) != FR_OK) && (f_mkdir(audio_directory) != FR_OK))
      print("ERROR: Unable to create audio storage directory: %s\n", audio_directory);
   snprintf(audio_directory + strlen(audio_directory), sizeof(audio_directory) - strlen(audio_directory), "/%010lu", (unsigned long)day_bucket);
   if ((f_stat(audio_directory, &file_info) != FR_OK) && (f_mkdir(audio_directory) != FR_OK))
      print("ERROR: Unable to create audio storage directory: %s\n", audio_directory);
   snprintf(audio_directory + strlen(audio_directory), sizeof(audio_directory) - strlen(audio_directory), "/%010lu", (unsigned long)hour_bucket);
   if ((f_stat(audio_directory, &file_info) != FR_OK) && (f_mkdir(audio_directory) != FR_OK))
      print("ERROR: Unable to create audio storage directory: %s\n", audio_directory);
   audio_directory_timestamp = hour_bucket;
   return true;
}

static void storage_flush_imu_data(void)
{
   // Flush any unwritten IMU data in the storage buffer to the SD card
   const uint8_t *imu_data = NULL;
   uint32_t imu_data_len = 0;
   AM_CRITICAL_BEGIN
   if (imu_storage_index)
   {
      imu_data = (const uint8_t*)imu_storage_buffer;
      imu_data_len = sizeof(float) * 3 * imu_storage_index;
      imu_storage_buffer = (imu_storage_buffer == imu_data_buffer) ? &imu_data_buffer[IMU_BUFFER_MAX_SAMPLES] : &imu_data_buffer[0];
      imu_storage_index = 0;
   }
   AM_CRITICAL_END
   if (imu_data && imu_file_open)
   {
      UINT data_written = 0;
      if ((f_write(&imu_file, imu_data, imu_data_len, &data_written) != FR_OK) || (data_written != imu_data_len))
         note_write_failure();
   }
}

// Push the staged audio to the card as a single transfer
static bool flush_wav_staging(void)
{
   const uint32_t length = wav_staging_used;
   wav_staging_used = 0;
   if (!length)
      return true;
   if (!audio_file_open)
      return false;

   // Hold the card awake across the whole FatFs cluster-writing batch
   storage_sd_session_begin();

   // Write any outstanding IMU data alongside the audio so the two files are touched together
   storage_handle_imu_data();
   UINT data_written = 0;
   const bool written_ok = (f_write(&audio_file, wav_staging_buffer, length, &data_written) == FR_OK) && (data_written == length);
   storage_sd_session_end();
   if (written_ok)
   {
      note_write_success();
      return true;
   }
   note_write_failure();
   return false;
}

static bool storage_write_wav_audio(const void *data, uint32_t data_len, bool is_last_packet)
{
   // Only continue with storage if an audio file is already open
   if (!audio_file_open)
      return false;

   // Accumulate WAV audio and write it out in large batches
   bool success = true;
   const uint8_t *source = (const uint8_t*)data;
   uint32_t remaining = data_len;
   while (remaining)
   {
      const uint32_t space = WAV_STAGING_BUFFER_SIZE - wav_staging_used;
      const uint32_t chunk = MIN(space, remaining);
      memcpy(wav_staging_buffer + wav_staging_used, source, chunk);
      wav_staging_used += chunk;
      source += chunk;
      remaining -= chunk;
      if (wav_staging_used == WAV_STAGING_BUFFER_SIZE)
         success = flush_wav_staging() && success;
   }

   // The payload length is accounted as accepted, not as flushed, because the WAV header is patched
   // at close and the buffer is always flushed before that happens
   data_size += data_len;

   // Never carry audio across a clip boundary
   if (is_last_packet)
      success = flush_wav_staging() && success;
   return success;
}

static bool storage_write_audio_raw(const void *data, uint32_t data_len)
{
   UINT data_written = 0;
   if (audio_file_open && (f_write(&audio_file, data, data_len, &data_written) == FR_OK) && (data_written == data_len))
   {
      note_write_success();
      return true;
   }
   if (audio_file_open)
      note_write_failure();
   return false;
}

static bool storage_write_ogg_opus_audio(const void *data, uint32_t num_samples, bool is_last_packet)
{
   // Initialize function-local variables
   static const opus_frame_t *result_begin, *result_end;
   bool success = true;

   // Only continue with storage if an audio file is already open
   if (!audio_file_open)
      return false;

   // Encode the audio data into Opus data frames
   opusenc_encode(data, num_samples, &result_begin, &result_end);

   // Encapsulate each Opus frame into an Ogg page and store
   for (const opus_frame_t *frame = result_begin; frame != result_end; frame = frame->next)
   {
      const uint8_t is_last = is_last_packet && (frame->next == result_end);
      ogg_add_packet(&ogg_writer, &ogg_packet, frame->encoded_data, frame->num_encoded_bytes, is_last);
      if (!ogg_packet.data_len)
         continue;

      // Copy as much of the Ogg packet as will fit into the staging buffer
      const uint32_t bytes_to_copy = MIN(sizeof(opus_audio_buffer) - opus_audio_buffer_idx, ogg_packet.data_len);
      const uint32_t bytes_remaining = ogg_packet.data_len - bytes_to_copy;
      memcpy(opus_audio_buffer + opus_audio_buffer_idx, ogg_packet.data, bytes_to_copy);
      opus_audio_buffer_idx += bytes_to_copy;

      // Flush the staging buffer to the card once it is full
      if (bytes_remaining)
      {
         // Hold the card awake for the audio and IMU writes together so the pair costs one wake instead of two
         storage_sd_session_begin();

         // Write any outstanding IMU data at the same time as the audio data
         storage_handle_imu_data();

         // On failure the buffer index is still reset and the overflow still copied in
         UINT data_written = 0;
         const bool opus_written_ok = (f_write(&audio_file, opus_audio_buffer, sizeof(opus_audio_buffer), &data_written) == FR_OK) && (data_written == sizeof(opus_audio_buffer));
         storage_sd_session_end();
         if (opus_written_ok)
         {
            data_size += sizeof(opus_audio_buffer);
            note_write_success();
         }
         else
         {
            note_write_failure();
            success = false;
         }
         memcpy(opus_audio_buffer, ogg_packet.data + bytes_to_copy, bytes_remaining);
         opus_audio_buffer_idx = bytes_remaining;
      }
   }
   return success;
}

void storage_set_measured_sample_rate(uint32_t sample_rate_hz)
{
   // The rate the hardware is actually delivering measured against the RTC
   measured_sample_rate_hz = sample_rate_hz;
}

static void storage_close_wav_audio(void)
{
   // Finalize and close the currently open audio file
   if (audio_file_open)
   {
      // Flush before patching the header, so the file position and the recorded length agree
      flush_wav_staging();
      const uint32_t payload_size = data_size;
      const uint32_t riff_size = 36 + payload_size;
      f_lseek(&audio_file, 4);
      storage_write_audio_raw(&riff_size, 4);

      // Patch in the measured rate if it is available, otherwise the predicted one
      if (measured_sample_rate_hz)
      {
         const uint32_t byte_rate = measured_sample_rate_hz * wav_num_channels * 2u;
         f_lseek(&audio_file, 24);
         storage_write_audio_raw(&measured_sample_rate_hz, 4);
         storage_write_audio_raw(&byte_rate, 4);
      }
      f_lseek(&audio_file, 40);
      storage_write_audio_raw(&payload_size, 4);
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
      if (bytes_remaining)
      {
         if ((f_write(&audio_file, opus_audio_buffer, sizeof(opus_audio_buffer), &data_written) == FR_OK) &&
             (data_written == sizeof(opus_audio_buffer)))
            data_size += sizeof(opus_audio_buffer);
         else
            note_write_failure();
         memcpy(opus_audio_buffer, ogg_packet.data + bytes_to_copy, bytes_remaining);
         opus_audio_buffer_idx = bytes_remaining;
      }
      if (opus_audio_buffer_idx)
      {
         data_size += opus_audio_buffer_idx;
         if ((f_write(&audio_file, opus_audio_buffer, opus_audio_buffer_idx, &data_written) != FR_OK) ||
             (data_written != opus_audio_buffer_idx))
            note_write_failure();
      }
      opus_audio_buffer_idx = 0;
      f_close(&audio_file);
      audio_file_open = false;
   }
}

static bool storage_write_wav_header(uint32_t num_channels, uint32_t sample_rate_hz)
{
   // Both size fields are placeholders until storage_close_wav_audio() patches them
   if (audio_file_open)
   {
      uint32_t field = 36;
      const uint32_t bytes_per_sample = 2;
      bool success = storage_write_audio_raw("RIFF", 4);
      success = success && storage_write_audio_raw(&field, 4);
      success = success && storage_write_audio_raw("WAVE", 4);
      success = success && storage_write_audio_raw("fmt ", 4);
      field = 16;
      success = success && storage_write_audio_raw(&field, 4);
      field = 1;
      success = success && storage_write_audio_raw(&field, 2);
      success = success && storage_write_audio_raw(&num_channels, 2);
      success = success && storage_write_audio_raw(&sample_rate_hz, 4);
      wav_num_channels = num_channels;
      field = sample_rate_hz * num_channels * bytes_per_sample;
      success = success && storage_write_audio_raw(&field, 4);
      field = num_channels * bytes_per_sample;
      success = success && storage_write_audio_raw(&field, 2);
      field = 8 * bytes_per_sample;
      success = success && storage_write_audio_raw(&field, 2);
      success = success && storage_write_audio_raw("data", 4);
      field = 0;
      success = success && storage_write_audio_raw(&field, 4);
      data_size = 0;
      if (!success)
         storage_close_audio();
   }
   return audio_file_open;
}

static bool storage_open_wav_file(uint32_t activation_number, const char *device_label, uint32_t num_channels, uint32_t sample_rate_hz, uint32_t current_time)
{
   // Close an already-opened audio file
   if (audio_file_open)
      storage_close_audio();

   // Determine if time to create a new audio storage directory, rotating the log if so
   if (ensure_audio_directory(activation_number, device_label, current_time))
      storage_setup_logs();

   // Open the requested file
   data_size = wav_staging_used = 0;
   static char file_name[FF_MAX_LFN] = { 0 };
   snprintf(file_name, sizeof(file_name), "%s/%s.wav", audio_directory, time_string);
   audio_file_open = (f_open(&audio_file, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK);

   return storage_write_wav_header(num_channels, sample_rate_hz);
}

static bool storage_open_ogg_opus_file(uint32_t activation_number, const char *device_label, uint32_t current_time)
{
   // Close an already-opened audio file
   if (audio_file_open)
      storage_close_ogg_opus_audio();
   opus_audio_buffer_idx = wav_staging_used = 0;

   // Determine if time to create a new audio storage directory, rotating the log if so
   if (ensure_audio_directory(activation_number, device_label, current_time))
      storage_setup_logs();

   // Open the requested file
   data_size = 0;
   static char file_name[FF_MAX_LFN] = { 0 };
   snprintf(file_name, sizeof(file_name), "%s/%s.opus", audio_directory, time_string);
   audio_file_open = (f_open(&audio_file, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK);

   // Reset the Ogg writer and write the Ogg header
   if (audio_file_open)
   {
      ogg_reset_writer(&ogg_writer, &ogg_packet);
      if (!storage_write_audio_raw(ogg_packet.data, ogg_packet.data_len))
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
   audio_directory_timestamp = opus_audio_buffer_idx = wav_staging_used = 0;
   imu_storage_buffer = imu_data_buffer;
   imu_data_awaiting_storage = NULL;
   imu_storage_index = 0;
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
      .erase_block_sectors = 1,
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
   FRESULT res = f_mount(&file_system, "", 1);
   if (res == FR_NO_FILESYSTEM)
   {
      const MKFS_PARM opts = { .fmt = FM_EXFAT, .n_fat = 0, .align = 0, .n_root = 0, .au_size = SD_CARD_ALLOCATION_UNIT_BYTES };
      if (f_mkfs("", &opts, work_buf, sizeof(work_buf)) != FR_OK)
         printonly("ERROR: Unable to create a file system on the SD card\n");
      else if ((res = f_mount(&file_system, "", 1)) != FR_OK)
         printonly("ERROR: Unable to mount the newly created SD card file system\n");
   }
   else if (res != FR_OK)
      printonly("ERROR: Unable to mount the SD card file system\n");

   // A mount failure means no usable card, regardless of what the presence detect reported
   if (res != FR_OK)
      card_present = false;
}

void storage_deinit(void)
{
   // Close any open SD card files
   storage_close_imu();
   storage_close_audio();
   storage_close();
   if (log_open)
   {
      f_sync(&log_file);
      f_close(&log_file);
   }
   log_open = file_open = imu_file_open = audio_file_open = false;
   audio_directory_timestamp = 0;

   // De-initialize and power down the SD card host
   if (sd_card_host)
   {
      am_hal_card_pwrctrl_wakeup(&sd_card);
      sd_card_host->ops->deinit(sd_card_host->pHandle);
   }
   sd_session_depth = 0;
   sd_card_awake = false;
   am_hal_gpio_output_clear(PIN_SD_CARD_ENABLE);
   sd_disk_status = STA_NOINIT;
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
   // Close any log file left open in a previous directory
   if (log_open)
   {
      f_sync(&log_file);
      f_close(&log_file);
      log_open = false;
   }

   // The deployment log lives inside the current timestamped audio directory so that each folder is self-contained
   if (!card_present || (audio_directory[0] == '\0'))
      return;
   static char log_path[FF_MAX_LFN] = { 0 };
   snprintf(log_path, sizeof(log_path), "%s/%s", audio_directory, LOG_FILE_NAME);
   log_open = (f_open(&log_file, log_path, FA_OPEN_APPEND | FA_WRITE) == FR_OK);
   if (!log_open)
   {
      printonly("ERROR: Unable to open SD card log file for writing\n");
      return;
   }

   // Flush anything that was logged before a directory existed
   if (early_log_used)
   {
      UINT data_written = 0;
      if (early_log_overflowed)
         f_write(&log_file, "[earlier log output was truncated]\n", 35, &data_written);
      f_write(&log_file, early_log, early_log_used, &data_written);
      f_sync(&log_file);
      early_log_used = 0;
      early_log_overflowed = false;
   }
}

void storage_flush_early_log(void)
{
   // Nothing can be written without a card or an early log
   if (log_open || !early_log_used || !card_present)
      return;

   // Rescue anything buffered before a log file existed
   FIL boot_text;
   if (f_open(&boot_text, UNACTIVATED_LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
      return;
   UINT data_written = 0;
   if (early_log_overflowed)
      f_write(&boot_text, "[earlier log output was truncated]\n", 35, &data_written);
   f_write(&boot_text, early_log, early_log_used, &data_written);
   f_sync(&boot_text);
   f_close(&boot_text);
   early_log_used = 0;
   early_log_overflowed = false;
}

bool storage_rotate_log(uint32_t activation_number, const char *device_label, uint32_t current_time)
{
   // Establish the directory for the supplied time and move the log into it if it changed
   if (ensure_audio_directory(activation_number, device_label, current_time) || !log_open)
   {
      storage_setup_logs();
      return true;
   }
   return false;
}

static uint8_t boot_record_checksum(const char *record, uint32_t len)
{
   // Simple additive checksum over a boot record's payload
   uint32_t sum = 0;
   for (uint32_t i = 0; i < len; ++i)
      sum += (uint8_t)record[i];
   return (uint8_t)(sum & 0xFF);
}

bool storage_open_named_wav_file(const char *file_path, uint32_t num_channels, uint32_t sample_rate_hz)
{
   // Open a WAV at an explicit path rather than in the timestamped deployment tree
   if (audio_file_open)
      storage_close_audio();
   using_ogg = false;
   data_size = wav_staging_used = 0;
   audio_file_open = (f_open(&audio_file, file_path, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK);
   return storage_write_wav_header(num_channels, sample_rate_hz);
}

uint32_t storage_get_free_space_mb(void)
{
   // Remaining space on the card in megabytes
   FATFS *fs = NULL;
   DWORD free_clusters = 0;
   if (!card_present || (f_getfree("", &free_clusters, &fs) != FR_OK) || !fs)
      return 0;
   const uint64_t free_sectors = (uint64_t)free_clusters * fs->csize;
   return (uint32_t)((free_sectors * FF_MAX_SS) / (1024u * 1024u));
}

uint32_t storage_get_allocation_unit_bytes(void)
{
   // Cluster size of the mounted volume, which is whatever the card was last formatted with
   if (!card_present || !file_system.csize)
      return 0;
   return (uint32_t)file_system.csize * FF_MAX_SS;
}

uint32_t storage_get_total_space_mb(void)
{
   // Total usable size of the volume in megabytes counting only data clusters
   if (!card_present || !file_system.n_fatent)
      return 0;
   const uint64_t total_sectors = (uint64_t)(file_system.n_fatent - 2) * file_system.csize;
   return (uint32_t)((total_sectors * FF_MAX_SS) / (1024u * 1024u));
}

bool storage_write_device_info(const char *fw_version, const char *hw_revision, const char *build_datetime,
                               const char *device_uid, uint32_t activation_number, uint32_t timestamp,
                               uint32_t battery_mv, const char *last_stop_reason, bool recovered)
{
   // A zero timestamp means the clock was not usable when this was called; keep whatever the file already holds
   if (!timestamp)
      timestamp = storage_get_recorded_timestamp();

   // A small machine-readable file at the card root, overwritten on every boot
   snprintf(dev_fw_version, sizeof(dev_fw_version), "%s", fw_version);
   snprintf(dev_hw_revision, sizeof(dev_hw_revision), "%s", hw_revision);
   snprintf(dev_build_datetime, sizeof(dev_build_datetime), "%s", build_datetime);
   snprintf(dev_uid, sizeof(dev_uid), "%s", device_uid);
   dev_info_captured = true;

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
   f_printf(&info_file, "LAST_STOP_REASON = \"%s\"\n", last_stop_reason);
   f_printf(&info_file, "LAST_STOP_RECOVERED = \"%s\"\n", recovered ? "True" : "False");
   f_printf(&info_file, "CARD_ALLOCATION_UNIT_BYTES = \"%u\"\n", storage_get_allocation_unit_bytes());
   f_printf(&info_file, "CARD_CAPACITY_MB = \"%u\"\n", storage_get_total_space_mb());
   f_printf(&info_file, "CARD_FREE_MB = \"%u\"\n", storage_get_free_space_mb());
   f_close(&info_file);
   return true;
}

bool storage_refresh_device_info(uint32_t activation_number, uint32_t timestamp,
                                 uint32_t battery_mv, const char *last_stop_reason, bool recovered)
{
   // Rewrite the device file with current values, reusing the identity fields captured at boot
   if (!dev_info_captured)
      return false;
   return storage_write_device_info(dev_fw_version, dev_hw_revision, dev_build_datetime, dev_uid,
                                    activation_number, timestamp, battery_mv, last_stop_reason, recovered);
}

static uint32_t largest_numeric_entry(const char *path, bool want_directory)
{
   // Directory and file names in the audio tree are decimal timestamps, so the newest is simply the
   // numerically largest. Nothing here assumes the entries come back in any particular order.
   DIR directory;
   static FILINFO entry;
   uint32_t largest = 0;
   if (f_opendir(&directory, path) != FR_OK)
      return 0;
   while ((f_readdir(&directory, &entry) == FR_OK) && entry.fname[0])
   {
      if (((entry.fattrib & AM_DIR) != 0) != want_directory)
         continue;
      char *end = NULL;
      const uint32_t value = (uint32_t)strtoul(entry.fname, &end, 10);
      if (!value || (end == entry.fname))
         continue;
      if (want_directory ? (*end != '\0') : (strcmp(end, ".wav") != 0))
         continue;
      if (value > largest)
         largest = value;
   }
   f_closedir(&directory);
   return largest;
}

uint32_t storage_get_latest_audio_timestamp(const char *device_label)
{
   // The newest recording is the freshest proof of when the device was still running
   if (!card_present || !device_label || !device_label[0])
      return 0;
   const uint32_t activation = storage_get_current_activation_number(device_label);
   if (!activation)
      return 0;

   static char path[FF_MAX_LFN];
   snprintf(path, sizeof(path), "%s/Activation_%04lu", device_label, (unsigned long)activation);
   const uint32_t day = largest_numeric_entry(path, true);
   if (!day)
      return 0;
   snprintf(path + strlen(path), sizeof(path) - strlen(path), "/%lu", (unsigned long)day);
   const uint32_t hour = largest_numeric_entry(path, true);
   if (!hour)
      return 0;
   snprintf(path + strlen(path), sizeof(path) - strlen(path), "/%lu", (unsigned long)hour);
   const uint32_t start_time = largest_numeric_entry(path, false);
   if (!start_time)
      return 0;
   snprintf(path + strlen(path), sizeof(path) - strlen(path), "/%lu.wav", (unsigned long)start_time);

   // Length comes from the file on disk rather than the header
   FIL clip;
   uint32_t duration_seconds = 0;
   if (f_open(&clip, path, FA_READ) == FR_OK)
   {
      uint8_t header[44];
      UINT bytes_read = 0;
      if ((f_read(&clip, header, sizeof(header), &bytes_read) == FR_OK) && (bytes_read == sizeof(header)))
      {
         const uint32_t byte_rate = (uint32_t)header[28] | ((uint32_t)header[29] << 8) |
                                    ((uint32_t)header[30] << 16) | ((uint32_t)header[31] << 24);
         const uint32_t payload = (f_size(&clip) > sizeof(header)) ? (uint32_t)(f_size(&clip) - sizeof(header)) : 0;
         if (byte_rate)
            duration_seconds = payload / byte_rate;
      }
      f_close(&clip);
   }
   return start_time + duration_seconds;
}

uint32_t storage_get_recorded_timestamp(void)
{
   // Read back the timestamp the device file was last written with
   if (!card_present)
      return 0;
   FIL info;
   if (f_open(&info, DEVICE_INFO_FILE_NAME, FA_READ) != FR_OK)
      return 0;
   char line[64];
   uint32_t timestamp = 0;
   while (f_gets(line, sizeof(line), &info))
   {
      const char *marker = "LAST_TIMESTAMP = \"";
      const size_t marker_len = sizeof("LAST_TIMESTAMP = \"") - 1;
      if (strncmp(line, marker, marker_len) == 0)
      {
         timestamp = (uint32_t)strtoul(line + marker_len, NULL, 10);
         break;
      }
   }
   f_close(&info);
   return timestamp;
}

void storage_write_boot_record(const char *reason, uint32_t epoch, uint32_t reset_count, uint32_t detail, uint32_t timestamp)
{
   // The boot log is a fixed-size, pre-allocated, circular file of fixed-length records. Each record
   // is written at a computed offset and carries its own sequence number and checksum, so:
   //   - a power loss during a write can only damage the one record being written
   //   - every other record remains independently parseable, in any order
   //   - repeated reboots overwrite old records instead of growing the file without bound
   // There is deliberately no header, because a header would itself be a single point of corruption.
   if (!card_present)
      return;

   // Assemble the record of exactly BOOT_LOG_RECORD_LEN bytes including a trailing newline
   char record[BOOT_LOG_RECORD_LEN + 1];
   const uint32_t sequence = (epoch * 1000u) + reset_count;
   datetime_t when;
   datetime_from_timestamp(timestamp, &when);
   char stamp[DATETIME_STAMP_LEN];
   datetime_format_stamp(stamp, sizeof(stamp), &when);
   int written = snprintf(record, sizeof(record), "%08lu %s %-14s %08lX",
                          (unsigned long)sequence, stamp, reason, (unsigned long)detail);
   if (written < 0)
      return;
   for (uint32_t i = (written < (int)BOOT_LOG_RECORD_LEN) ? (uint32_t)written : BOOT_LOG_RECORD_LEN; i < BOOT_LOG_RECORD_LEN; ++i)
      record[i] = ' ';
   record[BOOT_LOG_RECORD_LEN - 4] = ' ';
   const uint8_t checksum = boot_record_checksum(record, BOOT_LOG_RECORD_LEN - 4);
   record[BOOT_LOG_RECORD_LEN - 3] = "0123456789ABCDEF"[(checksum >> 4) & 0xF];
   record[BOOT_LOG_RECORD_LEN - 2] = "0123456789ABCDEF"[checksum & 0xF];
   record[BOOT_LOG_RECORD_LEN - 1] = '\n';

   // Open the file, pre-allocating it to its full size on first creation so that writing a record
   // never has to allocate a cluster and update the FAT, which is the part most exposed to a
   // power loss part way through
   FIL boot_file;
   if (f_open(&boot_file, BOOT_LOG_FILE_NAME, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
      return;
   if (f_size(&boot_file) < BOOT_LOG_FILE_SIZE)
   {
      UINT filled = 0;
      memset(work_buf, ' ', sizeof(work_buf));
      f_lseek(&boot_file, 0);
      for (uint32_t offset = 0; offset < BOOT_LOG_FILE_SIZE; offset += sizeof(work_buf))
         if ((f_write(&boot_file, work_buf, sizeof(work_buf), &filled) != FR_OK) || (filled != sizeof(work_buf)))
            break;
      f_sync(&boot_file);
   }

   // Write the single record at its slot and flush it before releasing the file
   UINT data_written = 0;
   uint32_t slot;
   if (sequence < BOOT_LOG_RESERVED_RECORDS)
      slot = sequence;
   else
      slot = BOOT_LOG_RESERVED_RECORDS + ((sequence - BOOT_LOG_RESERVED_RECORDS) % (BOOT_LOG_NUM_RECORDS - BOOT_LOG_RESERVED_RECORDS));
   if (f_lseek(&boot_file, slot * BOOT_LOG_RECORD_LEN) == FR_OK)
      f_write(&boot_file, record, BOOT_LOG_RECORD_LEN, &data_written);
   f_sync(&boot_file);
   f_close(&boot_file);
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
   // Close any file still open under the previous format BEFORE switching formats
   if (audio_file_open)
      storage_close_audio();

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
   AM_CRITICAL_BEGIN
   imu_storage_index = 0;
   imu_data_awaiting_storage = NULL;
   AM_CRITICAL_END

   // Determine if time to create a new audio storage directory, rotating the log if so
   if (ensure_audio_directory(activation_number, device_label, current_time))
      storage_setup_logs();

   // Open the requested file
   static char file_name[FF_MAX_LFN] = { 0 };
   snprintf(file_name, sizeof(file_name), "%s/%s.imu", audio_directory, time_string);
   imu_file_open = (f_open(&imu_file, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK);

   // Write the sample rate and starting timestamp
   UINT data_written = 0;
   return imu_file_open &&
          (f_write(&imu_file, &sample_rate_hz, sizeof(sample_rate_hz), &data_written) == FR_OK) &&
          (f_write(&imu_file, &current_time, sizeof(current_time), &data_written) == FR_OK);
}

uint32_t storage_get_current_activation_number(const char *device_label)
{
   // Search for the most recent activation directory
   FILINFO file_info;
   uint32_t activation_number = 0;
   char search_directory[MAX_DEVICE_LABEL_LEN + 24] = { 0 };
   while (activation_number < 9999)
   {
      snprintf(search_directory, sizeof(search_directory), "%s/Activation_%04lu", device_label, (unsigned long)(activation_number + 1));
      if (f_stat(search_directory, &file_info) != FR_OK)
         break;
      ++activation_number;
   }
   return activation_number;
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
         storage_write_wav_audio(data, data_len, is_last_packet);
}

bool storage_recover_audio(uint32_t activation_number, const char *device_label, uint32_t num_channels, uint32_t sample_rate_hz, uint32_t current_time)
{
   // First try closing and reopening the audio file in place
   print("WARNING: Audio write failed (%u consecutive, %u total) - attempting recovery\n", health.consecutive_failures, health.write_failures);
   for (uint32_t attempt = 0; attempt < STORAGE_MAX_REOPEN_ATTEMPTS; ++attempt)
   {
      if (audio_file_open)
      {
         f_close(&audio_file);
         audio_file_open = false;
      }
      if (storage_open_audio_file(activation_number, device_label, num_channels, sample_rate_hz, current_time, using_ogg))
      {
         ++health.reopen_recoveries;
         health.consecutive_failures = 0;
         print("INFO: Recovered by reopening the audio file\n");
         return true;
      }
   }

   // Then try unmounting and remounting the volume
   for (uint32_t attempt = 0; attempt < STORAGE_MAX_REMOUNT_ATTEMPTS; ++attempt)
   {
      storage_close_imu();
      storage_close();
      if (log_open)
      {
         f_close(&log_file);
         log_open = false;
      }
      audio_file_open = false;
      f_mount(NULL, "", 0);
      storage_deinit();
      storage_init();
      if (card_present)
      {
         audio_directory_timestamp = 0;
         if (storage_open_audio_file(activation_number, device_label, num_channels, sample_rate_hz, current_time, using_ogg))
         {
            ++health.remount_recoveries;
            health.consecutive_failures = 0;
            print("INFO: Recovered by remounting the SD card\n");
            return true;
         }
      }
   }

   // Neither recovery worked
   print("ERROR: SD card recovery failed - resetting device\n");
   return false;
}

void storage_get_health(storage_health_t *health_out)
{
   *health_out = health;
}

void storage_write_log(const char *fmt, ...)
{
   // Write the requested data to the log file
   if (log_open)
   {
      va_list args;
      va_start(args, fmt);
      f_vprintf(&log_file, fmt, args);
      va_end(args);
   }
   else if (early_log_used < sizeof(early_log))
   {
      // No log file yet, so buffer this boot-time output produced before a timestamped directory existed
      const uint32_t space = (uint32_t)sizeof(early_log) - early_log_used;
      va_list args;
      va_start(args, fmt);
      const int written = vsnprintf(early_log + early_log_used, space, fmt, args);
      va_end(args);
      if (written < 0)
         early_log_overflowed = true;
      else if ((uint32_t)written >= space)
      {
         early_log_used = sizeof(early_log) - 1;
         early_log_overflowed = true;
      }
      else
         early_log_used += (uint32_t)written;
   }
   else
      early_log_overflowed = true;
}

void storage_write_event(const char *code, const char *fmt, ...)
{
   // Emit a machine-readable event line alongside the human-readable log: EVT|<CODE>|key=value,key=value
   va_list args;
   if (log_open)
   {
      storage_write_log("EVT|%s|", code);
      va_start(args, fmt);
      f_vprintf(&log_file, fmt, args);
      va_end(args);
      storage_write_log("\n");
      return;
   }

   // No log file yet, use the early log buffer to store the event line
   static char payload[EVENT_PAYLOAD_MAX_BYTES];
   va_start(args, fmt);
   const uint32_t written = am_util_stdio_vsnprintf(payload, sizeof(payload), fmt, args);
   va_end(args);
   payload[(written < sizeof(payload)) ? written : (sizeof(payload) - 1)] = 0;
   storage_write_log("EVT|%s|%s\n", code, payload);
}

void storage_flush_log(void)
{
   // Flush the log file to ensure that contents are not lost upon power loss
   if (!log_open)
      return;
   storage_sd_session_begin();
   const FRESULT sync_result = f_sync(&log_file);
   storage_sd_session_end();
   if (sync_result != FR_OK)
   {
      // A failed sync means the log file object is no longer usable; reopen it in place
      f_close(&log_file);
      log_open = false;
      storage_setup_logs();
   }
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
      if (imu_data_awaiting_storage)  // The main loop has not yet written the previous buffer
         ++health.imu_buffers_dropped;
      else
      {
         imu_data_awaiting_storage = (uint8_t*)imu_storage_buffer;
         imu_storage_buffer = (imu_storage_buffer == imu_data_buffer) ? &imu_data_buffer[IMU_BUFFER_MAX_SAMPLES] : &imu_data_buffer[0];
      }
   }
}

void storage_handle_imu_data(void)
{
   // Check if the IMU data buffer is full and needs to be written to storage
   const uint8_t *pending = (const uint8_t*)imu_data_awaiting_storage;
   if (pending)
   {
      if (imu_file_open)
      {
         UINT data_written = 0;
         const uint32_t length = sizeof(float) * 3 * IMU_BUFFER_MAX_SAMPLES;
         storage_sd_session_begin();
         const bool imu_written_ok = (f_write(&imu_file, pending, length, &data_written) == FR_OK) && (data_written == length);
         storage_sd_session_end();
         if (!imu_written_ok)
            note_write_failure();
      }
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

void storage_delete(const char *file_path)
{
   // Attempt to unlink the existing file
   f_unlink(file_path);
}

void storage_close_audio(void)
{
   // Keep the card awake for all closing SD card writes
   storage_sd_session_begin();
   if (using_ogg)
      storage_close_ogg_opus_audio();
   else
      storage_close_wav_audio();
   storage_sd_session_end();
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
