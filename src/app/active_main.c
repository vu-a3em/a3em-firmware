#include "audio.h"
#include "battery.h"
#include "comparator.h"
#include "henrik.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "mram.h"
#include "rtc.h"
#include "silence.h"
#include "storage.h"
#include "system.h"
#include "vhf.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static volatile uint8_t *imu_data_awaiting_storage;
static volatile uint32_t num_clips_stored, imu_storage_index;
static volatile bool *device_active, phase_ended, audio_timer_triggered, in_motion, new_imu_stream, validation_time;
static uint32_t phase_end_timestamp, vhf_enable_timestamp, led_active_seconds, imu_sampling_rate_hz, activation_number;
static float imu_data_buffer[2*IMU_BUFFER_NUM_SAMPLES][3], (*imu_storage_buffer)[3];
static float last_lat = 0.0, last_lon = 0.0, last_height = 0.0;
static am_hal_timer_config_t audio_processing_timer_config;
static bool record_imu_with_audio, use_silence_filter;
static char device_label[MAX_DEVICE_LABEL_LEN];
static uint8_t imu_degrees_of_freedom;


// Private Helper Functions --------------------------------------------------------------------------------------------

static uint32_t seconds_until_next_scheduled_recording(uint32_t num_schedules, start_end_time_t *schedule, uint32_t current_seconds_of_day)
{
   // Return the number of seconds until the next scheduling recording phase or 0 if already in an active phase
   uint32_t current_index = 0;
   while ((current_index < num_schedules) && (current_seconds_of_day >= schedule[current_index].end_time))
      ++current_index;
   if (current_index < num_schedules)
      return (schedule[current_index].start_time <= current_seconds_of_day) ? 0 : (schedule[current_index].start_time - current_seconds_of_day);
   return num_schedules ? (86400 - current_seconds_of_day + schedule[0].start_time) : 0;
}

static void validate_device_settings(uint32_t current_timestamp)
{
   // Check if the battery voltage is too low to continue
   const battery_result_t battery_details = battery_monitor_get_details();
   phase_ended = (battery_details.millivolts <= config_get_battery_mV_low());

   // Check if the current phase has ended or if it is time to activate the VHF radio
   static uint32_t previous_timestamp = 0;
   uint32_t wakeup_timestamp = MIN(current_timestamp + MIN_LOG_DATA_INTERVAL_SECONDS, phase_end_timestamp);
   if (current_timestamp >= phase_end_timestamp)
      phase_ended = true;
   else if (vhf_enable_timestamp)
   {
      if (current_timestamp >= vhf_enable_timestamp)
      {
         vhf_activate();
         vhf_enable_timestamp = 0;
      }
      else
         wakeup_timestamp = MIN(wakeup_timestamp, vhf_enable_timestamp);
   }

   // Check if it is time to deactivate the LEDs
   if (led_active_seconds)
   {
      if ((current_timestamp - config_get_deployment_start_time()) >= led_active_seconds)
      {
         leds_enable(false);
         led_active_seconds = 0;
      }
      else
         wakeup_timestamp = MIN(wakeup_timestamp, config_get_deployment_start_time() + led_active_seconds);
   }

   // Ensure that the RTC is still ticking
   if (current_timestamp == previous_timestamp)
   {
      print("ERROR: RTC appears to have stopped ticking...resetting device\n");
      phase_ended = true;
   }
   previous_timestamp = current_timestamp;

   // Log relevant current device information
   mram_set_last_known_timestamp(current_timestamp);
   print("INFO: Current Device Details:\n"
         "   UTC Timestamp: %u\n"
         "   Battery Voltage (mV): %u\n"
         "   Temperature (C): %0.2f\n"
         "   Location: [%0.6f, %0.6f, %0.2f]\n"
         "   LEDs Active: %s\n"
         "   VHF Active: %s\n",
         current_timestamp, battery_details.millivolts, battery_details.celcius,
         last_lat, last_lon, last_height, leds_are_enabled() ? "True" : "False", vhf_activated() ? "True" : "False");
   storage_flush_log();

   // Restart the RTC alarm for the next wakeup time
   validation_time = false;
   if (!phase_ended && *device_active)
      rtc_set_wakeup_timestamp(wakeup_timestamp);
}

static void flush_remaining_imu_data(void)
{
   // Flush any unwritten IMU data in the storage buffer to the SD card
   __disable_irq();
   if (imu_storage_index)
   {
      const uint8_t* imu_data = (uint8_t*)imu_storage_buffer;
      const uint32_t imu_data_len = sizeof(float) * 3 * imu_storage_index;
      imu_storage_buffer = (imu_storage_buffer == imu_data_buffer) ? &imu_data_buffer[IMU_BUFFER_NUM_SAMPLES] : &imu_data_buffer[0];
      imu_storage_index = 0;
      __enable_irq();
      storage_write_imu_data(imu_data, imu_data_len);
   }
   else
      __enable_irq();
}


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void am_rtc_isr(void)
{
   // Clear the RTC interrupt status and validate all device settings
   static am_hal_rtc_alarm_repeat_e repeat_interval;
   AM_CRITICAL_BEGIN
   am_hal_rtc_alarm_get(NULL, &repeat_interval);
   am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);
   AM_CRITICAL_END
   validation_time = true;
}

void imu_data_callback(float accel_x_mg, float accel_y_mg, float accel_z_mg)
{
   // Store IMU data into storage buffer
   imu_storage_buffer[imu_storage_index][0] = accel_x_mg;
   imu_storage_buffer[imu_storage_index][1] = accel_y_mg;
   imu_storage_buffer[imu_storage_index][2] = accel_z_mg;

   // Set storage flag for the main thread if buffer is full
   if (++imu_storage_index == IMU_BUFFER_NUM_SAMPLES)
   {
      imu_storage_index = 0;
      imu_data_awaiting_storage = (uint8_t*)imu_storage_buffer;
      imu_storage_buffer = (imu_storage_buffer == imu_data_buffer) ? &imu_data_buffer[IMU_BUFFER_NUM_SAMPLES] : &imu_data_buffer[0];
   }
}

void imu_motion_change_callback(bool new_in_motion)
{
   // Only handle callback firing if this is actually a change in motion
   if (new_in_motion != in_motion)
   {
      // Subscribe or unsubscribe from IMU data based on the current motion status
      in_motion = new_in_motion;
      if (in_motion)
      {
         new_imu_stream = true;
         imu_enable_raw_data_output(in_motion, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);
      }
      else
         imu_enable_raw_data_output(in_motion, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);
   }
}

void am_timer00_isr(void)
{
   // Clear the timer interrupt and reset the clips stored counter
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE_BOTH));
   audio_timer_triggered = true;
   num_clips_stored = 0;
}

static void henrik_data_available(henrik_msg_t message_type, const void *new_data)
{
   // Handle the incoming message based on type
   switch (message_type)
   {
      case MSG_GPS:
      {
         // Sync RTC to GPS time whenever an update is received
         const henrik_gps_data_t *gps_data = (const henrik_gps_data_t*)new_data;
         mram_set_last_known_timestamp(gps_data->utc_timestamp);
         rtc_set_time_from_timestamp(gps_data->utc_timestamp);
         last_height = gps_data->height;
         last_lat = gps_data->lat;
         last_lon = gps_data->lon;
         break;
      }
      case MSG_CONFIG:
         // TODO: Handle configuration change requests
         //const henrik_config_data_t *config_data = (const henrik_config_data_t*)new_data;
         break;
      default:
         break;
   }
}


// Audio Processing Loops ----------------------------------------------------------------------------------------------

static void process_audio_continuous(uint32_t sampling_rate, uint32_t num_audio_reads_per_clip)
{
   // Initialize all necessary local variables
   bool audio_clip_in_progress = false;
   uint32_t num_audio_reads = 0;
   int16_t *audio_buffer;

   // Begin reading audio data and IMU data if enabled
   audio_begin_reading();
   if (record_imu_with_audio)
      imu_enable_raw_data_output(true, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);

   // Handling incoming audio clips until the phase has ended or the device has been deactivated
   system_enable_watchdog();
   while (!phase_ended && *device_active)
   {
      // Determine if time to re-validate device settings
      const uint32_t current_time = rtc_get_timestamp();
      if (validation_time)
         validate_device_settings(current_time);

      // Store any pending IMU data
      if (new_imu_stream)
      {
         flush_remaining_imu_data();
         storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
         new_imu_stream = false;
      }
      if (imu_data_awaiting_storage)
      {
         storage_write_imu_data((uint8_t*)imu_data_awaiting_storage, sizeof(float) * 3 * IMU_BUFFER_NUM_SAMPLES);
         imu_data_awaiting_storage = NULL;
      }

      // Handle any newly available audio data
      if (audio_error_encountered())
         system_reset();
      else if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Determine if time to create a new WAV file
         if (!audio_clip_in_progress)
         {
            // Check for total silence if silence filtering is enabled
            if (!use_silence_filter || !silence_filter_is_silence(audio_buffer, sampling_rate * AUDIO_BUFFER_NUM_SECONDS))
            {
               // Generate a new audio file using the current date and time
               if (storage_open_wav_file(activation_number, device_label, AUDIO_NUM_CHANNELS, sampling_rate, current_time))
               {
                  // Signal start of a new audio clip
                  if (record_imu_with_audio)
                  {
                     flush_remaining_imu_data();
                     storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
                  }
                  audio_clip_in_progress = true;
                  led_indicate_clip_begin();
               }
            }
         }

         // Write the audio clip to storage if currently in-progress
         if (audio_clip_in_progress)
         {
            system_feed_watchdog();  // TODO: FINISH WATCHDOG EVERYWHERE - ALSO NEED TO HANDLE IF SILENCE FILTER IS USED SINCE THIS MAY BE DELAYED
            led_indicate_clip_progress();
            storage_write_audio(audio_buffer, sizeof(int16_t) * sampling_rate * AUDIO_BUFFER_NUM_SECONDS);
            if (++num_audio_reads >= num_audio_reads_per_clip)
            {
               // Finalize the current WAV file
               storage_close_audio();
               led_indicate_clip_end();
               audio_clip_in_progress = false;
               num_audio_reads = 0;
            }
         }
      }
      else if (!validation_time)
         system_enter_deep_sleep_mode();
   }

   // Stop reading IMU data if enabled
   if (record_imu_with_audio)
      imu_enable_raw_data_output(false, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);

   // Ensure that the most recent audio file has been gracefully closed
   led_indicate_clip_end();
   storage_close_audio();
}

static void process_audio_scheduled(uint32_t sampling_rate, uint32_t num_audio_reads_per_clip, bool interval_based, int32_t clip_interval_seconds, uint32_t num_schedules, start_end_time_t *schedule)
{
   // Initialize all necessary local variables
   bool audio_clip_in_progress = false, reading_audio = false;
   uint32_t num_audio_reads = 0;
   int16_t *audio_buffer;

   // Start the clip creation timer if interval-based
   if (interval_based)
   {
      audio_timer_triggered = true;
      audio_processing_timer_config.eFunction =  AM_HAL_TIMER_FN_UPCOUNT;
      audio_processing_timer_config.ui32Compare0 = (uint32_t)(clip_interval_seconds * TIMER_AUDIO_PROCESSING_TICK_RATE) - 1;
      am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
      am_hal_timer_clear(TIMER_NUMBER_AUDIO_PROCESSING);
   }

   // Handling incoming audio clips until the phase has ended or the device has been deactivated
   while (!phase_ended && *device_active)
   {
      // Determine if time to re-validate device settings
      const uint32_t current_time = rtc_get_timestamp();
      if (validation_time)
         validate_device_settings(current_time);

      // Store any pending IMU data
      if (new_imu_stream)
      {
         flush_remaining_imu_data();
         storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
         new_imu_stream = false;
      }
      if (imu_data_awaiting_storage)
      {
         storage_write_imu_data((uint8_t*)imu_data_awaiting_storage, sizeof(float) * 3 * IMU_BUFFER_NUM_SAMPLES);
         imu_data_awaiting_storage = NULL;
      }

      // Determine if time to create a new WAV file
      const uint32_t seconds_til_next_scheduled_recording = seconds_until_next_scheduled_recording(num_schedules, schedule, (uint32_t)((int32_t)current_time + config_get_utc_offset_seconds()) % 86400);
      if (!audio_clip_in_progress)
      {
         // Go to sleep if time remains until the next scheduled audio recording
         if (interval_based && !audio_timer_triggered)
         {
            while (!audio_timer_triggered && !phase_ended && !validation_time && *device_active)
               system_enter_deep_sleep_mode();
            continue;
         }
         else if (!interval_based)
         {
            uint32_t seconds_to_sleep = MIN(seconds_til_next_scheduled_recording, phase_end_timestamp - current_time);
            if (seconds_to_sleep)
            {
               audio_timer_triggered = false;
               audio_processing_timer_config.ui32Compare0 = (uint32_t)(seconds_to_sleep * TIMER_AUDIO_PROCESSING_TICK_RATE);
               am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
               am_hal_timer_clear(TIMER_NUMBER_AUDIO_PROCESSING);
               while (!audio_timer_triggered && !phase_ended && !validation_time && *device_active)
                  system_enter_deep_sleep_mode();
               continue;
            }
         }
         audio_timer_triggered = false;

         // Generate a new audio file using the current date and time
         if (storage_open_wav_file(activation_number, device_label, AUDIO_NUM_CHANNELS, sampling_rate, current_time))
         {
            // Signal start of a new audio clip
            audio_clip_in_progress = true;
            led_indicate_clip_begin();

            // Begin reading IMU data if enabled
            if (record_imu_with_audio)
            {
               flush_remaining_imu_data();
               storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
               imu_enable_raw_data_output(true, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);
            }

            // Trigger reading audio samples if currently stopped
            if (!reading_audio)
            {
               audio_begin_reading();
               reading_audio = true;
            }
         }
      }

      // Handle any newly available audio data
      if (audio_error_encountered())
         system_reset();
      else if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         led_indicate_clip_progress();
         storage_write_audio(audio_buffer, sizeof(int16_t) * sampling_rate * AUDIO_BUFFER_NUM_SECONDS);
         if (++num_audio_reads >= num_audio_reads_per_clip)
         {
            // Finalize the current WAV file and stop reading if interval-based or if the current schedule has ended
            storage_close_audio();
            if (interval_based || (num_schedules && seconds_til_next_scheduled_recording))
            {
               audio_stop_reading();
               reading_audio = false;
            }
            led_indicate_clip_end();
            audio_clip_in_progress = false;
            num_audio_reads = 0;

            // Stop reading IMU data if enabled
            if (record_imu_with_audio)
               imu_enable_raw_data_output(false, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);
         }
      }
      else if (!validation_time)
         system_enter_deep_sleep_mode();
   }

   // Ensure that the most recent audio file has been gracefully closed
   led_indicate_clip_end();
   storage_close_audio();
}

static void process_audio_triggered(bool allow_extended_audio_clips, uint32_t sampling_rate, uint32_t num_audio_reads_per_clip, uint32_t max_clips, uint32_t per_num_seconds)
{
   // Initialize all necessary local variables
   bool audio_clip_in_progress = false, awaiting_trigger = false;
   uint32_t num_audio_reads = 0;
   int16_t *audio_buffer;
   num_clips_stored = 0;

   // Start timer to ensure that no more than "max_clips" are stored during the given number of seconds
   audio_processing_timer_config.eFunction =  AM_HAL_TIMER_FN_UPCOUNT;
   audio_processing_timer_config.ui32Compare0 = (uint32_t)(per_num_seconds * TIMER_AUDIO_PROCESSING_TICK_RATE) - 1;
   am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
   am_hal_timer_clear(TIMER_NUMBER_AUDIO_PROCESSING);

   // Handling incoming audio clips until the phase has ended or the device has been deactivated
   while (!phase_ended && *device_active)
   {
      // Determine if time to re-validate device settings
      const uint32_t current_time = rtc_get_timestamp();
      if (validation_time)
         validate_device_settings(current_time);

      // Determine if time to start listening for a new audio clip
      if (!awaiting_trigger && !audio_clip_in_progress && (num_clips_stored < max_clips))
      {
         audio_begin_reading();
         awaiting_trigger = true;
      }

      // Store any pending IMU data
      if (new_imu_stream)
      {
         flush_remaining_imu_data();
         storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
         new_imu_stream = false;
      }
      if (imu_data_awaiting_storage)
      {
         storage_write_imu_data((uint8_t*)imu_data_awaiting_storage, sizeof(float) * 3 * IMU_BUFFER_NUM_SAMPLES);
         imu_data_awaiting_storage = NULL;
      }

      // Handle any newly available audio data
      if (audio_error_encountered())
         system_reset();
      else if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Create a WAV file if this is a new audio clip
         if (!audio_clip_in_progress)
         {
            // Generate a new audio file using the current date and time
            if (storage_open_wav_file(activation_number, device_label, AUDIO_NUM_CHANNELS, sampling_rate, current_time))
            {
               // Signal start of a new audio clip
               audio_clip_in_progress = true;
               led_indicate_clip_begin();

               // Begin reading IMU data if enabled
               if (record_imu_with_audio)
               {
                  flush_remaining_imu_data();
                  storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
                  imu_enable_raw_data_output(true, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);
               }
            }
         }

         // Store the audio data to the WAV file
         led_indicate_clip_progress();
         storage_write_audio(audio_buffer, sizeof(int16_t) * sampling_rate * AUDIO_BUFFER_NUM_SECONDS);
         if (++num_audio_reads >= num_audio_reads_per_clip)
         {
            awaiting_trigger = audio_clip_in_progress = false;
            led_indicate_clip_end();
            audio_stop_reading();
            num_audio_reads = 0;
            ++num_clips_stored;
            storage_close_audio();

            // Stop reading IMU data if enabled
            if (record_imu_with_audio)
               imu_enable_raw_data_output(false, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, imu_sampling_rate_hz * AUDIO_BUFFER_NUM_SECONDS, imu_data_callback);
         }
      }
      else if (!validation_time)
         system_enter_deep_sleep_mode();
   }

   // Ensure that the most recent audio file has been gracefully closed
   led_indicate_clip_end();
   storage_close_audio();
}


// Public Main Function ------------------------------------------------------------------------------------------------

void active_main(volatile bool *device_activated, int32_t phase_index)
{
   // Ensure that a storage directory with the device name exists and is active on the SD card
   print("INFO: Starting main deployment activity for Phase #%d\n", phase_index+1);
   print("INFO: Validating existence of SD card storage directory...");
   config_get_device_label(device_label, sizeof(device_label));
   activation_number = config_get_activation_number();
   if (device_label[0] == '\0')
      memcpy(device_label, "Default", sizeof("Default"));
   const bool success = storage_mkdir(device_label);
   print("%s\n", success ? "SUCCESS" : "FAILURE");

   // Validate device settings (and implicitly set an RTC alarm for the next important event)
   validation_time = false;
   device_active = device_activated;
   vhf_enable_timestamp = config_get_vhf_start_timestamp();
   led_active_seconds = config_get_leds_active_seconds();
   phase_end_timestamp = config_get_end_time(phase_index);
   validate_device_settings(rtc_get_timestamp());

   // Set up the silence detection filter if enabled
   use_silence_filter = config_get_silence_filter_threshold(phase_index) > 0.0f;
   if (use_silence_filter)
   {
      const float threshold = config_get_silence_filter_threshold(phase_index);
      const uint32_t audio_sampling_rate = config_get_audio_sampling_rate_hz(phase_index);
      const frequency_range_t frequencies = config_get_frequencies_of_interest(phase_index);
      silence_filter_initialize(audio_sampling_rate, frequencies.min_frequency, frequencies.max_frequency, threshold);
   }

   // Initialize the audio processing timer
   am_hal_timer_default_config_set(&audio_processing_timer_config);
   audio_processing_timer_config.eInputClock = TIMER_AUDIO_PROCESSING_CLOCK;
   audio_processing_timer_config.ui32Compare0 = (uint32_t)(300 * TIMER_AUDIO_PROCESSING_TICK_RATE);
   am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
   am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE0));
   NVIC_SetPriority(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING, AUDIO_TIMER_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING);

   // Listen for incoming data messages from the Henrik board
   if (config_gps_available())
      henrik_register_data_callback(henrik_data_available);

   // Enable IMU detection and recording functionality
   imu_storage_index = 0;
   imu_data_awaiting_storage = NULL;
   imu_storage_buffer = imu_data_buffer;
   in_motion = new_imu_stream = record_imu_with_audio = false;
   imu_degrees_of_freedom = config_get_imu_degrees_of_freedom(phase_index);
   imu_sampling_rate_hz = config_get_imu_sampling_rate_hz(phase_index);
   switch (config_get_imu_recording_mode(phase_index))
   {
      case ACTIVITY:
         // TODO: Use this: float motion_trigger_threshold = config_get_imu_trigger_threshold_level(phase_index);
         imu_enable_motion_change_detection(true, imu_motion_change_callback);
         break;
      case AUDIO:
         record_imu_with_audio = true;
         break;
      case NONE:   // Intentional fall-through
      default:
         break;
   }

   // Determine how to schedule audio clip recordings
   audio_timer_triggered = false;
   const uint32_t audio_sampling_rate_hz = config_get_audio_sampling_rate_hz(phase_index);
   const uint32_t num_audio_reads_per_clip = config_get_audio_clip_length_seconds(phase_index) / AUDIO_BUFFER_NUM_SECONDS;
   switch (config_get_audio_recording_mode(phase_index))
   {
      case AMPLITUDE:
      {
         time_scale_t unit_time;
         uint32_t max_num_clips, max_clips_interval_seconds;
         config_get_max_audio_clips(phase_index, &max_num_clips, &unit_time);
         bool allow_extended_audio_clips = config_extend_clip_for_continuous_audio(phase_index);
         switch (unit_time)
         {
            case MINUTES:
               max_clips_interval_seconds = 60;
               break;
            case HOURS:
               max_clips_interval_seconds = 3600;
               break;
            case DAYS:
               max_clips_interval_seconds = 86400;
               break;
            case SECONDS:   // Intentional fall-through
            default:
               max_clips_interval_seconds = 1;
               break;
         }
         audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, COMPARATOR_THRESHOLD, config_get_audio_trigger_threshold(phase_index), device_activated);
         process_audio_triggered(allow_extended_audio_clips, audio_sampling_rate_hz, num_audio_reads_per_clip, max_num_clips, max_clips_interval_seconds);
         break;
      }
      case SCHEDULED:
      {
         start_end_time_t *schedule;
         uint32_t num_schedules = config_get_audio_trigger_schedule(phase_index, &schedule);
         if (config_get_mic_type() == MIC_ANALOG)
            audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, device_activated);
         else
            audio_digital_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db());
         process_audio_scheduled(audio_sampling_rate_hz, num_audio_reads_per_clip, false, 0, num_schedules, schedule);
         break;
      }
      case INTERVAL:
      {
         time_scale_t unit_time;
         uint32_t audio_recording_interval;
         config_get_audio_trigger_interval(phase_index, &audio_recording_interval, &unit_time);
         switch (unit_time)
         {
            case MINUTES:
               audio_recording_interval *= 60;
               break;
            case HOURS:
               audio_recording_interval *= 3600;
               break;
            case DAYS:
               audio_recording_interval *= 86400;
               break;
            case SECONDS:   // Intentional fall-through
            default:
               audio_recording_interval *= 1;
               break;
         }
         if (config_get_mic_type() == MIC_ANALOG)
            audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, device_activated);
         else
            audio_digital_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db());
         process_audio_scheduled(audio_sampling_rate_hz, num_audio_reads_per_clip, true, (int32_t)audio_recording_interval, 0, NULL);
         break;
      }
      case CONTINUOUS:  // Intentional fall-through
      default:
         if (config_get_mic_type() == MIC_ANALOG)
            audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, device_activated);
         else
            audio_digital_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db());
         process_audio_continuous(audio_sampling_rate_hz, num_audio_reads_per_clip);
         break;
   }

   // Stop all running timers
   am_hal_rtc_interrupt_disable(AM_HAL_RTC_INT_ALM);
   am_hal_timer_disable(TIMER_NUMBER_AUDIO_PROCESSING);
   NVIC_DisableIRQ(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING);
   am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE0));

   // Close any open storage files
   storage_close();
   storage_close_imu();
   storage_close_audio();
   print("INFO: Leaving main deployment activity for Phase #%d\n", phase_index+1);
}
