#include "audio.h"
#include "audio_filter.h"
#include "battery.h"
#include "comparator.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "mram.h"
#include "opus_config.h"
#include "rtc.h"
#include "silence.h"
#include "storage.h"
#include "system.h"
#include "tracker.h"
#include "vhf.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static volatile uint32_t num_clips_stored, audio_samples_per_dma;
static volatile bool *device_active, phase_ended, audio_timer_triggered;
static volatile bool in_motion, new_imu_stream, validation_time, motion_change_pending;
static uint32_t phase_end_timestamp, vhf_enable_timestamp, led_active_seconds, imu_sampling_rate_hz, activation_number;
static uint32_t end_of_phase_reason = RESET_REASON_PHASE_COMPLETE;
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

static void report_microphone_health(void)
{
   // Report the state of the microphone signal path for the window just recorded
   if (!audio_health_available())
      return;

   const audio_health_t health = audio_health_get();
   const char *verdict = "PASS";
   if (health.constant_output)
      verdict = "FAIL_CONSTANT";
   else if (health.silent)
      verdict = "WARN_SILENT";

   log_event("MIC_HEALTH", "result=%s,rms=%u,peak=%u,min=%d,max=%d,mean=%d,samples=%u,dc_offset=%d",
             verdict, health.rms, health.peak, (int)health.min_sample, (int)health.max_sample,
             (int)health.mean, health.num_samples, (int)audio_get_dc_offset());

   // Start a fresh window so each report describes only its own directory
   audio_health_reset();
}

static void validate_device_settings(uint32_t current_timestamp)
{
   // Check if the battery voltage is too low to continue
   const battery_result_t battery_details = battery_monitor_get_details();
   phase_ended = battery_monitor_is_critically_low(config_get_battery_mV_low());
   if (phase_ended)
   {
      print("WARNING: Battery confirmed low at %u mV - ending phase\n", battery_details.millivolts);
      log_event("BATTERY_LOW", "batt_mv=%u,cutoff_mv=%u", battery_details.millivolts, config_get_battery_mV_low());
      end_of_phase_reason = RESET_REASON_BATTERY_LOW;
   }

   // Check if the current phase has ended or if it is time to activate the VHF radio
   static uint32_t previous_timestamp = 0;
   uint32_t wakeup_timestamp = MIN(current_timestamp + MIN_LOG_DATA_INTERVAL_SECONDS, phase_end_timestamp);
   if (current_timestamp >= phase_end_timestamp)
   {
      phase_ended = true;
      end_of_phase_reason = RESET_REASON_PHASE_COMPLETE;
   }
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
      const uint32_t deployment_start = config_get_deployment_start_time();
      if ((current_timestamp >= deployment_start) && ((current_timestamp - deployment_start) >= led_active_seconds))
      {
         leds_enable(false);
         led_active_seconds = 0;
      }
      else if (current_timestamp >= deployment_start)
         wakeup_timestamp = MIN(wakeup_timestamp, deployment_start + led_active_seconds);
   }

   // Ensure that the RTC is still ticking
   if (current_timestamp && (current_timestamp == previous_timestamp))
   {
      print("ERROR: RTC appears to have stopped ticking...resetting device\n");
      phase_ended = true;
      end_of_phase_reason = RESET_REASON_RTC_STOPPED;
   }
   previous_timestamp = current_timestamp;

   // Log relevant current device information
   mram_set_last_known_timestamp(current_timestamp);
   audio_stats_t audio_stats;
   storage_health_t storage_health;
   audio_get_stats(&audio_stats);
   storage_get_health(&storage_health);
   uint64_t cache_accesses = 0, cache_served = 0;
   float cache_hit_rate = 0.0f;
   const bool cache_valid = system_get_cache_stats(&cache_accesses, &cache_served, &cache_hit_rate);
   log_event("TELEM", "time=%u,batt_mv=%u,temp_c=%0.2f,lat=%0.6f,lon=%0.6f,alt=%0.2f,"
                      "leds=%u,vhf=%u,sd_free_mb=%u,sd_write_fail=%u,sd_reopen=%u,sd_remount=%u,"
                      "imu_dropped=%u,audio_dropped=%u,audio_buffers=%u,dcmp=%s,"
                      "icache_accesses=%llu,icache_served=%llu,icache_hit_pct=%0.2f,rate_est_hz=%u,rate_settled=%u",
             current_timestamp, battery_details.millivolts, battery_details.celcius, last_lat, last_lon, last_height,
             leds_are_enabled() ? 1u : 0u, vhf_activated() ? 1u : 0u,
             storage_get_free_space_mb(), storage_health.write_failures,
             storage_health.reopen_recoveries, storage_health.remount_recoveries,
             storage_health.imu_buffers_dropped, audio_stats.buffers_dropped, audio_stats.buffers_captured,
             !audio_stats.dcmp_applicable ? "n/a" : (audio_stats.dcmp_trusted ? "trusted" : "unproven"),
             cache_valid ? cache_accesses : 0ull, cache_valid ? cache_served : 0ull,
             cache_valid ? cache_hit_rate : 0.0f,
             audio_get_rate_estimate(), audio_rate_is_settled() ? 1u : 0u);

   // Report the cumulative health counters when one of them actually moves
   static uint32_t last_reported_health[6];
   const uint32_t imu_overruns = imu_get_fifo_overrun_count(), hal_failures = system_get_hal_failure_count();
   const uint32_t health_now[6] = { audio_stats.buffers_dropped, audio_stats.missed_completions, storage_health.write_failures, storage_health.imu_buffers_dropped, imu_overruns, hal_failures };
   bool health_changed = false;
   for (uint32_t i = 0; i < 6; ++i)
      if (health_now[i] != last_reported_health[i])
      {
         last_reported_health[i] = health_now[i];
         health_changed = true;
      }
   if (health_changed)
   {
      print("   DEGRADED: %u audio buffers dropped, %u DMA completions recovered\n"
            "   DEGRADED: %u write failures, %u reopens, %u remounts, %u IMU buffers dropped, %u IMU FIFO overruns\n",
            audio_stats.buffers_dropped, audio_stats.missed_completions,
            storage_health.write_failures, storage_health.reopen_recoveries,
            storage_health.remount_recoveries, storage_health.imu_buffers_dropped, imu_overruns);
      if (hal_failures)
      {
         uint32_t line = 0, status = 0;
         const char *file = system_get_first_hal_failure(&line, &status);
         print("   DEGRADED: %u HAL failures, first at %s:%u status %u\n", hal_failures, file ? file : "unknown", line, status);
      }
   }

   // Check if time to roll into a new audio directory
   if (storage_rotate_log(activation_number, device_label, current_timestamp))
   {
      // A new directory means a new recording window; summarise the one that just closed
      report_microphone_health();
   }
   storage_flush_log();

   // Restart the RTC alarm for the next wakeup time
   validation_time = false;
   if (!phase_ended && *device_active)
      rtc_set_wakeup_timestamp(wakeup_timestamp);
}

static bool recover_from_audio_write_failure(uint32_t sampling_rate, uint32_t current_time)
{
   // Handle a failed audio write by escalating through the storage layer's recovery options
   if (storage_recover_audio(activation_number, device_label, AUDIO_NUM_CHANNELS, audio_get_actual_sample_rate(), current_time))
      return true;
   end_of_phase_reason = RESET_REASON_STORAGE_FAILURE;
   return false;
}

static void service_background_work(void)
{
   // Perform work that every audio processing loop requires on each pass
   system_feed_watchdog();
   magnet_sensor_handle_pending_validation();
   battery_monitor_service_tempco(rtc_get_timestamp());

   // Hand the measured rate to storage so each WAV header can be patched with it at close
   storage_set_measured_sample_rate(audio_get_measured_sample_rate());

   // Fold the cache counters into their 64-bit totals before the 32-bit hardware ones can wrap
   system_accumulate_cache_stats();

   // Apply a GPS update from the main loop rather than from the tracker interrupt
   tracker_gps_data_t gps;
   if (tracker_get_pending_gps_data(&gps))
   {
      last_height = gps.height;
      last_lat = gps.lat;
      last_lon = gps.lon;
      if (gps.utc_timestamp)
      {
         // Record the correction before applying it; everything after is true UTC
         const uint32_t before_sync = rtc_get_timestamp();
         mram_set_last_known_timestamp(gps.utc_timestamp);
         rtc_set_time_from_timestamp(gps.utc_timestamp);
         if (before_sync != gps.utc_timestamp)
            log_event("CLOCK_SYNC", "source=GPS,before=%u,after=%u", before_sync, gps.utc_timestamp);
      }
   }

   // Apply a pending motion-state change recorded by the IMU interrupt
   if (motion_change_pending)
   {
      motion_change_pending = false;
      const bool subscribe = in_motion;
      if (subscribe)
         new_imu_stream = true;
      imu_enable_raw_data_output(subscribe, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, storage_write_imu_data);
   }

   // Treat a passed validation deadline as equivalent to the RTC interrupt firing
   if (!validation_time && rtc_wakeup_elapsed())
      validation_time = true;
}


// Interrupt Service Routines ------------------------------------------------------------------------------------------

#define audio_processing_timer_isr       am_timer_isr1(TIMER_NUMBER_AUDIO_PROCESSING)

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

void imu_motion_change_callback(bool new_in_motion)
{
   if (new_in_motion != in_motion)
   {
      in_motion = new_in_motion;
      motion_change_pending = true;
   }
}

void audio_processing_timer_isr(void)
{
   // Clear the timer interrupt and reset the clips stored counter
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE_BOTH));
   audio_timer_triggered = true;
   num_clips_stored = 0;
}

static void tracker_data_available(tracker_msg_t message_type, const void *new_data)
{
   // This runs in the I2C slave interrupt handler, so it must do essentially nothing. GPS messages
   // are queued by the tracker driver itself and applied by service_background_work() from the main
   // loop: the RTC update needs calendar conversion and the location update needs an MRAM program,
   // and neither belongs in an interrupt while audio DMA is running.
   (void)new_data;
   switch (message_type)
   {
      case MSG_CONFIG:
         // TODO: Handle configuration change requests
         break;
      default:
         break;
   }
}


// Audio Processing Loops ----------------------------------------------------------------------------------------------

static void process_audio_continuous(uint32_t sampling_rate, uint32_t num_audio_reads_per_clip, bool ogg_encode)
{
   // Initialize all necessary local variables
   audio_samples_per_dma = audio_num_seconds_per_dma() * sampling_rate;
   bool audio_clip_in_progress = false;
   uint32_t num_audio_reads = 0;
   int16_t *audio_buffer;

   // Begin reading audio data and IMU data if enabled
   audio_begin_reading();
   if (record_imu_with_audio)
      imu_enable_raw_data_output(true, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, storage_write_imu_data);

   // Handling incoming audio clips until the phase has ended or the device has been deactivated
   while (!phase_ended && *device_active)
   {
      // Perform the per-pass background work, then determine if time to re-validate device settings
      service_background_work();
      const uint32_t current_time = rtc_get_timestamp();
      if (validation_time)
         validate_device_settings(current_time);

      // Check if time to open a new IMU file
      if (new_imu_stream)
      {
         storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
         new_imu_stream = false;
      }
      storage_handle_imu_data();

      // Handle any newly available audio data
      if (audio_error_encountered())
      {
         print("ERROR: Audio DMA error encountered - resetting device\n");
         system_reset_with_reason(RESET_REASON_AUDIO_ERROR);
      }
      else if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Determine if time to create a new audio file
         if (!audio_clip_in_progress)
         {
            // Check for total silence if silence filtering is enabled
            if (!use_silence_filter || !silence_filter_is_silence(audio_buffer, audio_samples_per_dma))
            {
               // Generate a new audio file using the current date and time
               if (storage_open_audio_file(activation_number, device_label, AUDIO_NUM_CHANNELS, audio_get_actual_sample_rate(), current_time, ogg_encode))
               {
                  // Signal start of a new audio clip
                  if (record_imu_with_audio)
                     storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
                  audio_clip_in_progress = true;
                  led_indicate_clip_begin();
               }
            }
         }

         // Write the audio clip to storage if currently in-progress
         if (audio_clip_in_progress)
         {
            led_indicate_clip_progress();
            if (!storage_write_audio(audio_buffer, sizeof(int16_t) * audio_samples_per_dma, (num_audio_reads + 1) >= num_audio_reads_per_clip))
            {
               if (!recover_from_audio_write_failure(sampling_rate, current_time))
                  system_reset_with_reason(RESET_REASON_STORAGE_FAILURE);
               num_audio_reads = 0;
            }
            else if (++num_audio_reads >= num_audio_reads_per_clip)
            {
               // Finalize the current audio file
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
      imu_enable_raw_data_output(false, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, storage_write_imu_data);

   // Ensure that the most recent audio file has been gracefully closed
   led_indicate_clip_end();
   storage_close_audio();
}

static void process_audio_scheduled(uint32_t sampling_rate, uint32_t num_audio_reads_per_clip, bool interval_based, int32_t clip_interval_seconds, uint32_t num_schedules, start_end_time_t *schedule, bool ogg_encode)
{
   // Initialize all necessary local variables
   audio_samples_per_dma = audio_num_seconds_per_dma() * sampling_rate;
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
      // Perform the per-pass background work, then determine if time to re-validate device settings
      service_background_work();
      const uint32_t current_time = rtc_get_timestamp();
      if (validation_time)
         validate_device_settings(current_time);

      // Check if time to open a new IMU file
      if (new_imu_stream)
      {
         storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
         new_imu_stream = false;
      }
      storage_handle_imu_data();

      // Determine if time to create a new WAV file
      const uint32_t seconds_til_next_scheduled_recording = seconds_until_next_scheduled_recording(num_schedules, schedule, (uint32_t)((int32_t)current_time + config_get_utc_offset_seconds()) % 86400);
      if (!audio_clip_in_progress)
      {
         // Go to sleep if time remains until the next scheduled audio recording
         if (interval_based && !audio_timer_triggered)
         {
            while (!audio_timer_triggered && !phase_ended && !validation_time && *device_active)
            {
               service_background_work();
               if (!audio_timer_triggered && !phase_ended && !validation_time && *device_active)
                  system_enter_deep_sleep_mode();
            }
            continue;
         }
         else if (!interval_based)
         {
            const uint32_t seconds_until_phase_end = (phase_end_timestamp > current_time) ? (phase_end_timestamp - current_time) : 0;
            uint32_t seconds_to_sleep = MIN(seconds_til_next_scheduled_recording, seconds_until_phase_end);
            if (seconds_to_sleep)
            {
               audio_timer_triggered = false;
               audio_processing_timer_config.ui32Compare0 = (uint32_t)(seconds_to_sleep * TIMER_AUDIO_PROCESSING_TICK_RATE);
               am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
               am_hal_timer_clear(TIMER_NUMBER_AUDIO_PROCESSING);
               while (!audio_timer_triggered && !phase_ended && !validation_time && *device_active)
               {
                  service_background_work();
                  if (!audio_timer_triggered && !phase_ended && !validation_time && *device_active)
                     system_enter_deep_sleep_mode();
               }
               continue;
            }
         }
         audio_timer_triggered = false;

         // Generate a new audio file using the current date and time
         if (storage_open_audio_file(activation_number, device_label, AUDIO_NUM_CHANNELS, audio_get_actual_sample_rate(), current_time, ogg_encode))
         {
            // Signal start of a new audio clip
            audio_clip_in_progress = true;
            led_indicate_clip_begin();

            // Begin reading IMU data if enabled
            if (record_imu_with_audio)
            {
               storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
               imu_enable_raw_data_output(true, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, storage_write_imu_data);
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
      {
         print("ERROR: Audio DMA error encountered - resetting device\n");
         system_reset_with_reason(RESET_REASON_AUDIO_ERROR);
      }
      else if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         led_indicate_clip_progress();
         if (!storage_write_audio(audio_buffer, sizeof(int16_t) * audio_samples_per_dma, (num_audio_reads + 1) >= num_audio_reads_per_clip))
         {
            if (!recover_from_audio_write_failure(sampling_rate, current_time))
               system_reset_with_reason(RESET_REASON_STORAGE_FAILURE);
            num_audio_reads = 0;
         }
         else if (++num_audio_reads >= num_audio_reads_per_clip)
         {
            // Finalize the current audio file and stop reading if interval-based or if the current schedule has ended
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
               imu_enable_raw_data_output(false, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, storage_write_imu_data);
         }
      }
      else if (!validation_time)
         system_enter_deep_sleep_mode();
   }

   // Ensure that the most recent audio file has been gracefully closed
   led_indicate_clip_end();
   storage_close_audio();
}

static void process_audio_triggered(bool allow_extended_audio_clips, uint32_t sampling_rate, uint32_t num_audio_reads_per_clip, uint32_t max_clips, uint32_t per_num_seconds, bool ogg_encode)
{
   if (allow_extended_audio_clips)
      print("WARNING: AUDIO_EXTEND_CLIP is set but is not implemented and will have no effect\n");
   // Initialize all necessary local variables
   audio_samples_per_dma = audio_num_seconds_per_dma() * sampling_rate;
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
      // Perform the per-pass background work, then determine if time to re-validate device settings
      service_background_work();
      const uint32_t current_time = rtc_get_timestamp();
      if (validation_time)
         validate_device_settings(current_time);

      // Determine if time to start listening for a new audio clip
      if (!awaiting_trigger && !audio_clip_in_progress && (!max_clips || (num_clips_stored < max_clips)))
      {
         audio_begin_reading();
         awaiting_trigger = true;
      }

      // Check if time to open a new IMU file
      if (new_imu_stream)
      {
         storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
         new_imu_stream = false;
      }
      storage_handle_imu_data();

      // Handle any newly available audio data
      if (audio_error_encountered())
      {
         print("ERROR: Audio DMA error encountered - resetting device\n");
         system_reset_with_reason(RESET_REASON_AUDIO_ERROR);
      }
      else if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Create a WAV file if this is a new audio clip
         if (!audio_clip_in_progress)
         {
            // Generate a new audio file using the current date and time
            if (storage_open_audio_file(activation_number, device_label, AUDIO_NUM_CHANNELS, audio_get_actual_sample_rate(), current_time, ogg_encode))
            {
               // Signal start of a new audio clip
               audio_clip_in_progress = true;
               led_indicate_clip_begin();

               // Begin reading IMU data if enabled
               if (record_imu_with_audio)
               {
                  storage_open_imu_file(activation_number, device_label, current_time, imu_sampling_rate_hz);
                  imu_enable_raw_data_output(true, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, storage_write_imu_data);
               }
            }
         }

         // Store the audio data to the audio file
         led_indicate_clip_progress();
         if (!storage_write_audio(audio_buffer, sizeof(int16_t) * audio_samples_per_dma, (num_audio_reads + 1) >= num_audio_reads_per_clip))
         {
            if (!recover_from_audio_write_failure(sampling_rate, current_time))
               system_reset_with_reason(RESET_REASON_STORAGE_FAILURE);
            num_audio_reads = 0;
         }
         else if (++num_audio_reads >= num_audio_reads_per_clip)
         {
            awaiting_trigger = audio_clip_in_progress = false;
            led_indicate_clip_end();
            audio_stop_reading();
            num_audio_reads = 0;
            ++num_clips_stored;
            storage_close_audio();

            // Stop reading IMU data if enabled
            if (record_imu_with_audio)
               imu_enable_raw_data_output(false, LIS2DU12_2g, imu_sampling_rate_hz, LIS2DU12_ODR_div_2, storage_write_imu_data);
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
   config_get_device_label(device_label, sizeof(device_label));
   activation_number = config_get_activation_number();
   if (device_label[0] == '\0')
      memcpy(device_label, "Default", sizeof("Default"));
   const bool success = storage_mkdir(device_label);
   print("INFO: Validating existence of SD card storage directory...%s\n", success ? "SUCCESS" : "FAILURE");

   // Establish the timestamped directory for this phase and move the deployment log into it
   storage_rotate_log(activation_number, device_label, rtc_get_timestamp());

   // Validate device settings (and implicitly set an RTC alarm for the next important event)
   validation_time = false;
   phase_ended = false;
   device_active = device_activated;
   end_of_phase_reason = RESET_REASON_PHASE_COMPLETE;
   vhf_enable_timestamp = config_get_vhf_start_timestamp();
   led_active_seconds = config_get_leds_active_seconds();
   phase_end_timestamp = config_get_end_time(phase_index);
   validate_device_settings(rtc_get_timestamp());

   log_event("PHASE_START", "phase=%d", (int)(phase_index + 1));

   // Set up band-limiting of the recorded audio
   const uint32_t audio_sampling_rate_hz = config_get_audio_sampling_rate_hz(phase_index);
   const audio_filter_type_t filter_type = config_get_audio_filter_type(phase_index);
   const frequency_range_t filter_range = config_get_audio_filter_range(phase_index);
   audio_filter_initialize(filter_type, audio_sampling_rate_hz, filter_range.min_frequency, filter_range.max_frequency);
   if (audio_filter_enabled())
   {
      print("INFO: Audio band-limited to [%u, %u] Hz\n", filter_range.min_frequency, filter_range.max_frequency);
      log_event("AUDIO_FILTER", "type=%d,low_hz=%u,high_hz=%u",
                (int)filter_type, filter_range.min_frequency, filter_range.max_frequency);
   }

   // Set up the silence detection filter if enabled
   use_silence_filter = config_get_silence_filter_threshold(phase_index) > 0.0f;
   if (use_silence_filter)
   {
      const float threshold = config_get_silence_filter_threshold(phase_index);
      const frequency_range_t frequencies = config_get_frequencies_of_interest(phase_index);
      use_silence_filter = silence_filter_initialize(audio_sampling_rate_hz, frequencies.min_frequency, frequencies.max_frequency, threshold);
   }
   if (use_silence_filter && (config_get_audio_recording_mode(phase_index) != CONTINUOUS))
      print("WARNING: A silence threshold is configured but only takes effect in CONTINUOUS mode\n");

   // Set up Ogg Opus encoding if enabled
   const int32_t encoding_bitrate = config_use_opus_encoding(phase_index) ? config_get_opus_bitrate(phase_index) : 0;
   if (encoding_bitrate > 0)
      opusenc_init(encoding_bitrate);

   // Initialize the audio processing timer
   am_hal_timer_default_config_set(&audio_processing_timer_config);
   audio_processing_timer_config.eInputClock = TIMER_AUDIO_PROCESSING_CLOCK;
   audio_processing_timer_config.ui32Compare0 = (uint32_t)(300 * TIMER_AUDIO_PROCESSING_TICK_RATE);
   am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
   am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE0));
   NVIC_SetPriority(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING, AUDIO_TIMER_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING);

   // Listen for incoming data messages from the GPS Tracker board
   if (config_gps_available())
      tracker_register_data_callback(tracker_data_available);

   // Enable IMU detection and recording functionality
   in_motion = new_imu_stream = record_imu_with_audio = motion_change_pending = false;
   imu_degrees_of_freedom = config_get_imu_degrees_of_freedom(phase_index);
   imu_sampling_rate_hz = config_get_imu_sampling_rate_hz(phase_index);
   switch (config_get_imu_recording_mode(phase_index))
   {
      case ACTIVITY:
         imu_enable_motion_change_detection(true, config_get_imu_trigger_threshold_level(phase_index), imu_motion_change_callback);
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
   const uint32_t audio_clip_length_seconds = config_get_audio_clip_length_seconds(phase_index);
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
         if (!audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, audio_clip_length_seconds, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, COMPARATOR_THRESHOLD, config_get_audio_trigger_threshold(phase_index), device_activated))
            break;
         process_audio_triggered(allow_extended_audio_clips, audio_sampling_rate_hz, audio_clip_length_seconds / audio_num_seconds_per_dma(), max_num_clips, max_clips_interval_seconds, encoding_bitrate > 0);
         break;
      }
      case SCHEDULED:
      {
         start_end_time_t *schedule;
         uint32_t num_schedules = config_get_audio_trigger_schedule(phase_index, &schedule);
         if (!((config_get_mic_type() == MIC_ANALOG) ?
               audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, audio_clip_length_seconds, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, device_activated) :
               audio_digital_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, audio_clip_length_seconds, config_get_mic_amplification_db())))
            break;
         process_audio_scheduled(audio_sampling_rate_hz, audio_clip_length_seconds / audio_num_seconds_per_dma(), false, 0, num_schedules, schedule, encoding_bitrate > 0);
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
         if (!((config_get_mic_type() == MIC_ANALOG) ?
               audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, audio_clip_length_seconds, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, device_activated) :
               audio_digital_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, audio_clip_length_seconds, config_get_mic_amplification_db())))
            break;
         process_audio_scheduled(audio_sampling_rate_hz, audio_clip_length_seconds / audio_num_seconds_per_dma(), true, (int32_t)audio_recording_interval, 0, NULL, encoding_bitrate > 0);
         break;
      }
      case CONTINUOUS:  // Intentional fall-through
      default:
         if (!((config_get_mic_type() == MIC_ANALOG) ?
               audio_analog_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, audio_clip_length_seconds, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, device_activated) :
               audio_digital_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, audio_clip_length_seconds, config_get_mic_amplification_db())))
            break;
         process_audio_continuous(audio_sampling_rate_hz, audio_clip_length_seconds / audio_num_seconds_per_dma(), encoding_bitrate > 0);
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
   storage_flush_log();
   print("INFO: Leaving main deployment activity for Phase #%d\n", phase_index+1);
}

uint32_t active_main_get_end_reason(void)
{
   return end_of_phase_reason;
}

void pre_active_main(volatile bool *device_activated)
{
   // Ensure that a storage directory with the device name exists and is active on the SD card
   print("INFO: Starting pre-deployment recording activity\n");
   config_get_device_label(device_label, sizeof(device_label));
   activation_number = config_get_activation_number();
   if (device_label[0] == '\0')
      memcpy(device_label, "Default", sizeof("Default"));
   const bool success = storage_mkdir(device_label);
   print("INFO: Validating existence of SD card storage directory...%s\n", success ? "SUCCESS" : "FAILURE");

   // Establish the timestamped directory and move the deployment log into it
   const uint32_t start_time = rtc_get_timestamp();
   storage_rotate_log(activation_number, device_label, start_time);

   // Initialize the correct audio input channel
   if (!((config_get_mic_type() == MIC_ANALOG) ?
         audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_PRE_DEPLOYMENT_SAMPLE_RATE_HZ, AUDIO_PRE_DEPLOYMENT_CLIP_LENGTH_SECONDS, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, device_activated) :
         audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_PRE_DEPLOYMENT_SAMPLE_RATE_HZ, AUDIO_PRE_DEPLOYMENT_CLIP_LENGTH_SECONDS, config_get_mic_amplification_db())))
   {
      print("ERROR: Unable to initialize audio for pre-deployment recording\n");
      return;
   }
   const uint32_t seconds_per_dma = audio_num_seconds_per_dma();
   if (!seconds_per_dma)
   {
      print("ERROR: Invalid audio DMA period for pre-deployment recording\n");
      return;
   }
   const uint32_t num_audio_reads_per_clip = AUDIO_PRE_DEPLOYMENT_CLIP_LENGTH_SECONDS / seconds_per_dma;
   audio_samples_per_dma = seconds_per_dma * AUDIO_PRE_DEPLOYMENT_SAMPLE_RATE_HZ;
   int16_t *audio_buffer;
   audio_begin_reading();

   // Handling incoming audio clips until the phase has ended or the device has been deactivated
   led_indicate_clip_begin();
   storage_open_audio_file(activation_number, device_label, AUDIO_NUM_CHANNELS, audio_get_actual_sample_rate(), start_time, false);
   for (uint32_t num_audio_reads = 0; *device_activated && (num_audio_reads < num_audio_reads_per_clip); )
   {
      // Feed the watchdog, deliver any deferred magnet validation, keep the voltage trim current
      system_feed_watchdog();
      magnet_sensor_handle_pending_validation();
      battery_monitor_service_tempco(rtc_get_timestamp());

   // Hand the measured rate to storage so each WAV header can be patched with it at close
   storage_set_measured_sample_rate(audio_get_measured_sample_rate());

      // Handle any newly available audio data
      if (audio_error_encountered())
      {
         print("ERROR: Audio DMA error during pre-deployment recording - resetting device\n");
         system_reset_with_reason(RESET_REASON_AUDIO_ERROR);
      }
      else if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Write the audio clip to storage, giving up on this recording if the card will not take it
         led_indicate_clip_progress();
         if (!storage_write_audio(audio_buffer, sizeof(int16_t) * audio_samples_per_dma, (num_audio_reads + 1) >= num_audio_reads_per_clip))
         {
            print("ERROR: Unable to write pre-deployment audio - abandoning the recording\n");
            break;
         }
         ++num_audio_reads;
      }
      else
         system_enter_deep_sleep_mode();
   }
   led_indicate_clip_end();
   storage_close_audio();
   audio_stop_reading();

   // Close any open storage files and return
   storage_close();
   storage_flush_log();
   print("INFO: Leaving pre-deployment recording activity\n");
}
