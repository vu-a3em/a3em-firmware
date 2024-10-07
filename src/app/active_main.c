#include <time.h>
#include "audio.h"
#include "battery.h"
#include "comparator.h"
#include "henrik.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"
#include "vhf.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static float last_known_temperature;
static volatile uint32_t num_clips_stored;
static volatile bool phase_ended, auto_restart_timer;
static uint32_t phase_end_timestamp, vhf_enable_timestamp, led_active_seconds;
static am_hal_timer_config_t audio_processing_timer_config;


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


// Interrupt Service Routines ------------------------------------------------------------------------------------------

void am_rtc_isr(void)
{
   // Clear the RTC interrupt status
   static am_hal_rtc_alarm_repeat_e repeat_interval;
   AM_CRITICAL_BEGIN
   am_hal_rtc_alarm_get(NULL, &repeat_interval);
   am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);
   AM_CRITICAL_END

   // Check if the battery voltage is too low to continue
   battery_result_t battery_details = battery_monitor_get_details();
   last_known_temperature = battery_details.celcius;
   if (battery_details.millivolts <= BATTERY_LOW)
      phase_ended = true;

   // Check if the current phase has ended or if it is time to activate the VHF radio
   const uint32_t current_timestamp = rtc_get_timestamp();
   uint32_t wakeup_timestamp = current_timestamp + MIN_LOG_DATA_INTERVAL_SECONDS;
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
         leds_deinit();
         led_active_seconds = 0;
      }
      else
         wakeup_timestamp = MIN(wakeup_timestamp, config_get_deployment_start_time() + led_active_seconds);
   }

   // TODO: Log relevant device statistics (battery voltage, temperature, UTC timestamp, GPS location, LEDs active, VHF active)
   // TODO: Update storage to always have the log file open

   // Restart the RTC alarm for the next wakeup time
   if (!phase_ended)
      rtc_set_wakeup_timestamp(wakeup_timestamp);
}

void am_timer00_isr(void)
{
   // Clear the timer interrupt and reset the clips stored counter
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE_BOTH));
   num_clips_stored = 0;

   // Automatically restart if requested
   if (auto_restart_timer)
      am_hal_timer_clear(TIMER_NUMBER_AUDIO_PROCESSING);
}

static void henrik_data_available(henrik_data_t new_data)
{
   // Sync RTC to GPS time whenever an update is received
   rtc_set_time_from_timestamp(new_data.utc_timestamp);
}


// Audio Processing Loops ----------------------------------------------------------------------------------------------

static void process_audio_scheduled(uint32_t sampling_rate, uint32_t num_reads_per_clip, time_scale_t align_to, bool interval_based, uint32_t clip_interval_seconds, uint32_t num_schedules, start_end_time_t *schedule)
{
   // Initialize all necessary local variables
   bool audio_clip_in_progress = false, reading_audio = false;
   uint32_t num_audio_reads = 0, last_audio_read_time = 0;
   int16_t *audio_buffer;

   // Handling incoming audio clips until the phase has ended
   while (!phase_ended)
   {
      // Determine if time to create a new WAV file
      const uint32_t current_time = rtc_get_timestamp();
      const uint32_t seconds_til_next_scheduled_recording = seconds_until_next_scheduled_recording(num_schedules, schedule, current_time % 86400);
      if (!audio_clip_in_progress)
      {
         // Go to sleep if there is time until the next scheduled audio recording
         uint32_t seconds_to_sleep = interval_based ? MIN(0, clip_interval_seconds - (current_time - last_audio_read_time)) : seconds_til_next_scheduled_recording;
         seconds_to_sleep = MIN(seconds_to_sleep, phase_end_timestamp - current_time);
         if (seconds_to_sleep)
         {
            audio_processing_timer_config.ui32Compare0 = (uint32_t)(seconds_to_sleep * TIMER_AUDIO_PROCESSING_TICK_RATE);
            am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
            am_hal_timer_clear(TIMER_NUMBER_AUDIO_PROCESSING);
            continue;
         }
         else
         {
            // Generate a file name from the current date and time
            char file_name[32] = { 0 };
            const time_t timestamp = (time_t)current_time;
            strftime(file_name, sizeof(file_name), "%F %H-%M-%S.wav", gmtime(&timestamp));
            if (storage_open(file_name, true))
            {
               // Write the WAV file header contents
               storage_write_wav_header(AUDIO_NUM_CHANNELS, sampling_rate);
               last_audio_read_time = current_time;
               audio_clip_in_progress = true;
               led_indicate_clip_begin();

               // Trigger reading audio samples if currently stopped
               if (!reading_audio)
               {
                  audio_begin_reading(IMMEDIATE);
                  reading_audio = true;
               }
            }
         }
      }

      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Handle any newly available audio data
      if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         led_indicate_clip_progress();
         storage_write(audio_buffer, sizeof(int16_t) * AUDIO_BUFFER_NUM_SAMPLES);
         if (++num_audio_reads >= num_reads_per_clip)
         {
            // Finalize the current WAV file and stop reading if interval-based or if the current schedule has ended
            storage_close();
            if (interval_based || (num_schedules && seconds_til_next_scheduled_recording))
            {
               audio_stop_reading();
               reading_audio = false;
            }
            led_indicate_clip_end();
            audio_clip_in_progress = false;
            num_audio_reads = 0;
         }
      }
   }

   // Ensure that the most recent audio file has been gracefully closed
   led_indicate_clip_end();
   storage_close();
}

static void process_audio_triggered(bool allow_extended_audio_clips, uint32_t sampling_rate, uint32_t num_reads_per_clip, uint32_t max_clips, uint32_t per_num_seconds, float trigger_threshold)
{
   // Initialize all necessary local variables
   bool audio_clip_in_progress = false, awaiting_trigger = false;
   uint32_t num_audio_reads = 0;
   auto_restart_timer = true;
   int16_t *audio_buffer;
   num_clips_stored = 0;

   // Start timer to ensure that no more than "max_clips" are stored during the given number of seconds
   audio_processing_timer_config.ui32Compare0 = (uint32_t)(per_num_seconds * TIMER_AUDIO_PROCESSING_TICK_RATE);
   am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
   am_hal_timer_clear(TIMER_NUMBER_AUDIO_PROCESSING);

   // Handling incoming audio clips until the phase has ended
   while (!phase_ended)
   {
      // Determine if time to start listening for a new audio clip
      if (!awaiting_trigger && !audio_clip_in_progress && (num_clips_stored < max_clips))
      {
         print("Initializing audio read trigger\n");
         audio_begin_reading(COMPARATOR_THRESHOLD);
         awaiting_trigger = true;
      }

      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Handle any newly available audio data
      if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Create a WAV file if this is a new audio clip
         if (!audio_clip_in_progress)
         {
            // Generate a file name from the current date and time
            char file_name[32] = { 0 };
            const time_t timestamp = (time_t)rtc_get_timestamp();
            strftime(file_name, sizeof(file_name), "%F %H-%M-%S.wav", gmtime(&timestamp));
            if (storage_open(file_name, true))
            {
               storage_write_wav_header(AUDIO_NUM_CHANNELS, sampling_rate);
               audio_clip_in_progress = true;
               led_indicate_clip_begin();
            }
         }

         // Store the audio data to the WAV file
         print("New audio data available: %u\n", num_audio_reads+1);
         led_indicate_clip_progress();
         storage_write(audio_buffer, sizeof(int16_t) * AUDIO_BUFFER_NUM_SAMPLES);
         if (++num_audio_reads >= num_reads_per_clip)
         {
            print("Full audio clip processed...\n");
            awaiting_trigger = audio_clip_in_progress = false;
            led_indicate_clip_end();
            audio_stop_reading();
            num_audio_reads = 0;
            ++num_clips_stored;
            storage_close();
         }
      }
   }

   // Ensure that the most recent audio file has been gracefully closed
   led_indicate_clip_end();
   storage_close();
}


// Public Main Function ------------------------------------------------------------------------------------------------

void active_main(volatile bool *device_actived, int32_t phase_index)
{
   // Ensure that a storage directory with the device name exists and is active on the SD card
   print("INFO: Starting main deployment phase activity...\n");
   print("INFO: Validating existence of SD card storage directory...");
   char device_label[MAX_DEVICE_LABEL_LEN] = { 0 };
   config_get_device_label(device_label, sizeof(device_label));
   if (device_label[0] == '\0')
      memcpy(device_label, "Default", sizeof("Default"));
   bool success = storage_mkdir(device_label) && storage_chdir(device_label);
   print("%s\n", success ? "SUCCESS" : "FAILURE");

   // Determine when the LEDs should be disabled, if not already
   uint32_t current_timestamp = rtc_get_timestamp();
   uint32_t wakeup_timestamp = current_timestamp + MIN_LOG_DATA_INTERVAL_SECONDS;
   led_active_seconds = config_get_leds_active_seconds();
   if (led_active_seconds)
   {
      if ((current_timestamp - config_get_deployment_start_time()) >= led_active_seconds)
      {
         led_active_seconds = 0;
         leds_enable(false);
      }
      else
         wakeup_timestamp = MIN(wakeup_timestamp, config_get_deployment_start_time() + led_active_seconds);
   }

   // Determine when the VHF radio should activate, if not already
   vhf_enable_timestamp = config_get_vhf_start_timestamp();
   if (vhf_enable_timestamp && (current_timestamp < vhf_enable_timestamp))
      wakeup_timestamp = MIN(wakeup_timestamp, vhf_enable_timestamp);
   else
      vhf_enable_timestamp = 0;

   // Initialize the audio processing timer
   am_hal_timer_default_config_set(&audio_processing_timer_config);
   audio_processing_timer_config.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV4K;
   audio_processing_timer_config.ui32Compare0 = (uint32_t)(600 * TIMER_AUDIO_PROCESSING_TICK_RATE);
   am_hal_timer_config(TIMER_NUMBER_AUDIO_PROCESSING, &audio_processing_timer_config);
   am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE0));
   NVIC_SetPriority(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING, AUDIO_TIMER_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING);

   // Listen for incoming data messages from the Henrik board
   if (config_gps_available())
      henrik_register_data_callback(henrik_data_available);

   // Set an RTC alarm to expire when the next important event is expected to happen
   phase_end_timestamp = config_get_end_time(phase_index);
   phase_ended = phase_end_timestamp <= current_timestamp;
   if (!phase_ended)
   {
      wakeup_timestamp = MIN(wakeup_timestamp, phase_end_timestamp);
      rtc_set_wakeup_timestamp(wakeup_timestamp);
   }

   // TODO: Implement IMU recording stuff
   /*const uint32_t imu_sampling_rate_hz = config_get_imu_sampling_rate_hz(phase_index);
   switch (config_get_imu_recording_mode(phase_index))
   {
      case ACTIVITY:
      {
         float motion_trigger_threshold = config_get_imu_trigger_threshold_level(phase_index);
         uint8_t degrees_of_freedom = config_get_imu_degrees_of_freedom(phase_index);
         break;
      }
      case AUDIO:   // Intentional fall-through
      default:
         break;
   }*/

   // Determine how to schedule audio clip recordings
   auto_restart_timer = false;
   const uint32_t audio_sampling_rate_hz = config_get_audio_sampling_rate_hz(phase_index);
   audio_init(AUDIO_NUM_CHANNELS, audio_sampling_rate_hz, config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE);
   const uint32_t num_reads_per_clip = audio_num_reads_per_n_seconds(config_get_audio_clip_length_seconds(phase_index));
   switch (config_get_audio_recording_mode(phase_index))
   {
      case AMPLITUDE:
      {
         time_scale_t unit_time;
         uint32_t max_num_clips, max_clips_interval_seconds;
         config_get_max_audio_clips(phase_index, &max_num_clips, &unit_time);
         float audio_trigger_threshold = config_get_audio_trigger_threshold(phase_index);
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
         process_audio_triggered(allow_extended_audio_clips, audio_sampling_rate_hz, num_reads_per_clip, max_num_clips, max_clips_interval_seconds, audio_trigger_threshold);
         break;
      }
      case SCHEDULED:
      {
         start_end_time_t *schedule;
         uint32_t num_schedules = config_get_audio_trigger_schedule(phase_index, &schedule);
         process_audio_scheduled(audio_sampling_rate_hz, num_reads_per_clip, SECONDS, false, 0, num_schedules, schedule);
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
         process_audio_scheduled(audio_sampling_rate_hz, num_reads_per_clip, unit_time, true, audio_recording_interval, 0, NULL);
         break;
      }
      case CONTINUOUS:  // Intentional fall-through
      default:
         process_audio_scheduled(audio_sampling_rate_hz, num_reads_per_clip, SECONDS, false, 0, 0, NULL);
         break;
   }

   // Stop all running timers
   am_hal_rtc_interrupt_disable(AM_HAL_RTC_INT_ALM);
   am_hal_timer_disable(TIMER_NUMBER_AUDIO_PROCESSING);
   NVIC_DisableIRQ(TIMER0_IRQn + TIMER_NUMBER_AUDIO_PROCESSING);
   am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(TIMER_NUMBER_AUDIO_PROCESSING, AM_HAL_TIMER_COMPARE0));
}


// TODO: Re-route "print" to SD card if not in debug mode (if logfile closed, ignore print, can we keep logfile open while WAV file is also open?)
// TODO: Reset if RTC is not increasing on each tick (do we need a watchdog for this?)
