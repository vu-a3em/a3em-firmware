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

static volatile bool phase_ended;

/*static void timer_isr??(void)
{
   // TODO: Use this to force us to stop the current phase AND/OR to enable the VHF (config_get_vhf_start_timestamp()

   // Determine if the VHF radio should already be activated
   uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
   if (vhf_enable_timestamp && (rtc_get_timestamp() >= vhf_enable_timestamp))
      vhf_activate();

   // TODO:
   phase_ended = true;
}*/

static uint32_t seconds_until_next_scheduled_recording(uint32_t num_schedules, start_end_time_t *schedule, uint32_t current_seconds_of_day)
{
   // Return the number of seconds until the next scheduling phase or 0 if already in an active phase
   uint32_t current_index = 0;
   while ((current_index < num_schedules) && (current_seconds_of_day >= schedule[current_index].end_time))
      ++current_index;
   if (current_index < num_schedules)
      return (schedule[current_index].start_time <= current_seconds_of_day) ? 0 : (schedule[current_index].start_time - current_seconds_of_day);
   return num_schedules ? (86400 - current_seconds_of_day + schedule[0].start_time) : 0;
}

static void process_audio(uint32_t sampling_rate, uint32_t num_reads_per_clip, time_scale_t align_to, bool interval_based, uint32_t clip_interval_seconds, uint32_t num_schedules, start_end_time_t *schedule)
{
   // Initialize all necessary local variables
   bool audio_clip_in_progress = false, reading_audio = false;
   uint32_t num_audio_reads = 0, last_audio_read_time = 0;
   int16_t audio_buffer[AUDIO_BUFFER_NUM_SAMPLES];

   // Handling incoming audio clips until the phase has ended
   while (!phase_ended)
   {
      // Determine if time to create a new WAV file
      // TODO: Align recording start to the top of the time-interval specified with respect to RTC time
      uint32_t current_time = rtc_get_timestamp();
      uint32_t seconds_til_next_scheduled_recording = num_schedules ? seconds_until_next_scheduled_recording(num_schedules, schedule, current_time % 86400) : 0xFFFFFFFF;
      bool time_for_next_recording = (!interval_based && !num_schedules) ||
                                     (interval_based && ((last_audio_read_time + clip_interval_seconds) <= current_time)) ||
                                     !seconds_til_next_scheduled_recording;
      if (!audio_clip_in_progress && time_for_next_recording)
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

            // Trigger reading audio samples if currently stopped
            if (!reading_audio)
            {
               audio_begin_reading(IMMEDIATE);
               reading_audio = true;
            }
         }
      }

      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode(); // TODO: Only sleep for remaining number of seconds

      // Handle any newly available audio data
      if (audio_data_available() && audio_read_data(audio_buffer))
      {
         storage_write(audio_buffer, sizeof(audio_buffer));
         if (++num_audio_reads >= num_reads_per_clip)
         {
            // Finalize the current WAV file and stop reading if interval-based or if the current schedule has ended
            storage_close();
            if (interval_based || (num_schedules && seconds_til_next_scheduled_recording))
            {
               audio_stop_reading();
               reading_audio = false;
               // TODO: Sleep for remaining seconds until next interval time
            }
            audio_clip_in_progress = false;
            num_audio_reads = 0;
         }
      }
   }

   // Ensure that the most recent audio file has been gracefully closed
   storage_close();
}

static void process_audio_triggered(bool allow_extended_audio_clips, uint32_t sampling_rate, uint32_t num_reads_per_clip, uint32_t max_clips, uint32_t per_num_seconds, float trigger_threshold)
{
   // TODO: Start timer at now that expires every "max_clips_interval_seconds", ensure that no more than "max_num_clips" clips are stored during each timer interval
   // TODO: FULLY IMPLEMENT THIS
   // Initialize all necessary local variables
   int16_t audio_buffer[AUDIO_BUFFER_NUM_SAMPLES];
   bool audio_clip_in_progress = false;
   uint32_t num_audio_reads = 0;

   // Handling incoming audio clips until the phase has ended
   while (!phase_ended)
   {
      // Determine if time to start listening for a new audio clip
      if (!audio_clip_in_progress)
      {
         print("Initializing audio read trigger\n");
         audio_begin_reading(COMPARATOR_THRESHOLD);
         audio_clip_in_progress = true;
      }

      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Handle any newly available audio data
      if (audio_data_available() && audio_read_data(audio_buffer))
      {
         print("New audio data available: %u\n", num_audio_reads+1);
         // TODO: Do something with the audio
         if (++num_audio_reads >= num_reads_per_clip)
         {
            print("Full audio clip processed...\n");
            // Create new WAV file
            audio_clip_in_progress = false;
            audio_stop_reading();
            num_audio_reads = 0;
         }
      }
   }

   // Ensure that the most recent audio file has been gracefully closed
   storage_close();
}

void active_main(int32_t phase_index)
{
   // Ensure that a storage directory with the device name exists and is active on the SD card
   phase_ended = false;
   print("INFO: Starting main deployment phase activity...\n");
   print("INFO: Validating existence of SD card storage directory...");
   char device_label[MAX_DEVICE_LABEL_LEN];
   config_get_device_label(device_label, sizeof(device_label));
   print("%s\n", (storage_mkdir(device_label) && storage_chdir(device_label)) ? "SUCCESS" : "FAILURE");

   // Configure all phase-specific peripherals
   if (config_get_leds_enabled())
   {
      if ((rtc_get_timestamp() - config_get_deployment_start_time()) > config_get_leds_active_seconds())
         leds_deinit();
      else
         ;// TODO: Set a timer to expire when it's time to turn off the LEDs
   }

   // TODO: Set a timer to expire when this deployment phase ends
   //const uint32_t phase_end_time = config_get_end_time(phase_index);

   // TODO: Set a timer to expire when the VHF radio should be activated, if prior to this phase ending
   uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
   if (vhf_enable_timestamp && (vhf_enable_timestamp < config_get_deployment_end_time()) && (rtc_get_timestamp() < vhf_enable_timestamp))
      ;

   // TODO: Start a timer to synchronize the RTC to GPS once per day
   if (config_gps_available())
      ;

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
   audio_init(AUDIO_NUM_CHANNELS, config_get_audio_sampling_rate_hz(phase_index), config_get_mic_amplification_db(), AUDIO_MIC_BIAS_VOLTAGE);
   const uint32_t num_reads_per_clip = audio_num_reads_per_n_seconds(config_get_audio_clip_length_seconds(phase_index));
   const uint32_t audio_sampling_rate_hz = config_get_audio_sampling_rate_hz(phase_index);
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
         process_audio(audio_sampling_rate_hz, num_reads_per_clip, SECONDS, false, 0, num_schedules, schedule);
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
         process_audio(audio_sampling_rate_hz, num_reads_per_clip, unit_time, true, audio_recording_interval, 0, NULL);
         break;
      }
      case CONTINUOUS:  // Intentional fall-through
      default:
         process_audio(audio_sampling_rate_hz, num_reads_per_clip, SECONDS, false, 0, 0, NULL);
         break;
   }
}


// TODO: GUI Updates: Add GPS Available, Awake on Magnet, fix IMU threshold values, make mic amp levels dB [0, 45]
// TODO: GUI Updates: ensure for interval audio reading that interval is >= clip_length_seconds, ensure all datetimes make logical sense before storing
// TODO: Test that VHF stays active in power off mode
// TODO: Re-route "print" to SD card if not in debug mode (if logfile closed, ignore print)
// TODO: Reset if RTC is not increasing on each tick (do we need a watchdog for this?)
// TODO: Try to use DMA audio buffer directly to cut down on memory usage
// TODO: Fix audio to only store/deal with mono output
// TODO: Add battery monitoring, shut down if too low
