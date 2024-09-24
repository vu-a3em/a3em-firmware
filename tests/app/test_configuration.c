#include <time.h>
#include "logging.h"
#include "storage.h"
#include "system.h"

int main(void)
{
   // Set up the system hardware
   setup_hardware();
   storage_init();
   system_enable_interrupts(true);

   // Read and print out the contents of the runtime configuration file
   char label[32];
   print("Reading configuration file...%s\n\n", fetch_runtime_configuration() ? "SUCCESS" : "FAILURE");
   config_get_device_label(label, sizeof(label));
   print("Device Label: %s\n", label);
   time_t start_time = (time_t)config_get_deployment_start_time();
   if (start_time > 0)
   {
      char buffer[32];
      struct tm *utc_time = gmtime(&start_time);
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", utc_time);
      print("Deployment Start Time: %s\n", buffer);
   }
   else
      print("Deployment Start Time: Immediate\n");
   time_t end_time = (time_t)config_get_deployment_end_time();
   if (end_time > 0)
   {
      char buffer[32];
      struct tm *utc_time = gmtime(&end_time);
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", utc_time);
      print("Deployment End Time: %s\n", buffer);
   }
   else
      print("Deployment End Time: Unspecified\n");
   print("RTC set to Deployment Start Time upon Magnet Detect: %s\n", config_set_rtc_at_magnet_detect() ? "Yes" : "No");
   if (config_get_leds_enabled())
   {
      uint32_t active_seconds = config_get_leds_active_seconds();
      print("LEDs Enabled: Yes\n");
      if (active_seconds > 0)
         print("   Active for %u seconds\n", active_seconds);
      else
         print("   Active indefinitely\n");
   }
   else
      print("LEDs Enabled: No\n");
   time_t vhf_time = (time_t)config_get_vhf_start_timestamp();
   if (vhf_time > 0)
   {
      char buffer[32];
      struct tm *utc_time = gmtime(&vhf_time);
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", utc_time);
      print("VHF Transmitter Enabled: %s\n", buffer);
   }
   else
      print("VHF Transmitter Enabled: Never\n");
   switch (config_get_mic_amplification())
   {
      case LOW:
         print("Microphone Amplification: Low\n");
         break;
      case MEDIUM:
         print("Microphone Amplification: Medium\n");
         break;
      default:
         print("Microphone Amplification: High\n");
         break;
   }
   int32_t num_phases = config_get_num_deployment_phases();
   for (int32_t i = 0; (num_phases != -1) && (i < num_phases); ++i)
   {
      print("\nDeployment Phase:\n");
      start_time = config_get_start_time(i);
      if (start_time > 0)
      {
         char buffer[32];
         struct tm *utc_time = gmtime(&start_time);
         strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", utc_time);
         print("Start Time: %s\n", buffer);
      }
      else
         print("Start Time: Immediate\n");
      end_time = config_get_end_time(i);
      if (end_time > 0)
      {
         char buffer[32];
         struct tm *utc_time = gmtime(&end_time);
         strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", utc_time);
         print("End Time: %s\n", buffer);
      }
      else
         print("End Time: Unspecified\n");
      switch (config_get_audio_recording_mode(i))
      {
         case AMPLITUDE:
         {
            uint32_t max_clips;
            time_scale_t unit_time;
            config_get_max_audio_clips(i, &max_clips, &unit_time);
            print("Audio Recording Mode: Amplitude\n");
            print("   Max Audio Clips: %u per ", max_clips);
            switch (unit_time)
            {
               case SECONDS:
                  print("second\n");
                  break;
               case MINUTES:
                  print("minute\n");
                  break;
               case HOURS:
                  print("hour\n");
                  break;
               default:
                  print("day\n");
                  break;
            }
            print("   Trigger Threshold: %.2f\n", config_get_audio_trigger_threshold(i));
            break;
         }
         case SCHEDULED:
         {
            start_end_time_t *schedule;
            uint32_t num_schedules = config_get_audio_trigger_schedule(i, &schedule);
            print("Audio Recording Mode: Scheduled\n");
            for (uint32_t i = 0; i < num_schedules; ++i)
               print("   Recording: %02u:%02u:%02u - %02u:%02u:%02u\n",
                     schedule[i].start_time / 3600, (schedule[i].start_time / 60) % 60, schedule[i].start_time % 60,
                     schedule[i].end_time / 3600, (schedule[i].end_time / 60) % 60, schedule[i].end_time % 60);
            break;
         }
         case INTERVAL:
         {
            uint32_t interval;
            time_scale_t unit_time;
            config_get_audio_trigger_interval(i, &interval, &unit_time);
            print("Audio Recording Mode: Interval\n");
            print("   Recording: Once every %u ", interval);
            switch (unit_time)
            {
               case SECONDS:
                  print("second(s)\n");
                  break;
               case MINUTES:
                  print("minute(s)\n");
                  break;
               case HOURS:
                  print("hour(s)\n");
                  break;
               default:
                  print("day(s)\n");
                  break;
            }
            break;
         }
         default:
            print("Audio Recording Mode: Continuous\n");
            break;
      }
      print("Audio Sampling Rate: %u Hz\n", config_get_audio_sampling_rate_hz(i));
      print("Audio Clip Length: %u seconds\n", config_get_audio_clip_length_seconds(i));
      print("Extend Clip for Continuing Audio: %s\n", config_extend_clip_for_continuous_audio(i) ? "Yes" : "No");
      print("IMU Degrees of Freedom: %u\n", (uint32_t)config_get_imu_degrees_of_freedom(i));
      switch (config_get_imu_recording_mode(i))
      {
         case ACTIVITY:
            print("IMU Recording Mode: Activity-Based\n");
            print("   Activity Trigger Threshold: %.2f\n", config_get_imu_trigger_threshold_level(i));
            break;
         default:
            print("IMU Recording Mode: Audio-Synced\n");
            break;
      }
      print("IMU Sampling Rate: %u Hz\n", config_get_imu_sampling_rate_hz(i));
   }

   // Go to sleep forever
   while (true)
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

   // Should never reach this point
   system_reset();
   return 0;
}
