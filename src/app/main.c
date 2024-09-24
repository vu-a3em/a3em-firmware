#include "audio.h"
#include "comparator.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"
#include "vhf.h"

int main(void)
{
   // Set up the system hardware and retrieve the device ID
   setup_hardware();
   static uint8_t device_id[DEVICE_ID_LEN];
   system_read_ID(device_id, sizeof(device_id));
   print("System hardware initialized, UID = ");
   for (size_t i = DEVICE_ID_LEN - 1; i > 0; --i)
      print("%02X:", device_id[i]);
   print("%02X\n", device_id[0]);

   // Initialize required peripherals and start up the RTC
   print("Initializing peripherals...\n");
   leds_init();
   rtc_init();
   rtc_set_time_to_compile_time(); // TODO: We don't actually want to do this on every reboot
   storage_init();

   // Retrieve the runtime configuration from storage
   print("Fetching runtime configuration...%s\n",
         fetch_runtime_configuration() ? "SUCCESS" : "FAILURE (Using default configuration)");

   // Initialize all remaining peripherals
   vhf_init();
   imu_init();
   magnet_sensor_init();
   audio_init(AUDIO_NUM_CHANNELS, config_get_audio_sampling_rate_hz(), AUDIO_MIC_BIAS_VOLTAGE);
   comparator_init(false, 0, 0.5, true);
   system_enable_interrupts(true);
   print("All peripherals initialized!\n");

   // Loop forever handling incoming audio clips
   const uint32_t num_reads_per_clip = audio_num_reads_per_n_seconds(config_get_audio_clip_length_seconds());
   static int16_t audio_buffer[2*AUDIO_BUFFER_NUM_SAMPLES];
   bool audio_clip_in_progress = false;
   uint32_t num_audio_reads = 0;
   while (true)
   {
      // Determine if time to start listening for a new audio clip
      if (!audio_clip_in_progress)
      {
         print("Initializing audio read trigger\n");
         audio_begin_reading(AUDIO_BUFFER_READ_TRIGGER);
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
         transmit_audio(audio_buffer, sizeof(audio_buffer));
         if (++num_audio_reads >= num_reads_per_clip)
         {
            print("Full audio clip processed...stopping reading\n");
            audio_clip_in_progress = false;
            audio_stop_reading();
            num_audio_reads = 0;
         }
      }
   }

   // Should never reach this point
   system_reset();
   return 0;
}
