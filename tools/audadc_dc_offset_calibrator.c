#include "audio.h"
#include "logging.h"
#include "system.h"

int main(void)
{
   // Set up the system hardware
   setup_hardware();
   audio_init(AUDIO_NUM_CHANNELS, 40000, 35.0f, AUDIO_MIC_BIAS_VOLTAGE);
   system_enable_interrupts(true);

   // Read audio for long enough to skip startup noise and determine the DC offset
   const uint32_t num_reads_per_skip = audio_num_reads_per_n_seconds(5);
   const uint32_t num_reads_per_clip = audio_num_reads_per_n_seconds(10);
   audio_begin_reading(IMMEDIATE);

   // Skip first five seconds of audio
   int16_t *audio_buffer;
   print("Skipping first 5 seconds of audio...\n");
   for (uint32_t num_reads = 0; num_reads < num_reads_per_skip; )
   {
      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Process any newly available audio data
      if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
         ++num_reads;
   }

   // Process next ten seconds of audio forever
   while (true)
   {
      int64_t average = 0;
      print("Processing next 10 seconds of audio...\n");
      for (uint32_t num_reads = 0; num_reads < num_reads_per_clip; )
      {
         // Sleep while no errors or audio to process
         if (audio_error_encountered())
            system_reset();
         else if (!audio_data_available())
            system_enter_deep_sleep_mode();

         // Process any newly available audio data
         if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
         {
            ++num_reads;
            for (int i = 0; i < AUDIO_BUFFER_NUM_SAMPLES; ++i)
               average += (int64_t)audio_buffer[i];
         }
      }

      // Output the measured DC offset and store to persistent memory
      average /= (num_reads_per_clip * AUDIO_BUFFER_NUM_SAMPLES);
      print("Measured DC Offset: %d\n", (int32_t)average);
      bool success = mram_store_audadc_dc_offset(average);
      print("Storage to persistent memory %s!\n", success ? "SUCCESSFUL" : "FAILED");
   }

   // Should never reach this point
   system_reset();
   return 0;
}
