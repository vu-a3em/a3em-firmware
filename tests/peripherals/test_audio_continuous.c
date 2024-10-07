#include "audio.h"
#include "logging.h"
#include "storage.h"
#include "system.h"

// Edit this definition to change the length of audio to store continuously
#define DESIRED_CLIP_LENGTH_SECONDS     60

int main(void)
{
   // Set up the system hardware
   setup_hardware();
   storage_init();
   audio_init(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, 35.0f, AUDIO_MIC_BIAS_VOLTAGE);
   system_enable_interrupts(true);

   // Open a new WAV file and immediately begin reading continuous audio
   uint32_t num_audio_reads = 0;
   if (storage_open("wav_test.wav", true) && storage_write_wav_header(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ))
      print("Opening \"wav_test.wav\" for writing\n");
   else
      print("ERROR: Unable to open WAV file for writing\n");
   audio_begin_reading(IMMEDIATE);

   // Loop forever handling incoming audio clips
   const uint32_t num_reads_per_clip = audio_num_reads_per_n_seconds(DESIRED_CLIP_LENGTH_SECONDS);
   int16_t *audio_buffer;
   while (true)
   {
      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Store any newly available audio data
      if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         bool success = storage_write(audio_buffer, sizeof(int16_t) * AUDIO_BUFFER_NUM_SAMPLES);
         print("%s: %u\n", success ? "Wrote audio data" : "ERROR writing audio data", num_audio_reads+1);
         if (++num_audio_reads >= num_reads_per_clip)
         {
            // Stop reading audio and close the WAV file
            audio_stop_reading();
            storage_close();
            print("WAV file successfully written!\n");
         }
      }
   }

   // Should never reach this point
   system_reset();
   return 0;
}
