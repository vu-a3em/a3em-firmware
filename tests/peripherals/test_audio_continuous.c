#include "audio.h"
#include "logging.h"
#include "storage.h"
#include "system.h"

// Edit this definition to change the length of audio to store continuously
#define AUDIO_MIC_TYPE                  MIC_DIGITAL
#define DESIRED_CLIP_LENGTH_SECONDS     10
#define AUDIO_GAIN_DB                   35.0f

int main(void)
{
   // Set up the system hardware
   setup_hardware();
   storage_init();
   if (AUDIO_MIC_TYPE == MIC_ANALOG)
      audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, AUDIO_GAIN_DB);
   system_enable_interrupts(true);

   // Open a new WAV file and immediately begin reading continuous audio
   uint32_t num_audio_reads = 0;
   if (storage_mkdir("wav_test") && storage_open_wav_file("wav_test", AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, 1729274454))
      print("Opening \"wav_test.wav\" for writing\n");
   else
      print("ERROR: Unable to open WAV file for writing\n");
   audio_begin_reading();

   // Loop forever handling incoming audio clips
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
         bool success = storage_write_audio(audio_buffer, sizeof(int16_t) * AUDIO_DEFAULT_SAMPLING_RATE_HZ);
         print("%s: %u\n", success ? "Wrote audio data" : "ERROR writing audio data", num_audio_reads+1);
         if (++num_audio_reads >= DESIRED_CLIP_LENGTH_SECONDS)
         {
            // Stop reading audio and close the WAV file
            audio_stop_reading();
            storage_close_audio();
            print("WAV file successfully written!\n");
         }
      }
   }

   // Should never reach this point
   system_reset();
   return 0;
}
