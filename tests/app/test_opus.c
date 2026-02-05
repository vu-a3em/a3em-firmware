#include "audio.h"
#include "logging.h"
#include "opus_config.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"

#define TIMER_NUMBER                            7

#define AUDIO_SAMPLING_RATE                     OPUS_REQUIRED_SAMPLE_RATE_HZ
#define AUDIO_GAIN_DB                           25.0
#define AUDIO_CLIP_LENGTH_SECONDS               AUDIO_DEFAULT_CLIP_LENGTH_SECONDS
#define AUDIO_MIC_TYPE                          MIC_DIGITAL

#define OPUS_BITRATE                            OPUS_DEFAULT_ENCODING_BITRATE

#define TEST_WITH_STORAGE

static volatile bool device_activated = true;

int main(void)
{
   // Set up the system hardware
   setup_hardware();
   rtc_init();
   rtc_set_time_to_compile_time();
   if (AUDIO_MIC_TYPE == MIC_ANALOG)
      audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_CLIP_LENGTH_SECONDS, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, &device_activated);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_CLIP_LENGTH_SECONDS, AUDIO_GAIN_DB);
   system_enable_interrupts(true);
#ifdef TEST_WITH_STORAGE
   storage_init();
   storage_mkdir("TestOpus");
#endif

   // Create a timer to measure code performance
   am_hal_timer_config_t timer_config;
   am_hal_timer_default_config_set(&timer_config);
   am_hal_timer_config(TIMER_NUMBER, &timer_config);

   // Initialize the Opus encoder
   const uint32_t num_audio_reads_per_clip = AUDIO_CLIP_LENGTH_SECONDS / audio_num_seconds_per_dma();
   const uint32_t audio_samples_per_dma = audio_num_seconds_per_dma() * AUDIO_SAMPLING_RATE;
   opusenc_init(OPUS_BITRATE);

   // Loop forever handling incoming audio clips
   bool audio_clip_in_progress = false;
   uint32_t num_audio_reads = 0;
   int16_t *audio_buffer;
   while (true)
   {
      // Determine if time to start listening for a new audio clip
      if (!audio_clip_in_progress)
      {
         printonly("Initializing audio read trigger\n");
         audio_begin_reading();
         audio_clip_in_progress = true;
#ifdef TEST_WITH_STORAGE
         if (!storage_open_audio_file(1, "TestOpus", AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, rtc_get_timestamp(), true))
            printonly("ERROR: Unable to create a new Ogg Opus file on the SD card!\n");
#endif
      }

      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Handle any newly available audio data
      if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Encode the audio data
         printonly("New audio data available: %u\n", num_audio_reads+1);
         am_hal_timer_clear(TIMER_NUMBER);
#ifdef TEST_WITH_STORAGE
         if (!storage_write_audio(audio_buffer, sizeof(int16_t) * audio_samples_per_dma, (num_audio_reads + 1) >= num_audio_reads_per_clip))
            printonly("ERROR: Unable to write to Ogg Opus file!\n");
#else
         opusenc_encode(audio_buffer, audio_samples_per_dma, &result_begin, &result_end);
#endif

         // Print out the execution time of the encoding process
         const uint32_t timer_val = am_hal_timer_read(TIMER_NUMBER);
         am_hal_timer_stop(TIMER_NUMBER);
         printonly("Execution time: %u ms\n", (uint32_t)(((float)timer_val / (AM_HAL_CLKGEN_FREQ_MAX_HZ / 16)) * 1000.0f));

         // Finalize the audio clip if done reading
         if (++num_audio_reads >= num_audio_reads_per_clip)
         {
            printonly("Full audio clip processed...stopping reading\n");
            audio_clip_in_progress = false;
            storage_close_audio();
            audio_stop_reading();
            num_audio_reads = 0;
         }
      }
   }

   // Should never reach this point
   system_reset();
   return 0;
}
