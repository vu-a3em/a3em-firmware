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
#define AUDIO_NUM_READS_PER_CLIP                (AUDIO_CLIP_LENGTH_SECONDS / AUDIO_BUFFER_NUM_SECONDS)

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
      audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, &device_activated);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_GAIN_DB);
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
   opusenc_init(OPUS_BITRATE);

   // Loop forever handling incoming audio clips
   static const opus_frame_t *result_begin, *result_end;
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
         if (!storage_open_ogg_opus_file(1, "TestOpus", rtc_get_timestamp()))
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
         opusenc_encode(audio_buffer, &result_begin, &result_end);

         // Encapsulate each resulting Opus frame into an Ogg page and store
#ifdef TEST_WITH_STORAGE
         for (const opus_frame_t *frame = result_begin; frame != result_end; frame = frame->next)
         {
            const uint8_t is_last = ((num_audio_reads + 1) >= AUDIO_NUM_READS_PER_CLIP) && (frame->next == result_end);
            if (!storage_write_ogg_opus_audio(frame->encoded_data, frame->num_encoded_bytes, is_last))
               printonly("ERROR: Unable to write to Ogg Opus file!\n");
         }
#endif

         // Print out the execution time of the encoding process
         const uint32_t timer_val = am_hal_timer_read(TIMER_NUMBER);
         am_hal_timer_stop(TIMER_NUMBER);
         printonly("Execution time: %u ms\n", (uint32_t)(((float)timer_val / (AM_HAL_CLKGEN_FREQ_MAX_HZ / 16)) * 1000.0f));

         // Finalize the audio clip if done reading
         if (AUDIO_NUM_READS_PER_CLIP && (++num_audio_reads >= AUDIO_NUM_READS_PER_CLIP))
         {
            printonly("Full audio clip processed...stopping reading\n");
            audio_clip_in_progress = false;
            storage_close_ogg_opus_audio();
            audio_stop_reading();
            num_audio_reads = 0;
         }
      }
   }

   // Should never reach this point
   system_reset();
   return 0;
}
