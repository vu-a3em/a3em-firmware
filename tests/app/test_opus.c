#include "audio.h"
#include "logging.h"
#include "opus_config.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"

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
      audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, &device_activated);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_GAIN_DB);
   system_enable_interrupts(true);
#ifdef TEST_WITH_STORAGE
   storage_init();
#endif

   // Initialize the Opus encoder
   opusenc_init(OPUS_BITRATE);

   // Loop forever handling incoming audio clips
   static const opus_frame_t *result_begin, *result_end;
   static int16_t audio_buffer[AUDIO_SAMPLING_RATE];
   bool audio_clip_in_progress = false;
   uint32_t num_audio_reads = 0;
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
            print("ERROR: Unable to create a new Ogg Opus file on the SD card!\n");
#endif
      }

      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Handle any newly available audio data
      if (audio_data_available() && audio_read_data(audio_buffer))
      {
         // Encode the audio data
         printonly("New audio data available: %u\n", num_audio_reads+1);
         opusenc_encode(audio_buffer, &result_begin, &result_end);

         // Encapsulate each resulting Opus frame into an Ogg page and store
#ifdef TEST_WITH_STORAGE
         for (const opus_frame_t *frame = result_begin; frame != result_end; frame = frame->next)
         {
            const uint8_t is_last = ((num_audio_reads + 1) >= AUDIO_CLIP_LENGTH_SECONDS) && (frame->next == result_end);
            if (!storage_write_ogg_opus_audio(frame->encoded_data, frame->num_encoded_bytes, is_last))
               print("ERROR: Unable to write to Ogg Opus file!\n");
         }
#endif

         // Finalize the audio clip if done reading
         if (AUDIO_CLIP_LENGTH_SECONDS && (++num_audio_reads >= AUDIO_CLIP_LENGTH_SECONDS))
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
