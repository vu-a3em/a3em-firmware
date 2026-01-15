#include "audio.h"
#include "logging.h"
#include "system.h"
#include "opus_config.h"

#define AUDIO_SAMPLING_RATE                     OPUS_REQUIRED_SAMPLE_RATE_HZ
#define AUDIO_GAIN_DB                           25.0
#define AUDIO_CLIP_LENGTH_SECONDS               AUDIO_DEFAULT_CLIP_LENGTH_SECONDS
#define AUDIO_MIC_TYPE                          MIC_DIGITAL

#define OPUS_BITRATE                            OPUS_DEFAULT_ENCODING_BITRATE

static volatile bool device_activated = true;

int main(void)
{
   // Set up the system hardware
   setup_hardware();
   if (AUDIO_MIC_TYPE == MIC_ANALOG)
      audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, &device_activated);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLING_RATE, AUDIO_GAIN_DB);
   system_enable_interrupts(true);

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

         // TODO: Write the encoded data to storage

         // Finalize the audio clip if done reading
         if (AUDIO_CLIP_LENGTH_SECONDS && (++num_audio_reads >= AUDIO_CLIP_LENGTH_SECONDS))
         {
            printonly("Full audio clip processed...stopping reading\n");
            // TODO: Close Opus file
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
