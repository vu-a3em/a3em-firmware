#include "audio.h"
#include "comparator.h"
#include "logging.h"
#include "system.h"

#define TRIGGER_IMMEDIATE    IMMEDIATE
#define TRIGGER_COMPARATOR   COMPARATOR_THRESHOLD

// Change this definition to either constantly trigger audio reads or wait for a certain loudness
#define AUDIO_READ_TRIGGER                      TRIGGER_IMMEDIATE
#define AUDIO_GAIN_DB                           35.0        // Max of 45.0
#define AUDIO_TRIGGER_THRESHOLD_PERCENT         0.5
#define AUDIO_CLIP_LENGTH_SECONDS               10

int main(void)
{
   // Set up the system hardware
   setup_hardware();
   audio_init(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE);
   comparator_init(false, 0, AUDIO_TRIGGER_THRESHOLD_PERCENT, true);
   system_enable_interrupts(true);

   // Loop forever handling incoming audio clips
   const uint32_t num_reads_per_clip = audio_num_reads_per_n_seconds(AUDIO_CLIP_LENGTH_SECONDS);
   static int16_t audio_buffer[AUDIO_BUFFER_NUM_SAMPLES];
   bool audio_clip_in_progress = false;
   uint32_t num_audio_reads = 0;
   while (true)
   {
      // Determine if time to start listening for a new audio clip
      if (!audio_clip_in_progress)
      {
         print("Initializing audio read trigger\n");
         audio_begin_reading(AUDIO_READ_TRIGGER);
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
         if (AUDIO_CLIP_LENGTH_SECONDS && (++num_audio_reads >= num_reads_per_clip))
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