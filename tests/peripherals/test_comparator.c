#include "audio.h"
#include "comparator.h"
#include "logging.h"
#include "system.h"

#define AUDIO_GAIN_DB                       35.0        // Max of 45.0
#define AUDIO_TRIGGER_THRESHOLD_PERCENT     0.25

static volatile bool device_activated = true;

int main(void)
{
   // Set up system hardware
   setup_hardware();
   audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, 1, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, COMPARATOR_THRESHOLD, AUDIO_TRIGGER_THRESHOLD_PERCENT, &device_activated);
   system_enable_interrupts(true);

   // Sleep while the comparator threshold has not been breached
   int i = 0;
   audio_begin_reading();
   while (true)
   {
      if (comparator_triggered())
      {
         print("Comparator trigger count: %d\n", ++i);
         comparator_reset();
      }
      else
         system_enter_deep_sleep_mode();
   }

   // Should never reach this point
   return 0;
}
