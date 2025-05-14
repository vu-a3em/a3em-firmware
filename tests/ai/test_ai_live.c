#include <math.h>
#include "ai.h"
#include "audio.h"
#include "logging.h"
#include "rtc.h"
#include "system.h"

#define AUDIO_MIC_TYPE                  MIC_DIGITAL
#define AUDIO_GAIN_DB                   35.0f

#define TIMER_NUMBER                    2

int main(void)
{
   // Set up system hardware
   setup_hardware();
   rtc_init();
   if (AUDIO_MIC_TYPE == MIC_ANALOG)
      audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, AUDIO_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_DEFAULT_SAMPLING_RATE_HZ, AUDIO_GAIN_DB);
   system_enable_interrupts(true);

   // Create a timer to measure code performance
   am_hal_timer_config_t timer_config;
   am_hal_timer_default_config_set(&timer_config);
   am_hal_timer_config(TIMER_NUMBER, &timer_config);

   // Initialize the AI library components and begin reading continuous audio
   ai_initialize();
   audio_begin_reading();

   // Handle incoming audio clips for 10 seconds
   int16_t *audio_buffer = NULL;
   for (int i = 0; i < 10; ++i)
   {
      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();

      // Handle any newly available audio data
      if (audio_data_available() && (audio_buffer = audio_read_data_direct()))
      {
         // Invoke the AI model and output the result
         bool store = ai_invoke(audio_buffer);
         print("AI Result: %s item\n", store ? "STORE" : "DO NOT store");
      }
   }

   // Execute AI invocation again to time performance
   am_hal_delay_us(250000);
   am_hal_timer_clear(TIMER_NUMBER);
   ai_invoke(audio_buffer);
   uint32_t timer_val = am_hal_timer_read(TIMER_NUMBER);
   am_hal_timer_stop(TIMER_NUMBER);
   print("Execution time: %u ms\n", (uint32_t)(((float)timer_val / (AM_HAL_CLKGEN_FREQ_MAX_HZ / 16)) * 1000.0f));

   // Sleep forever
   while (true)
   {
      print("Going to sleep...\n");
      system_enter_deep_sleep_mode();
   }

   // Should never reach this point
   return 0;
}
