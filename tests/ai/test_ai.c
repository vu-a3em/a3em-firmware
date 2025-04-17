#include <math.h>
#include "ai.h"
#include "logging.h"
#include "mfcc.h"
#include "system.h"

#define AUDIO_SAMPLE_RATE_HZ    16000
#define INPUT_LENGTH_MS         1000
#define WINDOW_LENGTH_MS        30
#define HOP_LENGTH_MS           15

#define TIMER_NUMBER            2

static uint32_t generate_sine_wave(uint32_t frequency, uint32_t sample_rate, uint32_t duration_ms, int16_t **output)
{
   float delta_time = 1.0f / sample_rate;
   uint32_t num_samples = (uint32_t)(sample_rate * ((float)duration_ms / 1000.0));
   *output = (int16_t*)malloc(num_samples * sizeof(int16_t));
   for (uint32_t i = 0; i < num_samples; ++i)
      (*output)[i] = (int16_t)(32767.0 * sin(2.0 * M_PI * delta_time * i * frequency));
   return num_samples;
}

int main(void)
{
   // Set up system hardware
   setup_hardware();
   system_enable_interrupts(true);

   // Create a timer to measure code performance
   am_hal_timer_config_t timer_config;
   am_hal_timer_default_config_set(&timer_config);
   am_hal_timer_config(TIMER_NUMBER, &timer_config);

   // Initialize the MFCC and AI libraries
   mfcc_initialize(AUDIO_SAMPLE_RATE_HZ, INPUT_LENGTH_MS, WINDOW_LENGTH_MS, HOP_LENGTH_MS);
   ai_initialize();

   // Generate a test sine wave
   int16_t *sine_wave;
   generate_sine_wave(1234, AUDIO_SAMPLE_RATE_HZ, INPUT_LENGTH_MS, &sine_wave);

   // Compute MFCCs for the sine wave
   const uint32_t num_time_slices = 1 + ((INPUT_LENGTH_MS - WINDOW_LENGTH_MS) / HOP_LENGTH_MS);
   float mfccs[MFCC_NUM_COEFFS * num_time_slices];
   mfcc_compute(sine_wave, mfccs);

   // Pass the MFCCs as features to the AI model
   float *embeddings = ai_invoke(mfccs);
   print("Embeddings: [ ");
   for (uint32_t i = 0; i < 16; ++i)
      print("%f ", embeddings[i]);
   print("]\n");

   // Execute call again to time performance
   am_hal_delay_us(250000);
   am_hal_timer_clear(TIMER_NUMBER);
   ai_invoke(mfccs);
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
