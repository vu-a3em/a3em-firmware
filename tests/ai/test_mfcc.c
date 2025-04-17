#include <math.h>
#include "logging.h"
#include "mfcc.h"
#include "system.h"

#define AUDIO_SAMPLE_RATE_HZ    16000
#define INPUT_LENGTH_MS         1200
#define WINDOW_LENGTH_MS        30
#define HOP_LENGTH_MS           15

#define NUM_SINE_WAVE_SAMPLES   (AUDIO_SAMPLE_RATE_HZ * INPUT_LENGTH_MS / 1000)

#define TIMER_NUMBER            2

static void generate_sine_wave(uint32_t frequency, int16_t *sine_wave)
{
   float delta_time = 1.0f / AUDIO_SAMPLE_RATE_HZ;
   const uint32_t num_samples = NUM_SINE_WAVE_SAMPLES;
   for (uint32_t i = 0; i < num_samples; ++i)
      sine_wave[i] = (int16_t)(32767.0 * sin(2.0 * M_PI * delta_time * i * frequency));
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

   // Initialize the MFCC library
   mfcc_initialize(AUDIO_SAMPLE_RATE_HZ, INPUT_LENGTH_MS, WINDOW_LENGTH_MS, HOP_LENGTH_MS);

   // Generate a test sine wave
   int16_t sine_wave[NUM_SINE_WAVE_SAMPLES];
   generate_sine_wave(1234, sine_wave);

   // Compute MFCCs for the sine wave
   const uint32_t num_time_slices = 1 + ((INPUT_LENGTH_MS - WINDOW_LENGTH_MS) / HOP_LENGTH_MS);
   float output[MFCC_NUM_COEFFS * num_time_slices];
   mfcc_compute(sine_wave, output);

   // Output MFCCs
   for (uint32_t t = 0; t < num_time_slices; ++t)
   {
      print("[%u]: [ ", t);
      for (uint32_t i = 0; i < MFCC_NUM_COEFFS; ++i)
         print("%f ", output[t * MFCC_NUM_COEFFS + i]);
      print("]\n");
   }

   // Execute call again to time performance
   am_hal_delay_us(250000);
   am_hal_timer_clear(TIMER_NUMBER);
   mfcc_compute(sine_wave, output);
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
