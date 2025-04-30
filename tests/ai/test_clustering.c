#include <math.h>
#include "clustering.h"
#include "logging.h"
#include "mfcc.h"
#include "nn.h"
#include "system.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// YOU MUST REDEFINE AI_NUM_OUTPUT_FEATURES TO "2" AND MAX_NUM_CLUSTERS TO "3" IN "static_config.h" FOR THIS TEST //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define M_PI                    3.14159265358979323846
#define NUM_SINE_WAVE_SAMPLES   (AI_AUDIO_SAMPLE_RATE_HZ * AI_INPUT_LENGTH_MS / 1000)
#define TIMER_NUMBER            2

static void generate_sine_wave(uint32_t frequency, int16_t *sine_wave)
{
   float delta_time = 1.0f / AI_AUDIO_SAMPLE_RATE_HZ;
   const uint32_t num_samples = NUM_SINE_WAVE_SAMPLES;
   for (uint32_t i = 0; i < num_samples; ++i)
      sine_wave[i] = (int16_t)(32767.0 * sin(2.0 * M_PI * delta_time * i * frequency));
}

static bool check_filter(const float (*expected_means)[2], const float *expected_weights)
{
   const float (*means)[2] = clustering_get_means(), *weights = clustering_get_weights();
   for (int i = 0; i < 3; ++i)
   {
      for (int j = 0; j < 2; ++j)
         if (fabsf(means[i][j] - expected_means[i][j]) >= 0.0001)
            return false;
      if (fabsf(weights[i] - expected_weights[i]) >= 0.0001)
         return false;
   }
   return true;
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

   // Initialize the MFCC, AI, and clustering libraries
   mfcc_initialize();
   nn_initialize();
   clustering_initialize();

   // Generate a test sine wave
   int16_t sine_wave[NUM_SINE_WAVE_SAMPLES];
   generate_sine_wave(1234, sine_wave);

   // Compute MFCCs for the sine wave
   float mfccs[AI_NUM_INPUT_FEATURES];
   mfcc_compute(sine_wave, mfccs);

   // Pass the MFCCs as features to the NN model and cluster the results
   float *embeddings = nn_invoke(mfccs);
   bool store = clustering_invoke(embeddings);
   print("Clustering Result: %s item\n", store ? "STORE" : "DO NOT store");

   // Check weights and means of the clustering algorithm
   clustering_initialize();
   print("Testing clustering weights and means...\n");
   print("Test 1: %s\n", check_filter((float[3][2]){{0, 0}, {0, 0}, {0, 0}}, (float[3]){0, 0, 0}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){1, 2});
   print("Test 2: %s\n", store && check_filter((float[3][2]){{0, 0}, {0, 0}, {1, 2}}, (float[3]){0, 0, 1}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-3, 2});
   print("Test 3: %s\n", store && check_filter((float[3][2]){{0, 0}, {1, 2}, {-3, 2}}, (float[3]){0, 1, 1}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-3.2, 2.1});
   print("Test 4: %s\n", !store && check_filter((float[3][2]){{0, 0}, {1, 2}, {-3.1, 2.05}}, (float[3]){0, 1, 2}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-4, 1});
   print("Test 5: %s\n", store && check_filter((float[3][2]){{1, 2}, {-3.1, 2.05}, {-4, 1}}, (float[3]){1, 2, 1}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-4, 1.8});
   print("Test 6: %s\n", store && check_filter((float[3][2]){{-3.1, 2.05}, {-4, 1}, {-4, 1.8}}, (float[3]){2, 1, 1}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-3.4, 2.2});
   print("Test 7: %s\n", !store && check_filter((float[3][2]){{-4, 1}, {-4, 1.8}, {-3.2, 2.1}}, (float[3]){1, 1, 3}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-4.1, 1.41});
   print("Test 8: %s\n", !store && check_filter((float[3][2]){{0, 0}, {-3.2, 2.1}, {-4.033333, 1.4033333}}, (float[3]){0, 3, 3}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-3, 2});
   print("Test 9: %s\n", !store && check_filter((float[3][2]){{0, 0}, {-4.033333, 1.4033333}, {-3.15, 2.075}}, (float[3]){0, 3, 4}) ? "SUCCESS" : "FAILURE");
   store = clustering_invoke((float[2]){-3.59, 1.74});
   print("Test 10: %s\n", !store && check_filter((float[3][2]){{0, 0}, {0, 0}, {-3.53624875, 1.78125}}, (float[3]){0, 0, 5}) ? "SUCCESS" : "FAILURE");

   // Execute call again to time performance
   am_hal_delay_us(250000);
   am_hal_timer_clear(TIMER_NUMBER);
   store = clustering_invoke(embeddings);
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
