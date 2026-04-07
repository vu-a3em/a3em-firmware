#include <arm_math.h>
#include "audio.h"
#include "logging.h"
#include "system.h"


// Silence filter implementation ---------------------------------------------------------------------------------------

#define NEXT_POW2(x)         ((((x)-1) | ((x)-1) >> 1 | ((x)-1) >> 2 | ((x)-1) >> 4 | ((x)-1) >> 8 | ((x)-1) >> 16) + 1)

#define AUDIO_MIC_TYPE                  MIC_ANALOG

#define AUDIO_SAMPLE_RATE_HZ            16000
#define AUDIO_INPUT_GAIN_DB             25.0

#define WINDOW_LENGTH_MS                30
#define HOP_LENGTH_MS                   15

#define WINDOW_SIZE                     (WINDOW_LENGTH_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define STEP_SIZE                       (HOP_LENGTH_MS * AUDIO_SAMPLE_RATE_HZ / 1000)
#define FFT_SIZE                        NEXT_POW2(WINDOW_SIZE)
#define NUM_FREQUENCY_BINS              (FFT_SIZE / 2)
#define ZERO_PADDING_LENGTH             (FFT_SIZE - WINDOW_SIZE)

#define NUM_WINDOWS                     (AUDIO_SAMPLE_RATE_HZ / STEP_SIZE)
#define AVAILABLE_WINDOWS_PER_BUFFER    (1 + (AUDIO_SAMPLE_RATE_HZ - WINDOW_SIZE) / STEP_SIZE)
#define MISSING_WINDOWS_PER_BUFFER      (NUM_WINDOWS - AVAILABLE_WINDOWS_PER_BUFFER)

static volatile bool device_activated = true;
static float fft_input[FFT_SIZE], hanning_window[WINDOW_SIZE];
static int16_t pending_audio[WINDOW_SIZE];
static arm_rfft_fast_instance_f32 fft;

static void silence_filter_initialize(void)
{
   // Initialize the FFT library and data structures
   arm_rfft_fast_init_f32(&fft, FFT_SIZE);
   arm_hanning_f32(hanning_window, WINDOW_SIZE);
   memset(pending_audio, 0, sizeof(pending_audio));
}

static void silence_filter_compute_slice(const int16_t *input_audio)
{
   // Statically define all necessary computation structures and constants
   static float input_signal[WINDOW_SIZE], fft_output[FFT_SIZE], fft_magnitudes[FFT_SIZE / 2];

   // Convert the audio signal to a normalized floating point representation
   arm_q15_to_float(input_audio, input_signal, WINDOW_SIZE);

   // Apply a Hanning window to the input signal
   memset(&fft_input[WINDOW_SIZE], 0, ZERO_PADDING_LENGTH * sizeof(fft_input[0]));
   arm_mult_f32(input_signal, hanning_window, fft_input, WINDOW_SIZE);

   // Compute the FFT of the signal followed by its complex magnitude squared
   arm_rfft_fast_f32(&fft, fft_input, fft_output, 0);
   arm_cmplx_mag_squared_f32(fft_output, fft_magnitudes, NUM_FREQUENCY_BINS);

   // TODO: THE ALGORITHM
}

static bool silence_filter_invoke(const int16_t *audio_buffer)
{
   // Finish carrying out the algorithm on the last little bit of the previous audio frame
   bool pure_silence = true;
   for (uint32_t i = 0, window = AVAILABLE_WINDOWS_PER_BUFFER; window < NUM_WINDOWS; i += STEP_SIZE, ++window)
   {
      // Copy the beginning of the current audio frame into the end of the previous frame
      arm_copy_q15(&audio_buffer[i], &pending_audio[WINDOW_SIZE - STEP_SIZE], STEP_SIZE);

      // Execute the silence filter computation for a single slice of audio
      silence_filter_compute_slice(pending_audio);

      // Shift the previous audio frame over to make room for the next samples
      memmove(&pending_audio[0], &pending_audio[STEP_SIZE], sizeof(pending_audio[0]) * (WINDOW_SIZE - STEP_SIZE));
   }

   // Iterate through all remaining spectrogram windows
   for (uint32_t i = 0, window = 0; window < AVAILABLE_WINDOWS_PER_BUFFER; i += STEP_SIZE, ++window)
   {
      // Execute the silence filter computation for a single slice of audio
      silence_filter_compute_slice(&audio_buffer[i]);
   }

   // Store pending audio data to use with the next incoming frame
   arm_copy_q15(&audio_buffer[AVAILABLE_WINDOWS_PER_BUFFER * STEP_SIZE], &pending_audio[0], WINDOW_SIZE - STEP_SIZE);

   // Return whether the frame contains pure silence
   return pure_silence;
}


// Test harness --------------------------------------------------------------------------------------------------------

#define TIMER_NUMBER            03

typedef bool (*function_under_test)(const int16_t *audio_buffer);

static uint32_t silence_filter_invoke_timed(const int16_t *audio_buffer)
{
   // Calculate the real execution time (in ms) of the silence filter
   am_hal_timer_clear(TIMER_NUMBER);
   silence_filter_invoke(audio_buffer);
   const uint32_t timer_val = am_hal_timer_read(TIMER_NUMBER);
   am_hal_timer_stop(TIMER_NUMBER);
   return (uint32_t)(((float)timer_val / (AM_HAL_CLKGEN_FREQ_MAX_HZ / 16)) * 1000.0f);
}

int main(void)
{
   // Set up system hardware
   setup_hardware();
   if (AUDIO_MIC_TYPE == MIC_ANALOG)
      audio_analog_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLE_RATE_HZ, 1, AUDIO_INPUT_GAIN_DB, AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0, &device_activated);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, AUDIO_SAMPLE_RATE_HZ, 1, AUDIO_INPUT_GAIN_DB);
   system_enable_interrupts(true);

   // Create a timer to measure code performance
   am_hal_timer_config_t timer_config;
   am_hal_timer_default_config_set(&timer_config);
   am_hal_timer_config(TIMER_NUMBER, &timer_config);

   // Initialize the silence filter
   silence_filter_initialize();

   // Set up required audio variables and begin streaming audio
   int16_t *audio_packet;
   audio_begin_reading();

   // Loop forever processing incoming audio
   while (true)
   {
      // Handle any newly available audio data
      if (audio_data_available() && (audio_packet = audio_read_data_direct()))
      {
         const uint32_t execution_time_ms = silence_filter_invoke_timed(audio_packet);
         printonly("Silence filter execution time: %lu ms\n", execution_time_ms);
      }

      // Sleep while no errors or audio to process
      if (audio_error_encountered())
         system_reset();
      else if (!audio_data_available())
         system_enter_deep_sleep_mode();
   }

   // Should never reach this point
   return 0;
}
