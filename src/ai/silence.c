// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <arm_math.h>
#include <math.h>
#include "fft.h"
#include "silence.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define M_2PI               6.283185307179586476925286766559005
#define MAX_INPUT_LEN       4096

static rfft_fast_instance_f32 fft;
static uint16_t fft_length, min_bin, max_bin;
static float window[MAX_INPUT_LEN], normalizer, threshold;


// Private Helper Functions --------------------------------------------------------------------------------------------

void compute_min_and_max_bins(uint32_t audio_sample_rate_hz, uint32_t min_frequency, uint32_t max_frequency)
{
   // Calculate the FFT bin parameters
   const uint16_t num_bins = (fft_length / 2) - 1;
   const float bin_width_hz = (float)audio_sample_rate_hz / fft_length;
   const float half_bin_width_hz = 0.5f * bin_width_hz;

   // Locate the minimum and maximum FFT bins of interest
   min_bin = max_bin = 1;
   for (uint16_t bin = 1; bin < num_bins; ++bin)
   {
      const float center_frequency = bin_width_hz * bin;
      if ((center_frequency + half_bin_width_hz) < min_frequency)
         min_bin = max_bin = bin + 1;
      if ((center_frequency - half_bin_width_hz) < max_frequency)
         max_bin = bin;
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void silence_filter_initialize(uint32_t audio_sample_rate_hz, uint32_t min_frequency, uint32_t max_frequency, float silence_threshold)
{
   // Determine the FFT length to keep the frequency bin size between 7.81Hz and 11.72Hz
   fft_length = (audio_sample_rate_hz <= 8000) ? 1024 : ((audio_sample_rate_hz <= 24000) ? 2048 : 4096);
   normalizer = 1.0f / (float)fft_length;
   threshold = silence_threshold;

   // Determine the FFT bins of interest and initialize the FFT structure
   compute_min_and_max_bins(audio_sample_rate_hz, min_frequency, max_frequency);
   rfft_fast_init_f32(&fft, fft_length);

   // Create a Hanning window function
   for (uint16_t i = 0; i < fft_length; ++i)
      window[i] = 0.5 * (1.0 - arm_cos_f32(M_2PI * ((float)i) / fft_length));
}

bool silence_filter_is_silence(const int16_t *audio, uint32_t num_samples)
{
   // Iterate through all FFT frames in the audio
   static float input[MAX_INPUT_LEN], output[MAX_INPUT_LEN];
   for (uint32_t start_sample = 0; start_sample + fft_length <= num_samples; start_sample += fft_length)
   {
      // Apply the Hanning window to the input audio data
      arm_q15_to_float(audio + start_sample, output, fft_length);
      arm_mult_f32(output, window, input, fft_length);

      // Compute the FFT (stored as [r0, rN/2-1, r1, i1, r2, i2, ...])
      rfft_fast_f32(&fft, input, output);

      // Calculate the total amplitude within the frequency bands of interest
      float total_amplitude = 0.0f;
      for (uint16_t bin = min_bin; bin <= max_bin; ++bin)
      {
         const float real = output[2*bin], im = output[2*bin + 1];
         total_amplitude += normalizer * sqrtf(real * real + im * im);
      }

      // Immediately return false if any frame does not contain silence
      if (total_amplitude >= threshold)
         return false;
   }
   return true;
}
