// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <float.h>
#include <math.h>
#include "logging.h"
#include "fft.h"
#include "mfcc.h"


// MFCC Local Type Definitions -----------------------------------------------------------------------------------------

#define M_2PI                       6.283185307179586476925286766559005

#define MFCC_MAX_FRAME_LEN          4096
#define MFCC_MAX_FILTERBANK_LEN     8000
#define MFCC_ARENA_SIZE             4 * (MFCC_MAX_FRAME_LEN * 2 + MFCC_NUM_FBANK_BINS * (3 + MFCC_NUM_COEFFS) + MFCC_MAX_FILTERBANK_LEN)

typedef struct {
   uint8_t arena[MFCC_ARENA_SIZE];
   uint32_t sample_frequency, num_fbank_bins, low_freq, high_freq, hop_size;
   uint32_t num_input_samples, num_frames, num_coeffs, frame_len, frame_len_pow2;
   float *frame, *buffer, *energies, *window, *dct_matrix;
   int32_t *mfcc_bank_first, *mfcc_bank_last;
   float *mel_percentages;
} mfcc_t;


// Static Global Variables ---------------------------------------------------------------------------------------------

static rfft_fast_instance_f32 mfcc_rfft;
static mfcc_t mfcc;


// Private Helper Functions --------------------------------------------------------------------------------------------

static inline float mel_scale(float freq) { return 1127.0f * logf(1.0f + freq / 700.0f); }

static void create_mel_filter_bank(void)
{
   // Calculate the Mel frequency band characteristics
   const int32_t num_fft_bins = mfcc.frame_len_pow2 / 2;
   const float fft_bin_width = ((float)mfcc.sample_frequency) / mfcc.frame_len_pow2;
   const float mel_low_freq = mel_scale(mfcc.low_freq), mel_high_freq = mel_scale(mfcc.high_freq);
   const float mel_freq_delta = (mel_high_freq - mel_low_freq) / (mfcc.num_fbank_bins + 1);

   // Create a filter bank for each Mel band
   for (int32_t bin = 0, idx = 0; bin < mfcc.num_fbank_bins; ++bin)
   {
      // Determine the left, right, and center frequencies for this band
      mfcc.mfcc_bank_first[bin] = mfcc.mfcc_bank_last[bin] = -1;
      const float left_mel = mel_low_freq + bin * mel_freq_delta;
      const float center_mel = mel_low_freq + (bin + 1) * mel_freq_delta;
      const float right_mel = mel_low_freq + (bin + 2) * mel_freq_delta;

      // Store the FFT bin percentages to attribute to this Mel band
      for (int32_t i = 0; i < num_fft_bins; ++i)
      {
         const float mel = mel_scale(fft_bin_width * i);
         if (mel > left_mel && mel < right_mel)
         {
            mfcc.mel_percentages[idx++] = (mel <= center_mel) ?
                  ((mel - left_mel) / (center_mel - left_mel)) :
                  ((right_mel - mel) / (right_mel - center_mel));
            if (mfcc.mfcc_bank_first[bin] == -1)
               mfcc.mfcc_bank_first[bin] = i;
            mfcc.mfcc_bank_last[bin] = i;
         }
      }
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void mfcc_initialize(uint32_t audio_sample_rate, float input_length_ms, float fft_window_length_ms, float hop_length_ms)
{
   // Set up the MFCC computation structure
   const uint32_t fft_size = (uint32_t)((fft_window_length_ms / 1000.0f) * audio_sample_rate);
   mfcc.sample_frequency = audio_sample_rate;
   mfcc.num_fbank_bins = MFCC_NUM_FBANK_BINS;
   mfcc.low_freq = 20;
   mfcc.high_freq = audio_sample_rate / 2;
   mfcc.hop_size = (uint32_t)((hop_length_ms / 1000.0f) * audio_sample_rate);
   mfcc.num_input_samples = (uint32_t)((input_length_ms / 1000.0f) * audio_sample_rate);
   mfcc.num_frames = 1 + ((mfcc.num_input_samples - fft_size) / mfcc.hop_size);
   mfcc.num_coeffs = MFCC_NUM_COEFFS;
   mfcc.frame_len = fft_size;
   mfcc.frame_len_pow2 = 1 << (32 - __builtin_clzl(fft_size-1));

   // Separate the memory arena into named blocks
   mfcc.frame = (float*)mfcc.arena;
   mfcc.dct_matrix = mfcc.frame + mfcc.frame_len_pow2;
   mfcc.buffer = mfcc.dct_matrix + (mfcc.num_fbank_bins * mfcc.num_coeffs);
   mfcc.energies = mfcc.buffer + mfcc.frame_len_pow2;
   mfcc.window = mfcc.energies + mfcc.num_fbank_bins;
   mfcc.mfcc_bank_first = (int32_t*)(mfcc.window + mfcc.frame_len);
   mfcc.mfcc_bank_last = mfcc.mfcc_bank_first + mfcc.num_fbank_bins;
   mfcc.mel_percentages = (float*)(mfcc.mfcc_bank_last + mfcc.num_fbank_bins);

   // Create the Mel filter bank and Hann windowing function
   create_mel_filter_bank();
   for (uint32_t i = 0; i < mfcc.frame_len; ++i)
      mfcc.window[i] = 0.5 - 0.5 * cos(M_2PI * ((float)i) / (mfcc.frame_len));

   // Create the DCT matrix
   float normalizer = sqrtf(2.0f / (float)mfcc.num_fbank_bins);
   for (uint32_t k = 0; k < mfcc.num_coeffs; ++k)
      for (uint32_t n = 0; n < mfcc.num_fbank_bins; ++n)
         mfcc.dct_matrix[k * mfcc.num_fbank_bins + n] = normalizer * cos(((double)M_PI) / mfcc.num_fbank_bins * (n + 0.5) * k);

   // Initialize the FFT structure
   rfft_fast_init_f32(&mfcc_rfft, mfcc.frame_len_pow2);
}

void mfcc_compute(const int16_t *audio, float *mfcc_out)
{
   // Determine maximum amplitude within full input dimension for normalization
   int16_t max_amplitude = 1;
   for (uint32_t i = 0; i < mfcc.num_input_samples; ++i)
      max_amplitude = (abs(audio[i]) > max_amplitude) ? abs(audio[i]) : max_amplitude;

   // Iterate through all audio frames
   for (uint32_t t = 0, a = 0; t < mfcc.num_frames; ++t, a += mfcc.hop_size)
   {
      // Normalize audio data to [-1, 1] and apply Hann window
      for (uint32_t i = 0; i < mfcc.frame_len; ++i)
         mfcc.frame[i] = ((float)audio[a + i] / max_amplitude) * mfcc.window[i];
      memset(mfcc.frame + mfcc.frame_len, 0, sizeof(float) * (mfcc.frame_len_pow2 - mfcc.frame_len));

      // Compute the FFT and calculate its power spectrum [FFT stored as [r0, rN/2-1, r1, i1, r2, i2, ...]
      rfft_fast_f32(&mfcc_rfft, mfcc.frame, mfcc.buffer);
      const uint32_t half_dim = mfcc.frame_len_pow2 / 2;
      float first_energy = mfcc.buffer[0] * mfcc.buffer[0], last_energy = mfcc.buffer[1] * mfcc.buffer[1];
      for (uint32_t i = 1; i < half_dim; ++i)
      {
         const float real = mfcc.buffer[i * 2], im = mfcc.buffer[i * 2 + 1];
         mfcc.buffer[i] = real * real + im * im;
      }
      mfcc.buffer[0] = first_energy;
      mfcc.buffer[half_dim] = last_energy;

      // Apply Mel filterbanks and take the log
      for (uint32_t bin = 0, idx = 0; bin < mfcc.num_fbank_bins; ++bin)
      {
         mfcc.energies[bin] = FLT_MIN;
         const int32_t first_index = mfcc.mfcc_bank_first[bin], last_index = mfcc.mfcc_bank_last[bin];
         for (int32_t i = first_index; i <= last_index; ++i)
            mfcc.energies[bin] += mfcc.buffer[i] * mfcc.mel_percentages[idx++];
         mfcc.energies[bin] = logf(mfcc.energies[bin]);
      }

      // Use matrix multiplication to take the DCT
      for (uint32_t i = 0; i < mfcc.num_coeffs; ++i)
      {
         mfcc_out[t * mfcc.num_coeffs + i] = 0.0f;
         for (uint32_t j = 0; j < mfcc.num_fbank_bins; ++j)
            mfcc_out[t * mfcc.num_coeffs + i] += mfcc.dct_matrix[i * mfcc.num_fbank_bins + j] * mfcc.energies[j];
      }
   }
}
