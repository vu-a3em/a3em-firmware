// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <float.h>
#include <math.h>
#include "logging.h"
#include "fft.h"
#include "mfcc.h"


// MFCC Local Type Definitions -----------------------------------------------------------------------------------------

#define M_PI                        3.14159265358979323846
#define M_2PI                       6.283185307179586476925286766559005

#define NUM_INPUT_SAMPLES           (AI_INPUT_LENGTH_MS * AI_AUDIO_SAMPLE_RATE_HZ / 1000)
#define FFT_SIZE                    (AI_WINDOW_LENGTH_MS * AI_AUDIO_SAMPLE_RATE_HZ / 1000)
#define HOP_SIZE                    (AI_HOP_LENGTH_MS * AI_AUDIO_SAMPLE_RATE_HZ / 1000)
#define NUM_FRAMES                  (1 + ((NUM_INPUT_SAMPLES - FFT_SIZE) / HOP_SIZE))

#define MFCC_MAX_FILTERBANK_LEN     8000
#define MFCC_ARENA_SIZE             4 * (FFT_SIZE * 4 + MFCC_NUM_FBANK_BINS * (3 + MFCC_NUM_COEFFS) + MFCC_MAX_FILTERBANK_LEN)

typedef struct {
   uint8_t arena[MFCC_ARENA_SIZE];
   uint32_t frame_len_pow2;
   float *frame, *buffer, *energies, *window, *dct_matrix;
   int32_t *mfcc_bank_first, *mfcc_bank_last;
   float *mel_percentages;
} mfcc_t;


// Static Global Variables ---------------------------------------------------------------------------------------------

static rfft_fast_instance_f32 mfcc_rfft;
static mfcc_t mfcc;


// Private Helper Functions --------------------------------------------------------------------------------------------

static inline float mel_scale(float freq) { return 1127.0f * logf(1.0f + freq / 700.0f); }

static void create_mel_filter_bank(uint32_t low_freq, uint32_t high_freq)
{
   // Calculate the Mel frequency band characteristics
   const int32_t num_fft_bins = mfcc.frame_len_pow2 / 2;
   const float fft_bin_width = ((float)AI_AUDIO_SAMPLE_RATE_HZ) / mfcc.frame_len_pow2;
   const float mel_low_freq = mel_scale(low_freq), mel_high_freq = mel_scale(high_freq);
   const float mel_freq_delta = (mel_high_freq - mel_low_freq) / (MFCC_NUM_FBANK_BINS + 1);

   // Create a filter bank for each Mel band
   for (int32_t bin = 0, idx = 0; bin < MFCC_NUM_FBANK_BINS; ++bin)
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

void mfcc_initialize(void)
{
   // Separate the memory arena into named blocks
   mfcc.frame = (float*)mfcc.arena;
   mfcc.frame_len_pow2 = 1 << (32 - __builtin_clzl(FFT_SIZE-1));
   mfcc.dct_matrix = mfcc.frame + mfcc.frame_len_pow2;
   mfcc.buffer = mfcc.dct_matrix + (MFCC_NUM_FBANK_BINS * MFCC_NUM_COEFFS);
   mfcc.energies = mfcc.buffer + mfcc.frame_len_pow2;
   mfcc.window = mfcc.energies + MFCC_NUM_FBANK_BINS;
   mfcc.mfcc_bank_first = (int32_t*)(mfcc.window + FFT_SIZE);
   mfcc.mfcc_bank_last = mfcc.mfcc_bank_first + MFCC_NUM_FBANK_BINS;
   mfcc.mel_percentages = (float*)(mfcc.mfcc_bank_last + MFCC_NUM_FBANK_BINS);

   // Create the Mel filter bank and Hann windowing function
   create_mel_filter_bank(20, AI_AUDIO_SAMPLE_RATE_HZ / 2);
   for (uint32_t i = 0; i < FFT_SIZE; ++i)
      mfcc.window[i] = 0.5 - 0.5 * cos(M_2PI * ((float)i) / FFT_SIZE);

   // Create the DCT matrix
   float normalizer = sqrtf(2.0f / MFCC_NUM_FBANK_BINS);
   for (uint32_t k = 0; k < MFCC_NUM_COEFFS; ++k)
      for (uint32_t n = 0; n < MFCC_NUM_FBANK_BINS; ++n)
         mfcc.dct_matrix[k * MFCC_NUM_FBANK_BINS + n] = normalizer * cos(((double)M_PI) / MFCC_NUM_FBANK_BINS * (n + 0.5) * k);

   // Initialize the FFT structure
   rfft_fast_init_f32(&mfcc_rfft, mfcc.frame_len_pow2);
}

void mfcc_compute(const int16_t *audio, float *mfcc_out)
{
   // Determine maximum amplitude within full input dimension for normalization
   int16_t max_amplitude = 1;
   for (uint32_t i = 0; i < NUM_INPUT_SAMPLES; ++i)
      max_amplitude = (abs(audio[i]) > max_amplitude) ? abs(audio[i]) : max_amplitude;
   const float max_ampl_inv = 1.0f / max_amplitude;

   // Iterate through all audio frames
   memset(mfcc_out, 0, AI_NUM_INPUT_FEATURES * sizeof(float));
   for (uint32_t t = 0, a = 0; t < NUM_FRAMES; ++t, a += HOP_SIZE)
   {
      // Normalize audio data to [-1, 1] and apply Hann window
      for (uint32_t i = 0; i < FFT_SIZE; ++i)
         mfcc.frame[i] = max_ampl_inv * audio[a + i] * mfcc.window[i];
      memset(mfcc.frame + FFT_SIZE, 0, sizeof(float) * (mfcc.frame_len_pow2 - FFT_SIZE));

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
      for (uint32_t bin = 0, idx = 0; bin < MFCC_NUM_FBANK_BINS; ++bin)
      {
         mfcc.energies[bin] = FLT_MIN;
         const int32_t first_index = mfcc.mfcc_bank_first[bin], last_index = mfcc.mfcc_bank_last[bin];
         for (int32_t i = first_index; i <= last_index; ++i)
            mfcc.energies[bin] += mfcc.buffer[i] * mfcc.mel_percentages[idx++];
         mfcc.energies[bin] = log10f(mfcc.energies[bin]);
      }

      // Use matrix multiplication to take the DCT
      for (uint32_t i = 0, base_idx = t * MFCC_NUM_COEFFS; i < MFCC_NUM_COEFFS; ++i)
      {
         const uint32_t out_idx = base_idx + i, dct_base_idx = i * MFCC_NUM_FBANK_BINS;
         for (uint32_t j = 0; j < MFCC_NUM_FBANK_BINS; ++j)
            mfcc_out[out_idx] += mfcc.dct_matrix[dct_base_idx + j] * mfcc.energies[j];
      }
   }
}
