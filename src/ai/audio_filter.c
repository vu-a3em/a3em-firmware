// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <arm_math.h>
#include <math.h>
#include "audio_filter.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define MAX_FILTER_STAGES     2
#define BLOCK_SAMPLES         256

#define FILTER_PI             3.14159265358979323846f
#define FILTER_SQRT2          1.41421356237309504880f

static arm_biquad_casd_df1_inst_f32 filter_instance;
static float32_t filter_coefficients[MAX_FILTER_STAGES * 5], filter_state[MAX_FILTER_STAGES * 4];
static float32_t block_in[BLOCK_SAMPLES], block_out[BLOCK_SAMPLES];
static uint32_t num_stages;


// Private Helper Functions --------------------------------------------------------------------------------------------

// Second-order Butterworth sections, in the {b0, b1, b2, a1, a2} layout CMSIS expects.
// Note that CMSIS wants the feedback coefficients NEGATED relative to the usual
// transfer-function convention, which is a standing trap in this API.

static void design_low_pass(float32_t *coefficients, float cutoff_hz, float sample_rate_hz)
{
   const float omega = tanf(FILTER_PI * cutoff_hz / sample_rate_hz);
   const float norm = 1.0f / (1.0f + FILTER_SQRT2 * omega + omega * omega);
   const float b0 = omega * omega * norm;

   coefficients[0] = b0;
   coefficients[1] = 2.0f * b0;
   coefficients[2] = b0;
   coefficients[3] = -(2.0f * (omega * omega - 1.0f) * norm);
   coefficients[4] = -((1.0f - FILTER_SQRT2 * omega + omega * omega) * norm);
}

static void design_high_pass(float32_t *coefficients, float cutoff_hz, float sample_rate_hz)
{
   const float omega = tanf(FILTER_PI * cutoff_hz / sample_rate_hz);
   const float norm = 1.0f / (1.0f + FILTER_SQRT2 * omega + omega * omega);

   coefficients[0] = norm;
   coefficients[1] = -2.0f * norm;
   coefficients[2] = norm;
   coefficients[3] = -(2.0f * (omega * omega - 1.0f) * norm);
   coefficients[4] = -((1.0f - FILTER_SQRT2 * omega + omega * omega) * norm);
}


// Public API Functions ------------------------------------------------------------------------------------------------

void audio_filter_initialize(audio_filter_type_t type, uint32_t sample_rate_hz, uint32_t low_frequency_hz, uint32_t high_frequency_hz)
{
   num_stages = 0;
   if ((type == FILTER_NONE) || !sample_rate_hz)
      return;

   // Keep both corners strictly inside the representable band
   const float nyquist = (float)sample_rate_hz * 0.5f;
   const float low = fminf(fmaxf((float)low_frequency_hz, 1.0f), nyquist - 1.0f);
   const float high = fminf(fmaxf((float)high_frequency_hz, 1.0f), nyquist - 1.0f);

   if ((type == FILTER_HIGH_PASS) || (type == FILTER_BAND_PASS))
      design_high_pass(&filter_coefficients[num_stages++ * 5], low, (float)sample_rate_hz);
   if ((type == FILTER_LOW_PASS) || (type == FILTER_BAND_PASS))
      design_low_pass(&filter_coefficients[num_stages++ * 5], high, (float)sample_rate_hz);

   if (num_stages)
      arm_biquad_cascade_df1_init_f32(&filter_instance, (uint8_t)num_stages, filter_coefficients, filter_state);
}

bool audio_filter_enabled(void)
{
   return num_stages > 0;
}

void audio_filter_apply(int16_t *samples, uint32_t num_samples)
{
   if (!num_stages)
      return;

   // Processed in blocks so the float scratch buffers stay small
   for (uint32_t offset = 0; offset < num_samples; offset += BLOCK_SAMPLES)
   {
      const uint32_t count = (num_samples - offset < BLOCK_SAMPLES) ? (num_samples - offset) : BLOCK_SAMPLES;
      for (uint32_t i = 0; i < count; ++i)
         block_in[i] = (float32_t)samples[offset + i];

      arm_biquad_cascade_df1_f32(&filter_instance, block_in, block_out, count);

      for (uint32_t i = 0; i < count; ++i)
      {
         // Saturate rather than wrap
         const float32_t value = block_out[i];
         if (value >= 32767.0f)
            samples[offset + i] = 32767;
         else if (value <= -32768.0f)
            samples[offset + i] = -32768;
         else
            samples[offset + i] = (int16_t)value;
      }
   }
}
