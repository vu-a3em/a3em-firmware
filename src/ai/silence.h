#ifndef __SILENCE_HEADER_H__
#define __SILENCE_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void silence_filter_initialize(uint32_t audio_sample_rate_hz, uint32_t min_frequency, uint32_t max_frequency, float silence_threshold);
bool silence_filter_is_silence(const int16_t *audio, uint32_t num_samples);

#endif  // #ifndef __SILENCE_HEADER_H__
