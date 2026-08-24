#ifndef __AUDIO_FILTER_HEADER_H__
#define __AUDIO_FILTER_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void audio_filter_initialize(audio_filter_type_t type, uint32_t sample_rate_hz, uint32_t low_frequency_hz, uint32_t high_frequency_hz);
bool audio_filter_enabled(void);
void audio_filter_apply(int16_t *samples, uint32_t num_samples);

#endif  // #ifndef __AUDIO_FILTER_HEADER_H__
