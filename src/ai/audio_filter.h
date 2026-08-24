#ifndef __AUDIO_FILTER_HEADER_H__
#define __AUDIO_FILTER_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

// Band-limits recorded audio before it is written to the card.
//
// Distinct from the silence filter, which only decides whether a clip is worth keeping
// and never alters what is stored. This changes the audio itself, discarding energy
// outside the band of interest -- wind rumble below a few hundred hertz, or hiss above
// the calls being surveyed.
//
// Implemented as a cascade of second-order Butterworth sections using the CMSIS-DSP
// biquad routines. A band-pass is a high-pass section followed by a low-pass one.

// Prepares the filter for a phase. A type of FILTER_NONE disables it entirely, and the
// audio path then costs nothing.
void audio_filter_initialize(audio_filter_type_t type, uint32_t sample_rate_hz,
                             uint32_t low_frequency_hz, uint32_t high_frequency_hz);

// True when a filter is configured, so callers can skip the work altogether.
bool audio_filter_enabled(void);

// Filters a buffer of samples in place.
void audio_filter_apply(int16_t *samples, uint32_t num_samples);

#endif  // #ifndef __AUDIO_FILTER_HEADER_H__
