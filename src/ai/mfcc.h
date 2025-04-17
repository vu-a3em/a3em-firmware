#ifndef __MFCC_HEADER_H__
#define __MFCC_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void mfcc_initialize(uint32_t audio_sample_rate, float input_length_ms, float fft_window_length_ms, float hop_length_ms);
void mfcc_compute(const int16_t *audio, float *mfcc_out);

#endif  // #ifndef __MFCC_HEADER_H__
