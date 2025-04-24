#ifndef __MFCC_HEADER_H__
#define __MFCC_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void mfcc_initialize(void);
void mfcc_compute(const int16_t *audio, float *mfcc_out);

#endif  // #ifndef __MFCC_HEADER_H__
