#ifndef __AI_HEADER_H__
#define __AI_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// FFT-Specific Type Definitions ---------------------------------------------------------------------------------------

typedef struct
{
   uint16_t fftLen;
   const float *pTwiddle;
   const uint16_t *pBitRevTable;
   uint16_t bitRevLength;
} cfft_instance_f32;

typedef struct
{
   cfft_instance_f32 Sint;
   uint16_t fftLenRFFT;
   const float *pTwiddleRFFT;
} rfft_fast_instance_f32 ;


// Public API Functions ------------------------------------------------------------------------------------------------

void rfft_fast_init_f32(rfft_fast_instance_f32 *S, uint16_t fftLen);
void rfft_fast_f32(rfft_fast_instance_f32 *S, float *p, float *pOut);

#endif  // #ifndef __AI_HEADER_H__
