#ifndef __OPUSENC_HEADER_H__
#define __OPUSENC_HEADER_H__

#include "static_config.h"

#define OPUS_FRAME_MAX_BYTES                 256

#ifndef MIN
#define MIN(a,b) ((a)<(b) ? (a):(b))
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b) ? (a):(b))
#endif

struct __attribute__ ((__packed__, aligned (4))) opus_frame_t;

typedef struct __attribute__ ((__packed__, aligned (4))) opus_frame_t
{
   uint8_t num_encoded_bytes;
   uint8_t encoded_data[OPUS_FRAME_MAX_BYTES];
   struct opus_frame_t *next;
} opus_frame_t;

void opusenc_init(int32_t bitrate);
void opusenc_encode(const int16_t* restrict audio_in, const opus_frame_t** restrict result_begin, const opus_frame_t** restrict result_end);

#endif  // #ifndef __OPUSENC_HEADER_H__
