#ifndef __OPUSENC_HEADER_H__
#define __OPUSENC_HEADER_H__

#include "static_config.h"

struct __attribute__ ((__packed__, aligned (4))) opus_frame_t;

typedef struct __attribute__ ((__packed__, aligned (4))) opus_frame_t
{
   uint16_t num_encoded_bytes;
   uint8_t encoded_data[2 * ((OPUS_MAX_ENCODING_BITRATE/8) / (1000/OPUS_MS_PER_FRAME))];
   struct opus_frame_t *next;
} opus_frame_t;

void opusenc_init(int32_t bitrate);
void opusenc_encode(const int16_t* restrict audio_in, const opus_frame_t** restrict result_begin, const opus_frame_t** restrict result_end);
const opus_frame_t* opusenc_get_history(void);

#endif  // #ifndef __OPUSENC_HEADER_H__
