#ifndef OPUS_H
#define OPUS_H

#include <stdint.h>

#define COMBFILTER_MAXPERIOD 1024
#define MAX_FRAME_SIZE 960
#define MAX_OVERLAP 120
#define MAX_BANDS 21

typedef struct {
   int32_t bitrate, target_vbr;
   unsigned char toc;
   float delayedIntra, preemph_mem, spec_avg, in_mem[COMBFILTER_MAXPERIOD + MAX_OVERLAP];
   float oldBandE[MAX_BANDS], oldLogE[MAX_BANDS], oldLogE2[MAX_BANDS], energyError[MAX_BANDS];
} opus_encoder;

void opus_encoder_create(opus_encoder *encoder, int32_t bitrate_bps, int32_t frame_size);
int32_t opus_encode(opus_encoder *encoder, const float *pcm, unsigned char *data, uint32_t out_data_bytes);

#endif  // OPUS_H
