#ifndef OGG_WRITER_H
#define OGG_WRITER_H

#include <stdint.h>
#include "opus_config.h"

typedef struct {
   uint64_t granule_pos;
   uint32_t seq_no;
   uint16_t page_buffer_len;
   uint8_t flags, segment_count;
   uint8_t segment_table[255];
   uint8_t page_buffer[OGG_MAX_PACKET_SIZE];
} ogg_writer_t;

typedef struct {
   uint32_t data_len;
   uint8_t data[sizeof(((ogg_writer_t*)0)->page_buffer) + 282];
} ogg_data_packet_t;

void ogg_reset_writer(ogg_writer_t *ogg_writer, ogg_data_packet_t *output);
void ogg_add_packet(ogg_writer_t *ogg_writer, ogg_data_packet_t *output, const uint8_t* restrict packet_data, uint8_t packet_len, uint8_t is_last_packet);
void ogg_flush_page(ogg_writer_t *ogg_writer, ogg_data_packet_t *output);

#endif  // OGG_WRITER_H
