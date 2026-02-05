#include "opus_config.h"
#include "ogg_writer.h"


// Ogg Writer Type Definitions ------------------------------------------------

#define OGG_AUDIO_SAMPLE_RATE_HZ              48000
#define OGG_OPUS_FRAME_SIZE                   (48 * OPUS_MS_PER_FRAME)


// Private Helper Functions ---------------------------------------------------

#if defined(OGG_USE_CRC) && (OGG_USE_CRC > 0)

static void init_crc_table(uint32_t crc_table[256])
{
   // Initializes the CRC lookup table for Ogg page checksums
   const uint32_t poly = 0x04c11db7;
   for (int i = 0; i < 256; ++i)
   {
      uint32_t r = i << 24;
      for (int j = 0; j < 8; ++j)
         r = (r & 0x80000000) ? ((r << 1) ^ poly) : (r << 1);
      crc_table[i] = r;
   }
}

static uint32_t update_crc(uint32_t crc, const unsigned char *restrict buffer, uint32_t len)
{
   // Ensure that the CRC table is initialized before use
   static uint8_t crc_initialized = 0;
   static uint32_t crc_table[256];
   if (!crc_initialized)
   {
      init_crc_table(crc_table);
      crc_initialized = 1;
   }
    
   // Update the CRC value with the given buffer data
   for (uint32_t i = 0; i < len; ++i)
   {
      const uint8_t byte = buffer[i];
      int idx = ((crc >> 24) ^ byte) & 0xFF;
      crc = (crc << 8) ^ crc_table[idx];
   }
   return crc;
}

#else

#define update_crc(crc, buffer, len) 0

#endif  // OGG_USE_CRC


// Ogg Encapsulation Functions ------------------------------------------------

void ogg_reset_writer(ogg_writer_t *ogg_writer, ogg_data_packet_t *output)
{
   // Reset the Ogg stream state
   ogg_writer->seq_no = 0;
   ogg_writer->granule_pos = 0;
   ogg_writer->flags = 0x02;
   ogg_writer->segment_count = 0;
   ogg_writer->page_buffer_len = 0;

   // Set up the encoder vendor string for the Opus Comment header
   const uint32_t vendor_string_length = sizeof(OGG_ENCODER_NAME) - 1;
   uint8_t opus_tags[16 + sizeof(OGG_ENCODER_NAME) - 1] = { 'O', 'p', 'u', 's', 'T', 'a', 'g', 's' };
   memcpy(opus_tags + 8, &vendor_string_length, sizeof(vendor_string_length));
   memcpy(opus_tags + 12, OGG_ENCODER_NAME, vendor_string_length);

   // Set up the Opus ID header
   ogg_data_packet_t temp_output;
   const uint16_t preskip = 0;
   const uint32_t audio_sample_rate = OGG_AUDIO_SAMPLE_RATE_HZ;
   uint8_t opus_head[19] = { 'O', 'p', 'u', 's', 'H', 'e', 'a', 'd', 1, 1 };
   memcpy(opus_head + 10, &preskip, sizeof(preskip));
   memcpy(opus_head + 12, &audio_sample_rate, sizeof(audio_sample_rate));
   
   // Write the Opus ID header followed by the Comment header
   ogg_add_packet(ogg_writer, output, opus_head, sizeof(opus_head), 0);
   ogg_writer->granule_pos = 0;
   ogg_flush_page(ogg_writer, output, 0);
   ogg_add_packet(ogg_writer, &temp_output, opus_tags, sizeof(opus_tags), 0);
   ogg_writer->granule_pos = 0;
   ogg_flush_page(ogg_writer, &temp_output, 0);
   memcpy(output->data + output->data_len, temp_output.data, temp_output.data_len);
   output->data_len += temp_output.data_len;
}

void ogg_add_packet(ogg_writer_t *ogg_writer, ogg_data_packet_t *output, const uint8_t* restrict packet_data, uint8_t packet_len, uint8_t is_last_packet)
{
   // Check if flushing is required before adding this packet
   output->data_len = 0;
   if ((ogg_writer->segment_count >= 255) || ((ogg_writer->page_buffer_len + packet_len) > sizeof(ogg_writer->page_buffer)))
      ogg_flush_page(ogg_writer, output, 0);

   // Add the new packet to the buffer as a single segment
   ogg_writer->segment_table[ogg_writer->segment_count++] = packet_len;
   memcpy(ogg_writer->page_buffer + ogg_writer->page_buffer_len, packet_data, packet_len);
   ogg_writer->page_buffer_len += packet_len;
   ogg_writer->granule_pos += OGG_OPUS_FRAME_SIZE;
   
   // Handle the last packet
   if (is_last_packet)
      ogg_writer->flags |= 0x04;
}

void ogg_flush_page(ogg_writer_t *ogg_writer, ogg_data_packet_t *output, uint8_t is_last_page)
{
   // Ensure there is data to flush
   if (ogg_writer->segment_count)
   {
      // Handle the last packet
      if (is_last_page)
         ogg_writer->flags |= 0x04;

      // Create the Ogg page header with an arbitrary serial number
      uint8_t header[27] = { 'O', 'g', 'g', 'S', 0, ogg_writer->flags };
      const uint32_t serial_no = 13872;
      memcpy(header + 6, &ogg_writer->granule_pos, sizeof(ogg_writer->granule_pos));
      memcpy(header + 14, &serial_no, sizeof(serial_no));
      memcpy(header + 18, &ogg_writer->seq_no, sizeof(ogg_writer->seq_no));
      header[26] = ogg_writer->segment_count;

      // Calculate the CRC
      uint32_t crc = update_crc(0, header, sizeof(header));
      crc = update_crc(crc, ogg_writer->segment_table, ogg_writer->segment_count);
      crc = update_crc(crc, ogg_writer->page_buffer, ogg_writer->page_buffer_len);
      memcpy(header + 22, &crc, sizeof(crc));

      // Write all data to the output buffer
      memcpy(output->data, header, sizeof(header));
      memcpy(output->data + sizeof(header), ogg_writer->segment_table, ogg_writer->segment_count);
      memcpy(output->data + sizeof(header) + ogg_writer->segment_count, ogg_writer->page_buffer, ogg_writer->page_buffer_len);
      output->data_len = sizeof(header) + ogg_writer->segment_count + ogg_writer->page_buffer_len;

      // Update the internal Ogg writer state
      ogg_writer->seq_no++;
      ogg_writer->flags = 0;
      ogg_writer->segment_count = 0;
      ogg_writer->page_buffer_len = 0;
   }
}
