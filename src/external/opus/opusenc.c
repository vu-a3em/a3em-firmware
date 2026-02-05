// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "opus_config.h"
#include "opus.h"


// Opus Encoder Type Definitions ---------------------------------------------------------------------------------------

#define OPUS_FRAME_SIZE                      ((OPUS_REQUIRED_SAMPLE_RATE_HZ / 1000) * OPUS_MS_PER_FRAME)
#define OPUS_NUM_HISTORICAL_FRAMES           (OPUS_HISTORY_MS / OPUS_MS_PER_FRAME)
#define OPUS_AUDIO_BUFFER_NUM_SAMPLES        (OPUS_REQUIRED_SAMPLE_RATE_HZ * AUDIO_BUFFER_NUM_SECONDS)


// Static Opus Encoding Variables --------------------------------------------------------------------------------------

__attribute__ ((aligned (4)))
static float encode_buffer[OPUS_FRAME_SIZE];

static opus_encoder_t encoder;
static opus_frame_t opus_frames[OPUS_NUM_HISTORICAL_FRAMES], *opus_frame;


// Public API Functions ------------------------------------------------------------------------------------------------

void opusenc_init(int32_t bitrate)
{
   // Initialize a linked list of historical Opus frames
   for (uint32_t i = 0; i < OPUS_NUM_HISTORICAL_FRAMES; ++i)
      opus_frames[i].next = ((i + 1) == OPUS_NUM_HISTORICAL_FRAMES) ? &opus_frames[0] : &opus_frames[i+1];
   opus_frame = &opus_frames[0];

   // Initialize an Opus encoder for the specified runtime configuration
   opus_encoder_create(&encoder, bitrate, OPUS_FRAME_SIZE);
}

void opusenc_encode(const int16_t* restrict audio_in, const opus_frame_t** restrict result_begin, const opus_frame_t** restrict result_end)
{
   // Initialize the result start and end pointers
   static size_t encode_buffer_index = 0;
   *result_begin = *result_end = opus_frame;

   // Loop until all input samples have been consumed
   for (size_t i = 0; i < OPUS_AUDIO_BUFFER_NUM_SAMPLES; )
   {
      // Copy enough data to fill the Opus encoding buffer
      const size_t samples_to_copy = MIN(OPUS_FRAME_SIZE - encode_buffer_index, OPUS_AUDIO_BUFFER_NUM_SAMPLES - i);
      for (size_t sample = 0; sample < samples_to_copy; ++sample)
         encode_buffer[encode_buffer_index + sample] = audio_in[i + sample];
      encode_buffer_index = (encode_buffer_index + samples_to_copy) % OPUS_FRAME_SIZE;
      i += samples_to_copy;

      // If encoding buffer is full, encode the audio data frame
      if (!encode_buffer_index)
      {
         opus_frame->num_encoded_bytes = (uint16_t)opus_encode(&encoder, encode_buffer, opus_frame->encoded_data, sizeof(opus_frame->encoded_data));
         *result_end = opus_frame = opus_frame->next;
      }
   }
}

const opus_frame_t* opusenc_get_history(void)
{
   // Return the history frame pointer which currently points to the oldest frame
   //   Caller should iterate through history via opus_frame->next until reaching the original pointer
   return opus_frame;
}
