#ifndef __AUDIO_HEADER_H__
#define __AUDIO_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef enum { IMMEDIATE, COMPARATOR_THRESHOLD } audio_trigger_t;

typedef struct
{
   uint32_t buffers_captured;    // DMA buffers handed to the application
   uint32_t buffers_dropped;     // DMA buffers overwritten before the application read them
   uint32_t missed_completions;  // Times the DMA-complete interrupt had to be recovered by the backstop
   bool dcmp_trusted;            // Whether the DMA-complete interrupt has proven reliable
} audio_stats_t;


// Public API Functions ------------------------------------------------------------------------------------------------

bool audio_digital_init(uint32_t num_channels, uint32_t sample_rate_hz, uint32_t clip_length_seconds, float gain_db);
bool audio_analog_init(uint32_t num_channels, uint32_t sample_rate_hz, uint32_t clip_length_seconds, float gain_db, float mic_bias_voltage, audio_trigger_t trigger, float trigger_threshold_percent, volatile bool *device_activated);
void audio_deinit(void);
void audio_begin_reading(void);
void audio_stop_reading(void);
bool audio_data_available(void);
bool audio_error_encountered(void);
bool audio_read_data(int16_t *buffer);
int16_t* audio_read_data_direct(void);
uint32_t audio_num_seconds_per_dma(void);
void audio_get_stats(audio_stats_t *stats);
uint32_t audio_get_actual_sample_rate(void);


// Statistics accumulated over every sample that passes through the read path, for both
// analog and digital microphones.
//
// Broken microphone wiring has already cost at least one deployment, and there was
// previously no way to know from a returned card whether the microphone worked. The
// analog DC-offset check at startup catches a path that is dead before recording
// begins; these statistics additionally catch one that dies partway through, and catch
// a microphone that is electrically present but producing nothing.
typedef struct {
   int16_t min_sample, max_sample;
   int32_t mean;                 // DC bias remaining after offset removal
   uint32_t rms;                 // signal level
   uint32_t peak;                // largest absolute excursion
   uint32_t num_samples;
   bool constant_output;         // every sample identical: a stuck or dead signal path
   bool silent;                  // no excursion beyond the noise floor
} audio_health_t;

void audio_health_reset(void);
bool audio_health_available(void);
audio_health_t audio_health_get(void);
int32_t audio_get_dc_offset(void);

#endif  // #ifndef __AUDIO_HEADER_H__
