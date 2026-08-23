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

#endif  // #ifndef __AUDIO_HEADER_H__
