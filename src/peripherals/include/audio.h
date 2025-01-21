#ifndef __AUDIO_HEADER_H__
#define __AUDIO_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef enum { IMMEDIATE, COMPARATOR_THRESHOLD } audio_trigger_t;


// Public API Functions ------------------------------------------------------------------------------------------------

void audio_digital_init(uint32_t num_channels, uint32_t sample_rate_hz, float gain_db);
void audio_analog_init(uint32_t num_channels, uint32_t sample_rate_hz, float gain_db, float mic_bias_voltage, audio_trigger_t trigger, float trigger_threshold_percent);
void audio_deinit(void);
uint32_t audio_num_reads_per_n_seconds(uint32_t seconds);
void audio_begin_reading(void);
void audio_stop_reading(void);
bool audio_data_available(void);
bool audio_error_encountered(void);
bool audio_read_data(int16_t *buffer);
int16_t* audio_read_data_direct(void);

#endif  // #ifndef __AUDIO_HEADER_H__
