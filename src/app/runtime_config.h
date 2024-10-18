#ifndef __RUNTIME_CONFIG_HEADER_H__
#define __RUNTIME_CONFIG_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "static_config.h"


// Configuration Type Definitions --------------------------------------------------------------------------------------

typedef struct { uint32_t start_time, end_time; } start_end_time_t;

typedef enum { AMPLITUDE, SCHEDULED, INTERVAL, CONTINUOUS } audio_recording_mode_t;
typedef enum { NONE, ACTIVITY, AUDIO } imu_recording_mode_t;
typedef enum { SECONDS, MINUTES, HOURS, DAYS } time_scale_t;


// Public API Functions ------------------------------------------------------------------------------------------------

bool fetch_runtime_configuration(void);
void config_get_device_label(char *label, uint32_t max_size);
bool config_is_device_activated(void);
void config_set_activation_status(bool active);
bool config_gps_available(void);
bool config_awake_on_magnet(void);
uint32_t config_get_battery_mV_low(void);
int32_t config_get_utc_offset_hour(void);
int32_t config_get_utc_offset_seconds(void);
int32_t config_get_num_deployment_phases(void);
int32_t config_get_active_deployment_phase_index(uint32_t current_time);
int32_t config_get_next_deployment_phase_index(uint32_t current_time, uint32_t *next_start_time);
uint32_t config_get_deployment_start_time(void);
uint32_t config_get_deployment_end_time(void);
uint32_t config_get_magnetic_field_validation_length(void);
bool config_set_rtc_at_magnet_detect(void);
bool config_get_leds_enabled(void);
uint32_t config_get_leds_active_seconds(void);
uint32_t config_get_vhf_start_timestamp(void);
float config_get_mic_amplification_db(void);
uint32_t config_get_start_time(int32_t phase_index);
uint32_t config_get_end_time(int32_t phase_index);
audio_recording_mode_t config_get_audio_recording_mode(int32_t phase_index);
bool config_get_max_audio_clips(int32_t phase_index, uint32_t *max_clips, time_scale_t *unit_time);
float config_get_audio_trigger_threshold(int32_t phase_index);
uint32_t config_get_audio_trigger_schedule(int32_t phase_index, start_end_time_t **schedule);
bool config_get_audio_trigger_interval(int32_t phase_index, uint32_t *interval, time_scale_t *unit_time);
uint32_t config_get_audio_clip_length_seconds(int32_t phase_index);
bool config_extend_clip_for_continuous_audio(int32_t phase_index);
uint32_t config_get_audio_sampling_rate_hz(int32_t phase_index);
imu_recording_mode_t config_get_imu_recording_mode(int32_t phase_index);
float config_get_imu_trigger_threshold_level(int32_t phase_index);
uint8_t config_get_imu_degrees_of_freedom(int32_t phase_index);
uint32_t config_get_imu_sampling_rate_hz(int32_t phase_index);

#endif  // #ifndef __RUNTIME_CONFIG_HEADER_H__
