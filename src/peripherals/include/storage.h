#ifndef __STORAGE_HEADER_H__
#define __STORAGE_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef struct
{
   uint32_t write_failures;        // f_write calls that did not complete
   uint32_t reopen_recoveries;     // Files successfully reopened after a failure
   uint32_t remount_recoveries;    // Volumes successfully remounted after a failure
   uint32_t imu_buffers_dropped;   // IMU buffers discarded because storage could not keep up
   uint32_t consecutive_failures;  // Failures since the last successful write
} storage_health_t;


// Public API Functions ------------------------------------------------------------------------------------------------

// Typical storage maintenance functionality
void storage_init(void);
void storage_deinit(void);
bool storage_sd_card_error(void);
bool storage_mkdir(const char *directory);
bool storage_open(const char *file_path, bool writeable);
uint32_t storage_get_current_activation_number(const char *device_label);
bool storage_write(const void *data, uint32_t data_len);
uint32_t storage_read(uint8_t *read_buffer, uint32_t buffer_len);
int32_t storage_read_line(char *read_buffer, uint32_t buffer_len);
void storage_delete(const char *file_path);
void storage_close(void);

// Audio and IMU capture file functionality
bool storage_open_audio_file(uint32_t activation_number, const char *device_label, uint32_t num_channels, uint32_t sample_rate_hz, uint32_t current_time, bool use_ogg);
bool storage_open_named_wav_file(const char *file_path, uint32_t num_channels, uint32_t sample_rate_hz);
bool storage_open_imu_file(uint32_t activation_number, const char *device_label, uint32_t current_time, uint32_t sample_rate_hz);
bool storage_write_audio(const void *data, uint32_t data_len, bool is_last_packet);
void storage_write_imu_data(float accel_x_mg, float accel_y_mg, float accel_z_mg);
void storage_handle_imu_data(void);
void storage_close_audio(void);
void storage_set_measured_sample_rate(uint32_t sample_rate_hz);
void storage_close_imu(void);

// Storage recovery functionality
bool storage_recover_audio(uint32_t activation_number, const char *device_label, uint32_t num_channels, uint32_t sample_rate_hz, uint32_t current_time);
void storage_get_health(storage_health_t *health);
uint32_t storage_get_free_space_mb(void);

// Deployment log file functionality
void storage_setup_logs(void);
void storage_flush_early_log(void);
bool storage_rotate_log(uint32_t activation_number, const char *device_label, uint32_t current_time);
void storage_write_log(const char *fmt, ...);
void storage_write_event(const char *code, const char *fmt, ...);
void storage_flush_log(void);

// Fixed-record boot log functionality
bool storage_write_device_info(const char *fw_version, const char *hw_revision, const char *build_datetime, const char *device_uid, uint32_t activation_number, uint32_t timestamp, uint32_t battery_mv, const char *last_stop_reason, bool recovered);
bool storage_refresh_device_info(uint32_t activation_number, uint32_t timestamp, uint32_t battery_mv, const char *last_stop_reason, bool recovered);
void storage_write_boot_record(const char *reason, uint32_t epoch, uint32_t reset_count, uint32_t detail, uint32_t timestamp);

#endif  // #ifndef __STORAGE_HEADER_H__
