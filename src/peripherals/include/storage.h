#ifndef __STORAGE_HEADER_H__
#define __STORAGE_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void storage_init(void);
void storage_deinit(void);
void storage_setup_logs(void);
bool storage_sd_card_error(void);
bool storage_mkdir(const char *directory);
bool storage_open(const char *file_path, bool writeable);
bool storage_open_wav_file(const char *device_label, uint32_t num_channels, uint32_t sample_rate_hz, uint32_t current_time);
bool storage_open_imu_file(void);
bool storage_write(const void *data, uint32_t data_len);
bool storage_write_audio(const void *data, uint32_t data_len);
void storage_write_log(const char *fmt, ...);
void storage_flush_log(void);
bool storage_start_imu_data_stream(uint32_t timestamp, uint32_t sample_rate_hz);
bool storage_write_imu_data(const void *data, uint32_t data_len);
void storage_finish_imu_data_stream(void);
uint32_t storage_read(uint8_t *read_buffer, uint32_t buffer_len);
int32_t storage_read_line(char *read_buffer, uint32_t buffer_len);
void storage_delete(const char *file_path);
void storage_close_audio(void);
void storage_close(void);

#endif  // #ifndef __STORAGE_HEADER_H__
