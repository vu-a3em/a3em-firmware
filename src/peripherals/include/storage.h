#ifndef __STORAGE_HEADER_H__
#define __STORAGE_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void storage_init(void);
void storage_deinit(void);
bool storage_chdir(const char *directory);
bool storage_mkdir(const char *directory);
bool storage_open(const char *file_path, bool writeable);
bool storage_write(const void *data, uint32_t data_len);
bool storage_write_wav_header(uint32_t num_channels, uint32_t sample_rate_hz);
uint32_t storage_read(uint8_t *read_buffer, uint32_t buffer_len);
uint32_t storage_read_line(char *read_buffer, uint32_t buffer_len);
void storage_delete(const char *file_path);
void storage_close(void);

#endif  // #ifndef __STORAGE_HEADER_H__
