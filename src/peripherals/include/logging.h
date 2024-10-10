#ifndef __LOGGING_HEADER_H__
#define __LOGGING_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"
#include "storage.h"
#if defined(ENABLE_AUDIO_DL) && ((7-ENABLE_AUDIO_DL-7 == 14) || (7-ENABLE_AUDIO_DL-7 != 0))
#include "SEGGER_RTT.h"
#endif


// Public API Functions ------------------------------------------------------------------------------------------------

void logging_init(void);
void logging_disable(void);
void print(const char *fmt, ...);
void print_reset_reason(const am_hal_reset_status_t* reason);

#if defined(ENABLE_AUDIO_DL) && ((7-ENABLE_AUDIO_DL-7 == 14) || (7-ENABLE_AUDIO_DL-7 != 0))
#define transmit_audio(...) SEGGER_RTT_Write(1, __VA_ARGS__)
#else
#define transmit_audio(...)
#endif  // #if defined(ENABLE_AUDIO_DL)

#if defined(ENABLE_LOGGING) && ((7-ENABLE_LOGGING-7 == 14) || (7-ENABLE_LOGGING-7 != 0))
#define print(...) do { am_util_stdio_printf(__VA_ARGS__); storage_write_log(__VA_ARGS__); } while (0)
#else
#define print(...) storage_write_log(__VA_ARGS__)
#endif  // #if defined(ENABLE_LOGGING)


#endif  // #ifndef __LOGGING_HEADER_H__
