#ifndef __DATETIME_HEADER_H__
#define __DATETIME_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "static_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

// Replaces "struct tm" so that no code path reaches malloc()
typedef struct
{
   uint16_t year;      // Full year, e.g. 2026
   uint8_t month;      // 1 - 12
   uint8_t day;        // 1 - 31
   uint8_t hour;       // 0 - 23
   uint8_t minute;     // 0 - 59
   uint8_t second;     // 0 - 59
   uint8_t weekday;    // 0 = Sunday ... 6 = Saturday
} datetime_t;

// Minimum buffer sizes for the formatting functions below, including the NUL terminator
#define DATETIME_STAMP_LEN    20   // "YYYY-MM-DD HH-MM-SS"
#define DATETIME_DATE_LEN     11   // "YYYY-MM-DD"
#define DATETIME_HOUR_LEN      3   // "HH"


// Public API Functions ------------------------------------------------------------------------------------------------

// All conversions are pure UTC and fully reentrant
void datetime_from_timestamp(uint32_t timestamp, datetime_t *datetime);
uint32_t datetime_to_timestamp(const datetime_t *datetime);
bool datetime_is_plausible(const datetime_t *datetime);

// Formatting helpers. Each writes a NUL-terminated string and returns the number of characters
// written excluding the terminator, or 0 if the supplied buffer is too small.
uint32_t datetime_format_stamp(char *buffer, uint32_t buffer_len, const datetime_t *datetime);
uint32_t datetime_format_date(char *buffer, uint32_t buffer_len, const datetime_t *datetime);
uint32_t datetime_format_hour(char *buffer, uint32_t buffer_len, const datetime_t *datetime);

#endif  // #ifndef __DATETIME_HEADER_H__
