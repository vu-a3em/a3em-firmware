// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "datetime.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define SECONDS_PER_DAY    86400u

#define PLAUSIBLE_MIN_YEAR 2020
#define PLAUSIBLE_MAX_YEAR 2099


// Private Helper Functions --------------------------------------------------------------------------------------------

// Days elapsed since 1970-01-01 for a proleptic Gregorian calendar date, after Howard Hinnant's
// "chrono-Compatible Low-Level Date Algorithms". Valid for any year the 32-bit range can express
// and free of the iterative year-by-year loops that make the newlib implementations slow.
static int32_t days_from_civil(int32_t year, uint32_t month, uint32_t day)
{
   year -= (month <= 2);
   const int32_t era = (year >= 0 ? year : year - 399) / 400;
   const uint32_t year_of_era = (uint32_t)(year - (era * 400));                          // 0 - 399
   const uint32_t day_of_year = ((153 * (month + ((month > 2) ? -3 : 9))) + 2) / 5 + day - 1;  // 0 - 365
   const uint32_t day_of_era = (year_of_era * 365) + (year_of_era / 4) - (year_of_era / 100) + day_of_year;
   return (era * 146097) + (int32_t)day_of_era - 719468;
}

// Inverse of days_from_civil()
static void civil_from_days(int32_t days, int32_t *year, uint32_t *month, uint32_t *day)
{
   days += 719468;
   const int32_t era = (days >= 0 ? days : days - 146096) / 146097;
   const uint32_t day_of_era = (uint32_t)(days - (era * 146097));                        // 0 - 146096
   const uint32_t year_of_era = (day_of_era - (day_of_era / 1460) + (day_of_era / 36524) - (day_of_era / 146096)) / 365;
   const uint32_t day_of_year = day_of_era - ((365 * year_of_era) + (year_of_era / 4) - (year_of_era / 100));
   const uint32_t month_prime = ((5 * day_of_year) + 2) / 153;                           // 0 - 11
   *day = day_of_year - (((153 * month_prime) + 2) / 5) + 1;                             // 1 - 31
   *month = month_prime + ((month_prime < 10) ? 3 : -9);                                 // 1 - 12
   *year = (int32_t)year_of_era + (era * 400) + ((*month <= 2) ? 1 : 0);
}

// Write a zero-padded unsigned decimal value of exactly "num_digits" characters
static void write_padded(char *buffer, uint32_t value, uint32_t num_digits)
{
   for (uint32_t i = num_digits; i > 0; --i)
   {
      buffer[i - 1] = (char)('0' + (value % 10));
      value /= 10;
   }
}


// Public API Functions ------------------------------------------------------------------------------------------------

void datetime_from_timestamp(uint32_t timestamp, datetime_t *datetime)
{
   // Split the timestamp into whole days and the seconds remaining within the current day
   const uint32_t days = timestamp / SECONDS_PER_DAY;
   const uint32_t seconds_of_day = timestamp - (days * SECONDS_PER_DAY);

   // Convert the day count into a calendar date
   int32_t year = 0;
   uint32_t month = 0, day = 0;
   civil_from_days((int32_t)days, &year, &month, &day);

   // Populate the output structure
   datetime->year = (uint16_t)year;
   datetime->month = (uint8_t)month;
   datetime->day = (uint8_t)day;
   datetime->hour = (uint8_t)(seconds_of_day / 3600u);
   datetime->minute = (uint8_t)((seconds_of_day / 60u) % 60u);
   datetime->second = (uint8_t)(seconds_of_day % 60u);

   // 1970-01-01 was a Thursday, which is weekday index 4 counting from Sunday
   datetime->weekday = (uint8_t)((days + 4u) % 7u);
}

uint32_t datetime_to_timestamp(const datetime_t *datetime)
{
   // Reject any field that is out of range rather than silently producing a wild timestamp
   if ((datetime->month < 1) || (datetime->month > 12) || (datetime->day < 1) || (datetime->day > 31) ||
       (datetime->hour > 23) || (datetime->minute > 59) || (datetime->second > 59))
      return 0;

   // Convert the calendar date to a day count and add the time of day
   const int32_t days = days_from_civil((int32_t)datetime->year, datetime->month, datetime->day);
   if (days < 0)
      return 0;
   return ((uint32_t)days * SECONDS_PER_DAY) + (datetime->hour * 3600u) + (datetime->minute * 60u) + datetime->second;
}

bool datetime_is_plausible(const datetime_t *datetime)
{
   // Verify that every field is in range and that the year falls inside the supported window
   return (datetime->year >= PLAUSIBLE_MIN_YEAR) && (datetime->year <= PLAUSIBLE_MAX_YEAR) &&
          (datetime->month >= 1) && (datetime->month <= 12) &&
          (datetime->day >= 1) && (datetime->day <= 31) &&
          (datetime->hour <= 23) && (datetime->minute <= 59) && (datetime->second <= 59);
}

uint32_t datetime_format_stamp(char *buffer, uint32_t buffer_len, const datetime_t *datetime)
{
   // "YYYY-MM-DD HH-MM-SS" -- the separator between time fields is a dash because this string is
   // used as a FAT file name, where a colon is not a legal character.
   if (buffer_len < DATETIME_STAMP_LEN)
      return 0;
   write_padded(buffer, datetime->year, 4);
   buffer[4] = '-';
   write_padded(buffer + 5, datetime->month, 2);
   buffer[7] = '-';
   write_padded(buffer + 8, datetime->day, 2);
   buffer[10] = ' ';
   write_padded(buffer + 11, datetime->hour, 2);
   buffer[13] = '-';
   write_padded(buffer + 14, datetime->minute, 2);
   buffer[16] = '-';
   write_padded(buffer + 17, datetime->second, 2);
   buffer[19] = '\0';
   return DATETIME_STAMP_LEN - 1;
}

uint32_t datetime_format_date(char *buffer, uint32_t buffer_len, const datetime_t *datetime)
{
   // "YYYY-MM-DD"
   if (buffer_len < DATETIME_DATE_LEN)
      return 0;
   write_padded(buffer, datetime->year, 4);
   buffer[4] = '-';
   write_padded(buffer + 5, datetime->month, 2);
   buffer[7] = '-';
   write_padded(buffer + 8, datetime->day, 2);
   buffer[10] = '\0';
   return DATETIME_DATE_LEN - 1;
}

uint32_t datetime_format_hour(char *buffer, uint32_t buffer_len, const datetime_t *datetime)
{
   // "HH"
   if (buffer_len < DATETIME_HOUR_LEN)
      return 0;
   write_padded(buffer, datetime->hour, 2);
   buffer[2] = '\0';
   return DATETIME_HOUR_LEN - 1;
}
