// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "rtc.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static const char *month_strings[] = { "Invalid Entry", "January", "February", "March", "April", "May", "June", "July",
                                       "August", "September", "October", "November", "December", "Invalid Month" };
static const char *day_strings[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
static volatile uint32_t rtc_stat, wakeup_timestamp;


// Private Helper Functions --------------------------------------------------------------------------------------------

static int to_val(const char *ascii_string)
{
   int val = ascii_string[1] - '0';
   val += (ascii_string[0] == ' ') ? 0 : ((ascii_string[0] - '0') * 10);
   return val;
}

static int month_to_index(const char *month_string)
{
   // Bound is inclusive so that December is matched by the loop rather than only by the fallback
   for (int i = 1; i <= 12; ++i)
      if (am_util_string_strnicmp(month_strings[i], month_string, 3) == 0)
         return i;
   return 12;
}

static int day_to_index(const char *day_string)
{
   for (int i = 0; i < 7; ++i)
      if (am_util_string_strnicmp(day_strings[i], day_string, 3) == 0)
         return i;
   return 0;
}

static uint32_t to_unix_timestamp(const am_hal_rtc_time_t *time)
{
   const datetime_t datetime = {
      .year = (uint16_t)(2000 + time->ui32Year),
      .month = (uint8_t)time->ui32Month,
      .day = (uint8_t)time->ui32DayOfMonth,
      .hour = (uint8_t)time->ui32Hour,
      .minute = (uint8_t)time->ui32Minute,
      .second = (uint8_t)time->ui32Second,
      .weekday = (uint8_t)time->ui32Weekday
   };
   return datetime_to_timestamp(&datetime);
}

static am_hal_rtc_time_t to_rtc_time(uint32_t unix_timestamp)
{
   datetime_t datetime;
   datetime_from_timestamp(unix_timestamp, &datetime);
   am_hal_rtc_time_t new_rtc_time = {
      .ui32ReadError = 0,
      .ui32CenturyBit = RTC_CTRUP_CB_2000,
      .ui32Hour = datetime.hour,
      .ui32Minute = datetime.minute,
      .ui32Second = datetime.second,
      .ui32Hundredths = 0,
      .ui32Weekday = datetime.weekday,
      .ui32DayOfMonth = datetime.day,
      .ui32Month = datetime.month,
      .ui32Year = (datetime.year >= 2000) ? (datetime.year - 2000) : 0
   };
   return new_rtc_time;
}

static void arm_hardware_alarm(uint32_t timestamp)
{
   // Arm the hardware alarm for the stored target timestamp
   am_hal_rtc_time_t wakeup_time = to_rtc_time(timestamp);
   am_hal_rtc_alarm_set(&wakeup_time, AM_HAL_RTC_ALM_RPT_YR);
   am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);
   am_hal_rtc_interrupt_enable(AM_HAL_RTC_INT_ALM);
   NVIC_SetPriority(RTC_IRQn, RTC_ALARM_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(RTC_IRQn);
}

static uint32_t read_timestamp_locked(void)
{
   rtc_stat = RTC->RTCSTAT;  // Read RTCSTAT to mitigate RTC hanging as per errata
   static am_hal_rtc_time_t rtc_time;
   return (am_hal_rtc_time_get(&rtc_time) == AM_HAL_STATUS_SUCCESS) ? to_unix_timestamp(&rtc_time) : 0;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void rtc_init(void)
{
   // Enable the XT clock for the RTC
   wakeup_timestamp = 0;
   configASSERT0(am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL, NULL));
   am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);

   // Enable the RTC
   configASSERT0(am_hal_rtc_osc_enable());
}

void rtc_set_time_to_compile_time(void)
{
   const char _datetime[] = _DATETIME;  // Format: "Tue Jan  1 00:00:00 UTC 2000"
   am_hal_rtc_time_t new_rtc_time = {
      .ui32ReadError = 0,
      .ui32CenturyBit = RTC_CTRUP_CB_2000,
      .ui32Hour = to_val(&_datetime[11]),
      .ui32Minute = to_val(&_datetime[14]),
      .ui32Second = to_val(&_datetime[17]),
      .ui32Hundredths = 0,
      .ui32Weekday = day_to_index(&_datetime[0]),
      .ui32DayOfMonth = to_val(&_datetime[8]),
      .ui32Month = month_to_index(&_datetime[4]),
      .ui32Year = to_val(&_datetime[26])
   };
   configASSERT0(am_hal_rtc_time_set(&new_rtc_time));
}

bool rtc_set_time_from_timestamp(uint32_t timestamp)
{
   bool success = false;
   AM_CRITICAL_BEGIN
   am_hal_rtc_time_t new_rtc_time = to_rtc_time(timestamp);
   success = (am_hal_rtc_time_set(&new_rtc_time) == AM_HAL_STATUS_SUCCESS);

   // Re-arm any pending alarm against the new clock value
   if (success && wakeup_timestamp)
   {
      if (timestamp >= wakeup_timestamp)
      {
         // The deadline is already behind us, so raise the interrupt now rather than waiting
         am_hal_rtc_interrupt_set(AM_HAL_RTC_INT_ALM);
         NVIC_SetPriority(RTC_IRQn, RTC_ALARM_INTERRUPT_PRIORITY);
         NVIC_EnableIRQ(RTC_IRQn);
      }
      else
         arm_hardware_alarm(wakeup_timestamp);
   }
   AM_CRITICAL_END
   return success;
}

void rtc_set_wakeup_timestamp(uint32_t timestamp)
{
   AM_CRITICAL_BEGIN
   wakeup_timestamp = timestamp;
   arm_hardware_alarm(timestamp);
   AM_CRITICAL_END
}

uint32_t rtc_get_wakeup_timestamp(void)
{
   return wakeup_timestamp;
}

bool rtc_wakeup_elapsed(void)
{
   // Report whether the recorded deadline has passed, independently of whether the hardware alarm actually fired
   const uint32_t target = wakeup_timestamp;
   if (!target)
      return false;
   const uint32_t now = rtc_get_timestamp();
   return now && (now >= target);
}

void rtc_clear_wakeup(void)
{
   AM_CRITICAL_BEGIN
   wakeup_timestamp = 0;
   am_hal_rtc_interrupt_disable(AM_HAL_RTC_INT_ALM);
   am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);
   AM_CRITICAL_END
}

uint32_t rtc_get_timestamp(void)
{
   uint32_t timestamp = 0;
   AM_CRITICAL_BEGIN
   timestamp = read_timestamp_locked();
   AM_CRITICAL_END
   return timestamp;
}

uint32_t rtc_get_time_of_day(void)
{
   uint32_t timestamp = 0;
   AM_CRITICAL_BEGIN
   rtc_stat = RTC->RTCSTAT;  // Read RTCSTAT to mitigate RTC hanging as per errata
   static am_hal_rtc_time_t rtc_time;
   timestamp = (am_hal_rtc_time_get(&rtc_time) == AM_HAL_STATUS_SUCCESS) ? ((3600 * rtc_time.ui32Hour) + (60 * rtc_time.ui32Minute) + rtc_time.ui32Second) : 0;
   AM_CRITICAL_END
   return timestamp;
}

bool rtc_is_valid(void)
{
   // Validate against the shared plausibility window
   bool success = false;
   AM_CRITICAL_BEGIN
   rtc_stat = RTC->RTCSTAT;  // Read RTCSTAT to mitigate RTC hanging as per errata
   static am_hal_rtc_time_t rtc_time;
   if (am_hal_rtc_time_get(&rtc_time) == AM_HAL_STATUS_SUCCESS)
   {
      const datetime_t datetime = {
         .year = (uint16_t)(2000 + rtc_time.ui32Year),
         .month = (uint8_t)rtc_time.ui32Month,
         .day = (uint8_t)rtc_time.ui32DayOfMonth,
         .hour = (uint8_t)rtc_time.ui32Hour,
         .minute = (uint8_t)rtc_time.ui32Minute,
         .second = (uint8_t)rtc_time.ui32Second,
         .weekday = (uint8_t)rtc_time.ui32Weekday
      };
      success = datetime_is_plausible(&datetime);
   }
   AM_CRITICAL_END
   return success;
}
