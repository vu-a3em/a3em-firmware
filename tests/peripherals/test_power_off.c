#include "led.h"
#include "logging.h"
#include "rtc.h"
#include "system.h"

typedef enum { WAKE_WITH_RTC = 1, WAKE_WITH_PIN } wake_criteria_t;
static const wake_criteria_t wake_criteria = WAKE_WITH_RTC;
static const uint32_t wakeup_pin = 0;

int main(void)
{
   // Setup all hardware and fetch the device UID
   setup_hardware();
   static uint8_t device_id[DEVICE_ID_LEN];
   system_read_ID(device_id, sizeof(device_id));

   // Initialize all required peripherals
   leds_init();
   rtc_init();
   rtc_set_time_to_compile_time();
   system_enable_interrupts(true);

   // Flash the red LED for 2 seconds to show that we are awake
   led_on(LED_RED);
   am_hal_delay_us(2000000);
   led_off(LED_RED);

   // Enter power-down mode until awoken 5 seconds in the future (or by a GPIO pin level change)
   system_enter_power_off_mode(wakeup_pin, (wake_criteria == WAKE_WITH_RTC) ? rtc_get_timestamp() + 5 : 0);
   system_reset();

   // Should never reach this point
   return 0;
}
