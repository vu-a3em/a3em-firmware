#include "battery.h"
#include "logging.h"
#include "system.h"

int main(void)
{
   // Set up system hardware
   setup_hardware();
   battery_monitor_init();
   system_enable_interrupts(true);

   // Retrieve battery status every 15 seconds
   while (true)
   {
      battery_result_t details = battery_monitor_get_details();
      print("Battery Status: %u mV, Level is %s, Temp is %f C\n",
            details.millivolts,
            (details.millivolts <= BATTERY_DEFAULT_LOW_LEVEL_MV) ? "LOW" : "GOOD",
            details.celcius);
      am_hal_delay_us(15000000);
   }

   // Should never reach this point
   return 0;
}
