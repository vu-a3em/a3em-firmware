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
      uint32_t battery_voltage = battery_monitor_get_level_mV();
      print("Battery Status: %u mV, Level is %s\n",
            battery_voltage,
            (battery_voltage <= BATTERY_EMPTY) ? "EMPTY" :
               (battery_voltage <= BATTERY_CRITICAL) ? "CRITICAL" :
                  (battery_voltage <= BATTERY_NOMINAL) ? "NOMINAL" : "FULL");
      am_hal_delay_us(15000000);
   }

   // Should never reach this point
   return 0;
}
