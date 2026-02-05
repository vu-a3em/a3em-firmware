#include "logging.h"
#include "magnet.h"
#include "system.h"

static void magnet_sensor_validated(bool validated)
{
   print("Magnetic field %s validated!\n", validated ? "SUCCESSFULLY" : "NOT");
}

static void magnet_sensor_activated(bool field_detected)
{
   if (field_detected)
   {
      print("Magnetic field detected...validating for %u ms\n", MAGNET_FIELD_DEFAULT_VALIDATION_LENGTH_MS);
      magnet_sensor_verify_field(MAGNET_FIELD_DEFAULT_VALIDATION_LENGTH_MS, magnet_sensor_validated);
   }
   else
      print("Magnetic field lost...\n");
}

int main(void)
{
   // Set up system hardware
   setup_hardware();
   magnet_sensor_init();
   system_enable_interrupts(true);

   // Initiate interrupt-based magnetic field detection
   magnet_sensor_register_callback(magnet_sensor_activated);
   print("Magnetic field detection started...\n");
   while (true)
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

   // Should never reach this point
   return 0;
}
