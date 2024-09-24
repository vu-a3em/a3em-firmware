#include "logging.h"
#include "system.h"

static uint8_t device_id[DEVICE_ID_LEN];

int main(void)
{
   // Set up the system hardware and retrieve the device ID
   setup_hardware();
   system_read_ID(device_id, sizeof(device_id));
   print("System initialized, ID = ");
   for (size_t i = DEVICE_ID_LEN - 1; i > 0; --i)
      print("%02X:", device_id[i]);
   print("%02X\n", device_id[0]);

   // Put the CPU into deep sleep forever
   while (true)
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

   // Should never reach this point
   return 0;
}
