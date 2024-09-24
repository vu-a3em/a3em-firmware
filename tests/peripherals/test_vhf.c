#include "logging.h"
#include "system.h"
#include "vhf.h"

int main(void)
{
   // Set up system hardware
   setup_hardware();
   vhf_init();
   system_enable_interrupts(true);

   // Toggle VHF activation status every 10 seconds
   while (true)
   {
      vhf_deactivate();
      print("VHF disabled\n");
      for (int i = 0; i < 10; ++i)
         am_hal_delay_us(1000000);
      vhf_activate();
      print("VHF enabled\n");
      for (int i = 0; i < 10; ++i)
         am_hal_delay_us(1000000);
   }

   // Should never reach this point
   return 0;
}
