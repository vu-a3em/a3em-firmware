#include "digipot.h"
#include "logging.h"
#include "system.h"


int main(void)
{
   // Set up system hardware
   setup_hardware();
   digipot_init();
   system_enable_interrupts(true);

   // Turn on the external microphone
   const am_hal_gpio_pincfg_t mic_en_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   configASSERT0(am_hal_gpio_pinconfig(PIN_MICROPHONE_ENABLE, mic_en_config));
   am_hal_gpio_output_set(PIN_MICROPHONE_ENABLE);

   // Increase the digipot output every 2 seconds
   float level = 0.0;
   while (true)
   {
      print("Setting digipot level to %u%%\n", (uint32_t)(100.0*level));
      digipot_set_percent_output(level);
      am_hal_delay_us(2000000);
      level = (level < 1.0) ? (level + 0.1) : 0.0;
   }

   // Should never reach this point
   return 0;
}
