// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "vhf.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void vhf_init(void)
{
   // Initialize the VHF control GPIO
   const am_hal_gpio_pincfg_t enable_pin_config = AM_HAL_GPIO_PINCFG_OUTPUT;
   configASSERT0(am_hal_gpio_pinconfig(PIN_VHF_ENABLE, enable_pin_config));

   // Ensure that VHF is initially disabled
   am_hal_gpio_output_clear(PIN_VHF_ENABLE);
}

void vhf_deinit(void)
{
   // Disable VHF
   am_hal_gpio_output_clear(PIN_VHF_ENABLE);
}

void vhf_activate(void)
{
   // Enable VHF
   am_hal_gpio_output_set(PIN_VHF_ENABLE);
}

void vhf_deactivate(void)
{
   // Disable VHF
   am_hal_gpio_output_clear(PIN_VHF_ENABLE);
}
