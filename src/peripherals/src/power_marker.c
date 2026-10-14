// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "power_marker.h"

#ifdef POWER_MARKERS


// Static Global Variables ---------------------------------------------------------------------------------------------

static am_hal_gpio_pincfg_t saved_clk_config, saved_data_config;


// Private Helper Functions --------------------------------------------------------------------------------------------

static void marker_claim(void)
{
   // Take the lines over as open-drain outputs, releasing them before the switch so neither glitches low
   am_hal_gpio_pinconfig_get(PIN_EXT_HW_I2C_SCL, &saved_clk_config);
   am_hal_gpio_pinconfig_get(PIN_EXT_HW_I2C_SDA, &saved_data_config);
   am_hal_gpio_output_set(PIN_EXT_HW_I2C_SCL);
   am_hal_gpio_output_set(PIN_EXT_HW_I2C_SDA);
   am_hal_gpio_pinconfig(PIN_EXT_HW_I2C_SCL, am_hal_gpio_pincfg_opendrain);
   am_hal_gpio_pinconfig(PIN_EXT_HW_I2C_SDA, am_hal_gpio_pincfg_opendrain);
}

static void marker_release(void)
{
   // Let both lines float back up to the pull-ups and hand the pins back to their previous function
   am_hal_gpio_output_set(PIN_EXT_HW_I2C_SCL);
   am_hal_gpio_output_set(PIN_EXT_HW_I2C_SDA);
   am_hal_gpio_pinconfig(PIN_EXT_HW_I2C_SCL, saved_clk_config);
   am_hal_gpio_pinconfig(PIN_EXT_HW_I2C_SDA, saved_data_config);
}


// Public API Functions ------------------------------------------------------------------------------------------------

void power_marker_frame(uint8_t code)
{
   // Clock out the code most significant bit first, with DATA settled before each CLK pulse begins
   marker_claim();
   for (int32_t bit = 7; bit >= 0; --bit)
   {
      if (code & (1u << bit))
         am_hal_gpio_output_set(PIN_EXT_HW_I2C_SDA);
      else
         am_hal_gpio_output_clear(PIN_EXT_HW_I2C_SDA);
      am_hal_delay_us(POWER_MARKER_BIT_HIGH_US);
      am_hal_gpio_output_clear(PIN_EXT_HW_I2C_SCL);
      am_hal_delay_us(POWER_MARKER_BIT_LOW_US);
      am_hal_gpio_output_set(PIN_EXT_HW_I2C_SCL);
   }
   marker_release();
}

void power_marker_end(void)
{
   // One long CLK pulse, distinguishable from any data bit by its width alone
   marker_claim();
   am_hal_gpio_output_clear(PIN_EXT_HW_I2C_SCL);
   am_hal_delay_us(POWER_MARKER_END_LOW_US);
   marker_release();
}

#endif  // #ifdef POWER_MARKERS
