// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "led.h"
#include "logging.h"
#include "system.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static bool leds_enabled = false, leds_initialized = false;
static const am_devices_led_t leds[] = {
   {PIN_LED_RED,    AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
   {PIN_LED_GREEN,  AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
};


// Public API Functions ------------------------------------------------------------------------------------------------

void leds_init(void)
{
   // Initialize all LEDs and turn them off
   if (!leds_initialized)
   {
      leds_initialized = true;
      am_devices_led_array_init((am_devices_led_t*)leds, sizeof(leds) / sizeof(leds[0]));
      led_off(LED_ALL);
   }
}

void leds_deinit(void)
{
   // Turn off all LEDs and disable them
   if (leds_initialized)
   {
      led_off(LED_ALL);
      am_devices_led_array_disable((am_devices_led_t*)leds, sizeof(leds) / sizeof(leds[0]));
      leds_initialized = false;
   }
}

void leds_enable(bool enable)
{
   // Enable or disable standard LED usage
   print("INFO: LEDs are %s\n", enable ? "ENABLED" : "DISABLED");
   leds_enabled = enable;
   led_off(LED_ALL);
}

bool leds_are_enabled(void)
{
   // Return whether the LEDs are currently enabled
   return leds_enabled;
}

void led_on(led_color_t color)
{
   // Turn on the LED corresponding to the requested color
   if (leds_initialized)
   {
      switch (color)
      {
         case LED_RED:
            am_devices_led_on((am_devices_led_t*)leds, 0);
            break;
         case LED_GREEN:
            am_devices_led_on((am_devices_led_t*)leds, 1);
            break;
         case LED_ALL:
            am_devices_led_on((am_devices_led_t*)leds, 0);
            am_devices_led_on((am_devices_led_t*)leds, 1);
            break;
         default:
            break;
      }
   }
}

void led_off(led_color_t color)
{
   // Turn off the LED corresponding to the requested color
   if (leds_initialized)
   {
      switch (color)
      {
         case LED_RED:
            am_devices_led_off((am_devices_led_t*)leds, 0);
            break;
         case LED_GREEN:
            am_devices_led_off((am_devices_led_t*)leds, 1);
            break;
         case LED_ALL:
            am_devices_led_off((am_devices_led_t*)leds, 0);
            am_devices_led_off((am_devices_led_t*)leds, 1);
            break;
         default:
            break;
      }
   }
}

void led_toggle(led_color_t color)
{
   // Toggle the LED corresponding to the requested color
   if (leds_initialized)
   {
      switch (color)
      {
         case LED_RED:
            am_devices_led_toggle((am_devices_led_t*)leds, 0);
            break;
         case LED_GREEN:
            am_devices_led_toggle((am_devices_led_t*)leds, 1);
            break;
         case LED_ALL:
            am_devices_led_toggle((am_devices_led_t*)leds, 0);
            am_devices_led_toggle((am_devices_led_t*)leds, 1);
            break;
         default:
            break;
      }
   }
}

void led_indicate_clip_begin(void)
{
   if (leds_enabled)
   {
      led_off(LED_ALL);
      for (int i = 0; i < 3; ++i)
      {
         led_on(LED_GREEN);
         system_delay(20000);
         led_off(LED_GREEN);
         system_delay(20000);
      }
   }
}

void led_indicate_clip_progress(void)
{
   if (leds_enabled)
      led_toggle(LED_GREEN);
}

void led_indicate_clip_end(void)
{
   if (leds_enabled)
      led_off(LED_ALL);
}

void led_indicate_sd_card_error(void)
{
   led_off(LED_ALL);
   for (int i = 0; i < 20; ++i)
   {
      led_on(LED_RED);
      system_delay(150000);
      led_off(LED_RED);
      system_delay(100000);
   }
}

void led_indicate_missing_config_file(void)
{
   led_off(LED_ALL);
   for (int i = 0; i < 20; ++i)
   {
      led_on(LED_ALL);
      system_delay(150000);
      led_off(LED_ALL);
      system_delay(100000);
   }
}

void led_indicate_magnet_presence(bool field_present)
{
   if (field_present)
      led_on(LED_ALL);
   else
      led_off(LED_ALL);
}

void led_indicate_activation(bool activated)
{
   led_off(LED_ALL);
   for (int i = 0; i < 30; ++i)
   {
      led_on(activated ? LED_GREEN : LED_RED);
      system_delay(100000);
      led_off(activated ? LED_GREEN : LED_RED);
      system_delay(100000);
   }
}

void led_toggle_validation_phase_change(void)
{
   led_toggle(LED_ALL);
}

void led_indicate_validation_failed(void)
{
   led_off(LED_RED);
   led_on(LED_GREEN);
   for (int i = 0; i < 6; ++i)
   {
      led_on(LED_RED);
      led_off(LED_GREEN);
      system_delay(100000);
      led_off(LED_RED);
      led_on(LED_GREEN);
      system_delay(100000);
   }
   led_off(LED_ALL);
}
