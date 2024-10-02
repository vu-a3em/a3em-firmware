// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "led.h"
#include "logging.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static bool leds_enabled;
static const am_devices_led_t leds[] = {
   {PIN_LED_RED,    AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
   {PIN_LED_GREEN,  AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
};


// Public API Functions ------------------------------------------------------------------------------------------------

void leds_init(void)
{
   // Initialize all LEDs and turn them off
   if (!leds_enabled)
   {
      leds_enabled = true;
      am_devices_led_array_init((am_devices_led_t*)leds, sizeof(leds) / sizeof(leds[0]));
      led_off(LED_ALL);
      print("INFO: LEDs are ENABLED\n");
   }
}

void leds_deinit(void)
{
   // Turn off all LEDs and disable them
   if (leds_enabled)
   {
      led_off(LED_ALL);
      am_devices_led_array_disable((am_devices_led_t*)leds, sizeof(leds) / sizeof(leds[0]));
      leds_enabled = false;
      print("INFO: LEDs are DISABLED\n");
   }
}

void led_on(led_color_t color)
{
   // Turn on the LED corresponding to the requested color
   if (leds_enabled)
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
   if (leds_enabled)
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
   if (leds_enabled)
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
