// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "led.h"
#include "logging.h"
#include "system.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define led_status_timer_isr    am_timer_isr1(TIMER_NUMBER_STATUS_LED)

typedef struct
{
   led_color_t on_color;      // Driven during the first half of each cycle
   led_color_t off_color;     // Driven during the second half (LED_NONE for dark)
   led_color_t final_color;   // Left lit once the pattern completes (LED_NONE for dark)
   uint16_t on_ms, off_ms, cycles;
} led_pattern_t;

static bool leds_enabled = false, leds_initialized = false;
static const am_devices_led_t leds[] = {
   {PIN_LED_RED,    AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
   {PIN_LED_GREEN,  AM_DEVICES_LED_ON_LOW | AM_DEVICES_LED_POL_OPEN_DRAIN},
};
static volatile led_pattern_t active_pattern;
static volatile uint16_t pattern_cycles_remaining;
static volatile bool pattern_running, pattern_in_on_phase;


// Blink Pattern Engine ------------------------------------------------------------------------------------------------

static void drive_color(led_color_t color)
{
   switch (color)
   {
      case LED_RED:
         led_on(LED_RED);
         led_off(LED_GREEN);
         break;
      case LED_GREEN:
         led_on(LED_GREEN);
         led_off(LED_RED);
         break;
      case LED_ALL:
         led_on(LED_ALL);
         break;
      case LED_NONE:   // Intentional fall-through
      default:
         led_off(LED_ALL);
         break;
   }
}

static uint32_t ms_to_ticks(uint32_t milliseconds)
{
   uint32_t ticks = (uint32_t)(((uint64_t)milliseconds * TIMER_STATUS_LED_TICK_RATE) / 1000u);
   return ticks ? ticks : 1;
}

void led_status_timer_isr(void)
{
   am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUMBER_STATUS_LED, AM_HAL_TIMER_COMPARE_BOTH));
   if (!pattern_running)
   {
      am_hal_timer_clear_stop(TIMER_NUMBER_STATUS_LED);
      return;
   }

   if (pattern_in_on_phase)
   {
      // Finished the lit half of this cycle
      pattern_in_on_phase = false;
      drive_color(active_pattern.off_color);
      am_hal_timer_compare0_set(TIMER_NUMBER_STATUS_LED, ms_to_ticks(active_pattern.off_ms));
      am_hal_timer_clear(TIMER_NUMBER_STATUS_LED);
   }
   else if (--pattern_cycles_remaining)
   {
      // Start the next cycle
      pattern_in_on_phase = true;
      drive_color(active_pattern.on_color);
      am_hal_timer_compare0_set(TIMER_NUMBER_STATUS_LED, ms_to_ticks(active_pattern.on_ms));
      am_hal_timer_clear(TIMER_NUMBER_STATUS_LED);
   }
   else
   {
      // Pattern complete
      am_hal_timer_clear_stop(TIMER_NUMBER_STATUS_LED);
      drive_color(active_pattern.final_color);
      pattern_running = false;
   }
}

static void led_pattern_start(const led_pattern_t *pattern)
{
   if (!leds_initialized || !pattern->cycles)
      return;

   // Replace any pattern already in flight
   am_hal_timer_clear_stop(TIMER_NUMBER_STATUS_LED);
   AM_CRITICAL_BEGIN
   active_pattern = *pattern;
   pattern_cycles_remaining = pattern->cycles;
   pattern_in_on_phase = true;
   pattern_running = true;
   AM_CRITICAL_END

   // Light the first half-cycle immediately so the feedback is instant
   drive_color(pattern->on_color);
   am_hal_timer_compare0_set(TIMER_NUMBER_STATUS_LED, ms_to_ticks(pattern->on_ms));
   am_hal_timer_clear(TIMER_NUMBER_STATUS_LED);
}

// Public API Functions ------------------------------------------------------------------------------------------------

void leds_init(void)
{
   // Initialize all LEDs and turn them off
   if (!leds_initialized)
   {
      leds_initialized = true;
      am_devices_led_array_init((am_devices_led_t*)leds, sizeof(leds) / sizeof(leds[0]));
      led_off(LED_ALL);

      // Set up the blink pattern timer, clocked from the 32 kHz crystal
      pattern_running = false;
      am_hal_timer_config_t pattern_timer_config;
      am_hal_timer_default_config_set(&pattern_timer_config);
      pattern_timer_config.eInputClock = TIMER_STATUS_LED_CLOCK;
      pattern_timer_config.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
      pattern_timer_config.ui32Compare0 = TIMER_STATUS_LED_TICK_RATE;
      am_hal_timer_config(TIMER_NUMBER_STATUS_LED, &pattern_timer_config);
      am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(TIMER_NUMBER_STATUS_LED, AM_HAL_TIMER_COMPARE0));
      NVIC_SetPriority(TIMER0_IRQn + TIMER_NUMBER_STATUS_LED, STATUS_LED_TIMER_INTERRUPT_PRIORITY);
      NVIC_EnableIRQ(TIMER0_IRQn + TIMER_NUMBER_STATUS_LED);
      am_hal_timer_clear_stop(TIMER_NUMBER_STATUS_LED);
   }
}

void leds_deinit(void)
{
   // Turn off all LEDs and disable them
   if (leds_initialized)
   {
      led_pattern_cancel();
      NVIC_DisableIRQ(TIMER0_IRQn + TIMER_NUMBER_STATUS_LED);
      am_hal_timer_interrupt_disable(AM_HAL_TIMER_MASK(TIMER_NUMBER_STATUS_LED, AM_HAL_TIMER_COMPARE0));
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

bool led_pattern_active(void)
{
   return pattern_running;
}

void led_pattern_wait(void)
{
   // Hold until the pattern finishes to show a confirmation and then reset the device
   for (uint32_t iterations = 0; pattern_running && (iterations < LED_PATTERN_MAX_WAIT_WAKEUPS); ++iterations)
   {
      system_feed_watchdog();
      if (pattern_running)
         system_enter_deep_sleep_mode();
   }
   if (pattern_running)
   {
      // The pattern stalled; abandon it rather than blocking the caller any longer
      led_pattern_cancel();
   }
}

void led_pattern_cancel(void)
{
   am_hal_timer_clear_stop(TIMER_NUMBER_STATUS_LED);
   pattern_running = false;
   led_off(LED_ALL);
}

void led_indicate_clip_begin(void)
{
   // Three quick green flashes, then green held steady for the duration of the clip
   if (leds_enabled)
   {
      static const led_pattern_t pattern = { LED_GREEN, LED_NONE, LED_GREEN, 20, 20, 3 };
      led_pattern_start(&pattern);
   }
}

void led_indicate_clip_progress(void)
{
   // Skipped while a pattern is running so the two do not fight over the LEDs
   if (leds_enabled && !pattern_running)
      led_toggle(LED_GREEN);
}

void led_indicate_clip_end(void)
{
   if (leds_enabled && !pattern_running)
      led_off(LED_ALL);
}

void led_indicate_sd_card_error(void)
{
   // Red flashes that are not gated on leds_enabled
   static const led_pattern_t pattern = { LED_RED, LED_NONE, LED_NONE, 150, 100, 20 };
   led_pattern_start(&pattern);
}

void led_indicate_missing_config_file(void)
{
   // Both LEDs flashing together distinguishes this from the SD card fault above
   static const led_pattern_t pattern = { LED_ALL, LED_NONE, LED_NONE, 150, 100, 20 };
   led_pattern_start(&pattern);
}

void led_indicate_magnet_presence(bool field_present, bool device_activated)
{
   // Immediate "field detected, keep holding" feedback, deliberately ungated
   if (!pattern_running)
   {
      led_off(LED_ALL);
      if (field_present)
         led_on(device_activated ? LED_GREEN : LED_RED);
   }
}

void led_indicate_activation(bool activated)
{
   // Green means the device is now active, red means it is now inactive.
   static led_pattern_t pattern = { LED_GREEN, LED_NONE, LED_NONE, 100, 100, 25 };
   pattern.on_color = activated ? LED_GREEN : LED_RED;
   led_pattern_start(&pattern);
}

void led_toggle_validation_phase_change(void)
{
   if (!pattern_running)
      led_toggle(LED_ALL);
}

void led_indicate_validation_failed(void)
{
   // Alternating red and green, the counterpart to led_indicate_activation()
   static const led_pattern_t pattern = { LED_RED, LED_GREEN, LED_NONE, 100, 100, 6 };
   led_pattern_start(&pattern);
}
