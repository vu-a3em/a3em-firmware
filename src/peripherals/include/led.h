#ifndef __LED_HEADER_H__
#define __LED_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef enum { LED_RED, LED_GREEN, LED_ALL } led_color_t;


// Public API Functions ------------------------------------------------------------------------------------------------

void leds_init(void);
void leds_deinit(void);
void leds_enable(bool enable);
bool leds_are_enabled(void);
void led_on(led_color_t color);
void led_off(led_color_t color);
void led_toggle(led_color_t color);
void led_indicate_clip_begin(void);
void led_indicate_clip_progress(void);
void led_indicate_clip_end(void);
void led_indicate_error(void);
void led_indicate_magnet_presence(bool field_present);
void led_indicate_activation(bool activated);

#endif  // #ifndef __LED_HEADER_H__
