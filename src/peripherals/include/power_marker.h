#ifndef __POWER_MARKER_HEADER_H__
#define __POWER_MARKER_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

// Markers align a current capture with what the firmware was doing at the time. They are sent on the two external I2C
// test points, which are open-drain and pulled up off-board: both lines idle released, so they cost nothing between
// frames. A frame is eight short CLK low pulses with an 8-bit code on DATA, most significant bit first, sampled while
// CLK is low. A single long CLK low pulse closes the window the preceding frame opened; a frame with no closing pulse
// before the next one marks a point in time instead. A line held low sinks current through its pull-up, so frames are
// sent only at the edges of what they mark, never inside a measurement.

#define POWER_MARKER_BIT_LOW_US                     100
#define POWER_MARKER_BIT_HIGH_US                    100
#define POWER_MARKER_END_LOW_US                     1000

typedef enum {
   POWER_MARK_BOOT = 0xF0,
   POWER_MARK_PHASE_START,
   POWER_MARK_PHASE_END,
   POWER_MARK_SELF_TEST_START,
   POWER_MARK_SELF_TEST_END,
   POWER_MARK_POWER_OFF
} power_mark_t;


// Public API Functions ------------------------------------------------------------------------------------------------

#ifdef POWER_MARKERS

void power_marker_frame(uint8_t code);
void power_marker_end(void);
#define power_marker_point(code) do { if (!config_gps_available()) power_marker_frame(code); } while (0)

#else

#define power_marker_frame(code)
#define power_marker_end()
#define power_marker_point(code)

#endif

#endif  // #ifndef __POWER_MARKER_HEADER_H__
