#ifndef __SELF_TEST_HEADER_H__
#define __SELF_TEST_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

// Which subsystem failed, if any. The blink count on the red LED matches the numeric
// value, so a failure can be read off a sealed unit with no computer present.
typedef enum {
   SELF_TEST_PASS = 0,
   SELF_TEST_FAIL_MICROPHONE = 1,
   SELF_TEST_FAIL_STORAGE = 2,
   SELF_TEST_FAIL_IMU = 3,
   SELF_TEST_FAIL_POWER_OR_CLOCK = 4
} self_test_result_t;


// Public API Functions ------------------------------------------------------------------------------------------------

// Verifies the hardware immediately after magnetic activation, before any deployment
// recording begins.
//
// Runs once per activation. Broken microphone wiring, an unseated SD card, and a
// microphone port sealed shut by potting compound have all cost deployments, and none
// of them are detectable once a unit is closed up and in the field.
//
// While the microphone window is recording, the green LED tracks the audio level about
// once a second, so tapping the enclosure and watching the LED confirms the acoustic
// path all the way through the housing -- something no electrical check can establish.
self_test_result_t run_self_test(void);

#endif  // #ifndef __SELF_TEST_HEADER_H__
