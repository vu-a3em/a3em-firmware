#ifndef __STATIC_CONFIG_HEADER_H__
#define __STATIC_CONFIG_HEADER_H__

// Application Configuration -------------------------------------------------------------------------------------------

#ifdef AM_DEBUG_PRINTF

#define ENABLE_LOGGING  1
#ifndef ENABLE_AUDIO_DL
#define ENABLE_AUDIO_DL 0
#endif

#endif


// Common Header Inclusions --------------------------------------------------------------------------------------------

#ifndef __cplusplus
#include <stdlib.h>
#include <string.h>
#include "am_bsp.h"
#include "am_util.h"
#include "pinout.h"
#endif


// Common Application Definitions --------------------------------------------------------------------------------------

#define DEVICE_ID_LEN                               6
#define MAX_DEVICE_LABEL_LEN                        31
#define MAX_AUDIO_TRIGGER_TIMES                     12
#define MAX_NUM_DEPLOYMENT_PHASES                   6
#define MAX_CFG_FILE_LINE_LENGTH                    80
#define MIN_LOG_DATA_INTERVAL_SECONDS               300
#define NUM_HOURS_PER_AUDIO_DIRECTORY               4
#define AUDIO_BUFFER_MAX_SIZE                       65536
#define NUM_SECONDS_PER_AUDIO_DIRECTORY             (NUM_HOURS_PER_AUDIO_DIRECTORY * 60 * 60)

#define CONFIG_FILE_NAME                            "_a3em.cfg"
#define LOG_FILE_NAME                               "a3em.log"
#define DEVICE_INFO_FILE_NAME                       "_a3em.dev"

// Alternative log names tried within a timestamped directory before giving up. Log
// entries belong beside the audio they describe, so a damaged log file is worked
// around in place rather than by diverting entries to the root log.
#define MAX_LOG_FILE_ALTERNATIVES                   10

// Firmware version, supplied by the Makefile as "<version>+<githash>[-dirty]".
// Fallback exists only so the tree still compiles outside the normal build.
#ifndef _FW_VERSION
#define _FW_VERSION                                 "unknown"
#endif

// Expected range of the analog microphone DC offset. A healthy path sits near mid-
// scale; a disconnected or shorted microphone sits at a rail. Used to log a pass/fail
// verdict so the microphone path can be verified after assembly.
//
// PROVISIONAL: chosen so that the one reference deployment available (which measured
// 32320) passes comfortably. Narrow this once several known-good units have been
// measured -- a tolerance this wide will pass some genuinely marginal paths.
#define MIC_DC_OFFSET_NOMINAL                       32768
#define MIC_DC_OFFSET_TOLERANCE                     4000

// Peak excursion at or below which a recording window is reported as silent. Distinct
// from a constant output, which is unambiguously a broken signal path. This is only
// reported, never acted upon: a sealed enclosure in a quiet place is legitimately
// near-silent, so treating silence as a fault would produce false alarms.
#define AUDIO_HEALTH_SILENCE_FLOOR                  4

// Number of samples examined at startup to verify a digital microphone is producing a
// varying signal. The analog path has its DC-offset calibration to lean on; the
// digital path had no startup check at all.
#define MIC_DIGITAL_CHECK_SAMPLES                   4096

// Subsampling stride for the RMS accumulation only, which is the sole part of the
// health calculation carrying a multiply. 1 means every sample.
//
// Measured cost at 1 is about 0.4% of the CPU even at 48 kHz, so this exists as a
// tuning knob rather than a necessity. If it is ever raised, note that the samples
// examined are spread across each buffer with a ROTATING START OFFSET: taking the
// first N samples of every buffer instead would be a periodic sample that aliases
// against any periodic signal content, and would systematically mis-measure a tone.
//
// Minimum, maximum, and constant-output detection always examine EVERY sample. They
// cost two comparisons, and subsampling them would be unsound: a peak is by definition
// a rare event, and a subset that happened to be flat would report a working
// microphone as a dead one.
#define AUDIO_HEALTH_RMS_STRIDE                     1


// Self-Test Definitions -----------------------------------------------------------------------------------------------

// Run once per magnetic activation, before any deployment recording begins. Verifies
// the failures that are invisible once a unit is sealed and in the field.
#define SELF_TEST_RESULTS_FILE_NAME                 "_a3em.test.results"
#define SELF_TEST_CLIP_FILE_NAME                    "_a3em.test.wav"
#define SELF_TEST_STORAGE_FILE_NAME                 "_a3em.test.tmp"

// Length of the microphone window. Long enough to tap the enclosure several times and
// watch the LED respond, short enough not to delay every activation noticeably.
#define SELF_TEST_AUDIO_SECONDS                     20
#define SELF_TEST_AUDIO_SAMPLE_RATE_HZ              16000

// Peak sample above which the green LED lights during the live level monitor. Set well
// clear of the noise floor so an untouched unit sits dark and a tap is unmistakable.
#define SELF_TEST_LIVE_LEVEL_THRESHOLD              600

// Round-tripped through the filesystem to prove the card is seated and writable.
#define SELF_TEST_STORAGE_BYTES                     4096

// A stationary device reads one g in some direction whatever its orientation.
#define SELF_TEST_IMU_MIN_MG                        700.0f
#define SELF_TEST_IMU_MAX_MG                        1300.0f

#define SELF_TEST_BATTERY_MIN_MV                    2500
#define SELF_TEST_BATTERY_MAX_MV                    5000
#define SELF_TEST_TEMPERATURE_MIN_C                 (-40.0f)
#define SELF_TEST_TEMPERATURE_MAX_C                 85.0f

#define SELF_TEST_PASS_INDICATION_US                3000000
#define SELF_TEST_FAIL_INDICATION_REPEATS           3

#ifndef MIN
#define MIN(a, b)                                   (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)                                   (((a) < (b)) ? (b) : (a))
#endif

#ifdef AM_DEBUG_PRINTF
extern void vAssertCalled(const char * const pcFileName, unsigned long ulLine);
#define configASSERT0( x ) if( ( x ) != 0 ) vAssertCalled( __FILE__, __LINE__ )
#define configASSERT1( x ) if( ( x ) != 1 ) vAssertCalled( __FILE__, __LINE__ )
#else
#define configASSERT0( x ) x
#define configASSERT1( x ) x
#endif


// Interrupt Priorities ------------------------------------------------------------------------------------------------

#define COMPARATOR_THRESHOLD_INTERRUPT_PRIORITY         (AM_IRQ_PRIORITY_DEFAULT)
#define MAGNET_SENSOR_INTERRUPT_PRIORITY                (AM_IRQ_PRIORITY_DEFAULT)
#define MAGNET_VALIDATION_TIMER_INTERRUPT_PRIORITY      (AM_IRQ_PRIORITY_DEFAULT)
#define AUDIO_ADC_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT - 1)
#define IMU_DATA_INTERRUPT_PRIORITY                     (AM_IRQ_PRIORITY_DEFAULT - 2)
#define STORAGE_INTERRUPT_PRIORITY                      (AM_IRQ_PRIORITY_DEFAULT - 2)
#define RTC_ALARM_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT)
#define BATT_ADC_INTERRUPT_PRIORITY                     (AM_IRQ_PRIORITY_DEFAULT - 1)
#define AUDIO_TIMER_INTERRUPT_PRIORITY                  (AM_IRQ_PRIORITY_DEFAULT + 1)
#define EXT_HW_INTERRUPT_PRIORITY                       (AM_IRQ_PRIORITY_DEFAULT)


// Audio Sampling Definitions ------------------------------------------------------------------------------------------

#define AUDIO_NUM_CHANNELS                              1
#define AUDIO_MIC_BIAS_VOLTAGE                          0.0f
#define AUDIO_DEFAULT_SAMPLING_RATE_HZ                  16000
#define AUDIO_BUFFER_MAX_SAMPLES                        96000
#define AUDIO_BUFFER_MAX_NUM_SECONDS                    (AUDIO_BUFFER_MAX_SAMPLES / 8000)
#define AUDIO_DEFAULT_CLIP_LENGTH_SECONDS               10

#define AUDIO_PRE_DEPLOYMENT_CLIP_LENGTH_SECONDS        60
#define AUDIO_PRE_DEPLOYMENT_SAMPLE_RATE_HZ             16000


// Audio Encoding Definitions ------------------------------------------------------------------------------------------

#define OPUS_REQUIRED_SAMPLE_RATE_HZ                    48000
#define OPUS_MAX_ENCODING_BITRATE                       128000
#define OPUS_DEFAULT_ENCODING_BITRATE                   32000
#define OPUS_MS_PER_FRAME                               20

#define OGG_MAX_PAGE_SIZE                               4096
#define OGG_ENCODER_NAME                                "A3EM"
#define OGG_USE_CRC                                     1


// Battery Monitoring Definitions -------------------------------------------------------------------------------------

#define BATTERY_DEFAULT_LOW_LEVEL_MV                    3250


// IMU Sampling Definitions --------------------------------------------------------------------------------------------

#define IMU_DEFAULT_SAMPLING_RATE_HZ                    25
#define IMU_BUFFER_MAX_SAMPLES                          ((AUDIO_BUFFER_MAX_SAMPLES / 80) + 100)


// Magnetic Sensing Definitions ----------------------------------------------------------------------------------------

#define MAGNET_FIELD_DEFAULT_VALIDATION_LENGTH_MS       5000
#define MAGNET_FIELD_TIMER_TICK_RATE_HZ                 (AM_HAL_CLKGEN_FREQ_MAX_HZ / 16)


// AI and Clustering Definitions ---------------------------------------------------------------------------------------

#define MFCC_NUM_COEFFS                     16
#define MFCC_NUM_FBANK_BINS                 16

#define AI_AUDIO_SAMPLE_RATE_HZ             8000
#define AI_INPUT_LENGTH_MS                  1000
#define AI_WINDOW_LENGTH_MS                 30
#define AI_HOP_LENGTH_MS                    15
#define AI_NUM_INPUT_FEATURES               (MFCC_NUM_COEFFS * (1 + ((AI_INPUT_LENGTH_MS - AI_WINDOW_LENGTH_MS) / AI_HOP_LENGTH_MS)))
#define AI_NUM_OUTPUT_FEATURES              16

#define MAX_NUM_CLUSTERS                    64
#define CLUSTERING_BASE_RADIUS              0.5f
#define CLUSTERING_MAX_WEIGHT               5.0f


#endif  // #ifndef __STATIC_CONFIG_HEADER_H__
