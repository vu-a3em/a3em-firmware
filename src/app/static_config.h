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

#define _STRINGIFY_INNER(x)                         #x
#define _STRINGIFY(x)                               _STRINGIFY_INNER(x)

#define DEVICE_ID_LEN                               6
#define MAX_DEVICE_LABEL_LEN                        31
#define MAX_AUDIO_TRIGGER_TIMES                     12
#define MAX_NUM_DEPLOYMENT_PHASES                   6
#define MAX_CFG_FILE_LINE_LENGTH                    80
#define MIN_LOG_DATA_INTERVAL_SECONDS               300
#define NUM_HOURS_PER_AUDIO_DIRECTORY               4
#define AUDIO_BUFFER_MAX_SIZE                       65536
#define NUM_SECONDS_PER_AUDIO_DIRECTORY             (NUM_HOURS_PER_AUDIO_DIRECTORY * 60 * 60)
#define SD_CARD_ALLOCATION_UNIT_BYTES               4096
#define WAV_STAGING_BUFFER_SIZE                     (256 * 1024)

#define CONFIG_FILE_NAME                            "_a3em.cfg"
#define DEVICE_INFO_FILE_NAME                       "_a3em.dev"
#define LOG_FILE_NAME                               "a3em.log"
#define BOOT_LOG_FILE_NAME                          "boot.log"


// Diagnostics Definitions ---------------------------------------------------------------------------------------------

#define ENABLE_CACHE_MONITOR                        1


// Watchdog Definitions ------------------------------------------------------------------------------------------------

#define WATCHDOG_TIMEOUT_SECONDS                    120

#define GPS_TIME_MAX_ATTEMPTS                       60
#define LED_PATTERN_MAX_WAIT_WAKEUPS                200


// Failure Handling and Diagnostics Definitions ------------------------------------------------------------------------

// Consecutive-failure budgets before escalating
#define STORAGE_MAX_REOPEN_ATTEMPTS                 3
#define STORAGE_MAX_REMOUNT_ATTEMPTS                2
#define STORAGE_MAX_CONSECUTIVE_FAILURES            8

// Bounds on the spin loops that wait for peripherals to reach a requested state
#define PERIPHERAL_MAX_WAIT_ATTEMPTS                1000
#define PERIPHERAL_WAIT_INTERVAL_US                 100

// Fixed-size boot log record layout. Records are written at a fixed offset inside a pre-allocated
// file and each carries its own sequence number and checksum, so a torn write during power loss
// can damage at most one record and every surviving record remains independently parseable.
// The first BOOT_LOG_RESERVED_RECORDS slots are written once and never overwritten
#define BOOT_LOG_RECORD_LEN                         64
#define BOOT_LOG_NUM_RECORDS                        4096
#define BOOT_LOG_RESERVED_RECORDS                   64
#define BOOT_LOG_FILE_SIZE                          (BOOT_LOG_RECORD_LEN * BOOT_LOG_NUM_RECORDS)
#define BOOT_LOG_MIN_RECORD_LEN                     56

#ifndef __cplusplus
_Static_assert((4096 % BOOT_LOG_RECORD_LEN) == 0, "BOOT_LOG_RECORD_LEN must divide 4096 so that no record straddles a sector boundary");
_Static_assert(BOOT_LOG_RECORD_LEN >= BOOT_LOG_MIN_RECORD_LEN, "BOOT_LOG_RECORD_LEN is too small to hold a record plus its checksum and newline");
_Static_assert(BOOT_LOG_NUM_RECORDS > 0, "BOOT_LOG_NUM_RECORDS must be positive");
_Static_assert(BOOT_LOG_RESERVED_RECORDS < BOOT_LOG_NUM_RECORDS, "BOOT_LOG_RESERVED_RECORDS must leave at least one record for the wrapping region");
#endif

// Software reset reason codes stored in MCUCTRL->SCRATCH0 across a reset
#define RESET_REASON_NONE                           0x00
#define RESET_REASON_UNKNOWN                        0x01
#define RESET_REASON_HARD_FAULT                     0x02
#define RESET_REASON_PHASE_COMPLETE                 0x03
#define RESET_REASON_AUDIO_ERROR                    0x04
#define RESET_REASON_STORAGE_FAILURE                0x05
#define RESET_REASON_RTC_STOPPED                    0x06
#define RESET_REASON_BATTERY_LOW                    0x07
#define RESET_REASON_MISSING_CONFIG                 0x08
#define RESET_REASON_MAGNET_DEACTIVATED             0x09
#define RESET_REASON_ACTIVATED                      0x0A
#define RESET_REASON_PERIPHERAL_TIMEOUT             0x0B

#ifndef MIN
#define MIN(a, b)                                   (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)                                   (((a) < (b)) ? (b) : (a))
#endif

// Records a HAL call that returned an unexpected status. Deliberately non-fatal: a failed pin
// configuration should be visible in the logs, not a reason to brick a deployed device. The first
// failure and a running count are surfaced through system_get_hal_failure_count() and logged with
// the periodic device details.
extern void system_note_hal_failure(const char *file, uint32_t line, uint32_t status);

#ifdef AM_DEBUG_PRINTF
extern void vAssertCalled(const char * const pcFileName, unsigned long ulLine);
#define configASSERT0( x ) if( ( x ) != 0 ) vAssertCalled( __FILE__, __LINE__ )
#define configASSERT1( x ) if( ( x ) != 1 ) vAssertCalled( __FILE__, __LINE__ )
#else
#define configASSERT0( x ) do { const uint32_t _hal_status = (uint32_t)( x ); \
   if (_hal_status != 0) system_note_hal_failure(__FILE__, __LINE__, _hal_status); } while (0)
#define configASSERT1( x ) do { const uint32_t _hal_status = (uint32_t)( x ); \
   if (_hal_status != 1) system_note_hal_failure(__FILE__, __LINE__, _hal_status); } while (0)
#endif



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


// Interrupt Priorities ------------------------------------------------------------------------------------------------

// NOTE: The magnet sensor input and the IMU interrupt line are serviced by a single NVIC vector
#define GPIO_INTERRUPT_PRIORITY                         (AM_IRQ_PRIORITY_DEFAULT)

#define COMPARATOR_THRESHOLD_INTERRUPT_PRIORITY         (AM_IRQ_PRIORITY_DEFAULT)
#define MAGNET_SENSOR_INTERRUPT_PRIORITY                (GPIO_INTERRUPT_PRIORITY)
#define MAGNET_VALIDATION_TIMER_INTERRUPT_PRIORITY      (AM_IRQ_PRIORITY_DEFAULT)
#define AUDIO_ADC_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT - 1)
#define IMU_DATA_INTERRUPT_PRIORITY                     (GPIO_INTERRUPT_PRIORITY)
#define STORAGE_INTERRUPT_PRIORITY                      (AM_IRQ_PRIORITY_DEFAULT - 2)
#define RTC_ALARM_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT)
#define BATT_ADC_INTERRUPT_PRIORITY                     (AM_IRQ_PRIORITY_DEFAULT - 1)
#define AUDIO_TIMER_INTERRUPT_PRIORITY                  (AM_IRQ_PRIORITY_DEFAULT + 1)
#define AUDIO_DMA_TIMER_INTERRUPT_PRIORITY              (AM_IRQ_PRIORITY_DEFAULT - 1)
#define STATUS_LED_TIMER_INTERRUPT_PRIORITY             (AM_IRQ_PRIORITY_DEFAULT + 1)
#define EXT_HW_INTERRUPT_PRIORITY                       (AM_IRQ_PRIORITY_DEFAULT)


// Audio Sampling Definitions ------------------------------------------------------------------------------------------

// Nominal DC bias of a healthy analog microphone path, in raw AUDADC counts. A working
// path settles near mid-scale; a disconnected or shorted microphone sits at a rail.
//
// PROVISIONAL: chosen so that the one reference deployment available (which measured
// 32320) passes comfortably. Narrow this once several known-good units have been
// measured -- a tolerance this wide will pass some genuinely marginal paths.
#define MIC_DC_OFFSET_NOMINAL                           32768
#define MIC_DC_OFFSET_TOLERANCE                         4000

// Microphone health thresholds. The silence floor is in int16 LSBs: a real microphone in
// a genuinely quiet place still shows a few, so anything at or below this is a path that
// is not producing signal rather than a quiet night.
#define AUDIO_HEALTH_SILENCE_FLOOR                      4
// Every sample participates in min/max; only the sum and sum-of-squares are subsampled.
#define AUDIO_HEALTH_RMS_STRIDE                         1

#define AUDIO_NUM_CHANNELS                              1
#define AUDIO_MIC_BIAS_VOLTAGE                          0.0f
#define AUDIO_DEFAULT_SAMPLING_RATE_HZ                  16000

#define AUDIO_BUFFER_MAX_SAMPLES                        48000
#define AUDIO_BUFFER_MAX_NUM_SECONDS                    (AUDIO_BUFFER_MAX_SAMPLES / 8000)
#define AUDIO_DEFAULT_CLIP_LENGTH_SECONDS               10

#define AUDIO_MIN_SAMPLING_RATE_HZ                      4000
#define AUDIO_MAX_SAMPLING_RATE_HZ                      AUDIO_BUFFER_MAX_SAMPLES
#define AUDIO_MIN_CLIP_LENGTH_SECONDS                   1
#define AUDIO_MAX_CLIP_LENGTH_SECONDS                   3600

#define AUDIO_DMA_BACKSTOP_MARGIN_MS                    8
#define AUDIO_DMA_DCMP_CONFIDENCE                       16

#define DC_OFFSET_RECAL_DELTA_C                         10.0f
#define DC_OFFSET_TRACK_MAX_STEP                        2

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

#define IMU_FIFO_WATERMARK                              96
#define IMU_FIFO_MAX_LEVEL                              128
#define IMU_FIFO_BURST_ENTRIES                          16

#define IMU_MOTION_FULL_SCALE_MG                        2000.0f
#define IMU_MOTION_THRESHOLD_STEPS                      255
#define IMU_MOTION_THRESHOLD_STEP_MG                    (IMU_MOTION_FULL_SCALE_MG / IMU_MOTION_THRESHOLD_STEPS)


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
