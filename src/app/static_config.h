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

#define MRAM_PERSISTENT_STORAGE_ADDRESS             AM_HAL_MRAM_INSTANCE_SIZE

#define DEVICE_ID_LEN                               6
#define MAX_DEVICE_LABEL_LEN                        31
#define MAX_AUDIO_TRIGGER_TIMES                     12
#define MAX_NUM_DEPLOYMENT_PHASES                   6
#define MAX_CFG_FILE_LINE_LENGTH                    80
#define MIN_LOG_DATA_INTERVAL_SECONDS               300
#define NUM_HOURS_PER_AUDIO_DIRECTORY               4
#define NUM_SECONDS_PER_AUDIO_DIRECTORY             (NUM_HOURS_PER_AUDIO_DIRECTORY * 60 * 60)

#define CONFIG_FILE_NAME                            "_a3em.cfg"
#define LOG_FILE_NAME                               "a3em.log"
#define IMU_FILE_NAME                               "a3em.imu"

#define MIN(a, b)                                   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                                   (((a) < (b)) ? (b) : (a))

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
#define AUDIO_ADC_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT - 1)
#define IMU_DATA_INTERRUPT_PRIORITY                     (AM_IRQ_PRIORITY_DEFAULT - 2)
#define STORAGE_INTERRUPT_PRIORITY                      (AM_IRQ_PRIORITY_DEFAULT - 2)
#define RTC_ALARM_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT)
#define BATT_ADC_INTERRUPT_PRIORITY                     (AM_IRQ_PRIORITY_DEFAULT - 1)
#define AUDIO_TIMER_INTERRUPT_PRIORITY                  (AM_IRQ_PRIORITY_DEFAULT + 1)


// Audio Sampling Definitions ------------------------------------------------------------------------------------------

#define AUDIO_NUM_CHANNELS                              1
#define AUDIO_MIC_BIAS_VOLTAGE                          0.0f
#define AUDIO_BUFFER_NUM_SAMPLES                        4000
#define AUDIO_DEFAULT_SAMPLING_RATE_HZ                  16000
#define AUDIO_DEFAULT_CLIP_LENGTH_SECONDS               10


// Battery Monitoring Definitions -------------------------------------------------------------------------------------

#define BATTERY_DEFAULT_LOW_LEVEL_MV                    3250


// IMU Sampling Definitions --------------------------------------------------------------------------------------------

#define IMU_DEFAULT_SAMPLING_RATE_HZ                    25
#define IMU_BUFFER_NUM_SAMPLES                          128
#define IMU_DATA_DELIMITER                              0xFFFEFDFC


// Magnetic Sensing Definitions ----------------------------------------------------------------------------------------

#define MAGNET_FIELD_DEFAULT_VALIDATION_LENGTH_MS       5000
#define MAGNET_FIELD_VALIDATION_TIMER_TICK_RATE_HZ      (AM_HAL_CLKGEN_FREQ_MAX_HZ / 16)


// AI and Clustering Definitions ---------------------------------------------------------------------------------------

#define MFCC_NUM_COEFFS                     32
#define MFCC_NUM_FBANK_BINS                 16

#define AI_AUDIO_SAMPLE_RATE_HZ             8000
#define AI_INPUT_LENGTH_MS                  1000
#define AI_WINDOW_LENGTH_MS                 30
#define AI_HOP_LENGTH_MS                    15
#define AI_NUM_INPUT_FEATURES               (MFCC_NUM_COEFFS * (1 + ((AI_INPUT_LENGTH_MS - AI_WINDOW_LENGTH_MS) / AI_HOP_LENGTH_MS)))
#define AI_NUM_OUTPUT_FEATURES              16


#endif  // #ifndef __STATIC_CONFIG_HEADER_H__
