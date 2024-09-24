#ifndef __STATIC_CONFIG_HEADER_H__
#define __STATIC_CONFIG_HEADER_H__

// Application Configuration -------------------------------------------------------------------------------------------

#ifdef AM_DEBUG_PRINTF

#define ENABLE_LOGGING  1
#define ENABLE_AUDIO_DL 1

#endif


// Common Header Inclusions --------------------------------------------------------------------------------------------

#include <stdlib.h>
#include <string.h>
#include "am_bsp.h"
#include "am_util.h"
#include "pinout.h"


// Common Application Definitions --------------------------------------------------------------------------------------

#define DEVICE_ID_LEN                               6
#define MAX_DEVICE_LABEL_LEN                        15
#define MAX_AUDIO_TRIGGER_TIMES                     12
#define MAX_NUM_DEPLOYMENT_PHASES                   6
#define MAX_CFG_FILE_LINE_LENGTH                    80
#define CONFIG_FILE_NAME                            "_a3em.cfg"

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
#define AUDIO_ADC_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT)
#define IMU_DATA_INTERRUPT_PRIORITY                     (AM_IRQ_PRIORITY_DEFAULT - 1)
#define RTC_ALARM_INTERRUPT_PRIORITY                    (AM_IRQ_PRIORITY_DEFAULT)
#define BATT_ADC_INTERRUPT_PRIORITY                     (AM_IRQ_PRIORITY_DEFAULT + 1)


// Audio Sampling Definitions ------------------------------------------------------------------------------------------

#define AUDIO_NUM_CHANNELS                              1
#define AUDIO_MIC_BIAS_VOLTAGE                          0.0f
#define AUDIO_BUFFER_NUM_SAMPLES                        16384
#define AUDIO_BUFFER_READ_TRIGGER                       COMPARATOR_THRESHOLD
#define AUDIO_DEFAULT_SAMPLING_RATE_HZ                  20000
#define AUDIO_DEFAULT_CLIP_LENGTH_SECONDS               10


// IMU Sampling Definitions --------------------------------------------------------------------------------------------

#define IMU_DEFAULT_SAMPLING_RATE_HZ                    6


// Magnetic Sensing Definitions ----------------------------------------------------------------------------------------

#define MAGNET_FIELD_DEFAULT_VALIDATION_LENGTH_MS       5000
#define MAGNET_FIELD_VALIDATION_TIMER_TICK_RATE_HZ      (AM_HAL_CLKGEN_FREQ_MAX_HZ / 16)


#endif  // #ifndef __STATIC_CONFIG_HEADER_H__
