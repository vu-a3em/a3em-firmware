#ifndef __PINOUT_HEADER_H__
#define __PINOUT_HEADER_H__

// Hardware Revision ID
#include "revisions.h"
#define REVISION_ID                                 REVISION_A

// Analog Comparator
#define PIN_COMPARATOR                              11  // Unused - comparator connected to AUDADC
#define PIN_COMPARATOR_FN                           AM_HAL_PIN_11_CMPIN1
#define PIN_COMPARATOR_REF                          9
#define PIN_COMPARATOR_REF_FN                       AM_HAL_PIN_9_CMPRF0
#define PIN_COMPARATOR_REF_CFG_FN                   VCOMP_CFG_NSEL_VREFEXT1

// Audio
#define PDM_MODULE                                  0
#define PIN_MICROPHONE_ENABLE                       27
#define PIN_DIGITAL_MIC_CLK                         50
#define PIN_DIGITAL_MIC_DATA                        51
#define PIN_DIGITAL_MIC_PWR                         2

// Battery
#define PIN_BATTERY_VOLTAGE                         15
#define PIN_BATTERY_VOLTAGE_FUNCTION                AM_HAL_PIN_15_ADCSE4
#define PIN_BATTERY_VOLTAGE_ADC_CHANNEL             AM_HAL_ADC_SLOT_CHSEL_SE4
#define VOLTAGE_DIVIDER_LOWER                       330
#define VOLTAGE_DIVIDER_UPPER                       1000

// External Sensor Board
#define EXT_HW_I2C_NUMBER                           0
#define EXT_HW_I2C_ADDRESS                          0x58
#define PIN_EXT_HW_I2C_SCL                          0
#define PIN_EXT_HW_I2C_SDA                          1
#define PIN_EXT_HW_I2C_SCL_FUNCTION                 AM_HAL_PIN_0_SLSCL
#define PIN_EXT_HW_I2C_SDA_FUNCTION                 AM_HAL_PIN_1_SLSDAWIR3
#define PIN_EXT_HW_INTERRUPT                        35

// IMU
#define IMU_I2C_NUMBER                              0
#define IMU_I2C_ADDRESS                             0x19
#define PIN_IMU_I2C_SCL                             5
#define PIN_IMU_I2C_SDA                             6
#define PIN_IMU_I2C_SCL_FUNCTION                    AM_HAL_PIN_5_M0SCL
#define PIN_IMU_I2C_SDA_FUNCTION                    AM_HAL_PIN_6_M0SDAWIR3
#define PIN_IMU_INTERRUPT1                          68
#define PIN_IMU_INTERRUPT2                          64

// LEDs
#define PIN_LED_RED                                 52
#define PIN_LED_GREEN                               53

// Logging
#define PIN_SWO                                     28
#define PIN_SWO_FUNCTION                            AM_HAL_PIN_28_SWO

// Magnet Sensor
#define MAG_DETECT_TIMER_NUMBER                     2
#define PIN_MAG_SENSOR_INP                          66
// TODO: Use these for the next revision
//#define PIN_MAG_SENSOR_INP_RISING                   66
//#define PIN_MAG_SENSOR_INP_FALLING                  51
#define PIN_MAG_SENSOR_INP2                         49
#define PIN_MAG_SENSOR_DIS                          65

// Reference Voltage
#define DIGIPOT_I2C_NUMBER                          3
#define DIGIPOT_I2C_ADDRESS                         0x50
#define PIN_DIGIPOT_I2C_SCL                         31
#define PIN_DIGIPOT_I2C_SDA                         32
#define PIN_DIGIPOT_I2C_SCL_FUNCTION                AM_HAL_PIN_31_M3SCL
#define PIN_DIGIPOT_I2C_SDA_FUNCTION                AM_HAL_PIN_32_M3SDAWIR3

// Storage
#define PIN_SD_CARD_DETECT                          40
#define PIN_SD_CARD_ENABLE                          79
#define PIN_SD_CARD_CLK                             88
#define PIN_SD_CARD_CMD                             83
#define PIN_SD_CARD_DAT0                            84
#define PIN_SD_CARD_DAT1                            85
#define PIN_SD_CARD_DAT2                            86
#define PIN_SD_CARD_DAT3                            87

// Timers
#define TIMER_NUMBER_AUDIO_PROCESSING               0
#define TIMER_AUDIO_PROCESSING_CLOCK                AM_HAL_TIMER_CLOCK_XT_DIV128
#define TIMER_AUDIO_PROCESSING_TICK_RATE            (32768 / 128)

// VHF
#define PIN_VHF_ENABLE                              34

#endif  // #ifndef __PINOUT_HEADER_H__
