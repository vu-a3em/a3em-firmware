#ifndef __MAGNET_HEADER_H__
#define __MAGNET_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef void (*magnet_sensor_callback_t)(void);
typedef void (*magnetic_field_validation_callback_t)(bool validated);


// Public API Functions ------------------------------------------------------------------------------------------------

void magnet_sensor_init(void);
void magnet_sensor_deinit(void);
void magnet_sensor_enable_for_wakeup(void);
bool magnet_sensor_field_present(void);
void magnet_sensor_register_callback(magnet_sensor_callback_t callback);
void magnet_sensor_verify_field(uint32_t milliseconds, magnetic_field_validation_callback_t callback);

#endif  // #ifndef __MAGNET_HEADER_H__
