#ifndef __HENRIK_HEADER_H__
#define __HENRIK_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef struct { uint32_t utc_timestamp; float lat, lon, height; bool new_data; } henrik_data_t;
typedef void (*henrik_data_callback_t)(henrik_data_t new_data);


// Public API Functions ------------------------------------------------------------------------------------------------

void henrik_init(void);
void henrik_deinit(void);
henrik_data_t henrik_get_data(void);
void henrik_register_data_callback(henrik_data_callback_t callback);
void henrik_send_alert(void);

#endif  // #ifndef __HENRIK_HEADER_H__
