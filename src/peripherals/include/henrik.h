#ifndef __HENRIK_HEADER_H__
#define __HENRIK_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void henrik_init(void);
void henrik_deinit(void);
uint32_t henrik_get_gps_timestamp(void);
void henrik_get_gps_location(float *lat, float *lon, float *height);
void henrik_send_alert(void);

#endif  // #ifndef __HENRIK_HEADER_H__
