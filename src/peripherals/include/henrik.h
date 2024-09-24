#ifndef __HENRIK_HEADER_H__
#define __HENRIK_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void henrik_init(void);
void henrik_deinit(void);
void henrik_get_gps_reading(void);
void henrik_send_alert(void);

#endif  // #ifndef __HENRIK_HEADER_H__
