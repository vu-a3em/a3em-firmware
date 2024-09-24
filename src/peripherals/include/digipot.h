#ifndef __DIGIPOT_HEADER_H__
#define __DIGIPOT_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void digipot_init(void);
void digipot_deinit(void);
float digipot_get_percent_output(void);
void digipot_set_percent_output(float percent);

#endif  // #ifndef __DIGIPOT_HEADER_H__
