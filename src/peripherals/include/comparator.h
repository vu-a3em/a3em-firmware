#ifndef __COMPARATOR_HEADER_H__
#define __COMPARATOR_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void comparator_init(bool internal_reference, uint32_t internal_threshold_millivolts, float external_threshold_percent, bool compare_to_audadc_input);
void comparator_deinit(void);
void comparator_start(void);
void comparator_stop(void);
void comparator_reset(void);
void comparator_set_threshold(float percent);
bool comparator_triggered(void);

#endif  // #ifndef __COMPARATOR_HEADER_H__
