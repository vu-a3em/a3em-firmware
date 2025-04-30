#ifndef __CLUSTERING_HEADER_H__
#define __CLUSTERING_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void clustering_initialize(void);
bool clustering_invoke(const float *embedding);
const float (*clustering_get_means(void))[AI_NUM_OUTPUT_FEATURES];
const float* clustering_get_weights(void);

#endif  // #ifndef __CLUSTERING_HEADER_H__
