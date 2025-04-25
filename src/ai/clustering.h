#ifndef __CLUSTERING_HEADER_H__
#define __CLUSTERING_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

void clustering_initialize(void);
void clustering_invoke(float *embeddings);

#endif  // #ifndef __CLUSTERING_HEADER_H__
