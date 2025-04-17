#ifndef __AI_HEADER_H__
#define __AI_HEADER_H__

#ifdef __cplusplus
extern "C" {
#endif

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <stdbool.h>


// Public API Functions ------------------------------------------------------------------------------------------------

bool ai_initialize(void);
float* ai_invoke(float *input);

#ifdef __cplusplus
}
#endif

#endif  // #ifndef __AI_HEADER_H__
