#ifndef __NN_HEADER_H__
#define __NN_HEADER_H__

#ifdef __cplusplus
extern "C" {
#endif

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <stdbool.h>


// Public API Functions ------------------------------------------------------------------------------------------------

bool nn_initialize(void);
float* nn_invoke(float *input);

#ifdef __cplusplus
}
#endif

#endif  // #ifndef __NN_HEADER_H__
