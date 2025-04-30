#ifndef __AI_HEADER_H__
#define __AI_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Public API Functions ------------------------------------------------------------------------------------------------

bool ai_initialize(void);
bool ai_worth_exploring(const int16_t *audio);
bool ai_continue(void);
bool ai_invoke(const int16_t *audio);

#endif  // #ifndef __AI_HEADER_H__
