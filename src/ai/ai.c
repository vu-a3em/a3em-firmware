// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "ai.h"
#include "clustering.h"
#include "mfcc.h"
#include "nn.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static float mfccs[AI_NUM_INPUT_FEATURES];


// Public API Functions ------------------------------------------------------------------------------------------------

bool ai_initialize(void)
{
   // Initialize all necessary AI components
   mfcc_initialize();
   clustering_initialize();
   return nn_initialize();
}

bool ai_worth_exploring(const int16_t *audio)
{
   // TODO: Check whether the audio clip is worth exploring further
   mfcc_compute(audio, mfccs);
   return true;
}

void ai_continue(void)
{
   // Simply invoke the neural network and carry out clustering
   // under the assumption that ai_worth_exploring was called first
   clustering_invoke(nn_invoke(mfccs));
}

void ai_invoke(const int16_t *audio)
{
   // Compute MFCC features, invoke the neural network, and carry out clustering
   mfcc_compute(audio, mfccs);
   clustering_invoke(nn_invoke(mfccs));
}
