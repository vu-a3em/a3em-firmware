// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <math.h>
#include "clustering.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static float means[MAX_NUM_CLUSTERS][AI_NUM_OUTPUT_FEATURES], weights[MAX_NUM_CLUSTERS];
static const float base_radius_sqr = CLUSTERING_BASE_RADIUS * CLUSTERING_BASE_RADIUS;


// Private Helper Functions --------------------------------------------------------------------------------------------

static float l2_norm_sqr(const float *a, const float *b)
{
   float res = 0.0f;
   for (int i = 0; i < AI_NUM_OUTPUT_FEATURES; ++i)
   {
      const float c = a[i] - b[i];
      res += c * c;
   }
   return res;
}


// Public API Functions ------------------------------------------------------------------------------------------------

void clustering_initialize(void)
{
   // Initialize all clustering data variables
   memset(means, 0, sizeof(means));
   memset(weights, 0, sizeof(weights));
}

bool clustering_invoke(const float *embedding)
{
   // Initialize a new cluster for the passed-in embedding
   float center[AI_NUM_OUTPUT_FEATURES], center_weight = 1.0f;
   memcpy(center, embedding, sizeof(center));

   // Determine which existing clusters contain this embedding
   int matching_clusters = MAX_NUM_CLUSTERS;
   for (int i = MAX_NUM_CLUSTERS - 1; i >= 0; --i)
   {
      // Test if the embedding lies within the existing cluster
      const float weight = weights[i], *mean = means[i];
      if (l2_norm_sqr(mean, embedding) <= (weight * base_radius_sqr))
      {
         // Combine the new embedding with the existing cluster
         for (int j = 0; j < AI_NUM_OUTPUT_FEATURES; ++j)
            center[j] += mean[j] * weight;
         center_weight += weight;
      }
      else if (--matching_clusters != i)
      {
         // Move this cluster forward within the queue to discard matched clusters
         weights[matching_clusters] = weight;
         memcpy(means[matching_clusters], mean, sizeof(means[0]));
      }
   }

   // If no clusters contained the embedding, force the oldest one out
   if (!matching_clusters)
      matching_clusters = 1;

   // Move all remaining clusters up one slot and zero any excess matching clusters
   memmove(&means[matching_clusters - 1], &means[matching_clusters], (MAX_NUM_CLUSTERS - matching_clusters) * sizeof(means[0]));
   memmove(&weights[matching_clusters - 1], &weights[matching_clusters], (MAX_NUM_CLUSTERS - matching_clusters) * sizeof(weights[0]));
   memset(means[0], 0, (matching_clusters - 1) * sizeof(means[0]));
   memset(weights, 0, (matching_clusters - 1) * sizeof(weights[0]));

   // Create a new cluster at the end of the queue for the passed-in embedding
   const float center_weight_inv = 1.0f / center_weight;
   for (int i = 0; i < AI_NUM_OUTPUT_FEATURES; ++i)
      means[MAX_NUM_CLUSTERS - 1][i] = center[i] * center_weight_inv;
   weights[MAX_NUM_CLUSTERS - 1] = fminf(center_weight, CLUSTERING_MAX_WEIGHT);

   // Audio should be stored only if the embedding was not found in an existing cluster
   return center_weight == 1.0f;
}

const float (*clustering_get_means(void))[AI_NUM_OUTPUT_FEATURES]
{
   return means;
}

const float* clustering_get_weights(void)
{
   return weights;
}
