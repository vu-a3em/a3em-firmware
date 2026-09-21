#ifndef __SOLAR_HEADER_H__
#define __SOLAR_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "static_config.h"


// Solar Type Definitions ----------------------------------------------------------------------------------------------

typedef enum { SOLAR_DAWN = 0, SOLAR_SUNRISE = 1, SOLAR_SUNSET = 2, SOLAR_DUSK = 3, SOLAR_NUM_ANCHORS = 4 } solar_anchor_t;

typedef struct {
   bool available[SOLAR_NUM_ANCHORS];
   int32_t seconds_of_day[SOLAR_NUM_ANCHORS];
   bool polar_day, polar_night;
} solar_day_t;


// Public API Functions ------------------------------------------------------------------------------------------------

bool solar_position_valid(double latitude_deg, double longitude_deg);
void solar_compute(double latitude_deg, double longitude_deg, uint32_t utc_timestamp, int32_t utc_offset_seconds, solar_day_t *result);
bool solar_parse_anchor(const char *text, solar_anchor_t *anchor);
const char *solar_anchor_name(solar_anchor_t anchor);

#endif  // #ifndef __SOLAR_HEADER_H__
