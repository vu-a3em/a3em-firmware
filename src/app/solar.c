// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <math.h>
#include "solar.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

#define SOLAR_ZENITH_SUNRISE_DEG   90.833
#define SOLAR_ZENITH_CIVIL_DEG     96.0
#define SOLAR_PI                   3.14159265358979323846
#define SECONDS_PER_DAY            86400

static const double anchor_zenith[SOLAR_NUM_ANCHORS] = { SOLAR_ZENITH_CIVIL_DEG, SOLAR_ZENITH_SUNRISE_DEG, SOLAR_ZENITH_SUNRISE_DEG, SOLAR_ZENITH_CIVIL_DEG };
static const bool anchor_rising[SOLAR_NUM_ANCHORS] = { true, true, false, false };
static const char *anchor_names[SOLAR_NUM_ANCHORS] = { "DAWN", "SUNRISE", "SUNSET", "DUSK" };


// Private Helper Functions --------------------------------------------------------------------------------------------

static double to_radians(double degrees) { return degrees * SOLAR_PI / 180.0; }
static double to_degrees(double radians) { return radians * 180.0 / SOLAR_PI; }

static int64_t floor_div(int64_t numerator, int64_t denominator)
{
   const int64_t quotient = numerator / denominator;
   return ((numerator % denominator) && ((numerator < 0) != (denominator < 0))) ? (quotient - 1) : quotient;
}

static double julian_century(double julian_day) { return (julian_day - 2451545.0) / 36525.0; }

static double geom_mean_long_sun_deg(double t)
{
   const double longitude = fmod(280.46646 + t * (36000.76983 + t * 0.0003032), 360.0);
   return (longitude < 0.0) ? (longitude + 360.0) : longitude;
}

static double geom_mean_anomaly_sun_deg(double t)
{
   return 357.52911 + t * (35999.05029 - 0.0001537 * t);
}

static double eccentricity_earth_orbit(double t)
{
   return 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
}

static double sun_eq_of_centre_deg(double t)
{
   const double m = to_radians(geom_mean_anomaly_sun_deg(t));
   return sin(m) * (1.914602 - t * (0.004817 + 0.000014 * t)) + sin(2.0 * m) * (0.019993 - 0.000101 * t) + sin(3.0 * m) * 0.000289;
}

static double apparent_longitude_deg(double t)
{
   const double true_longitude = geom_mean_long_sun_deg(t) + sun_eq_of_centre_deg(t);
   return true_longitude - 0.00569 - 0.00478 * sin(to_radians(125.04 - 1934.136 * t));
}

static double obliquity_corrected_deg(double t)
{
   const double arc_seconds = 21.448 - t * (46.815 + t * (0.00059 - t * 0.001813));
   const double mean_obliquity = 23.0 + (26.0 + arc_seconds / 60.0) / 60.0;
   return mean_obliquity + 0.00256 * cos(to_radians(125.04 - 1934.136 * t));
}

static double solar_declination_deg(double t)
{
   return to_degrees(asin(sin(to_radians(obliquity_corrected_deg(t))) * sin(to_radians(apparent_longitude_deg(t)))));
}

static double equation_of_time_minutes(double t)
{
   // Compute the minutes by which true solar time runs ahead of mean solar time
   const double epsilon = to_radians(obliquity_corrected_deg(t));
   const double mean_longitude = to_radians(geom_mean_long_sun_deg(t));
   const double mean_anomaly = to_radians(geom_mean_anomaly_sun_deg(t));
   const double eccentricity = eccentricity_earth_orbit(t);
   const double half_tangent = tan(epsilon / 2.0);
   const double y = half_tangent * half_tangent;
   return 4.0 * to_degrees((y * sin(2.0 * mean_longitude)) -
                           (2.0 * eccentricity * sin(mean_anomaly)) +
                           (4.0 * eccentricity * y * sin(mean_anomaly) * cos(2.0 * mean_longitude)) -
                           (0.5 * y * y * sin(4.0 * mean_longitude)) -
                           (1.25 * eccentricity * eccentricity * sin(2.0 * mean_anomaly)));
}

static bool hour_angle_deg(double latitude_deg, double declination_deg, double zenith_deg, double *result)
{
   // Compute the hour angle at which the sun reaches zenith_deg
   const double latitude = to_radians(latitude_deg);
   const double declination = to_radians(declination_deg);
   const double cos_hour_angle = cos(to_radians(zenith_deg)) / (cos(latitude) * cos(declination)) - tan(latitude) * tan(declination);
   if ((cos_hour_angle > 1.0) || (cos_hour_angle < -1.0))
      return false;
   *result = to_degrees(acos(cos_hour_angle));
   return true;
}

static bool event_minutes_utc(double julian_day, double latitude_deg, double longitude_deg, double zenith_deg, bool rising, double *result)
{
   // Compute the minutes after 00:00 UTC at which the event occurs
   double minutes = 720.0;
   for (int32_t pass = 0; pass < 2; ++pass)
   {
      const double t = julian_century(julian_day + minutes / 1440.0);
      double hour_angle;
      if (!hour_angle_deg(latitude_deg, solar_declination_deg(t), zenith_deg, &hour_angle))
         return false;
      minutes = 720.0 - 4.0 * (longitude_deg + (rising ? hour_angle : -hour_angle)) - equation_of_time_minutes(t);
   }
   *result = minutes;
   return true;
}


// Public API Functions ------------------------------------------------------------------------------------------------

bool solar_position_valid(double latitude_deg, double longitude_deg)
{
   return !isnan(latitude_deg) && !isnan(longitude_deg) && (latitude_deg >= -90.0) && (latitude_deg <= 90.0) && (longitude_deg >= -180.0) && (longitude_deg <= 180.0);
}

void solar_compute(double latitude_deg, double longitude_deg, uint32_t utc_timestamp, int32_t utc_offset_seconds, solar_day_t *result)
{
   // Validate the input and initialize the result structure
   memset(result, 0, sizeof(solar_day_t));
   if (!solar_position_valid(latitude_deg, longitude_deg))
      return;

   // Compute the local day this instant falls in
   const int64_t local_timestamp = (int64_t)utc_timestamp + (int64_t)utc_offset_seconds;
   const int64_t local_day = floor_div(local_timestamp, SECONDS_PER_DAY);
   const int64_t local_midnight_utc = local_day * SECONDS_PER_DAY - (int64_t)utc_offset_seconds;

   // NOAA's series is anchored to UTC midnight, so work from the UTC day containing local noon
   const int64_t utc_day = floor_div(local_midnight_utc + (SECONDS_PER_DAY / 2), SECONDS_PER_DAY);
   const double julian_day = (double)utc_day + 2440587.5;
   for (int32_t anchor = 0; anchor < SOLAR_NUM_ANCHORS; ++anchor)
   {
      double minutes;
      if (!event_minutes_utc(julian_day, latitude_deg, longitude_deg, anchor_zenith[anchor], anchor_rising[anchor], &minutes))
         continue;

      const int64_t event_utc = utc_day * SECONDS_PER_DAY + (int64_t)floor(minutes * 60.0 + 0.5);
      int64_t seconds_of_day = event_utc + (int64_t)utc_offset_seconds - local_day * SECONDS_PER_DAY;

      // Fold onto the local day
      seconds_of_day %= SECONDS_PER_DAY;
      if (seconds_of_day < 0)
         seconds_of_day += SECONDS_PER_DAY;

      result->seconds_of_day[anchor] = (int32_t)seconds_of_day;
      result->available[anchor] = true;
   }

   // Determine which kind of "no sunrise" this is
   if (!result->available[SOLAR_SUNRISE] && !result->available[SOLAR_SUNSET])
   {
      const double noon_altitude = 90.0 - fabs(latitude_deg - solar_declination_deg(julian_century(julian_day + 0.5)));
      result->polar_day = (noon_altitude > (90.0 - SOLAR_ZENITH_SUNRISE_DEG));
      result->polar_night = !result->polar_day;
   }
}

bool solar_parse_anchor(const char *text, solar_anchor_t *anchor)
{
   for (int32_t index = 0; index < SOLAR_NUM_ANCHORS; ++index)
      if (strcmp(text, anchor_names[index]) == 0)
      {
         *anchor = (solar_anchor_t)index;
         return true;
      }
   return false;
}

const char *solar_anchor_name(solar_anchor_t anchor)
{
   return ((int32_t)anchor < SOLAR_NUM_ANCHORS) ? anchor_names[anchor] : "DAWN";
}
