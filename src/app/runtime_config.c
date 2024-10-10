// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "logging.h"
#include "runtime_config.h"
#include "storage.h"


// Storage Structure --------------------------------------------------------------------------------------------------

typedef struct {
   imu_recording_mode_t imu_recording_mode;
   audio_recording_mode_t audio_recording_mode;
   bool extend_clip_if_continuous_audio;
   float audio_trigger_threshold, imu_trigger_threshold;
   uint32_t max_audio_clips, audio_trigger_interval, audio_sampling_rate;
   uint32_t audio_clip_length, imu_sampling_rate, num_audio_trigger_times;
   time_scale_t max_clips_time_scale, audio_trigger_interval_time_scale;
   start_end_time_t phase_time, audio_trigger_times[MAX_AUDIO_TRIGGER_TIMES];
   uint8_t imu_degrees_of_freedom;
} deployment_phase_t;


// Static Global Variables ---------------------------------------------------------------------------------------------

static char device_label[1 + MAX_DEVICE_LABEL_LEN];
static uint32_t magnetic_field_validation_length_ms;
static uint32_t leds_active_seconds, vhf_start_timestamp;
static bool set_rtc_at_magnet_detect, leds_enabled, device_activated, gps_available, awake_on_magnet;
static deployment_phase_t deployment_phases[MAX_NUM_DEPLOYMENT_PHASES];
static int32_t num_deployment_phases, utc_offset;
static float microphone_amplification_db;
static start_end_time_t deployment_time;


// Private Helper Functions --------------------------------------------------------------------------------------------

static audio_recording_mode_t parse_audio_recording_mode(const char *value)
{
   if (memcmp(value, "AMPLITUDE", sizeof("AMPLITUDE")) == 0)
      return AMPLITUDE;
   else if (memcmp(value, "SCHEDULED", sizeof("SCHEDULED")) == 0)
      return SCHEDULED;
   else if (memcmp(value, "INTERVAL", sizeof("INTERVAL")) == 0)
      return INTERVAL;
   else
      return CONTINUOUS;
}

static imu_recording_mode_t parse_imu_recording_mode(const char *value)
{
   if (memcmp(value, "ACTIVITY", sizeof("ACTIVITY")) == 0)
      return ACTIVITY;
   else
      return AUDIO;
}

static time_scale_t parse_time_scale(const char *value)
{
   if (memcmp(value, "SECONDS", sizeof("SECONDS")) == 0)
      return SECONDS;
   else if (memcmp(value, "HOURS", sizeof("HOURS")) == 0)
      return HOURS;
   else if (memcmp(value, "DAYS", sizeof("DAYS")) == 0)
      return DAYS;
   else
      return MINUTES;
}

static void parse_line(char *line, int32_t line_length)
{
   // Return if not a valid line
   if (line_length < 4)
      return;

   // Locate the configuration key and value
   int32_t i;
   const char *key = line;
   for (i = 0; (i < line_length) && ((key[0] == ' ') || (key[0] == '\t')); ++i)
      key = line + i;
   char *value = line + i;
   for ( ; (i < line_length) && (value[0] != '"'); ++i)
      value = line + i;
   value += 1;
   for (i = i + 1; (i < line_length) && (line[i] != '"'); ++i);
   if (i < line_length)
      line[i] = 0;
   else if (memcmp(key, "[PHASE]", sizeof("[PHASE]")-1) == 0)
   {
      deployment_phases[++num_deployment_phases].phase_time.start_time = deployment_time.start_time;
      deployment_phases[num_deployment_phases].phase_time.end_time = deployment_time.end_time;
   }
   else
      return;

   // Parse the configuration item according to its key
   if (memcmp(key, "DEVICE_LABEL", sizeof("DEVICE_LABEL")-1) == 0)
      strcpy(device_label, value);
   else if (memcmp(key, "DEVICE_UTC_OFFSET", sizeof("DEVICE_UTC_OFFSET")-1) == 0)
      utc_offset = strtol(value, NULL, 10);
   else if (memcmp(key, "SET_RTC_AT_MAGNET_DETECT", sizeof("SET_RTC_AT_MAGNET_DETECT")-1) == 0)
      set_rtc_at_magnet_detect = (memcmp(value, "True", sizeof("True")-1) == 0);
   else if (memcmp(key, "DEPLOYMENT_START_TIME", sizeof("DEPLOYMENT_START_TIME")-1) == 0)
      deployment_time.start_time = strtoul(value, NULL, 10);
   else if (memcmp(key, "DEPLOYMENT_END_TIME", sizeof("DEPLOYMENT_END_TIME")-1) == 0)
      deployment_time.end_time = strtoul(value, NULL, 10);
   else if (memcmp(key, "GPS_AVAILABLE", sizeof("GPS_AVAILABLE")-1) == 0)
      gps_available = (memcmp(value, "True", sizeof("True")) == 0);
   else if (memcmp(key, "AWAKE_ON_MAGNET", sizeof("AWAKE_ON_MAGNET")-1) == 0)
      awake_on_magnet = (memcmp(value, "True", sizeof("True")) == 0);
   else if (memcmp(key, "LEDS_ENABLED", sizeof("LEDS_ENABLED")-1) == 0)
      leds_enabled = (memcmp(value, "True", sizeof("True")) == 0);
   else if (memcmp(key, "LEDS_ACTIVE_SECONDS", sizeof("LEDS_ACTIVE_SECONDS")-1) == 0)
      leds_active_seconds = strtoul(value, NULL, 10);
   else if (memcmp(key, "MIC_AMPLIFICATION", sizeof("MIC_AMPLIFICATION")-1) == 0)
      microphone_amplification_db = strtof(value, NULL);
   else if (memcmp(key, "MAGNET_FIELD_VALIDATION_MS", sizeof("MAGNET_FIELD_VALIDATION_MS")-1) == 0)
      magnetic_field_validation_length_ms = strtoul(value, NULL, 10);
   else if (memcmp(key, "VHF_RADIO_START_TIME", sizeof("VHF_RADIO_START_TIME")-1) == 0)
      vhf_start_timestamp = strtoul(value, NULL, 10);
   else if (memcmp(key, "PHASE_START_TIME", sizeof("PHASE_START_TIME")-1) == 0)
      deployment_phases[num_deployment_phases].phase_time.start_time = strtoul(value, NULL, 10);
   else if (memcmp(key, "PHASE_END_TIME", sizeof("PHASE_END_TIME")-1) == 0)
      deployment_phases[num_deployment_phases].phase_time.end_time = strtoul(value, NULL, 10);
   else if (memcmp(key, "AUDIO_RECORDING_MODE", sizeof("AUDIO_RECORDING_MODE")-1) == 0)
      deployment_phases[num_deployment_phases].audio_recording_mode = parse_audio_recording_mode(value);
   else if (memcmp(key, "AUDIO_EXTEND_CLIP", sizeof("AUDIO_EXTEND_CLIP")-1) == 0)
      deployment_phases[num_deployment_phases].extend_clip_if_continuous_audio = (memcmp(value, "True", sizeof("True")-1) == 0);
   else if (memcmp(key, "AUDIO_MAX_CLIPS_NUMBER", sizeof("AUDIO_MAX_CLIPS_NUMBER")-1) == 0)
      deployment_phases[num_deployment_phases].max_audio_clips = strtoul(value, NULL, 10);
   else if (memcmp(key, "AUDIO_MAX_CLIPS_TIME_SCALE", sizeof("AUDIO_MAX_CLIPS_TIME_SCALE")-1) == 0)
      deployment_phases[num_deployment_phases].max_clips_time_scale = parse_time_scale(value);
   else if (memcmp(key, "AUDIO_TRIGGER_THRESHOLD", sizeof("AUDIO_TRIGGER_THRESHOLD")-1) == 0)
      deployment_phases[num_deployment_phases].audio_trigger_threshold = strtof(value, NULL);
   else if (memcmp(key, "AUDIO_TRIGGER_INTERVAL_TIME_SCALE", sizeof("AUDIO_TRIGGER_INTERVAL_TIME_SCALE")-1) == 0)
      deployment_phases[num_deployment_phases].audio_trigger_interval_time_scale = parse_time_scale(value);
   else if (memcmp(key, "AUDIO_TRIGGER_INTERVAL", sizeof("AUDIO_TRIGGER_INTERVAL")-1) == 0)
      deployment_phases[num_deployment_phases].audio_trigger_interval = strtoul(value, NULL, 10);
   else if (memcmp(key, "AUDIO_TRIGGER_SCHEDULE", sizeof("AUDIO_TRIGGER_SCHEDULE")-1) == 0)
   {
      char *end_time = value;
      while (*end_time != '-')
         end_time += 1;
      *end_time = 0;
      end_time += 1;
      deployment_phases[num_deployment_phases].audio_trigger_times[deployment_phases[num_deployment_phases].num_audio_trigger_times].start_time = strtoul(value, NULL, 10);
      deployment_phases[num_deployment_phases].audio_trigger_times[deployment_phases[num_deployment_phases].num_audio_trigger_times++].end_time = strtoul(end_time, NULL, 10);
   }
   else if (memcmp(key, "AUDIO_SAMPLING_RATE_HZ", sizeof("AUDIO_SAMPLING_RATE_HZ")-1) == 0)
      deployment_phases[num_deployment_phases].audio_sampling_rate = strtoul(value, NULL, 10);
   else if (memcmp(key, "AUDIO_CLIP_LENGTH_SECONDS", sizeof("AUDIO_CLIP_LENGTH_SECONDS")-1) == 0)
      deployment_phases[num_deployment_phases].audio_clip_length = strtoul(value, NULL, 10);
   else if (memcmp(key, "IMU_RECORDING_MODE", sizeof("IMU_RECORDING_MODE")-1) == 0)
      deployment_phases[num_deployment_phases].imu_recording_mode = parse_imu_recording_mode(value);
   else if (memcmp(key, "IMU_DEGREES_OF_FREEDOM", sizeof("IMU_DEGREES_OF_FREEDOM")-1) == 0)
      deployment_phases[num_deployment_phases].imu_degrees_of_freedom = strtoul(value, NULL, 10);
   else if (memcmp(key, "IMU_TRIGGER_THRESHOLD", sizeof("IMU_TRIGGER_THRESHOLD")-1) == 0)
      deployment_phases[num_deployment_phases].imu_trigger_threshold = strtof(value, NULL);
   else if (memcmp(key, "IMU_SAMPLING_RATE_HZ", sizeof("IMU_SAMPLING_RATE_HZ")-1) == 0)
      deployment_phases[num_deployment_phases].imu_sampling_rate = strtoul(value, NULL, 10);
}


// Public API Functions ------------------------------------------------------------------------------------------------

bool fetch_runtime_configuration(void)
{
   // Set default configuration values
   utc_offset = 0;
   awake_on_magnet = true;
   num_deployment_phases = -1;
   microphone_amplification_db = 35.0f;
   leds_active_seconds = vhf_start_timestamp = 0;
   set_rtc_at_magnet_detect = leds_enabled = device_activated = gps_available = false;
   magnetic_field_validation_length_ms = MAGNET_FIELD_DEFAULT_VALIDATION_LENGTH_MS;
   memset(device_label, 0, sizeof(device_label));
   memset(&deployment_time, 0, sizeof(deployment_time));
   for (int i = 0; i < MAX_NUM_DEPLOYMENT_PHASES; ++i)
   {
      deployment_phases[i].audio_clip_length = AUDIO_DEFAULT_CLIP_LENGTH_SECONDS;
      deployment_phases[i].audio_recording_mode = AMPLITUDE;
      deployment_phases[i].audio_sampling_rate = AUDIO_DEFAULT_SAMPLING_RATE_HZ;
      deployment_phases[i].audio_trigger_interval = 1;
      deployment_phases[i].audio_trigger_interval_time_scale = MINUTES;
      deployment_phases[i].audio_trigger_threshold = 0.25;
      memset(&deployment_phases[i].phase_time, 0, sizeof(deployment_phases[i].phase_time));
      memset(deployment_phases[i].audio_trigger_times, 0, sizeof(deployment_phases[i].audio_trigger_times));
      deployment_phases[i].extend_clip_if_continuous_audio = false;
      deployment_phases[i].imu_degrees_of_freedom = 3;
      deployment_phases[i].imu_recording_mode = AUDIO;
      deployment_phases[i].imu_sampling_rate = IMU_DEFAULT_SAMPLING_RATE_HZ;
      deployment_phases[i].imu_trigger_threshold = 0.25;
      deployment_phases[i].max_audio_clips = 0;
      deployment_phases[i].max_clips_time_scale = HOURS;
      deployment_phases[i].num_audio_trigger_times = 0;
   }

   // Open and parse the stored runtime configuration file
   int32_t line_length;
   char line_buffer[MAX_CFG_FILE_LINE_LENGTH];
   bool success = storage_open(CONFIG_FILE_NAME, false);
   while (success && (line_length = storage_read_line(line_buffer, sizeof(line_buffer))) >= 0)
      parse_line(line_buffer, line_length);
   storage_close();

   // Check whether the device activation file exists
   device_activated = storage_open(ACTIVATION_FILE_NAME, false);
   if (device_activated)
      storage_close();

   // Return whether configuration parsing was successful
   ++num_deployment_phases;
   return success;
}

void config_get_device_label(char *label, uint32_t max_size)
{
   memset(label, 0, max_size);
   for (uint32_t i = 0; (i < max_size) && (i <= MAX_DEVICE_LABEL_LEN) && (device_label[i] != 0); ++i)
      label[i] = device_label[i];
}

bool config_is_device_activated(void)
{
   return device_activated || (gps_available && !awake_on_magnet);
}

void config_set_activation_status(bool active)
{
   if (active)
   {
      storage_open(ACTIVATION_FILE_NAME, true);
      storage_write("OPEN", 4);
      storage_close();
   }
   else
      storage_delete(ACTIVATION_FILE_NAME);
}

bool config_gps_available(void)
{
   return gps_available;
}

bool config_awake_on_magnet(void)
{
   return awake_on_magnet;
}

int32_t config_get_utc_offset(void)
{
   return utc_offset;
}

int32_t config_get_num_deployment_phases(void)
{
   return num_deployment_phases;
}

int32_t config_get_active_deployment_phase_index(uint32_t current_time)
{
   for (int32_t i = 0; (num_deployment_phases != -1) && (i < num_deployment_phases); ++i)
      if ((deployment_phases[i].phase_time.start_time <= current_time) && (current_time <= deployment_phases[i].phase_time.end_time))
         return i;
   return -1;
}

int32_t config_get_next_deployment_phase_index(uint32_t current_time, uint32_t *next_start_time)
{
   for (int32_t i = 0; (num_deployment_phases != -1) && (i < num_deployment_phases); ++i)
      if (current_time < deployment_phases[i].phase_time.start_time)
      {
         *next_start_time = deployment_phases[i].phase_time.start_time;
         return i;
      }
   return -1;
}

uint32_t config_get_deployment_start_time(void)
{
   return deployment_time.start_time;
}

uint32_t config_get_deployment_end_time(void)
{
   return deployment_time.end_time;
}

uint32_t config_get_magnetic_field_validation_length(void)
{
   return magnetic_field_validation_length_ms;
}

bool config_set_rtc_at_magnet_detect(void)
{
   return set_rtc_at_magnet_detect;
}

bool config_get_leds_enabled(void)
{
   return leds_enabled;
}

uint32_t config_get_leds_active_seconds(void)
{
   return leds_enabled ? leds_active_seconds : 0;
}

uint32_t config_get_vhf_start_timestamp(void)
{
   return vhf_start_timestamp;
}

float config_get_mic_amplification_db(void)
{
   return microphone_amplification_db;
}

uint32_t config_get_start_time(int32_t phase_index)
{
   return deployment_phases[phase_index].phase_time.start_time;
}

uint32_t config_get_end_time(int32_t phase_index)
{
   return deployment_phases[phase_index].phase_time.end_time;
}

audio_recording_mode_t config_get_audio_recording_mode(int32_t phase_index)
{
   return deployment_phases[phase_index].audio_recording_mode;
}

bool config_get_max_audio_clips(int32_t phase_index, uint32_t *max_clips, time_scale_t *unit_time)
{
   *max_clips = deployment_phases[phase_index].max_audio_clips;
   *unit_time = deployment_phases[phase_index].max_clips_time_scale;
   return (deployment_phases[phase_index].audio_recording_mode == AMPLITUDE);
}

float config_get_audio_trigger_threshold(int32_t phase_index)
{
   return (deployment_phases[phase_index].audio_recording_mode == AMPLITUDE) ?
         deployment_phases[phase_index].audio_trigger_threshold : 0.0;
}

uint32_t config_get_audio_trigger_schedule(int32_t phase_index, start_end_time_t **schedule)
{
   *schedule = deployment_phases[phase_index].audio_trigger_times;
   return (deployment_phases[phase_index].audio_recording_mode == SCHEDULED) ?
         deployment_phases[phase_index].num_audio_trigger_times : 0;
}

bool config_get_audio_trigger_interval(int32_t phase_index, uint32_t *interval, time_scale_t *unit_time)
{
   *interval = deployment_phases[phase_index].audio_trigger_interval;
   *unit_time = deployment_phases[phase_index].audio_trigger_interval_time_scale;
   return (deployment_phases[phase_index].audio_recording_mode == INTERVAL);
}

uint32_t config_get_audio_clip_length_seconds(int32_t phase_index)
{
   return deployment_phases[phase_index].audio_clip_length;
}

bool config_extend_clip_for_continuous_audio(int32_t phase_index)
{
   return deployment_phases[phase_index].extend_clip_if_continuous_audio;
}

uint32_t config_get_audio_sampling_rate_hz(int32_t phase_index)
{
   return deployment_phases[phase_index].audio_sampling_rate;
}

imu_recording_mode_t config_get_imu_recording_mode(int32_t phase_index)
{
   return deployment_phases[phase_index].imu_recording_mode;
}

float config_get_imu_trigger_threshold_level(int32_t phase_index)
{
   return (deployment_phases[phase_index].imu_recording_mode == ACTIVITY) ?
         deployment_phases[phase_index].imu_trigger_threshold : 0.0;
}

uint8_t config_get_imu_degrees_of_freedom(int32_t phase_index)
{
   return deployment_phases[phase_index].imu_degrees_of_freedom;
}

uint32_t config_get_imu_sampling_rate_hz(int32_t phase_index)
{
   return deployment_phases[phase_index].imu_sampling_rate;
}
