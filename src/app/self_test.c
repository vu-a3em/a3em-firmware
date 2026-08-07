// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <math.h>
#include <stdio.h>
#include "audio.h"
#include "battery.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "rtc.h"
#include "self_test.h"
#include "storage.h"
#include "system.h"


// Static Global Variables ---------------------------------------------------------------------------------------------

static uint8_t storage_pattern[SELF_TEST_STORAGE_BYTES];


// Private Helper Functions --------------------------------------------------------------------------------------------

static bool test_storage(uint32_t *bytes_verified)
{
   // Write a known pattern, read it back, and compare.
   //
   // An unseated or failing SD card is a leading cause of lost deployments and is
   // invisible until the unit is opened again. A round trip through the filesystem
   // catches it in seconds at the bench.
   *bytes_verified = 0;
   for (uint32_t i = 0; i < sizeof(storage_pattern); ++i)
      storage_pattern[i] = (uint8_t)(i * 31u + 7u);

   if (!storage_open(SELF_TEST_STORAGE_FILE_NAME, true))
      return false;
   const bool written = storage_write(storage_pattern, sizeof(storage_pattern));
   storage_close();
   if (!written)
      return false;

   // Read back into the same buffer, so the comparison is against regenerated values
   // rather than against a copy that could share a corrupted source.
   if (!storage_open(SELF_TEST_STORAGE_FILE_NAME, false))
      return false;
   const uint32_t read_bytes = storage_read(storage_pattern, sizeof(storage_pattern));
   storage_close();
   storage_delete(SELF_TEST_STORAGE_FILE_NAME);

   if (read_bytes != sizeof(storage_pattern))
      return false;
   for (uint32_t i = 0; i < sizeof(storage_pattern); ++i)
      if (storage_pattern[i] != (uint8_t)(i * 31u + 7u))
         return false;

   *bytes_verified = sizeof(storage_pattern);
   return true;
}

static bool test_imu(float *magnitude_mg)
{
   // A stationary device reads one g in some direction, whatever its orientation. A
   // sensor that is unpowered, unresponsive, or not talking over I2C reads zero or a
   // rail, neither of which lands near gravity.
   float x = 0.0f, y = 0.0f, z = 0.0f;
   imu_read_accel_data(&x, &y, &z);
   *magnitude_mg = sqrtf((x * x) + (y * y) + (z * z));
   return (*magnitude_mg >= SELF_TEST_IMU_MIN_MG) && (*magnitude_mg <= SELF_TEST_IMU_MAX_MG);
}

static bool test_power_and_clock(uint32_t rtc_before, uint32_t rtc_after, battery_result_t battery)
{
   // The clock check is free: the microphone window took real time, so the RTC must
   // have advanced across it. A stopped clock silently destroys every timestamp on the
   // card, and is otherwise only discovered after retrieval.
   const bool clock_ticking = (rtc_after > rtc_before);
   const bool battery_plausible = (battery.millivolts >= SELF_TEST_BATTERY_MIN_MV) &&
                                  (battery.millivolts <= SELF_TEST_BATTERY_MAX_MV);
   const bool temperature_plausible = (battery.celcius >= SELF_TEST_TEMPERATURE_MIN_C) &&
                                      (battery.celcius <= SELF_TEST_TEMPERATURE_MAX_C);
   if (!clock_ticking)
      log_event("SELF_TEST_DETAIL", "check=RTC,result=FAIL,before=%u,after=%u", rtc_before, rtc_after);
   if (!battery_plausible)
      log_event("SELF_TEST_DETAIL", "check=BATTERY,result=FAIL,mv=%u", battery.millivolts);
   if (!temperature_plausible)
      log_event("SELF_TEST_DETAIL", "check=TEMPERATURE,result=FAIL,c=%0.2f", battery.celcius);
   return clock_ticking && battery_plausible && temperature_plausible;
}

static bool test_microphone(uint32_t activation_number, const char *device_label, audio_health_t *health)
{
   // Record a short window, tracking the level on the green LED as it goes.
   //
   // The clip length is deliberately one second: it sets the DMA buffer size, and the
   // deployment default of several seconds would make the LED lag far too much to tap
   // the enclosure and see a response.
   const uint32_t sample_rate = SELF_TEST_AUDIO_SAMPLE_RATE_HZ;
   if (config_get_mic_type() == MIC_ANALOG)
      audio_analog_init(AUDIO_NUM_CHANNELS, sample_rate, 1, config_get_mic_amplification_db(),
                        AUDIO_MIC_BIAS_VOLTAGE, IMMEDIATE, 0.0f, NULL);
   else
      audio_digital_init(AUDIO_NUM_CHANNELS, sample_rate, 1, config_get_mic_amplification_db());

   const uint32_t samples_per_buffer = audio_num_seconds_per_dma() * sample_rate;
   audio_health_reset();

   // Save the clip so the microphone port can be confirmed open by ear, which is how
   // a returned card gets checked anyway.
   const bool file_open = storage_open_named_wav_file(SELF_TEST_CLIP_FILE_NAME, AUDIO_NUM_CHANNELS, sample_rate);
   (void)activation_number;
   (void)device_label;

   audio_begin_reading();
   int16_t *buffer;
   for (uint32_t captured = 0; captured < SELF_TEST_AUDIO_SECONDS; )
   {
      if (audio_error_encountered())
         break;
      if (audio_data_available() && (buffer = audio_read_data_direct()))
      {
         if (file_open)
            storage_write_audio(buffer, sizeof(int16_t) * samples_per_buffer, false);

         // Live level feedback: the LED follows the loudest sample in this buffer, so
         // tapping the housing produces a visible response within about a second.
         int32_t peak = 0;
         for (uint32_t i = 0; i < samples_per_buffer; ++i)
         {
            const int32_t magnitude = (buffer[i] < 0) ? -(int32_t)buffer[i] : (int32_t)buffer[i];
            if (magnitude > peak)
               peak = magnitude;
         }
         if (peak > SELF_TEST_LIVE_LEVEL_THRESHOLD)
            led_on(LED_GREEN);
         else
            led_off(LED_GREEN);

         captured += audio_num_seconds_per_dma();
      }
      else
         system_enter_deep_sleep_mode();
   }
   audio_stop_reading();
   led_off(LED_GREEN);
   if (file_open)
      storage_close_audio();
   audio_deinit();

   *health = audio_health_get();

   // A constant output means the signal path is dead. Silence is reported but not
   // failed: a sealed enclosure in a quiet room is legitimately near-silent, and
   // failing on it would train people to ignore the check.
   if (health->constant_output)
   {
      log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=FAIL_CONSTANT,value=%d",
                (int)health->min_sample);
      return false;
   }
   if (config_get_mic_type() == MIC_ANALOG)
   {
      const int32_t dc = audio_get_dc_offset();
      const int32_t deviation = (dc > MIC_DC_OFFSET_NOMINAL) ? (dc - MIC_DC_OFFSET_NOMINAL)
                                                             : (MIC_DC_OFFSET_NOMINAL - dc);
      if (deviation > MIC_DC_OFFSET_TOLERANCE)
      {
         log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=FAIL_DC_OFFSET,dc_offset=%d", (int)dc);
         return false;
      }
   }
   return true;
}

static void indicate_result(self_test_result_t result)
{
   // Communicate the verdict without a computer: solid green for a pass, otherwise one
   // red blink per subsystem number so a failure can be identified at the bench.
   led_off(LED_ALL);
   if (result == SELF_TEST_PASS)
   {
      led_on(LED_GREEN);
      system_delay(SELF_TEST_PASS_INDICATION_US);
      led_off(LED_GREEN);
      return;
   }
   for (uint32_t repeat = 0; repeat < SELF_TEST_FAIL_INDICATION_REPEATS; ++repeat)
   {
      for (uint32_t blink = 0; blink < (uint32_t)result; ++blink)
      {
         led_on(LED_RED);
         system_delay(250000);
         led_off(LED_RED);
         system_delay(250000);
      }
      system_delay(1000000);
   }
}

static void write_results(self_test_result_t result, const audio_health_t *health,
                          uint32_t storage_bytes, float imu_magnitude_mg,
                          battery_result_t battery, uint32_t rtc_timestamp)
{
   // Same KEY = "value" grammar as the configuration and device info files, so the
   // dashboard reads it with the parser it already has.
   char label[1 + MAX_DEVICE_LABEL_LEN] = { 0 };
   config_get_device_label(label, sizeof(label));
   if (!storage_open(SELF_TEST_RESULTS_FILE_NAME, true))
      return;

   char line[128];
   #define WRITE_RESULT(fmt, ...) \
      do { \
         const int len = snprintf(line, sizeof(line), fmt "\n", ##__VA_ARGS__); \
         if (len > 0) storage_write(line, (uint32_t)len); \
      } while (0)

   WRITE_RESULT("FW_VERSION = \"%s\"", _FW_VERSION);
   WRITE_RESULT("DEVICE_LABEL = \"%s\"", label);
   WRITE_RESULT("TEST_TIMESTAMP = \"%u\"", (unsigned)rtc_timestamp);
   WRITE_RESULT("ACTIVATION_NUMBER = \"%u\"", (unsigned)config_get_activation_number());
   WRITE_RESULT("OVERALL = \"%s\"", (result == SELF_TEST_PASS) ? "PASS" : "FAIL");
   WRITE_RESULT("FAILED_SUBSYSTEM = \"%d\"", (int)result);

   WRITE_RESULT("MIC_TYPE = \"%s\"", (config_get_mic_type() == MIC_ANALOG) ? "ANALOG" : "DIGITAL");
   WRITE_RESULT("MIC_RESULT = \"%s\"", (result == SELF_TEST_FAIL_MICROPHONE) ? "FAIL" :
                                       (health->silent ? "PASS_SILENT" : "PASS"));
   WRITE_RESULT("MIC_RMS = \"%u\"", (unsigned)health->rms);
   WRITE_RESULT("MIC_PEAK = \"%u\"", (unsigned)health->peak);
   WRITE_RESULT("MIC_MIN = \"%d\"", (int)health->min_sample);
   WRITE_RESULT("MIC_MAX = \"%d\"", (int)health->max_sample);
   WRITE_RESULT("MIC_MEAN = \"%d\"", (int)health->mean);
   WRITE_RESULT("MIC_SAMPLES = \"%u\"", (unsigned)health->num_samples);
   WRITE_RESULT("MIC_DC_OFFSET = \"%d\"", (int)audio_get_dc_offset());
   WRITE_RESULT("MIC_CONSTANT_OUTPUT = \"%s\"", health->constant_output ? "True" : "False");

   WRITE_RESULT("SD_RESULT = \"%s\"", storage_bytes ? "PASS" : "FAIL");
   WRITE_RESULT("SD_BYTES_VERIFIED = \"%u\"", (unsigned)storage_bytes);
   WRITE_RESULT("SD_FREE_MB = \"%u\"", (unsigned)storage_get_free_space_mb());

   WRITE_RESULT("IMU_RESULT = \"%s\"", (result == SELF_TEST_FAIL_IMU) ? "FAIL" : "PASS");
   WRITE_RESULT("IMU_MAGNITUDE_MG = \"%d\"", (int)imu_magnitude_mg);

   WRITE_RESULT("BATTERY_MV = \"%u\"", (unsigned)battery.millivolts);
   WRITE_RESULT("TEMPERATURE_C = \"%0.2f\"", battery.celcius);
   WRITE_RESULT("RTC_VALID = \"%s\"", rtc_is_valid() ? "True" : "False");
   #undef WRITE_RESULT

   storage_close();
}


// Public API Functions ------------------------------------------------------------------------------------------------

self_test_result_t run_self_test(void)
{
   log_event("SELF_TEST_START", "fw=%s", _FW_VERSION);

   // Force the LEDs on for the duration regardless of configuration: the indication is
   // the entire point of running this, and a unit configured for a dark deployment
   // still needs to be verifiable on the bench.
   const bool leds_were_enabled = leds_are_enabled();
   leds_enable(true);

   char device_label[1 + MAX_DEVICE_LABEL_LEN] = { 0 };
   config_get_device_label(device_label, sizeof(device_label));
   const uint32_t activation_number = config_get_activation_number();
   const uint32_t rtc_before = rtc_get_timestamp();

   // Storage first: everything after this wants somewhere to record results.
   uint32_t storage_bytes = 0;
   const bool storage_ok = test_storage(&storage_bytes);
   log_event("SELF_TEST_DETAIL", "check=STORAGE,result=%s,bytes=%u",
             storage_ok ? "PASS" : "FAIL", storage_bytes);

   float imu_magnitude_mg = 0.0f;
   const bool imu_ok = test_imu(&imu_magnitude_mg);
   log_event("SELF_TEST_DETAIL", "check=IMU,result=%s,magnitude_mg=%d",
             imu_ok ? "PASS" : "FAIL", (int)imu_magnitude_mg);

   audio_health_t health = { 0 };
   const bool microphone_ok = test_microphone(activation_number, device_label, &health);
   log_event("SELF_TEST_DETAIL", "check=MICROPHONE,result=%s,rms=%u,peak=%u,silent=%s",
             microphone_ok ? "PASS" : "FAIL", health.rms, health.peak,
             health.silent ? "True" : "False");

   const battery_result_t battery = battery_monitor_get_details();
   const uint32_t rtc_after = rtc_get_timestamp();
   const bool power_ok = test_power_and_clock(rtc_before, rtc_after, battery);
   log_event("SELF_TEST_DETAIL", "check=POWER,result=%s,mv=%u,temp_c=%0.2f",
             power_ok ? "PASS" : "FAIL", battery.millivolts, battery.celcius);

   // Report the first failure, in the order a failure most likely matters
   self_test_result_t result = SELF_TEST_PASS;
   if (!microphone_ok)
      result = SELF_TEST_FAIL_MICROPHONE;
   else if (!storage_ok)
      result = SELF_TEST_FAIL_STORAGE;
   else if (!imu_ok)
      result = SELF_TEST_FAIL_IMU;
   else if (!power_ok)
      result = SELF_TEST_FAIL_POWER_OR_CLOCK;

   write_results(result, &health, storage_bytes, imu_magnitude_mg, battery, rtc_after);
   log_event("SELF_TEST_END", "result=%s,failed_subsystem=%d",
             (result == SELF_TEST_PASS) ? "PASS" : "FAIL", (int)result);
   storage_flush_log();

   indicate_result(result);
   leds_enable(leds_were_enabled);
   return result;
}
