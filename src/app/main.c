#include <stdio.h>
#include "battery.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "mram.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"

// _HW_REVISION arrives from the Makefile as a bare token, so it needs two levels of
// expansion to become a string literal.
#define _STRINGIFY_INNER(x)  #x
#define _STRINGIFY(x)        _STRINGIFY_INNER(x)
#include "tracker.h"
#include "vhf.h"

static volatile bool magnetic_field_verified, device_activated;

extern void active_main(volatile bool*, int32_t);
extern void pre_active_main(volatile bool*);
extern uint32_t active_main_get_end_reason(void);

// Render a reset reason code as the short label written into the root boot log
static void log_boot_reason(void)
{
   // Record why the device restarted, both in the human-readable deployment log and in the fixed-record root boot log
   const system_boot_info_t *boot = system_get_boot_info();
   const am_hal_reset_status_t *hw = &boot->hardware_status;
   uint32_t hardware_bits = 0;
   hardware_bits |= hw->bEXTStat      ? (1u << 0)  : 0;
   hardware_bits |= hw->bPORStat      ? (1u << 1)  : 0;
   hardware_bits |= hw->bBODStat      ? (1u << 2)  : 0;
   hardware_bits |= hw->bSWPORStat    ? (1u << 3)  : 0;
   hardware_bits |= hw->bSWPOIStat    ? (1u << 4)  : 0;
   hardware_bits |= hw->bDBGRStat     ? (1u << 5)  : 0;
   hardware_bits |= hw->bWDTStat      ? (1u << 6)  : 0;
   hardware_bits |= hw->bBOUnregStat  ? (1u << 7)  : 0;
   hardware_bits |= hw->bBOCOREStat   ? (1u << 8)  : 0;
   hardware_bits |= hw->bBOMEMStat    ? (1u << 9)  : 0;
   hardware_bits |= hw->bBOHPMEMStat  ? (1u << 10) : 0;
   hardware_bits |= hw->bBOLPCOREStat ? (1u << 11) : 0;
   // Stamped into every log so a retrieved card says which build produced it. Without
   // this, reconciling odd data against a firmware change means guessing from dates.
   print("INFO: A3EM firmware %s, built %s\n", _FW_VERSION, _DATETIME);
   print("INFO: Restart #%u of power-on epoch %u, reason %s (hardware bits 0x%03X)\n", boot->resets_this_epoch, boot->boot_epoch, reset_reason_name(boot->software_reason), hardware_bits);
   if (boot->software_reason == RESET_REASON_HARD_FAULT)
      print("ERROR: Previous run ended in a hard fault at address 0x%08X\n", boot->fault_address);
   if (hw->bWDTStat)
      print("ERROR: Previous run was terminated by the watchdog\n");

   // The detail field carries the fault address for a fault, and the hardware reset bits otherwise
   const uint32_t detail = (boot->software_reason == RESET_REASON_HARD_FAULT) ? boot->fault_address : hardware_bits;
   storage_write_boot_record(reset_reason_name(boot->software_reason), boot->boot_epoch,
                             boot->resets_this_epoch, detail, rtc_get_timestamp());
}

static void magnet_sensor_validated(bool validated)
{
   // Set device activation and field verification flags and indicate status via LED
   if (validated)
   {
      if (!device_activated)
         device_activated = true;
      else if (config_is_deactivation_allowed())
         device_activated = false;
      led_indicate_activation(device_activated);
   }
   magnetic_field_verified = true;
}

static void magnet_sensor_activated(bool field_detected)
{
   // Indicate magnetic field presence via LED and begin verification
   led_indicate_magnet_presence(field_detected);
   if (field_detected)
      magnet_sensor_verify_field(config_get_magnetic_field_validation_length(), magnet_sensor_validated);
}

static void handle_magnetic_field(bool store_activated_result, bool store_deactivated_result)
{
   // Validate a magnetic activation or deactivation
   magnetic_field_verified = false;
   magnet_sensor_register_callback(magnet_sensor_activated);
   magnet_sensor_verify_field(config_get_magnetic_field_validation_length(), magnet_sensor_validated);
   while (!magnetic_field_verified)
   {
      // The validation callback is delivered from here rather than from the timer interrupt
      system_feed_watchdog();
      magnet_sensor_handle_pending_validation();
      if (!magnetic_field_verified && !magnet_sensor_validation_in_progress())
         break;
      if (!magnetic_field_verified)
         system_enter_deep_sleep_mode();
   }

   // Let any confirmation pattern finish before returning
   led_pattern_wait();
   if (store_activated_result && device_activated)
      config_set_activation_status(true);
   else if (store_deactivated_result && !device_activated)
      config_set_activation_status(false);
}

int main(void)
{
   // Set up the system hardware, retrieve the device ID, and initialize all peripherals
   setup_hardware();
   static uint8_t device_id[DEVICE_ID_LEN];
   system_read_ID(device_id, sizeof(device_id));
   system_initialize_peripherals();

   // Arm the watchdog for the whole life of the application
   system_enable_watchdog();

   print("\nINFO: System hardware initialized, UID = ");
   for (size_t i = DEVICE_ID_LEN - 1; i > 0; --i)
      print("%02X:", device_id[i]);
   print("%02X\n", device_id[0]);

   // Retrieve the runtime configuration from storage
   bool success = fetch_runtime_configuration();
   print("INFO: Fetching runtime configuration...%s\n", success ? "SUCCESS" : "FAILURE");

   // Record device identity and last-known state where the dashboard can read it without
   // parsing a multi-megabyte log. Rewritten on every boot, so it is always current.
   if (!storage_sd_card_error())
   {
      char device_uid[(3 * DEVICE_ID_LEN) + 1] = { 0 };
      for (size_t i = 0; i < DEVICE_ID_LEN; ++i)
         snprintf(device_uid + (3 * i), sizeof(device_uid) - (3 * i),
                  (i == (DEVICE_ID_LEN - 1)) ? "%02X" : "%02X:", device_id[DEVICE_ID_LEN - 1 - i]);
      const system_boot_info_t *boot_info = system_get_boot_info();
      storage_write_device_info(_FW_VERSION, _STRINGIFY(_HW_REVISION), _DATETIME, device_uid,
                                config_get_activation_number(), rtc_get_timestamp(),
                                battery_monitor_get_details().millivolts,
                                reset_reason_name(boot_info->software_reason));
   }
   const bool use_magnetic_activation = config_awake_on_magnet();
   if (storage_sd_card_error())
      led_indicate_sd_card_error();
   else if (!success)
      led_indicate_missing_config_file();
   led_pattern_wait();
   leds_enable(config_get_leds_enabled());

   // Ensure that the RTC has a usable (but "invalid") time so that it can function for sleeping/wake-up
   if (!rtc_is_valid())
      rtc_set_time_from_timestamp(1604083082);

   // Record why the device restarted, now that storage is available to receive it
   log_boot_reason();

   // Release the I2C slave used to talk to the GPS tracker board when no tracker is configured
   if (!config_gps_available())
      tracker_deinit();

   // Reboot after 15 seconds if missing SD card or configuration file
   if (storage_sd_card_error() || !success)
   {
      system_enter_power_off_mode(PIN_MAG_SENSOR_INP, rtc_get_timestamp() + 15, false);
      system_reset_with_reason(RESET_REASON_MISSING_CONFIG);
   }

   // Determine if the battery voltage is too low to continue
   bool battery_too_low = false;
   while (battery_monitor_is_critically_low(config_get_battery_mV_low()))
   {
      const uint32_t current_timestamp = rtc_get_timestamp();
      const uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
      print("WARNING: Battery confirmed low @ %u...shutting down for 1 hour\n", current_timestamp);
      const bool vhf_enabled = config_is_device_activated() && vhf_enable_timestamp && (current_timestamp >= vhf_enable_timestamp);
      if (vhf_enabled)
         vhf_activate();
      system_enter_power_off_mode(PIN_MAG_SENSOR_INP, current_timestamp + 3600, true);
      battery_too_low = true;
   }
   if (battery_too_low)
      system_reset_with_reason(RESET_REASON_BATTERY_LOW);

   // Determine whether the device has been activated
   device_activated = config_is_device_activated();
   if (device_activated)
   {
      // Check for RTC errors and attempt to correct them to allow the deployment to continue
      uint32_t gps_time_attempts = 0;
      while (!rtc_is_valid())
      {
         if (config_gps_available() && (gps_time_attempts < GPS_TIME_MAX_ATTEMPTS))
         {
            // Wait until a valid GPS time has been received
            ++gps_time_attempts;
            print("INFO: Obtaining current time from GPS...\n");
            uint32_t utc_time = tracker_get_current_time();
            if (utc_time)
            {
               print("INFO: GPS time obtained: %u\n", utc_time);
               mram_set_last_known_timestamp(utc_time);
               rtc_set_time_from_timestamp(utc_time);
            }
            else
            {
               print("INFO: Sleeping until GPS response received\n");
               if (use_magnetic_activation)
                  magnet_sensor_register_callback(magnet_sensor_activated);
               system_feed_watchdog();
               magnet_sensor_handle_pending_validation();
               system_enter_deep_sleep_mode();
               if (use_magnetic_activation && !device_activated)
               {
                  config_set_activation_status(false);
                  system_reset_with_reason(RESET_REASON_MAGNET_DEACTIVATED);
               }
            }
         }
         else
         {
            // Log this error and set the RTC to the last known timestamp
            if (config_gps_available())
               print("ERROR: No GPS time after %u attempts - falling back to the last known timestamp\n", gps_time_attempts);
            uint32_t last_known_timestamp = mram_get_last_known_timestamp();
            if (last_known_timestamp)
               rtc_set_time_from_timestamp(last_known_timestamp);
            else
               rtc_set_time_to_compile_time();
            print("ERROR: RTC time appears to have been lost...setting to last known timestamp: %u\n", rtc_get_timestamp());
         }
      }

      // Determine if the VHF radio should already be active
      print("INFO: Device is ACTIVATED\n");
      print("INFO: Current activation is #%u\n", config_get_activation_number());
      uint32_t current_timestamp = rtc_get_timestamp();
      print("INFO: Current RTC timestamp = %u\n", current_timestamp);
      const uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
      if (vhf_enable_timestamp && (current_timestamp >= vhf_enable_timestamp))
         vhf_activate();

      // Verify that the current time is within the deployment start and end times
      if (current_timestamp < config_get_deployment_start_time())
      {
         // Run continuous recording for 1 minute to allow user to make voice notes
         pre_active_main(&device_activated);
         current_timestamp = rtc_get_timestamp();

         // Go to sleep until deployment start time
         while (device_activated && (current_timestamp < config_get_deployment_start_time()))
         {
            print("INFO: Deployment starts in %u seconds\n", config_get_deployment_start_time() - current_timestamp);
            system_enter_power_off_mode(use_magnetic_activation ? PIN_MAG_SENSOR_INP : 0, config_get_deployment_start_time(), use_magnetic_activation);
            current_timestamp = rtc_get_timestamp();
            if (use_magnetic_activation && (current_timestamp < config_get_deployment_start_time()))
               handle_magnetic_field(false, true);
         }
      }
      else if (current_timestamp >= config_get_deployment_end_time())
      {
         // Go to sleep forever or until time for the VHF radio to be activated
         print("INFO: Deployment is COMPLETED\n");
         system_enter_power_off_mode(use_magnetic_activation ? PIN_MAG_SENSOR_INP : 0, (current_timestamp >= vhf_enable_timestamp) ? 0 : vhf_enable_timestamp, use_magnetic_activation);
         if (use_magnetic_activation)
            handle_magnetic_field(false, true);
      }
      else
      {
         // Search for the current and next active deployment phases
         uint32_t next_phase_start_time = 0;
         int32_t active_phase = config_get_active_deployment_phase_index(current_timestamp);
         int32_t next_phase = config_get_next_deployment_phase_index(current_timestamp, &next_phase_start_time);

         // Check if there is a currently active phase or if one will become active in the future
         if ((active_phase < 0) && (next_phase < 0))
         {
            // Go to sleep forever or until time for the VHF radio to be activated
            print("WARNING: Deployment active, but no scheduled phases remaining!\n");
            system_enter_power_off_mode(use_magnetic_activation ? PIN_MAG_SENSOR_INP : 0, (current_timestamp >= vhf_enable_timestamp) ? 0 : vhf_enable_timestamp, use_magnetic_activation);
            if (use_magnetic_activation)
               handle_magnetic_field(false, true);
         }
         else if (active_phase >= 0)
         {
            // Optionally register a magnetic field detection callback and start the main activity loop
            if (use_magnetic_activation)
               magnet_sensor_register_callback(magnet_sensor_activated);
            active_main(&device_activated, active_phase);

            // Store a change in device activation
            if (use_magnetic_activation && !device_activated)
            {
               print("INFO: Device was magnetically deactivated!\n");
               config_set_activation_status(false);
               storage_flush_log();
               led_pattern_wait();
               system_reset_with_reason(RESET_REASON_MAGNET_DEACTIVATED);
            }
            storage_flush_log();
            system_reset_with_reason(active_main_get_end_reason());
         }
         else if (vhf_enable_timestamp && (vhf_enable_timestamp > current_timestamp))
         {
            // If waiting for first phase, run pre-deployment recording activity
            if (!next_phase)
            {
               pre_active_main(&device_activated);
               current_timestamp = rtc_get_timestamp();
            }

            // Sleep until the next phase starts
            print("INFO: Sleeping until next deployment phase or VHF radio should be enabled\n");
            while (device_activated && (current_timestamp < vhf_enable_timestamp) && (current_timestamp < next_phase_start_time))
            {
               system_enter_power_off_mode(use_magnetic_activation ? PIN_MAG_SENSOR_INP : 0, (vhf_enable_timestamp < next_phase_start_time) ? vhf_enable_timestamp : next_phase_start_time, use_magnetic_activation);
               current_timestamp = rtc_get_timestamp();
               if ((current_timestamp < vhf_enable_timestamp) && (current_timestamp < next_phase_start_time))
                  handle_magnetic_field(false, true);
            }
         }
         else
         {
            // If waiting for first phase, run pre-deployment recording activity
            if (!next_phase)
            {
               pre_active_main(&device_activated);
               current_timestamp = rtc_get_timestamp();
            }

            // Sleep until the next phase starts
            print("INFO: Sleeping until next deployment phase\n");
            while (device_activated && (current_timestamp < next_phase_start_time))
            {
               system_enter_power_off_mode(use_magnetic_activation ? PIN_MAG_SENSOR_INP : 0, next_phase_start_time, use_magnetic_activation);
               current_timestamp = rtc_get_timestamp();
               if ((current_timestamp < next_phase_start_time))
                  handle_magnetic_field(false, true);
            }
         }
      }
   }
   else
   {
      // Wait until a magnetic activation has been detected
      print("INFO: Device is NOT ACTIVATED\n");
      print("INFO: Waiting for magnetic activation...\n");
      while (!device_activated)
      {
         // Enter the lowest possible power mode, listening only for magnetic fields
         system_enter_power_off_mode(PIN_MAG_SENSOR_INP, 0, true);
         handle_magnetic_field(true, false);
      }
      print("INFO: Device activated!\n");
      config_increase_activation_number();
      if (config_set_rtc_at_magnet_detect())
      {
         print("INFO: Setting RTC to the deployment start time: %u\n", config_get_deployment_start_time());
         mram_set_last_known_timestamp(config_get_deployment_start_time());
         rtc_set_time_from_timestamp(config_get_deployment_start_time());
      }
      system_delay(200000);
   }

   // Reboot the system and start over from the top
   storage_flush_log();
   system_reset_with_reason(device_activated ? RESET_REASON_ACTIVATED : RESET_REASON_UNKNOWN);
   return 0;
}
