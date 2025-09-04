#include "battery.h"
#include "henrik.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "mram.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"
#include "vhf.h"

static volatile bool magnetic_field_verified, device_activated;

extern void active_main(volatile bool*, int32_t);

static void magnet_sensor_validated(bool validated)
{
   // Set device activation and field verification flags and indicate status via LED
   device_activated = validated ? !device_activated : device_activated;
   led_indicate_activation(device_activated);
   magnetic_field_verified = true;
}

static void magnet_sensor_activated(bool field_detected)
{
   // Indicate magnetic field presence via LED and begin verification
   led_indicate_magnet_presence(field_detected);
   magnetic_field_verified = !field_detected;
   if (field_detected)
      magnet_sensor_verify_field(config_get_magnetic_field_validation_length(), magnet_sensor_validated, true);
}

static void handle_magnetic_field(bool store_activated_result, bool store_deactivated_result)
{
   // Validate a magnetic activation or deactivation
   magnetic_field_verified = false;
   magnet_sensor_verify_field(config_get_magnetic_field_validation_length(), magnet_sensor_validated, false);
   magnet_sensor_register_callback(magnet_sensor_activated);
   while (!magnetic_field_verified)
      system_enter_deep_sleep_mode();
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
   print("\nINFO: System hardware initialized, UID = ");
   for (size_t i = DEVICE_ID_LEN - 1; i > 0; --i)
      print("%02X:", device_id[i]);
   print("%02X\n", device_id[0]);

   // Retrieve the runtime configuration from storage
   bool success = fetch_runtime_configuration();
   print("INFO: Fetching runtime configuration...%s\n", success ? "SUCCESS" : "FAILURE (Using defaults)");
   const bool use_magnetic_activation = config_awake_on_magnet();
   if (storage_sd_card_error())
      led_indicate_sd_card_error();
   else if (!success)
      led_indicate_missing_config_file();
   leds_enable(config_get_leds_enabled());

   // Determine if the battery voltage is too low to continue
   bool battery_too_low = false;
   while (battery_monitor_get_details().millivolts <= config_get_battery_mV_low())
   {
      const uint32_t current_timestamp = rtc_get_timestamp();
      const uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
      print("WARNING: Battery low @ %u...shutting down for 1 hour\n", current_timestamp);
      const bool vhf_enabled = config_is_device_activated() && vhf_enable_timestamp && (current_timestamp >= vhf_enable_timestamp);
      if (vhf_enabled)
         vhf_activate();
      system_enter_power_off_mode(PIN_MAG_SENSOR_INP, current_timestamp + 3600, true);
      battery_too_low = true;
   }
   if (battery_too_low)
      system_reset();

   // Determine whether the device has been activated
   device_activated = config_is_device_activated();
   if (device_activated)
   {
      // Check for RTC errors and attempt to correct them to allow the deployment to continue
      while (!rtc_is_valid())
      {
         if (config_gps_available())
         {
            // Wait until a valid GPS time has been received
            print("INFO: Obtaining current time from GPS...\n");
            uint32_t utc_time = henrik_get_current_time();
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
               system_enter_deep_sleep_mode();
               if (use_magnetic_activation && !device_activated)
               {
                  config_set_activation_status(false);
                  system_reset();
               }
            }
         }
         else
         {
            // Log this error and set the RTC to the last known timestamp
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
      uint32_t current_timestamp = rtc_get_timestamp();
      print("INFO: Current RTC timestamp = %u\n", current_timestamp);
      const uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
      if (vhf_enable_timestamp && (current_timestamp >= vhf_enable_timestamp))
         vhf_activate();

      // Verify that the current time is within the deployment start and end times
      if (current_timestamp < config_get_deployment_start_time())
      {
         // Go to sleep until deployment start time
         while (device_activated && (current_timestamp < config_get_deployment_start_time()))
         {
            print("INFO: Deployment starts in %u seconds\n", config_get_deployment_start_time() - current_timestamp);
            system_enter_power_off_mode(use_magnetic_activation ? PIN_MAG_SENSOR_INP : 0, config_get_deployment_start_time(), use_magnetic_activation);
            current_timestamp = rtc_get_timestamp();
            if (current_timestamp < config_get_deployment_start_time())
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
            }
         }
         else if (vhf_enable_timestamp && (vhf_enable_timestamp > current_timestamp))
         {
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
      if (config_set_rtc_at_magnet_detect())
      {
         print("INFO: Setting RTC to the deployment start time: %u\n", config_get_deployment_start_time());
         mram_set_last_known_timestamp(config_get_deployment_start_time());
         rtc_set_time_from_timestamp(config_get_deployment_start_time());
      }
      system_delay(200000);
   }

   // Reboot the system and start over from the top
   system_reset();
   return 0;
}
