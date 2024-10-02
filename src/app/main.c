#include "audio.h"
#include "battery.h"
#include "comparator.h"
#include "henrik.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "magnet.h"
#include "rtc.h"
#include "storage.h"
#include "system.h"
#include "vhf.h"

extern void active_main(int32_t);

static volatile bool magnetic_field_verified, device_activated;

static void magnet_sensor_validated(bool validated)
{
   print("INFO: Magnetic field %s validated!\n", validated ? "SUCCESSFULLY" : "NOT");
   device_activated = validated;
   magnetic_field_verified = true;
}

static void magnet_sensor_activated(void)
{
   magnetic_field_verified = device_activated = false;
   const uint32_t validation_length_ms = config_get_magnetic_field_validation_length();
   print("INFO: Magnetic field detected...validating for %u ms\n", validation_length_ms);
   magnet_sensor_verify_field(validation_length_ms, magnet_sensor_validated);
}

int main(void)
{
   // Set up the system hardware and retrieve the device ID
   setup_hardware();
   static uint8_t device_id[DEVICE_ID_LEN];
   system_read_ID(device_id, sizeof(device_id));
   print("INFO: System hardware initialized, UID = ");
   for (size_t i = DEVICE_ID_LEN - 1; i > 0; --i)
      print("%02X:", device_id[i]);
   print("%02X\n", device_id[0]);

   // Initialize peripherals and start up the RTC
   print("INFO: Initializing peripherals...\n");
   leds_init();
   rtc_init();
   vhf_init();
   imu_init();
   henrik_init();
   magnet_sensor_init();
   comparator_init(false, 0, 1.0, true);
   battery_monitor_init();
   storage_init();
   system_enable_interrupts(true);
   print("INFO: Peripherals initialized!\n");

   // Retrieve the runtime configuration from storage
   magnetic_field_verified = device_activated = false;
   print("INFO: Fetching runtime configuration...%s\n", fetch_runtime_configuration() ? "SUCCESS" : "FAILURE (Using defaults)");
   if (!config_get_leds_enabled())
      leds_deinit();

   // Determine if the battery voltage is too low to continue
   if (battery_monitor_get_details().millivolts <= BATTERY_LOW)
   {
      print("WARNING: Battery low...shutting down for 1 hour\n");
      const uint32_t current_timestamp = rtc_get_timestamp();
      const uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
      const bool vhf_enabled = config_is_device_activated() && vhf_enable_timestamp && (current_timestamp >= vhf_enable_timestamp);
      if (vhf_enabled)
         vhf_activate();
      system_enter_power_off_mode(0, current_timestamp + 3600, !vhf_enabled, false);
      system_reset();
   }

   // Determine whether the device has been activated or not
   if (config_is_device_activated())
   {
      // Determine if the VHF radio should already be activated
      print("INFO: Device is ACTIVATED\n");
      const uint32_t current_timestamp = rtc_get_timestamp();
      const uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
      if (vhf_enable_timestamp && (current_timestamp >= vhf_enable_timestamp))
         vhf_activate();

      // Check that the device was not erroneously activated and that the deployment has not ended
      if (!rtc_is_valid() || (current_timestamp < config_get_deployment_start_time()))
         config_set_activation_status(false);
      else if (current_timestamp >= config_get_deployment_end_time())
      {
         // Go to sleep forever or until time for the VHF radio to be activated
         print("INFO: Deployment is COMPLETED\n");
         if (current_timestamp >= vhf_enable_timestamp)
            system_enter_power_off_mode(0, 0, !vhf_enable_timestamp, false);
         else
            system_enter_power_off_mode(0, vhf_enable_timestamp, true, false);
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
            if (current_timestamp >= vhf_enable_timestamp)
               system_enter_power_off_mode(0, 0, !vhf_enable_timestamp, false);
            else
               system_enter_power_off_mode(0, vhf_enable_timestamp, true, false);
         }
         else if (active_phase >= 0)
            active_main(active_phase);
         else if (vhf_enable_timestamp && (vhf_enable_timestamp > current_timestamp))
         {
            print("INFO: Sleeping until next deployment phase or VHF radio should be enabled\n");
            system_enter_power_off_mode(0, (vhf_enable_timestamp < next_phase_start_time) ? vhf_enable_timestamp : next_phase_start_time, true, false);
         }
         else
         {
            print("INFO: Sleeping until next deployment phase\n");
            system_enter_power_off_mode(0, next_phase_start_time, true, false);
         }
      }
   }
   else if (!rtc_is_valid())
   {
      // Check whether we should listen for a valid GPS timestamp
      print("INFO: Device is NOT ACTIVATED\n");
      print("WARNING: RTC time has NOT been established!\n");
      if (config_gps_available())
      {
         // Register a magnetic field callback to immediately activate the device
         if (config_awake_on_magnet())
            magnet_sensor_register_callback(magnet_sensor_activated);

         // Wait until a valid GPS time or magnetic activation has been received
         print("INFO: Obtaining current time from GPS...\n");
         while (!device_activated)
         {
            uint32_t utc_time = henrik_get_gps_timestamp();
            if (utc_time)
            {
               print("INFO: GPS time obtained!\n");
               rtc_set_time_from_timestamp(utc_time);
               if (utc_time >= config_get_deployment_start_time())
                  config_set_activation_status(true);
               system_reset();
            }
            print("INFO: Sleeping until GPS response received\n");
            system_enter_deep_sleep_mode();
         }

         // Magnetic activation requested
         print("INFO: Device magnetically activated!\n");
         if (config_set_rtc_at_magnet_detect())
         {
            print("INFO: Setting RTC time to the deployment start time\n");
            rtc_set_time_from_timestamp(config_get_deployment_start_time());
         }
         config_set_activation_status(true);
      }
      else
      {
         // Wait until a magnetic activation has been detected
         print("INFO: Waiting for magnetic activation...\n");
         while (!device_activated)
         {
            // Enter the lowest possible power mode, listening only for magnetic fields
            system_enter_power_off_mode(PIN_MAG_SENSOR_INP, 0, true, true);
            print("INFO: Device woke up. Sensing for magnetic field...\n");

            // Magnetic field detected, re-initialize and validate activation
            magnet_sensor_init();
            magnetic_field_verified = device_activated = false;
            magnet_sensor_verify_field(config_get_magnetic_field_validation_length(), magnet_sensor_validated);
            while (!magnetic_field_verified)
               system_enter_deep_sleep_mode();
         }
         print("INFO: Device magnetically activated!\n");
         if (config_set_rtc_at_magnet_detect())
         {
            print("INFO: Setting RTC time to the deployment start time\n");
            rtc_set_time_from_timestamp(config_get_deployment_start_time());
         }
         config_set_activation_status(true);
      }
   }
   else
   {
      // Check that it is not already past the desired device activation time
      print("INFO: Device is NOT ACTIVATED, but the RTC time has been established!\n");
      const uint32_t vhf_enable_timestamp = config_get_vhf_start_timestamp();
      const uint32_t activation_timestamp = config_get_deployment_start_time();
      if (rtc_get_timestamp() >= activation_timestamp)
         device_activated = true;
      else if (vhf_enable_timestamp && (rtc_get_timestamp() >= vhf_enable_timestamp))
         vhf_activate();

      // Loop waiting for time- or magnetic-based activations
      while (!device_activated)
      {
         // Enter the lowest possible power mode until activation time (or magnetic activation)
         if (vhf_enable_timestamp && (vhf_enable_timestamp > rtc_get_timestamp()))
            system_enter_power_off_mode(config_awake_on_magnet() ? PIN_MAG_SENSOR_INP : 0, (vhf_enable_timestamp < activation_timestamp) ? vhf_enable_timestamp : activation_timestamp, true, true);
         else
            system_enter_power_off_mode(config_awake_on_magnet() ? PIN_MAG_SENSOR_INP : 0, activation_timestamp, !vhf_enable_timestamp, true);

         // Check that the current time is after the desired device activation time
         if (rtc_get_timestamp() >= activation_timestamp)
            device_activated = true;
         else
         {
            // Magnetic field detected, re-initialize and validate activation
            print("INFO: Device woke up. Sensing for magnetic field...\n");
            magnet_sensor_init();
            magnetic_field_verified = device_activated = false;
            magnet_sensor_verify_field(config_get_magnetic_field_validation_length(), magnet_sensor_validated);
            while (!magnetic_field_verified)
               system_enter_deep_sleep_mode();
         }
      }
      print("INFO: Device activated!\n");
      config_set_activation_status(true);
   }

   // Reboot the system and start over from the top
   system_reset();
   return 0;
}
