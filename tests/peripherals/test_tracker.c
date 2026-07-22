#include "logging.h"
#include "system.h"
#include "tracker.h"

static void tracker_data_available(tracker_msg_t message_type, const void *new_data)
{
   // Handle the incoming message based on its type
   switch (message_type)
   {
      case MSG_GPS:
      {
         const tracker_gps_data_t *gps_data = (const tracker_gps_data_t*)new_data;
         print("GPS data received: Timestamp = %u, Lat = %.2f, Lon = %.2f, Ht = %.2f\n",
               gps_data->utc_timestamp, gps_data->lat, gps_data->lon, gps_data->height);
         break;
      }
      case MSG_CONFIG:
         const tracker_config_data_t *config_data = (const tracker_config_data_t*)new_data;
         print("Config data received: Msg Type = %u, TODO\n", (uint32_t)config_data->msg_type);
         break;
      default:
         break;
   }
}

int main(void)
{
   // Set up system hardware
   setup_hardware();
   tracker_init();
   system_enable_interrupts(true);

   // Listen for incoming data from the GPS Tracker board
   tracker_register_data_callback(tracker_data_available);

   // Loop forever
   uint32_t current_time = 0;
   while (true)
   {
      // Test GPS time requests
      print("Waiting for a GPS time update from Tracker...\n");
      am_hal_delay_us(1000000);
      while (!(current_time = tracker_get_current_time()))
         am_hal_delay_us(1000000);
      print("GPS time received!\n");

      // Test intermittent status updates
      for (int i = 0; i < 3; ++i)
      {
         for (int j = 0; j < 3; ++j)
         {
            print("Updating on-device status data (time = %u)...\n", ++current_time);
            tracker_update_status_data(current_time);
            am_hal_delay_us(1000000);
         }
         print("Sending status update to Tracker...\n");
         tracker_send_status_update();
         print("Status update sent!\n");
         am_hal_delay_us(1000000);
      }

      // Test sending alerts
      for (int i = 0; i < 3; ++i)
      {
         print("Sending alert data (time = %u)...\n", ++current_time);
         const tracker_alert_data_t alert_data = { .msg_type = MSG_CRITICAL_ALERT, .utc_timestamp = current_time, .alert_type = ALERT_GUNSHOT };
         tracker_send_alert(&alert_data);
         am_hal_delay_us(1000000);
      }
   }

   // Should never reach this point
   return 0;
}
