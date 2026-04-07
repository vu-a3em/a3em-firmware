#ifndef __TRACKER_HEADER_H__
#define __TRACKER_HEADER_H__

// Header Inclusions ---------------------------------------------------------------------------------------------------

#include "runtime_config.h"


// Peripheral Type Definitions -----------------------------------------------------------------------------------------

typedef enum {
   MSG_CRITICAL_ALERT = 0x01,
   MSG_ALERT = 0x02,
   MSG_STATUS = 0x03,
   MSG_CONFIG = 0x04,
   MSG_GPS = 0x05,
   MSG_GPS_REQUEST = 0x06,
   MSG_STATUS_REQUEST = 0x07
} tracker_msg_t;

typedef enum {
   ALERT_GUNSHOT = 0x01,
} tracker_alert_msg_t;

typedef struct __attribute__((packed)) { uint8_t msg_type, current_gps_data; uint32_t utc_timestamp; float lat, lon, height; uint8_t alert_type; } tracker_alert_data_t;
typedef struct __attribute__((packed)) { uint8_t msg_type; uint32_t utc_timestamp; uint32_t battery_level, storage_remaining; } tracker_status_data_t;
typedef struct __attribute__((packed)) { uint8_t msg_type; } tracker_config_data_t;
typedef struct __attribute__((packed)) { uint8_t msg_type; uint32_t utc_timestamp; float lat, lon, height; } tracker_gps_data_t;
typedef void (*tracker_data_callback_t)(tracker_msg_t message_type, const void *new_data);


// Public API Functions ------------------------------------------------------------------------------------------------

void tracker_init(void);
void tracker_deinit(void);
void tracker_register_data_callback(tracker_data_callback_t callback);
uint32_t tracker_get_current_time(void);
void tracker_update_status_data(uint32_t timestamp);
void tracker_send_alert(const tracker_alert_data_t *alert);
void tracker_send_status_update(void);

#endif  // #ifndef __TRACKER_HEADER_H__
