#ifndef __HENRIK_HEADER_H__
#define __HENRIK_HEADER_H__

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
} henrik_msg_t;

typedef enum {
   ALERT_GUNSHOT = 0x01,
} henrik_alert_msg_t;

typedef struct __attribute__((packed)) { uint8_t msg_type, current_gps_data; uint32_t utc_timestamp; float lat, lon, height; uint8_t alert_type; } henrik_alert_data_t;
typedef struct __attribute__((packed)) { uint8_t msg_type; uint32_t utc_timestamp; uint32_t battery_level, storage_remaining; } henrik_status_data_t;
typedef struct __attribute__((packed)) { uint8_t msg_type; } henrik_config_data_t;
typedef struct __attribute__((packed)) { uint8_t msg_type; uint32_t utc_timestamp; float lat, lon, height; } henrik_gps_data_t;
typedef void (*henrik_data_callback_t)(henrik_msg_t message_type, const void *new_data);


// Public API Functions ------------------------------------------------------------------------------------------------

void henrik_init(void);
void henrik_deinit(void);
void henrik_register_data_callback(henrik_data_callback_t callback);
uint32_t henrik_get_current_time(void);
void henrik_update_status_data(uint32_t timestamp);
void henrik_send_alert(const henrik_alert_data_t *alert);
void henrik_send_status_update(void);

#endif  // #ifndef __HENRIK_HEADER_H__
