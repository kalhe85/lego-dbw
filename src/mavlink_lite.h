// mavlink_lite.h — minimal MAVLink 2 encoder for the few messages the gateway sends
// (no hardware dependencies). Frames are verified against pymavlink in tests/.

#ifndef MAVLINK_LITE_H
#define MAVLINK_LITE_H

#include <stdint.h>
#include <stddef.h>

#define MAVLITE_MAX_FRAME   (10 + 54 + 2)    // header + largest payload (STATUSTEXT) + CRC

#define MAV_TYPE_ONBOARD_CONTROLLER 18
#define MAV_AUTOPILOT_INVALID        8
#define MAV_STATE_ACTIVE             4

#define MAV_SEVERITY_CRITICAL 2
#define MAV_SEVERITY_ERROR    3
#define MAV_SEVERITY_WARNING  4
#define MAV_SEVERITY_NOTICE   5
#define MAV_SEVERITY_INFO     6

typedef struct {
    uint8_t sysid;
    uint8_t compid;
    uint8_t seq;
} mavlite_t;

void mavlite_init(mavlite_t *m, uint8_t sysid, uint8_t compid);

// Each function writes one complete frame into buf (at least MAVLITE_MAX_FRAME bytes) and
// returns its length.
size_t mavlite_heartbeat(mavlite_t *m, uint8_t *buf, uint8_t type, uint8_t autopilot,
                         uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
size_t mavlite_named_value_float(mavlite_t *m, uint8_t *buf, uint32_t time_boot_ms,
                                 const char *name, float value);
size_t mavlite_statustext(mavlite_t *m, uint8_t *buf, uint8_t severity, const char *text);

uint16_t mavlite_crc(const uint8_t *data, size_t len, uint16_t crc);

#endif // MAVLINK_LITE_H
