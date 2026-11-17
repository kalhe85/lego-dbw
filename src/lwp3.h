// lwp3.h — LEGO Wireless Protocol 3 message builders and parser (no hardware dependencies).

#ifndef LWP3_H
#define LWP3_H

#include <stdint.h>
#include <stdbool.h>

#define LWP3_MAX_MSG 16

// Port Output Command (0x81) startup/completion byte: execute immediately, no feedback.
#define LWP3_STARTUP_IMMEDIATE  0x10

// Motor modes used here (Technic L/XL/angular motors)
#define LWP3_MODE_POWER  0
#define LWP3_MODE_POS    2     // relative position, int32 degrees

// UseProfile bits for StartSpeed / GotoAbsolutePosition
#define LWP3_PROFILE_ACC 0x01
#define LWP3_PROFILE_DEC 0x02

#define LWP3_POWER_FLOAT 0
#define LWP3_POWER_BRAKE 127

// Builders: write the message into out[] and return its length.
uint16_t lwp3_start_power(uint8_t port, int8_t power, uint8_t out[8]);
uint16_t lwp3_start_speed(uint8_t port, int8_t speed, uint8_t max_power, uint8_t use_profile,
                          uint8_t out[9]);
uint16_t lwp3_goto_abs(uint8_t port, int32_t pos, int8_t speed, uint8_t max_power,
                       uint8_t end_state, uint8_t use_profile, uint8_t out[14]);
uint16_t lwp3_set_acc_time(uint8_t port, uint16_t time_ms, uint8_t out[9]);
uint16_t lwp3_input_format_setup(uint8_t port, uint8_t mode, uint32_t delta, bool notify,
                                 uint8_t out[10]);
uint16_t lwp3_port_value_request(uint8_t port, uint8_t out[5]);
uint16_t lwp3_hub_prop_enable_updates(uint8_t prop, uint8_t out[5]);
uint16_t lwp3_hub_alert_enable(uint8_t alert, uint8_t out[5]);

// Parsed upstream message
typedef enum {
    LWP3_MSG_UNKNOWN = 0,
    LWP3_MSG_BATTERY,       // hub property 0x06 update: value = percent
    LWP3_MSG_ALERT,         // hub alert: a = alert type, value = 1 active / 0 cleared
    LWP3_MSG_ATTACHED_IO,   // a = port, b = event (0 detached, 1 attached, 2 virtual), value = io type
    LWP3_MSG_ERROR,         // a = command type, b = error code
    LWP3_MSG_PORT_VALUE,    // a = port, value = signed value (1, 2 or 4 bytes)
    LWP3_MSG_INPUT_FORMAT   // a = port, b = mode, value = notify enabled
} lwp3_msg_type_t;

typedef struct {
    lwp3_msg_type_t type;
    uint8_t a;
    uint8_t b;
    int32_t value;
} lwp3_msg_t;

// Returns false if the buffer is too short or malformed.
bool lwp3_parse(const uint8_t *v, uint16_t len, lwp3_msg_t *out);

const char *lwp3_alert_name(uint8_t alert);

#endif // LWP3_H
