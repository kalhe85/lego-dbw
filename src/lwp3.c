// lwp3.c — LEGO Wireless Protocol 3 message builders and parser.
// All multi-byte fields are little-endian.

#include "lwp3.h"

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

// Port Output Command header: [len][hub=0][0x81][port][startup][subcommand]
static void output_header(uint8_t *out, uint8_t len, uint8_t port, uint8_t subcmd) {
    out[0] = len;
    out[1] = 0x00;
    out[2] = 0x81;
    out[3] = port;
    out[4] = LWP3_STARTUP_IMMEDIATE;
    out[5] = subcmd;
}

// StartPower via WriteDirectModeData (0x51), mode 0. Power 0 = float, 127 = brake.
uint16_t lwp3_start_power(uint8_t port, int8_t power, uint8_t out[8]) {
    output_header(out, 8, port, 0x51);
    out[6] = LWP3_MODE_POWER;
    out[7] = (uint8_t)power;
    return 8;
}

// StartSpeed (0x07): [speed int8][max power][use profile]. Speed 0 = active hold.
uint16_t lwp3_start_speed(uint8_t port, int8_t speed, uint8_t max_power, uint8_t use_profile,
                          uint8_t out[9]) {
    output_header(out, 9, port, 0x07);
    out[6] = (uint8_t)speed;
    out[7] = max_power;
    out[8] = use_profile;
    return 9;
}

// GotoAbsolutePosition (0x0D): [abs pos int32][speed][max power][end state][use profile]
uint16_t lwp3_goto_abs(uint8_t port, int32_t pos, int8_t speed, uint8_t max_power,
                       uint8_t end_state, uint8_t use_profile, uint8_t out[14]) {
    output_header(out, 14, port, 0x0D);
    put_u32(&out[6], (uint32_t)pos);
    out[10] = (uint8_t)speed;
    out[11] = max_power;
    out[12] = end_state;
    out[13] = use_profile;
    return 14;
}

// SetAccTime (0x05): [time int16 ms][profile number]. Profile 0 is used by StartSpeed's
// acceleration bit.
uint16_t lwp3_set_acc_time(uint8_t port, uint16_t time_ms, uint8_t out[9]) {
    output_header(out, 9, port, 0x05);
    put_u16(&out[6], time_ms);
    out[8] = 0x00;
    return 9;
}

// Port Input Format Setup (Single) (0x41): [port][mode][delta uint32][notify]
uint16_t lwp3_input_format_setup(uint8_t port, uint8_t mode, uint32_t delta, bool notify,
                                 uint8_t out[10]) {
    out[0] = 10;
    out[1] = 0x00;
    out[2] = 0x41;
    out[3] = port;
    out[4] = mode;
    put_u32(&out[5], delta);
    out[9] = notify ? 1 : 0;
    return 10;
}

// Port Information Request (0x21), information type 0x00 = port value. The hub answers with a
// Port Value (0x45) message for the port's current input mode.
uint16_t lwp3_port_value_request(uint8_t port, uint8_t out[5]) {
    out[0] = 5; out[1] = 0x00; out[2] = 0x21; out[3] = port; out[4] = 0x00;
    return 5;
}

// Hub Properties (0x01): enable updates (0x02)
uint16_t lwp3_hub_prop_enable_updates(uint8_t prop, uint8_t out[5]) {
    out[0] = 5; out[1] = 0x00; out[2] = 0x01; out[3] = prop; out[4] = 0x02;
    return 5;
}

// Hub Alerts (0x03): enable updates (0x01)
uint16_t lwp3_hub_alert_enable(uint8_t alert, uint8_t out[5]) {
    out[0] = 5; out[1] = 0x00; out[2] = 0x03; out[3] = alert; out[4] = 0x01;
    return 5;
}

bool lwp3_parse(const uint8_t *v, uint16_t len, lwp3_msg_t *m) {
    m->type = LWP3_MSG_UNKNOWN;
    m->a = m->b = 0;
    m->value = 0;
    if (len < 3 || v[0] != len) return false;   // only short (1-byte length) messages

    switch (v[2]) {
        case 0x01: // Hub property update: [len][0][01][prop][06][payload]
            if (len >= 6 && v[3] == 0x06 && v[4] == 0x06) {
                m->type = LWP3_MSG_BATTERY;
                m->value = v[5];
                return true;
            }
            return false;

        case 0x03: // Hub alert: [len][0][03][type][04][00 ok / FF alert]
            if (len >= 6 && v[4] == 0x04) {
                m->type = LWP3_MSG_ALERT;
                m->a = v[3];
                m->value = v[5] ? 1 : 0;
                return true;
            }
            return false;

        case 0x04: // Attached I/O: [len][0][04][port][event][io type LE]
            if (len >= 5) {
                m->type = LWP3_MSG_ATTACHED_IO;
                m->a = v[3];
                m->b = v[4];
                m->value = (len >= 7) ? (int32_t)(v[5] | (v[6] << 8)) : 0;
                return true;
            }
            return false;

        case 0x05: // Generic error: [len][0][05][command type][error code]
            if (len >= 5) {
                m->type = LWP3_MSG_ERROR;
                m->a = v[3];
                m->b = v[4];
                return true;
            }
            return false;

        case 0x45: // Port value (single): [len][0][45][port][value: 1, 2 or 4 bytes LE]
            m->a = v[3];
            if (len == 5) {
                m->value = (int8_t)v[4];
            } else if (len == 6) {
                m->value = (int16_t)(v[4] | (v[5] << 8));
            } else if (len == 8) {
                m->value = (int32_t)((uint32_t)v[4] | ((uint32_t)v[5] << 8) |
                                     ((uint32_t)v[6] << 16) | ((uint32_t)v[7] << 24));
            } else {
                return false;
            }
            m->type = LWP3_MSG_PORT_VALUE;
            return true;

        case 0x47: // Port input format (single): [len][0][47][port][mode][delta u32][notify]
            if (len >= 10) {
                m->type = LWP3_MSG_INPUT_FORMAT;
                m->a = v[3];
                m->b = v[4];
                m->value = v[9];
                return true;
            }
            return false;

        default:
            return false;
    }
}

const char *lwp3_alert_name(uint8_t alert) {
    switch (alert) {
        case 0x01: return "LOW VOLTAGE";
        case 0x02: return "HIGH CURRENT";
        case 0x03: return "LOW SIGNAL";
        case 0x04: return "OVER POWER";
        default:   return "?";
    }
}
