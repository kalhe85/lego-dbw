// mavlink_lite.c — minimal MAVLink 2 encoder.
//
// Frame: STX(0xFD) LEN INCOMPAT COMPAT SEQ SYSID COMPID MSGID(3, LE) PAYLOAD CRC(2, LE)
// CRC: X.25 (CRC-16/MCRF4XX) over LEN..PAYLOAD, then the message's CRC_EXTRA byte.
// Payload fields are little-endian and ordered by size (largest first), as MAVLink requires.
// Trailing zero bytes of the payload are truncated (MAVLink 2 rule), keeping at least one byte.

#include "mavlink_lite.h"

#include <string.h>

#define MSG_HEARTBEAT          0
#define MSG_NAMED_VALUE_FLOAT  251
#define MSG_STATUSTEXT         253

#define CRCX_HEARTBEAT         50
#define CRCX_NAMED_VALUE_FLOAT 170
#define CRCX_STATUSTEXT        83

void mavlite_init(mavlite_t *m, uint8_t sysid, uint8_t compid) {
    m->sysid = sysid;
    m->compid = compid;
    m->seq = 0;
}

uint16_t mavlite_crc(const uint8_t *data, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; i++) {
        uint8_t tmp = data[i] ^ (uint8_t)(crc & 0xFF);
        tmp ^= (uint8_t)(tmp << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4));
    }
    return crc;
}

static size_t finish_frame(mavlite_t *m, uint8_t *buf, uint32_t msgid, const uint8_t *payload,
                           size_t len, uint8_t crc_extra) {
    while (len > 1 && payload[len - 1] == 0) len--;

    buf[0] = 0xFD;
    buf[1] = (uint8_t)len;
    buf[2] = 0;                 // incompat flags (no signing)
    buf[3] = 0;                 // compat flags
    buf[4] = m->seq++;
    buf[5] = m->sysid;
    buf[6] = m->compid;
    buf[7] = (uint8_t)(msgid & 0xFF);
    buf[8] = (uint8_t)((msgid >> 8) & 0xFF);
    buf[9] = (uint8_t)((msgid >> 16) & 0xFF);
    memcpy(&buf[10], payload, len);

    uint16_t crc = mavlite_crc(&buf[1], 9 + len, 0xFFFF);
    crc = mavlite_crc(&crc_extra, 1, crc);
    buf[10 + len] = (uint8_t)(crc & 0xFF);
    buf[11 + len] = (uint8_t)(crc >> 8);
    return 12 + len;
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

size_t mavlite_heartbeat(mavlite_t *m, uint8_t *buf, uint8_t type, uint8_t autopilot,
                         uint8_t base_mode, uint32_t custom_mode, uint8_t system_status) {
    // custom_mode u32, type, autopilot, base_mode, system_status, mavlink_version
    uint8_t p[9];
    put_u32(&p[0], custom_mode);
    p[4] = type;
    p[5] = autopilot;
    p[6] = base_mode;
    p[7] = system_status;
    p[8] = 3;   // mavlink_version
    return finish_frame(m, buf, MSG_HEARTBEAT, p, sizeof(p), CRCX_HEARTBEAT);
}

size_t mavlite_named_value_float(mavlite_t *m, uint8_t *buf, uint32_t time_boot_ms,
                                 const char *name, float value) {
    // time_boot_ms u32, value float, name char[10]
    uint8_t p[18];
    memset(p, 0, sizeof(p));
    put_u32(&p[0], time_boot_ms);
    uint32_t fbits;
    memcpy(&fbits, &value, sizeof(fbits));
    put_u32(&p[4], fbits);
    size_t n = strlen(name);
    memcpy(&p[8], name, n > 10 ? 10 : n);
    return finish_frame(m, buf, MSG_NAMED_VALUE_FLOAT, p, sizeof(p), CRCX_NAMED_VALUE_FLOAT);
}

size_t mavlite_statustext(mavlite_t *m, uint8_t *buf, uint8_t severity, const char *text) {
    // severity u8, text char[50], (extensions) id u16, chunk_seq u8 -> left zero
    uint8_t p[54];
    memset(p, 0, sizeof(p));
    p[0] = severity;
    size_t n = strlen(text);
    memcpy(&p[1], text, n > 50 ? 50 : n);
    return finish_frame(m, buf, MSG_STATUSTEXT, p, sizeof(p), CRCX_STATUSTEXT);
}
