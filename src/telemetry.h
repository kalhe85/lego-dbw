// telemetry.h — MAVLink telemetry to the flight controller over UART0 (TX only).
//
// The gateway appears as its own MAVLink component (TELEM_SYSID / TELEM_COMPID). ArduPilot
// forwards its broadcast messages to the ground station, so the values show up in Mission
// Planner's MAVLink Inspector and the status messages in the Messages tab.

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool    link_ready;     // BLE link to the hub is READY
    bool    armed;          // gateway armed (enable high, signals fresh)
    int32_t thr_cmd;        // throttle command [-100..100]
    int32_t steer_rel_deg;  // steering target relative to centre (motor degrees)
    int32_t steer_pos_deg;  // measured steering position relative to centre, or INT32_MIN
    float   speed_dps;      // measured drive-motor speed
    int32_t battery_pct;    // hub battery, -1 if unknown
    int32_t cal_state;      // steer_cal_state_t
    uint32_t tx_drop;       // BLE writes rejected
} telem_snapshot_t;

void telem_init(void);
// Call once per control tick: drains the TX buffer and schedules periodic messages.
void telem_tick(uint32_t now_ms, const telem_snapshot_t *s);
// Queue a STATUSTEXT (prefixed with "DBW: ").
void telem_event(uint8_t severity, const char *text);

#endif // TELEMETRY_H
