// supervisor.h — arming, steering-calibration sequencing and drive interlock
// (no hardware dependencies; used when FEAT_STEER_AUTOCAL is on).
//
// Rules:
//  * Calibration starts only after an enable off->on edge, once the hub is initialised and the
//    throttle is at neutral. It never starts by itself after a reconnect.
//  * Disarming aborts a running calibration; a failed calibration is retried on the next arm.
//  * The drive is held at 0 while disarmed, until calibration is DONE (or FAILED, which falls
//    back to uncalibrated steering), and then until the throttle has been seen at neutral.
//  * Losing the BLE link discards the calibration (the hub may have reset its encoder zero).

#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>

#include "steering.h"

typedef struct {
    steer_cal_t cal;
    bool cal_requested;
    bool prev_armed;
    bool drive_interlock;
} dbw_sup_t;

typedef struct {
    uint32_t now_ms;
    bool     armed;          // enable high and all PWM channels fresh
    bool     link_ready;     // BLE READY
    bool     hub_init_done;  // subscriptions sent
    bool     have_pos;       // steering position known
    int32_t  pos;            // steering position (motor degrees)
    int32_t  thr_cmd;        // throttle command before the interlock
} dbw_sup_in_t;

typedef struct {
    steer_cal_cmd_t   cal_cmd;         // motor command while calibrating, else NONE
    int32_t           thr_cmd;         // throttle after the interlock
    steer_cal_state_t prev_cal_state;  // calibration state before this step (for logging)
    bool              cal_started;
    bool              cal_aborted;
} dbw_sup_out_t;

void dbw_sup_init(dbw_sup_t *s);
void dbw_sup_link_lost(dbw_sup_t *s);
void dbw_sup_step(dbw_sup_t *s, const steer_cal_params_t *p, const dbw_sup_in_t *in,
                  dbw_sup_out_t *out);

#endif // SUPERVISOR_H
