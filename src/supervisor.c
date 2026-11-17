// supervisor.c — arming, steering-calibration sequencing and drive interlock.

#include "supervisor.h"

void dbw_sup_init(dbw_sup_t *s) {
    steer_cal_reset(&s->cal);
    s->cal_requested = false;
    s->prev_armed = false;
    s->drive_interlock = true;
}

void dbw_sup_link_lost(dbw_sup_t *s) {
    steer_cal_reset(&s->cal);
    s->cal_requested = false;
    s->drive_interlock = true;
    // prev_armed is kept: if the enable switch stays on through the link loss, there is no new
    // off->on edge, so the operator must flip it to recalibrate.
}

void dbw_sup_step(dbw_sup_t *s, const steer_cal_params_t *p, const dbw_sup_in_t *in,
                  dbw_sup_out_t *out) {
    out->cal_cmd = STEER_CAL_CMD_NONE;
    out->prev_cal_state = s->cal.state;
    out->cal_started = false;
    out->cal_aborted = false;

    // Arming edge
    if (in->armed && !s->prev_armed) s->cal_requested = true;
    if (!in->armed) s->cal_requested = false;
    s->prev_armed = in->armed;

    // Calibration sequencing
    if (!in->armed) {
        if (steer_cal_busy(&s->cal)) {
            steer_cal_reset(&s->cal);
            out->cal_aborted = true;
        } else if (s->cal.state == STEER_CAL_FAILED) {
            steer_cal_reset(&s->cal);          // retry on the next arm
        }
    } else if (in->link_ready && in->hub_init_done) {
        if (s->cal.state == STEER_CAL_IDLE && s->cal_requested && in->thr_cmd == 0) {
            s->cal_requested = false;
            steer_cal_start(&s->cal, in->now_ms);
            out->cal_started = true;
        }
        if (steer_cal_busy(&s->cal)) {
            out->cal_cmd = steer_cal_step(&s->cal, p, in->now_ms, in->have_pos, in->pos);
        }
    }

    // Drive interlock
    bool settled = (s->cal.state == STEER_CAL_DONE || s->cal.state == STEER_CAL_FAILED);
    if (!in->armed || !settled) s->drive_interlock = true;
    else if (s->drive_interlock && in->thr_cmd == 0) s->drive_interlock = false;

    out->thr_cmd = s->drive_interlock ? 0 : in->thr_cmd;
}
