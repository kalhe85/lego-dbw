// steering.c — steering mapping and automatic end-stop calibration.

#include "steering.h"

#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
// Calibration
// ============================================================
//
// The motor is driven at low speed and low torque into one end-stop, then the other. An end-stop
// is detected when the position stops changing (less than stall_deg for stall_ms). The centre is
// the midpoint of the two end positions, and the usable range is margin_pct of the half-span, so
// normal steering never pushes into the mechanical stop.

void steer_cal_reset(steer_cal_t *c) {
    c->state = STEER_CAL_IDLE;
    c->state_ms = 0;
    c->anchor_pos = 0;
    c->anchor_ms = 0;
    c->end_neg = 0;
    c->end_pos = 0;
    c->centre = 0;
    c->limit = 0;
    c->fail_reason = 0;
}

static void enter(steer_cal_t *c, steer_cal_state_t s, uint32_t now_ms, int32_t pos) {
    c->state = s;
    c->state_ms = now_ms;
    c->anchor_pos = pos;
    c->anchor_ms = now_ms;
}

static void fail(steer_cal_t *c, uint32_t now_ms, const char *why) {
    c->state = STEER_CAL_FAILED;
    c->state_ms = now_ms;
    c->fail_reason = why;
}

void steer_cal_start(steer_cal_t *c, uint32_t now_ms) {
    steer_cal_reset(c);
    enter(c, STEER_CAL_WAIT_POS, now_ms, 0);
}

bool steer_cal_busy(const steer_cal_t *c) {
    return c->state == STEER_CAL_WAIT_POS ||
           c->state == STEER_CAL_SWEEP_NEG ||
           c->state == STEER_CAL_SWEEP_POS;
}

static void finish(steer_cal_t *c, const steer_cal_params_t *p, uint32_t now_ms) {
    int32_t lo = c->end_neg < c->end_pos ? c->end_neg : c->end_pos;
    int32_t hi = c->end_neg < c->end_pos ? c->end_pos : c->end_neg;
    int32_t half = (hi - lo) / 2;

    if (half < p->min_half_deg) {
        fail(c, now_ms, "range too small (motor not moving? raise STEER_CAL_POWER)");
        return;
    }
    if (half > p->max_half_deg) {
        fail(c, now_ms, "range too large (no end-stop?)");
        return;
    }
    c->centre = lo + half;
    c->limit = (half * p->margin_pct) / 100;
    c->state = STEER_CAL_DONE;
    c->state_ms = now_ms;
}

steer_cal_cmd_t steer_cal_step(steer_cal_t *c, const steer_cal_params_t *p, uint32_t now_ms,
                               bool have_pos, int32_t pos) {
    switch (c->state) {
        case STEER_CAL_WAIT_POS:
            if (have_pos) {
                enter(c, STEER_CAL_SWEEP_NEG, now_ms, pos);
                return STEER_CAL_CMD_RUN_NEG;
            }
            if ((now_ms - c->state_ms) > p->feedback_timeout_ms) {
                fail(c, now_ms, "no position feedback from steering motor");
                return STEER_CAL_CMD_NONE;
            }
            return STEER_CAL_CMD_HOLD;

        case STEER_CAL_SWEEP_NEG:
        case STEER_CAL_SWEEP_POS: {
            bool neg = (c->state == STEER_CAL_SWEEP_NEG);

            if (have_pos && abs(pos - c->anchor_pos) >= p->stall_deg) {
                c->anchor_pos = pos;
                c->anchor_ms = now_ms;
            }

            uint32_t elapsed = now_ms - c->state_ms;
            bool stalled = elapsed >= p->min_run_ms && (now_ms - c->anchor_ms) >= p->stall_ms;

            if (stalled) {
                if (neg) {
                    c->end_neg = pos;
                    enter(c, STEER_CAL_SWEEP_POS, now_ms, pos);
                    return STEER_CAL_CMD_RUN_POS;
                }
                c->end_pos = pos;
                finish(c, p, now_ms);
                return STEER_CAL_CMD_NONE;
            }
            if (elapsed > p->sweep_timeout_ms) {
                fail(c, now_ms, "no end-stop found before timeout");
                return STEER_CAL_CMD_NONE;
            }
            return neg ? STEER_CAL_CMD_RUN_NEG : STEER_CAL_CMD_RUN_POS;
        }

        default:
            return STEER_CAL_CMD_NONE;
    }
}

const char *steer_cal_state_name(steer_cal_state_t s) {
    switch (s) {
        case STEER_CAL_IDLE:      return "idle";
        case STEER_CAL_WAIT_POS:  return "wait-pos";
        case STEER_CAL_SWEEP_NEG: return "sweep-";
        case STEER_CAL_SWEEP_POS: return "sweep+";
        case STEER_CAL_DONE:      return "done";
        case STEER_CAL_FAILED:    return "failed";
        default:                  return "?";
    }
}

// ============================================================
// Mapping
// ============================================================

float steer_gain(const steer_map_params_t *p, float speed_dps) {
    if (!p->speed_gain) return 1.0f;
    float v = fabsf(speed_dps);
    if (v <= p->fade_start_dps) return 1.0f;
    if (v >= p->fade_end_dps || p->fade_end_dps <= p->fade_start_dps) return p->gain_min;
    float t = (v - p->fade_start_dps) / (p->fade_end_dps - p->fade_start_dps);
    return 1.0f - t * (1.0f - p->gain_min);
}

// Curvature-linear mapping (bicycle model): the stick commands path curvature
// k = s * k_max, with k_max = tan(d_max) / L. The wheel angle for curvature k is
// d = atan(L * k) = atan(s * tan(d_max)); the wheelbase L cancels out. The result is normalised
// by d_max so that full stick still means full lock.
//
// Note: true Ackermann geometry (inner wheel steering more than the outer one) is set by the
// mechanical linkage; one steering motor cannot change it. What this mapping does is make the
// turning radius respond linearly to the stick.
float steer_shape(const steer_map_params_t *p, float s) {
    s = clampf(s, -1.0f, 1.0f);
    if (!p->curvature_map) return s;
    float dmax = p->wheel_max_deg * (float)M_PI / 180.0f;
    if (dmax <= 0.0f || dmax >= 1.5f) return s;   // invalid parameter: stay linear
    return atanf(s * tanf(dmax)) / dmax;
}

int32_t steer_target(const steer_map_params_t *p, const steer_cal_t *cal, float stick,
                     float speed_dps) {
    float s = clampf(stick, -1.0f, 1.0f) * steer_gain(p, speed_dps);
    float u = steer_shape(p, s);

    if (cal && cal->state == STEER_CAL_DONE) {
        return cal->centre + (int32_t)lroundf(u * (float)cal->limit);
    }
    return (int32_t)lroundf(u * (float)p->uncal_limit_deg);
}
