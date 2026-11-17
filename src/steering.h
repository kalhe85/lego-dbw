// steering.h — steering mapping and automatic end-stop calibration (no hardware dependencies).
//
// Coordinates: all positions are steering-motor degrees in the hub's encoder frame (mode 2 POS,
// zero = motor position when the hub was switched on). GotoAbsolutePosition uses the same frame.

#ifndef STEERING_H
#define STEERING_H

#include <stdint.h>
#include <stdbool.h>

// ---------------- Mapping ----------------

typedef struct {
    float   wheel_max_deg;     // wheel angle at full lock (only the curvature map uses it)
    int32_t uncal_limit_deg;   // motor-degree limit when no calibration is available
    bool    curvature_map;     // stick -> path curvature instead of stick -> wheel angle
    bool    speed_gain;        // reduce steering authority with speed
    float   gain_min;          // 0..1, authority at and above fade_end_dps
    float   fade_start_dps;
    float   fade_end_dps;
} steer_map_params_t;

// ---------------- Calibration ----------------

typedef struct {
    int32_t  stall_deg;
    uint32_t stall_ms;
    uint32_t min_run_ms;
    uint32_t sweep_timeout_ms;
    uint32_t feedback_timeout_ms;
    int32_t  min_half_deg;
    int32_t  max_half_deg;
    int32_t  margin_pct;
} steer_cal_params_t;

typedef enum {
    STEER_CAL_IDLE = 0,
    STEER_CAL_WAIT_POS,     // waiting for the first position report
    STEER_CAL_SWEEP_NEG,    // driving towards the negative end-stop
    STEER_CAL_SWEEP_POS,    // driving towards the positive end-stop
    STEER_CAL_DONE,
    STEER_CAL_FAILED
} steer_cal_state_t;

typedef struct {
    steer_cal_state_t state;
    uint32_t state_ms;
    int32_t  anchor_pos;
    uint32_t anchor_ms;
    int32_t  end_neg;
    int32_t  end_pos;
    int32_t  centre;       // valid when DONE
    int32_t  limit;        // valid when DONE: usable +/- range around centre
    const char *fail_reason;
} steer_cal_t;

// What the steering motor should do during calibration.
typedef enum {
    STEER_CAL_CMD_NONE = 0,     // calibration not running: use normal steering control
    STEER_CAL_CMD_RUN_NEG,      // run at -sweep speed (low power)
    STEER_CAL_CMD_RUN_POS,      // run at +sweep speed (low power)
    STEER_CAL_CMD_HOLD          // waiting for feedback: keep the motor still
} steer_cal_cmd_t;

void steer_cal_reset(steer_cal_t *c);
void steer_cal_start(steer_cal_t *c, uint32_t now_ms);
bool steer_cal_busy(const steer_cal_t *c);
steer_cal_cmd_t steer_cal_step(steer_cal_t *c, const steer_cal_params_t *p, uint32_t now_ms,
                               bool have_pos, int32_t pos);
const char *steer_cal_state_name(steer_cal_state_t s);

// ---------------- Mapping functions ----------------

// Steering authority factor for a given drive speed (deg/s, sign ignored). 1.0 when disabled.
float steer_gain(const steer_map_params_t *p, float speed_dps);

// Normalised stick [-1..1] -> normalised wheel angle [-1..1].
float steer_shape(const steer_map_params_t *p, float s);

// Stick [-1..1] -> absolute steering-motor target. Uses the calibration when it is DONE,
// otherwise +/- uncal_limit_deg around the power-on zero.
int32_t steer_target(const steer_map_params_t *p, const steer_cal_t *cal, float stick,
                     float speed_dps);

#endif // STEERING_H
