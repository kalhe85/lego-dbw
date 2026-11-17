// test_logic.c — host-side unit tests for the hardware-independent modules.
// Build and run with tests/run_tests.sh.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drive.h"
#include "lwp3.h"
#include "mavlink_lite.h"
#include "steering.h"
#include "supervisor.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures = 0;
static int checks = 0;

#define CHECK(cond) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { checks++; long _a = (long)(a), _b = (long)(b); if (_a != _b) { \
    failures++; printf("FAIL %s:%d: %s == %ld, expected %ld\n", __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define CHECK_NEAR(a, b, tol) do { checks++; double _a = (a), _b = (b); if (fabs(_a - _b) > (tol)) { \
    failures++; printf("FAIL %s:%d: %s == %g, expected %g +/- %g\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); } } while (0)

static int bytes_eq(const uint8_t *a, const uint8_t *b, size_t n) { return memcmp(a, b, n) == 0; }

// ============================================================
// LWP3
// ============================================================

static void test_lwp3_builders(void) {
    uint8_t m[16];

    CHECK_EQ(lwp3_start_power(0x00, -100, m), 8);
    { const uint8_t e[] = {8, 0, 0x81, 0x00, 0x10, 0x51, 0x00, 0x9C}; CHECK(bytes_eq(m, e, 8)); }

    CHECK_EQ(lwp3_start_speed(0x01, -25, 35, LWP3_PROFILE_ACC, m), 9);
    { const uint8_t e[] = {9, 0, 0x81, 0x01, 0x10, 0x07, 0xE7, 35, 0x01}; CHECK(bytes_eq(m, e, 9)); }

    CHECK_EQ(lwp3_goto_abs(0x03, -240, 80, 80, 126, 0, m), 14);
    { const uint8_t e[] = {14, 0, 0x81, 0x03, 0x10, 0x0D, 0x10, 0xFF, 0xFF, 0xFF, 80, 80, 126, 0};
      CHECK(bytes_eq(m, e, 14)); }

    CHECK_EQ(lwp3_set_acc_time(0x00, 300, m), 9);
    { const uint8_t e[] = {9, 0, 0x81, 0x00, 0x10, 0x05, 0x2C, 0x01, 0x00}; CHECK(bytes_eq(m, e, 9)); }

    CHECK_EQ(lwp3_input_format_setup(0x03, LWP3_MODE_POS, 1, true, m), 10);
    { const uint8_t e[] = {10, 0, 0x41, 0x03, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01}; CHECK(bytes_eq(m, e, 10)); }

    CHECK_EQ(lwp3_input_format_setup(0x00, LWP3_MODE_POS, 30, true, m), 10);
    CHECK_EQ(m[5], 30);

    CHECK_EQ(lwp3_port_value_request(0x03, m), 5);
    { const uint8_t e[] = {5, 0, 0x21, 0x03, 0x00}; CHECK(bytes_eq(m, e, 5)); }
}

static void test_lwp3_parse(void) {
    lwp3_msg_t r;

    const uint8_t pos[] = {8, 0, 0x45, 0x03, 0x10, 0xFF, 0xFF, 0xFF};   // -240
    CHECK(lwp3_parse(pos, sizeof(pos), &r));
    CHECK_EQ(r.type, LWP3_MSG_PORT_VALUE);
    CHECK_EQ(r.a, 0x03);
    CHECK_EQ(r.value, -240);

    const uint8_t pos_big[] = {8, 0, 0x45, 0x00, 0x40, 0x42, 0x0F, 0x00};   // 1000000
    CHECK(lwp3_parse(pos_big, sizeof(pos_big), &r));
    CHECK_EQ(r.value, 1000000);

    const uint8_t spd8[] = {5, 0, 0x45, 0x00, 0xF6};   // int8 -10
    CHECK(lwp3_parse(spd8, sizeof(spd8), &r));
    CHECK_EQ(r.value, -10);

    const uint8_t batt[] = {6, 0, 0x01, 0x06, 0x06, 87};
    CHECK(lwp3_parse(batt, sizeof(batt), &r));
    CHECK_EQ(r.type, LWP3_MSG_BATTERY);
    CHECK_EQ(r.value, 87);

    const uint8_t alert[] = {6, 0, 0x03, 0x02, 0x04, 0xFF};
    CHECK(lwp3_parse(alert, sizeof(alert), &r));
    CHECK_EQ(r.type, LWP3_MSG_ALERT);
    CHECK_EQ(r.a, 0x02);
    CHECK_EQ(r.value, 1);

    const uint8_t io[] = {15, 0, 0x04, 0x03, 0x01, 0x2E, 0x00, 0, 0, 0, 0, 0, 0, 0, 0};
    CHECK(lwp3_parse(io, sizeof(io), &r));
    CHECK_EQ(r.type, LWP3_MSG_ATTACHED_IO);
    CHECK_EQ(r.value, 0x2E);

    const uint8_t fmt[] = {10, 0, 0x47, 0x03, 0x02, 0x01, 0, 0, 0, 0x01};
    CHECK(lwp3_parse(fmt, sizeof(fmt), &r));
    CHECK_EQ(r.type, LWP3_MSG_INPUT_FORMAT);
    CHECK_EQ(r.b, 2);

    const uint8_t bad_len[] = {9, 0, 0x45, 0x03, 0x10};       // length byte mismatch
    CHECK(!lwp3_parse(bad_len, sizeof(bad_len), &r));
    const uint8_t odd_val[] = {7, 0, 0x45, 0x03, 1, 2, 3};    // 3-byte value: invalid
    CHECK(!lwp3_parse(odd_val, sizeof(odd_val), &r));
    CHECK(!lwp3_parse(pos, 2, &r));
}

// ============================================================
// Steering mapping
// ============================================================

static steer_map_params_t base_map(void) {
    steer_map_params_t p = {
        .wheel_max_deg = 40.0f, .uncal_limit_deg = 240, .curvature_map = false,
        .speed_gain = false, .gain_min = 0.55f, .fade_start_dps = 300.0f, .fade_end_dps = 1200.0f,
    };
    return p;
}

static void test_steer_mapping(void) {
    steer_map_params_t p = base_map();

    // Uncalibrated, linear: same as v1.0
    CHECK_EQ(steer_target(&p, NULL, 1.0f, 0), 240);
    CHECK_EQ(steer_target(&p, NULL, -1.0f, 0), -240);
    CHECK_EQ(steer_target(&p, NULL, 0.5f, 0), 120);
    CHECK_EQ(steer_target(&p, NULL, 0.0f, 0), 0);
    CHECK_EQ(steer_target(&p, NULL, 3.0f, 0), 240);     // clamped

    // Calibration not DONE is ignored
    steer_cal_t cal;
    steer_cal_reset(&cal);
    cal.centre = 50; cal.limit = 180;
    CHECK_EQ(steer_target(&p, &cal, 1.0f, 0), 240);

    // Calibrated: around the measured centre, within the measured range
    cal.state = STEER_CAL_DONE;
    CHECK_EQ(steer_target(&p, &cal, 0.0f, 0), 50);
    CHECK_EQ(steer_target(&p, &cal, 1.0f, 0), 230);
    CHECK_EQ(steer_target(&p, &cal, -1.0f, 0), -130);

    // Curvature map: full stick still full lock; half stick = atan(0.5 tan 40deg) / 40deg
    p.curvature_map = true;
    double expect = atan(0.5 * tan(40.0 * M_PI / 180.0)) / (40.0 * M_PI / 180.0);
    CHECK_NEAR(steer_shape(&p, 0.5f), expect, 1e-4);
    CHECK_NEAR(steer_shape(&p, 1.0f), 1.0, 1e-5);
    CHECK_NEAR(steer_shape(&p, -0.5f), -expect, 1e-4);
    CHECK_EQ(steer_target(&p, NULL, 0.5f, 0), (long)lround(expect * 240));
    // Monotonic
    float prev = -2.0f;
    int mono = 1;
    for (int i = -100; i <= 100; i++) {
        float v = steer_shape(&p, i / 100.0f);
        if (v < prev) mono = 0;
        prev = v;
    }
    CHECK(mono);
    // Invalid wheel angle -> stays linear
    p.wheel_max_deg = 0.0f;
    CHECK_NEAR(steer_shape(&p, 0.5f), 0.5, 1e-6);
    p.wheel_max_deg = 40.0f;
    p.curvature_map = false;

    // Speed gain
    CHECK_NEAR(steer_gain(&p, 2000.0f), 1.0, 1e-6);   // disabled
    p.speed_gain = true;
    CHECK_NEAR(steer_gain(&p, 0.0f), 1.0, 1e-6);
    CHECK_NEAR(steer_gain(&p, 300.0f), 1.0, 1e-6);
    CHECK_NEAR(steer_gain(&p, 750.0f), 1.0 - 0.5 * 0.45, 1e-5);
    CHECK_NEAR(steer_gain(&p, -750.0f), 1.0 - 0.5 * 0.45, 1e-5);   // reverse too
    CHECK_NEAR(steer_gain(&p, 5000.0f), 0.55, 1e-6);
    CHECK_EQ(steer_target(&p, NULL, 1.0f, 5000.0f), 132);
}

// ============================================================
// Steering calibration (simulated motor)
// ============================================================

typedef struct {
    double pos;
    double lo, hi;          // mechanical end-stops
    double dps_per_pct;     // speed per % command
    double jitter;          // position noise amplitude at rest
    int    feedback;        // 0 = no position reports
    int    moves;           // 0 = too weak to move
} sim_motor_t;

static steer_cal_params_t cal_params(void) {
    steer_cal_params_t c = {
        .stall_deg = 3, .stall_ms = 250, .min_run_ms = 300, .sweep_timeout_ms = 4000,
        .feedback_timeout_ms = 1500, .min_half_deg = 40, .max_half_deg = 900, .margin_pct = 90,
    };
    return c;
}

// Runs the calibration against the simulated motor; returns the elapsed time in ms.
static uint32_t run_cal(steer_cal_t *cal, sim_motor_t *m) {
    steer_cal_params_t p = cal_params();
    uint32_t t = 1000;
    steer_cal_start(cal, t);
    steer_cal_cmd_t cmd = STEER_CAL_CMD_HOLD;
    int k = 0;
    while (steer_cal_busy(cal) && t < 60000) {
        t += 40;   // 25 Hz control tick
        double v = 0;
        if (cmd == STEER_CAL_CMD_RUN_NEG) v = -25 * m->dps_per_pct;
        if (cmd == STEER_CAL_CMD_RUN_POS) v = 25 * m->dps_per_pct;
        if (m->moves) m->pos += v * 0.040;
        if (m->pos < m->lo) m->pos = m->lo;
        if (m->pos > m->hi) m->pos = m->hi;
        double noise = (k++ % 2) ? m->jitter : -m->jitter;
        int32_t reported = (int32_t)lround(m->pos + noise);
        cmd = steer_cal_step(cal, &p, t, m->feedback != 0, reported);
    }
    return t - 1000;
}

static void test_steer_calibration(void) {
    steer_cal_t cal;

    // Asymmetric end-stops: -150 .. +250 -> centre 50, half 200, usable 180
    sim_motor_t m = { .pos = 0, .lo = -150, .hi = 250, .dps_per_pct = 10, .feedback = 1, .moves = 1 };
    uint32_t dur = run_cal(&cal, &m);
    CHECK_EQ(cal.state, STEER_CAL_DONE);
    CHECK_EQ(cal.end_neg, -150);
    CHECK_EQ(cal.end_pos, 250);
    CHECK_EQ(cal.centre, 50);
    CHECK_EQ(cal.limit, 180);
    CHECK(dur < 6000);
    printf("  calibration (asymmetric) took %u ms\n", (unsigned)dur);

    // Starting already at an end-stop, with +/-1 deg jitter
    m = (sim_motor_t){ .pos = -150, .lo = -150, .hi = 250, .dps_per_pct = 10, .jitter = 1,
                       .feedback = 1, .moves = 1 };
    run_cal(&cal, &m);
    CHECK_EQ(cal.state, STEER_CAL_DONE);
    CHECK_NEAR(cal.centre, 50, 2);

    // Reversed encoder direction is handled (end_neg ends up the larger value)
    m = (sim_motor_t){ .pos = 0, .lo = -150, .hi = 250, .dps_per_pct = -10, .feedback = 1, .moves = 1 };
    run_cal(&cal, &m);
    CHECK_EQ(cal.state, STEER_CAL_DONE);
    CHECK_EQ(cal.centre, 50);
    CHECK_EQ(cal.limit, 180);

    // No position feedback -> fails after the feedback timeout
    m = (sim_motor_t){ .pos = 0, .lo = -150, .hi = 250, .dps_per_pct = 10, .feedback = 0, .moves = 1 };
    dur = run_cal(&cal, &m);
    CHECK_EQ(cal.state, STEER_CAL_FAILED);
    CHECK(dur >= 1500 && dur < 2000);

    // Motor too weak to move -> range too small
    m = (sim_motor_t){ .pos = 0, .lo = -150, .hi = 250, .dps_per_pct = 10, .feedback = 1, .moves = 0 };
    run_cal(&cal, &m);
    CHECK_EQ(cal.state, STEER_CAL_FAILED);
    CHECK(cal.fail_reason && strstr(cal.fail_reason, "too small"));

    // No end-stop (free-spinning) -> sweep timeout
    m = (sim_motor_t){ .pos = 0, .lo = -1e9, .hi = 1e9, .dps_per_pct = 10, .feedback = 1, .moves = 1 };
    dur = run_cal(&cal, &m);
    CHECK_EQ(cal.state, STEER_CAL_FAILED);
    CHECK(dur > 4000 && dur < 5000);

    // Abort/reset returns to IDLE and not busy
    steer_cal_reset(&cal);
    CHECK_EQ(cal.state, STEER_CAL_IDLE);
    CHECK(!steer_cal_busy(&cal));
}

// ============================================================
// Supervisor: arming, calibration sequencing, drive interlock
// ============================================================

typedef struct {
    dbw_sup_t s;
    sim_motor_t m;
    steer_cal_params_t p;
    uint32_t t;
    steer_cal_cmd_t cmd;
} sup_rig_t;

static void rig_init(sup_rig_t *r, int feedback) {
    dbw_sup_init(&r->s);
    r->m = (sim_motor_t){ .pos = 0, .lo = -150, .hi = 250, .dps_per_pct = 10,
                          .feedback = feedback, .moves = 1 };
    r->p = cal_params();
    r->t = 5000;
    r->cmd = STEER_CAL_CMD_NONE;
}

// One 40 ms control tick. Returns the throttle the gateway would command.
static int32_t rig_tick(sup_rig_t *r, bool armed, bool link, int32_t thr, dbw_sup_out_t *o_out) {
    r->t += 40;
    double v = 0;
    if (r->cmd == STEER_CAL_CMD_RUN_NEG) v = -25 * r->m.dps_per_pct;
    if (r->cmd == STEER_CAL_CMD_RUN_POS) v = 25 * r->m.dps_per_pct;
    r->m.pos += v * 0.040;
    if (r->m.pos < r->m.lo) r->m.pos = r->m.lo;
    if (r->m.pos > r->m.hi) r->m.pos = r->m.hi;

    dbw_sup_in_t in = { .now_ms = r->t, .armed = armed, .link_ready = link, .hub_init_done = link,
                        .have_pos = r->m.feedback != 0, .pos = (int32_t)lround(r->m.pos),
                        .thr_cmd = thr };
    dbw_sup_out_t o;
    dbw_sup_step(&r->s, &r->p, &in, &o);
    r->cmd = o.cal_cmd;
    if (o_out) *o_out = o;
    return o.thr_cmd;
}

// Runs ticks with the given inputs until calibration is no longer busy (max 10 s).
// Returns the largest throttle output seen while it ran.
static int32_t rig_run_cal(sup_rig_t *r, int32_t thr_during) {
    int32_t max_thr = 0;
    for (int i = 0; i < 250 && steer_cal_busy(&r->s.cal); i++) {
        int32_t o = rig_tick(r, true, true, thr_during, NULL);
        if (o > max_thr) max_thr = o;
    }
    return max_thr;
}

static void test_supervisor(void) {
    sup_rig_t r;
    dbw_sup_out_t o;

    // 1. Disarmed: throttle always 0, nothing starts
    rig_init(&r, 1);
    for (int i = 0; i < 10; i++) CHECK_EQ(rig_tick(&r, false, true, 80, NULL), 0);
    CHECK_EQ(r.s.cal.state, STEER_CAL_IDLE);

    // 2. Arm at neutral -> calibration starts; throttle stays 0 during it even if pushed
    CHECK_EQ(rig_tick(&r, true, true, 0, &o), 0);
    CHECK(o.cal_started);
    CHECK(steer_cal_busy(&r.s.cal));
    CHECK_EQ(rig_run_cal(&r, 50), 0);
    CHECK_EQ(r.s.cal.state, STEER_CAL_DONE);
    CHECK_EQ(r.s.cal.centre, 50);
    // Done, but stick still held -> interlocked until neutral
    CHECK_EQ(rig_tick(&r, true, true, 50, NULL), 0);
    CHECK_EQ(rig_tick(&r, true, true, 0, NULL), 0);
    CHECK_EQ(rig_tick(&r, true, true, 60, NULL), 60);
    CHECK_EQ(rig_tick(&r, true, true, -30, NULL), -30);

    // 3. Disarm/re-arm after DONE: no new calibration, interlock until neutral
    CHECK_EQ(rig_tick(&r, false, true, 60, NULL), 0);
    CHECK_EQ(rig_tick(&r, true, true, 60, &o), 0);
    CHECK(!o.cal_started);
    CHECK_EQ(r.s.cal.state, STEER_CAL_DONE);
    CHECK_EQ(rig_tick(&r, true, true, 0, NULL), 0);
    CHECK_EQ(rig_tick(&r, true, true, 60, NULL), 60);

    // 4. Link lost while armed and driving: drive 0, calibration discarded, and it does NOT
    //    restart by itself when the link returns with the switch still on
    dbw_sup_link_lost(&r.s);
    CHECK_EQ(rig_tick(&r, true, false, 60, NULL), 0);
    for (int i = 0; i < 50; i++) {
        CHECK_EQ(rig_tick(&r, true, true, 0, &o), 0);
        CHECK(!o.cal_started);
    }
    CHECK_EQ(r.s.cal.state, STEER_CAL_IDLE);
    //    Operator flips enable off and on -> calibration runs, then drive is allowed
    rig_tick(&r, false, true, 0, NULL);
    rig_tick(&r, true, true, 0, &o);
    CHECK(o.cal_started);
    rig_run_cal(&r, 0);
    CHECK_EQ(r.s.cal.state, STEER_CAL_DONE);
    CHECK_EQ(rig_tick(&r, true, true, 0, NULL), 0);
    CHECK_EQ(rig_tick(&r, true, true, 40, NULL), 40);

    // 5. Arm with the throttle held: calibration waits for neutral, drive stays 0
    rig_init(&r, 1);
    for (int i = 0; i < 20; i++) {
        CHECK_EQ(rig_tick(&r, true, true, 40, &o), 0);
        CHECK(!o.cal_started);
    }
    CHECK_EQ(rig_tick(&r, true, true, 0, &o), 0);
    CHECK(o.cal_started);

    // 6. Disarm mid-calibration aborts it (motor command NONE) and re-arm restarts it
    for (int i = 0; i < 5; i++) rig_tick(&r, true, true, 0, NULL);
    CHECK(steer_cal_busy(&r.s.cal));
    rig_tick(&r, false, true, 0, &o);
    CHECK(o.cal_aborted);
    CHECK_EQ(o.cal_cmd, STEER_CAL_CMD_NONE);
    CHECK_EQ(r.s.cal.state, STEER_CAL_IDLE);
    rig_tick(&r, true, true, 0, &o);
    CHECK(o.cal_started);
    rig_run_cal(&r, 0);
    CHECK_EQ(r.s.cal.state, STEER_CAL_DONE);

    // 7. Calibration fails (no feedback): drive allowed after neutral with uncalibrated
    //    steering; disarm clears the failure and the next arm retries
    rig_init(&r, 0);
    rig_tick(&r, true, true, 0, &o);
    CHECK(o.cal_started);
    rig_run_cal(&r, 0);
    CHECK_EQ(r.s.cal.state, STEER_CAL_FAILED);
    CHECK_EQ(rig_tick(&r, true, true, 0, NULL), 0);
    CHECK_EQ(rig_tick(&r, true, true, 70, NULL), 70);
    rig_tick(&r, false, true, 0, NULL);
    CHECK_EQ(r.s.cal.state, STEER_CAL_IDLE);
    rig_tick(&r, true, true, 0, &o);
    CHECK(o.cal_started);

    // 8. Armed before the link is up: calibration starts once the hub is ready
    rig_init(&r, 1);
    for (int i = 0; i < 10; i++) CHECK_EQ(rig_tick(&r, true, false, 0, NULL), 0);
    rig_tick(&r, true, true, 0, &o);
    CHECK(o.cal_started);
}

// ============================================================
// Speed estimation
// ============================================================

static void test_speed_estimator(void) {
    speed_est_t e;
    speed_est_reset(&e);
    CHECK_NEAR(speed_est_get(&e, 0, 300), 0.0, 1e-9);

    // 600 deg/s, reports every 30 deg (50 ms) with +/-15 ms arrival jitter
    uint32_t t = 10000;
    int32_t pos = 0;
    const int jit[] = {0, 15, -10, 5, -15, 10, 0, -5};
    for (int i = 0; i < 40; i++) {
        pos += 30;
        uint32_t base = 10000 + (uint32_t)(i + 1) * 50;
        t = base + (uint32_t)jit[i % 8];
        speed_est_update(&e, pos, t, 100);
    }
    CHECK_NEAR(speed_est_get(&e, t, 300), 600.0, 120.0);

    // Reverse
    speed_est_reset(&e);
    t = 0; pos = 0;
    for (int i = 0; i < 20; i++) { pos -= 30; t += 25; speed_est_update(&e, pos, t, 100); }
    CHECK_NEAR(speed_est_get(&e, t, 300), -1200.0, 1.0);

    // No reports -> 0 after the timeout
    CHECK_NEAR(speed_est_get(&e, t + 301, 300), 0.0, 1e-9);

    // Restart after a long stop: no bogus low average over the idle time
    uint32_t t2 = t + 5000;
    speed_est_update(&e, pos - 30, t2, 100);
    CHECK_NEAR(speed_est_get(&e, t2, 300), 0.0, 1e-9);
    for (int i = 0; i < 10; i++) { pos -= 30; t2 += 25; speed_est_update(&e, pos - 30, t2, 100); }
    CHECK_NEAR(speed_est_get(&e, t2, 300), -1200.0, 1.0);
}

// ============================================================
// MAVLink: frames are written to a file and decoded with pymavlink by run_tests.sh
// ============================================================

static void test_mavlink(const char *out_path) {
    mavlite_t m;
    mavlite_init(&m, 1, 25);
    uint8_t buf[MAVLITE_MAX_FRAME];
    FILE *f = fopen(out_path, "wb");
    CHECK(f != NULL);
    if (!f) return;

    size_t n = mavlite_heartbeat(&m, buf, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0,
                                 MAV_STATE_ACTIVE);
    CHECK(n <= MAVLITE_MAX_FRAME);
    fwrite(buf, 1, n, f);
    n = mavlite_named_value_float(&m, buf, 123456, "DBW_SPEED", 612.5f);
    fwrite(buf, 1, n, f);
    n = mavlite_named_value_float(&m, buf, 0, "DBW_ARM", 0.0f);   // heavy zero truncation
    fwrite(buf, 1, n, f);
    n = mavlite_named_value_float(&m, buf, 7, "DBW_STRPOS", -137.0f);   // 10-char name
    fwrite(buf, 1, n, f);
    n = mavlite_statustext(&m, buf, MAV_SEVERITY_WARNING, "DBW: steer cal ok, centre 50 range +/-180");
    CHECK(n <= MAVLITE_MAX_FRAME);
    fwrite(buf, 1, n, f);
    char longtext[80];
    memset(longtext, 'x', sizeof(longtext) - 1);
    longtext[79] = 0;
    n = mavlite_statustext(&m, buf, MAV_SEVERITY_INFO, longtext);   // truncated to 50
    CHECK(n <= MAVLITE_MAX_FRAME);
    fwrite(buf, 1, n, f);
    fclose(f);

    // Known CRC check value of CRC-16/MCRF4XX for "123456789" is 0x6F91
    CHECK_EQ(mavlite_crc((const uint8_t *)"123456789", 9, 0xFFFF), 0x6F91);
}

int main(int argc, char **argv) {
    test_lwp3_builders();
    test_lwp3_parse();
    test_steer_mapping();
    test_steer_calibration();
    test_supervisor();
    test_speed_estimator();
    test_mavlink(argc > 1 ? argv[1] : "mavlink_frames.bin");

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
