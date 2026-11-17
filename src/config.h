// config.h — all user-tunable settings for the LEGO DBW gateway.
//
// Feature stages
// --------------
// New features are switched on in stages so they can be tested on the car one at a time
// (see docs/test-plan.md). Select the stage at configure time:
//
//     cmake -S . -B build -DDBW_STAGE=3
//
//   0  v1.0 behaviour (GPIO-IRQ PWM capture, open-loop drive, manual steering centre)
//   1  + PIO-based PWM capture
//   2  + MAVLink telemetry to ArduPilot over UART0 (GP0)
//   3  + motor position feedback (steering angle, drive speed) — read-only
//   4  + automatic steering calibration (end-stop sweep) and throttle interlock
//   5  + curvature-linear steering mapping (Ackermann / bicycle model)
//   6  + closed-loop drive speed (hub speed controller, acceleration ramp)
//   7  + speed-dependent steering gain
//
// Any FEAT_* below can also be forced individually with -DFEAT_xxx=0/1.

#ifndef DBW_CONFIG_H
#define DBW_CONFIG_H

#define DBW_VERSION "v1.1-dev"

#ifndef DBW_STAGE
#define DBW_STAGE 1
#endif

#ifndef FEAT_PIO_CAPTURE
#define FEAT_PIO_CAPTURE      (DBW_STAGE >= 1)
#endif
#ifndef FEAT_TELEMETRY
#define FEAT_TELEMETRY        (DBW_STAGE >= 2)
#endif
#ifndef FEAT_MOTOR_FEEDBACK
#define FEAT_MOTOR_FEEDBACK   (DBW_STAGE >= 3)
#endif
#ifndef FEAT_STEER_AUTOCAL
#define FEAT_STEER_AUTOCAL    (DBW_STAGE >= 4)
#endif
#ifndef FEAT_STEER_CURVATURE
#define FEAT_STEER_CURVATURE  (DBW_STAGE >= 5)
#endif
#ifndef FEAT_DRIVE_SPEED_LOOP
#define FEAT_DRIVE_SPEED_LOOP (DBW_STAGE >= 6)
#endif
#ifndef FEAT_STEER_SPEED_GAIN
#define FEAT_STEER_SPEED_GAIN (DBW_STAGE >= 7)
#endif

#if FEAT_STEER_AUTOCAL && !FEAT_MOTOR_FEEDBACK
#error "FEAT_STEER_AUTOCAL needs FEAT_MOTOR_FEEDBACK (steering position)"
#endif
#if FEAT_STEER_SPEED_GAIN && !FEAT_MOTOR_FEEDBACK
#error "FEAT_STEER_SPEED_GAIN needs FEAT_MOTOR_FEEDBACK (measured speed)"
#endif

// ============================================================
// Hub
// ============================================================

// MAC of your Technic Hub, as printed by nRF Connect: 90:84:2B:7F:D4:40
#define HUB_ADDR_BYTES  { 0x90, 0x84, 0x2B, 0x7F, 0xD4, 0x40 }

#define LEGO_PORT_A  0x00     // drive motor
#define LEGO_PORT_B  0x01     // drive motor
#define LEGO_PORT_D  0x03     // steering motor

// ============================================================
// PWM input (Matek -> Pico)
// ============================================================

#define GPIO_PWM_THROTTLE   2
#define GPIO_PWM_STEERING   3
#define GPIO_PWM_ENABLE     4

#define PWM_MIN_US          1000
#define PWM_MAX_US          2000
#define PWM_NEUTRAL_US      1500
#define PWM_DEADBAND_US     30
#define PWM_VALID_MIN_US    800     // pulses outside this window are treated as noise
#define PWM_VALID_MAX_US    2200

#define PWM_TIMEOUT_MS      250     // channel counts as lost after this
#define ENABLE_THRESHOLD_US 1600    // enable channel must be above this to arm

// ============================================================
// Control loop
// ============================================================

#define CONTROL_HZ          25
#define CONTROL_PERIOD_MS   (1000 / CONTROL_HZ)
#define OUTPUT_REFRESH_MS   300     // resend unchanged commands at this period

// ============================================================
// Drive
// ============================================================

#define THROTTLE_MAX_POWER  100     // open loop: max power [0..100]

// Closed loop (FEAT_DRIVE_SPEED_LOOP): throttle maps to a speed set-point in % of the
// motor's max speed; the hub's own speed controller holds it under load.
#define DRIVE_SPEED_MAX_PCT 100     // speed set-point at full throttle [0..100]
#define DRIVE_MAX_POWER     100     // torque limit for the speed controller [0..100]
#define DRIVE_ACCEL_MS      300     // 0..100 % ramp time; 0 disables. Deceleration is never
                                    // ramped, so stops stay immediate.

// Speed measurement (FEAT_MOTOR_FEEDBACK): position notifications from drive motor A.
#define DRIVE_POS_DELTA_DEG 30      // hub reports every N motor degrees (limits BLE traffic)
#define SPEED_WINDOW_MS     100     // speed is averaged over at least this window
#define SPEED_TIMEOUT_MS    300     // no notification for this long -> speed = 0

// ============================================================
// Steering
// ============================================================

#define STEER_TARGET_DEG_MAX   40   // wheel angle at full lock (estimate, degrees)
#define STEER_GEAR_RATIO       6    // motor degrees per wheel degree (measured ~6:1)
#define STEER_MOTOR_DEG_LIMIT  240  // uncalibrated limit (motor degrees); used when
                                    // auto-calibration is off or failed
#define STEER_RESEND_DEG       3    // ignore target changes smaller than this (motor deg)
#define STEER_GOTO_SPEED       80   // 1..100
#define STEER_GOTO_MAXPOWER    80   // 0..100 (100 draws high stall current at the end-stop)
#define STEER_ENDSTATE         126  // 126=HOLD, 127=BRAKE, 0=FLOAT

// Auto-calibration (FEAT_STEER_AUTOCAL): runs once per connection, the first time the car is
// armed with the throttle at neutral. The steering sweeps to both end-stops at low power,
// then centres. The usable range is set to STEER_CAL_MARGIN_PCT of the measured half-span.
#define STEER_CAL_SPEED        25   // sweep speed [1..100]
#define STEER_CAL_POWER        35   // sweep torque limit [1..100] — keep low
#define STEER_CAL_STALL_DEG    3    // movement below this ...
#define STEER_CAL_STALL_MS     250  // ... for this long = end-stop reached
#define STEER_CAL_MIN_RUN_MS   300  // ignore stalls right after starting a sweep
#define STEER_CAL_SWEEP_TIMEOUT_MS 4000
#define STEER_CAL_FEEDBACK_TIMEOUT_MS 1500
#define STEER_CAL_MIN_HALF_DEG 40   // sanity limits on the measured half-span (motor deg)
#define STEER_CAL_MAX_HALF_DEG 900
#define STEER_CAL_MARGIN_PCT   90

// Speed-dependent gain (FEAT_STEER_SPEED_GAIN): full steering below FADE_START, reduced
// linearly to GAIN_MIN_PCT at FADE_END (drive-motor degrees per second).
#define STEER_GAIN_MIN_PCT     55
#define STEER_GAIN_FADE_START_DPS 300
#define STEER_GAIN_FADE_END_DPS   1200

// ============================================================
// BLE
// ============================================================

#define BLE_SETUP_TIMEOUT_MS   8000
#define SCAN_INTERVAL          0x0060
#define SCAN_WINDOW            0x0030
#define CONN_INTERVAL_MIN      12   // x1.25 ms = 15 ms
#define CONN_INTERVAL_MAX      24   // x1.25 ms = 30 ms
#define CONN_LATENCY           0
#define CONN_SUPERVISION_TO    200  // x10 ms = 2 s

#define DEBUG_PRINT_ADV_REPORTS  0

// ============================================================
// Telemetry (FEAT_TELEMETRY): MAVLink 2 over UART0 TX = GP0 -> flight controller UART RX
// ============================================================

#define TELEM_UART_TX_GPIO     0
#define TELEM_BAUD             115200
#define TELEM_SYSID            1    // must match the flight controller's SYSID_THISMAV
#define TELEM_COMPID           25   // MAV_COMP_ID_USER1
#define TELEM_VALUE_PERIOD_TICKS 2  // one NAMED_VALUE_FLOAT every N control ticks

// ============================================================
// Watchdog
// ============================================================

#define WATCHDOG_TIMEOUT_MS    1000

#endif // DBW_CONFIG_H
