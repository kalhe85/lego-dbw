// lego_dbw_system.c
// Pico W PWM -> BLE LEGO Technic Hub (LWP3) Drive-by-Wire Gateway
//
// Design notes:
//  1. The control loop runs as a BTstack run-loop timer (same context as BTstack), never from
//     an IRQ. Nothing calls cyw43_arch_poll()/BTstack/printf from interrupt context.
//  2. TX flow control: write return codes are checked; commands are sent only on change or
//     periodic refresh; no per-command feedback is requested (startup byte 0x10).
//  3. Explicit connection parameters: supervision timeout 2 s, slave latency 0.
//  4. Timeouts for CONNECTING / discovery / CCCD steps -> cancel / disconnect and rescan.
//  5. Notifications: Generic Error, Attached I/O, battery %, low-voltage / high-current alerts,
//     and (FEAT_MOTOR_FEEDBACK) steering position and drive-motor position.
//  6. PWM capture: PIO state machines (FEAT_PIO_CAPTURE) or GPIO IRQ timestamps; pulses outside
//     [800..2200] us are rejected.
//  7. Hardware watchdog (1 s).
//  8. Steering: mapping and automatic end-stop calibration in steering.c.
//  9. GATT selection prefers the exact LEGO Hub characteristic UUID, heuristic as fallback.
//
// All settings and the feature stage are in config.h.
// Build: Pico SDK 2.2.x (see CMakeLists.txt).

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

#include "config.h"
#include "lwp3.h"
#include "steering.h"
#include "drive.h"
#include "supervisor.h"
#if FEAT_PIO_CAPTURE
#include "pwm_capture.h"
#endif
#if FEAT_TELEMETRY
#include "telemetry.h"
#endif

#if FEAT_TELEMETRY
#define TELEM_EVENT(sev, text) telem_event((sev), (text))
#else
#define TELEM_EVENT(sev, text) do { (void)(sev); (void)(text); } while (0)
#endif

static const bd_addr_t HUB_ADDR = HUB_ADDR_BYTES;
static const bd_addr_type_t HUB_ADDR_TYPE = BD_ADDR_TYPE_LE_PUBLIC;

// LEGO Hub characteristic 00001624-1212-EFDE-1623-785FEABCD123 (BTstack big-endian order)
static const uint8_t LEGO_HUB_CHAR_UUID128[16] = {
    0x00, 0x00, 0x16, 0x24, 0x12, 0x12, 0xEF, 0xDE,
    0x16, 0x23, 0x78, 0x5F, 0xEA, 0xBC, 0xD1, 0x23
};

// ============================================================
// PWM CAPTURE
// ============================================================

typedef struct {
    volatile uint32_t rise_us;
    volatile bool     rise_valid;
    volatile uint16_t pulse_us;
    volatile uint32_t last_update_us;
    volatile bool     have_pulse;
} pwm_chan_t;

static pwm_chan_t pwm_thr = {0};
static pwm_chan_t pwm_str = {0};
static pwm_chan_t pwm_en  = {0};
static bool pio_capture_active = false;

static inline bool pulse_valid(uint32_t us) {
    return us >= PWM_VALID_MIN_US && us <= PWM_VALID_MAX_US;
}

// GPIO-IRQ capture (used at stage 0, or if PIO resources are unavailable).
static void pwm_gpio_irq(uint gpio, uint32_t events) {
    uint32_t t = time_us_32();
    pwm_chan_t *ch;

    if (gpio == GPIO_PWM_THROTTLE)      ch = &pwm_thr;
    else if (gpio == GPIO_PWM_STEERING) ch = &pwm_str;
    else if (gpio == GPIO_PWM_ENABLE)   ch = &pwm_en;
    else return;

    bool rise = (events & GPIO_IRQ_EDGE_RISE) != 0;
    bool fall = (events & GPIO_IRQ_EDGE_FALL) != 0;

    // Both edges latched in one IRQ: edge order/timing unknown (dt would be 0 -> full reverse).
    if (rise && fall) {
        ch->rise_valid = false;
        return;
    }
    if (rise) {
        ch->rise_us = t;
        ch->rise_valid = true;
        return;
    }
    if (fall && ch->rise_valid) {
        ch->rise_valid = false;
        uint32_t dt = t - ch->rise_us;
        if (pulse_valid(dt)) {
            ch->pulse_us = (uint16_t)dt;
            ch->last_update_us = t;
            ch->have_pulse = true;
        }
    }
}

#if FEAT_PIO_CAPTURE
// Drain the PIO FIFOs into the same channel structs (called from the control tick, not an IRQ).
static void pwm_pio_poll_all(void) {
    pwm_chan_t *chans[3] = { &pwm_thr, &pwm_str, &pwm_en };
    for (unsigned i = 0; i < 3; i++) {
        uint16_t us;
        if (pwm_capture_poll(i, &us) && pulse_valid(us)) {
            chans[i]->pulse_us = us;
            chans[i]->last_update_us = time_us_32();
            chans[i]->have_pulse = true;
        }
    }
}
#endif

typedef struct {
    uint16_t pulse_us;
    bool     fresh;
} pwm_sample_t;

// Coherent snapshot of one channel; age computed after the snapshot so it can't underflow.
static pwm_sample_t pwm_sample(const pwm_chan_t *ch) {
    uint32_t irq = save_and_disable_interrupts();
    bool     have  = ch->have_pulse;
    uint16_t pulse = ch->pulse_us;
    uint32_t last  = ch->last_update_us;
    restore_interrupts(irq);

    uint32_t age_us = time_us_32() - last;
    pwm_sample_t s = { pulse, have && (age_us <= (uint32_t)PWM_TIMEOUT_MS * 1000u) };
    return s;
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Map pulse width to [-max_abs..+max_abs] with deadband around neutral.
static int32_t map_pwm(uint16_t us, int32_t max_abs) {
    int32_t d = (int32_t)us - PWM_NEUTRAL_US;
    if (d > -PWM_DEADBAND_US && d < PWM_DEADBAND_US) return 0;
    int32_t x = clamp_i32((int32_t)us, PWM_MIN_US, PWM_MAX_US);
    int32_t p = ((x - PWM_NEUTRAL_US) * max_abs) / (PWM_MAX_US - PWM_NEUTRAL_US);
    return clamp_i32(p, -max_abs, max_abs);
}

// ============================================================
// STEERING / DRIVE STATE
// ============================================================

static const steer_map_params_t steer_map = {
    .wheel_max_deg   = (float)STEER_TARGET_DEG_MAX,
    .uncal_limit_deg = (STEER_TARGET_DEG_MAX * STEER_GEAR_RATIO < STEER_MOTOR_DEG_LIMIT)
                         ? STEER_TARGET_DEG_MAX * STEER_GEAR_RATIO : STEER_MOTOR_DEG_LIMIT,
    .curvature_map   = FEAT_STEER_CURVATURE,
    .speed_gain      = FEAT_STEER_SPEED_GAIN,
    .gain_min        = STEER_GAIN_MIN_PCT / 100.0f,
    .fade_start_dps  = (float)STEER_GAIN_FADE_START_DPS,
    .fade_end_dps    = (float)STEER_GAIN_FADE_END_DPS,
};

#if FEAT_STEER_AUTOCAL
static const steer_cal_params_t steer_cal_params = {
    .stall_deg           = STEER_CAL_STALL_DEG,
    .stall_ms            = STEER_CAL_STALL_MS,
    .min_run_ms          = STEER_CAL_MIN_RUN_MS,
    .sweep_timeout_ms    = STEER_CAL_SWEEP_TIMEOUT_MS,
    .feedback_timeout_ms = STEER_CAL_FEEDBACK_TIMEOUT_MS,
    .min_half_deg        = STEER_CAL_MIN_HALF_DEG,
    .max_half_deg        = STEER_CAL_MAX_HALF_DEG,
    .margin_pct          = STEER_CAL_MARGIN_PCT,
};
#endif

static dbw_sup_t sup;                  // sup.cal stays IDLE when auto-calibration is disabled
static bool    have_steer_pos = false;
static int32_t steer_pos = 0;
static speed_est_t drive_speed;

// ============================================================
// OUTPUT CACHE (send on change + periodic refresh)
// ============================================================

typedef enum { OUT_NONE = 0, OUT_POWER, OUT_SPEED, OUT_GOTO } out_kind_t;

typedef struct {
    bool       valid;
    out_kind_t kind;
    int32_t    value;
    uint32_t   sent_ms;
} out_cache_t;

static out_cache_t out_a, out_b, out_steer;

static void out_cache_invalidate_all(void) {
    memset(&out_a, 0, sizeof(out_a));
    memset(&out_b, 0, sizeof(out_b));
    memset(&out_steer, 0, sizeof(out_steer));
}

static bool out_need_send(const out_cache_t *c, out_kind_t kind, int32_t v, int32_t min_delta,
                          uint32_t now) {
    if (!c->valid || c->kind != kind) return true;
    if (v != c->value) {
        int32_t d = v - c->value;
        if (d < 0) d = -d;
        if (d >= min_delta || v == 0) return true;   // always honour exact stop / zero
    }
    return (now - c->sent_ms) >= OUTPUT_REFRESH_MS;
}

static void out_mark_sent(out_cache_t *c, out_kind_t kind, int32_t v, uint32_t now) {
    c->valid = true;
    c->kind = kind;
    c->value = v;
    c->sent_ms = now;
}

// ============================================================
// BLE CENTRAL STATE MACHINE
// ============================================================

typedef enum {
    BLE_OFF = 0,
    BLE_SCANNING,
    BLE_CONNECTING,
    BLE_W4_SERVICES,
    BLE_W4_CHARS,
    BLE_W4_NOTIFY_EN,
    BLE_READY
} ble_state_t;

static ble_state_t ble_state = BLE_OFF;
static uint32_t    ble_state_since_ms = 0;
static hci_con_handle_t conn_handle = HCI_CON_HANDLE_INVALID;
static btstack_packet_callback_registration_t hci_event_reg;

#define MAX_SERVICES 24
static gatt_client_service_t services[MAX_SERVICES];
static uint8_t service_count = 0;
static uint8_t service_index = 0;
static gatt_client_service_t current_service;

static uint16_t write_value_handle = 0;
static bool     write_is_wo_resp   = true;
static gatt_client_characteristic_t notify_char;

static uint16_t tmp_write_value_handle = 0;
static bool     tmp_write_is_wo_resp   = false;
static gatt_client_characteristic_t tmp_notify_char;
static bool     tmp_notify_char_found  = false;
static bool     tmp_exact_match        = false;

static bool     fb_valid = false;
static uint16_t fb_write_value_handle = 0;
static bool     fb_write_is_wo_resp = false;
static gatt_client_characteristic_t fb_notify_char;

static gatt_client_notification_t notif_listener;
static bool notif_listening = false;

// Hub init: subscriptions and motor settings sent once after READY
#define HUB_INIT_STEPS 8
static uint8_t hub_init_step = 0;

static uint32_t tx_ok = 0, tx_drop = 0;
static int      last_battery_pct = -1;

// Buffer for write-with-response: BTstack keeps the pointer until the write completes.
static uint8_t wr_resp_buf[20];

static void gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void hub_notification_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void ble_set_state(ble_state_t s) {
    ble_state = s;
    ble_state_since_ms = btstack_run_loop_get_time_ms();
}

static void ble_reset_context(void) {
    if (notif_listening) {
        gatt_client_stop_listening_for_characteristic_value_updates(&notif_listener);
        notif_listening = false;
    }
    conn_handle = HCI_CON_HANDLE_INVALID;

    memset(services, 0, sizeof(services));
    service_count = 0;
    service_index = 0;
    memset(&current_service, 0, sizeof(current_service));

    write_value_handle = 0;
    write_is_wo_resp = true;
    memset(&notify_char, 0, sizeof(notify_char));

    tmp_write_value_handle = 0;
    tmp_write_is_wo_resp = false;
    tmp_notify_char_found = false;
    tmp_exact_match = false;
    memset(&tmp_notify_char, 0, sizeof(tmp_notify_char));

    fb_valid = false;
    fb_write_value_handle = 0;
    fb_write_is_wo_resp = false;
    memset(&fb_notify_char, 0, sizeof(fb_notify_char));

    hub_init_step = 0;
    last_battery_pct = -1;
    out_cache_invalidate_all();

    // The hub may have been power-cycled: its encoder zero is gone, so calibrate again.
    dbw_sup_link_lost(&sup);
    have_steer_pos = false;
    steer_pos = 0;
    speed_est_reset(&drive_speed);
    ble_set_state(BLE_OFF);
}

static inline bool ble_ready(void) {
    return (ble_state == BLE_READY) &&
           (conn_handle != HCI_CON_HANDLE_INVALID) &&
           (write_value_handle != 0);
}

static inline bool hub_init_done(void) {
    return hub_init_step >= HUB_INIT_STEPS;
}

static inline bool adv_is_target(const bd_addr_t addr) {
    return memcmp(addr, HUB_ADDR, sizeof(bd_addr_t)) == 0;
}

static const char *disc_reason_str(uint8_t r) {
    switch (r) {
        case 0x08: return "connection timeout (RF / peer stopped responding / peer reset)";
        case 0x13: return "remote user terminated (hub closed the link)";
        case 0x16: return "terminated by local host";
        case 0x22: return "LL response timeout";
        case 0x3B: return "unacceptable connection parameters";
        case 0x3E: return "failed to be established";
        default:   return "other";
    }
}

static void start_scan(void) {
    gap_set_scan_parameters(0, SCAN_INTERVAL, SCAN_WINDOW);   // passive scan is enough
    gap_set_scan_duplicate_filter(true);
    gap_start_scan();
    ble_set_state(BLE_SCANNING);
    printf("[BLE] scanning...\n");
}

static void connect_to_hub(void) {
    gap_stop_scan();
    gap_set_connection_parameters(SCAN_INTERVAL, SCAN_WINDOW,
                                  CONN_INTERVAL_MIN, CONN_INTERVAL_MAX,
                                  CONN_LATENCY, CONN_SUPERVISION_TO,
                                  0, 0);
    ble_set_state(BLE_CONNECTING);
    printf("[BLE] connecting to %02X:%02X:%02X:%02X:%02X:%02X...\n",
           HUB_ADDR[0], HUB_ADDR[1], HUB_ADDR[2], HUB_ADDR[3], HUB_ADDR[4], HUB_ADDR[5]);
    uint8_t st = gap_connect(HUB_ADDR, HUB_ADDR_TYPE);
    if (st != ERROR_CODE_SUCCESS) {
        printf("[BLE] gap_connect error 0x%02x -> rescan\n", st);
        ble_reset_context();
        start_scan();
    }
}

static void ble_abort_link(const char *why) {
    printf("[BLE] abort: %s\n", why);
    if (conn_handle != HCI_CON_HANDLE_INVALID) gap_disconnect(conn_handle);
    // HCI_EVENT_DISCONNECTION_COMPLETE will reset + rescan
}

// Called from the control tick: un-sticks the state machine if a step never completes.
static void ble_supervise(uint32_t now) {
    bool in_setup = (ble_state == BLE_CONNECTING)  ||
                    (ble_state == BLE_W4_SERVICES) ||
                    (ble_state == BLE_W4_CHARS)    ||
                    (ble_state == BLE_W4_NOTIFY_EN);
    if (!in_setup) return;
    if ((now - ble_state_since_ms) < BLE_SETUP_TIMEOUT_MS) return;

    printf("[BLE] setup timeout in state %d\n", (int)ble_state);
    ble_state_since_ms = now;   // rate-limit retries
    if (ble_state == BLE_CONNECTING) {
        gap_connect_cancel();   // controller then reports LE conn complete with error -> rescan
    } else {
        ble_abort_link("setup step timed out");
    }
}

// ---------------- GATT discovery ----------------

static void gatt_start_service_discovery(void) {
    service_count = 0;
    service_index = 0;
    fb_valid = false;
    ble_set_state(BLE_W4_SERVICES);
    printf("[GATT] discover all primary services...\n");
    gatt_client_discover_primary_services(gatt_handler, conn_handle);
}

static void set_ready(void) {
    ble_set_state(BLE_READY);
    printf("[BLE] READY\n");
    TELEM_EVENT(6 /* INFO */, "hub connected");
}

static void enable_notifications(void) {
    gatt_client_listen_for_characteristic_value_updates(&notif_listener, hub_notification_handler,
                                                        conn_handle, &notify_char);
    notif_listening = true;

    ble_set_state(BLE_W4_NOTIFY_EN);
    uint8_t st = gatt_client_write_client_characteristic_configuration(
        gatt_handler, conn_handle, &notify_char,
        GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
    if (st != ERROR_CODE_SUCCESS) {
        printf("[GATT] CCCD write not started (0x%02x) -> READY without notify\n", st);
        set_ready();
    }
}

static void commit_selection(uint16_t wh, bool wo_resp, const gatt_client_characteristic_t *nc,
                             const char *how) {
    write_value_handle = wh;
    write_is_wo_resp = wo_resp;
    notify_char = *nc;
    printf("[GATT] selected (%s): write=0x%04x (%s) notify=0x%04x\n",
           how, wh, wo_resp ? "WO_RESP" : "WRITE", nc->value_handle);
    enable_notifications();
}

static void gatt_start_char_discovery_for_index(uint8_t idx) {
    if (idx >= service_count) {
        if (fb_valid) {
            commit_selection(fb_write_value_handle, fb_write_is_wo_resp, &fb_notify_char,
                             "heuristic fallback");
        } else {
            ble_abort_link("no service with write+notify");
        }
        return;
    }

    current_service = services[idx];
    tmp_write_value_handle = 0;
    tmp_write_is_wo_resp = false;
    tmp_notify_char_found = false;
    tmp_exact_match = false;
    memset(&tmp_notify_char, 0, sizeof(tmp_notify_char));

    ble_set_state(BLE_W4_CHARS);
    gatt_client_discover_characteristics_for_service(gatt_handler, conn_handle, &current_service);
}

static void gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {

        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            gatt_client_service_t s;
            gatt_event_service_query_result_get_service(packet, &s);
            if (service_count < MAX_SERVICES) services[service_count++] = s;
            break;
        }

        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t c;
            gatt_event_characteristic_query_result_get_characteristic(packet, &c);

            bool can_wnr = (c.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
            bool can_w   = (c.properties & ATT_PROPERTY_WRITE) != 0;
            bool can_ntf = (c.properties & ATT_PROPERTY_NOTIFY) != 0;

            if (tmp_exact_match) break;

            // Preferred: the LEGO Hub characteristic (write + notify on the same char)
            if (memcmp(c.uuid128, LEGO_HUB_CHAR_UUID128, 16) == 0 && (can_wnr || can_w) && can_ntf) {
                tmp_write_value_handle = c.value_handle;
                tmp_write_is_wo_resp = can_wnr;
                tmp_notify_char = c;
                tmp_notify_char_found = true;
                tmp_exact_match = true;
                break;
            }

            // Heuristic candidates (fallback)
            if (tmp_write_value_handle == 0 && (can_wnr || can_w)) {
                tmp_write_value_handle = c.value_handle;
                tmp_write_is_wo_resp = can_wnr;
            }
            if (!tmp_notify_char_found && can_ntf) {
                tmp_notify_char = c;
                tmp_notify_char_found = true;
            }
            break;
        }

        case GATT_EVENT_QUERY_COMPLETE: {
            uint8_t att = gatt_event_query_complete_get_att_status(packet);

            switch (ble_state) {
                case BLE_W4_SERVICES:
                    if (att != ATT_ERROR_SUCCESS || service_count == 0) {
                        ble_abort_link("service discovery failed");
                        return;
                    }
                    printf("[GATT] %u services\n", (unsigned)service_count);
                    service_index = 0;
                    gatt_start_char_discovery_for_index(service_index);
                    return;

                case BLE_W4_CHARS:
                    if (att != ATT_ERROR_SUCCESS) {
                        ble_abort_link("characteristic discovery failed");
                        return;
                    }
                    if (tmp_exact_match) {
                        commit_selection(tmp_write_value_handle, tmp_write_is_wo_resp,
                                         &tmp_notify_char, "LEGO UUID");
                        return;
                    }
                    if (!fb_valid && tmp_write_value_handle != 0 && tmp_notify_char_found) {
                        fb_valid = true;
                        fb_write_value_handle = tmp_write_value_handle;
                        fb_write_is_wo_resp = tmp_write_is_wo_resp;
                        fb_notify_char = tmp_notify_char;
                    }
                    service_index++;
                    gatt_start_char_discovery_for_index(service_index);
                    return;

                case BLE_W4_NOTIFY_EN:
                    if (att != ATT_ERROR_SUCCESS) {
                        printf("[GATT] CCCD write failed att=0x%02x (continuing without notify)\n", att);
                    }
                    set_ready();
                    return;

                case BLE_READY:
                    // completion of a write-with-response; never tear down the link for this
                    if (att != ATT_ERROR_SUCCESS) {
                        printf("[GATT] write failed att=0x%02x\n", att);
                    }
                    return;

                default:
                    return;
            }
        }

        default:
            break;
    }
}

// ---------------- Hub notifications ----------------

static void hub_notification_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != GATT_EVENT_NOTIFICATION) return;

    uint16_t len = gatt_event_notification_get_value_length(packet);
    const uint8_t *v = gatt_event_notification_get_value(packet);

    lwp3_msg_t m;
    if (!lwp3_parse(v, len, &m)) return;

    switch (m.type) {
        case LWP3_MSG_BATTERY:
            if (m.value != last_battery_pct) {
                printf("[HUB] battery %ld%%\n", (long)m.value);
                last_battery_pct = (int)m.value;
            }
            break;

        case LWP3_MSG_ALERT: {
            printf("[HUB] alert %s: %s\n", lwp3_alert_name(m.a), m.value ? "ACTIVE" : "cleared");
            if (m.value) {
                char t[40];
                snprintf(t, sizeof(t), "hub alert %s", lwp3_alert_name(m.a));
                TELEM_EVENT(4 /* WARNING */, t);
            }
            break;
        }

        case LWP3_MSG_ATTACHED_IO:
            printf("[HUB] port 0x%02x %s io_type=0x%04lx\n", m.a,
                   m.b == 0 ? "detached" : (m.b == 1 ? "attached" : "virtual"), (unsigned long)m.value);
            break;

        case LWP3_MSG_ERROR:
            printf("[HUB] ERROR cmd=0x%02x code=0x%02x\n", m.a, m.b);
            break;

        case LWP3_MSG_INPUT_FORMAT:
            printf("[HUB] port 0x%02x mode %u notify=%ld\n", m.a, m.b, (long)m.value);
            break;

        case LWP3_MSG_PORT_VALUE:
            if (m.a == LEGO_PORT_D) {
                steer_pos = m.value;
                have_steer_pos = true;
            } else if (m.a == LEGO_PORT_A) {
                speed_est_update(&drive_speed, m.value, btstack_run_loop_get_time_ms(),
                                 SPEED_WINDOW_MS);
            }
            break;

        default:
            break;
    }
}

// ---------------- HCI events ----------------

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {

        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                ble_reset_context();
                start_scan();
            }
            break;

        case GAP_EVENT_ADVERTISING_REPORT: {
            bd_addr_t addr;
            gap_event_advertising_report_get_address(packet, addr);
#if DEBUG_PRINT_ADV_REPORTS
            printf("[ADV] %02X:%02X:%02X:%02X:%02X:%02X rssi=%d\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                   (int)(int8_t)gap_event_advertising_report_get_rssi(packet));
#endif
            if (ble_state == BLE_SCANNING && adv_is_target(addr)) {
                printf("[BLE] hub seen rssi=%d -> connect\n",
                       (int)(int8_t)gap_event_advertising_report_get_rssi(packet));
                connect_to_hub();
            }
            break;
        }

        case HCI_EVENT_META_GAP:
            if (hci_event_gap_meta_get_subevent_code(packet) == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
                uint8_t status = gap_subevent_le_connection_complete_get_status(packet);
                if (status != ERROR_CODE_SUCCESS) {
                    printf("[BLE] connect failed status=0x%02x -> rescan\n", status);
                    ble_reset_context();
                    start_scan();
                    return;
                }
                conn_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
                printf("[BLE] connected handle=0x%04x interval=%u x1.25ms\n", conn_handle,
                       (unsigned)gap_subevent_le_connection_complete_get_conn_interval(packet));
                gatt_start_service_discovery();
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            printf("[BLE] disconnected 0x%02x (%s) tx_ok=%lu tx_drop=%lu -> rescan\n",
                   reason, disc_reason_str(reason),
                   (unsigned long)tx_ok, (unsigned long)tx_drop);
            char t[40];
            snprintf(t, sizeof(t), "hub disconnected 0x%02x", reason);
            TELEM_EVENT(3 /* ERROR */, t);
            ble_reset_context();
            start_scan();
            break;
        }

        default:
            break;
    }
}

// ============================================================
// ACTUATION
// ============================================================

// Returns true only if BTstack accepted the packet. Callers keep their cached state on failure
// and retry on the next tick (latest value wins; nothing is queued).
static bool write_lwp3(const uint8_t *msg, uint16_t len) {
    if (!ble_ready()) return false;

    uint8_t st;
    if (write_is_wo_resp) {
        st = gatt_client_write_value_of_characteristic_without_response(
            conn_handle, write_value_handle, len, (uint8_t *)msg);
    } else {
        if (len > sizeof(wr_resp_buf)) return false;
        memcpy(wr_resp_buf, msg, len);
        st = gatt_client_write_value_of_characteristic(
            gatt_handler, conn_handle, write_value_handle, len, wr_resp_buf);
    }

    if (st != ERROR_CODE_SUCCESS) {
        tx_drop++;
        return false;
    }
    tx_ok++;
    return true;
}

// Drive motor command. Disarmed: power 0 (float), as in v1.0. Armed: open-loop power, or a
// speed set-point for the hub's speed controller (FEAT_DRIVE_SPEED_LOOP).
static void set_drive(uint8_t port, out_cache_t *c, bool armed, int32_t thr_cmd, uint32_t now) {
    uint8_t msg[LWP3_MAX_MSG];
    uint16_t len;
    out_kind_t kind;
    int32_t value;

#if FEAT_DRIVE_SPEED_LOOP
    if (armed) {
        kind = OUT_SPEED;
        value = (thr_cmd * DRIVE_SPEED_MAX_PCT) / 100;
        if (!out_need_send(c, kind, value, 1, now)) return;
        len = lwp3_start_speed(port, (int8_t)value, DRIVE_MAX_POWER,
                               DRIVE_ACCEL_MS > 0 ? LWP3_PROFILE_ACC : 0, msg);
        if (write_lwp3(msg, len)) out_mark_sent(c, kind, value, now);
        return;
    }
#endif
    kind = OUT_POWER;
    value = armed ? thr_cmd : 0;
    if (!out_need_send(c, kind, value, 1, now)) return;
    len = lwp3_start_power(port, (int8_t)value, msg);
    if (write_lwp3(msg, len)) out_mark_sent(c, kind, value, now);
}

static void set_steer_goto(int32_t motor_deg, uint32_t now) {
    if (!out_need_send(&out_steer, OUT_GOTO, motor_deg, STEER_RESEND_DEG, now)) return;
    uint8_t msg[LWP3_MAX_MSG];
    uint16_t len = lwp3_goto_abs(LEGO_PORT_D, motor_deg,
                                 (int8_t)STEER_GOTO_SPEED, STEER_GOTO_MAXPOWER,
                                 STEER_ENDSTATE, 0, msg);
    if (write_lwp3(msg, len)) out_mark_sent(&out_steer, OUT_GOTO, motor_deg, now);
}

#if FEAT_STEER_AUTOCAL
static void set_steer_speed(int32_t speed, uint32_t now) {
    if (!out_need_send(&out_steer, OUT_SPEED, speed, 1, now)) return;
    uint8_t msg[LWP3_MAX_MSG];
    uint16_t len = lwp3_start_speed(LEGO_PORT_D, (int8_t)speed, STEER_CAL_POWER, 0, msg);
    if (write_lwp3(msg, len)) out_mark_sent(&out_steer, OUT_SPEED, speed, now);
}
#endif

// One message per step, sent once after READY. Returns 0 for steps disabled at this stage.
static uint16_t hub_init_build(uint8_t step, uint8_t *msg) {
    switch (step) {
        case 0: return lwp3_hub_prop_enable_updates(0x06, msg);    // battery %
        case 1: return lwp3_hub_alert_enable(0x01, msg);           // low voltage
        case 2: return lwp3_hub_alert_enable(0x02, msg);           // high current
#if FEAT_MOTOR_FEEDBACK
        case 3: return lwp3_input_format_setup(LEGO_PORT_D, LWP3_MODE_POS, 1, true, msg);
        case 4: return lwp3_input_format_setup(LEGO_PORT_A, LWP3_MODE_POS,
                                               DRIVE_POS_DELTA_DEG, true, msg);
        // Ask for the current steering position explicitly, in case the hub only reports
        // on change (calibration needs a position before the motor moves).
        case 5: return lwp3_port_value_request(LEGO_PORT_D, msg);
#endif
#if FEAT_DRIVE_SPEED_LOOP
        case 6: return DRIVE_ACCEL_MS > 0 ? lwp3_set_acc_time(LEGO_PORT_A, DRIVE_ACCEL_MS, msg) : 0;
        case 7: return DRIVE_ACCEL_MS > 0 ? lwp3_set_acc_time(LEGO_PORT_B, DRIVE_ACCEL_MS, msg) : 0;
#endif
        default: return 0;
    }
}

static void hub_init_service(void) {
    while (hub_init_step < HUB_INIT_STEPS) {
        uint8_t msg[LWP3_MAX_MSG];
        uint16_t len = hub_init_build(hub_init_step, msg);
        if (len > 0 && !write_lwp3(msg, len)) return;   // retry next tick
        hub_init_step++;
    }
}

// ============================================================
// STEERING CALIBRATION SUPERVISION
// ============================================================

#if FEAT_STEER_AUTOCAL
static void report_cal_transition(steer_cal_state_t prev) {
    if (sup.cal.state == prev) return;
    char t[51];
    switch (sup.cal.state) {
        case STEER_CAL_SWEEP_NEG:
            printf("[CAL] sweeping to first end-stop (start pos %ld)\n", (long)steer_pos);
            break;
        case STEER_CAL_SWEEP_POS:
            printf("[CAL] end-stop A at %ld, sweeping to end-stop B\n", (long)sup.cal.end_neg);
            break;
        case STEER_CAL_DONE:
            printf("[CAL] done: end-stops %ld / %ld, centre %ld, usable +/-%ld motor deg\n",
                   (long)sup.cal.end_neg, (long)sup.cal.end_pos,
                   (long)sup.cal.centre, (long)sup.cal.limit);
            snprintf(t, sizeof(t), "steer cal ok, centre %ld range +/-%ld",
                     (long)sup.cal.centre, (long)sup.cal.limit);
            TELEM_EVENT(6 /* INFO */, t);
            break;
        case STEER_CAL_FAILED:
            printf("[CAL] FAILED: %s -> uncalibrated steering (+/-%ld around power-on zero)\n",
                   sup.cal.fail_reason ? sup.cal.fail_reason : "?",
                   (long)steer_map.uncal_limit_deg);
            TELEM_EVENT(4 /* WARNING */, "steer cal failed, uncalibrated");
            break;
        default:
            break;
    }
}

// Logs calibration events produced by the supervisor step.
static void report_cal(const dbw_sup_out_t *o) {
    if (o->cal_started) printf("[CAL] starting steering calibration\n");
    if (o->cal_aborted) printf("[CAL] aborted (disarmed)\n");
    report_cal_transition(o->prev_cal_state);
}
#endif

// ============================================================
// CONTROL LOOP (BTstack run-loop timer: same context as BTstack, NOT an IRQ)
// ============================================================

static btstack_timer_source_t control_timer;
static uint32_t diag_counter = 0;

static void control_tick(btstack_timer_source_t *ts) {
    btstack_run_loop_set_timer(ts, CONTROL_PERIOD_MS);
    btstack_run_loop_add_timer(ts);

    watchdog_update();

    uint32_t now = btstack_run_loop_get_time_ms();
    ble_supervise(now);

#if FEAT_PIO_CAPTURE
    if (pio_capture_active) pwm_pio_poll_all();
#endif

    pwm_sample_t thr = pwm_sample(&pwm_thr);
    pwm_sample_t str = pwm_sample(&pwm_str);
    pwm_sample_t en  = pwm_sample(&pwm_en);

    bool enable_high  = en.fresh && (en.pulse_us >= ENABLE_THRESHOLD_US);
    bool armed        = thr.fresh && str.fresh && enable_high;
    bool transport_ok = ble_ready();

    // Safe defaults: stop + centre
    int32_t thr_cmd = 0;
    float   stick   = 0.0f;
    if (armed) {
        thr_cmd = map_pwm(thr.pulse_us, THROTTLE_MAX_POWER);
        stick   = (float)map_pwm(str.pulse_us, 1000) / 1000.0f;
    }

    float speed = FEAT_MOTOR_FEEDBACK ? speed_est_get(&drive_speed, now, SPEED_TIMEOUT_MS) : 0.0f;

#if FEAT_STEER_AUTOCAL
    // Calibration sequencing and drive interlock (rules in supervisor.h)
    dbw_sup_in_t sin = {
        .now_ms = now, .armed = armed, .link_ready = transport_ok,
        .hub_init_done = hub_init_done(), .have_pos = have_steer_pos, .pos = steer_pos,
        .thr_cmd = thr_cmd,
    };
    dbw_sup_out_t sout;
    dbw_sup_step(&sup, &steer_cal_params, &sin, &sout);
    report_cal(&sout);
    steer_cal_cmd_t cal_cmd = sout.cal_cmd;
    thr_cmd = sout.thr_cmd;
#endif

    int32_t steer_abs = steer_target(&steer_map, &sup.cal, stick, speed);
    int32_t centre = (sup.cal.state == STEER_CAL_DONE) ? sup.cal.centre : 0;

    if (transport_ok) {
        // drive first: stop commands have priority over steering
        set_drive(LEGO_PORT_A, &out_a, armed, thr_cmd, now);
        set_drive(LEGO_PORT_B, &out_b, armed, thr_cmd, now);
#if FEAT_STEER_AUTOCAL
        if (cal_cmd == STEER_CAL_CMD_RUN_NEG)      set_steer_speed(-STEER_CAL_SPEED, now);
        else if (cal_cmd == STEER_CAL_CMD_RUN_POS) set_steer_speed(STEER_CAL_SPEED, now);
        else if (cal_cmd == STEER_CAL_CMD_HOLD)    set_steer_speed(0, now);
        else
#endif
        set_steer_goto(steer_abs, now);
        hub_init_service();
    }

#if FEAT_TELEMETRY
    telem_snapshot_t snap = {
        .link_ready    = transport_ok,
        .armed         = armed,
        .thr_cmd       = thr_cmd,
        .steer_rel_deg = steer_abs - centre,
        .steer_pos_deg = have_steer_pos ? steer_pos - centre : INT32_MIN,
        .speed_dps     = speed,
        .battery_pct   = last_battery_pct,
        .cal_state     = (int32_t)sup.cal.state,
        .tx_drop       = tx_drop,
    };
    telem_tick(now, &snap);
#endif

    if ((++diag_counter % CONTROL_HZ) == 0) {
        printf("[DBG] ble=%d st=%d | thr=%u%s str=%u%s en=%u%s | armed=%d | thr_cmd=%ld steer=%ld",
               (int)transport_ok, (int)ble_state,
               (unsigned)thr.pulse_us, thr.fresh ? "" : "(stale)",
               (unsigned)str.pulse_us, str.fresh ? "" : "(stale)",
               (unsigned)en.pulse_us,  en.fresh  ? "" : "(stale)",
               (int)armed, (long)thr_cmd, (long)(steer_abs - centre));
#if FEAT_MOTOR_FEEDBACK
        if (have_steer_pos) printf(" pos=%ld", (long)(steer_pos - centre));
        else                printf(" pos=?");
        printf(" spd=%ld", (long)speed);
#endif
#if FEAT_STEER_AUTOCAL
        printf(" cal=%s%s", steer_cal_state_name(sup.cal.state), sup.drive_interlock ? " lock" : "");
        if (armed && sup.cal.state == STEER_CAL_IDLE && !sup.cal_requested)
            printf(" (flip enable off/on to calibrate)");
#endif
        printf(" | tx ok=%lu drop=%lu\n", (unsigned long)tx_ok, (unsigned long)tx_drop);
    }
}

// ============================================================
// MAIN
// ============================================================

int main(void) {
    stdio_init_all();

    absolute_time_t t0 = get_absolute_time();
    while (!stdio_usb_connected()) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > 5 * 1000 * 1000) break;
        sleep_ms(50);
    }
    sleep_ms(200);

    printf("\n=== LEGO DBW Pico W %s (stage %d) ===\n", DBW_VERSION, DBW_STAGE);
    printf("features: pio=%d telem=%d feedback=%d autocal=%d curvature=%d speedloop=%d speedgain=%d\n",
           FEAT_PIO_CAPTURE, FEAT_TELEMETRY, FEAT_MOTOR_FEEDBACK, FEAT_STEER_AUTOCAL,
           FEAT_STEER_CURVATURE, FEAT_DRIVE_SPEED_LOOP, FEAT_STEER_SPEED_GAIN);
    bool wd_reboot = watchdog_caused_reboot();
    if (wd_reboot) printf("[BOOT] *** previous run was reset by WATCHDOG ***\n");

#if FEAT_TELEMETRY
    telem_init();
    if (wd_reboot) telem_event(2 /* CRITICAL */, "gateway rebooted by watchdog");
#endif

    dbw_sup_init(&sup);
    speed_est_reset(&drive_speed);

    // PWM inputs
    const uint pins[3] = { GPIO_PWM_THROTTLE, GPIO_PWM_STEERING, GPIO_PWM_ENABLE };
    for (int i = 0; i < 3; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_down(pins[i]);
    }

    if (cyw43_arch_init()) {
        printf("ERROR: cyw43_arch_init failed\n");
        while (1) sleep_ms(1000);
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(100);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // PWM capture backend: PIO (claimed after the CYW43 driver took its PIO), else GPIO IRQ.
#if FEAT_PIO_CAPTURE
    const unsigned cap_pins[3] = { GPIO_PWM_THROTTLE, GPIO_PWM_STEERING, GPIO_PWM_ENABLE };
    pio_capture_active = pwm_capture_init(cap_pins);
    if (!pio_capture_active) printf("[PWM] PIO unavailable -> falling back to GPIO IRQ capture\n");
#endif
    if (!pio_capture_active) {
        gpio_set_irq_enabled_with_callback(GPIO_PWM_THROTTLE,
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pwm_gpio_irq);
        gpio_set_irq_enabled(GPIO_PWM_STEERING, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        gpio_set_irq_enabled(GPIO_PWM_ENABLE,   GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }
    printf("[PWM] capture: %s\n", pio_capture_active ? "PIO" : "GPIO IRQ");

    l2cap_init();
    sm_init();
    gatt_client_init();

    hci_event_reg.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_reg);

    // Control loop lives inside the BTstack run loop
    btstack_run_loop_set_timer_handler(&control_timer, control_tick);
    btstack_run_loop_set_timer(&control_timer, CONTROL_PERIOD_MS);
    btstack_run_loop_add_timer(&control_timer);

    hci_power_control(HCI_POWER_ON);

    // If the run loop ever wedges, reboot the Pico instead of hanging forever.
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    btstack_run_loop_execute();   // never returns
    return 0;
}
