// lego_dbw_system_v2.c
// Pico W PWM -> BLE LEGO Technic Hub (LWP3) Drive-by-Wire Gateway — v2 (stability rework)
//
// Changes vs v1:
//  1. Control loop runs as a BTstack run-loop timer (same context as BTstack) instead of a
//     repeating_timer IRQ. Nothing calls cyw43_arch_poll()/BTstack/printf from IRQ any more.
//  2. TX flow control: write return codes are checked; commands are sent only on change or
//     periodic refresh; command-feedback flag dropped (startup byte 0x11 -> 0x10).
//  3. Explicit connection parameters: supervision timeout 2 s, slave latency 0.
//  4. Timeouts for CONNECTING / discovery / CCCD steps -> cancel / disconnect and rescan.
//  5. Notifications enabled: logs Generic Error, Attached I/O, battery %, low-voltage /
//     high-current alerts (to diagnose brown-outs).
//  6. PWM capture: pulses outside [800..2200] us rejected; coalesced rise+fall events dropped
//     (v1 could produce pulse_us = 0 -> full reverse throttle).
//  7. Hardware watchdog (1 s).
//  8. Steering: gear ratio + motor-angle soft limit; lower max power to limit stall current.
//  9. GATT selection prefers the exact LEGO Hub characteristic UUID, heuristic as fallback.
//
// Build: Pico SDK 2.2.x
// Link libs: pico_stdlib hardware_gpio hardware_timer hardware_watchdog
//            pico_btstack_ble pico_btstack_cyw43 pico_cyw43_arch_poll
// Requires btstack_config.h with ENABLE_LE_CENTRAL and MAX_NR_GATT_CLIENTS >= 1.

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

// ============================================================
// USER CONFIG
// ============================================================

// PWM GPIOs (Matek -> Pico)
#define GPIO_PWM_THROTTLE   2
#define GPIO_PWM_STEERING   3
#define GPIO_PWM_ENABLE     4

// PWM range / mapping
#define PWM_MIN_US          1000
#define PWM_MAX_US          2000
#define PWM_NEUTRAL_US      1500
#define PWM_DEADBAND_US     30
#define PWM_VALID_MIN_US    800     // pulses outside this window are treated as garbage
#define PWM_VALID_MAX_US    2200

// Safety
#define PWM_TIMEOUT_MS      250
#define ENABLE_THRESHOLD_US 1600

// Control loop
#define CONTROL_HZ          25
#define CONTROL_PERIOD_MS   (1000 / CONTROL_HZ)

// Drive
#define THROTTLE_MAX_POWER  100     // [-100..100]

// Steering "servo" (GotoAbsolutePosition)
#define STEER_TARGET_DEG_MAX   40   // wheel-side degrees at full stick
#define STEER_GEAR_RATIO       6    // motor deg per wheel deg (measured ~6:1)
#define STEER_MOTOR_DEG_LIMIT  240  // TODO: measure the mechanical end-stop, set ~10% below it
#define STEER_RESEND_DEG       3    // ignore target changes smaller than this (motor deg)
#define STEER_GOTO_SPEED       80   // 1..100
#define STEER_GOTO_MAXPOWER    80   // 0..100 (v1 doc used 100 -> high stall current at end-stop)
#define STEER_ENDSTATE         126  // 126=HOLD, 127=BRAKE, 0=FLOAT

// Output scheduling
#define OUTPUT_REFRESH_MS      300  // resend unchanged commands at this period (keep-alive)

// BLE
#define BLE_SETUP_TIMEOUT_MS   8000 // max time in CONNECTING / discovery / CCCD before abort
#define SCAN_INTERVAL          0x0060
#define SCAN_WINDOW            0x0030
#define CONN_INTERVAL_MIN      12   // x1.25 ms = 15 ms
#define CONN_INTERVAL_MAX      24   // x1.25 ms = 30 ms
#define CONN_LATENCY           0
#define CONN_SUPERVISION_TO    200  // x10 ms = 2 s (BTstack default is ~720 ms)

// Watchdog
#define WATCHDOG_TIMEOUT_MS    1000

// Hub address
static const bd_addr_t HUB_ADDR = { 0x90, 0x84, 0x2B, 0x7F, 0xD4, 0x40 };
static const bd_addr_type_t HUB_ADDR_TYPE = BD_ADDR_TYPE_LE_PUBLIC;

// LEGO Hub characteristic 00001624-1212-EFDE-1623-785FEABCD123 (BTstack big-endian order)
static const uint8_t LEGO_HUB_CHAR_UUID128[16] = {
    0x00, 0x00, 0x16, 0x24, 0x12, 0x12, 0xEF, 0xDE,
    0x16, 0x23, 0x78, 0x5F, 0xEA, 0xBC, 0xD1, 0x23
};

// LEGO port IDs
#define LEGO_PORT_A  0x00
#define LEGO_PORT_B  0x01
#define LEGO_PORT_D  0x03

// LWP3 startup/completion byte: execute immediately, no command feedback
#define LWP3_STARTUP_IMMEDIATE  0x10

#define DEBUG_PRINT_ADV_REPORTS  0

// ============================================================
// PWM CAPTURE (IRQ side does only timestamping)
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

static void pwm_gpio_irq(uint gpio, uint32_t events) {
    uint32_t t = time_us_32();
    pwm_chan_t *ch;

    if (gpio == GPIO_PWM_THROTTLE)      ch = &pwm_thr;
    else if (gpio == GPIO_PWM_STEERING) ch = &pwm_str;
    else if (gpio == GPIO_PWM_ENABLE)   ch = &pwm_en;
    else return;

    bool rise = (events & GPIO_IRQ_EDGE_RISE) != 0;
    bool fall = (events & GPIO_IRQ_EDGE_FALL) != 0;

    // Both edges latched in one IRQ = we were too late to know the edge order/timing.
    // v1 computed dt = 0 here -> pulse_us = 0 -> mapped to full reverse. Drop it.
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
        if (dt >= PWM_VALID_MIN_US && dt <= PWM_VALID_MAX_US) {
            ch->pulse_us = (uint16_t)dt;
            ch->last_update_us = t;
            ch->have_pulse = true;
        }
    }
}

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
// LWP3 PACKETS
// ============================================================

// StartPower via WriteDirectModeData (0x51), mode 0
static uint16_t lwp3_start_power(uint8_t port, int8_t power, uint8_t out[8]) {
    out[0] = 8;
    out[1] = 0x00;
    out[2] = 0x81;
    out[3] = port;
    out[4] = LWP3_STARTUP_IMMEDIATE;
    out[5] = 0x51;
    out[6] = 0x00;
    out[7] = (uint8_t)power;
    return 8;
}

// GotoAbsolutePosition (0x0D): [AbsPos int32 LE][Speed][MaxPower][EndState][UseProfile]
static uint16_t lwp3_goto_abs(uint8_t port, int32_t pos, int8_t speed, int8_t max_power,
                              uint8_t end_state, uint8_t out[14]) {
    uint32_t u = (uint32_t)pos;
    out[0]  = 14;
    out[1]  = 0x00;
    out[2]  = 0x81;
    out[3]  = port;
    out[4]  = LWP3_STARTUP_IMMEDIATE;
    out[5]  = 0x0D;
    out[6]  = (uint8_t)(u & 0xFF);
    out[7]  = (uint8_t)((u >> 8) & 0xFF);
    out[8]  = (uint8_t)((u >> 16) & 0xFF);
    out[9]  = (uint8_t)((u >> 24) & 0xFF);
    out[10] = (uint8_t)speed;
    out[11] = (uint8_t)max_power;
    out[12] = end_state;
    out[13] = 0x00;   // UseProfile = 0
    return 14;
}

// Hub Properties (0x01): enable updates (0x02)
static uint16_t lwp3_hub_prop_enable_updates(uint8_t prop, uint8_t out[5]) {
    out[0] = 5; out[1] = 0x00; out[2] = 0x01; out[3] = prop; out[4] = 0x02;
    return 5;
}

// Hub Alerts (0x03): enable updates (0x01)
static uint16_t lwp3_hub_alert_enable(uint8_t alert, uint8_t out[5]) {
    out[0] = 5; out[1] = 0x00; out[2] = 0x03; out[3] = alert; out[4] = 0x01;
    return 5;
}

// ============================================================
// OUTPUT CACHE (send on change + periodic refresh)
// ============================================================

typedef struct {
    bool     valid;
    int32_t  value;
    uint32_t sent_ms;
} out_cache_t;

static out_cache_t out_a, out_b, out_steer;

static void out_cache_invalidate_all(void) {
    memset(&out_a, 0, sizeof(out_a));
    memset(&out_b, 0, sizeof(out_b));
    memset(&out_steer, 0, sizeof(out_steer));
}

static bool out_need_send(const out_cache_t *c, int32_t v, int32_t min_delta, uint32_t now) {
    if (!c->valid) return true;
    if (v != c->value) {
        int32_t d = v - c->value;
        if (d < 0) d = -d;
        if (d >= min_delta || v == 0) return true;   // always honour exact stop / center
    }
    return (now - c->sent_ms) >= OUTPUT_REFRESH_MS;
}

static void out_mark_sent(out_cache_t *c, int32_t v, uint32_t now) {
    c->valid = true;
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

// Selected handles
static uint16_t write_value_handle = 0;
static bool     write_is_wo_resp   = true;
static gatt_client_characteristic_t notify_char;
static bool     notify_char_found  = false;

// Per-service candidates
static uint16_t tmp_write_value_handle = 0;
static bool     tmp_write_is_wo_resp   = false;
static gatt_client_characteristic_t tmp_notify_char;
static bool     tmp_notify_char_found  = false;
static bool     tmp_exact_match        = false;

// Heuristic fallback (first service with write+notify), used only if no exact UUID match
static bool     fb_valid = false;
static uint16_t fb_write_value_handle = 0;
static bool     fb_write_is_wo_resp = false;
static gatt_client_characteristic_t fb_notify_char;

// Notifications
static gatt_client_notification_t notif_listener;
static bool notif_listening = false;

// Hub init (battery / alerts subscription) progress
static uint8_t hub_init_step = 0;

// Stats
static uint32_t tx_ok = 0, tx_drop = 0;
static int      last_battery_pct = -1;

// Buffer for write-with-response: BTstack keeps the pointer until the write completes,
// so it must not live on the stack (v1 passed a stack buffer).
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
    notify_char_found = false;
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

    ble_set_state(BLE_OFF);
}

static inline bool ble_ready(void) {
    return (ble_state == BLE_READY) &&
           (conn_handle != HCI_CON_HANDLE_INVALID) &&
           (write_value_handle != 0);
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
        ble_set_state(BLE_READY);
    }
}

static void commit_selection(uint16_t wh, bool wo_resp, const gatt_client_characteristic_t *nc,
                             const char *how) {
    write_value_handle = wh;
    write_is_wo_resp = wo_resp;
    notify_char = *nc;
    notify_char_found = true;
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

            // Heuristic candidates (same as v1)
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
                    ble_set_state(BLE_READY);
                    printf("[BLE] READY\n");
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
    if (len < 3) return;

    switch (v[2]) {
        case 0x01: // Hub property update: [len][0][01][prop][06][payload]
            if (len >= 6 && v[3] == 0x06 && v[4] == 0x06) {
                int pct = v[5];
                if (pct != last_battery_pct) {
                    printf("[HUB] battery %d%%\n", pct);
                    last_battery_pct = pct;
                }
            }
            break;

        case 0x03: // Hub alert: [len][0][03][type][04][00=ok / FF=alert]
            if (len >= 6 && v[4] == 0x04) {
                const char *name = (v[3] == 0x01) ? "LOW VOLTAGE" :
                                   (v[3] == 0x02) ? "HIGH CURRENT" :
                                   (v[3] == 0x03) ? "LOW SIGNAL" :
                                   (v[3] == 0x04) ? "OVER POWER" : "?";
                printf("[HUB] alert %s: %s\n", name, v[5] ? "ACTIVE" : "cleared");
            }
            break;

        case 0x04: // Hub attached I/O: [len][0][04][port][event][io_type LE]
            if (len >= 5) {
                uint16_t io = (len >= 7) ? (uint16_t)(v[5] | (v[6] << 8)) : 0;
                printf("[HUB] port 0x%02x %s io_type=0x%04x\n", v[3],
                       v[4] == 0 ? "detached" : (v[4] == 1 ? "attached" : "virtual"), io);
            }
            break;

        case 0x05: // Generic error: [len][0][05][cmd_type][error_code]
            if (len >= 5) printf("[HUB] ERROR cmd=0x%02x code=0x%02x\n", v[3], v[4]);
            break;

        default:
            break;  // 0x82 port output feedback etc. ignored
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

// Returns true only if BTstack accepted the packet. Callers keep their "dirty" state on failure
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

static void set_drive(uint8_t port, out_cache_t *c, int32_t power, uint32_t now) {
    if (!out_need_send(c, power, 1, now)) return;
    uint8_t msg[8];
    uint16_t len = lwp3_start_power(port, (int8_t)power, msg);
    if (write_lwp3(msg, len)) out_mark_sent(c, power, now);
}

static void set_steer(int32_t motor_deg, uint32_t now) {
    if (!out_need_send(&out_steer, motor_deg, STEER_RESEND_DEG, now)) return;
    uint8_t msg[14];
    uint16_t len = lwp3_goto_abs(LEGO_PORT_D, motor_deg,
                                 (int8_t)STEER_GOTO_SPEED, (int8_t)STEER_GOTO_MAXPOWER,
                                 (uint8_t)STEER_ENDSTATE, msg);
    if (write_lwp3(msg, len)) out_mark_sent(&out_steer, motor_deg, now);
}

// Subscribe to battery % and low-voltage / high-current alerts (diagnostics for brown-outs).
static void hub_init_service(void) {
    while (hub_init_step < 3) {
        uint8_t msg[5];
        uint16_t len = 0;
        switch (hub_init_step) {
            case 0: len = lwp3_hub_prop_enable_updates(0x06, msg); break;  // battery voltage %
            case 1: len = lwp3_hub_alert_enable(0x01, msg); break;         // low voltage
            default: len = lwp3_hub_alert_enable(0x02, msg); break;        // high current
        }
        if (!write_lwp3(msg, len)) return;   // retry next tick
        hub_init_step++;
    }
}

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

    pwm_sample_t thr = pwm_sample(&pwm_thr);
    pwm_sample_t str = pwm_sample(&pwm_str);
    pwm_sample_t en  = pwm_sample(&pwm_en);

    bool enable_high  = en.fresh && (en.pulse_us >= ENABLE_THRESHOLD_US);
    bool armed        = thr.fresh && str.fresh && enable_high;
    bool transport_ok = ble_ready();

    // Safe defaults: stop + center
    int32_t thr_cmd   = 0;
    int32_t steer_cmd = 0;

    if (armed) {
        thr_cmd = map_pwm(thr.pulse_us, THROTTLE_MAX_POWER);
        int32_t stick = map_pwm(str.pulse_us, 1000);   // [-1000..1000] for resolution
        steer_cmd = (stick * STEER_TARGET_DEG_MAX * STEER_GEAR_RATIO) / 1000;
        steer_cmd = clamp_i32(steer_cmd, -STEER_MOTOR_DEG_LIMIT, STEER_MOTOR_DEG_LIMIT);
    }

    if (transport_ok) {
        // drive first: stop commands have priority over steering
        set_drive(LEGO_PORT_A, &out_a, thr_cmd, now);
        set_drive(LEGO_PORT_B, &out_b, thr_cmd, now);
        set_steer(steer_cmd, now);
        hub_init_service();
    }

    if ((++diag_counter % CONTROL_HZ) == 0) {
        printf("[DBG] ble=%d st=%d | thr=%u%s str=%u%s en=%u%s | armed=%d | thr_cmd=%ld steer=%ldmdeg | tx ok=%lu drop=%lu\n",
               (int)transport_ok, (int)ble_state,
               (unsigned)thr.pulse_us, thr.fresh ? "" : "(stale)",
               (unsigned)str.pulse_us, str.fresh ? "" : "(stale)",
               (unsigned)en.pulse_us,  en.fresh  ? "" : "(stale)",
               (int)armed, (long)thr_cmd, (long)steer_cmd,
               (unsigned long)tx_ok, (unsigned long)tx_drop);
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

    printf("\n=== LEGO DBW Pico W v2 ===\n");
    if (watchdog_caused_reboot()) printf("[BOOT] *** previous run was reset by WATCHDOG ***\n");

    // PWM inputs
    const uint pins[3] = { GPIO_PWM_THROTTLE, GPIO_PWM_STEERING, GPIO_PWM_ENABLE };
    for (int i = 0; i < 3; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_down(pins[i]);
    }
    gpio_set_irq_enabled_with_callback(GPIO_PWM_THROTTLE,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pwm_gpio_irq);
    gpio_set_irq_enabled(GPIO_PWM_STEERING, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(GPIO_PWM_ENABLE,   GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    if (cyw43_arch_init()) {
        printf("ERROR: cyw43_arch_init failed\n");
        while (1) sleep_ms(1000);
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(100);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

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
