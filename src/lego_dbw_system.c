// lego_dbw_system.c
// Pico W PWM -> BLE LEGO Technic Hub Drive-by-Wire Gateway
//
// FIX: Steering auto-centers WITHOUT needing position feedback
// - Drive (Port A/B): StartPower (same as your working behavior)
// - Steering (Port D): Port Output Command "GotoAbsolutePosition" (0x0D) so when stick returns to 0 => wheel returns to center
//
// IMPORTANT: We use robust GATT discovery:
// - Discover all primary services
// - For each service: discover all characteristics
// - Select the FIRST service that contains BOTH:
//     * a WRITE/WRITE_WO_RESP characteristic (we send to this)
//     * a NOTIFY characteristic (even if we don't parse it, it proves we found the correct LPF2 UART service)
//
// Build: Pico SDK 2.2.x
// Link libs: pico_stdlib hardware_gpio hardware_timer pico_btstack_ble pico_btstack_cyw43 pico_cyw43_arch_poll
// Ensure btstack_config.h exists in project root.

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
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

// Safety
#define PWM_TIMEOUT_MS      250
#define ENABLE_THRESHOLD_US 1600

// Control loop
#define CONTROL_HZ          25
#define CONTROL_PERIOD_MS   (1000 / CONTROL_HZ)

// Output scaling
#define THROTTLE_MAX_POWER  100     // [-100..100]
#define STEER_STICK_MAX     60      // stick map [-60..60]

// Steering "servo" config (GotoAbsolutePosition)
#define STEER_TARGET_DEG_MAX   35   // tune
#define STEER_GOTO_SPEED       60   // 1..100
#define STEER_GOTO_MAXPOWER    80   // 0..100
#define STEER_ENDSTATE_HOLD    126  // 126=HOLD, 127=BRAKE, 0=FLOAT

// Hub address
static const bd_addr_t HUB_ADDR = { 0x90, 0x84, 0x2B, 0x7F, 0xD4, 0x40 };
static const bd_addr_type_t HUB_ADDR_TYPE = BD_ADDR_TYPE_LE_PUBLIC;

// LEGO port IDs (A=0, B=1, C=2, D=3 typical)
#define LEGO_PORT_A  0x00
#define LEGO_PORT_B  0x01
#define LEGO_PORT_D  0x03

#define DEBUG_PRINT_ADV_REPORTS  0

// ============================================================
// PWM CAPTURE (IRQ-safe)
// ============================================================

typedef struct {
    volatile uint32_t rise_us;
    volatile uint16_t pulse_us;
    volatile uint32_t last_update_us;
    volatile bool     have_pulse;
} pwm_chan_t;

static pwm_chan_t pwm_thr = {0};
static pwm_chan_t pwm_str = {0};
static pwm_chan_t pwm_en  = {0};

static inline uint32_t now_us_32(void) { return time_us_32(); }

static void pwm_gpio_irq(uint gpio, uint32_t events) {
    uint32_t t = now_us_32();
    pwm_chan_t *ch = NULL;

    if (gpio == GPIO_PWM_THROTTLE) ch = &pwm_thr;
    else if (gpio == GPIO_PWM_STEERING) ch = &pwm_str;
    else if (gpio == GPIO_PWM_ENABLE) ch = &pwm_en;
    else return;

    if (events & GPIO_IRQ_EDGE_RISE) {
        ch->rise_us = t;
    }
    if (events & GPIO_IRQ_EDGE_FALL) {
        uint32_t dt = t - ch->rise_us;
        if (dt < 3000) {
            ch->pulse_us = (uint16_t)dt;
            ch->last_update_us = t;
            ch->have_pulse = true;
        }
    }
}

static inline bool pwm_fresh(const pwm_chan_t *ch, uint32_t timeout_ms) {
    if (!ch->have_pulse) return false;
    uint32_t age_us = now_us_32() - ch->last_update_us;
    return age_us <= (timeout_ms * 1000u);
}

static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int map_pwm_to_power(uint16_t us, int max_abs_power) {
    int d = (int)us - PWM_NEUTRAL_US;
    if (d > -PWM_DEADBAND_US && d < PWM_DEADBAND_US) return 0;

    int x = clamp_int((int)us, PWM_MIN_US, PWM_MAX_US);
    int num = (x - PWM_NEUTRAL_US) * max_abs_power;
    int den = (PWM_MAX_US - PWM_NEUTRAL_US); // 500
    int p = num / den;
    return clamp_int(p, -max_abs_power, +max_abs_power);
}

// ============================================================
// LWP3 PACKETS
// ============================================================

// StartPower via WriteDirectModeData
// [Len][Hub=0][0x81][Port][Startup/Completion][0x51][Mode=0][Power int8]
static uint16_t build_lwp3_start_power(uint8_t port, int8_t power, uint8_t out[8]) {
    out[0] = 8;
    out[1] = 0x00;
    out[2] = 0x81;
    out[3] = port;
    out[4] = 0x11;   // execute immediately
    out[5] = 0x51;   // WriteDirectModeData
    out[6] = 0x00;   // mode 0
    out[7] = (uint8_t)power;
    return 8;
}

// Port Output Command: GotoAbsolutePosition (0x0D) - FULL form including UseProfile
// Payload: [0x0D][AbsPos int32 LE][Speed int8][MaxPower int8][EndState int8][UseProfile int8]
// Total length = 14 bytes
static uint16_t build_lwp3_goto_abs_pos(uint8_t port, int32_t abs_pos_deg, int8_t speed, int8_t max_power, int8_t end_state, uint8_t out[14]) {
    out[0]  = 14;
    out[1]  = 0x00;
    out[2]  = 0x81;
    out[3]  = port;
    out[4]  = 0x11;   // execute immediately
    out[5]  = 0x0D;   // GotoAbsolutePosition
    out[6]  = (uint8_t)((uint32_t)abs_pos_deg & 0xFF);
    out[7]  = (uint8_t)(((uint32_t)abs_pos_deg >> 8) & 0xFF);
    out[8]  = (uint8_t)(((uint32_t)abs_pos_deg >> 16) & 0xFF);
    out[9]  = (uint8_t)(((uint32_t)abs_pos_deg >> 24) & 0xFF);
    out[10] = (uint8_t)speed;       // 1..100
    out[11] = (uint8_t)max_power;   // 0..100
    out[12] = (uint8_t)end_state;   // 126 hold / 127 brake / 0 float
    out[13] = 0x00;                 // UseProfile = 0
    return 14;
}

// ============================================================
// BLE CENTRAL STATE MACHINE (robust discovery)
// ============================================================

typedef enum {
    BLE_OFF = 0,
    BLE_SCANNING,
    BLE_CONNECTING,
    BLE_W4_SERVICES,
    BLE_W4_CHARS,
    BLE_READY
} ble_state_t;

static ble_state_t ble_state = BLE_OFF;
static hci_con_handle_t conn_handle = HCI_CON_HANDLE_INVALID;
static btstack_packet_callback_registration_t hci_event_reg;

#define MAX_SERVICES 24
static gatt_client_service_t services[MAX_SERVICES];
static uint8_t service_count = 0;
static uint8_t service_index = 0;
static gatt_client_service_t current_service;

// Selected handles must be in SAME service
static uint16_t write_value_handle = 0;
static bool     write_is_wo_resp   = true;
static gatt_client_characteristic_t notify_char;
static bool notify_char_found = false;

// Temporary candidates during discovery of one service
static uint16_t tmp_write_value_handle = 0;
static bool     tmp_write_is_wo_resp   = false;
static gatt_client_characteristic_t tmp_notify_char;
static bool tmp_notify_char_found = false;

static inline void ble_reset_context(void) {
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
    memset(&tmp_notify_char, 0, sizeof(tmp_notify_char));

    ble_state = BLE_OFF;
}

static inline bool ble_ready(void) {
    return (ble_state == BLE_READY) &&
           (conn_handle != HCI_CON_HANDLE_INVALID) &&
           (write_value_handle != 0) &&
           (notify_char_found);
}

static inline bool adv_is_target(const bd_addr_t addr) {
    return memcmp(addr, HUB_ADDR, sizeof(bd_addr_t)) == 0;
}

static void start_scan(void) {
    gap_set_scan_parameters(1, 0x0030, 0x0030);
    gap_set_scan_duplicate_filter(true);
    gap_start_scan();
    ble_state = BLE_SCANNING;
    printf("[BLE] scanning...\n");
}

static void connect_to_hub(void) {
    gap_stop_scan();
    ble_state = BLE_CONNECTING;
    printf("[BLE] connecting to %02X:%02X:%02X:%02X:%02X:%02X type=%u...\n",
           HUB_ADDR[0], HUB_ADDR[1], HUB_ADDR[2], HUB_ADDR[3], HUB_ADDR[4], HUB_ADDR[5],
           (unsigned)HUB_ADDR_TYPE);
    gap_connect(HUB_ADDR, HUB_ADDR_TYPE);
}

static void gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void gatt_start_service_discovery(void) {
    service_count = 0;
    service_index = 0;
    write_value_handle = 0;
    notify_char_found = false;

    ble_state = BLE_W4_SERVICES;
    printf("[GATT] discover ALL primary services...\n");
    gatt_client_discover_primary_services(gatt_handler, conn_handle);
}

static void gatt_start_char_discovery_for_index(uint8_t idx) {
    if (idx >= service_count) {
        printf("[GATT] No service contained BOTH write+notify -> disconnect/rescan\n");
        if (conn_handle != HCI_CON_HANDLE_INVALID) gap_disconnect(conn_handle);
        return;
    }

    current_service = services[idx];

    tmp_write_value_handle = 0;
    tmp_write_is_wo_resp = false;
    tmp_notify_char_found = false;
    memset(&tmp_notify_char, 0, sizeof(tmp_notify_char));

    ble_state = BLE_W4_CHARS;
    printf("[GATT] discover chars in service #%u (0x%04x-0x%04x)\n",
           (unsigned)idx,
           current_service.start_group_handle,
           current_service.end_group_handle);

    gatt_client_discover_characteristics_for_service(gatt_handler, conn_handle, &current_service);
}

static void gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t ev = hci_event_packet_get_type(packet);

    switch (ev) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            gatt_client_service_t s;
            gatt_event_service_query_result_get_service(packet, &s);

            if (service_count < MAX_SERVICES) {
                services[service_count++] = s;
                printf("[GATT] service #%u: 0x%04x-0x%04x\n",
                       (unsigned)(service_count - 1),
                       s.start_group_handle, s.end_group_handle);
            }
            break;
        }

        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t c;
            gatt_event_characteristic_query_result_get_characteristic(packet, &c);

            // Candidate WRITE characteristic
            if (tmp_write_value_handle == 0) {
                if (c.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) {
                    tmp_write_value_handle = c.value_handle;
                    tmp_write_is_wo_resp = true;
                    printf("[GATT]   write_wo_resp candidate: 0x%04x\n", tmp_write_value_handle);
                } else if (c.properties & ATT_PROPERTY_WRITE) {
                    tmp_write_value_handle = c.value_handle;
                    tmp_write_is_wo_resp = false;
                    printf("[GATT]   write candidate: 0x%04x\n", tmp_write_value_handle);
                }
            }

            // Candidate NOTIFY characteristic
            if (!tmp_notify_char_found && (c.properties & ATT_PROPERTY_NOTIFY)) {
                tmp_notify_char = c;
                tmp_notify_char_found = true;
                printf("[GATT]   notify candidate: 0x%04x\n", c.value_handle);
            }
            break;
        }

        case GATT_EVENT_QUERY_COMPLETE: {
            uint8_t att_status = gatt_event_query_complete_get_att_status(packet);
            if (att_status != ATT_ERROR_SUCCESS) {
                printf("[GATT] query failed att=0x%02x -> disconnect\n", att_status);
                if (conn_handle != HCI_CON_HANDLE_INVALID) gap_disconnect(conn_handle);
                return;
            }

            if (ble_state == BLE_W4_SERVICES) {
                if (service_count == 0) {
                    printf("[GATT] no services found -> disconnect\n");
                    if (conn_handle != HCI_CON_HANDLE_INVALID) gap_disconnect(conn_handle);
                    return;
                }
                printf("[GATT] services=%u; searching service with write+notify...\n", (unsigned)service_count);
                service_index = 0;
                gatt_start_char_discovery_for_index(service_index);
                return;
            }

            if (ble_state == BLE_W4_CHARS) {
                if (tmp_write_value_handle != 0 && tmp_notify_char_found) {
                    write_value_handle = tmp_write_value_handle;
                    write_is_wo_resp = tmp_write_is_wo_resp;
                    notify_char = tmp_notify_char;
                    notify_char_found = true;

                    ble_state = BLE_READY;
                    printf("[BLE] READY ✅ write=0x%04x (%s) notify=0x%04x\n",
                           write_value_handle,
                           write_is_wo_resp ? "WO_RESP" : "WRITE",
                           notify_char.value_handle);
                    return;
                }

                // Try next service
                service_index++;
                gatt_start_char_discovery_for_index(service_index);
                return;
            }
            break;
        }

        default:
            break;
    }
}

// ============================================================
// HCI event handler
// ============================================================

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t ev = hci_event_packet_get_type(packet);

    switch (ev) {
        case BTSTACK_EVENT_STATE: {
            int st = btstack_event_state_get_state(packet);
            printf("[HCI] state=%d\n", st);
            if (st == HCI_STATE_WORKING) {
                ble_reset_context();
                start_scan();
            }
            break;
        }

        case GAP_EVENT_ADVERTISING_REPORT: {
            bd_addr_t addr;
            bd_addr_type_t type = gap_event_advertising_report_get_address_type(packet);
            gap_event_advertising_report_get_address(packet, addr);
            int8_t rssi = gap_event_advertising_report_get_rssi(packet);

#if DEBUG_PRINT_ADV_REPORTS
            printf("[ADV] %02X:%02X:%02X:%02X:%02X:%02X type=%u rssi=%d\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
                   (unsigned)type, (int)rssi);
#endif

            if (ble_state == BLE_SCANNING && adv_is_target(addr)) {
                printf("[BLE] hub seen (type=%u rssi=%d) -> connect\n", (unsigned)type, (int)rssi);
                connect_to_hub();
            }
            break;
        }

        case HCI_EVENT_META_GAP: {
            uint8_t sub = hci_event_gap_meta_get_subevent_code(packet);
            if (sub == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
                uint8_t status = gap_subevent_le_connection_complete_get_status(packet);
                if (status != ERROR_CODE_SUCCESS) {
                    printf("[BLE] connect failed status=0x%02x -> rescan\n", status);
                    ble_reset_context();
                    start_scan();
                    return;
                }

                conn_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
                printf("[BLE] connected handle=0x%04x\n", conn_handle);

                gatt_start_service_discovery();
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            printf("[BLE] disconnected (0x%02x) -> rescan\n", reason);
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

static inline void write_lwp3(const uint8_t *msg, uint16_t len) {
    if (!ble_ready()) return;

    if (write_is_wo_resp) {
        gatt_client_write_value_of_characteristic_without_response(
            conn_handle, write_value_handle, len, msg);
    } else {
        gatt_client_write_value_of_characteristic(
            gatt_handler, conn_handle, write_value_handle, len, msg);
    }
}

static void send_drive_power(uint8_t port, int8_t power) {
    uint8_t msg[8];
    uint16_t len = build_lwp3_start_power(port, power, msg);
    write_lwp3(msg, len);
}

static void send_steer_goto_abs(int32_t target_deg) {
    uint8_t msg[14];
    uint16_t len = build_lwp3_goto_abs_pos(
        LEGO_PORT_D,
        target_deg,
        (int8_t)STEER_GOTO_SPEED,
        (int8_t)STEER_GOTO_MAXPOWER,
        (int8_t)STEER_ENDSTATE_HOLD,
        msg
    );
    write_lwp3(msg, len);
}

static void safe_stop(void) {
    send_drive_power(LEGO_PORT_A, 0);
    send_drive_power(LEGO_PORT_B, 0);

    // actively return steering to center
    send_steer_goto_abs(0);
}

// ============================================================
// CONTROL LOOP
// ============================================================

static uint32_t diag_counter = 0;

static bool control_timer_cb(struct repeating_timer *t) {
    (void)t;

    cyw43_arch_poll();

    bool ok_thr = pwm_fresh(&pwm_thr, PWM_TIMEOUT_MS);
    bool ok_str = pwm_fresh(&pwm_str, PWM_TIMEOUT_MS);
    bool ok_en  = pwm_fresh(&pwm_en,  PWM_TIMEOUT_MS);

    bool enable_high = ok_en && (pwm_en.pulse_us >= ENABLE_THRESHOLD_US);
    bool transport_ok = ble_ready();

    diag_counter++;
    if ((diag_counter % CONTROL_HZ) == 0) {
        int stick = ok_str ? map_pwm_to_power(pwm_str.pulse_us, STEER_STICK_MAX) : 0;
        int target_deg = (stick * STEER_TARGET_DEG_MAX) / STEER_STICK_MAX;

        printf("[DBG] ble=%d write=0x%04x | thr_us=%u str_us=%u en_us=%u | en_high=%d | thr=%d | steer_target=%ddeg\n",
               (int)transport_ok,
               (unsigned)write_value_handle,
               (unsigned)pwm_thr.pulse_us,
               (unsigned)pwm_str.pulse_us,
               (unsigned)pwm_en.pulse_us,
               (int)enable_high,
               ok_thr ? map_pwm_to_power(pwm_thr.pulse_us, THROTTLE_MAX_POWER) : 0,
               target_deg);
    }

    if (!(ok_thr && ok_str && enable_high && transport_ok)) {
        safe_stop();
        return true;
    }

    // Drive
    int thr = map_pwm_to_power(pwm_thr.pulse_us, THROTTLE_MAX_POWER);
    send_drive_power(LEGO_PORT_A, (int8_t)thr);
    send_drive_power(LEGO_PORT_B, (int8_t)thr);

    // Steering auto-center
    int stick = map_pwm_to_power(pwm_str.pulse_us, STEER_STICK_MAX);
    int32_t target_deg = (stick * STEER_TARGET_DEG_MAX) / STEER_STICK_MAX;
    send_steer_goto_abs(target_deg);

    return true;
}

// ============================================================
// MAIN
// ============================================================

int main(void) {
    stdio_init_all();

    // Wait for USB CDC enumerate
    absolute_time_t t0 = get_absolute_time();
    while (!stdio_usb_connected()) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > 5 * 1000 * 1000) break;
        sleep_ms(50);
    }
    sleep_ms(200);

    printf("\n=== LEGO DBW Pico W Boot ===\n");
    printf("PWM pins: THR=%d STR=%d EN=%d\n", GPIO_PWM_THROTTLE, GPIO_PWM_STEERING, GPIO_PWM_ENABLE);

    // PWM GPIO init
    gpio_init(GPIO_PWM_THROTTLE); gpio_set_dir(GPIO_PWM_THROTTLE, GPIO_IN); gpio_pull_down(GPIO_PWM_THROTTLE);
    gpio_init(GPIO_PWM_STEERING); gpio_set_dir(GPIO_PWM_STEERING, GPIO_IN); gpio_pull_down(GPIO_PWM_STEERING);
    gpio_init(GPIO_PWM_ENABLE);   gpio_set_dir(GPIO_PWM_ENABLE,   GPIO_IN); gpio_pull_down(GPIO_PWM_ENABLE);

    gpio_set_irq_enabled_with_callback(GPIO_PWM_THROTTLE,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pwm_gpio_irq);
    gpio_set_irq_enabled(GPIO_PWM_STEERING,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(GPIO_PWM_ENABLE,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // CYW43 init
    if (cyw43_arch_init()) {
        printf("ERROR: cyw43_arch_init failed\n");
        while (1) sleep_ms(1000);
    }

    // Blink LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(100);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // BTstack init
    l2cap_init();
    sm_init();
    gatt_client_init();

    hci_event_reg.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_reg);

    printf("[BOOT] hci_power_control(HCI_POWER_ON)\n");
    hci_power_control(HCI_POWER_ON);

    // Control loop timer
    static struct repeating_timer control_timer;
    add_repeating_timer_ms(-CONTROL_PERIOD_MS, control_timer_cb, NULL, &control_timer);

    // Fallback start scan
    sleep_ms(1000);
    if (ble_state == BLE_OFF) start_scan();

    btstack_run_loop_execute();
    return 0;
}
