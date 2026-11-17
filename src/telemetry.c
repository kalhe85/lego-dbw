// telemetry.c — MAVLink telemetry to the flight controller over UART0 (TX only).
//
// Never blocks: frames go into a ring buffer that is drained into the UART FIFO without
// waiting. If the buffer is full, the frame is dropped.

#include "telemetry.h"

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "mavlink_lite.h"

#define TELEM_UART    uart0
#define RING_SIZE     512

static uint8_t  ring[RING_SIZE];
static uint16_t ring_head = 0, ring_tail = 0;   // head = write, tail = read
static mavlite_t mav;
static uint32_t last_hb_ms = 0;
static uint32_t tick_count = 0;
static uint8_t  value_index = 0;
static bool     ready = false;

static uint16_t ring_free(void) {
    return (uint16_t)((ring_tail + RING_SIZE - ring_head - 1) % RING_SIZE);
}

static void ring_put(const uint8_t *data, size_t len) {
    if (len > ring_free()) return;     // drop whole frame rather than send half of it
    for (size_t i = 0; i < len; i++) {
        ring[ring_head] = data[i];
        ring_head = (uint16_t)((ring_head + 1) % RING_SIZE);
    }
}

static void ring_drain(void) {
    while (ring_tail != ring_head && uart_is_writable(TELEM_UART)) {
        uart_putc_raw(TELEM_UART, (char)ring[ring_tail]);
        ring_tail = (uint16_t)((ring_tail + 1) % RING_SIZE);
    }
}

void telem_init(void) {
    uart_init(TELEM_UART, TELEM_BAUD);
    gpio_set_function(TELEM_UART_TX_GPIO, GPIO_FUNC_UART);
    uart_set_format(TELEM_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(TELEM_UART, true);
    mavlite_init(&mav, TELEM_SYSID, TELEM_COMPID);
    ready = true;
}

void telem_event(uint8_t severity, const char *text) {
    if (!ready) return;
    char msg[51];
    snprintf(msg, sizeof(msg), "DBW: %s", text);
    uint8_t frame[MAVLITE_MAX_FRAME];
    size_t n = mavlite_statustext(&mav, frame, severity, msg);
    ring_put(frame, n);
    ring_drain();
}

static void send_value(uint32_t now_ms, const char *name, float value) {
    uint8_t frame[MAVLITE_MAX_FRAME];
    size_t n = mavlite_named_value_float(&mav, frame, now_ms, name, value);
    ring_put(frame, n);
}

void telem_tick(uint32_t now_ms, const telem_snapshot_t *s) {
    if (!ready) return;
    uint8_t frame[MAVLITE_MAX_FRAME];

    if ((now_ms - last_hb_ms) >= 1000) {
        last_hb_ms = now_ms;
        size_t n = mavlite_heartbeat(&mav, frame, MAV_TYPE_ONBOARD_CONTROLLER,
                                     MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        ring_put(frame, n);
    }

    // One NAMED_VALUE_FLOAT every TELEM_VALUE_PERIOD_TICKS, round-robin over the fields.
    // Names are limited to 10 characters.
    if ((++tick_count % TELEM_VALUE_PERIOD_TICKS) == 0) {
        switch (value_index) {
            case 0: send_value(now_ms, "DBW_LINK",  s->link_ready ? 1.0f : 0.0f); break;
            case 1: send_value(now_ms, "DBW_ARM",   s->armed ? 1.0f : 0.0f); break;
            case 2: send_value(now_ms, "DBW_THR",   (float)s->thr_cmd); break;
            case 3: send_value(now_ms, "DBW_STRCMD", (float)s->steer_rel_deg); break;
            case 4:
                if (s->steer_pos_deg != INT32_MIN) {
                    send_value(now_ms, "DBW_STRPOS", (float)s->steer_pos_deg);
                }
                break;
            case 5: send_value(now_ms, "DBW_SPEED", s->speed_dps); break;
            case 6:
                if (s->battery_pct >= 0) send_value(now_ms, "DBW_BATT", (float)s->battery_pct);
                break;
            case 7: send_value(now_ms, "DBW_CAL",   (float)s->cal_state); break;
            default: send_value(now_ms, "DBW_DROP", (float)s->tx_drop); break;
        }
        value_index = (uint8_t)((value_index + 1) % 9);
    }

    ring_drain();
}
