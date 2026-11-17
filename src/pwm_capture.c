// pwm_capture.c — RC PWM pulse-width capture with PIO state machines.

#include "pwm_capture.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pwm_capture.pio.h"

#define CAPTURE_SM_HZ 2000000.0f    // 2 MHz SM clock, 2 cycles per loop -> 1 us per count

static PIO  s_pio;
static uint s_sm[PWM_CAPTURE_CHANNELS];
static uint s_offset;
static bool s_ready = false;

bool pwm_capture_init(const unsigned pins[PWM_CAPTURE_CHANNELS]) {
    uint sm0;
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&pwm_capture_program, &s_pio, &sm0,
                                                          &s_offset, pins[0], 1, true)) {
        return false;
    }
    s_sm[0] = sm0;

    for (int i = 1; i < PWM_CAPTURE_CHANNELS; i++) {
        int sm = pio_claim_unused_sm(s_pio, false);
        if (sm < 0) {
            for (int j = 1; j < i; j++) pio_sm_unclaim(s_pio, s_sm[j]);
            pio_remove_program_and_unclaim_sm(&pwm_capture_program, s_pio, sm0, s_offset);
            return false;
        }
        s_sm[i] = (uint)sm;
    }

    float div = (float)clock_get_hz(clk_sys) / CAPTURE_SM_HZ;

    for (int i = 0; i < PWM_CAPTURE_CHANNELS; i++) {
        pio_sm_config c = pwm_capture_program_get_default_config(s_offset);
        sm_config_set_in_pins(&c, pins[i]);
        sm_config_set_jmp_pin(&c, pins[i]);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);   // 8-deep RX FIFO
        sm_config_set_clkdiv(&c, div);
        pio_sm_set_consecutive_pindirs(s_pio, s_sm[i], pins[i], 1, false);
        pio_sm_init(s_pio, s_sm[i], s_offset, &c);
        pio_sm_set_enabled(s_pio, s_sm[i], true);
    }
    s_ready = true;
    return true;
}

bool pwm_capture_poll(unsigned ch, uint16_t *pulse_us) {
    if (!s_ready || ch >= PWM_CAPTURE_CHANNELS) return false;
    bool got = false;
    uint32_t v = 0;
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm[ch])) {
        v = pio_sm_get(s_pio, s_sm[ch]);
        got = true;
    }
    if (got) *pulse_us = (v > 0xFFFF) ? 0xFFFF : (uint16_t)v;
    return got;
}
