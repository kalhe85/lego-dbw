// pwm_capture.h — RC PWM pulse-width capture with PIO state machines.

#ifndef PWM_CAPTURE_H
#define PWM_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

#define PWM_CAPTURE_CHANNELS 3

// Claims one PIO state machine per pin (after cyw43_arch_init, which also uses PIO).
// Returns false and releases everything if the resources are not available.
bool pwm_capture_init(const unsigned pins[PWM_CAPTURE_CHANNELS]);

// Drains the channel's FIFO. Returns true and the most recent pulse width if at least one new
// sample arrived since the last call.
bool pwm_capture_poll(unsigned ch, uint16_t *pulse_us);

#endif // PWM_CAPTURE_H
