// drive.h — drive-speed estimation from motor position reports (no hardware dependencies).

#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include <stdbool.h>

// The hub reports the drive motor's position every DRIVE_POS_DELTA_DEG degrees. Report arrival
// times are bunched by the BLE connection interval, so speed is computed over a window of at
// least window_ms rather than between consecutive reports.
typedef struct {
    bool     have;
    int32_t  anchor_pos;
    uint32_t anchor_ms;
    uint32_t last_ms;
    float    dps;          // last computed speed, motor degrees per second
} speed_est_t;

void  speed_est_reset(speed_est_t *e);
void  speed_est_update(speed_est_t *e, int32_t pos, uint32_t now_ms, uint32_t window_ms);
// Returns 0 when no report has arrived for timeout_ms (motor stopped or feedback lost).
float speed_est_get(const speed_est_t *e, uint32_t now_ms, uint32_t timeout_ms);

#endif // DRIVE_H
