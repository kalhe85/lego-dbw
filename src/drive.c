// drive.c — drive-speed estimation from motor position reports.

#include "drive.h"

void speed_est_reset(speed_est_t *e) {
    e->have = false;
    e->anchor_pos = 0;
    e->anchor_ms = 0;
    e->last_ms = 0;
    e->dps = 0.0f;
}

void speed_est_update(speed_est_t *e, int32_t pos, uint32_t now_ms, uint32_t window_ms) {
    if (!e->have) {
        e->have = true;
        e->anchor_pos = pos;
        e->anchor_ms = now_ms;
        e->last_ms = now_ms;
        e->dps = 0.0f;
        return;
    }
    // After a long gap (motor was stopped) start a fresh window instead of averaging the
    // first movement over the idle time.
    if ((now_ms - e->last_ms) > 4 * window_ms) {
        e->anchor_pos = pos;
        e->anchor_ms = now_ms;
        e->last_ms = now_ms;
        e->dps = 0.0f;
        return;
    }
    e->last_ms = now_ms;
    uint32_t dt = now_ms - e->anchor_ms;
    if (dt >= window_ms && dt > 0) {
        e->dps = (float)(pos - e->anchor_pos) * 1000.0f / (float)dt;
        e->anchor_pos = pos;
        e->anchor_ms = now_ms;
    }
}

float speed_est_get(const speed_est_t *e, uint32_t now_ms, uint32_t timeout_ms) {
    if (!e->have) return 0.0f;
    if ((now_ms - e->last_ms) > timeout_ms) return 0.0f;
    return e->dps;
}
