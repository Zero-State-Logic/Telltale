/*
 * telltale_motion.c - commanded servo trajectories (portable C99).
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#include "telltale_motion.h"

#include <string.h>

#define SERVO_MIN_DUTY 1638u   /* 0.5 ms of a 20 ms period at 16 bits */
#define SERVO_MAX_DUTY 8192u   /* 2.5 ms */

static uint32_t ramp_ms(float from_deg, float to_deg)
{
    const float d = (to_deg > from_deg) ? (to_deg - from_deg) : (from_deg - to_deg);
    return (uint32_t)(d * (float)TT_RAMP_MS_PER_DEG + 0.5f);
}

static void add_seg(tt_motion_t *m, float target_deg, uint32_t ms)
{
    if (m->nseg < TT_MOTION_MAX_SEG) {
        m->seg[m->nseg].target_deg = target_deg;
        m->seg[m->nseg].ms = ms;
        m->nseg++;
    }
}

void tt_motion_unlock(tt_motion_t *m)
{
    if (m == 0) {
        return;
    }
    memset(m, 0, sizeof(*m));
    m->start_deg = TT_ANGLE_LOCKED;
    add_seg(m, TT_ANGLE_OVERSHOOT, ramp_ms(TT_ANGLE_LOCKED, TT_ANGLE_OVERSHOOT));
    add_seg(m, TT_ANGLE_OVERSHOOT, TT_OVERSHOOT_HOLD_MS);
    add_seg(m, TT_ANGLE_UNLOCKED, ramp_ms(TT_ANGLE_OVERSHOOT, TT_ANGLE_UNLOCKED));
    m->dwell_ms = TT_DWELL_MS;
}

void tt_motion_relock(tt_motion_t *m)
{
    if (m == 0) {
        return;
    }
    memset(m, 0, sizeof(*m));
    m->start_deg = TT_ANGLE_UNLOCKED;
    add_seg(m, TT_ANGLE_SEAT, ramp_ms(TT_ANGLE_UNLOCKED, TT_ANGLE_SEAT));
    add_seg(m, TT_ANGLE_SEAT, TT_OVERSHOOT_HOLD_MS);
    add_seg(m, TT_ANGLE_LOCKED, ramp_ms(TT_ANGLE_SEAT, TT_ANGLE_LOCKED));
    m->dwell_ms = TT_DWELL_MS;
}

void tt_motion_ramp(tt_motion_t *m, float from_deg, float to_deg)
{
    if (m == 0) {
        return;
    }
    memset(m, 0, sizeof(*m));
    m->start_deg = from_deg;
    add_seg(m, to_deg, ramp_ms(from_deg, to_deg));
    m->dwell_ms = TT_DWELL_MS;
}

uint32_t tt_motion_motion_ms(const tt_motion_t *m)
{
    uint32_t i;
    uint32_t total = 0u;
    if (m == 0) {
        return 0u;
    }
    for (i = 0; i < m->nseg; ++i) {
        total += m->seg[i].ms;
    }
    return total;
}

uint32_t tt_motion_total_ms(const tt_motion_t *m)
{
    return (m == 0) ? 0u : tt_motion_motion_ms(m) + m->dwell_ms;
}

float tt_motion_angle_at(const tt_motion_t *m, uint32_t t_ms)
{
    uint32_t i;
    float from;
    if (m == 0) {
        return 0.0f;
    }
    from = m->start_deg;
    for (i = 0; i < m->nseg; ++i) {
        const tt_motion_seg_t *s = &m->seg[i];
        if (t_ms < s->ms) {
            const float frac = (float)t_ms / (float)s->ms;
            return from + (s->target_deg - from) * frac;
        }
        t_ms -= s->ms;
        from = s->target_deg;
    }
    return from;
}

float tt_motion_frame_angle(const tt_motion_t *m, uint32_t t_ms)
{
    const uint32_t frame_start = t_ms - (t_ms % TT_SERVO_FRAME_MS);
    return tt_motion_angle_at(m, frame_start);
}

uint32_t tt_motion_sample(const tt_motion_t *m, uint32_t dt_us, float *out,
                          uint32_t n)
{
    uint32_t i;
    if (m == 0 || out == 0 || dt_us == 0u) {
        return 0u;
    }
    for (i = 0; i < n; ++i) {
        const uint32_t t_ms = (uint32_t)(((uint64_t)i * dt_us) / 1000u);
        out[i] = tt_motion_frame_angle(m, t_ms);
    }
    return n;
}

uint32_t tt_servo_duty(float deg)
{
    float d = deg;
    if (d < 0.0f) {
        d = 0.0f;
    }
    if (d > 180.0f) {
        d = 180.0f;
    }
    return SERVO_MIN_DUTY +
           (uint32_t)((float)(SERVO_MAX_DUTY - SERVO_MIN_DUTY) * d / 180.0f + 0.5f);
}
