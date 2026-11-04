/*
 * telltale_motion.h - commanded servo trajectories (portable C99).
 *
 * The firmware and the simulator both take the commanded angle from these
 * functions, so the evaluation drives the model with exactly the trajectory
 * the lock uses. Ramps keep the original Phantom Door feel: 8 ms per degree,
 * an overshoot past the unlock angle to break static friction, then a settle
 * back. Relock now "seats" the bolt 3 degrees past the locked angle (the
 * original relock overshoot constant was 170 degrees, a typo for -10).
 *
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#ifndef TELLTALE_MOTION_H
#define TELLTALE_MOTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TT_ANGLE_LOCKED      10.0f   /* deg: bolt thrown, servo holding */
#define TT_ANGLE_UNLOCKED    90.0f   /* deg: bolt retracted */
#define TT_ANGLE_OVERSHOOT   100.0f  /* deg: break-away overshoot on unlock */
#define TT_ANGLE_SEAT        7.0f    /* deg: seat the bolt on relock */
#define TT_RAMP_MS_PER_DEG   8u      /* soft ramp, as in the original sketch */
#define TT_OVERSHOOT_HOLD_MS 80u
#define TT_DWELL_MS          200u    /* sampled after motion ends (settle) */
#define TT_SERVO_FRAME_MS    20u     /* 50 Hz servo frame */
#define TT_MOTION_MAX_SEG    6u

typedef struct {
    float target_deg;   /* ramp linearly from the previous angle to this one */
    uint32_t ms;        /* over this many milliseconds (0 = jump) */
} tt_motion_seg_t;

typedef struct {
    float start_deg;
    tt_motion_seg_t seg[TT_MOTION_MAX_SEG];
    uint32_t nseg;
    uint32_t dwell_ms;
} tt_motion_t;

void tt_motion_unlock(tt_motion_t *m);
void tt_motion_relock(tt_motion_t *m);
/* A single soft ramp between two angles (used for retreats after a jam). */
void tt_motion_ramp(tt_motion_t *m, float from_deg, float to_deg);

uint32_t tt_motion_motion_ms(const tt_motion_t *m);
uint32_t tt_motion_total_ms(const tt_motion_t *m);
float tt_motion_angle_at(const tt_motion_t *m, uint32_t t_ms);
/* What the servo actually sees: the angle latched at the start of the
 * current 50 Hz frame (the PWM peripheral updates once per period). */
float tt_motion_frame_angle(const tt_motion_t *m, uint32_t t_ms);
/* Fills out[0..n) with the frame-latched angle at t = 0, dt, 2dt, ...
 * (dt in microseconds). Returns the number of values written. */
uint32_t tt_motion_sample(const tt_motion_t *m, uint32_t dt_us, float *out,
                          uint32_t n);

/* 16-bit LEDC duty at 50 Hz for 0.5-2.5 ms pulses over 0-180 degrees. */
uint32_t tt_servo_duty(float deg);

#ifdef __cplusplus
}
#endif

#endif /* TELLTALE_MOTION_H */
