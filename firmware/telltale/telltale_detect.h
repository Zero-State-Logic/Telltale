/*
 * telltale_detect.h - proprioceptive tamper and fault detector for a servo lock.
 *
 * The lock's own servo is the sensor. A shunt monitor (INA219) samples the
 * servo supply current; this module learns the lock's healthy current
 * signature and flags deviations:
 *
 *   - per actuation (unlock and relock learned separately): eight features
 *     (peak, mean, energy, time-to-peak, settle time, plateau, template
 *     residual, load centroid) scored with a diagonal Mahalanobis distance
 *     (time-to-peak reported, not scored by default) against a model
 *     learned from the first N actuations (Welford), adapted afterwards only
 *     when an actuation is clearly healthy (gated EWMA);
 *   - while locked and idle: a clipped two-sided CUSUM (in mA) on the holding current
 *     flags sustained force on the bolt ("forced while locked") or a drop
 *     below the learned idle baseline ("servo power lost"). Each locked
 *     period re-anchors its upward reference on a quiet stretch, within
 *     +/- anchor_max of the baseline, so where the servo happened to stop
 *     inside its deadband does not masquerade as force.
 *
 * Portable C99: fixed-width integers, float32 maths, no heap, no VLAs.
 * The same file is compiled into the ESP32 firmware, the host unit tests and
 * the Python evaluation (through ctypes).
 *
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#ifndef TELLTALE_DETECT_H
#define TELLTALE_DETECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- sizes */
#define TT_NBINS 32u   /* template resolution (bins over the commanded motion) */
#define TT_NFEAT 8u    /* features per actuation */

/* Feature indices. */
#define TT_FEAT_PEAK     0u  /* max current, mA */
#define TT_FEAT_MEAN     1u  /* mean current over the commanded motion, mA */
#define TT_FEAT_ENERGY   2u  /* integral of I^2 over the window, mA^2*s */
#define TT_FEAT_TPEAK    3u  /* time of the maximum, ms from start */
#define TT_FEAT_SETTLE   4u  /* time after motion end until the current is steady, ms */
#define TT_FEAT_PLATEAU  5u  /* mean current over the last half of the dwell, mA */
#define TT_FEAT_RESID    6u  /* RMS distance to the learned motion template, mA */
#define TT_FEAT_CENTROID 7u  /* current-weighted mean time of the motion, ms */

#define TT_FEAT_ALL 0xFFu    /* every feature */
/* Default score mask: time-to-peak is reported but not scored. On a 50 Hz
 * servo the maximum can sit on either of two similar humps (e.g. the start
 * of a relock or its seat push), so it is bimodal on healthy runs; the load
 * centroid carries the timing information without that ambiguity. */
#define TT_FEAT_DEFAULT_MASK (TT_FEAT_ALL & ~(1u << TT_FEAT_TPEAK))

/* Actuation kinds (learned separately). */
#define TT_ACT_UNLOCK 0u
#define TT_ACT_RELOCK 1u
#define TT_ACT_KINDS  2u

/* Actuation status. */
#define TT_STATUS_LEARNING 0
#define TT_STATUS_HEALTHY  1
#define TT_STATUS_ANOMALY  2

/* Fault classes (from the sign pattern of the feature deviations). */
#define TT_FAULT_NONE     0
#define TT_FAULT_STALL    1  /* jammed latch / obstruction */
#define TT_FAULT_LINKAGE  2  /* broken or slack cord/linkage: load drops */
#define TT_FAULT_BROWNOUT 3  /* weak or sagging servo supply */
#define TT_FAULT_FRICTION 4  /* added friction: dry, dirty or iced mechanism */
#define TT_FAULT_UNKNOWN  5  /* anomalous, but no pattern matched */

/* Idle monitor events. */
#define TT_IDLE_NONE       0
#define TT_IDLE_LEARNING   1  /* still learning the idle baseline */
#define TT_IDLE_FORCED     2  /* sustained rise: forced while locked */
#define TT_IDLE_POWER_LOSS 3  /* sustained drop: servo supply or wiring lost */
#define TT_IDLE_BLANKED    4  /* inside the settle/refractory blanking window */
#define TT_IDLE_ANCHORING  5  /* averaging this period's reference */

/* Return codes. */
#define TT_OK         0
#define TT_ERR_ARG   -1
#define TT_ERR_STATE -2
#define TT_ERR_DATA  -3

/* --------------------------------------------------------- configuration */
typedef struct {
    uint16_t learn_runs;           /* healthy actuations per kind before scoring */
    uint16_t resid_warmup;         /* runs before residual statistics start */
    uint32_t feature_mask;         /* bit j enables feature j in the score */
    float threshold;               /* score above which an actuation is anomalous */
    float update_gate;             /* adapt the model only when score < gate */
    float ewma_alpha;              /* adaptation gain after learning */
    float z_clip;                  /* per-feature |z| clip */
    float floor_abs[TT_NFEAT];     /* absolute std-dev floor per feature */
    float floor_rel[TT_NFEAT];     /* relative std-dev floor (times |mean|) */
    float settle_band_abs;         /* settle band, mA */
    float settle_band_rel;         /* settle band, fraction of level */
    uint16_t settle_run;           /* consecutive in-band samples to call it settled */
    float z_strong;                /* classifier: strong deviation */
    float z_weak;                  /* classifier: weak deviation */
} tt_act_config_t;

typedef struct {
    /* CUSUM in physical units (mA, mA*samples). The upward allowance is set
     * above the largest steady current a healthy servo can be left drawing
     * when gear stiction stops it just outside its deadband (after a relock
     * or a knock); the downward allowance is small because the holding
     * current never legitimately falls below the electronics' own draw. */
    float k_hi_ma;                 /* upward allowance, mA */
    float k_lo_ma;                 /* downward allowance, mA */
    float clip_ma;                 /* per-sample deviation clip, mA */
    float h_hi;                    /* upward alarm threshold, mA*samples */
    float h_lo;                    /* downward alarm threshold, mA*samples */
    uint32_t learn_samples;        /* idle samples used to learn the baseline */
    uint32_t blank_samples;        /* ignored after entering idle (servo settles) */
    uint32_t refractory_samples;   /* ignored after an alarm */
    float track_alpha;             /* baseline tracking gain per sample */
    float track_gate_ma;           /* track only while |x - ref| < gate and CUSUM ~ 0 */
    float drift_max;               /* max baseline drift from the learned value, mA */
    uint32_t anchor_samples;       /* per-period reference averaged after blanking */
    float anchor_max;              /* max distance of that reference from baseline, mA */
    uint32_t anchor_timeout;       /* give up anchoring (use the baseline) after this */
} tt_idle_config_t;

/* --------------------------------------------------------------- models */
typedef struct {
    float mean[TT_NFEAT];
    float var[TT_NFEAT];
    float tmpl[TT_NBINS];          /* learned mean current per bin, mA */
    uint16_t n;                    /* healthy actuations absorbed (saturates) */
    uint16_t n_resid;              /* actuations absorbed into residual stats */
} tt_act_model_t;

/* Streaming accumulator for the actuation in progress. */
typedef struct {
    float bin_sum[TT_NBINS];
    uint16_t bin_cnt[TT_NBINS];
    float peak;
    float sum_motion;
    float sum_pos;                 /* sum of positive current over the motion */
    float sum_tx;                  /* sum of idx * positive current */
    float sum_sq;
    float plateau_sum;
    float settle_ref;
    uint16_t total;                /* expected samples in the window */
    uint16_t motion;               /* samples until commanded motion ends */
    uint16_t idx;                  /* samples received */
    uint16_t peak_idx;
    uint16_t plateau_cnt;
    uint16_t settle_start;
    uint16_t settle_len;
    uint16_t settle_idx;
    float dt_ms;                   /* sample period, ms */
    uint8_t kind;
    uint8_t active;
    uint8_t settled;
    uint8_t pad_;
} tt_act_acc_t;

typedef struct {
    float mean;                    /* long-term idle baseline current, mA */
    float var;                     /* idle noise variance, mA^2 */
    float learned_mean;            /* baseline at the end of learning */
    float ref;                     /* this idle period's CUSUM reference, mA */
    float anchor_sum;              /* running sum while anchoring the reference */
    uint32_t anchor_cnt;
    uint32_t anchor_age;           /* samples spent anchoring this period */
    float s_hi;                    /* CUSUM, upward */
    float s_lo;                    /* CUSUM, downward */
    uint32_t n;                    /* learning samples absorbed */
    uint32_t since_enter;          /* samples since the lock entered idle */
    uint32_t refractory;           /* samples left in the refractory window */
    uint32_t sample_idx;           /* samples since the lock entered idle */
    uint32_t hi_zero_idx;          /* last sample where s_hi was zero */
    uint32_t lo_zero_idx;          /* last sample where s_lo was zero */
    uint32_t last_onset;           /* samples between change point and last alarm */
} tt_idle_t;

typedef struct {
    tt_act_config_t act_cfg;
    tt_idle_config_t idle_cfg;
    tt_act_model_t model[TT_ACT_KINDS];
    tt_act_acc_t acc;
    tt_idle_t idle;
} tt_detector_t;

/* Result of one actuation. int32 fields keep the layout identical for ctypes. */
typedef struct {
    float features[TT_NFEAT];
    float z[TT_NFEAT];
    float score;
    int32_t status;                /* TT_STATUS_* */
    int32_t fault;                 /* TT_FAULT_* */
    int32_t updated;               /* 1 if the model absorbed this actuation */
} tt_act_result_t;

/* ------------------------------------------------------------------ API */
void tt_act_config_default(tt_act_config_t *cfg);
void tt_idle_config_default(tt_idle_config_t *cfg);
void tt_detector_init(tt_detector_t *det);
void tt_detector_reset_models(tt_detector_t *det);

/* Actuations: begin, feed one sample per call, finish when feed returns 1. */
int tt_act_begin(tt_detector_t *det, uint32_t kind, uint32_t total_samples,
                 uint32_t motion_samples, float dt_ms);
int tt_act_feed(tt_detector_t *det, float current_ma);
int tt_act_finish(tt_detector_t *det, tt_act_result_t *out);
int tt_act_learning(const tt_detector_t *det, uint32_t kind);

/* Convenience: run a whole recorded actuation through begin/feed/finish. */
int tt_act_process(tt_detector_t *det, uint32_t kind, const float *samples,
                   uint32_t n, uint32_t motion_samples, float dt_ms,
                   tt_act_result_t *out);

/* Idle monitor: call tt_idle_enter() each time the lock settles into the
 * locked hold, then feed every idle sample. */
void tt_idle_enter(tt_detector_t *det);
int tt_idle_feed(tt_detector_t *det, float current_ma, float *stat_out);
int tt_idle_learned(const tt_detector_t *det);
uint32_t tt_idle_onset_samples(const tt_detector_t *det);
uint32_t tt_idle_feed_block(tt_detector_t *det, const float *samples, uint32_t n,
                            uint8_t *events_out);

/* Pure helpers, exposed for tests. */
float tt_score_from_z(const float *z, uint32_t mask);
int tt_classify(const float *z, float score, float threshold, float z_strong,
                float z_weak);
float tt_template_residual(const float *bins, const float *tmpl);

/* Layout helpers for host bindings (ctypes) and size reports. */
uint32_t tt_detector_size(void);
uint32_t tt_result_size(void);
uint32_t tt_act_config_size(void);
uint32_t tt_idle_config_size(void);
tt_act_config_t *tt_detector_act_config(tt_detector_t *det);
tt_idle_config_t *tt_detector_idle_config(tt_detector_t *det);

#ifdef __cplusplus
}
#endif

#endif /* TELLTALE_DETECT_H */
