/*
 * telltale_detect.c - proprioceptive tamper and fault detector (portable C99).
 * See telltale_detect.h for the method. Copyright (c) 2026 Faysal Ali Shah. MIT.
 */
#include "telltale_detect.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------ defaults */
void tt_act_config_default(tt_act_config_t *cfg)
{
    /* Std-dev floors keep a model learned from 30 runs from becoming
     * over-confident on features that barely vary (e.g. time-to-peak is set
     * by the commanded ramp and jitters by a frame or two). */
    static const float floor_abs[TT_NFEAT] = {
        3.0f,   /* peak, mA */
        2.0f,   /* mean, mA */
        60.0f,  /* energy, mA^2*s */
        30.0f,  /* time-to-peak, ms */
        20.0f,  /* settle, ms */
        2.0f,   /* plateau, mA */
        1.0f,   /* residual, mA */
        8.0f    /* centroid, ms */
    };
    static const float floor_rel[TT_NFEAT] = {
        0.04f, 0.04f, 0.08f, 0.0f, 0.0f, 0.05f, 0.10f, 0.0f
    };
    if (cfg == 0) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->learn_runs = 30u;
    cfg->resid_warmup = 10u;
    cfg->feature_mask = TT_FEAT_DEFAULT_MASK;
    cfg->threshold = 7.24f;          /* chosen on validation locks, results/eval.json */
    cfg->update_gate = 3.5f;
    cfg->ewma_alpha = 0.02f;
    cfg->z_clip = 50.0f;
    memcpy(cfg->floor_abs, floor_abs, sizeof(floor_abs));
    memcpy(cfg->floor_rel, floor_rel, sizeof(floor_rel));
    cfg->settle_band_abs = 4.0f;
    cfg->settle_band_rel = 0.10f;
    cfg->settle_run = 20u;
    cfg->z_strong = 6.0f;
    cfg->z_weak = 1.0f;
}

void tt_idle_config_default(tt_idle_config_t *cfg)
{
    if (cfg == 0) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->k_hi_ma = 8.0f;
    cfg->k_lo_ma = 2.0f;
    cfg->clip_ma = 40.0f;
    cfg->h_hi = 360.0f;              /* chosen on validation idle streams, results/eval.json */
    cfg->h_lo = 150.0f;
    cfg->learn_samples = 2000u;      /* 20 s at 100 Hz */
    cfg->blank_samples = 50u;        /* 0.5 s after the relock settles */
    cfg->refractory_samples = 300u;  /* 3 s between alarms */
    cfg->track_alpha = 1.0e-4f;      /* ~100 s time constant at 100 Hz */
    cfg->track_gate_ma = 2.0f;
    cfg->drift_max = 3.0f;
    cfg->anchor_samples = 50u;       /* 0.5 s */
    cfg->anchor_max = 8.0f;
    cfg->anchor_timeout = 300u;      /* 3 s */
}

void tt_detector_reset_models(tt_detector_t *det)
{
    if (det == 0) {
        return;
    }
    memset(det->model, 0, sizeof(det->model));
    memset(&det->acc, 0, sizeof(det->acc));
    memset(&det->idle, 0, sizeof(det->idle));
}

void tt_detector_init(tt_detector_t *det)
{
    if (det == 0) {
        return;
    }
    memset(det, 0, sizeof(*det));
    tt_act_config_default(&det->act_cfg);
    tt_idle_config_default(&det->idle_cfg);
}

uint32_t tt_detector_size(void) { return (uint32_t)sizeof(tt_detector_t); }
uint32_t tt_result_size(void) { return (uint32_t)sizeof(tt_act_result_t); }
uint32_t tt_act_config_size(void) { return (uint32_t)sizeof(tt_act_config_t); }
uint32_t tt_idle_config_size(void) { return (uint32_t)sizeof(tt_idle_config_t); }

tt_act_config_t *tt_detector_act_config(tt_detector_t *det)
{
    return (det == 0) ? 0 : &det->act_cfg;
}

tt_idle_config_t *tt_detector_idle_config(tt_detector_t *det)
{
    return (det == 0) ? 0 : &det->idle_cfg;
}

/* ------------------------------------------------------------- helpers */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    return (x > hi) ? hi : x;
}

/* One step of a weighted mean/variance update. With gain = 1/n this is
 * Welford's algorithm (population variance); with a constant gain it is an
 * exponentially weighted mean and variance. */
static void stat_update(float *mean, float *var, float x, float gain)
{
    const float d = x - *mean;
    *mean += gain * d;
    *var = (1.0f - gain) * (*var + gain * d * d);
}

static float floored_std(const tt_act_config_t *cfg, uint32_t j, float mean,
                         float var)
{
    const float f = cfg->floor_abs[j] + cfg->floor_rel[j] * fabsf(mean);
    const float v = (var > f * f) ? var : f * f;
    return sqrtf(v);
}

float tt_template_residual(const float *bins, const float *tmpl)
{
    float acc = 0.0f;
    uint32_t b;
    if (bins == 0 || tmpl == 0) {
        return 0.0f;
    }
    for (b = 0; b < TT_NBINS; ++b) {
        const float d = bins[b] - tmpl[b];
        acc += d * d;
    }
    return sqrtf(acc / (float)TT_NBINS);
}

float tt_score_from_z(const float *z, uint32_t mask)
{
    float acc = 0.0f;
    uint32_t j;
    if (z == 0) {
        return 0.0f;
    }
    for (j = 0; j < TT_NFEAT; ++j) {
        if ((mask >> j) & 1u) {
            acc += z[j] * z[j];
        }
    }
    return sqrtf(acc);
}

int tt_classify(const float *z, float score, float threshold, float z_strong,
                float z_weak)
{
    float peak, mean, plateau;
    if (z == 0 || score <= threshold) {
        return TT_FAULT_NONE;
    }
    peak = z[TT_FEAT_PEAK];
    mean = z[TT_FEAT_MEAN];
    plateau = z[TT_FEAT_PLATEAU];

    /* Still pushing hard after the commanded motion ended: blocked. */
    if (plateau >= z_strong && peak >= z_strong) {
        return TT_FAULT_STALL;
    }
    /* Less current while moving: the servo is not pulling the latch. */
    if (mean <= -z_weak) {
        return TT_FAULT_LINKAGE;
    }
    /* More current for the same work, the static hold included, but no
     * higher peaks: supply current scales with 1/V at constant torque while
     * a weak supply caps the current spikes. */
    if (mean >= z_weak && plateau >= z_weak && peak < z_weak) {
        return TT_FAULT_BROWNOUT;
    }
    /* More current while moving and higher peaks: friction. */
    if (mean >= z_weak || peak >= z_weak) {
        return TT_FAULT_FRICTION;
    }
    return TT_FAULT_UNKNOWN;
}

/* ----------------------------------------------------------- actuation */
int tt_act_begin(tt_detector_t *det, uint32_t kind, uint32_t total_samples,
                 uint32_t motion_samples, float dt_ms)
{
    tt_act_acc_t *acc;
    if (det == 0 || kind >= TT_ACT_KINDS || total_samples > 0xFFFFu ||
        motion_samples < TT_NBINS || motion_samples > total_samples ||
        !(dt_ms > 0.0f)) {
        return TT_ERR_ARG;
    }
    acc = &det->acc;
    memset(acc, 0, sizeof(*acc));
    acc->total = (uint16_t)total_samples;
    acc->motion = (uint16_t)motion_samples;
    acc->dt_ms = dt_ms;
    acc->kind = (uint8_t)kind;
    acc->peak = -1.0e30f;
    acc->active = 1u;
    return TT_OK;
}

static uint32_t plateau_start(const tt_act_acc_t *acc)
{
    const uint32_t dwell = (uint32_t)acc->total - acc->motion;
    if (dwell >= 2u) {
        return (uint32_t)acc->total - dwell / 2u;
    }
    return (uint32_t)acc->total - (uint32_t)acc->total / 10u;
}

static void settle_track(tt_act_acc_t *acc, const tt_act_config_t *cfg, float x)
{
    if (acc->settled || acc->idx < acc->motion) {
        return;
    }
    if (acc->settle_len == 0u) {
        acc->settle_ref = x;
        acc->settle_start = acc->idx;
        acc->settle_len = 1u;
    } else {
        const float band = cfg->settle_band_abs +
                           cfg->settle_band_rel * fabsf(acc->settle_ref);
        if (fabsf(x - acc->settle_ref) <= band) {
            acc->settle_len++;
            acc->settle_ref += (x - acc->settle_ref) / (float)acc->settle_len;
        } else {
            acc->settle_ref = x;
            acc->settle_start = acc->idx;
            acc->settle_len = 1u;
        }
    }
    if (acc->settle_len >= cfg->settle_run) {
        acc->settled = 1u;
        acc->settle_idx = acc->settle_start;
    }
}

int tt_act_feed(tt_detector_t *det, float x)
{
    tt_act_acc_t *acc;
    uint32_t bin;
    if (det == 0) {
        return TT_ERR_ARG;
    }
    acc = &det->acc;
    if (!acc->active) {
        return TT_ERR_STATE;
    }
    if (acc->idx >= acc->total) {
        return 1;
    }
    if (x > acc->peak) {
        acc->peak = x;
        acc->peak_idx = acc->idx;
    }
    if (acc->idx < acc->motion) {
        bin = ((uint32_t)acc->idx * TT_NBINS) / acc->motion;
        acc->bin_sum[bin] += x;
        acc->bin_cnt[bin]++;
        acc->sum_motion += x;
        if (x > 0.0f) {
            acc->sum_pos += x;
            acc->sum_tx += (float)acc->idx * x;
        }
    }
    acc->sum_sq += x * x;
    if ((uint32_t)acc->idx >= plateau_start(acc)) {
        acc->plateau_sum += x;
        acc->plateau_cnt++;
    }
    settle_track(acc, &det->act_cfg, x);
    acc->idx++;
    return (acc->idx >= acc->total) ? 1 : 0;
}

static void compute_features(const tt_act_acc_t *acc, float *f, float *bins)
{
    uint32_t b;
    const uint32_t settle_end = acc->settled ? acc->settle_idx : acc->total;
    for (b = 0; b < TT_NBINS; ++b) {
        bins[b] = (acc->bin_cnt[b] > 0u) ? acc->bin_sum[b] / (float)acc->bin_cnt[b]
                                         : 0.0f;
    }
    f[TT_FEAT_PEAK] = acc->peak;
    f[TT_FEAT_MEAN] = acc->sum_motion / (float)acc->motion;
    f[TT_FEAT_ENERGY] = acc->sum_sq * acc->dt_ms * 1.0e-3f;
    f[TT_FEAT_TPEAK] = (float)acc->peak_idx * acc->dt_ms;
    f[TT_FEAT_SETTLE] = (float)(settle_end - acc->motion) * acc->dt_ms;
    f[TT_FEAT_PLATEAU] = (acc->plateau_cnt > 0u)
                             ? acc->plateau_sum / (float)acc->plateau_cnt
                             : 0.0f;
    f[TT_FEAT_RESID] = 0.0f;
    /* Centroid of the (non-negative part of the) motion current. */
    f[TT_FEAT_CENTROID] = (acc->sum_pos > 1.0e-3f)
                              ? acc->sum_tx / acc->sum_pos * acc->dt_ms
                              : 0.5f * (float)acc->motion * acc->dt_ms;
}

/* Absorb one healthy actuation. Gain is 1/n while learning (exact Welford
 * mean and variance), then the small EWMA gain for slow drift tracking. */
static void model_absorb(tt_act_model_t *m, const tt_act_config_t *cfg,
                         const float *f, const float *bins)
{
    uint32_t j;
    const uint32_t had = m->n;
    float gain;
    if (m->n < 0xFFFFu) {
        m->n++;
    }
    gain = (m->n <= cfg->learn_runs) ? 1.0f / (float)m->n : cfg->ewma_alpha;
    for (j = 0; j < TT_NFEAT; ++j) {
        if (j != TT_FEAT_RESID) {
            stat_update(&m->mean[j], &m->var[j], f[j], gain);
        }
    }
    if (had >= cfg->resid_warmup) {
        const uint32_t resid_runs = (cfg->learn_runs > cfg->resid_warmup)
                                        ? (uint32_t)(cfg->learn_runs - cfg->resid_warmup)
                                        : 1u;
        float g_r;
        if (m->n_resid < 0xFFFFu) {
            m->n_resid++;
        }
        g_r = (m->n_resid <= resid_runs) ? 1.0f / (float)m->n_resid : cfg->ewma_alpha;
        stat_update(&m->mean[TT_FEAT_RESID], &m->var[TT_FEAT_RESID],
                    f[TT_FEAT_RESID], g_r);
    }
    for (j = 0; j < TT_NBINS; ++j) {
        m->tmpl[j] += gain * (bins[j] - m->tmpl[j]);
    }
}

static float score_actuation(const tt_act_model_t *m, const tt_act_config_t *cfg,
                             const float *f, float *z)
{
    uint32_t j;
    for (j = 0; j < TT_NFEAT; ++j) {
        const float s = floored_std(cfg, j, m->mean[j], m->var[j]);
        z[j] = clampf((f[j] - m->mean[j]) / s, -cfg->z_clip, cfg->z_clip);
    }
    /* A residual below the usual one is not suspicious. */
    if (z[TT_FEAT_RESID] < 0.0f) {
        z[TT_FEAT_RESID] = 0.0f;
    }
    return tt_score_from_z(z, cfg->feature_mask);
}

int tt_act_learning(const tt_detector_t *det, uint32_t kind)
{
    if (det == 0 || kind >= TT_ACT_KINDS) {
        return TT_ERR_ARG;
    }
    return (det->model[kind].n < det->act_cfg.learn_runs) ? 1 : 0;
}

int tt_act_finish(tt_detector_t *det, tt_act_result_t *out)
{
    float bins[TT_NBINS];
    tt_act_result_t r;
    tt_act_acc_t *acc;
    tt_act_model_t *m;
    const tt_act_config_t *cfg;
    if (det == 0 || out == 0) {
        return TT_ERR_ARG;
    }
    acc = &det->acc;
    if (!acc->active) {
        return TT_ERR_STATE;
    }
    acc->active = 0u;
    if (acc->idx < acc->total) {
        return TT_ERR_STATE;   /* incomplete window: discarded */
    }
    cfg = &det->act_cfg;
    m = &det->model[acc->kind];
    memset(&r, 0, sizeof(r));
    compute_features(acc, r.features, bins);
    if (m->n > 0u) {
        r.features[TT_FEAT_RESID] = tt_template_residual(bins, m->tmpl);
    }
    if (m->n < cfg->learn_runs) {
        model_absorb(m, cfg, r.features, bins);
        r.status = TT_STATUS_LEARNING;
        r.fault = TT_FAULT_NONE;
        r.updated = 1;
    } else {
        r.score = score_actuation(m, cfg, r.features, r.z);
        r.status = (r.score > cfg->threshold) ? TT_STATUS_ANOMALY : TT_STATUS_HEALTHY;
        r.fault = tt_classify(r.z, r.score, cfg->threshold, cfg->z_strong, cfg->z_weak);
        if (r.score < cfg->update_gate) {
            model_absorb(m, cfg, r.features, bins);
            r.updated = 1;
        }
    }
    *out = r;
    return TT_OK;
}

int tt_act_process(tt_detector_t *det, uint32_t kind, const float *samples,
                   uint32_t n, uint32_t motion_samples, float dt_ms,
                   tt_act_result_t *out)
{
    uint32_t i;
    int rc;
    if (samples == 0 || out == 0) {
        return TT_ERR_ARG;
    }
    rc = tt_act_begin(det, kind, n, motion_samples, dt_ms);
    if (rc != TT_OK) {
        return rc;
    }
    for (i = 0; i < n; ++i) {
        rc = tt_act_feed(det, samples[i]);
        if (rc < 0) {
            return rc;
        }
    }
    return tt_act_finish(det, out);
}

/* ---------------------------------------------------------------- idle */
void tt_idle_enter(tt_detector_t *det)
{
    tt_idle_t *st;
    if (det == 0) {
        return;
    }
    st = &det->idle;
    st->s_hi = 0.0f;
    st->s_lo = 0.0f;
    st->since_enter = 0u;
    st->refractory = 0u;
    st->sample_idx = 0u;
    st->hi_zero_idx = 0u;
    st->lo_zero_idx = 0u;
    st->anchor_sum = 0.0f;
    st->anchor_cnt = 0u;
    st->anchor_age = 0u;
    st->ref = st->mean;
}

int tt_idle_learned(const tt_detector_t *det)
{
    if (det == 0) {
        return 0;
    }
    return (det->idle.n >= det->idle_cfg.learn_samples) ? 1 : 0;
}

uint32_t tt_idle_onset_samples(const tt_detector_t *det)
{
    return (det == 0) ? 0u : det->idle.last_onset;
}

static int idle_learn(tt_idle_t *st, const tt_idle_config_t *cfg, float x)
{
    st->n++;
    stat_update(&st->mean, &st->var, x, 1.0f / (float)st->n);
    if (st->n >= cfg->learn_samples) {
        st->learned_mean = st->mean;
        st->ref = st->mean;
        st->anchor_cnt = cfg->anchor_samples;   /* this period: use the baseline */
    }
    return TT_IDLE_LEARNING;
}

/* Average a quiet stretch at the start of a locked period into its upward
 * reference, kept within anchor_max of the long-term baseline (a larger
 * offset is left for the CUSUM to flag). A transient inside the window
 * restarts it; if no quiet stretch arrives in time, the baseline is used. */
static int idle_anchor(tt_idle_t *st, const tt_idle_config_t *cfg, float x)
{
    st->anchor_age++;
    if (st->anchor_cnt > 0u &&
        fabsf(x - st->anchor_sum / (float)st->anchor_cnt) > cfg->track_gate_ma) {
        st->anchor_sum = 0.0f;
        st->anchor_cnt = 0u;
    }
    st->anchor_sum += x;
    st->anchor_cnt++;
    if (st->anchor_cnt >= cfg->anchor_samples) {
        const float m = st->anchor_sum / (float)st->anchor_cnt;
        st->ref = clampf(m, st->mean - cfg->anchor_max, st->mean + cfg->anchor_max);
    } else if (st->anchor_age >= cfg->anchor_timeout) {
        st->ref = st->mean;
        st->anchor_cnt = cfg->anchor_samples;
    }
    st->hi_zero_idx = st->sample_idx;
    st->lo_zero_idx = st->sample_idx;
    return TT_IDLE_ANCHORING;
}

/* Slow drift tracking, only while the CUSUM is at rest. Each reference
 * follows only samples close to itself, so a period's holding offset never
 * drags the long-term (electronics-only) baseline. */
static void idle_track(tt_idle_t *st, const tt_idle_config_t *cfg, float x)
{
    if ((st->s_hi + st->s_lo) > 0.0f) {
        return;
    }
    if (fabsf(x - st->mean) < cfg->track_gate_ma) {
        st->mean += cfg->track_alpha * (x - st->mean);
        st->mean = clampf(st->mean, st->learned_mean - cfg->drift_max,
                          st->learned_mean + cfg->drift_max);
    }
    if (fabsf(x - st->ref) < cfg->track_gate_ma) {
        st->ref += cfg->track_alpha * (x - st->ref);
        st->ref = clampf(st->ref, st->mean - cfg->anchor_max, st->mean + cfg->anchor_max);
    }
}

static int idle_alarm(tt_idle_t *st, const tt_idle_config_t *cfg, uint32_t zero_idx,
                      int event)
{
    st->last_onset = st->sample_idx - zero_idx;
    st->s_hi = 0.0f;
    st->s_lo = 0.0f;
    st->refractory = cfg->refractory_samples;
    st->anchor_sum = 0.0f;       /* re-anchor once the refractory window ends */
    st->anchor_cnt = 0u;
    st->anchor_age = 0u;
    return event;
}

int tt_idle_feed(tt_detector_t *det, float x, float *stat_out)
{
    tt_idle_t *st;
    const tt_idle_config_t *cfg;
    float dev, dev_lo;
    if (det == 0) {
        return TT_ERR_ARG;
    }
    st = &det->idle;
    cfg = &det->idle_cfg;
    if (stat_out != 0) {
        *stat_out = 0.0f;
    }
    if (st->since_enter < 0xFFFFFFFFu) {
        st->since_enter++;
    }
    st->sample_idx++;
    if (st->since_enter <= cfg->blank_samples) {
        st->hi_zero_idx = st->sample_idx;
        st->lo_zero_idx = st->sample_idx;
        return TT_IDLE_BLANKED;
    }
    if (st->refractory > 0u) {
        st->refractory--;
        st->hi_zero_idx = st->sample_idx;
        st->lo_zero_idx = st->sample_idx;
        return TT_IDLE_BLANKED;
    }
    if (st->n < cfg->learn_samples) {
        return idle_learn(st, cfg, x);
    }
    if (st->anchor_cnt < cfg->anchor_samples) {
        return idle_anchor(st, cfg, x);
    }
    dev = clampf(x - st->ref, -cfg->clip_ma, cfg->clip_ma);
    /* Downward evidence is measured against the long-term baseline (the
     * electronics' own draw is the floor) and negative, regenerative current
     * while the servo is back-driven is not "less power". */
    dev_lo = clampf(((x > 0.0f) ? x : 0.0f) - st->mean, -cfg->clip_ma, cfg->clip_ma);
    st->s_hi = (st->s_hi + dev - cfg->k_hi_ma > 0.0f) ? st->s_hi + dev - cfg->k_hi_ma : 0.0f;
    st->s_lo = (st->s_lo - dev_lo - cfg->k_lo_ma > 0.0f) ? st->s_lo - dev_lo - cfg->k_lo_ma : 0.0f;
    if (st->s_hi <= 0.0f) {
        st->hi_zero_idx = st->sample_idx;
    }
    if (st->s_lo <= 0.0f) {
        st->lo_zero_idx = st->sample_idx;
    }
    if (stat_out != 0) {
        const float a = st->s_hi / cfg->h_hi;
        const float b = st->s_lo / cfg->h_lo;
        *stat_out = (a > b) ? a : b;
    }
    if (st->s_hi > cfg->h_hi) {
        return idle_alarm(st, cfg, st->hi_zero_idx, TT_IDLE_FORCED);
    }
    if (st->s_lo > cfg->h_lo) {
        return idle_alarm(st, cfg, st->lo_zero_idx, TT_IDLE_POWER_LOSS);
    }
    idle_track(st, cfg, x);
    return TT_IDLE_NONE;
}

uint32_t tt_idle_feed_block(tt_detector_t *det, const float *samples, uint32_t n,
                            uint8_t *events_out)
{
    uint32_t i;
    uint32_t alarms = 0u;
    if (det == 0 || samples == 0) {
        return 0u;
    }
    for (i = 0; i < n; ++i) {
        const int ev = tt_idle_feed(det, samples[i], 0);
        if (events_out != 0) {
            events_out[i] = (uint8_t)((ev < 0) ? 0 : ev);
        }
        if (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS) {
            alarms++;
        }
    }
    return alarms;
}
