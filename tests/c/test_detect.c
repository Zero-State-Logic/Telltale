/*
 * test_detect.c - detector maths: features, learning (Welford), gated
 * adaptation, scoring, the sign-pattern classifier, the idle CUSUM, model
 * persistence, motion profiles and INA219 register maths.
 */
#include <string.h>

#include "../../firmware/telltale/telltale_detect.h"
#include "../../firmware/telltale/telltale_ina219.h"
#include "../../firmware/telltale/telltale_motion.h"
#include "../../firmware/telltale/telltale_persist.h"
#include "tt_test.h"

#define N_SAMPLES 400u
#define N_MOTION 300u

static uint32_t rng_state = 12345u;

static float uniform_pm1(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)(rng_state >> 8) / 8388608.0f - 1.0f;
}

static float gaussish(void)   /* zero mean, unit variance (sum of 3 uniforms) */
{
    return (uniform_pm1() + uniform_pm1() + uniform_pm1());
}

/* A hump (base + amp at the top) while moving, then a flat plateau. */
static void make_shape(float *x, float base, float amp, float plateau, float noise_amp)
{
    uint32_t i;
    for (i = 0; i < N_SAMPLES; ++i) {
        float v;
        if (i < N_MOTION) {
            const float u = (float)i / (float)N_MOTION;
            v = base + amp * (4.0f * u * (1.0f - u));
        } else {
            v = plateau;
        }
        x[i] = v + noise_amp * gaussish();
    }
}

static void make_trace(float *x, float scale, float plateau, float noise_amp)
{
    make_shape(x, 40.0f * scale, 80.0f * scale, plateau, noise_amp);
}

static void learn_healthy(tt_detector_t *det, uint32_t kind)
{
    float x[N_SAMPLES];
    tt_act_result_t r;
    uint32_t run;
    for (run = 0; run < det->act_cfg.learn_runs; ++run) {
        make_trace(x, 1.0f + 0.02f * uniform_pm1(), 60.0f + uniform_pm1(), 1.5f);
        CHECK(tt_act_process(det, kind, x, N_SAMPLES, N_MOTION, 1.0f, &r) == TT_OK);
        CHECK(r.status == TT_STATUS_LEARNING);
    }
}

static void test_defaults(void)
{
    tt_detector_t det;
    tt_detector_init(&det);
    CHECK(det.act_cfg.learn_runs == 30u);
    CHECK(det.act_cfg.feature_mask == TT_FEAT_DEFAULT_MASK);
    CHECK(det.act_cfg.threshold > det.act_cfg.update_gate);
    CHECK(det.idle_cfg.h_hi > 0.0f && det.idle_cfg.k_hi_ma > det.idle_cfg.k_lo_ma);
    CHECK(tt_detector_size() == sizeof(tt_detector_t));
    CHECK(tt_result_size() == sizeof(tt_act_result_t));
}

static void test_features_exact(void)
{
    float x[100];
    tt_detector_t det;
    tt_act_result_t r;
    uint32_t i;
    double energy = 0.0;
    tt_detector_init(&det);
    for (i = 0; i < 100u; ++i) {
        x[i] = (i < 60u) ? (float)i : 10.0f;
        energy += (double)x[i] * x[i];
    }
    CHECK(tt_act_process(&det, TT_ACT_UNLOCK, x, 100, 60, 1.0f, &r) == TT_OK);
    CHECK_NEAR(r.features[TT_FEAT_PEAK], 59.0, 1e-4);
    CHECK_NEAR(r.features[TT_FEAT_TPEAK], 59.0, 1e-4);
    CHECK_NEAR(r.features[TT_FEAT_MEAN], 29.5, 1e-4);
    CHECK_NEAR(r.features[TT_FEAT_PLATEAU], 10.0, 1e-4);
    CHECK_NEAR(r.features[TT_FEAT_SETTLE], 0.0, 1e-4);
    CHECK_NEAR(r.features[TT_FEAT_ENERGY], energy * 1e-3, 1e-2);
    CHECK_NEAR(r.features[TT_FEAT_RESID], 0.0, 1e-6);   /* no template yet */
}

static void test_settle_time(void)
{
    float x[200];
    tt_detector_t det;
    tt_act_result_t r;
    uint32_t i;
    tt_detector_init(&det);
    for (i = 0; i < 200u; ++i) {
        if (i < 100u) {
            x[i] = 50.0f;
        } else if (i < 130u) {
            x[i] = (i % 2u) ? 100.0f : 20.0f;   /* ringing after motion ends */
        } else {
            x[i] = 30.0f;
        }
    }
    CHECK(tt_act_process(&det, TT_ACT_RELOCK, x, 200, 100, 2.0f, &r) == TT_OK);
    CHECK_NEAR(r.features[TT_FEAT_SETTLE], 60.0, 1e-4);   /* 30 samples * 2 ms */
    CHECK_NEAR(r.features[TT_FEAT_TPEAK], 202.0, 1e-4);   /* first 100 at idx 101 */
}

static void test_argument_and_state_errors(void)
{
    tt_detector_t det;
    tt_act_result_t r;
    tt_detector_init(&det);
    CHECK(tt_act_begin(&det, 2, 100, 50, 1.0f) == TT_ERR_ARG);
    CHECK(tt_act_begin(&det, 0, 10, 5, 1.0f) == TT_ERR_ARG);       /* < NBINS */
    CHECK(tt_act_begin(&det, 0, 100, 0, 1.0f) == TT_ERR_ARG);
    CHECK(tt_act_begin(&det, 0, 100, 101, 1.0f) == TT_ERR_ARG);
    CHECK(tt_act_begin(&det, 0, 100, 50, 0.0f) == TT_ERR_ARG);
    CHECK(tt_act_feed(&det, 1.0f) == TT_ERR_STATE);
    CHECK(tt_act_begin(&det, 0, 100, 50, 1.0f) == TT_OK);
    CHECK(tt_act_feed(&det, 1.0f) == 0);
    CHECK(tt_act_finish(&det, &r) == TT_ERR_STATE);   /* incomplete window */
    CHECK(tt_act_finish(&det, &r) == TT_ERR_STATE);   /* and now inactive */
}

static void test_learning_then_healthy(void)
{
    tt_detector_t det;
    float x[N_SAMPLES];
    tt_act_result_t r;
    tt_detector_init(&det);
    CHECK(tt_act_learning(&det, TT_ACT_UNLOCK) == 1);
    learn_healthy(&det, TT_ACT_UNLOCK);
    CHECK(tt_act_learning(&det, TT_ACT_UNLOCK) == 0);
    CHECK(tt_act_learning(&det, TT_ACT_RELOCK) == 1);   /* kinds are separate */
    CHECK(det.model[TT_ACT_UNLOCK].n == 30u);
    CHECK(det.model[TT_ACT_UNLOCK].n_resid == 20u);
    CHECK_NEAR(det.model[TT_ACT_UNLOCK].mean[TT_FEAT_PLATEAU], 60.0, 1.0);
    make_trace(x, 1.0f, 60.0f, 1.5f);
    CHECK(tt_act_process(&det, TT_ACT_UNLOCK, x, N_SAMPLES, N_MOTION, 1.0f, &r) == TT_OK);
    CHECK(r.status == TT_STATUS_HEALTHY);
    CHECK(r.fault == TT_FAULT_NONE);
    CHECK(r.score < det.act_cfg.update_gate);
    CHECK(r.updated == 1);
}

static tt_act_result_t score_shape(const tt_detector_t *learned, float base,
                                   float amp, float plateau)
{
    tt_detector_t det = *learned;
    float x[N_SAMPLES];
    tt_act_result_t r;
    memset(&r, 0, sizeof(r));
    make_shape(x, base, amp, plateau, 1.5f);
    CHECK(tt_act_process(&det, TT_ACT_UNLOCK, x, N_SAMPLES, N_MOTION, 1.0f, &r) == TT_OK);
    return r;
}

static void test_fault_patterns(void)
{
    tt_detector_t det;
    tt_act_result_t r;
    tt_detector_init(&det);
    learn_healthy(&det, TT_ACT_UNLOCK);

    r = score_shape(&det, 200.0f, 400.0f, 600.0f);   /* blocked: pushing at stall */
    CHECK(r.status == TT_STATUS_ANOMALY);
    CHECK(r.fault == TT_FAULT_STALL);

    r = score_shape(&det, 12.0f, 24.0f, 10.0f);      /* no load: linkage gone */
    CHECK(r.status == TT_STATUS_ANOMALY);
    CHECK(r.fault == TT_FAULT_LINKAGE);

    r = score_shape(&det, 70.0f, 50.0f, 80.0f);      /* more mean and hold, same peak */
    CHECK(r.status == TT_STATUS_ANOMALY);
    CHECK(r.fault == TT_FAULT_BROWNOUT);

    r = score_shape(&det, 64.0f, 128.0f, 55.0f);     /* more current and higher peaks */
    CHECK(r.status == TT_STATUS_ANOMALY);
    CHECK(r.fault == TT_FAULT_FRICTION);
}

static void test_anomaly_does_not_update_model(void)
{
    tt_detector_t det;
    tt_act_model_t before;
    float x[N_SAMPLES];
    tt_act_result_t r;
    tt_detector_init(&det);
    learn_healthy(&det, TT_ACT_UNLOCK);
    before = det.model[TT_ACT_UNLOCK];
    make_trace(x, 0.3f, 10.0f, 1.5f);
    CHECK(tt_act_process(&det, TT_ACT_UNLOCK, x, N_SAMPLES, N_MOTION, 1.0f, &r) == TT_OK);
    CHECK(r.updated == 0);
    CHECK(memcmp(&before, &det.model[TT_ACT_UNLOCK], sizeof(before)) == 0);
}

static void test_slow_drift_is_tracked(void)
{
    tt_detector_t det;
    float x[N_SAMPLES];
    tt_act_result_t r;
    int run;
    int anomalies = 0;
    tt_detector_init(&det);
    learn_healthy(&det, TT_ACT_UNLOCK);
    /* +30 % over 600 actuations (0.05 % each): adaptation keeps up. */
    for (run = 0; run < 600; ++run) {
        const float s = 1.0f + 0.30f * (float)run / 600.0f;
        make_trace(x, s, 60.0f, 1.5f);
        CHECK(tt_act_process(&det, TT_ACT_UNLOCK, x, N_SAMPLES, N_MOTION, 1.0f, &r) == TT_OK);
        anomalies += (r.status == TT_STATUS_ANOMALY);
    }
    CHECK(anomalies == 0);
    CHECK(det.model[TT_ACT_UNLOCK].mean[TT_FEAT_MEAN] > 1.2f * 93.0f);
}

static void test_score_and_residual_helpers(void)
{
    float z[TT_NFEAT] = {3.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 12.0f};
    float a[TT_NBINS];
    float b[TT_NBINS];
    uint32_t i;
    CHECK_NEAR(tt_score_from_z(z, 0x03u), 5.0, 1e-5);
    CHECK_NEAR(tt_score_from_z(z, TT_FEAT_ALL), 13.0, 1e-5);
    for (i = 0; i < TT_NBINS; ++i) {
        a[i] = (float)i;
        b[i] = (float)i + 2.0f;
    }
    CHECK_NEAR(tt_template_residual(a, a), 0.0, 1e-6);
    CHECK_NEAR(tt_template_residual(a, b), 2.0, 1e-5);
}

static void test_classifier_rules(void)
{
    float z[TT_NFEAT] = {0};
    CHECK(tt_classify(z, 1.0f, 7.0f, 6.0f, 2.5f) == TT_FAULT_NONE);
    z[TT_FEAT_PLATEAU] = 20.0f;
    z[TT_FEAT_PEAK] = 20.0f;
    CHECK(tt_classify(z, 30.0f, 7.0f, 6.0f, 2.5f) == TT_FAULT_STALL);
    memset(z, 0, sizeof(z));
    z[TT_FEAT_MEAN] = -5.0f;
    z[TT_FEAT_PEAK] = -4.0f;
    CHECK(tt_classify(z, 8.0f, 7.0f, 6.0f, 2.5f) == TT_FAULT_LINKAGE);
    memset(z, 0, sizeof(z));
    z[TT_FEAT_MEAN] = 5.0f;
    z[TT_FEAT_PLATEAU] = 4.0f;
    CHECK(tt_classify(z, 8.0f, 7.0f, 6.0f, 2.5f) == TT_FAULT_BROWNOUT);
    z[TT_FEAT_PEAK] = 4.0f;
    CHECK(tt_classify(z, 8.0f, 7.0f, 6.0f, 2.5f) == TT_FAULT_FRICTION);
    z[TT_FEAT_PEAK] = 0.0f;
    z[TT_FEAT_PLATEAU] = -1.0f;
    CHECK(tt_classify(z, 8.0f, 7.0f, 6.0f, 2.5f) == TT_FAULT_FRICTION);
    memset(z, 0, sizeof(z));
    z[TT_FEAT_TPEAK] = 9.0f;
    CHECK(tt_classify(z, 9.0f, 7.0f, 6.0f, 2.5f) == TT_FAULT_UNKNOWN);
}

/* ------------------------------------------------------------ idle CUSUM */
static tt_detector_t idle_learned(void)
{
    tt_detector_t det;
    uint32_t i;
    tt_detector_init(&det);
    tt_idle_enter(&det);
    for (i = 0; i < det.idle_cfg.blank_samples; ++i) {
        CHECK(tt_idle_feed(&det, 500.0f, 0) == TT_IDLE_BLANKED);   /* ignored */
    }
    for (i = 0; i < det.idle_cfg.learn_samples; ++i) {
        CHECK(tt_idle_feed(&det, 5.0f + 0.4f * gaussish(), 0) == TT_IDLE_LEARNING);
    }
    CHECK(tt_idle_learned(&det) == 1);
    CHECK_NEAR(det.idle.mean, 5.0, 0.05);
    CHECK_NEAR(sqrtf(det.idle.var), 0.4, 0.05);
    return det;
}

static void test_idle_quiet_no_alarm(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    int alarms = 0;
    for (i = 0; i < 200000u; ++i) {   /* ~33 min at 100 Hz */
        const int ev = tt_idle_feed(&det, 5.0f + 0.4f * gaussish(), 0);
        alarms += (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS);
    }
    CHECK(alarms == 0);
}

static void test_idle_short_spike_ignored(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    int alarms = 0;
    for (i = 0; i < 1000u; ++i) {
        const float x = (i >= 500u && i < 510u) ? 400.0f : 5.0f + 0.4f * gaussish();
        const int ev = tt_idle_feed(&det, x, 0);
        alarms += (ev == TT_IDLE_FORCED);
    }
    CHECK(alarms == 0);   /* 100 ms knock: 10 * (40 - 8) = 320 < h_hi */
}

static void test_idle_sustained_force_detected(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    uint32_t alarm_at = 0;
    for (i = 0; i < 1000u && alarm_at == 0u; ++i) {
        const float x = (i >= 300u) ? 25.0f + 0.4f * gaussish() : 5.0f + 0.4f * gaussish();
        if (tt_idle_feed(&det, x, 0) == TT_IDLE_FORCED) {
            alarm_at = i;
        }
    }
    /* +20 mA against an 8 mA allowance: 12 per sample, h_hi/12 = 30 samples */
    CHECK(alarm_at >= 300u + 27u && alarm_at <= 300u + 31u);
    CHECK(tt_idle_onset_samples(&det) >= 28u && tt_idle_onset_samples(&det) <= 32u);
}

static void test_idle_power_loss_detected(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    int seen = 0;
    for (i = 0; i < 200u; ++i) {
        const float x = (i >= 50u) ? 0.0f : 5.0f + 0.4f * gaussish();
        seen |= (tt_idle_feed(&det, x, 0) == TT_IDLE_POWER_LOSS);
    }
    CHECK(seen);
}

static void test_idle_drift_bounded(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    int alarms = 0;
    /* A slow 1 mA offset drift over ~17 minutes is tracked without alarms... */
    for (i = 0; i < 100000u; ++i) {
        const float drift = (float)i / 100000.0f;
        const int ev = tt_idle_feed(&det, 5.0f + drift + 0.4f * gaussish(), 0);
        alarms += (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS);
    }
    CHECK(alarms == 0);
    CHECK_NEAR(det.idle.mean, 6.0, 0.35);   /* gated tracking lags a ramp slightly */
    /* ...but tracking never moves the baseline more than drift_max. */
    det.idle.mean = det.idle.learned_mean + det.idle_cfg.drift_max;
    tt_idle_feed(&det, det.idle.mean + 0.1f, 0);
    CHECK(det.idle.mean <= det.idle.learned_mean + det.idle_cfg.drift_max + 1e-4f);
}

static int run_period(tt_detector_t *det, float level, uint32_t n)
{
    uint32_t i;
    int alarms = 0;
    tt_idle_enter(det);
    for (i = 0; i < n; ++i) {
        const int ev = tt_idle_feed(det, level + 0.4f * gaussish(), 0);
        alarms += (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS);
    }
    return alarms;
}

static void test_idle_small_offset_ignored(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    int alarms = 0;
    /* A knock leaves the servo just outside its deadband, drawing +6 mA. */
    for (i = 0; i < 30000u; ++i) {
        const float x = (i < 10u) ? 300.0f : 11.0f + 0.4f * gaussish();
        const int ev = tt_idle_feed(&det, x, 0);
        alarms += (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS);
    }
    CHECK(alarms == 0);
}

static void test_idle_period_anchoring(void)
{
    tt_detector_t det = idle_learned();
    /* The servo stopped with a small steady hold (+4 mA): anchored, no alarm. */
    CHECK(run_period(&det, 9.0f, 3000u) == 0);
    CHECK_NEAR(det.idle.ref, 9.0, 0.2);
    /* A new period re-anchors near the plain baseline again. */
    CHECK(run_period(&det, 5.0f, 3000u) == 0);
    CHECK_NEAR(det.idle.ref, 5.0, 0.2);
    /* A heavy load from the start is beyond anchor_max and still alarms. */
    CHECK(run_period(&det, 40.0f, 300u) >= 1);
}

static void test_idle_knock_during_anchoring(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    int alarms = 0;
    tt_idle_enter(&det);
    for (i = 0; i < 20000u; ++i) {
        /* a knock right after blanking, inside the anchoring window */
        const float x = (i >= 60u && i < 75u) ? 250.0f : 5.0f + 0.4f * gaussish();
        const int ev = tt_idle_feed(&det, x, 0);
        alarms += (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS);
    }
    CHECK(alarms == 0);
    CHECK_NEAR(det.idle.ref, 5.0, 0.3);
}

static void test_idle_hold_released_is_not_power_loss(void)
{
    tt_detector_t det = idle_learned();
    uint32_t i;
    int alarms = 0;
    tt_idle_enter(&det);
    for (i = 0; i < 20000u; ++i) {
        /* anchored on a +7 mA hold, then a knock lets the servo relax */
        const float x = (i < 5000u) ? 12.0f + 0.4f * gaussish() : 5.0f + 0.4f * gaussish();
        const int ev = tt_idle_feed(&det, x, 0);
        alarms += (ev == TT_IDLE_FORCED || ev == TT_IDLE_POWER_LOSS);
    }
    CHECK(alarms == 0);
}

static void test_idle_block_matches_single(void)
{
    tt_detector_t a = idle_learned();
    tt_detector_t b = a;
    float x[600];
    uint8_t ev[600];
    uint32_t i;
    uint32_t alarms_single = 0;
    for (i = 0; i < 600u; ++i) {
        x[i] = (i > 200u && i < 400u) ? 30.0f : 5.0f + 0.4f * gaussish();
    }
    for (i = 0; i < 600u; ++i) {
        const int e = tt_idle_feed(&a, x[i], 0);
        alarms_single += (e == TT_IDLE_FORCED || e == TT_IDLE_POWER_LOSS);
    }
    CHECK(tt_idle_feed_block(&b, x, 600, ev) == alarms_single);
    CHECK(memcmp(&a.idle, &b.idle, sizeof(a.idle)) == 0);
}

/* ------------------------------------------------------- persistence etc. */
static void test_persist_roundtrip(void)
{
    tt_detector_t det;
    tt_detector_t other;
    tt_persist_t blob;
    tt_detector_init(&det);
    learn_healthy(&det, TT_ACT_UNLOCK);
    det.idle.mean = 5.5f;
    det.idle.n = 2000u;
    CHECK(tt_persist_pack(&det, &blob) == TT_OK);
    tt_detector_init(&other);
    CHECK(tt_persist_unpack(&other, &blob) == TT_OK);
    CHECK(memcmp(other.model, det.model, sizeof(det.model)) == 0);
    CHECK_NEAR(other.idle.mean, 5.5, 1e-6);
    ((uint8_t *)&blob)[20] ^= 0x01u;
    CHECK(tt_persist_unpack(&other, &blob) == TT_ERR_DATA);
    CHECK(tt_crc32("123456789", 9) == 0xCBF43926u);
}

static void test_motion_profiles(void)
{
    tt_motion_t m;
    float buf[60];
    tt_motion_unlock(&m);
    CHECK(tt_motion_motion_ms(&m) == 880u);
    CHECK(tt_motion_total_ms(&m) == 1080u);
    CHECK_NEAR(tt_motion_angle_at(&m, 0), 10.0, 1e-5);
    CHECK_NEAR(tt_motion_angle_at(&m, 360), 55.0, 1e-4);
    CHECK_NEAR(tt_motion_angle_at(&m, 720), 100.0, 1e-4);
    CHECK_NEAR(tt_motion_angle_at(&m, 799), 100.0, 1e-4);
    CHECK_NEAR(tt_motion_angle_at(&m, 880), 90.0, 1e-4);
    CHECK_NEAR(tt_motion_angle_at(&m, 5000), 90.0, 1e-4);
    CHECK_NEAR(tt_motion_frame_angle(&m, 39), tt_motion_angle_at(&m, 20), 1e-5);
    CHECK(tt_motion_sample(&m, 1000u, buf, 60) == 60u);
    CHECK_NEAR(buf[45], tt_motion_angle_at(&m, 40), 1e-5);
    tt_motion_relock(&m);
    CHECK(tt_motion_motion_ms(&m) == 768u);
    CHECK_NEAR(tt_motion_angle_at(&m, 664), 7.0, 1e-4);
    CHECK_NEAR(tt_motion_angle_at(&m, 768), 10.0, 1e-4);
    CHECK(tt_servo_duty(0.0f) == 1638u);
    CHECK(tt_servo_duty(180.0f) == 8192u);
    CHECK(tt_servo_duty(90.0f) == 4915u);
    CHECK(tt_servo_duty(-5.0f) == 1638u);
}

static void test_ina219_maths(void)
{
    CHECK(tt_ina219_config_word(1, TT_INA219_PGA_320MV, TT_INA219_ADC_12BIT,
                                TT_INA219_ADC_12BIT, 7) == 0x399Fu);   /* POR default */
    CHECK(tt_ina219_config_motion() == 0x0807u);
    CHECK(tt_ina219_config_idle() == 0x09E7u);
    CHECK_NEAR(tt_ina219_shunt_ma(1000, 0.1f), 100.0, 1e-4);
    CHECK_NEAR(tt_ina219_shunt_ma(-250, 0.1f), -25.0, 1e-4);
    CHECK_NEAR(tt_ina219_shunt_ma(100, 0.0f), 0.0, 1e-6);
    CHECK_NEAR(tt_ina219_bus_mv(0x2EE0u), 6000.0, 1e-3);
}

int main(void)
{
    RUN_TEST(test_defaults);
    RUN_TEST(test_features_exact);
    RUN_TEST(test_settle_time);
    RUN_TEST(test_argument_and_state_errors);
    RUN_TEST(test_learning_then_healthy);
    RUN_TEST(test_fault_patterns);
    RUN_TEST(test_anomaly_does_not_update_model);
    RUN_TEST(test_slow_drift_is_tracked);
    RUN_TEST(test_score_and_residual_helpers);
    RUN_TEST(test_classifier_rules);
    RUN_TEST(test_idle_quiet_no_alarm);
    RUN_TEST(test_idle_short_spike_ignored);
    RUN_TEST(test_idle_sustained_force_detected);
    RUN_TEST(test_idle_power_loss_detected);
    RUN_TEST(test_idle_drift_bounded);
    RUN_TEST(test_idle_small_offset_ignored);
    RUN_TEST(test_idle_period_anchoring);
    RUN_TEST(test_idle_knock_during_anchoring);
    RUN_TEST(test_idle_hold_released_is_not_power_loss);
    RUN_TEST(test_idle_block_matches_single);
    RUN_TEST(test_persist_roundtrip);
    RUN_TEST(test_motion_profiles);
    RUN_TEST(test_ina219_maths);
    return TEST_SUMMARY("test_detect");
}
