/*
 * bench.c - RAM footprint (sizeof of every state struct) and host compute
 * time per sample for the detector. Prints one JSON object to stdout; the
 * evaluation saves it as results/footprint.json.
 *
 * The timings are for the build machine, not the ESP32. They are reported
 * only to show the per-sample cost is a few dozen float operations.
 */
#include <stdio.h>
#include <time.h>

#include "../../firmware/telltale/telltale_auth.h"
#include "../../firmware/telltale/telltale_detect.h"
#include "../../firmware/telltale/telltale_motion.h"
#include "../../firmware/telltale/telltale_persist.h"
#include "../../firmware/telltale/telltale_sha256.h"

#define ACT_N 1080u
#define ACT_MOTION 880u
#define ACT_REPS 3000u
#define IDLE_N 2000000u

static double seconds_now(void)
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}

static float wave(uint32_t i)
{
    return 60.0f + (float)((i * 2654435761u) >> 26) * 0.25f;
}

static double bench_actuation(void)
{
    static tt_detector_t det;
    tt_act_result_t r;
    uint32_t rep;
    uint32_t i;
    double t0;
    double t1;
    volatile float sink = 0.0f;
    tt_detector_init(&det);
    t0 = seconds_now();
    for (rep = 0; rep < ACT_REPS; ++rep) {
        tt_act_begin(&det, rep & 1u, ACT_N, ACT_MOTION, 1.0f);
        for (i = 0; i < ACT_N; ++i) {
            tt_act_feed(&det, wave(i + rep));
        }
        tt_act_finish(&det, &r);
        sink += r.score;
    }
    t1 = seconds_now();
    (void)sink;
    return (t1 - t0) * 1.0e9 / ((double)ACT_REPS * ACT_N);
}

static double bench_idle(void)
{
    static tt_detector_t det;
    uint32_t i;
    double t0;
    double t1;
    volatile int sink = 0;
    tt_detector_init(&det);
    tt_idle_enter(&det);
    t0 = seconds_now();
    for (i = 0; i < IDLE_N; ++i) {
        sink += tt_idle_feed(&det, 5.0f + (float)((i * 2654435761u) >> 29) * 0.1f, 0);
    }
    t1 = seconds_now();
    (void)sink;
    return (t1 - t0) * 1.0e9 / (double)IDLE_N;
}

int main(void)
{
    const double act_ns = bench_actuation();
    const double idle_ns = bench_idle();
    printf("{\n");
    printf("  \"sizeof_bytes\": {\n");
    printf("    \"tt_detector_t\": %u,\n", (unsigned)sizeof(tt_detector_t));
    printf("    \"tt_act_model_t\": %u,\n", (unsigned)sizeof(tt_act_model_t));
    printf("    \"tt_act_acc_t\": %u,\n", (unsigned)sizeof(tt_act_acc_t));
    printf("    \"tt_idle_t\": %u,\n", (unsigned)sizeof(tt_idle_t));
    printf("    \"tt_act_config_t\": %u,\n", (unsigned)sizeof(tt_act_config_t));
    printf("    \"tt_idle_config_t\": %u,\n", (unsigned)sizeof(tt_idle_config_t));
    printf("    \"tt_act_result_t\": %u,\n", (unsigned)sizeof(tt_act_result_t));
    printf("    \"tt_persist_t\": %u,\n", (unsigned)sizeof(tt_persist_t));
    printf("    \"tt_auth_t\": %u,\n", (unsigned)sizeof(tt_auth_t));
    printf("    \"tt_hmac_t\": %u,\n", (unsigned)sizeof(tt_hmac_t));
    printf("    \"tt_motion_t\": %u\n", (unsigned)sizeof(tt_motion_t));
    printf("  },\n");
    printf("  \"host_ns_per_sample\": {\n");
    printf("    \"actuation_feed_plus_finish\": %.2f,\n", act_ns);
    printf("    \"idle_cusum\": %.2f\n", idle_ns);
    printf("  },\n");
    printf("  \"note\": \"Timings measured on the build host (x86-64, MSVC /O2), not on an ESP32.\"\n");
    printf("}\n");
    return 0;
}
