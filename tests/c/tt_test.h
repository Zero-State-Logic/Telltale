/*
 * tt_test.h - a tiny single-header test harness for the C core.
 * Each test binary prints one line per test and a summary, and returns a
 * non-zero exit code if any check failed.
 */
#ifndef TT_TEST_H
#define TT_TEST_H

#include <math.h>
#include <stdio.h>

static int tt_checks_ = 0;
static int tt_failures_ = 0;
static int tt_tests_ = 0;
static int tt_tests_failed_ = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        tt_checks_++;                                                        \
        if (!(cond)) {                                                       \
            tt_failures_++;                                                  \
            printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        const double tt_a_ = (double)(a);                                    \
        const double tt_b_ = (double)(b);                                    \
        tt_checks_++;                                                        \
        if (!(fabs(tt_a_ - tt_b_) <= (double)(tol))) {                       \
            tt_failures_++;                                                  \
            printf("    FAIL %s:%d: %s = %.6g, expected %.6g (tol %.3g)\n",  \
                   __FILE__, __LINE__, #a, tt_a_, tt_b_, (double)(tol));     \
        }                                                                    \
    } while (0)

#define RUN_TEST(fn)                                                         \
    do {                                                                     \
        const int tt_before_ = tt_failures_;                                 \
        fn();                                                                \
        tt_tests_++;                                                         \
        if (tt_failures_ != tt_before_) {                                    \
            tt_tests_failed_++;                                              \
            printf("[FAIL] %s\n", #fn);                                      \
        } else {                                                             \
            printf("[ ok ] %s\n", #fn);                                      \
        }                                                                    \
    } while (0)

#define TEST_SUMMARY(name)                                                   \
    (printf("%s: %d tests, %d passed, %d failed (%d checks)\n", name,        \
            tt_tests_, tt_tests_ - tt_tests_failed_, tt_tests_failed_,       \
            tt_checks_),                                                     \
     (tt_failures_ == 0) ? 0 : 1)

#endif /* TT_TEST_H */
