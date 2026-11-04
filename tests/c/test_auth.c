/*
 * test_auth.c - nonce issue/verify, single use, expiry, MAC checks, rate
 * limiting and the cross-language known-answer vector shared with the JS and
 * Python tests.
 */
#include <string.h>

#include "../../firmware/telltale/telltale_auth.h"
#include "tt_test.h"

static const char KEY[] = "0123456789abcdef0123456789abcdef";
#define KEY_LEN (sizeof(KEY) - 1u)

static const uint8_t RAND0[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

static void fresh(tt_auth_t *a)
{
    CHECK(tt_auth_init(a, (const uint8_t *)KEY, KEY_LEN) == TT_AUTH_OK);
}

static void random_bytes(uint8_t out[16], uint8_t seed)
{
    int i;
    for (i = 0; i < 16; ++i) {
        out[i] = (uint8_t)(seed * 31u + (uint8_t)i * 7u + 1u);
    }
}

static tt_auth_status_t sign_and_verify(tt_auth_t *a, uint32_t now, const char *nonce,
                                        const char *cmd, const char *arg)
{
    char mac[TT_MAC_HEX_LEN + 1];
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, cmd, arg, mac);
    return tt_auth_verify(a, now, nonce, cmd, arg, mac);
}

static void test_known_answer_vectors(void)
{
    char mac[TT_MAC_HEX_LEN + 1];
    const char *nonce = "000102030405060708090a0b0c0d0e0f";
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "unlock", "Remote (Web)", mac);
    CHECK(strcmp(mac, "4969da3a572eed67dd956d3adbfe3976aead66af75d301f8ea0b0c6305d8e35b") == 0);
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "enroll", "Faysal", mac);
    CHECK(strcmp(mac, "9bd826e6fabb14bc9be18645aefda31bdbf0338efb307ca29e1c14c92ba3e51f") == 0);
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "delete", "7", mac);
    CHECK(strcmp(mac, "b00c8220eadef29575465e84c8ca4d8a3a70c2280f7b675d06045cb7a425f11e") == 0);
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "relearn", "", mac);
    CHECK(strcmp(mac, "fbdfcddea93b6c36a40ec3bb6986e831ea482f960f68e68eb87dd393b9ac7474") == 0);
}

static void test_rejects_weak_key(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    CHECK(tt_auth_init(&a, (const uint8_t *)"short", 5) == TT_AUTH_NO_KEY);
    CHECK(tt_auth_issue(&a, 0, RAND0, nonce) == TT_AUTH_NO_KEY);
    CHECK(tt_auth_verify(&a, 0, "x", "unlock", "", "y") == TT_AUTH_NO_KEY);
}

static void test_issue_and_verify(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    fresh(&a);
    CHECK(tt_auth_issue(&a, 1000, RAND0, nonce) == TT_AUTH_OK);
    CHECK(strcmp(nonce, "000102030405060708090a0b0c0d0e0f") == 0);
    CHECK(sign_and_verify(&a, 1500, nonce, "unlock", "Remote (Web)") == TT_AUTH_OK);
}

static void test_nonce_is_single_use(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    fresh(&a);
    CHECK(tt_auth_issue(&a, 0, RAND0, nonce) == TT_AUTH_OK);
    CHECK(sign_and_verify(&a, 10, nonce, "unlock", "x") == TT_AUTH_OK);
    CHECK(sign_and_verify(&a, 20, nonce, "unlock", "x") == TT_AUTH_UNKNOWN_NONCE);
}

static void test_failed_attempt_burns_nonce(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    char mac[TT_MAC_HEX_LEN + 1];
    fresh(&a);
    CHECK(tt_auth_issue(&a, 0, RAND0, nonce) == TT_AUTH_OK);
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "unlock", "x", mac);
    mac[0] = (mac[0] == '0') ? '1' : '0';
    CHECK(tt_auth_verify(&a, 5, nonce, "unlock", "x", mac) == TT_AUTH_BAD_MAC);
    CHECK(sign_and_verify(&a, 6, nonce, "unlock", "x") == TT_AUTH_UNKNOWN_NONCE);
}

static void test_expiry(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    fresh(&a);
    CHECK(tt_auth_issue(&a, 0, RAND0, nonce) == TT_AUTH_OK);
    CHECK(sign_and_verify(&a, TT_AUTH_TTL_MS + 1u, nonce, "unlock", "") == TT_AUTH_EXPIRED);
    {
        uint8_t r[16];
        random_bytes(r, 2);
        CHECK(tt_auth_issue(&a, 100000, r, nonce) == TT_AUTH_OK);
        CHECK(sign_and_verify(&a, 100000 + TT_AUTH_TTL_MS, nonce, "unlock", "") == TT_AUTH_OK);
    }
}

static void test_tampered_command_or_arg(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    char mac[TT_MAC_HEX_LEN + 1];
    uint8_t r[16];
    fresh(&a);
    CHECK(tt_auth_issue(&a, 0, RAND0, nonce) == TT_AUTH_OK);
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "delete", "7", mac);
    CHECK(tt_auth_verify(&a, 1, nonce, "delete", "8", mac) == TT_AUTH_BAD_MAC);
    random_bytes(r, 3);
    CHECK(tt_auth_issue(&a, 1000, r, nonce) == TT_AUTH_OK);
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "enroll", "7", mac);
    CHECK(tt_auth_verify(&a, 1001, nonce, "delete", "7", mac) == TT_AUTH_BAD_MAC);
}

static void test_bad_formats(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    char mac[TT_MAC_HEX_LEN + 1];
    fresh(&a);
    CHECK(tt_auth_issue(&a, 0, RAND0, nonce) == TT_AUTH_OK);
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "unlock", "", mac);
    CHECK(tt_auth_verify(&a, 1, "00", "unlock", "", mac) == TT_AUTH_BAD_FORMAT);
    CHECK(tt_auth_verify(&a, 1, nonce, "UNLOCK", "", mac) == TT_AUTH_BAD_FORMAT);
    CHECK(tt_auth_verify(&a, 1, nonce, "un|lock", "", mac) == TT_AUTH_BAD_FORMAT);
    CHECK(tt_auth_verify(&a, 1, nonce, "unlock", 0, mac) == TT_AUTH_BAD_FORMAT);
    mac[10] = 'z';
    CHECK(tt_auth_verify(&a, 1, nonce, "unlock", "", mac) == TT_AUTH_BAD_FORMAT);
}

static void test_upper_case_nonce_is_canonicalised(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    char upper[TT_NONCE_HEX_LEN + 1];
    char mac[TT_MAC_HEX_LEN + 1];
    uint8_t r[16];
    size_t i;
    fresh(&a);
    random_bytes(r, 9);
    CHECK(tt_auth_issue(&a, 0, r, nonce) == TT_AUTH_OK);
    for (i = 0; i <= TT_NONCE_HEX_LEN; ++i) {
        upper[i] = (nonce[i] >= 'a' && nonce[i] <= 'f') ? (char)(nonce[i] - 32) : nonce[i];
    }
    tt_auth_mac_hex((const uint8_t *)KEY, KEY_LEN, nonce, "unlock", "", mac);
    CHECK(tt_auth_verify(&a, 1, upper, "unlock", "", mac) == TT_AUTH_OK);
}

static void test_issue_rate_limit(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    uint8_t r[16];
    fresh(&a);
    random_bytes(r, 4);
    CHECK(tt_auth_issue(&a, 0, r, nonce) == TT_AUTH_OK);
    CHECK(tt_auth_issue(&a, TT_AUTH_ISSUE_GAP_MS - 1u, r, nonce) == TT_AUTH_RATE_LIMITED);
    CHECK(tt_auth_issue(&a, TT_AUTH_ISSUE_GAP_MS, r, nonce) == TT_AUTH_OK);
}

static void test_failure_lockout(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    uint8_t r[16];
    uint32_t i;
    uint32_t t = 0;
    const char *bogus_mac =
        "0000000000000000000000000000000000000000000000000000000000000000";
    fresh(&a);
    for (i = 0; i < TT_AUTH_MAX_FAILS; ++i) {
        random_bytes(r, (uint8_t)(10u + i));
        t += 1000u;
        CHECK(tt_auth_issue(&a, t, r, nonce) == TT_AUTH_OK);
        CHECK(tt_auth_verify(&a, t + 1u, nonce, "unlock", "", bogus_mac) == TT_AUTH_BAD_MAC);
    }
    random_bytes(r, 40);
    CHECK(tt_auth_issue(&a, t + 2000u, r, nonce) == TT_AUTH_RATE_LIMITED);
    CHECK(tt_auth_verify(&a, t + 2000u, nonce, "unlock", "", bogus_mac) == TT_AUTH_RATE_LIMITED);
    t += 1u + TT_AUTH_LOCKOUT_MS;
    CHECK(tt_auth_issue(&a, t, r, nonce) == TT_AUTH_OK);
    CHECK(sign_and_verify(&a, t + 5u, nonce, "unlock", "") == TT_AUTH_OK);
}

static void test_slot_eviction_keeps_newest(void)
{
    tt_auth_t a;
    char nonces[TT_AUTH_SLOTS + 1][TT_NONCE_HEX_LEN + 1];
    uint8_t r[16];
    uint32_t i;
    fresh(&a);
    for (i = 0; i <= TT_AUTH_SLOTS; ++i) {
        random_bytes(r, (uint8_t)(50u + i));
        CHECK(tt_auth_issue(&a, i * 1000u, r, nonces[i]) == TT_AUTH_OK);
    }
    /* The oldest challenge was evicted, the newest still works. */
    CHECK(sign_and_verify(&a, 6000u, nonces[TT_AUTH_SLOTS], "unlock", "") == TT_AUTH_OK);
    CHECK(sign_and_verify(&a, 6001u, nonces[0], "unlock", "") == TT_AUTH_UNKNOWN_NONCE);
}

static void test_millis_wraparound(void)
{
    tt_auth_t a;
    char nonce[TT_NONCE_HEX_LEN + 1];
    fresh(&a);
    CHECK(tt_auth_issue(&a, 0xFFFFF000u, RAND0, nonce) == TT_AUTH_OK);
    CHECK(sign_and_verify(&a, 0x00000100u, nonce, "unlock", "") == TT_AUTH_OK);
}

int main(void)
{
    RUN_TEST(test_known_answer_vectors);
    RUN_TEST(test_rejects_weak_key);
    RUN_TEST(test_issue_and_verify);
    RUN_TEST(test_nonce_is_single_use);
    RUN_TEST(test_failed_attempt_burns_nonce);
    RUN_TEST(test_expiry);
    RUN_TEST(test_tampered_command_or_arg);
    RUN_TEST(test_bad_formats);
    RUN_TEST(test_upper_case_nonce_is_canonicalised);
    RUN_TEST(test_issue_rate_limit);
    RUN_TEST(test_failure_lockout);
    RUN_TEST(test_slot_eviction_keeps_newest);
    RUN_TEST(test_millis_wraparound);
    return TEST_SUMMARY("test_auth");
}
