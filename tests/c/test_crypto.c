/*
 * test_crypto.c - SHA-256 (FIPS 180 examples), HMAC-SHA256 (RFC 4231 test
 * cases 1-7), constant-time compare and hex helpers.
 */
#include <string.h>

#include "../../firmware/telltale/telltale_sha256.h"
#include "tt_test.h"

static int digest_is(const uint8_t *digest, size_t len, const char *hex)
{
    char buf[2 * TT_SHA256_DIGEST + 1];
    tt_hex_encode(digest, len, buf);
    return strcmp(buf, hex) == 0;
}

static void test_sha256_fips_examples(void)
{
    uint8_t d[TT_SHA256_DIGEST];
    tt_sha256("", 0, d);
    CHECK(digest_is(d, 32, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
    tt_sha256("abc", 3, d);
    CHECK(digest_is(d, 32, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
    {
        const char *m = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
        tt_sha256(m, strlen(m), d);
        CHECK(digest_is(d, 32, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"));
    }
}

static void test_sha256_million_a(void)
{
    uint8_t chunk[1000];
    uint8_t d[TT_SHA256_DIGEST];
    tt_sha256_t ctx;
    int i;
    memset(chunk, 'a', sizeof(chunk));
    tt_sha256_init(&ctx);
    for (i = 0; i < 1000; ++i) {
        tt_sha256_update(&ctx, chunk, sizeof(chunk));
    }
    tt_sha256_final(&ctx, d);
    CHECK(digest_is(d, 32, "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"));
}

static void test_sha256_streaming_matches_oneshot(void)
{
    uint8_t msg[300];
    uint8_t a[TT_SHA256_DIGEST];
    uint8_t b[TT_SHA256_DIGEST];
    size_t i;
    size_t pos = 0;
    size_t step = 1;
    tt_sha256_t ctx;
    for (i = 0; i < sizeof(msg); ++i) {
        msg[i] = (uint8_t)(i * 7u + 3u);
    }
    tt_sha256(msg, sizeof(msg), a);
    tt_sha256_init(&ctx);
    while (pos < sizeof(msg)) {
        size_t take = (pos + step > sizeof(msg)) ? sizeof(msg) - pos : step;
        tt_sha256_update(&ctx, msg + pos, take);
        pos += take;
        step = step * 3u % 67u + 1u;
    }
    tt_sha256_final(&ctx, b);
    CHECK(memcmp(a, b, sizeof(a)) == 0);
}

static void check_hmac(const uint8_t *key, size_t key_len, const void *data,
                       size_t len, const char *hex, size_t compare_bytes)
{
    uint8_t mac[TT_SHA256_DIGEST];
    tt_hmac_sha256(key, key_len, data, len, mac);
    CHECK(digest_is(mac, compare_bytes, hex));
}

static void test_hmac_rfc4231(void)
{
    uint8_t key[131];
    uint8_t data[50];
    size_t i;

    memset(key, 0x0b, 20);   /* case 1 */
    check_hmac(key, 20, "Hi There", 8,
               "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7", 32);

    check_hmac((const uint8_t *)"Jefe", 4, "what do ya want for nothing?", 28,   /* case 2 */
               "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843", 32);

    memset(key, 0xaa, 20);   /* case 3 */
    memset(data, 0xdd, 50);
    check_hmac(key, 20, data, 50,
               "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe", 32);

    for (i = 0; i < 25; ++i) {   /* case 4 */
        key[i] = (uint8_t)(i + 1);
    }
    memset(data, 0xcd, 50);
    check_hmac(key, 25, data, 50,
               "82558a389a443c0ea4cc819899f2083a85f0faa3e578f8077a2e3ff46729665b", 32);

    memset(key, 0x0c, 20);   /* case 5: output truncated to 128 bits */
    check_hmac(key, 20, "Test With Truncation", 20, "a3b6167473100ee06e0c796c2955552b", 16);

    memset(key, 0xaa, 131);  /* case 6: key longer than a block */
    {
        const char *m = "Test Using Larger Than Block-Size Key - Hash Key First";
        check_hmac(key, 131, m, strlen(m),
                   "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54", 32);
    }
    {                        /* case 7: key and data longer than a block */
        const char *m = "This is a test using a larger than block-size key and a larger "
                        "than block-size data. The key needs to be hashed before being "
                        "used by the HMAC algorithm.";
        check_hmac(key, 131, m, strlen(m),
                   "9b09ffa71b942fcb27635fbcd5b0e944bfdc63644f0713938a7f51535c3a35e2", 32);
    }
}

static void test_hmac_streaming_matches_oneshot(void)
{
    const uint8_t key[] = "a-pre-shared-key-for-tests";
    const char *msg = "000102030405060708090a0b0c0d0e0f|unlock|Remote (Web)";
    uint8_t a[TT_SHA256_DIGEST];
    uint8_t b[TT_SHA256_DIGEST];
    tt_hmac_t h;
    tt_hmac_sha256(key, sizeof(key) - 1, msg, strlen(msg), a);
    tt_hmac_init(&h, key, sizeof(key) - 1);
    tt_hmac_update(&h, msg, 10);
    tt_hmac_update(&h, msg + 10, strlen(msg) - 10);
    tt_hmac_final(&h, b);
    CHECK(memcmp(a, b, sizeof(a)) == 0);
}

static void test_ct_equal(void)
{
    uint8_t a[32];
    uint8_t b[32];
    int i;
    for (i = 0; i < 32; ++i) {
        a[i] = (uint8_t)i;
        b[i] = (uint8_t)i;
    }
    CHECK(tt_ct_equal(a, b, 32) == 1);
    b[31] ^= 0x80u;
    CHECK(tt_ct_equal(a, b, 32) == 0);
    b[31] = a[31];
    b[0] ^= 0x01u;
    CHECK(tt_ct_equal(a, b, 32) == 0);
    CHECK(tt_ct_equal(a, b, 0) == 1);
}

static void test_hex_roundtrip(void)
{
    const uint8_t in[4] = {0x00, 0x7f, 0xa5, 0xff};
    char hex[9];
    uint8_t out[4];
    tt_hex_encode(in, 4, hex);
    CHECK(strcmp(hex, "007fa5ff") == 0);
    CHECK(tt_hex_decode("007FA5ff", 8, out, 4) == 0);
    CHECK(memcmp(in, out, 4) == 0);
    CHECK(tt_hex_decode("007fa5f", 7, out, 4) == -1);
    CHECK(tt_hex_decode("007fa5fg", 8, out, 4) == -1);
}

int main(void)
{
    RUN_TEST(test_sha256_fips_examples);
    RUN_TEST(test_sha256_million_a);
    RUN_TEST(test_sha256_streaming_matches_oneshot);
    RUN_TEST(test_hmac_rfc4231);
    RUN_TEST(test_hmac_streaming_matches_oneshot);
    RUN_TEST(test_ct_equal);
    RUN_TEST(test_hex_roundtrip);
    return TEST_SUMMARY("test_crypto");
}
