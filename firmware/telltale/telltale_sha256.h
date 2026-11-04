/*
 * telltale_sha256.h - SHA-256 (FIPS 180-4) and HMAC-SHA256 (RFC 2104),
 * portable C99 with no heap. Verified against the FIPS 180 examples and the
 * RFC 4231 test vectors in tests/c/test_crypto.c.
 *
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#ifndef TELLTALE_SHA256_H
#define TELLTALE_SHA256_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TT_SHA256_BLOCK  64u
#define TT_SHA256_DIGEST 32u

typedef struct {
    uint32_t state[8];
    uint64_t length_bits;
    uint8_t block[TT_SHA256_BLOCK];
    uint32_t block_len;
} tt_sha256_t;

typedef struct {
    tt_sha256_t inner;
    uint8_t opad_key[TT_SHA256_BLOCK];
} tt_hmac_t;

void tt_sha256_init(tt_sha256_t *ctx);
void tt_sha256_update(tt_sha256_t *ctx, const void *data, size_t len);
void tt_sha256_final(tt_sha256_t *ctx, uint8_t out[TT_SHA256_DIGEST]);
void tt_sha256(const void *data, size_t len, uint8_t out[TT_SHA256_DIGEST]);

void tt_hmac_init(tt_hmac_t *ctx, const uint8_t *key, size_t key_len);
void tt_hmac_update(tt_hmac_t *ctx, const void *data, size_t len);
void tt_hmac_final(tt_hmac_t *ctx, uint8_t out[TT_SHA256_DIGEST]);
void tt_hmac_sha256(const uint8_t *key, size_t key_len, const void *data,
                    size_t len, uint8_t out[TT_SHA256_DIGEST]);

/* Constant-time comparison: returns 1 when equal, 0 otherwise. The running
 * time depends only on len, never on where the buffers differ. */
int tt_ct_equal(const uint8_t *a, const uint8_t *b, size_t len);

/* Lower-case hex encoding; out must hold 2*len + 1 bytes. */
void tt_hex_encode(const uint8_t *in, size_t len, char *out);
/* Strict hex decoding (either case). Returns 0 on success, -1 on bad input. */
int tt_hex_decode(const char *hex, size_t hex_len, uint8_t *out, size_t out_len);

/* Wipe a buffer in a way the compiler will not optimise away. */
void tt_secure_zero(void *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* TELLTALE_SHA256_H */
