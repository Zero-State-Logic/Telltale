/*
 * telltale_sha256.c - SHA-256 and HMAC-SHA256, portable C99.
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#include "telltale_sha256.h"

#include <string.h>

static const uint32_t K256[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u,
    0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u,
    0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
    0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
    0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au,
    0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
};

static uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32u - n)); }

static uint32_t load_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void store_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static void compress(uint32_t state[8], const uint8_t block[64])
{
    uint32_t w[64];
    uint32_t a, b, c, d, e, f, g, h;
    uint32_t i;
    for (i = 0; i < 16u; ++i) {
        w[i] = load_be32(block + 4u * i);
    }
    for (i = 16; i < 64u; ++i) {
        const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a = state[0]; b = state[1]; c = state[2]; d = state[3];
    e = state[4]; f = state[5]; g = state[6]; h = state[7];
    for (i = 0; i < 64u; ++i) {
        const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        const uint32_t ch = (e & f) ^ (~e & g);
        const uint32_t t1 = h + s1 + ch + K256[i] + w[i];
        const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const uint32_t t2 = s0 + maj;
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
    tt_secure_zero(w, sizeof(w));
}

void tt_sha256_init(tt_sha256_t *ctx)
{
    static const uint32_t iv[8] = {
        0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
        0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u
    };
    memcpy(ctx->state, iv, sizeof(iv));
    ctx->length_bits = 0u;
    ctx->block_len = 0u;
}

void tt_sha256_update(tt_sha256_t *ctx, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    if (len == 0u || data == 0) {
        return;
    }
    ctx->length_bits += (uint64_t)len * 8u;
    while (len > 0u) {
        size_t take = TT_SHA256_BLOCK - ctx->block_len;
        if (take > len) {
            take = len;
        }
        memcpy(ctx->block + ctx->block_len, p, take);
        ctx->block_len += (uint32_t)take;
        p += take;
        len -= take;
        if (ctx->block_len == TT_SHA256_BLOCK) {
            compress(ctx->state, ctx->block);
            ctx->block_len = 0u;
        }
    }
}

void tt_sha256_final(tt_sha256_t *ctx, uint8_t out[TT_SHA256_DIGEST])
{
    uint32_t i;
    const uint64_t bits = ctx->length_bits;
    ctx->block[ctx->block_len++] = 0x80u;
    if (ctx->block_len > 56u) {
        memset(ctx->block + ctx->block_len, 0, TT_SHA256_BLOCK - ctx->block_len);
        compress(ctx->state, ctx->block);
        ctx->block_len = 0u;
    }
    memset(ctx->block + ctx->block_len, 0, 56u - ctx->block_len);
    for (i = 0; i < 8u; ++i) {
        ctx->block[56u + i] = (uint8_t)(bits >> (56u - 8u * i));
    }
    compress(ctx->state, ctx->block);
    for (i = 0; i < 8u; ++i) {
        store_be32(out + 4u * i, ctx->state[i]);
    }
    tt_secure_zero(ctx, sizeof(*ctx));
}

void tt_sha256(const void *data, size_t len, uint8_t out[TT_SHA256_DIGEST])
{
    tt_sha256_t ctx;
    tt_sha256_init(&ctx);
    tt_sha256_update(&ctx, data, len);
    tt_sha256_final(&ctx, out);
}

void tt_hmac_init(tt_hmac_t *ctx, const uint8_t *key, size_t key_len)
{
    uint8_t k0[TT_SHA256_BLOCK];
    uint8_t ipad[TT_SHA256_BLOCK];
    uint32_t i;
    memset(k0, 0, sizeof(k0));
    if (key_len > TT_SHA256_BLOCK) {
        tt_sha256(key, key_len, k0);   /* long keys are hashed first */
    } else if (key_len > 0u && key != 0) {
        memcpy(k0, key, key_len);
    }
    for (i = 0; i < TT_SHA256_BLOCK; ++i) {
        ipad[i] = (uint8_t)(k0[i] ^ 0x36u);
        ctx->opad_key[i] = (uint8_t)(k0[i] ^ 0x5cu);
    }
    tt_sha256_init(&ctx->inner);
    tt_sha256_update(&ctx->inner, ipad, sizeof(ipad));
    tt_secure_zero(k0, sizeof(k0));
    tt_secure_zero(ipad, sizeof(ipad));
}

void tt_hmac_update(tt_hmac_t *ctx, const void *data, size_t len)
{
    tt_sha256_update(&ctx->inner, data, len);
}

void tt_hmac_final(tt_hmac_t *ctx, uint8_t out[TT_SHA256_DIGEST])
{
    uint8_t inner_digest[TT_SHA256_DIGEST];
    tt_sha256_t outer;
    tt_sha256_final(&ctx->inner, inner_digest);
    tt_sha256_init(&outer);
    tt_sha256_update(&outer, ctx->opad_key, TT_SHA256_BLOCK);
    tt_sha256_update(&outer, inner_digest, sizeof(inner_digest));
    tt_sha256_final(&outer, out);
    tt_secure_zero(inner_digest, sizeof(inner_digest));
    tt_secure_zero(ctx, sizeof(*ctx));
}

void tt_hmac_sha256(const uint8_t *key, size_t key_len, const void *data,
                    size_t len, uint8_t out[TT_SHA256_DIGEST])
{
    tt_hmac_t ctx;
    tt_hmac_init(&ctx, key, key_len);
    tt_hmac_update(&ctx, data, len);
    tt_hmac_final(&ctx, out);
}

int tt_ct_equal(const uint8_t *a, const uint8_t *b, size_t len)
{
    uint32_t diff = 0u;
    size_t i;
    for (i = 0; i < len; ++i) {
        diff |= (uint32_t)(a[i] ^ b[i]);
    }
    /* Map 0 -> 1 and any non-zero -> 0 without a data-dependent branch. */
    return (int)(1u & ((diff - 1u) >> 8));
}

void tt_hex_encode(const uint8_t *in, size_t len, char *out)
{
    static const char digits[] = "0123456789abcdef";
    size_t i;
    for (i = 0; i < len; ++i) {
        out[2u * i] = digits[in[i] >> 4];
        out[2u * i + 1u] = digits[in[i] & 0x0Fu];
    }
    out[2u * len] = '\0';
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

int tt_hex_decode(const char *hex, size_t hex_len, uint8_t *out, size_t out_len)
{
    size_t i;
    if (hex == 0 || out == 0 || hex_len != 2u * out_len) {
        return -1;
    }
    for (i = 0; i < out_len; ++i) {
        const int hi = hex_value(hex[2u * i]);
        const int lo = hex_value(hex[2u * i + 1u]);
        if (hi < 0 || lo < 0) {
            return -1;
        }
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return 0;
}

void tt_secure_zero(void *buf, size_t len)
{
    volatile uint8_t *p = (volatile uint8_t *)buf;
    while (len-- > 0u) {
        *p++ = 0u;
    }
}
