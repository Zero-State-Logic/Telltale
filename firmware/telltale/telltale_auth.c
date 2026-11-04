/*
 * telltale_auth.c - challenge/response authentication (portable C99).
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#include "telltale_auth.h"
#include "telltale_sha256.h"

#include <string.h>

/* Unsigned subtraction keeps working across the 49-day millis() wrap. */
static uint32_t elapsed_ms(uint32_t now, uint32_t then) { return now - then; }

tt_auth_status_t tt_auth_init(tt_auth_t *ctx, const uint8_t *key, size_t key_len)
{
    if (ctx == 0) {
        return TT_AUTH_NO_KEY;
    }
    memset(ctx, 0, sizeof(*ctx));
    if (key == 0 || key_len < TT_AUTH_MIN_KEY_LEN || key_len > TT_AUTH_MAX_KEY_LEN) {
        return TT_AUTH_NO_KEY;
    }
    memcpy(ctx->key, key, key_len);
    ctx->key_len = (uint32_t)key_len;
    ctx->has_key = 1u;
    return TT_AUTH_OK;
}

static int lockout_active(tt_auth_t *ctx, uint32_t now_ms)
{
    if (!ctx->locked) {
        return 0;
    }
    if (elapsed_ms(now_ms, ctx->locked_until_ms) < 0x80000000u) {
        /* now >= locked_until (modulo wrap): lockout over */
        ctx->locked = 0u;
        ctx->fail_count = 0u;
        return 0;
    }
    return 1;
}

static void record_failure(tt_auth_t *ctx, uint32_t now_ms)
{
    if (ctx->fail_count == 0u ||
        elapsed_ms(now_ms, ctx->fail_window_start_ms) > TT_AUTH_FAIL_WINDOW_MS) {
        ctx->fail_window_start_ms = now_ms;
        ctx->fail_count = 0u;
    }
    ctx->fail_count++;
    if (ctx->fail_count >= TT_AUTH_MAX_FAILS) {
        ctx->locked = 1u;
        ctx->locked_until_ms = now_ms + TT_AUTH_LOCKOUT_MS;
    }
}

static uint32_t pick_slot(const tt_auth_t *ctx, uint32_t now_ms)
{
    uint32_t i;
    uint32_t oldest = 0u;
    uint32_t oldest_age = 0u;
    for (i = 0; i < TT_AUTH_SLOTS; ++i) {
        const uint32_t age = elapsed_ms(now_ms, ctx->slots[i].issued_ms);
        if (!ctx->slots[i].in_use || age > TT_AUTH_TTL_MS) {
            return i;
        }
        if (age >= oldest_age) {
            oldest_age = age;
            oldest = i;
        }
    }
    return oldest;   /* all live: evict the oldest */
}

tt_auth_status_t tt_auth_issue(tt_auth_t *ctx, uint32_t now_ms,
                               const uint8_t random_bytes[TT_NONCE_BYTES],
                               char nonce_hex_out[TT_NONCE_HEX_LEN + 1])
{
    uint32_t slot;
    if (ctx == 0 || random_bytes == 0 || nonce_hex_out == 0) {
        return TT_AUTH_BAD_FORMAT;
    }
    if (!ctx->has_key) {
        return TT_AUTH_NO_KEY;
    }
    if (lockout_active(ctx, now_ms)) {
        return TT_AUTH_RATE_LIMITED;
    }
    if (ctx->has_issued && elapsed_ms(now_ms, ctx->last_issue_ms) < TT_AUTH_ISSUE_GAP_MS) {
        return TT_AUTH_RATE_LIMITED;
    }
    slot = pick_slot(ctx, now_ms);
    memcpy(ctx->slots[slot].nonce, random_bytes, TT_NONCE_BYTES);
    ctx->slots[slot].issued_ms = now_ms;
    ctx->slots[slot].in_use = 1u;
    ctx->last_issue_ms = now_ms;
    ctx->has_issued = 1u;
    tt_hex_encode(random_bytes, TT_NONCE_BYTES, nonce_hex_out);
    return TT_AUTH_OK;
}

static int command_ok(const char *command)
{
    size_t i;
    if (command == 0 || command[0] == '\0') {
        return 0;
    }
    for (i = 0; command[i] != '\0'; ++i) {
        const char c = command[i];
        if (i >= 16u || !((c >= 'a' && c <= 'z') || c == '_')) {
            return 0;
        }
    }
    return 1;
}

static void mac_bytes(const uint8_t *key, size_t key_len, const char *nonce_hex,
                      const char *command, const char *arg,
                      uint8_t out[TT_SHA256_DIGEST])
{
    tt_hmac_t h;
    tt_hmac_init(&h, key, key_len);
    tt_hmac_update(&h, nonce_hex, TT_NONCE_HEX_LEN);
    tt_hmac_update(&h, "|", 1u);
    tt_hmac_update(&h, command, strlen(command));
    tt_hmac_update(&h, "|", 1u);
    tt_hmac_update(&h, arg, strlen(arg));
    tt_hmac_final(&h, out);
}

void tt_auth_mac_hex(const uint8_t *key, size_t key_len, const char *nonce_hex,
                     const char *command, const char *arg,
                     char mac_hex_out[TT_MAC_HEX_LEN + 1])
{
    uint8_t mac[TT_SHA256_DIGEST];
    mac_bytes(key, key_len, nonce_hex, command, arg, mac);
    tt_hex_encode(mac, sizeof(mac), mac_hex_out);
}

/* Finds and consumes the slot holding this nonce. Returns its index or -1. */
static int take_slot(tt_auth_t *ctx, const uint8_t nonce[TT_NONCE_BYTES])
{
    uint32_t i;
    int found = -1;
    for (i = 0; i < TT_AUTH_SLOTS; ++i) {
        if (ctx->slots[i].in_use &&
            tt_ct_equal(ctx->slots[i].nonce, nonce, TT_NONCE_BYTES)) {
            found = (int)i;
        }
    }
    if (found >= 0) {
        ctx->slots[found].in_use = 0u;
    }
    return found;
}

static tt_auth_status_t fail(tt_auth_t *ctx, uint32_t now_ms, tt_auth_status_t why)
{
    record_failure(ctx, now_ms);
    return why;
}

tt_auth_status_t tt_auth_verify(tt_auth_t *ctx, uint32_t now_ms,
                                const char *nonce_hex, const char *command,
                                const char *arg, const char *mac_hex)
{
    uint8_t nonce[TT_NONCE_BYTES];
    uint8_t mac_given[TT_SHA256_DIGEST];
    uint8_t mac_expect[TT_SHA256_DIGEST];
    int slot;
    int equal;
    if (ctx == 0) {
        return TT_AUTH_BAD_FORMAT;
    }
    if (!ctx->has_key) {
        return TT_AUTH_NO_KEY;
    }
    if (lockout_active(ctx, now_ms)) {
        return TT_AUTH_RATE_LIMITED;
    }
    if (nonce_hex == 0 || mac_hex == 0 || arg == 0 || !command_ok(command) ||
        strlen(nonce_hex) != TT_NONCE_HEX_LEN || strlen(mac_hex) != TT_MAC_HEX_LEN ||
        tt_hex_decode(nonce_hex, TT_NONCE_HEX_LEN, nonce, sizeof(nonce)) != 0 ||
        tt_hex_decode(mac_hex, TT_MAC_HEX_LEN, mac_given, sizeof(mac_given)) != 0) {
        return fail(ctx, now_ms, TT_AUTH_BAD_FORMAT);
    }
    slot = take_slot(ctx, nonce);
    if (slot < 0) {
        return fail(ctx, now_ms, TT_AUTH_UNKNOWN_NONCE);
    }
    if (elapsed_ms(now_ms, ctx->slots[slot].issued_ms) > TT_AUTH_TTL_MS) {
        return fail(ctx, now_ms, TT_AUTH_EXPIRED);
    }
    /* The MAC covers the nonce exactly as issued (lower-case hex). */
    {
        char canonical[TT_NONCE_HEX_LEN + 1];
        tt_hex_encode(nonce, sizeof(nonce), canonical);
        mac_bytes(ctx->key, ctx->key_len, canonical, command, arg, mac_expect);
    }
    equal = tt_ct_equal(mac_expect, mac_given, sizeof(mac_expect));
    tt_secure_zero(mac_expect, sizeof(mac_expect));
    if (!equal) {
        return fail(ctx, now_ms, TT_AUTH_BAD_MAC);
    }
    return TT_AUTH_OK;
}

const char *tt_auth_status_name(tt_auth_status_t status)
{
    switch (status) {
    case TT_AUTH_OK: return "ok";
    case TT_AUTH_BAD_FORMAT: return "bad_format";
    case TT_AUTH_UNKNOWN_NONCE: return "unknown_nonce";
    case TT_AUTH_EXPIRED: return "expired";
    case TT_AUTH_BAD_MAC: return "bad_mac";
    case TT_AUTH_RATE_LIMITED: return "rate_limited";
    case TT_AUTH_NO_KEY: return "no_key";
    default: return "unknown";
    }
}
