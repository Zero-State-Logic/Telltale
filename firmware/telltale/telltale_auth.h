/*
 * telltale_auth.h - challenge/response authentication for remote commands.
 *
 * Protocol (also documented in docs/API.md):
 *   1. GET /challenge                      -> {"nonce":"<32 hex>","ttl_ms":30000}
 *   2. mac = HMAC-SHA256(psk, nonce + "|" + command + "|" + arg)   (lower-case hex)
 *   3. POST /<command> {"nonce":..., "arg":..., "mac":...}
 *
 * Each nonce is single use (consumed on the first verification attempt,
 * whatever the outcome) and expires after TT_AUTH_TTL_MS. MACs are compared
 * in constant time. Repeated failures lock remote commands out for a while.
 * The message has no ambiguity: the nonce has a fixed length and the command
 * names contain no '|', so the argument may contain any bytes.
 *
 * Portable C99, no heap. Time is passed in by the caller (millis()), and so
 * is the randomness for nonces (esp_random() on the ESP32), which keeps the
 * module deterministic under test.
 *
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#ifndef TELLTALE_AUTH_H
#define TELLTALE_AUTH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TT_AUTH_SLOTS          4u      /* outstanding challenges */
#define TT_NONCE_BYTES         16u
#define TT_NONCE_HEX_LEN       32u
#define TT_MAC_HEX_LEN         64u
#define TT_AUTH_TTL_MS         30000u  /* challenge lifetime */
#define TT_AUTH_ISSUE_GAP_MS   250u    /* at most 4 challenges per second */
#define TT_AUTH_FAIL_WINDOW_MS 60000u  /* failures counted over this window */
#define TT_AUTH_MAX_FAILS      5u      /* failures in the window before lockout */
#define TT_AUTH_LOCKOUT_MS     60000u  /* lockout length */
#define TT_AUTH_MIN_KEY_LEN    16u     /* refuse shorter pre-shared keys */
#define TT_AUTH_MAX_KEY_LEN    64u

typedef enum {
    TT_AUTH_OK = 0,
    TT_AUTH_BAD_FORMAT = 1,     /* malformed nonce, MAC or command */
    TT_AUTH_UNKNOWN_NONCE = 2,  /* never issued, already used, or evicted */
    TT_AUTH_EXPIRED = 3,        /* issued more than TT_AUTH_TTL_MS ago */
    TT_AUTH_BAD_MAC = 4,
    TT_AUTH_RATE_LIMITED = 5,   /* lockout active or challenges too fast */
    TT_AUTH_NO_KEY = 6          /* no usable key configured */
} tt_auth_status_t;

typedef struct {
    uint8_t nonce[TT_NONCE_BYTES];
    uint32_t issued_ms;
    uint8_t in_use;
} tt_nonce_slot_t;

typedef struct {
    uint8_t key[TT_AUTH_MAX_KEY_LEN];
    uint32_t key_len;
    tt_nonce_slot_t slots[TT_AUTH_SLOTS];
    uint32_t last_issue_ms;
    uint32_t fail_window_start_ms;
    uint32_t fail_count;
    uint32_t locked_until_ms;
    uint8_t has_issued;
    uint8_t locked;
    uint8_t has_key;
    uint8_t pad_;
} tt_auth_t;

/* Returns TT_AUTH_OK, or TT_AUTH_NO_KEY if the key is shorter than
 * TT_AUTH_MIN_KEY_LEN or longer than TT_AUTH_MAX_KEY_LEN. */
tt_auth_status_t tt_auth_init(tt_auth_t *ctx, const uint8_t *key, size_t key_len);

/* Issues a fresh challenge from 16 caller-supplied random bytes. */
tt_auth_status_t tt_auth_issue(tt_auth_t *ctx, uint32_t now_ms,
                               const uint8_t random_bytes[TT_NONCE_BYTES],
                               char nonce_hex_out[TT_NONCE_HEX_LEN + 1]);

/* Verifies one command. arg may be "" but not NULL. */
tt_auth_status_t tt_auth_verify(tt_auth_t *ctx, uint32_t now_ms,
                                const char *nonce_hex, const char *command,
                                const char *arg, const char *mac_hex);

/* Computes the expected MAC (hex) for a message; used by tests and tools. */
void tt_auth_mac_hex(const uint8_t *key, size_t key_len, const char *nonce_hex,
                     const char *command, const char *arg,
                     char mac_hex_out[TT_MAC_HEX_LEN + 1]);

const char *tt_auth_status_name(tt_auth_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* TELLTALE_AUTH_H */
