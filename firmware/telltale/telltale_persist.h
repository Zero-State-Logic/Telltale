/*
 * telltale_persist.h - save and restore the learned healthy model.
 *
 * The firmware stores a tt_persist_t blob in NVS (Preferences::putBytes) so
 * the lock keeps its learned signature across reboots. The blob carries a
 * magic number, a layout version, the model size and a CRC-32; anything that
 * does not match is rejected and the lock simply re-learns.
 *
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#ifndef TELLTALE_PERSIST_H
#define TELLTALE_PERSIST_H

#include <stddef.h>
#include <stdint.h>

#include "telltale_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TT_PERSIST_MAGIC   0x31544C54u   /* "TLT1" little-endian */
#define TT_PERSIST_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t model_bytes;
    tt_act_model_t model[TT_ACT_KINDS];
    float idle_mean;
    float idle_var;
    float idle_learned_mean;
    uint32_t idle_n;
    uint32_t crc;                 /* CRC-32 of every byte before this field */
} tt_persist_t;

uint32_t tt_crc32(const void *data, size_t len);
int tt_persist_pack(const tt_detector_t *det, tt_persist_t *out);
/* Returns TT_OK, or TT_ERR_DATA if the blob is corrupt or from another layout. */
int tt_persist_unpack(tt_detector_t *det, const tt_persist_t *in);

#ifdef __cplusplus
}
#endif

#endif /* TELLTALE_PERSIST_H */
