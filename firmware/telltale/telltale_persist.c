/*
 * telltale_persist.c - save and restore the learned model (portable C99).
 * Copyright (c) 2026 Faysal Ali Shah. MIT License.
 */
#include "telltale_persist.h"

#include <string.h>

uint32_t tt_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    size_t i;
    uint32_t bit;
    if (data == 0) {
        return 0u;
    }
    for (i = 0; i < len; ++i) {
        crc ^= p[i];
        for (bit = 0; bit < 8u; ++bit) {
            const uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t blob_crc(const tt_persist_t *blob)
{
    return tt_crc32(blob, offsetof(tt_persist_t, crc));
}

int tt_persist_pack(const tt_detector_t *det, tt_persist_t *out)
{
    if (det == 0 || out == 0) {
        return TT_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));   /* deterministic padding for the CRC */
    out->magic = TT_PERSIST_MAGIC;
    out->version = TT_PERSIST_VERSION;
    out->model_bytes = (uint32_t)sizeof(det->model);
    memcpy(out->model, det->model, sizeof(det->model));
    out->idle_mean = det->idle.mean;
    out->idle_var = det->idle.var;
    out->idle_learned_mean = det->idle.learned_mean;
    out->idle_n = det->idle.n;
    out->crc = blob_crc(out);
    return TT_OK;
}

int tt_persist_unpack(tt_detector_t *det, const tt_persist_t *in)
{
    if (det == 0 || in == 0) {
        return TT_ERR_ARG;
    }
    if (in->magic != TT_PERSIST_MAGIC || in->version != TT_PERSIST_VERSION ||
        in->model_bytes != (uint32_t)sizeof(det->model) || in->crc != blob_crc(in)) {
        return TT_ERR_DATA;
    }
    memcpy(det->model, in->model, sizeof(det->model));
    det->idle.mean = in->idle_mean;
    det->idle.var = in->idle_var;
    det->idle.learned_mean = in->idle_learned_mean;
    det->idle.n = in->idle_n;
    return TT_OK;
}
