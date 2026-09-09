#include "source.h"

#include <string.h>

#include "generated/vectors.h"

static int golden_read(source_t *s, int16_t *dst, size_t n)
{
    if (s->pos >= s->n_total) {
        return 0;
    }
    size_t avail = s->n_total - s->pos;
    if (n > avail) {
        n = avail;
    }
    memcpy(dst, s->q + s->pos, n * sizeof(int16_t));
    s->pos += n;
    return (int)n;
}

esp_err_t source_golden(source_t *s, int index)
{
    if (index < 0 || index >= GOLDEN_N_VECTORS) {
        return ESP_ERR_INVALID_ARG;
    }
    s->name = GOLDEN_VECTORS[index].name;
    s->preset = GOLDEN_VECTORS[index].preset;
    s->thr = GOLDEN_VECTORS[index].thr;
    s->read = golden_read;
    s->q = GOLDEN_VECTORS[index].q;
    s->n_total = GOLDEN_VECTORS[index].n;
    s->pos = 0;
    return ESP_OK;
}

const char *source_golden_name(int index)
{
    return (index >= 0 && index < GOLDEN_N_VECTORS)
           ? GOLDEN_VECTORS[index].name : "?";
}

const char *source_golden_preset(int index)
{
    return (index >= 0 && index < GOLDEN_N_VECTORS)
           ? GOLDEN_VECTORS[index].preset : "?";
}

double source_golden_thr(int index)
{
    return (index >= 0 && index < GOLDEN_N_VECTORS)
           ? GOLDEN_VECTORS[index].thr : 0.0;
}

uint32_t source_golden_len(int index)
{
    return (index >= 0 && index < GOLDEN_N_VECTORS)
           ? GOLDEN_VECTORS[index].n : 0;
}

int source_golden_count(void)
{
    return GOLDEN_N_VECTORS;
}

/* source_i2s() lives in source_i2s.c - it needs the driver headers and this
 * file must stay compilable with no hardware in sight. */
