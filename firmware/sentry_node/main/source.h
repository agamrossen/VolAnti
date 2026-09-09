/*
 * source.h - where samples come from.
 *
 * The detector must not know. Tonight the only real source is a const array
 * in flash; the microphones arrive at Stage 1b. Both go behind one call so
 * the swap is a constructor change and nothing else:
 *
 *     int source_read(source_t *s, int16_t *dst, size_t n);
 *
 * Returns the number of int16 samples written, or a negative esp_err_t.
 * source_i2s() exists TODAY as an error-returning stub: a named hole is
 * honest, a missing function invites the detector to grow a second entry
 * point that bypasses this one.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

struct source_s;

typedef int (*source_read_fn)(struct source_s *s, int16_t *dst, size_t n);

typedef struct source_s {
    const char    *name;
    const char    *preset;   /* the preset this vector was cut at */
    double         thr;      /* ... and its threshold. NOT an expected outcome */
    source_read_fn read;
    /* golden replay state */
    const int16_t *q;
    uint32_t       n_total;
    uint32_t       pos;
} source_t;

static inline int source_read(source_t *s, int16_t *dst, size_t n)
{
    return s->read(s, dst, n);
}

/* index into GOLDEN_VECTORS[] from generated/vectors.h */
esp_err_t source_golden(source_t *s, int index);
const char *source_golden_name(int index);
const char *source_golden_preset(int index);
double source_golden_thr(int index);
uint32_t source_golden_len(int index);
int source_golden_count(void);

/* The real microphone. Implemented in source_i2s.c; pins in board_pins.h.
 * Brings the I2S channel up on first call and leaves it up until
 * source_i2s_stop(), so a mode can be re-entered without re-clocking. */
esp_err_t source_i2s(source_t *s);
void source_i2s_stop(void);
uint32_t source_i2s_short_reads(void);
uint32_t source_i2s_timeouts(void);
