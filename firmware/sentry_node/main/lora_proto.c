#include "lora_proto.h"

#include <string.h>

uint16_t lora_crc16(const uint8_t *p, int n)
{
    uint16_t c = 0xFFFFu;
    for (int i = 0; i < n; i++) {
        c ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++) {
            c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u)
                              : (uint16_t)(c << 1);
        }
    }
    return c;
}

void lora_pkt_build(uint8_t *dst, const lora_pkt_t *p)
{
    /* Zero FIRST, so the seven reserved bytes are reserved rather than
     * whatever was on the caller's stack. Eighteen bytes of uninitialised
     * memory is not a leak worth having on an air interface. */
    memset(dst, 0, LORA_PKT_BYTES);
    dst[0] = LORA_MAGIC_0;
    dst[1] = LORA_MAGIC_1;
    dst[2] = LORA_PROTO_VER;
    dst[3] = p->flags;
    dst[4] = (uint8_t)(p->device_id & 0xFFu);
    dst[5] = (uint8_t)(p->device_id >> 8);
    dst[6] = p->seq;
    dst[7] = p->hop;
    dst[8] = p->tier;
    dst[9] = p->epoch;
    dst[10] = (uint8_t)(p->score_x100 & 0xFFu);
    dst[11] = (uint8_t)(p->score_x100 >> 8);
    dst[12] = (uint8_t)(p->thr1_milli & 0xFFu);
    dst[13] = (uint8_t)(p->thr1_milli >> 8);
    /* 14..15 stay zero */
    const uint16_t c = lora_crc16(dst, LORA_PKT_BYTES - 2);
    dst[16] = (uint8_t)(c & 0xFFu);
    dst[17] = (uint8_t)(c >> 8);
}

int lora_pkt_parse(const uint8_t *src, int n, lora_pkt_t *out)
{
    /* ORDER MATTERS. Length before any indexing, magic before the CRC, and
     * the CRC before anything is believed. A parser that checked the version
     * field of a packet whose length it had not established would read past
     * the buffer on the first truncated frame the air ever delivered. */
    if (n != LORA_PKT_BYTES) {
        return LORA_RX_BAD_LEN;
    }
    if (src[0] != LORA_MAGIC_0 || src[1] != LORA_MAGIC_1) {
        return LORA_RX_BAD_MAGIC;
    }
    const uint16_t want = (uint16_t)src[16] | ((uint16_t)src[17] << 8);
    if (lora_crc16(src, LORA_PKT_BYTES - 2) != want) {
        return LORA_RX_BAD_CRC;
    }
    /* Version AFTER the CRC: a corrupted version byte in an otherwise valid
     * v0 packet must be reported as corruption, not as a future protocol. */
    if (src[2] != LORA_PROTO_VER) {
        return LORA_RX_BAD_VER;
    }
    if (src[7] != 0u) {
        /* No v0 device originates a non-zero hop, so this is either a future
         * relay or a fault. Either way v0 does not act on it - which is what
         * makes a storm impossible by construction rather than by policy. */
        return LORA_RX_RELAYED;
    }
    out->flags = src[3];
    out->device_id = (uint16_t)src[4] | ((uint16_t)src[5] << 8);
    out->seq = src[6];
    out->hop = src[7];
    out->epoch = src[9];
    out->score_x100 = (uint16_t)src[10] | ((uint16_t)src[11] << 8);
    out->thr1_milli = (uint16_t)src[12] | ((uint16_t)src[13] << 8);
    out->tier = src[8];
    return LORA_RX_OK;
}

void lora_dedup_reset(lora_dedup_t *d)
{
    memset(d, 0, sizeof(*d));
}

bool lora_dedup_admit(lora_dedup_t *d, uint16_t id, uint8_t seq,
                      uint8_t epoch, uint32_t now_ms)
{
    int free_slot = -1;
    for (int i = 0; i < LORA_DEDUP_N; i++) {
        if (!d->used[i]) {
            if (free_slot < 0) {
                free_slot = i;
            }
            continue;
        }
        /* Expire on the way past. Unsigned subtraction, so the uint32
         * millisecond clock wrapping after 49.7 days is harmless - and this
         * device is designed to be left guarding for weeks. */
        if ((uint32_t)(now_ms - d->t_ms[i]) >= LORA_DEDUP_MS) {
            d->used[i] = 0;
            if (free_slot < 0) {
                free_slot = i;
            }
            continue;
        }
        if (d->id[i] == id && d->seq[i] == seq && d->epoch[i] == epoch) {
            return false;               /* a repeat of the same burst */
        }
    }
    /* No free slot means sixteen distinct peers have alerted inside sixty
     * seconds. Overwrite the oldest-claimed slot round-robin rather than
     * refuse: dropping a NEW alert to protect a dedupe table would be the
     * wrong way round for a device whose miss cost is a person. */
    const int i = (free_slot >= 0) ? free_slot : (d->next % LORA_DEDUP_N);
    d->next = (uint8_t)((i + 1) % LORA_DEDUP_N);
    d->id[i] = id;
    d->seq[i] = seq;
    d->epoch[i] = epoch;
    d->t_ms[i] = now_ms;
    d->used[i] = 1;
    return true;
}

uint32_t lora_tx_gap(uint32_t rnd)
{
    const uint32_t span = 2u * LORA_TX_JITTER_MS + 1u;
    return LORA_TX_GAP_MS - LORA_TX_JITTER_MS + (rnd % span);
}

uint16_t lora_id_from_mac(const uint8_t mac[6])
{
    return lora_crc16(mac, 6);
}

/* See the header: the whole receive decision, with no SPI in it. */
int lora_rx_admit(lora_dedup_t *d, const uint8_t *src, int n,
                  uint16_t self_id, uint32_t now_ms, lora_pkt_t *out)
{
    lora_pkt_t p;
    const int why = lora_pkt_parse(src, n, &p);
    if (why != LORA_RX_OK) {
        return why;
    }
    /* SELF BEFORE DEDUPE, deliberately. Entering our own id into our own ring
     * would let it reject a genuine peer that shared a sequence number. */
    if (p.device_id == self_id) {
        return LORA_RX_SELF;
    }
    if (!lora_dedup_admit(d, p.device_id, p.seq, p.epoch, now_ms)) {
        return LORA_RX_DUP;
    }
    if (out) {
        *out = p;
    }
    return LORA_RX_OK;
}
