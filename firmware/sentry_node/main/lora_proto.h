/*
 * lora_proto.h - the peer alert beacon: eighteen bytes, and the rules that
 * stop eighteen bytes becoming a storm.
 *
 * Scope, and it is absolute. This project is acoustic detection and local
 * alerting. The radio carries one message - "I just raised an alert" - and
 * nothing else, ever. No commands, no telemetry pull, no remote
 * configuration, no relaying, no countermeasure of any kind. The packet below
 * is the whole protocol, and the reserved bytes exist so a later version can
 * add a name, not a capability.
 *
 * Every device is equal: any device that raises a local alert broadcasts it,
 * and every device that hears a broadcast raises a remote alert and says who
 * it came from. No roles, no modes, no pairing ceremony, no per-device
 * firmware builds.
 *
 * There is no ESP-IDF in this file. Everything here is bytes and integers -
 * build, parse, CRC, a dedupe ring and a jitter bound - and none of it needs
 * SPI, a radio, a task or a clock, because the caller passes the milliseconds
 * in. The host tests compile this file directly and drive it through ctypes,
 * which is what makes an air test on two boards a hardware check rather than
 * a logic hunt.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ---- the v0 packet, frozen -------------------------------------------- */
/*
 *   0-1   magic       0x53 0x4E   "SN", in that byte order on the air
 *   2     proto_ver   0
 *   3     flags       bit0 = TEST message
 *                     bit1 = LINK REQUEST, bit2 = LINK REPLY (see below)
 *   4-5   device_id   CRC16(eFuse MAC), little-endian
 *   6     seq         per-device counter, +1 per local alert onset
 *   7     hop         0 in v0. Receivers ignore packets with hop != 0.
 *   8     tier        1=V1 2=T2 3=T3 4=T4, the originating alert's tier
 *   9     epoch       sender's boot counter, low byte. An older sender sends
 *                     0, which is what makes this a reserved-byte change
 *                     rather than a version bump.
 *   10-11 score       the score at the decision, x100, little-endian. 0 means
 *                     "not reported", which an older sender also sends.
 *   12-13 thr1_milli  sender's v1 threshold in milli-units, little-endian.
 *                     0 means "not reported". Two units can differ by this
 *                     one constant, so a remote alert that did not carry it
 *                     would be unreadable on the other board.
 *   14-15 reserved    0
 *   16-17 crc         CRC16-CCITT over bytes 0-15, little-endian
 *
 * Growth path: a layout change bumps proto_ver and v0 receivers drop the
 * packet silently. Fields that are meaningful as zero go in `reserved` with
 * no bump at all, which is what makes "add the device's name later" a
 * decision that can be taken later.
 */
#define LORA_PKT_BYTES   18
#define LORA_PROTO_VER   0u
#define LORA_MAGIC_0     0x53u
#define LORA_MAGIC_1     0x4Eu
#define LORA_FLAG_TEST   0x01u

/* ---- the link test ------------------------------------------------------
 * Confirming in a field that two units can hear each other needs a round
 * trip: a one-way transmission tells the sender nothing. So a link request is
 * answered by a link reply and the sender reports the result.
 *
 * That is a device transmitting in response to a received packet, which is
 * the shape this protocol otherwise forbids by construction. A reply that
 * could itself be replied to would give that property away for a convenience,
 * so the exchange is bounded at two messages structurally rather than by
 * policy:
 *
 *     a link request is answered, exactly once per dedup key
 *     a link reply is never answered, by anything, ever
 *
 * The second line is the whole safety argument. Two units exchange one
 * request and one reply and then fall silent, and no arrangement of units,
 * duplicates or reflections can produce a third message.
 *
 * Neither carries an alert. They raise no alarm, no alert window and no ALERT
 * screen - a beep, a short pulse and a line on the page. */
#define LORA_FLAG_LINKREQ 0x02u
#define LORA_FLAG_LINKACK 0x04u
#define LORA_FLAG_LINK    (LORA_FLAG_LINKREQ | LORA_FLAG_LINKACK)

typedef struct {
    uint16_t device_id;
    uint8_t  seq;
    uint8_t  hop;
    uint8_t  tier;
    uint8_t  flags;
    /* Both of these live in what were reserved bytes, so proto_ver does not
     * move and an older image still parses these frames - it reads 0 for
     * both, which is what "meaningful as zero" in the growth path above was
     * reserved to mean. */
    uint8_t  epoch;
    uint16_t score_x100;
    uint16_t thr1_milli;
} lora_pkt_t;

/* Parse verdicts. Every rejection has its own code, because "the radio is
 * receiving nothing" and "the radio is receiving something it will not
 * accept" are completely different faults and a single false would merge
 * them. `Lt` and the event ring report the counts. */
#define LORA_RX_OK        0
#define LORA_RX_BAD_LEN   1
#define LORA_RX_BAD_MAGIC 2
#define LORA_RX_BAD_VER   3
#define LORA_RX_BAD_CRC   4
#define LORA_RX_RELAYED   5   /* hop != 0 - no v0 device may originate one */
/* Appended, never inserted: 0..5 are printed by `Lt` and mirrored in the host
 * tests, so renumbering them would silently change what a field morning reads
 * off the board. */
#define LORA_RX_SELF      6   /* our own id - a device never triggers off    */
                              /* its own repeats                             */
#define LORA_RX_DUP       7   /* the 2nd or 3rd copy of one burst            */

/* CRC16-CCITT (FALSE): polynomial 0x1021, init 0xFFFF, no reflection, no
 * final xor. Named precisely because "CRC16-CCITT" is four different
 * algorithms depending on who is speaking, and two devices that disagree
 * about which one would simply never hear each other. */
uint16_t lora_crc16(const uint8_t *p, int n);

/* Writes exactly LORA_PKT_BYTES. Zeroes the reserved bytes itself, so a
 * caller cannot leak stack into the air. */
void lora_pkt_build(uint8_t *dst, const lora_pkt_t *p);

/* Returns LORA_RX_*. `out` is only written on LORA_RX_OK. */
int lora_pkt_parse(const uint8_t *src, int n, lora_pkt_t *out);


/* ---- the dedupe ring ---------------------------------------------------
 * A transmitter sends the same alert three times, so a receiver hears one
 * event as up to three packets. Without this, an operator gets three buzzes
 * for one drone and the event ring gets three entries.
 *
 * Sixteen keys with a sixty-second expiry: sixteen is more peers than this
 * deployment will ever have times the burst depth, and sixty seconds is
 * comfortably longer than a burst and comfortably shorter than the time it
 * takes a device's one-byte seq to wrap round to a value it used before.
 */
#define LORA_DEDUP_N   16
#define LORA_DEDUP_MS  60000u

typedef struct {
    uint16_t id[LORA_DEDUP_N];
    uint32_t t_ms[LORA_DEDUP_N];
    uint8_t  seq[LORA_DEDUP_N];
    /* The epoch is part of the key. `seq` restarts at 0 on every boot, so a
     * sender that resets between alerts sends seq 0 both times: two alerts
     * inside the expiry window would then be indistinguishable and the second
     * dropped as a repeat, silently. With the boot counter in the key, alerts
     * from either side of a reset are different frames. */
    uint8_t  epoch[LORA_DEDUP_N];
    uint8_t  used[LORA_DEDUP_N];
    uint8_t  next;
} lora_dedup_t;

void lora_dedup_reset(lora_dedup_t *d);

/* True if this key is new - and records it. False if it is a repeat still
 * inside the window. An entry older than LORA_DEDUP_MS is forgotten, so a
 * peer that alerts again an hour later with the same seq is heard. */
bool lora_dedup_admit(lora_dedup_t *d, uint16_t id, uint8_t seq,
                      uint8_t epoch, uint32_t now_ms);

/* The whole receive decision in one place, with no SPI in it:
 * parse -> is it ours? -> have we already heard this burst? Returns
 * LORA_RX_OK when the frame should raise a remote alert, and one of the
 * LORA_RX_* reasons when it should not. `out` is written on LORA_RX_OK only.
 *
 * It lives here rather than inside the receive interrupt so that the two
 * rules that matter most are reachable by a host test: that a device never
 * remote-triggers off its own repeats, and that a three-frame burst raises
 * exactly one alert.
 *
 * Order is load-bearing and is asserted by those tests: the SELF drop happens
 * before the dedupe ring is touched. A device that entered its own id into
 * its own ring would then reject a genuine peer that happened to share a
 * sequence number, silently. */
int lora_rx_admit(lora_dedup_t *d, const uint8_t *src, int n,
                  uint16_t self_id, uint32_t now_ms, lora_pkt_t *out);

/* ---- the transmit burst ------------------------------------------------
 * Three transmissions, LORA_TX_GAP_MS +- LORA_TX_JITTER_MS apart, on a local
 * alert onset and never on a repeat or a re-confirmation.
 *
 * Duty cycle, computed rather than assumed:
 *
 *   SF7:  51.5 ms a frame,  154 ms a burst -> 1.54 % at one alert per 10 s
 *   SF9: 185.3 ms a frame,  556 ms a burst -> 5.56 % at one alert per 10 s
 *
 * The 868.1 MHz g1 sub-band allows 1 %, so continuous alerting is over the
 * limit at any spreading factor. What holds: the burst stays inside 1 % as
 * long as alerts are no more frequent than about one a minute (0.19 % at one
 * per five minutes), which is the field case. A bench soak at one alert per
 * 30 s is 1.85 % and is a bench activity, not a deployment.
 *
 * The jitter is not decoration. Two devices that hear the same drone alert
 * within milliseconds of each other, and a fixed gap would make their three
 * repeats collide three times instead of once.
 */
#define LORA_TX_REPEATS    3
#define LORA_TX_GAP_MS     350u
#define LORA_TX_JITTER_MS  150u

/* A link test gets more chances than an alert. An alert is time-critical and
 * must not hog the air, so it keeps three repeats. A link test is neither -
 * it is a person standing still, pressing a button, waiting to hear a beep -
 * so it gets six, which costs the operator a couple of seconds and doubles
 * the number of chances the far unit has to hear one.
 *
 * This is not a fix for a weak link and must not be read as one. If a pair
 * only works one way, the test page's RX/SNR line on each unit is the
 * measurement that says why, and both antennas want to be vertical and
 * parallel before anything else is believed. */
#define LORA_LINK_TX_REPEATS  6

/* Maps any random word onto [GAP - JITTER, GAP + JITTER]. Pure, so the
 * bounds are a test rather than a hope. */
uint32_t lora_tx_gap(uint32_t rnd);

/* ---- identity ----------------------------------------------------------
 * device_id = CRC16-CCITT of the six-byte factory MAC. Every unit gets a
 * stable, unique-enough 16-bit id out of silicon with zero configuration and
 * one firmware image for all boards. Collisions across a handful of units are
 * negligible, and the id is display-and-dedupe only, so a collision costs a
 * confusing label and breaks nothing.
 */
uint16_t lora_id_from_mac(const uint8_t mac[6]);
