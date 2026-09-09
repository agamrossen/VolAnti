/*
 * lora_link.h - the radio, the task around it, and the remote alert.
 *
 * PCB only. Every function compiles to an inert stub when BOARD_HAS_LORA is 0
 * and the DevKit build calls them anyway, so the guard loop reads as one
 * behaviour instead of being wrapped in #if at four places and the linker
 * deletes what the breadboard cannot use. The protocol itself lives in
 * lora_proto.c, which is board-independent and host-tested.
 *
 * The hardware is an Ai-Thinker Ra-01H, an SX1276 module, on SPI2 sharing SCK
 * and MOSI with the e-paper and owning MISO alone because the panel is
 * write-only. Its own CS (idle high), RST and DIO0.
 *
 *   * PA_BOOST only. The Ra-01H bonds PA_BOOST and leaves RFO unconnected, so
 *     the PA select bit is not a preference on this module - it is the only
 *     setting that transmits at all.
 *   * The antenna is the band-specific piece. The module covers 803-930 MHz;
 *     868.100 MHz is the UK test channel, and the deployment frequency is a
 *     configuration set per local regulations with `U c lorahz` rather than a
 *     code change.
 *
 * Three hazards, and what closes each:
 *
 *  1. Self-interference. A remote alert drives this device's own buzzer and
 *     motor, which Tier-3 and Tier-4 would then hear. The output freeze keys
 *     on alert_ui_outputs_active() - is anything making noise, regardless of
 *     why - so a remote alert freezes them exactly as a local one does. The
 *     host tests assert that rather than trusting it.
 *  2. The shared bus. Two spi_master devices on one host, and the driver
 *     serialises transactions. The panel's long BUSY waits hold no bus (it
 *     polls a GPIO, not SPI), and LoRa transactions are sub-millisecond.
 *  3. The frame loop. Everything here runs on a task pinned to the non-audio
 *     core at low priority. The guard loop's only contact with the radio is
 *     two function calls per frame that read or set a word.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_pins.h"

/* How long a remote alert holds the outputs, which is longer than the local
 * burst: a remote alert means a peer somewhere else heard something, so there
 * is no source in front of the operator to look at and the indication has to
 * survive them walking to the box. */
#define LORA_REMOTE_LATCH_MS 10000u

/* Bring the radio up and start the link task. Lazy, idempotent, never fatal:
 * a device whose radio will not answer must still guard, and must SAY the
 * radio did not answer rather than pretending it did. */
bool lora_link_begin(void);
void lora_link_end(void);

/* True only when the radio answered its own version register and the link
 * task is running. The guard loop gates every radio line on this, so a board
 * whose module is missing or dead behaves exactly like a board with no radio
 * rather than like a board with a radio that never transmits. */
bool lora_link_ready(void);

/* The only thing the detector ever calls: announce a local alert onset, once
 * per onset, never per frame and never on a re-confirmation. Consumes one
 * sequence number and queues the three-transmission burst. Returns
 * immediately; the task does the air time.
 *
 * `score` is the score at the decision and rides in what were reserved bytes,
 * so proto_ver does not move. Pass 0 when there is no meaningful score; 0 is
 * what an older sender transmits and is displayed as "not reported" rather
 * than as a real zero. */
void lora_link_announce(uint8_t tier, float score);

/* True while a remote alert is latched. The guard loop ORs this into the same
 * alert path every tier uses, so there is one buzzer and one dismissal - an
 * operator being alerted does not care where it came from, and afterwards
 * cares about little else, which is why the panel and the ring get the
 * originating id. */
bool     lora_link_remote_active(uint32_t now_ms);
uint16_t lora_link_remote_id(void);
uint8_t  lora_link_remote_tier(void);

/* The operator silenced the local outputs. Snooze is local only: it neither
 * transmits nor suppresses future receives, because a device that went deaf to
 * its peers because somebody quieted a buzzer would be the worst kind of
 * surprise. */
void lora_link_snooze(void);

/* `Lt` - the self-test. Reads SX1276 register 0x42 (version, expect 0x12)
 * and reports what came back. */
bool lora_link_selftest(char *dst, int cap);

/* `Ls <0|1>` - send one test-flagged packet, for the two-board air test. A
 * separate entry point from lora_link_announce() so that a test packet can
 * never consume the alert sequence number, and a receiver can always tell a
 * drill from a drone. */
bool lora_link_send_test(bool on);

/* Runtime configuration, from the persisted settings. 0 leaves a value
 * alone. Applied at lora_link_begin(). */
void lora_link_configure(bool enabled, uint32_t freq_hz);

/* One line for `I` and the standalone banner. */
void lora_link_describe(char *dst, int cap);

/* This device's id, N-XXXX in hex on the panel and in the ring. */
uint16_t lora_link_device_id(void);

/* ---- link quality of the last frame -------------------------------------
 * False until one frame has passed CRC since power-on, which is a different
 * statement from "the last frame was weak" and is why it is not reported as a
 * 0 dBm reading. Captured for every CRC-good frame, including duplicates and
 * our own id, so a range walk can tell "arrived and discarded" from "did not
 * arrive". On a board with no radio this is always false. */
bool lora_link_last_rx_quality(int *rssi_dbm, int *snr_db);

/* Frames transmitted since power-on. Counts transmissions, not alerts: one
 * alert is a burst of three. */
uint32_t lora_link_tx_count(void);

/* The last accepted peer alert's score, x100. 0 means the peer did not report
 * one - which is what an older image sends, and is shown as no number rather
 * than as a score of zero. */
uint16_t lora_link_remote_score_x100(void);

/* The peer's v1 threshold in milli-units, 0 if it did not report one. A
 * paired field configuration can differ by exactly this, so a remote alert
 * would otherwise be unreadable on the other board. */
uint16_t lora_link_remote_thr1_milli(void);

/* ---- the link test ------------------------------------------------------
 * Confirms, standing in a field, that two units can hear each other. A
 * one-way transmission cannot show that, because the sender learns nothing,
 * so a request is answered by a reply and the sender displays the round trip.
 *
 * A reply is never answered, which is what keeps this from becoming the relay
 * the protocol forbids - see lora_proto.h. Neither packet raises an alert. */
#define LORA_LINK_NONE     0u
#define LORA_LINK_SENT     1u   /* this unit sent a request                 */
#define LORA_LINK_GOT_REQ  2u   /* a peer asked; a reply has been queued    */
#define LORA_LINK_GOT_ACK  3u   /* the round trip closed - the link works   */

bool lora_link_send_linktest(void);

/* One-shot: returns the pending event and clears it. `peer` is the other
 * unit's id, 0 for LORA_LINK_SENT. */
uint8_t lora_link_take_link_event(uint16_t *peer);

/* The loopback drill (`Ll`): injects one frame into the real receive path.
 *
 * An SX1276 is half duplex and cannot hear its own transmission, so `Ls` alone
 * can never exercise the receive half on a single board - which would leave
 * the self drop, the dedupe window and the whole remote-alert path untested on
 * a one-board bench.
 *
 *   as_peer = false   the frame carries our id and must be dropped as self
 *   as_peer = true    the frame carries a peer's and must raise a remote
 *                     alert; injected again it must be dropped as a duplicate
 *
 * Injected frames are counted apart from the air (rx_loop, not rx_ok), so a
 * bench drill can never be mistaken for a two-board air test. Returns false if
 * the radio was never started. Inert on a board with no radio. */
bool lora_link_loopback(bool as_peer, uint32_t now_ms);
