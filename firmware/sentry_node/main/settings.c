#include "settings.h"

#include <stdio.h>
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "actuators.h"
#include "alert_ui.h"
#include "board_pins.h"
#include "mic_cal.h"
#include "epaper.h"

#define NVS_NAMESPACE "sentry"
#define NVS_KEY       "cfg"

static settings_t s_cfg;
static bool       s_loaded;
static bool       s_nvs_ok;

/* The shipped configuration, in one place. Every zero here is meaningful:
 * a zero threshold means "use the compiled deployment default", so a field
 * device that loses its settings comes up at the documented operating point
 * rather than at 0.000, which would fire on silence. */
static void defaults(settings_t *c)
{
    memset(c, 0, sizeof(*c));
    c->version = SETTINGS_VERSION;
    /* 1.500, not 0. The compiled DEFAULT_THRESHOLD stays 1.700 because a
     * golden vector is replayed at it and the parity gate must keep
     * describing the sealed detector; the field default is a settings value.
     * The closest real rotor recording this project owns fires at 1.500 and
     * is silent at 1.600 and 1.700 - the comb score is not monotone in range,
     * because a near source's own broadband lifts the whitening floor and
     * flattens the very contrast the score measures. */
    c->thr1_milli = 1500;              /* 1.500, the field default          */
    c->thr2_milli = 0;                 /* -> T2_TAU2                        */
    c->thr3_milli = 0;                 /* -> T3_TAU3                        */
    c->thr4_milli = 0;                 /* -> T4_TAU4                        */
    c->t2_enabled = 1;
    c->t3_enabled = 1;                 /* the field build's choice - see the
                                        * standalone header in sentry_node.c */
    /* All four tiers on, against two standing reasons that are recorded here
     * rather than deleted:
     *
     *   (1) NOT CERTIFIED. 487 s of drone-free audio bounds Tier-4's
     *       false-alarm rate at 22/h against a 0.40/h allowance. Certifying
     *       needs about 7.5 h and the project owns eleven minutes.
     *   (2) It did not originally fit: 21.5 ms per update measured against a
     *       predicted "under 1.5", and armed, the guard ran p99 43.1 ms with
     *       234 frames over the 32 ms hop.
     *
     * (2) is what the frame-budget slicing addresses - the prominence walk is
     * resumable and proven bit-identical, and the frame loop spends it a few
     * bins at a time instead of whole. (1) is unchanged and is an operator
     * disclosure: Tier-4's alerts are attributed T4 and are data, not
     * detections. */
    c->t4_enabled = 1;
    c->t2_rate = 0;                    /* the compiled default (half rate)  */
    c->cx = 'a';
    /* PWM rather than DC, and this is a hypothesis rather than a settled
     * measurement. `U bt`'s ladder distinguishes the two transducer types: an
     * active buzzer is loudest on the DC step, a passive one near-silent on
     * it. A faint DC beep and a continuous tone under PWM both fit a passive
     * transducer, but the four rungs have not been ranked by ear. PWM is the
     * safer default for an unknown part - a passive buzzer is silent on DC
     * and loud on PWM near resonance, while an active one still sounds when
     * chopped at 2.7 kHz. `U bd0` restores DC; `U bd1 <hz>` picks another
     * frequency. */
    c->buzz_drive = BUZZ_DRIVE_PWM;
    c->buzz_hz = BUZZ_PWM_HZ_DEFAULT;
    /* The orientation the panel reads upright in the enclosure.
     *
     * A shipped default matters because a blank board has no stored blob to
     * read: it comes up on this number or it comes up sideways, and a field
     * device whose first act is to need a laptop is the thing the shipped
     * defaults exist to avoid. A board that already has settings keeps its
     * own rotation through the migration, so those need `U c rot` typed once
     * if they disagree. */
    c->epd_rotation = EPD_ROT_90;
    c->epd_mirror = 0;
    c->snooze_ms = SNOOZE_MS;
    c->alert_max_ms = ALERT_MAX_MS;
    c->autostart = 1;
    /* The radio ships on where there is one. Every device alerts every other
     * device, and a peer beacon that had to be switched on at each deployment
     * would be a peer beacon that is off on the one night it mattered. On a
     * board with no radio it compiles to a stub and this is 0, so the flag is
     * inert rather than merely unused. */
    c->lora_enabled = BOARD_HAS_LORA ? 1 : 0;
    c->lora_hz = 0;                    /* -> BOARD_LORA_HZ_DEFAULT          */
    /* The family rule, on. Measured non-inferior on the sealed corpus - no
     * regressions, two gains in 486 paired positives, McNemar p = 0.50 - and
     * the pairing that would overturn it is done offline by replaying every
     * capture with the rule on and off, so shipping it on costs nothing that
     * cannot be undone from the same recordings. */
    c->trk_family = 1;
    /* The voice and struck-note veto, on. Recalibrated and paired over 486
     * corpus positives it is 0 lost / 92 gained, McNemar p < 0.0001; it
     * removes a held vowel entirely and cuts a modelled piano by 47% and
     * livestock by 50%. A strict improvement rather than a trade, because the
     * false alarms it removes buy threshold headroom back. docs/VETO_CAL.md
     * has the measurement, including the three mechanisms rejected before
     * this one. The golden replay clears it regardless, so the parity gate is
     * unaffected. `U c veto 0` restores the sealed tracker. */
    c->veto_voice = 1;
    /* Test mode is off in the shipped defaults. A board that has never been
     * configured is a board on its way to a deployment, not a bench. */
    c->test_mode  = 0;
    /* The near-field gate, on by default: it is the difference between a
     * device that detects a real rotor and one that detects a piano. */
    c->nf_gate    = 1;
    /* No calibration until a human measures one. Unity is stored so the
     * numbers read sensibly in `U c`, but the enable is what decides whether
     * they are applied, and it ships off. */
    c->mic_cal_enabled = 0;
    c->batt_absent = 0;             /* a cell is assumed fitted */
    for (int i = 0; i < 4; i++) {
        c->mic_gain_milli[i] = MIC_CAL_UNITY;
    }
}

static void nvs_once(void)
{
    if (s_nvs_ok) {
        return;
    }
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* A partition that cannot be read is erased rather than nursed: the
         * only thing in it is this struct, and every field has a default. */
        if (nvs_flash_erase() == ESP_OK) {
            e = nvs_flash_init();
        }
    }
    s_nvs_ok = (e == ESP_OK);
}

const settings_t *settings_get(void)
{
    if (s_loaded) {
        return &s_cfg;
    }
    defaults(&s_cfg);
    s_loaded = true;

    nvs_once();
    if (!s_nvs_ok) {
        return &s_cfg;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return &s_cfg;                 /* never written yet - defaults stand */
    }
    settings_t tmp;
    size_t n = sizeof(tmp);
    esp_err_t e = nvs_get_blob(h, NVS_KEY, &tmp, &n);
    nvs_close(h);
    /* Size AND version must both match. A blob from an older build is
     * discarded, not field-by-field migrated: an unknown configuration in the
     * field is worse than the documented one. */
    if (e == ESP_OK && n == sizeof(tmp) && tmp.version == SETTINGS_VERSION) {
        s_cfg = tmp;
    } else if (e == ESP_OK && n > 0 && n <= sizeof(tmp) &&
               (tmp.version == 5u || tmp.version == 6u ||
                tmp.version == 7u || tmp.version == 8u ||
                /* Leaving a deployed version out of this list sends those
                 * boards down the discard path and costs them their rotation
                 * and their measured microphone calibration - the exact loss
                 * the migration exists to prevent. */
                tmp.version == 9u ||
                /* The previous version is what every deployed unit stores,
                 * and leaving it out of this list is not a hypothetical: a
                 * bump made without it brings boards up with cfg identical to
                 * ship, having thrown away the rotation an operator set.
                 *
                 * The lesson is the procedure rather than the constant: a
                 * version bump has two halves, the new number and the old
                 * number added here, and doing only the first silently resets
                 * every deployed unit to defaults. The host tests pin the
                 * previous version's presence in this list. */
                tmp.version == 10u)) {
        /* ---- v5 -> v6, AND THIS ONE MIGRATES RATHER THAN DISCARDS --------
         *
         * Every previous bump threw the stored blob away, and the comment
         * above still defends that: an unknown configuration in a field is
         * worse than the documented one. It was the right rule while a bump
         * meant bytes had CHANGED MEANING - v5's did, which is why a v4 blob
         * could not be read as gains.
         *
         * `n <= sizeof(tmp)`, NOT `n <`. The first version of this required
         * the blob to be SMALLER, on the assumption that appending a field
         * grows the struct. It does not always: veto_voice went into padding
         * the compiler was already leaving after batt_absent, so sizeof did
         * not move, a stored v5 blob matched neither branch, and it was
         * silently discarded - a board comes up on the shipped defaults and
         * loses its stored rotation, and on a board carrying a measured
         * microphone calibration it would throw that away too.
         *
         * v6 only APPENDS. Every v5 byte means in v6 exactly what it meant in
         * v5, so the stored blob is a valid prefix and copying it forward is
         * not a guess. That matters because discarding it would silently
         * throw away the operator's rotation and their measured microphone
         * calibration, and the first anyone would know is a sideways panel on
         * a hillside.
         *
         * What is not preserved, deliberately: trk_family. It shipped 0 in
         * every device that exists, so a stored 0 is the old default rather
         * than an operator's decision, and it is forced on below. The `I`
         * banner prints the live value so this is visible rather than
         * assumed. */
        memcpy(&s_cfg, &tmp, n);
        s_cfg.version = SETTINGS_VERSION;
        /* v5 has no veto_voice byte at all. It is forced below with the rest
         * of the set rather than zeroed here, because zeroing it would make a
         * migrated board and a fresh board run DIFFERENT DETECTORS off the
         * same firmware, decided by invisible stored state. That is the bug
         * the tier-forcing block below was written to end. */

        /* ---- v7 forces the tier set ------------------------------------
         *
         * A stored 0 in t2, t3, t4 or trk_family is the old default rather
         * than an operator's decision: every one of them shipped 0 at some
         * point, so a migration that faithfully carries the old value across
         * brings a board up reading "Tier-4 off" on a firmware that ships it
         * on.
         *
         * The console commands still work and still persist, so a bench can
         * turn a tier off; what cannot happen any more is a tier being off
         * because of what a previous FIRMWARE shipped. The boot line and INFO
         * both print the detector's live mask, so the answer to "is it on" is
         * never inferred from a stored byte. */
        s_cfg.t2_enabled = 1;
        s_cfg.t3_enabled = 1;
        s_cfg.t4_enabled = 1;
        s_cfg.trk_family = 1;
        /* veto_voice joins them on the same grounds and for the same reason.
         * It shipped 0 in every device that has ever existed, and in v5 it did
         * not exist at all, so a stored 0 cannot be an operator's decision. It
         * was 0 while its controls were uncalibrated; they are calibrated now
         * and it is the shipped default. `U c veto 0` still
         * turns it off and still persists, so a bench can have the sealed
         * tracker; what cannot happen is a board quietly firing on a piano
         * because of what an EARLIER FIRMWARE shipped. */
        s_cfg.veto_voice = 1;

        /* This one overrides an operator value. A stored thr1 of 1700 raised
         * nothing against a real rig at 3-5 m at any throttle, and it cannot
         * be distinguished from the default that produced that. nf_gate has
         * never existed in a stored blob, so a 0 there is padding. */
        s_cfg.nf_gate = 1;
        s_cfg.thr1_milli = 1500;

        /* ---- v11: the seven-second alarm, and why it is forced ----------
         *
         * alert_max_ms is a stored setting whose compiled default moved from
         * five seconds to seven. Every unit already in existence holds a v10
         * blob carrying the old value, and a blob that matches
         * SETTINGS_VERSION is taken by the exact-version branch above and
         * never reaches this force - so without the bump every one of them
         * would come up on the new firmware still alarming for five seconds,
         * and the only symptom would be that nothing changed. That is the
         * same trap v8 was bumped for, three defaults ago.
         *
         * It overrides an operator value deliberately, for the same reason
         * thr1 is forced: a stored 5000 cannot be told apart from the default
         * that produced it, and the longer alarm is wanted on every unit
         * rather than on the ones nobody happened to type into.
         *
         * Migrates, does not discard: a v10 blob is a valid prefix of v11, so
         * rotation and measured microphone calibration survive. */
        s_cfg.alert_max_ms = ALERT_MAX_MS;

        /* ---- WHY v8 EXISTS, AND IT IS NOT A LAYOUT CHANGE ---------------
         * No byte moved between v7 and v8. veto_voice is in both, in the same
         * place, meaning the same thing. The bump is here for exactly the
         * reason v7's was: a stored blob that matches SETTINGS_VERSION is
         * taken by the exact-version branch above and never reaches this
         * force. Boards flashed with v7 carry veto_voice = 0, so without the
         * bump they come up on the new firmware still firing on a piano, and
         * the only symptom is that nothing changed.
         *
         * That is affordable only because this branch MIGRATES rather than
         * discards: a v7 blob is a valid prefix of v8, so the operator keeps
         * their rotation and their measured microphone calibration. A bump
         * that discarded them would not be worth it for a default. */

        /* ---- v9 APPENDS test_mode, AND ZEROES IT BY HAND ----------------
         * Not a force: test mode is a bench decision and the firmware has no
         * business asserting one. It is written explicitly rather than left
         * to the memcpy because an appended uint8_t can land in padding the
         * compiler was ALREADY leaving - that is exactly what veto_voice did,
         * which is why `n <= sizeof(tmp)` is the test above. A blob whose
         * padding happened to hold a 1 would bring a unit up in test mode,
         * and the only symptom would be a small tag in the corner of a screen
         * nobody was looking at. */
        s_cfg.test_mode = 0;
    }
    return &s_cfg;
}

settings_t *settings_mut(void)
{
    (void)settings_get();
    return &s_cfg;
}

bool settings_save(void)
{
    (void)settings_get();
    nvs_once();
    if (!s_nvs_ok) {
        return false;
    }
    s_cfg.version = SETTINGS_VERSION;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    bool ok = (nvs_set_blob(h, NVS_KEY, &s_cfg, sizeof(s_cfg)) == ESP_OK) &&
              (nvs_commit(h) == ESP_OK);
    nvs_close(h);
    return ok;
}

bool settings_reset(void)
{
    defaults(&s_cfg);
    s_loaded = true;
    return settings_save();
}

/* ONE FORMATTER, TWO CALLERS. The running configuration and the shipped
 * defaults are printed by the same code so that they can be compared line for
 * line by eye. A second format would let the two drift into looking different
 * when they are the same, which is the failure this exists to prevent. */
static void describe_one(const char *tag, const settings_t *c,
                         char *dst, int cap)
{
    snprintf(dst, cap,
             "%s v%u thr1=%u thr2=%u thr3=%u thr4=%u t2=%u t3=%u t4=%u "
             "rate=%u cx=%c "
             "buzz=%s@%uHz rot=%udeg%s snooze=%ums burst=%ums autostart=%u "
             "lora=%u@%uHz trk_family=%u veto=%u nf=%u test=%u "
             "cal=%u[%d,%d,%d,%d] batt=%s\n",
             tag, (unsigned)c->version, (unsigned)c->thr1_milli,
             (unsigned)c->thr2_milli, (unsigned)c->thr3_milli,
             (unsigned)c->thr4_milli,
             (unsigned)c->t2_enabled, (unsigned)c->t3_enabled,
             (unsigned)c->t4_enabled,
             (unsigned)c->t2_rate, c->cx ? c->cx : 'a',
             c->buzz_drive == BUZZ_DRIVE_PWM ? "pwm" : "dc",
             (unsigned)c->buzz_hz,
             (unsigned)epaper_degrees_from_quadrant(c->epd_rotation),
             c->epd_mirror ? " mirrored" : "",
             (unsigned)c->snooze_ms, (unsigned)c->alert_max_ms,
             (unsigned)c->autostart,
             (unsigned)c->lora_enabled,
             (unsigned)(c->lora_hz ? c->lora_hz : (uint32_t)BOARD_LORA_HZ_DEFAULT),
             (unsigned)c->trk_family,
             (unsigned)c->veto_voice, (unsigned)c->nf_gate, (unsigned)c->test_mode,
             (unsigned)c->mic_cal_enabled,
             (int)c->mic_gain_milli[0], (int)c->mic_gain_milli[1],
             (int)c->mic_gain_milli[2], (int)c->mic_gain_milli[3],
             c->batt_absent ? "ABSENT" : "fitted");
}

void settings_describe(char *dst, int cap)
{
    describe_one("cfg", settings_get(), dst, cap);
}

/* THE SHIPPED FIELD CONFIGURATION - what a BLANK board comes up as.
 *
 * Built from a fresh defaults() struct on the stack, never from the cached
 * settings, so it answers "what will an erased board do" rather than "what is
 * this board doing". Those are different questions and only the first one can
 * be checked before a device leaves the bench.
 *
 * There is exactly one source of truth for it and it is defaults() itself.
 * This function cannot drift from the shipped values because it IS them; a
 * hand-maintained table beside them could, and that is the whole reason this
 * is code and not a comment. */
void settings_describe_defaults(char *dst, int cap)
{
    settings_t d;
    defaults(&d);
    describe_one("ship", &d, dst, cap);
}
