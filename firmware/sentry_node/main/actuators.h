/*
 * actuators.h - buzzer and vibration motor, plus the one safe-off contract.
 *
 * Both loads sit behind NPN low-side drivers whose bases hang off GPIO through
 * 1 kOhm. A high-Z pin therefore leaves the base floating, which is why
 * app_main drives BOTH pins LOW immediately at boot - the ONLY boot-time
 * addition this build is permitted to make (see the overnight brief, rule 3).
 *
 * POLARITY, everywhere in this project: HIGH = ON.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Drive GPIO17/GPIO18 as outputs, LOW. Safe to call before anything else and
 * safe to call twice. This is the boot-time call. */
void actuators_boot_safe(void);

/* ---- BUZZER DRIVE, and why it is a runtime choice ------------------------
 * GPIO17 is a plain on/off pin into an NPN base. What "maximum volume" means
 * depends on which transducer is fitted, and that is a bench observation, not
 * a compile-time fact:
 *
 *   ACTIVE buzzer  (has its own oscillator) - solid DC is already maximum.
 *                  Gating it with PWM can only make it quieter or change its
 *                  timbre. This is the default.
 *   PASSIVE buzzer (a bare transducer)      - DC makes one click and then
 *                  silence. It needs a square wave, and it is loudest within
 *                  a few hundred Hz of its mechanical resonance, typically
 *                  2-4 kHz.
 *
 * Both drives are implemented and switchable at runtime (`U bd0` / `U bd1
 * <hz>`), and `U bt` plays the ladder so the ear decides. The choice is
 * persisted, so the field build comes up loud without a laptop.
 *
 * Either way the ceiling is the hardware: this pin can do nothing about
 * supply voltage or the transducer's own sensitivity. If it is not loud
 * enough at maximum drive, the fix is a louder buzzer or a higher rail, not
 * firmware. */
#define BUZZ_DRIVE_DC   0u
#define BUZZ_DRIVE_PWM  1u
#define BUZZ_PWM_HZ_DEFAULT 2700u   /* mid-band for a 2-4 kHz resonance */

void actuators_buzzer_drive(uint8_t mode, uint32_t hz);
uint8_t  actuators_buzzer_drive_mode(void);
uint32_t actuators_buzzer_drive_hz(void);

void actuators_buzzer(bool on);

/* ---- an explicit-duty tone, for press feedback and for a duty sweep ------
 *
 * BTN_BEEP_MODE 1 and `U b <duty_pct> <ms>`. The TMB12A03 is an active buzzer
 * with its own oscillator, so what a chopped supply does to it is not
 * predictable from a datasheet: at some duties it will run quieter, at others
 * its oscillator may not start at all. Only ears can decide, which is why
 * mode 0 ships and this exists to be swept in a morning.
 *
 * hz is the LEDC carrier, duty_pct 0..100. `on=false` silences without
 * tearing down the timer, so a sweep does not re-init the pad each step. */
bool actuators_buzzer_tone(uint32_t hz, int duty_pct, bool on);
void actuators_motor(bool on);

bool actuators_buzzer_state(void);
bool actuators_motor_state(void);

/* Buzzer off, motor off, LED off. Callable before any lazy init has happened
 * without faulting - it must never be the reason a fatal path cannot clean up.
 * Called on entry to and exit from every mode that drives outputs. */
void actuators_safe_all_off(void);

/* THE SNOOZE MOTOR: continuous and weak, on LEDC at 25 kHz with a kick-start.
 * 25 kHz is above the 8 kHz useful band, so the PWM cannot inject a tone into
 * the detector's own passband from a transducer on the same board. Restores
 * PIN_MOTOR to a plain GPIO driven LOW when it stops, so the alert path is
 * unaffected. See the definition for why the kick is not optional. */
/* The alert's motor, driven through PWM so it can be ramped rather than
 * stepped. With the loads sequenced, every brownout measured in a twenty-alert
 * soak was stamped OUTPUTS, and the buzzer alone was already measured safe,
 * which leaves the ERM inrush. pct 0 releases the pin back to a plain GPIO
 * driven low. */
void actuators_motor_duty(int pct);

void actuators_motor_snooze(bool on);
