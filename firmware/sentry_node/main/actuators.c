#include "actuators.h"
#include "ui_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#include "board_pins.h"
#include "led_ws2812.h"

static bool s_booted;
static bool s_buzzer;
static bool s_motor;

/* ---- buzzer drive -------------------------------------------------------
 * DC by default, which is exactly the behaviour every previous capture was
 * taken with. The LEDC timer/channel is created on FIRST PWM USE and never at
 * boot, so the boot path is unchanged. */
#define BUZZ_LEDC_MODE  LEDC_LOW_SPEED_MODE
#define BUZZ_LEDC_TIMER LEDC_TIMER_0
#define BUZZ_LEDC_CHAN  LEDC_CHANNEL_0
#define BUZZ_LEDC_BITS  LEDC_TIMER_10_BIT
#define BUZZ_LEDC_HALF  512     /* 50% of 2^10 - the loudest square wave */

static uint8_t  s_drive = BUZZ_DRIVE_DC;
static uint32_t s_drive_hz = BUZZ_PWM_HZ_DEFAULT;
static bool     s_ledc_ready;

/* Hand the pad back to the GPIO peripheral and park it LOW. Detaching the
 * LEDC output signal matters: ledc_stop() alone leaves the pad wired to the
 * LEDC matrix, so a later gpio_set_level() would be ignored and the buzzer
 * could not be turned off by the DC path. */
static void buzzer_pad_to_gpio(void)
{
    esp_rom_gpio_connect_out_signal(PIN_BUZZER, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_level(PIN_BUZZER, 0);
}

static bool buzzer_ledc_init(uint32_t hz)
{
    ledc_timer_config_t t = {
        .speed_mode = BUZZ_LEDC_MODE,
        .duty_resolution = BUZZ_LEDC_BITS,
        .timer_num = BUZZ_LEDC_TIMER,
        .freq_hz = hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) {
        return false;
    }
    if (!s_ledc_ready) {
        ledc_channel_config_t ch = {
            .gpio_num = PIN_BUZZER,
            .speed_mode = BUZZ_LEDC_MODE,
            .channel = BUZZ_LEDC_CHAN,
            .timer_sel = BUZZ_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
        };
        if (ledc_channel_config(&ch) != ESP_OK) {
            return false;
        }
        s_ledc_ready = true;
    }
    return true;
}

void actuators_boot_safe(void)
{
    /* Level BEFORE direction: setting the level first means the pad never
     * spends even one instruction as a driven HIGH, which on an NPN base is
     * the difference between silence and a chirp at every reset. */
    gpio_set_level(PIN_BUZZER, 0);
    gpio_set_level(PIN_MOTOR, 0);

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_BUZZER) | (1ULL << PIN_MOTOR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_BUZZER, 0);
    gpio_set_level(PIN_MOTOR, 0);

    s_buzzer = false;
    s_motor = false;
    s_booted = true;
}

void actuators_buzzer_drive(uint8_t mode, uint32_t hz)
{
    if (!s_booted) {
        actuators_boot_safe();
    }
    const bool was_on = s_buzzer;
    actuators_buzzer(false);            /* silence before re-wiring the pad */

    if (mode == BUZZ_DRIVE_PWM) {
        if (hz < 100u)   { hz = 100u; }
        if (hz > 10000u) { hz = 10000u; }
        if (buzzer_ledc_init(hz)) {
            s_drive = BUZZ_DRIVE_PWM;
            s_drive_hz = hz;
        } else {
            s_drive = BUZZ_DRIVE_DC;    /* refuse silently-broken PWM */
            buzzer_pad_to_gpio();
        }
    } else {
        s_drive = BUZZ_DRIVE_DC;
        if (s_ledc_ready) {
            ledc_stop(BUZZ_LEDC_MODE, BUZZ_LEDC_CHAN, 0);
        }
        buzzer_pad_to_gpio();
    }
    if (was_on) {
        actuators_buzzer(true);
    }
}

bool actuators_buzzer_tone(uint32_t hz, int duty_pct, bool on)
{
    if (!s_booted) {
        actuators_boot_safe();
    }
    if (!on) {
        if (s_ledc_ready) {
            (void)ledc_set_duty(BUZZ_LEDC_MODE, BUZZ_LEDC_CHAN, 0);
            (void)ledc_update_duty(BUZZ_LEDC_MODE, BUZZ_LEDC_CHAN);
        }
        s_buzzer = false;
        return true;
    }
    if (hz < 100u)   { hz = 100u; }
    if (hz > 40000u) { hz = 40000u; }
    if (duty_pct < 0)   { duty_pct = 0; }
    if (duty_pct > 100) { duty_pct = 100; }
    if (!buzzer_ledc_init(hz)) {
        return false;
    }
    /* 10-bit resolution: 1024 counts full scale. */
    const uint32_t duty = (uint32_t)((duty_pct * 1024) / 100);
    if (ledc_set_duty(BUZZ_LEDC_MODE, BUZZ_LEDC_CHAN, duty) != ESP_OK) {
        return false;
    }
    if (ledc_update_duty(BUZZ_LEDC_MODE, BUZZ_LEDC_CHAN) != ESP_OK) {
        return false;
    }
    s_drive = BUZZ_DRIVE_PWM;
    s_drive_hz = hz;
    s_buzzer = (duty_pct > 0);
    return true;
}

uint8_t  actuators_buzzer_drive_mode(void) { return s_drive; }
uint32_t actuators_buzzer_drive_hz(void)   { return s_drive_hz; }

void actuators_buzzer(bool on)
{
    if (!s_booted) {
        actuators_boot_safe();
    }
    s_buzzer = on;
    if (s_drive == BUZZ_DRIVE_PWM && s_ledc_ready) {
        /* 50% duty is the maximum-energy square wave; anything else is a
         * narrower pulse and therefore quieter. */
        ledc_set_duty(BUZZ_LEDC_MODE, BUZZ_LEDC_CHAN, on ? BUZZ_LEDC_HALF : 0);
        ledc_update_duty(BUZZ_LEDC_MODE, BUZZ_LEDC_CHAN);
        return;
    }
    gpio_set_level(PIN_BUZZER, on ? 1 : 0);
}

/* The motor pad can be driven as a plain GPIO or by LEDC, and BOTH of those
 * are used: the burst ramp and the snooze pulse need PWM, everything else
 * does not. Declared up here because actuators_motor() below has to know
 * which peripheral currently owns the pad - writing a level the hardware
 * ignores is exactly the fault this arrangement exists to prevent. */
static bool s_motor_pwm;
static void motor_pwm_duty(int pct);
static void motor_pwm_down(void);

void actuators_motor(bool on)
{
    if (!s_booted) {
        actuators_boot_safe();
    }
    /* ---- whoever turns the motor off must own the pad --------------------
     * The symptom is a vibration motor that keeps running after the alert.
     *
     * Writing gpio_set_level() alone is correct only while the motor is a
     * plain GPIO and nothing else. The alert's inrush ramp puts the pad under
     * LEDC, and an LEDC-driven pad does not care what gpio_set_level() says:
     * outputs_off() at the end of the burst calls this, the call does
     * nothing, and the motor runs on at the ramp's last
     * duty with no way to stop it short of a reset.
     *
     * The fix is not a teardown here: it is that two code paths were driving
     * one pad through two different peripherals and only one of them knew.
     * Ownership is resolved in one place - if LEDC has the pad, this either
     * drives it through LEDC or takes it back, and no path is left that
     * writes a level the hardware will ignore. */
    if (s_motor_pwm) {
        if (on) {
            motor_pwm_duty(100);
            s_motor = true;
            return;
        }
        motor_pwm_down();
        s_motor = false;
        return;
    }
    s_motor = on;
    gpio_set_level(PIN_MOTOR, on ? 1 : 0);
}

bool actuators_buzzer_state(void) { return s_buzzer; }
bool actuators_motor_state(void)  { return s_motor; }

void actuators_safe_all_off(void)
{
    /* Order matters only in that the two loud/physical outputs go first; the
     * LED call is a no-op when the RMT channel was never lazily created. */
    actuators_buzzer(false);
    actuators_motor(false);
    led_off();
}

/* ---------------------------------------------------------------------------
 * The snooze motor: continuous, weak, and on its own LEDC channel.
 *
 * The alert motor is a plain GPIO at full strength; this is the weak run that
 * lasts the whole snooze, so an operator can feel that the device is snoozed
 * rather than off.
 *
 * 25 kHz, and that number is load-bearing. The acoustic scoring band is
 * 70-2000 Hz and the useful band reaches 8 kHz. A PWM frequency inside either
 * would inject a tone straight into the detector's own passband, from a
 * transducer bolted to the same PCB as four microphones. 25 kHz is above both
 * and inaudible.
 *
 * The kick is required. An ERM at 25 % duty from rest will very likely not
 * turn at all - static friction has to be broken with full drive first. So:
 * 100 % for SNOOZE_MOTOR_KICK_MS, then drop to the running duty.
 *
 * ON EXIT the pad goes back to being a plain GPIO driven LOW, so
 * actuators_boot_safe()'s guarantee still holds and the alert path - which
 * drives this same pin with gpio_set_level - is completely unaffected. */
#define MOTOR_LEDC_MODE  LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER LEDC_TIMER_1
#define MOTOR_LEDC_CHAN  LEDC_CHANNEL_1
#define MOTOR_LEDC_BITS  LEDC_TIMER_10_BIT
#define MOTOR_LEDC_HZ    25000

static void motor_pwm_duty(int pct)
{
    const uint32_t full = (1u << MOTOR_LEDC_BITS) - 1u;
    uint32_t d = (full * (uint32_t)pct) / 100u;
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHAN, d);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHAN);
}

/* Hands the motor pin back from LEDC to a plain GPIO driven LOW. A no-op if
 * LEDC never had it. */
static void motor_pwm_down(void)
{
    if (!s_motor_pwm) {
        return;
    }
    motor_pwm_duty(0);
    ledc_stop(MOTOR_LEDC_MODE, MOTOR_LEDC_CHAN, 0);
    /* BACK TO A PLAIN GPIO, DRIVEN LOW. Level before direction, the same
     * order boot_safe uses, so the pad never spends an instruction as a
     * driven HIGH into an NPN base. */
    gpio_set_level(PIN_MOTOR, 0);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_MOTOR,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_MOTOR, 0);
    s_motor_pwm = false;
}

/* Brings the motor pin up as an LEDC output. Shared by the snooze pulse and
 * by the alert's inrush ramp, so there is one place the timer and channel are
 * configured and no way for the two to disagree about them. */
static bool motor_pwm_up(void)
{
    if (s_motor_pwm) {
        return true;
    }
    ledc_timer_config_t t = {
        .speed_mode = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_LEDC_BITS,
        .timer_num = MOTOR_LEDC_TIMER,
        .freq_hz = MOTOR_LEDC_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) {
        return false;
    }
    ledc_channel_config_t c = {
        .gpio_num = PIN_MOTOR,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = MOTOR_LEDC_CHAN,
        .timer_sel = MOTOR_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    if (ledc_channel_config(&c) != ESP_OK) {
        return false;
    }
    s_motor_pwm = true;
    return true;
}

/* ---- the alert's motor, ramped ------------------------------------------
 * With the loads sequenced, 20 injected alerts produce 9 brownouts and the
 * phase stamp puts every one of them in OUTPUTS. The buzzer alone measures
 * safe, so what is left in that phase is the ERM's inrush, which is several
 * times its running current.
 *
 * Alternating the two outputs was tried and made it worse, because
 * alternating restarts the inrush three times a burst instead of once.
 * Ramping is the opposite move: the motor is brought up through PWM so
 * there is no step to draw an inrush at all, and it is one restart or none.
 *
 * pct 0 releases the pin back to a plain GPIO the way the snooze path does. */
void actuators_motor_duty(int pct)
{
    if (!s_booted) {
        actuators_boot_safe();
    }
    if (pct <= 0) {
        actuators_motor(false);
        return;
    }
    if (pct > 100) {
        pct = 100;
    }
    if (!motor_pwm_up()) {
        return;
    }
    motor_pwm_duty(pct);
    s_motor = true;
}

void actuators_motor_snooze(bool on)
{
    if (!s_booted) {
        actuators_boot_safe();
    }
    if (!on) {
        motor_pwm_down();
        s_motor = false;
        return;
    }
    if (!motor_pwm_up()) {
        return;
    }
    motor_pwm_duty(100);
    vTaskDelay(pdMS_TO_TICKS(SNOOZE_MOTOR_KICK_MS));
    motor_pwm_duty(SNOOZE_MOTOR_DUTY_PCT);
    /* The freeze predicate reads this, and the snooze motor is exactly the
     * kind of in-band rotor Tier-3 must not hear. */
    s_motor = true;
}
