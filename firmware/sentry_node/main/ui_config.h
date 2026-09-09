/*
 * ui_config.h - tuning surface for the operator interface.
 *
 * Every constant the alert, button, LED, display and power-policy code reads
 * lives here and nowhere else; nothing in the UI may hardcode one. These get
 * retuned in the field, so keeping them in one place means a change is one
 * line rather than a code hunt, and no constant can appear twice and disagree
 * with itself.
 *
 * No detector parameter belongs in this file. Thresholds, bands, weights,
 * tracker constants and floor time-constants are sealed and live in the
 * generated headers.
 */
#pragma once

/* ---------------------------------------------------------------- alert ---
 *
 * The beeper, motor, LED and screen all belong to one alert window. The three
 * outputs stop at ALERT_OUTPUT_MS; the screen returns home when its own hold
 * expires.
 *
 * Seven seconds is set by audibility, not by the detector: the alarm has to
 * turn the head of somebody twenty metres away. The buzzer already runs at
 * 50% duty, which is peak loudness for a square wave - any other duty is a
 * narrower pulse and quieter - so loudness is bought with duration and the
 * motor rather than with duty.
 *
 * The cost is deafness: the Tier-3 self-interference freeze covers the whole
 * output window plus FREEZE_TAIL_MS, so the device cannot hear on that tier
 * for about seven and a half seconds after every alert.
 */
#define ALERT_OUTPUT_MS          7000
#define ALERT_SCREEN_HOLD_MS     5000

/* A pulse train rather than a solid tone: seconds of continuous buzzer at
 * close range is unusable, and a pulse is both more audible and more obviously
 * artificial than a steady note. Set ALERT_BUZZER_PULSED to 0 for a solid
 * tone. */
#define ALERT_BUZZER_PULSED         1
#define ALERT_BUZZER_ON_MS        220
#define ALERT_BUZZER_OFF_MS       160

/* Full duty is safe because the radio burst finishes before the outputs start
 * (ALERT_OUTPUTS_START_MS), so the motor no longer competes with the PA for
 * the rail. ALERT_MOTOR_RAMP_MS is what protects it from its own inrush. */
#define ALERT_MOTOR_DUTY_PCT      100

/* Tier-3 and Tier-4 stay frozen this long after the outputs stop: a buzzer's
 * decay and an ERM's spin-down are still a rotor to an envelope statistic. */
#define FREEZE_TAIL_MS            500

/* --------------------------------------------------------------- snooze ---
 *
 * "Stop that so I can listen", not a mute switch. Ten seconds, with a weak
 * motor pulse so the operator can feel that the device is snoozed rather than
 * off. */
#define SNOOZE_MS               10000
#define SNOOZE_MOTOR_DUTY_PCT      25

/* Pulse at entry only. A continuous motor is a rotor at 100-200 Hz with
 * harmonics up the band: it forces the self-interference freeze to cover the
 * whole snooze, leaving the device deaf for 10.5 s per press, and it puts a
 * steady in-band comb into v1 and Tier-2 that the floor absorbs and then
 * releases at the tail - a self-alert waiting to happen. The continuous path
 * stays behind this constant. */
#define SNOOZE_MOTOR_CONTINUOUS      0
/* An ERM at 25% from rest will very likely not turn at all: static friction
 * needs breaking with full drive first, after which a much lower duty
 * sustains it. If the motor does not start even with this kick, raise
 * SNOOZE_MOTOR_DUTY_PCT. */
#define SNOOZE_MOTOR_KICK_MS       60

#define LED_SNOOZE_R                0
#define LED_SNOOZE_G                0
#define LED_SNOOZE_B              180

/* --------------------------------------------------------------- button ---
 *
 * One button, two gestures, separated only by timing: a tap is a press
 * released before BTN_HOLD_MS, a hold is a press still down at BTN_HOLD_MS.
 * The hold fires at that instant rather than at release, so there is no
 * timing window anywhere and no gesture can be missed by being too slow or
 * swallowed by being too quick.
 *
 * Snooze fires on release, not on press: otherwise a long press would snooze
 * on its way to OFF and the operator would get a buzz every time they turned
 * the device off. A tap release is about 100 ms, so this is still immediate.
 *
 * The pin is polled, not edge-triggered. GPIO21 reaches a tactile switch to
 * GND with only the chip's internal pull-up - tens of kOhm - on a board that
 * also carries a brushed ERM, a buzzer coil and a transmitter, which is
 * enough to produce phantom presses on an edge interrupt. A press needs
 * BTN_PRESS_SAMPLES consecutive lows and a release BTN_RELEASE_SAMPLES
 * consecutive highs, so a single spike can never register. The optional
 * 100 nF on the pad is a help, not a prerequisite. */
#define BTN_POLL_MS                  5      /* polled, not edge-triggered     */
#define BTN_PRESS_SAMPLES            5      /* 25 ms low to register a press  */
#define BTN_RELEASE_SAMPLES          5      /* 25 ms high to register release */
/* During an alert a press means "silence this now", and it is the one press
 * that cannot be taken back: a glitch there turns a seven-second alarm into
 * two beeps. It is also the noisiest moment the button ever sees, with the
 * buzzer and motor running centimetres from the pad. Twenty lows is 100 ms -
 * longer than any coupled transient, far shorter than a human press. Outside
 * an alert the 25 ms rule stands, because a missed press there is just a page
 * that did not turn. */
#define BTN_ALERT_PRESS_SAMPLES     20      /* 100 ms low during an ALERT     */
#define BTN_MIN_GAP_MS             120      /* a press closer than this to the
                                             * last release is a phantom      */
#define BTN_HOLD_MS              1500      /* hold this long = OFF, or wake  */

/* Press feedback on every press-down, tap and hold alike. */
#define BTN_MOTOR_PULSE_MS          60
#define BTN_BEEP_MODE                0      /* 0 = full-drive ticks, 1 = LEDC tone */
#define BTN_TICK_MS                 12      /* mode 0: tap tick length        */
#define BTN_HOLD_TICK_PERIOD_MS    250      /* mode 0: tick train period held */
#define BTN_BEEP_PWM_HZ          25000      /* mode 1: above the 8 kHz band   */
#define BTN_BEEP_DUTY_PCT           30      /* mode 1: swept with U b         */
#define BTN_CONFIRM_MOTOR_MS       150      /* one pulse when a hold completes */

/* ------------------------------------------------------ LED reliability ---
 *
 * The fitted part is a WS2812B-V6 (Worldsemi, LCSC C52917433), whose timing
 * windows are not the classic WS2812B ones many drivers still carry. V6:
 * T0H 220-380 ns, T0L 750-1600, T1H 750-1600, T1L 220-420, and a reset longer
 * than 280 us - one datasheet revision prints 300. The classic figures
 * (T0H 400 ns, reset 50 us, which is what the stock encoder emits) are out of
 * spec for this silicon on both counts and are the leading explanation for a
 * pixel emitting a colour nobody commanded.
 *
 * The repeats and periodic re-send below are tolerance, not correctness: they
 * make a single mis-decoded frame invisible to a human because the next frame
 * is 400 us or 500 ms away. That is worth having and it is not a fix for a
 * rail sitting at its floor. */
#define LED_T0H_NS                 300      /* V6 window 220..380             */
#define LED_T0L_NS                 900      /* V6 window 750..1600            */
#define LED_T1H_NS                 900      /* V6 window 750..1600            */
#define LED_T1L_NS                 300      /* V6 window 220..420             */
#define LED_RESET_US              320       /* V6 needs > 280; one rev says 300 */
#define LED_SEND_REPEATS            3
#define LED_INTERFRAME_US         400
#define LED_REFRESH_MS            500

/* The state language: one meaning per signal, always. Colours live here with
 * the cadences because together they are the language, and an operator reads
 * both at once. Fast red means alerting and nothing else; the no-audio fault
 * is magenta, a colour used nowhere else, because one signal with two meanings
 * would make a dead microphone bus and a drone overhead the same light.
 *
 * The guard heartbeat is 25% duty. Anything dimmer reads as a dead LED at a
 * glance - the only instrument that can judge whether a blink is visible is a
 * person looking at the device. The cost is roughly 0.5 mA average going to
 * roughly 1.8 mA, which is not worth trading a legible heartbeat for. It is
 * still visibly a pulse rather than the solid that means snoozed. */
#define LED_GUARD_ON_MS            300
#define LED_GUARD_PERIOD_MS       1200
#define LED_ALERT_ON_MS            100
#define LED_ALERT_OFF_MS           100
#define LED_FAULT_ON_MS            200
#define LED_FAULT_OFF_MS           400

#define LED_GUARD_R                  0
#define LED_GUARD_G                 96
#define LED_GUARD_B                  0
#define LED_ALERT_R                255
#define LED_ALERT_G                  0
#define LED_ALERT_B                  0
#define LED_FAULT_R                255
#define LED_FAULT_G                  0
#define LED_FAULT_B                255

/* How often the pattern engine re-evaluates. Fine enough that a 100 ms alert
 * flash is not visibly ragged, coarse enough to cost nothing on core 1. */
#define LED_TICK_MS                 20
/* Brightness cap. Full white is about 55 mA and sags a marginal rail hardest,
 * which can brown out the pixel's own decoder - the failure this cap exists to
 * avoid. Expected to stay plainly visible behind the enclosure's light pipe. */
#define LED_MAX_CHANNEL           180

/* --------------------------------------------------- the ambient bucket ---
 *
 * A coarse, slow-moving word for what the device is hearing: QUIET / LOW /
 * BUSY / LOUD. It is a quantisation of the detector's existing adaptive floor
 * and adds no signal processing of its own - the floor is already computed
 * every frame and this reads its mean once a second.
 *
 * The thresholds are in dB relative to the floor's own arbitrary units and
 * are provisional: set them from the field by running the device somewhere
 * quiet and somewhere busy and reading AMB= off the status page.
 *
 * AMBIENT_HYST_DB stops the word flickering on a boundary; it should change a
 * handful of times an hour. */
#define AMBIENT_DB_LOW          -55
#define AMBIENT_DB_BUSY         -45
#define AMBIENT_DB_LOUD         -35
#define AMBIENT_HYST_DB           3

/* ------------------------------------------- display refresh discipline ---
 *
 * An e-paper refresh costs hundreds of milliseconds and the home screen is up
 * for hours. It redraws only when something an operator would notice has
 * actually changed, and never faster than this. */
#define HOME_MIN_REDRAW_MS      60000
#define DEGHOST_EVERY_N_PARTIAL    20

/* ------------------------------------------------ boot and power policy ---
 *
 * The slide switch cuts the regulator's enable, not power, and C24 330 uF
 * plus C26 100 uF hold the 3V3 rail up afterwards. With the chip awake the
 * rail collapses in tens of milliseconds, so every flick of the switch is a
 * clean cold boot; with the chip in deep sleep it draws microamps and 430 uF
 * takes about a second to fall far enough to reset anything, so a quick
 * off-and-on would wake a device that is still asleep behind a stale OFF
 * image and appears dead.
 *
 * Any non-deep-sleep reset therefore comes up listening, and a deliberate OFF
 * stays awake for OFF_AWAKE_MAX_MS before deep sleeping with a button wake
 * that exists. */
#define BOOT_ALWAYS_LISTENING        1      /* any non-deep-sleep reset -> LISTENING */
#define BATT_VERDICT_GRACE_MS    10000      /* no EMPTY verdict earlier than this */
#define BATT_EMPTY_V              3.30f
#define BATT_EMPTY_SUSTAIN_MS    30000
#define BATT_RESUME_V             3.50f     /* EMPTY sleep resumes above this */
#define BATT_EMPTY_WAKE_S           60      /* timer wake while in EMPTY sleep */
/* Hold-to-off has to mean near-zero current, because the slider is hard to
 * reach inside the enclosure and the button is the only practical way to turn
 * a deployed unit off. Five seconds is long enough for the OFF image to
 * finish drawing and short enough that off means off; the cost is that a
 * slide-off/slide-on needs a five-second pause. Hold-to-wake is the universal
 * recovery either way. */
#define OFF_AWAKE_MAX_MS  (5UL*1000UL)         /* USER OFF sleeps this fast */

/* ----------------------------------------------------------- INFO page ---
 *
 * One live page, replacing the menu and every page indicator. Its repaint
 * cadence is deliberately exempt from HOME_MIN_REDRAW_MS: a page somebody is
 * standing in front of reading is not the resting screen. */
#define INFO_REFRESH_MS           3000      /* partial re-paint while shown   */
#define INFO_TIMEOUT_MS         120000      /* then back to the base state    */

/* Boot is silent: no buzzer and no motor at boot under any circumstance,
 * including after a brownout. A device that makes a noise it was not asked to
 * make trains its operator to ignore noises, so every sound the unit makes is
 * a press or an alert by construction.
 *
 * The enforcement is not a flag over the actuators. The button sampler
 * refuses to register a press until it has seen the pin genuinely released,
 * so a pad sitting low at reset cannot produce a press-down tick. The single
 * green pulse below is the only boot signal there is. */
#define BOOT_SILENT                  1
#define BOOT_READY_LED_MS          300

/* ------------------------------------------------------ alert timeline ---
 *
 * The alert's four loads - buzzer, motor inrush, LoRa transmit burst and a
 * full panel refresh - must not start together. Started at once they brown
 * the board out on a healthy cell, and every entry into ALERTING ends in a
 * reset: the operator hears the buzzer's first cadence step and then nothing.
 * Sequencing them is the whole of this block. A reset on a healthy cell
 * remains a hardware question; what sequencing does is stop the firmware
 * asking it.
 *
 * The radio goes first because its burst is the longest and least
 * interruptible load. At SF10/BW125/CR4-5 one 18-byte frame is 329.7 ms of
 * air, so the three-frame burst runs continuously for about a second and the
 * nominal spacing cannot separate frames that each take longer than the gap.
 * ALERT_TX_BURST_MS is derived from lora_link.c's own airtime figures and is
 * the constant to change if the radio changes; the static assertions below
 * keep the outputs and the screen behind it.
 *
 * The cost is latency: the buzzer starts ALERT_OUTPUTS_START_MS after the
 * decision rather than immediately, so with a detection latency of ~0.23 s
 * the operator hears the alarm about 1.7 s after the drone is detected.
 *
 * The window is fixed at ALERT_OUTPUT_MS from the outputs' start. A detection
 * inside it is counted and never extends it. */
#define ALERT_TX_LEAD_MS             0      /* packet 1, at detection         */
/* A phase label, not the schedule. The frames are spaced by lora_proto.c's
 * LORA_TX_GAP_MS +- LORA_TX_JITTER_MS; this constant only decides which of
 * TX1/TX2/TX3 a brownout is stamped with, so it tracks the nominal gap.
 * Changing it moves no packet. */
#define ALERT_TX_SPACING_MS        350      /* packets 2 and 3                */
/* Keep equal to lora_link.c's burst airtime or the load collision returns. */
#define ALERT_TX_BURST_MS         1400      /* worst case 2*500 gap + 330 air */
#define ALERT_OUTPUTS_START_MS    1500      /* buzzer and motor: AFTER the burst */
/* The ramp kills the ERM's inrush. alert_ui_tick() runs once per 32 ms frame,
 * so a 100 ms ramp is three steps of a third each - three inrushes, not a
 * ramp. 300 ms gives nine steps, small enough that each is one. */
#define ALERT_MOTOR_RAMP_MS        300      /* PWM 0 -> cap, kills inrush     */
/* The motor begins its ramp this long after the buzzer, by which time the
 * buzzer is a steady load rather than a step. Only the part of the alert that
 * is felt arrives late; the part that is heard is unaffected. */
#define ALERT_MOTOR_START_MS       250      /* after the outputs start        */

/* Whether the motor may run inside the transmit burst. 0 leaves the buzzer
 * alone in the burst. The snooze pulse is unaffected either way, so the motor
 * still works and can still be felt. */
#define ALERT_MOTOR_IN_BURST         0

/* ------------------------------------------------------ link test feedback -
 *
 * A link test has to be heard outdoors, so these beeps are long rather than
 * weak. What keeps them distinct from an alert is the pattern, not the
 * volume: an alert is a 220/160 stutter sustained for seconds, while a link
 * test is a couple of long, well separated beeps over in about a second.
 *
 * The count carries the meaning and the three cases must stay tellable apart:
 *
 *   2 beeps, local, on the press   "I sent a request"
 *   2 beeps, on the far unit       "I heard it"
 *   3 beeps, local, afterwards     "the reply came back, the link is proven"
 *
 * The buzzer is safe to lengthen. The motor deliberately is not: its inrush
 * is what the alert timeline above exists to manage, and this feature is not
 * worth reopening that. */
#define LINK_BEEP_MS               400
#define LINK_GAP_MS                250
#define LINK_MOTOR_MS              180
#define LINK_MOTOR_DUTY_PCT         60

/* The vibration is the part of an alert that reaches an operator who is not
 * looking at the device, so rather than drop it, it runs at the end of the
 * alert when nothing else is switching: the radio finished long before, the
 * buzzer has stopped and the panel refresh completed seconds ago. A single
 * ramped pulse, which doubles as an "the alert has finished" signal.
 * ALERT_MOTOR_TAIL_MS 0 disables it. */
#define ALERT_MOTOR_TAIL_MS        400
#define ALERT_MOTOR_TAIL_RAMP_MS   300
#define ALERT_MOTOR_TAIL_DUTY_PCT   60
/* After the outputs have started and after the burst: a 2 s full refresh is
 * the single biggest load on the panel rail. */
#define ALERT_SCREEN_START_MS     1800      /* the full refresh starts here   */
#define ALERT_FREEZE_TAIL_MS       500

/* ------------------------------------------------------------ test mode ---
 *
 * A persisted bench posture that changes the glass and the console and
 * nothing about detection: no threshold, no alert timing and no output is
 * conditioned on it.
 *
 * The live page is exempt from HOME_MIN_REDRAW_MS for the same reason the
 * INFO page is - that limiter exists to stop the home screen flapping, and a
 * page of live numbers that does not move is not live. A partial refresh is
 * not free and this page repaints all day at a test ladder, which is the
 * trade being made.
 *
 * The timeout is zero, meaning never. In product mode a page returns to MAIN
 * after 30 s so a unit left on a page still guards visibly; at a test ladder
 * the operator is twenty metres away with a rig running, and a page going
 * dark on its own is the failure rather than the safeguard. */
#define TEST_REFRESH_MS           5000
#define TEST_PAGE_TIMEOUT_MS         0    /* 0 = never times out in test mode */

/* The frozen alert page holds until somebody taps it, which is right at a
 * test ladder where the page is the measurement, and wrong everywhere else:
 * left unbounded it latches the box onto one page after the first alert and
 * every later alert is recorded without ever being shown.
 *
 * Ten minutes is long enough to walk over and photograph it - comfortably
 * past four page-return periods - and short enough that a box left alone
 * always comes back to LISTENING by itself. In product mode the ALERT screen
 * returns on ALERT_SCREEN_HOLD_MS and never freezes; this bounds only the
 * test-mode path that does. */
#define FROZEN_RETURN_MS        600000UL   /* ten minutes */
#define TEST_STICKY_AFTER_ALERT      1    /* an alert returns to TEST         */

/* `U alert every N` exists for the frame-budget gate and the brownout soak,
 * and it is the only thing on this device that fires the buzzer, motor and
 * radio on a repeat with nobody asking. If the host goes away mid-soak - a
 * dropped USB bus, a closed script - nothing is left to stop it, so the
 * repeat stops itself. Forty is comfortably more than a soak asks for and far
 * less than a night. `U alert every N` re-arms it and `U alert every 0` stops
 * it at once. This bounds a bench tool only: a real alert is not counted here
 * and is never capped. */
#define AUTO_ALERT_MAX_REPEATS      40

/* A channel is dead only when its 60 s RMS sits below 1/MIC_DEAD_RATIO of the
 * median of the other three for MIC_DEAD_S, evaluated once a minute. A looser
 * rule fires on ordinary variation and flickers the count on the resting
 * screen. */
#define MIC_DEAD_RATIO              20
#define MIC_DEAD_S                  60

/* Battery slices, millivolts. A full cell off the charger rests at 4.10-4.18 V
 * and sags 30-50 mV under this device's load, so the top slice has to sit
 * below that sag or a loaded full cell never reads 4/4. Hysteresis on every
 * edge, because on e-paper a flickering bar is a full 2 s refresh each way. */
#define BATT_SLICE4_MV            4000
#define BATT_SLICE3_MV            3850
#define BATT_SLICE2_MV            3720
#define BATT_SLICE1_MV            3550
#define BATT_SLICE_HYST_MV          30

/* Tier-4's slices must keep out of the other tiers' frames. The tier phases
 * are T2 on even frames, T3 on 3 (mod 4) and T4's decision on 1 (mod 8),
 * which leaves 1 (mod 4) free. Four slots per 16-frame update at 141 bins
 * covers all 563. */
#define T4_SLICE_PHASE_MOD           4
#define T4_SLICE_PHASE               1
