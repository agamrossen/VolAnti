# PORTING_NOTES.md — every place the C could have diverged from the Python

Companion to `main/detector.c`. `src/detector.py` is normative; this file
records each site where a faithful port required a decision, what was decided,
and why. Nothing here is a tolerance and nothing here is a tuning knob.

Result of the port: **all four golden vectors PASS** — zero decision-field
mismatches, zero chain divergence, at 61% of the frame budget.

(The two `f0_bin` differences that appear since the float32 whitening landed
are both frame 0, pre-warmup, at score ~5e-7: on frame 0 the floor is
initialised to the magnitude, so `S = ln2` in every bin and teeth-minus-gaps is
exactly zero for all ~1881 candidates. The argmax is a coin flip among exact
ties and can never reach a decision field.)

---

## 0. The reference is NOT uniformly float32

This is the single most important thing to understand before reading the C.
`src/detector.py` looks like a float32 pipeline and is not one. Reading the
code rather than the comments:

| site | Python dtype | why |
|---|---|---|
| `np.fft.rfft(block * window)` | **float64** (complex128) | numpy's FFT always promotes to double, whatever the input dtype |
| `st.floor` | **float64** from frame 0 onward | `np.where(up, a_up, a_fall)` builds a float64 array from two numpy float64 scalars, so `a*prev + (1-a)*mag` promotes |
| `r`, `m1`, `m2`, `flat` | **float64** (frame 0: float32) | consequence of the above |
| `log1p(mag/(floor+1e-9))` | **float64**, cast to float32 after `minimum` | consequence of the above |
| `S`, `tv`, `gv`, `tw`, `gw`, `znorm`, `scores` | **float32** | genuinely, by construction |
| tracker arithmetic (`f0`, `t`, thresholds, jitter) | **float64** | Python floats |

So "float32 throughout" would itself have been a divergence. The port matches
the dtype **at each site**: `double` for the floor/whitening/tracker path,
`float` for the spectrum gather and the comb score.

Two places now differ from the reference dtype, both deliberately:

* the **FFT**, which esp-dsp only provides in float32 — irreducible, and
  budgeted before a line of C was written (§1);
* the **whitening `log1p`**, which is float32 by choice
  because float64 cost 21.8 ms of a 32 ms frame and the port could not run in
  real time with it (§4). Measured, not argued: ≤1 ulp of float32 on `S`,
  ~150x below the measured decision tolerance, and re-proved against all four
  golden vectors with zero decision mismatches.

---

## 1. FFT — precision, scaling, ordering

### 1a. Precision: measured, not assumed

Before choosing esp-dsp's float32 FFT over a soft-float64 one (~2 M cycles per
frame), the question "how much FFT error can the golden set actually absorb?"
was answered by measurement, not by a rule of thumb.

`scratchpad/sensitivity.py` imports the unmodified reference detector, wraps
`front_end` to inject error into the spectrum, re-runs all three vectors and
counts decision-field mismatches against the shipped traces (3 seeds per
point). Two models: per-bin relative, and norm-scaled (`spec += eps*rms*n`),
the latter being what a float32 radix-2 FFT actually does — its roundoff is
proportional to the norm of the transform, so small bins take a large
*relative* hit.

```
model 'abs' (norm-scaled)
  eps=1e-8 .. 1e-5   0 decision-field mismatches on all three vectors
  eps=1e-4           12 mismatches + VERDICT FLIP on golden_marginal_pos
```

A float32 radix-2 FFT of length 2048 has norm-scaled roundoff of about
`eps_f32 * sqrt(log2 N)` ≈ 2e-7 — roughly **50x inside** the measured
tolerance. esp-dsp float32 was therefore chosen on evidence.

The tightest decision margins in the reference, for context:

| vector | frame | role | \|score − 2.14\| |
|---|---|---|---|
| golden_marginal_pos | 169 | takes chain 5 → 6, i.e. FIRES | 4.65e-4 |
| golden_marginal_pos | 74 | — | 1.78e-4 |

Both reproduced exactly (device 2.1404647827148438 vs reference 2.140465).

### 1b. Scaling

None. `dsps_fft2r_fc32` and `np.fft.rfft` are both the **unnormalised** DFT —
there is no 1/N on either side. Confirmed on-device with the `F` self-test
command: `x[n] = cos(2π·64n/2048)`, no window, gives `|X[64]| = 1024.0002`
(exactly N/2 as required) and ~1e-3 in every other bin.

### 1c. Ordering — the real difference

esp-dsp does the packed-real trick: the N real samples are consumed as N/2
**complex** values.

```c
dsps_fft2r_fc32 (buf, N/2);   /* N/2-point complex FFT   */
dsps_bit_rev2r_fc32(buf, N/2);   /* natural order           */
dsps_cplx2real_fc32(buf, N/2);   /* unpack to a real spectrum */
```

The result is bins 0..N/2−1 in place, **with the real-valued Nyquist bin
packed into the imaginary slot of bin 0** (`dsps_cplx2real_fc32_ansi_` does
`result[0].im = X[0].re − X[0].im`, which is X[N/2]). numpy's `rfft` instead
returns N/2+1 separate bins.

`front_end()` unpacks to the numpy layout, so nothing downstream ever learns
the difference:

```
spec[0]      = { buf[0], 0 }        DC, real
spec[k]      = { buf[2k], buf[2k+1] }   k = 1 .. 1023
spec[1024]   = { buf[1], 0 }        Nyquist, real, un-packed from bin 0's im
```

Bin 1024 is not decoration: `mag.sum()` (the tonality energy gate) and
`r.mean()` (the flatness moments) run over all 1025 bins, so dropping it
would move the floor gate.

`dsps_cplx2real_fc32` reads the **radix-4** twiddle table, so
`dsps_fft4r_init_fc32()` is required even though no radix-4 FFT is ever
called. Its `wind_step = table_size / N` needs `dsps_fft4r_w_table_size` to be
`2 * max_fft_size`, which is why both inits take `N_FFT >> 1`.

### 1d. Sign convention — deliberately not reconciled

numpy uses `exp(-2πi kn/N)`. Whether esp-dsp matches or conjugates is
irrelevant here: only `|X|` is ever consumed, and conjugation leaves the
magnitude unchanged. Do not "fix" this if beamforming later needs phase —
**check the convention then**, because the combiner is where phase first
matters.

### 1e. 16-byte alignment is REQUIRED, not preferred — this one bit

`dsps_fft2r_fc32` resolves on ESP32-S3 (with `CONFIG_DSP_OPTIMIZED`) to
`dsps_fft2r_fc32_aes3`, the SIMD assembly variant, whose 128-bit loads need
the data buffer 16-byte aligned. `__attribute__((aligned(16)))` on a struct
member does **not** reach the allocator, and `heap_caps_calloc` only promises
4.

The first run of the port produced a magnitude spectrum that was flat at ~330
across every bin and nearly identical from frame to frame — the FFT silently
returning nonsense rather than faulting. The allocation is now
`heap_caps_aligned_calloc(16, ...)` and `I` reports the alignment it actually
got. If a future refactor moves `detector_work_t`, keep the aligned
allocation.

---

## 2. numpy's summation order, reproduced exactly

`np.sum` / `np.mean` over a contiguous axis are **not** left-to-right. numpy
uses an 8-accumulator unrolled block up to `PW_BLOCKSIZE = 128` and recurses
above it:

```
n < 8      naive left-to-right
n <= 128   r[0..7] = a[0..7]; r[k] += a[i+k] in steps of 8 while i < n-(n%8);
           res = ((r0+r1)+(r2+r3)) + ((r4+r5)+(r6+r7)); then += the remainder
n > 128    n2 = (n/2) rounded DOWN to a multiple of 8; recurse on both halves
```

Left-to-right accumulation over the 1025-bin spectrum differs from numpy in
the 6th significant figure — the same order as the 1.78e-4 margin at
`golden_marginal_pos` frame 74. So the order is reproduced, in
`pw_sum_f32` / `pw_sum_f64`.

Validated bit-for-bit against numpy 2.4.6 on the host before any C was written
(`scratchpad/pairwise_check.py`): **0 mismatches** over 1800 random arrays at
n = 7 / 12 / 137 / 1025 / 2048 in both float32 and float64, plus `axis=1` rows
of 12 and `mean`.

Sites that depend on it:

| Python | n | dtype |
|---|---|---|
| `mag.sum()` — tonality energy gate | 1025 | float32 |
| `r.mean()`, `(r*r).mean()` — flatness | 1025 | float64 |
| `(tv*tw).sum(axis=1)`, `(gv*gw).sum(axis=1)` | 12 | float32 |

The 12-element case is not a special case worth skipping: it evaluates as
`((a0+a1)+(a2+a3))+((a4+a5)+(a6+a7))` followed by `+a8 +a9 +a10 +a11` in
order, which is *not* the same float32 result as a plain loop.

---

## 3. Compiler flags that are load-bearing

`main/CMakeLists.txt` compiles this component with:

```
-ffp-contract=off -fno-fast-math -fno-unsafe-math-optimizations
-fno-associative-math
```

`-ffp-contract=off` is the critical one. Xtensa has `MADD.S`, and without it
GCC is free to contract `S[i0]*(1-fr) + S[i0+1]*fr` into a single fused
operation with one rounding instead of two. That silently changes the comb
score in the last bits, on every one of 46,344 interpolations per frame. The
others forbid reassociation, which would defeat §2.

---

## 4. The floor, step by step

`_update_floor`, "tonality" branch, in the order the Python actually executes:

```
prev   = floor                     the OLD array (Python rebinds, C reads
                                   before writing, per bin)
e      = float(mag.sum())          float32 pairwise sum, then widened
rising = e > gate_ratio * e_slow   BEFORE e_slow is updated
r      = mag / (prev + 1e-9)
flat   = mean(r)^2 / (mean(r*r) + 1e-12)
fast   = rising and flat > flat_hi
e_slow = a_energy*e_slow + (1-a_energy)*e
a_up   = a_rise_fast if fast else a_rise
a      = where(mag > prev, a_up, a_fall)
floor  = a*prev + (1-a)*mag
S      = min(log1p(mag / (floor + 1e-9)), sat_log)     <-- the NEW floor
```

Two traps:

* **S whitens against the floor AFTER this frame's update, while the tonality
  gate looks at the one BEFORE.** Getting these the same way round is the
  difference between passing and a slow drift that only shows up 200 frames
  later.
* `rising` uses `e_slow` from the *previous* frame. On frame 0 `e_slow = e`,
  so `rising = e > 1.35e` is false for any `e >= 0` and the fast path can
  never fire on frame 0. The C relies on this rather than special-casing.

**Frame-0 dtype quirk, deliberately not reproduced.** On frame 0 only, Python's
`prev` is still float32 (`mag.copy()`), so `r` and the flatness moments are
computed in float32; from frame 1 they are float64. The C computes them in
double on every frame. This is safe *because* `fast` is false on frame 0
regardless of `flat` (see above) and `flat` is not a trace field, so the
quirk cannot reach any observable. Recorded here so nobody "fixes" it into a
real divergence.

**Smoothing coefficients** (`a_rise`, `a_fall`, `a_rise_fast`, `a_energy`) are
`exp(-hop/fs/tau)` evaluated **on the host in float64** by
`scripts/gen_headers.py` and frozen into `detector_config.h` as 17-digit
literals. The device never calls `exp()`, so it never depends on its libm
agreeing with numpy's.

---

## 5. Sample conversion

`x = q / 32767.0f` — 32767, not 32768, matching `make_golden.py`'s
`np.clip(np.round(x*32767), -32768, 32767)` and the inverse
`q.astype(np.float32) / 32767.0`. That division is a **float32** operation
under NumPy 2's NEP 50 weak-scalar promotion (float32 array ÷ Python float
stays float32), so the C does it in float32 too.

---

## 6. Gather tables — exported, not recomputed

`t_i0`, `t_fr`, `g_i0`, `g_fr`, `tw`, `gw`, `znorm`, `n_tooth` are exported
from an **instantiated `CombDetector`** by `scripts/gen_headers.py`, not
rederived in C. Recomputing them would introduce a second implementation of
`np.arange`, `np.clip`, integer truncation and the `1/sqrt(k)` normalisation —
four more chances to diverge, for no benefit. `znorm` in particular involves
two more float32 sums that would otherwise have to reproduce §2.

Float literals use `%.9g` (float32) and `%.17g` (float64), which IEEE-754
guarantees round-trip exactly.

`tooth_ok` is not exported as a mask: `tooth_f = k*f0` is monotone in `k`, so
the mask is always a **prefix**, and a single count per `f0` reproduces it
exactly. The generator asserts this rather than assuming it, and also asserts
`valid == (n_tooth >= n_harm_min)`.

### Indexed, not quantised

The float tables cost 460 KB of flash traffic **per frame**, and the comb score
was bandwidth bound on them. They are now stored as one byte per entry, an
index into the table's own distinct float32 values:

| table | distinct values | why that number |
|---|---|---|
| `tw`, `gw` | 76 | the normalised 1/sqrt(k) weights depend ONLY on how many harmonics fall below `f_max_harm`, and that count takes 10 values across the whole grid — not 1931 |
| `t_fr`, `g_fr` | 125 | `bin_w = fs/n_fft = 16000/2048 = 125/16`, so every position `f/bin_w` is an exact multiple of 1/125 and its fractional part can only be one of 125 values |

This is **not quantisation**. The LUT holds the original float32 values, so the
dequantised array is bit-identical — the generator asserts that on the raw
uint32 bit patterns and refuses to emit otherwise. Device traces are
byte-for-byte the same as with the float tables.

Scalar quantisation was measured on the host first and is the wrong answer:
uint8 broke the gate outright (13 decision mismatches and a verdict flip),
uint16 survived but is **bigger** than the exact scheme and 100x less accurate.

460.1 KB -> 188.6 KB, comb score 19.5 -> 10.9 ms/frame. `SENTRY_TABLES_LUT=0`
rebuilds with the float tables for an A/B.

Total flash `.rodata` is now 1,835,540 B against 11,854 B of `.data` — the
tables and all four vectors are in flash, not DRAM.

---

## 7. Tracker

Direct transcription of `TrackerState.step`. Points worth naming:

* `track_miss = 2`: a rejected frame does `count = max(0, count - 2)`, and the
  chain/`last_f0` reset happens **only when the counter reaches exactly 0** —
  including when the tracker had not fired. Both branches are reproduced.
* **Octave-tolerant continuity holds `last_f0`**: on an octave match the frame
  is accepted, `is_oct` is set, and `last_f0` is *not* updated; the held value
  is what enters `chain_f0s`.
* **The jitter gate runs on the RAW argmax**, not the accepted chain. With
  `reanchor` off the two are numerically identical, but the C keeps them as
  separate fields so re-enabling re-anchoring cannot silently change which
  series is gated.
* `np.median`: odd n → middle element; even n → mean of the two middle. The C
  sorts a copy and does the same.
* `above_thr` is computed **independently** of warmup, band and continuity,
  exactly as `track_frames` does — it is not "the frame was accepted".
* `band_threshold` is implemented in full even though both offsets are 0.0 and
  it currently returns `thr` unconditionally. The two-tier machinery is kept
  because the bench rig may move the priority band.

`SENTRY_MAX_CHAIN = 1024` bounds the chain arrays (longest vector is 916
frames). Overflow sets a flag that the host comparator treats as a hard
failure — it never wraps silently.

---

## 8. REJECTED features have no code path

Re-anchoring and comb-hold are **absent**, not disabled. `reanch` and
`n_held_bins` are emitted as constant 0 and the host comparator fails the run
if either is ever nonzero. `detector_config.h` carries `_Static_assert`s on
`CFG_REANCHOR_ENABLED == 0` and `CFG_HOLD_ENABLED == 0` with the rejection
rationale inline, so re-enabling one is a deliberate act with the measurement
that killed it in front of you.

`min_teeth == 0`, so the teeth gate is `#if`'d out rather than evaluated to
"always true" — but the `teeth` count itself is still computed and traced,
because it is a trace field.

---

## 10. The microphone: 24-in-32, and what is deliberately NOT done to it

The INMP441 emits **24-bit two's complement, MSB first, in a 32-bit slot**,
Philips timing (MSB one BCLK after the WS edge). The I2S peripheral assembles
that into a 32-bit word as

```
bits 31..8   the 24 data bits, sign correct by position
bits  7..0   zero - the mic drives nothing there
```

so the signed 24-bit value is `raw >> 8`, and the pipeline's int16 is the top
16 bits of it:

```c
dst[k] = (int16_t)(s_raw[k] >> 16);     /* arithmetic shift, sign preserved */
```

That is a pure rescale by 2^-16. Nothing else happens to the sample.

**NO DC removal, NO high-pass, NO gain, NO dither.** The Python reference has
no such stage, so adding one on the device would be an ALGORITHM CHANGE wearing
plumbing's clothes: it would move the floor, the energy gate and the flatness
moment, and every golden number would silently stop meaning what it says. The
DC offset is **measured instead** — meter mode reports it every 500 ms — and
the decision about whether it needs handling is a separate, evidenced one.

Where the DC offset actually lands, so the measurement can be read properly:
bin 0 is never a comb tooth (the lowest is `f0=70, k=1` -> bin 8), so DC cannot
contribute to a comb score directly. It *is* inside `mag.sum()` (the tonality
energy gate) and `r.mean()` / `r*r.mean()` (the flatness moments), so a large
offset biases the floor's fast/slow decision without ever showing up as a
score. That is the thing to watch in the meter numbers.

**Gain invariance, and its one limit.** `S = log1p(mag/floor)` with the floor a
linear EMA of `mag`, so scaling the input by k scales both and leaves S
unchanged; the energy gate is a ratio and the flatness moment is scale-free.
Microphone sensitivity therefore does NOT move the operating point. The limit
is the `+1e-9` in `mag/(floor + 1e-9)` and the `+1e-12` in the flatness
denominator: invariance holds only while levels stay well above those. Do not
right-shift the sample "to be safe" — that is the one way to break it.

### I2S configuration

| | |
|---|---|
| mode | `i2s_std`, master, RX only |
| slot | 32-bit, `I2S_SLOT_MODE_MONO`, `slot_mask = I2S_STD_SLOT_LEFT` |
| why LEFT | the mic's L/R pin is strapped to GND, so it drives the left slot |
| BCLK | `fs * 32 * 2` = 1.024 MHz — std mode always clocks two slots even when only one is stored |
| DMA | 6 descriptors x 512 frames = 3072 samples = **192 ms**, i.e. six whole hops of slack against a 19.6 ms compute |

Pins live in exactly one place, `main/board_pins.h`, and the running firmware
prints them in response to `I` — so what you read is what is flashed.

---

## 9. What is NOT covered by this port

* `combiner()` is the identity at `N_CHANNELS == 1`. The multi-channel branch
  is written but **never executed and never tested** — it is a placeholder for
  beamforming, not a validated path.
* `source_i2s()` is implemented (§10) but, as of writing, **has never seen a
  microphone**. It has been exercised end to end against a floating SD pin:
  the I2S clocks, DMA delivers, samples reach the detector, and C and Python
  agree decision-for-decision on 153 frames of that input. What is unproven is
  everything downstream of the connector.
* Both presets are now exercised: three parity vectors at NORMAL (2.14) and
  `golden_high_alert_pos` at HIGH_ALERT (1.70). Each vector is replayed at the
  threshold it was cut at, carried in `generated/vectors.h` as a named preset
  macro, and the comparator FAILS a run whose header threshold does not match
  the vector's entry in `golden_vectors.json`.
* Real-time is met: **19.6 ms p99 against a 32 ms hop, 61%**. See
  `PORT_LOG.md`.

---

## 11. TIER-2 — `main/detector_t2.c`

Tier-2 is a SECOND consumer of the same seam, not a second pipeline: it takes
the combiner's output exactly as `back_end` does, and it shares no mutable
state with `detector.c`. `G`, `Z`, `L`, `Y` and `R` never enter it. Everything
in §0–§8 above applies to it unchanged; this section records only what is
specific to the second tier.

### 11.1 The dtype decisions, all of them

| quantity | Python reference | C port | why |
|---|---|---|---|
| `floor2` | **float64** | **float64** | `np.where(up, a2_up, a2_dn)` builds a float64 array from two numpy scalars, exactly as v1's floor does. `src/detector_t2.py` mirrors that promotion ON PURPOSE — see its "NUMERICS" section — so the T2 port faces the identical, already-measured relationship the v1 port faces. One religion, one gate. |
| whitening `log1p` | float64 | **float32 `log1pf`** | The one deliberate divergence, and it is the SAME one v1 makes for the same reason: 1025 soft-float64 `log1p` calls cost ~21.8 ms of a 32 ms budget on a core with no double FPU. Within 1 ulp of float32 (~6e-8 on S). |
| magnitude | `np.abs(spec)` in float64, cast to float32 | `sqrtf(re*re+im*im)` in float32 | Identical line to `back_end`'s, identical divergence, already characterised in §0. |
| comb score | float32, numpy pairwise | float32, `t2_pw_sum_f32` | numpy's sum is not left-to-right even at n = 12. |
| decision arithmetic | Python floats | `double` | thresholds, continuity, f0. |
| `hits >= M2/2` | `st.hits * 2 >= m2` | `2 * st->hits >= c->m2` | written the same way on both sides so an odd M2 rounds identically. |

### 11.2 `t2_pw_sum_f32` is a byte-for-byte copy, and that is the point

`detector.c`'s `pw_sum_f32` is `static` and `detector.c` is frozen for this
branch, so the T2 port carries its own copy. If the two ever drift apart the
K-mode T2 gate fails, which is exactly what the gate is for.

### 11.3 The table duplication, measured

`generated/tables.h` declares its arrays `static const`, so including it in a
second translation unit gives that unit its own copy. **Measured cost: the
image grew from 0x1fc570 to 0x22de00, i.e. +197 KB** (about 189 KB of tables
plus ~8 KB of new code), leaving 73% of the app partition free.

D3 requires the T2 grid to be rows 130–730 of the EXISTING tables and forbids
new ones, and `detector.c` is frozen, so a shared-linkage table was not
available tonight. **The one-line fix for the first session allowed to touch
`detector.c`:** move the table definitions into a translation unit of their own
and declare them `extern const` in `tables.h`. Both tiers then share one copy
and the 189 KB comes back.

### 11.4 Coherence is evaluated at the teeth, not over the spectrum

The reference computes `C(b)` over all 1025 bins and then takes a
tooth-weighted mean. The port evaluates `C(b)` **only at the winner's teeth** —
the same ≤ 12 gathers the score already performs — because every other bin is
multiplied by a zero weight. Same number, roughly 40× less work: 24 bin reads
instead of four passes over the spectrum. The brief budgeted ~1.2 ms for this;
it is now negligible.

With one channel, `C(b) = |X|² / (1 · |X|²) = 1` identically, which is why a
mono `K` replay carries `kappa = 1.0` rather than a missing field — and why the
comparator FAILS a mono capture whose kappa is anything else.

### 11.5 Half rate

`st->frame_i++ % decim` is evaluated with the SAME pre-increment order as the
Python (`st.frame_i += 1` then test), so frame 0 always runs on both sides.
Every frame count halves and the floor coefficients are re-derived for the
doubled step, so the integration measured in SECONDS is unchanged. The
comparator detects a rate mismatch as a step-count difference and says so
rather than diffing misaligned series.

### 11.6 What is deliberately NOT in the parity gate

The Python event record carries a MEDIAN f0 over the event; the device's `AL2`
record carries the track f0 **at the firing frame**. Both are telemetry, and
neither is a per-frame field, so neither is diffed. Storing every f0 of a
latched event on the device would be an unbounded buffer for a reporting field.

### 11.7 CX-A is called, not reimplemented

`combiner_cx('a', ...)` returns `detector.c`'s own `combiner()` result, and
`combiner_cx('d', ..., busoff == 0)` falls through to it. So `H` with default
arguments is provably the same arithmetic as `G` — regrouping the sum as
`(M1+M2) + 1.0*(M3+M4)` would change the accumulation order and therefore the
last bits, which is precisely the class of difference this project refuses to
accept by inspection.
