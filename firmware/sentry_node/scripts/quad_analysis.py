"""
quad_analysis.py - the array analysis math, as PURE FUNCTIONS.

Deliberately importable and hardware-free so every judgement the bench will
make can be proven tonight against synthetic signals with known answers. The
bench then tests hardware, not arithmetic. This mirrors how the detector itself
was built: prove the maths on synthetic data first, and let the physical
session be about the physical device.

Geometry ground truth (decision D2 of the overnight brief, as-built):

    +y = NORTH = M3 = the FORWARD / USB-cable edge
    +x = EAST  = M2

    M1 WEST   M2 EAST    E-W baseline 40.64 mm  -> 1.90 samples max delay
    M3 NORTH  M4 SOUTH   N-S baseline 60.96 mm  -> 2.84 samples max delay
    centres coincident; whole 70-2000 Hz search band alias-free

Channel order everywhere in this project is [M1 M2 M3 M4].
"""
import numpy as np

FS = 16000
C_SOUND = 343.0

# ---------------------------------------------------------------------------
# GEOMETRY IS DATA (the port, D12). The numbers below are DERIVED from
# data/geometry_profiles.json rather than written out here, so a new board is
# a new entry in that file and a --geometry flag, not an edit to four scripts
# - three of which would then be updated and one forgotten.
#
# The module-level names keep their old meanings and their old values for the
# default profile, so every existing caller and every existing test still sees
# what it saw. `use_geometry(name)` rebinds them for a different board.
# ---------------------------------------------------------------------------
import geometry as _G                                           # noqa: E402

GEOMETRY = _G.load()

# as-built baselines, metres
BASELINE_EW_M = GEOMETRY.baseline_mm("M1", "M2") * 1e-3   # M1 <-> M2
BASELINE_NS_M = GEOMETRY.baseline_mm("M3", "M4") * 1e-3   # M3 <-> M4

# maximum acoustic delay across each baseline, in samples at FS
MAX_DELAY_EW = GEOMETRY.max_delay_samples("M1", "M2")     # 1.896
MAX_DELAY_NS = GEOMETRY.max_delay_samples("M3", "M4")     # 2.844


def use_geometry(name):
    """Switch the module to another array profile. Returns the Geometry.

    Every expectation this module checks is recomputed from the profile's
    positions, so the bench tools test the board they are pointed at rather
    than the board that happened to be on the desk when they were written."""
    global GEOMETRY, BASELINE_EW_M, BASELINE_NS_M, MAX_DELAY_EW, MAX_DELAY_NS
    GEOMETRY = _G.load(name)
    BASELINE_EW_M = GEOMETRY.baseline_mm("M1", "M2") * 1e-3
    BASELINE_NS_M = GEOMETRY.baseline_mm("M3", "M4") * 1e-3
    MAX_DELAY_EW = GEOMETRY.max_delay_samples("M1", "M2")
    MAX_DELAY_NS = GEOMETRY.max_delay_samples("M3", "M4")
    return GEOMETRY

CH_NAMES = ("M1_WEST", "M2_EAST", "M3_NORTH", "M4_SOUTH")

# The first ~500 ms of INMP441 audio is a known start-up transient (measured in
# the port: the first meter block reads full-scale before settling). Every
# analysis discards at least a second.
SETTLE_S = 1.0


# ---------------------------------------------------------------------------
# primitives
# ---------------------------------------------------------------------------
def envelope(x, fs=FS, smooth_ms=1.0):
    """Rectified, smoothed amplitude envelope."""
    x = np.asarray(x, float)
    n = max(1, int(round(smooth_ms * 1e-3 * fs)))
    k = np.ones(n) / n
    return np.convolve(np.abs(x - x.mean()), k, mode="same")


def mad(x):
    """Median absolute deviation - robust to the very spike we are hunting."""
    x = np.asarray(x, float)
    return float(np.median(np.abs(x - np.median(x))))


def xcorr_delay(a, b, upsample=16):
    """Delay of `b` relative to `a`, in samples. Positive means b LAGS a.

    FFT cross-correlation, zero-padded in the frequency domain (which is exact
    sinc interpolation of the correlation, not a resampling approximation),
    then a parabolic refinement of the interpolated peak. The zero-padding is
    what makes the parabolic step accurate: on a raw correlation the peak
    curvature is too coarse and the fit biases toward integers.
    """
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    a = a - a.mean()
    b = b - b.mean()
    n = len(a)
    nfft = int(2 ** np.ceil(np.log2(2 * n)))
    A = np.fft.rfft(a, nfft)
    B = np.fft.rfft(b, nfft)
    X = np.conj(A) * B

    m = nfft * upsample
    Xp = np.zeros(m // 2 + 1, dtype=complex)
    Xp[:X.size] = X
    r = np.fft.irfft(Xp, m)

    half = m // 2
    rr = np.concatenate([r[half:], r[:half]])          # lags -half..half-1
    lags = (np.arange(m) - half) / upsample

    i = int(np.argmax(rr))
    d = 0.0
    if 0 < i < len(rr) - 1:
        y0, y1, y2 = rr[i - 1], rr[i], rr[i + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            d = 0.5 * (y0 - y2) / den
    return float(lags[i] + d / upsample)


# ---------------------------------------------------------------------------
# tap attribution (channel <-> microphone mapping)
# ---------------------------------------------------------------------------
def detect_tap(chans, fs=FS, peak_over_mad=8.0, peak_over_others=2.0):
    """Which channel was tapped in this window?

    A tap qualifies when the channel's envelope peak exceeds BOTH
      * `peak_over_mad` x its own baseline MAD  (it is a real event), and
      * `peak_over_others` x every other channel's peak (it is THIS mic).

    Returns (index or None, detail dict). Ambiguity is reported as None rather
    than guessed: two mics tapped in one window must fail the mapping test, not
    silently attribute to the louder one.
    """
    chans = np.asarray(chans, float)
    envs = [envelope(c, fs) for c in chans]
    peaks = np.array([e.max() for e in envs])
    mads = np.array([mad(e) for e in envs])

    detail = {"peaks": peaks.tolist(), "mads": mads.tolist()}

    order = np.argsort(peaks)[::-1]
    best, second = int(order[0]), int(order[1])
    over_mad = peaks[best] / mads[best] if mads[best] > 0 else np.inf
    over_other = peaks[best] / peaks[second] if peaks[second] > 0 else np.inf
    detail["over_mad"] = float(over_mad)
    detail["over_other"] = float(over_other)

    if over_mad < peak_over_mad or over_other < peak_over_others:
        return None, detail
    return best, detail


def check_mapping(windows, fs=FS):
    """windows: list of 4 arrays shaped (4, n) - one capture window per mic,
    in the order they were tapped (M1, M2, M3, M4).

    PASS when every window attributes to its expected channel.
    """
    results = []
    ok = True
    for expected, w in enumerate(windows):
        got, detail = detect_tap(w, fs)
        hit = (got == expected)
        ok = ok and hit
        results.append({"expected": expected, "got": got, "pass": hit,
                        "expected_name": CH_NAMES[expected],
                        "got_name": CH_NAMES[got] if got is not None else None,
                        **detail})
    return ok, results


# ---------------------------------------------------------------------------
# clap timing
# ---------------------------------------------------------------------------
def find_transient(chans, fs=FS, skip_s=SETTLE_S):
    """Sample index of the sharpest common transient, from the summed
    envelope, ignoring the start-up settle."""
    chans = np.asarray(chans, float)
    skip = int(skip_s * fs)
    e = sum(envelope(c[skip:], fs) for c in chans)
    return int(np.argmax(e)) + skip


def channel_delays(chans, centre, fs=FS, win_ms=10.0, ref=0, upsample=16):
    """Sub-sample delay of every channel relative to `ref`, over a window
    centred on `centre`. Positive = that channel LAGS the reference."""
    chans = np.asarray(chans, float)
    half = int(round(win_ms * 1e-3 * fs / 2))
    a = max(0, centre - half)
    b = min(chans.shape[1], centre + half)
    seg = chans[:, a:b]
    return np.array([xcorr_delay(seg[ref], seg[c], upsample)
                     for c in range(seg.shape[0])])


def bus_offset(delays):
    """The inter-bus offset estimate: the COMMON delay of bus B {M3, M4}
    relative to bus A {M1, M2}.

    For an overhead clap the acoustic path difference is negligible (see
    overhead_clap_check), so whatever this returns is electrical/DMA - which is
    exactly the quantity decision D6 says to measure host-side rather than
    compile in."""
    d = np.asarray(delays, float)
    return float((d[2] + d[3]) / 2.0 - (d[0] + d[1]) / 2.0)


def overhead_clap_check(delays, tol_same_bus=0.3):
    """A clap ~1 m DIRECTLY ABOVE the array centre.

    Overhead means every capsule is equidistant to within about 0.02 samples,
    so any measured within-bus difference is electrical, and the between-bus
    common part is the bus offset. PASS requires both same-bus pairs to agree.
    """
    d = np.asarray(delays, float)
    ew = abs(d[1] - d[0])
    ns = abs(d[3] - d[2])
    off = bus_offset(d)
    return {
        "pass": bool(ew <= tol_same_bus and ns <= tol_same_bus),
        "ew_delta": float(ew), "ns_delta": float(ns),
        "tol": tol_same_bus, "bus_offset_samples": off,
    }


def north_clap_check(delays, offset=0.0, expect=None, tol=0.5,
                     tol_ew=0.5):
    """A clap from ~2 m due NORTH, at board height, AFTER removing the
    measured bus offset.

    +y = North = M3, so a wavefront arriving from the north reaches M3 FIRST
    and M4 last: M4 must LAG M3 by the full N-S baseline, 2.84 samples. M1 and
    M2 lie on the E-W axis, perpendicular to the arrival, so they should agree.

    This is the end-to-end validation of the axis and sign conventions - if the
    sign comes out inverted, the array's idea of North is mirrored.
    """
    # `expect` defaults to the ACTIVE geometry, read at call time rather than
    # bound at import time: use_geometry() must be able to change it.
    if expect is None:
        expect = MAX_DELAY_NS
    d = np.asarray(delays, float).copy()
    d[2] -= offset
    d[3] -= offset
    ns = d[3] - d[2]                      # positive => M4 lags M3 => from North
    ew = abs(d[1] - d[0])
    return {
        "pass": bool(abs(ns - expect) <= tol and ew <= tol_ew),
        "ns_delta": float(ns), "expected": float(expect), "tol": tol,
        "ew_delta": float(ew), "tol_ew": tol_ew,
        "sign_ok": bool(ns > 0),
    }


def offset_determinism(offsets, tol=0.25):
    """Across N captures with a board reset between each, the bus offset must
    be the SAME. A constant offset is fine (D6 compensates host-side); a
    varying one means the two RX engines can start on different WS edges, which
    is a planning-chat finding, not a bench fix."""
    o = np.asarray(offsets, float)
    spread = float(o.max() - o.min()) if o.size else 0.0
    return {"pass": bool(spread <= tol), "spread": spread, "tol": tol,
            "offsets": o.tolist(), "mean": float(o.mean()) if o.size else 0.0}


# ---------------------------------------------------------------------------
# capture assembly
# ---------------------------------------------------------------------------
def assemble_pcm4(records, n_ch=4):
    """Turn PCM4 records into (4, n) plus an honest drop report.

    Only the LONGEST CONTIGUOUS run of sequence numbers is returned. Splicing
    across a gap would shift every later frame and read as an inter-channel
    delay - the exact error the sequence numbers exist to prevent.
    """
    recs = sorted((r for r in records if r.get("_kind") == "pcm4"),
                  key=lambda r: r["seq"])
    if not recs:
        return np.zeros((n_ch, 0), np.int16), {"n_records": 0, "dropped": 0,
                                               "drop_rate": 0.0, "runs": 0}
    runs, cur = [], [recs[0]]
    for prev, r in zip(recs, recs[1:]):
        if r["seq"] == prev["seq"] + 1:
            cur.append(r)
        else:
            runs.append(cur)
            cur = [r]
    runs.append(cur)
    best = max(runs, key=len)

    frames = np.concatenate([np.asarray(r["samples"], np.int16)
                             .reshape(-1, r["n_ch"]) for r in best], axis=0)
    expected = recs[-1]["seq"] - recs[0]["seq"] + 1
    dropped = expected - len(recs)
    return frames.T.copy(), {
        "n_records": len(recs), "expected": expected, "dropped": int(dropped),
        "drop_rate": float(dropped) / expected if expected else 0.0,
        "runs": len(runs), "used_frames": int(frames.shape[0]),
    }
