"""
homespeech.py - real recorded speech through the real detector.

Why this matters. A synthetic speech family was measured firing the sealed v1
detector 33 times an hour - enough, on its own, to push v1 past its own
false-alarm budget. That was a proxy, and the only real evidence was 60
seconds of talk that produced zero alerts, far too little to bound a rate.

This module closes that gap with whatever real speech is recorded. It is
deliberately simple: decode, run the reference, count events, and report the
confidence bound - including when the answer is "still not enough audio to
say".

The statistics. With zero events observed in T hours, the 95% one-sided upper
bound on a Poisson rate is 3/T ("rule of three"). Six minutes of silence
therefore bounds the rate at 30/h and nothing tighter. Reporting "zero false
alarms" without that bound would misrepresent the same data.

The caveat. A laptop microphone is not an INMP441 behind a cone. Absolute
levels, the noise floor and the high-frequency response all differ. What
transfers is the mechanism under test - voiced speech is a harmonic comb with
a fundamental of 110-230 Hz and ten to twenty live harmonics, which is the
same structure the detector hunts, and that is a property of human phonation
rather than of the microphone.
"""

import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

FS = 16000
AUDIO_SUFFIXES = (".wav", ".m4a", ".mp3", ".aif", ".aiff", ".caf", ".flac",
                  ".ogg")


def _decode_to_wav(src: Path, dst: Path, fs=FS):
    """Decode anything macOS can read into 16 kHz mono 16-bit WAV.

    A Voice Memo exported as ".wav" is very often an M4A/AAC container with the
    wrong extension - libsndfile refuses it, and the failure looks like a
    corrupt file rather than a container mismatch. So the extension is never
    trusted: if soundfile cannot open it, hand it to a system decoder.
    """
    for tool, args in (
        ("afconvert", ["-f", "WAVE", "-d", f"LEI16@{fs}", "-c", "1",
                       str(src), str(dst)]),
        ("ffmpeg", ["-v", "error", "-y", "-i", str(src), "-ac", "1",
                    "-ar", str(fs), str(dst)]),
    ):
        exe = shutil.which(tool) or (f"/usr/bin/{tool}"
                                     if Path(f"/usr/bin/{tool}").exists()
                                     else None)
        if not exe:
            continue
        r = subprocess.run([exe] + args, capture_output=True)
        if r.returncode == 0 and dst.exists() and dst.stat().st_size > 44:
            return tool
    return None


def load_audio(path: Path, fs=FS):
    """(mono float32 at fs, how it was decoded). Returns (None, reason) on
    failure rather than raising - one unreadable file must not abort a
    session's worth of evidence."""
    import soundfile as sf
    path = Path(path)
    try:
        x, sr = sf.read(str(path), dtype="float32", always_2d=True)
        x = x.mean(axis=1)
        if sr != fs:
            raise ValueError(f"sample rate {sr} != {fs}")
        return x, "soundfile"
    except Exception:
        pass
    tmp = Path(tempfile.mkdtemp()) / "decoded.wav"
    how = _decode_to_wav(path, tmp, fs)
    if how is None:
        return None, "no decoder could read it (tried soundfile, afconvert, ffmpeg)"
    try:
        x, sr = sf.read(str(tmp), dtype="float32", always_2d=True)
        return x.mean(axis=1), how
    except Exception as e:                                  # noqa: BLE001
        return None, f"{type(e).__name__}: {e}"


def rule_of_three_upper(n_events, hours):
    """95% one-sided upper bound on a Poisson rate.

    Exact for n = 0 (the rule of three); for n > 0 this uses the chi-square
    form, so the reported bound is correct either way.
    """
    if hours <= 0:
        return float("nan")
    if n_events == 0:
        return 3.0 / hours
    try:
        from scipy.stats import chi2
        return float(chi2.ppf(0.95, 2 * (n_events + 1)) / (2 * hours))
    except Exception:                                       # noqa: BLE001
        return (n_events + 2.0 * math.sqrt(n_events + 1)) / hours


def analyse_file(path, cfg=None, thr=None, t2cfg=None):
    """One recording through v1 AND Tier-2, using THE reference."""
    import detector as D
    import detector_t2 as T2
    import operating_point as op
    if cfg is None:
        cfg, thr_default = op.preset_config("HIGH_ALERT")
        thr = thr_default if thr is None else thr
    x, how = load_audio(path)
    if x is None:
        return {"file": Path(path).name, "error": how}

    # Mono is analysed as mono. The combiner is the identity at N=1, and
    # replicating one channel four times would only scale the spectrum by 4 -
    # which the per-bin adaptive floor absorbs completely, leaving every
    # whitened value and every decision identical. So there is nothing to gain
    # by faking an array, and a fake array would invite the reader to think
    # four microphones were involved.
    r = T2.analyze_quad_t2(x[None, :], cfg, t2cfg or T2.T2Config(),
                           thr_ref=thr)
    ev1, fr1 = D.track_frames(r, thr, cfg)
    ev2 = r["t2_events"]
    secs = len(x) / FS
    return {
        "file": Path(path).name, "decoded_by": how, "seconds": secs,
        "peak": float(np.abs(x).max()),
        "v1_events": len(ev1), "t2_events": len(ev2),
        "v1_event_times": [round(float(e["t_on"]), 2) for e in ev1[:20]],
        "v1_event_f0": [round(float(e["f0"]), 1) for e in ev1[:20]],
        "score_med": float(np.median(r["score"])),
        "score_p99": float(np.percentile(r["score"], 99)),
        "score_max": float(r["score"].max()),
        "frac_above_thr": float((r["score"] > thr).mean()),
        "longest_chain": int(fr1["chain"].max()) if len(fr1["chain"]) else 0,
        "t2_score_med": float(np.median(r["t2"]["score2"])),
        "t2_score_max": float(r["t2"]["score2"].max()),
        "trace": r, "thr": thr, "cfg": cfg,
    }


def threshold_margin(res, cfg, thrs=(1.70, 1.50, 1.30, 1.10, 0.90)):
    """How far the threshold would have to fall before this recording fires.

    Far more informative than a single count at the shipped threshold. "Zero
    events at 1.70" is consistent with being one bad frame away; "zero events
    and a longest chain of 4 at 1.10" is a measured margin.
    """
    import detector as D
    out = []
    for t in thrs:
        ev, fr = D.track_frames(res["trace"], float(t), cfg)
        out.append({"thr": float(t), "events": len(ev),
                    "longest_chain": int(fr["chain"].max())
                    if len(fr["chain"]) else 0})
    return out


def analyse_dir(d, cfg=None, thr=None, t2cfg=None):
    """Every audio file in a directory. Returns (per-file rows, pooled)."""
    import operating_point as op
    if cfg is None:
        cfg, thr = op.preset_config("HIGH_ALERT")
    d = Path(d)
    files = sorted(p for p in d.rglob("*")
                   if p.is_file() and p.suffix.lower() in AUDIO_SUFFIXES)
    rows, secs, n1, n2 = [], 0.0, 0, 0
    for p in files:
        r = analyse_file(p, cfg, thr, t2cfg)
        if "error" in r:
            rows.append(r)
            continue
        r["margin"] = threshold_margin(r, cfg)
        secs += r["seconds"]
        n1 += r["v1_events"]
        n2 += r["t2_events"]
        rows.append({k: v for k, v in r.items() if k not in ("trace", "cfg")})
    hours = secs / 3600.0
    pooled = {
        "files": len([r for r in rows if "error" not in r]),
        "seconds": secs, "hours": hours,
        "v1_events": n1, "t2_events": n2,
        "v1_per_hour": n1 / hours if hours else float("nan"),
        "t2_per_hour": n2 / hours if hours else float("nan"),
        "v1_upper_95_per_hour": rule_of_three_upper(n1, hours),
        "t2_upper_95_per_hour": rule_of_three_upper(n2, hours),
    }
    return rows, pooled
