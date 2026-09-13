"""
real_negatives.py - the library of drone-free recordings, as a first-class
corpus.

Every threshold was first chosen against `synth.py`, because that was the only
option. It is no longer: there are real recordings, indoor and outdoor, device
and phone, and a threshold that fires on any of them is wrong however good it
looks on the corpus.

A manifest, not a copy. The audio lives under captures/; duplicating it to
make a "corpus directory" would create a second copy that can drift from the
first. This file is the list, with the provenance and the reason each clip is
a negative. The recordings themselves are not in this repository, so tools
that read this list report the clips as missing until you add your own.

Provenance decides what each clip can be used for:

    device-quad   four INMP441 channels off the board. No AGC, no codec.
                  Every tier may be priced on it.
    phone-mono    AAC from a phone, with AGC. Ratio statistics across
                  frequency - v1's comb score, Tier-4's slow comb - survive
                  it. Envelope statistics do not, because AGC rewrites the
                  modulation depth Tier-3 measures.

`for_tier()` enforces that, so an envelope tier cannot be priced on phone
audio by forgetting rather than by deciding.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CAP = ROOT / "firmware/sentry_node/captures"

#: name, path, provenance, why it is a negative
CLIPS = [
    ("phone wind 1", "2026-08-21_phone/Wind_recording.wav", "phone-mono",
     "outdoor wind at the first outdoor test site, no rig running"),
    ("phone wind 2", "2026-08-21_phone/Wind_recording_2.wav", "phone-mono",
     "outdoor wind at the first outdoor test site, no rig running"),
    ("dev quiet lab", "2026-08-13_1445_ldtest/quiet_raw.npz", "device-quad",
     "the 08-13 lab room with nothing running. NOT silent: it carries a "
     "245 Hz comb with harmonics at 5x, 6x, 10x and 15x - HVAC or similar - "
     "which makes it the hardest negative the project owns for any comb tier"),
    ("dev quiet baseline",
     "2026-08-13_replay_selftest/quiet_baseline_raw.npz", "device-quad",
     "The same bytes as 'dev quiet lab' - sha256 d99989a0... Kept in the list "
     "because the entry was made in good faith and deleting it would hide "
     "the error rather than record it; excluded from every total by the "
     "content hash in for_tier(). See Duplicates below"),
    ("dev talk", "2026-08-13_1445_ldtest/talk_raw.npz", "device-quad",
     "human speech at conversational distance"),
    ("dev speech 2m", "2026-08-13_replay_selftest/speech_2m_raw.npz",
     "device-quad",
     "The same bytes as 'dev talk' - sha256 aad13f95... The library does not "
     "hold speech at two distances; it holds ONE speech recording, listed "
     "twice under two descriptions, and at most one of those descriptions "
     "can be true. Excluded from every total. See Duplicates below"),
    ("dev rotor 4m", "2026-08-13_1445_ldtest/rotor_4m_0_raw.npz",
     "device-quad",
     "A negative on purpose, for in-band tiers only. Measured against the "
     "same room's quiet baseline this capture is +0.3 dB at 250-500 Hz and "
     "+17.5 dB above 3.2 kHz: the rotor is where Tier-3 listens and is not "
     "where v1, Tier-2 and Tier-4 listen. There is no in-band comb in it to "
     "find, which is why the two comb tiers raise zero events on it and "
     "Tier-3 latches 94%. Scoring an in-band tier against it would be marking "
     "an exam that was never set"),
    ("dev rotor 10m", "2026-08-13_1445_ldtest/rotor_10m_1_raw.npz",
     "device-quad", "the same, at 10 m: +17.5 dB high band, nothing in band"),
    ("dev field-1 selftest", "2026-08-21_field1/selftest_raw.npz",
     "device-quad",
     "the only four-channel audio from the first outdoor test - 5 s of capture-path "
     "self-test. The day's scripted stages were not archived"),
]

# ---------------------------------------------------------------------------
# Duplicates
#
# `2026-08-13_replay_selftest/` is not a second session. It is a set of copies
# of `2026-08-13_1445_ldtest/`, renamed after the stages they were replayed
# as, and four of its files are byte-identical to their originals:
#
#     quiet_baseline_raw.npz == quiet_raw.npz        d99989a00594aa74...
#     speech_2m_raw.npz      == talk_raw.npz         aad13f95ae3945a8...
#     solo_m1_hover_raw.npz  == rotor_4m_0_raw.npz   0969fe880fd239bc...
#     solo_m2_hover_raw.npz  == rotor_10m_1_raw.npz  e13b76d13fb680b3...
#
# Two of those pairs are both in the list above, so the library counted 180 s
# of audio twice:
#
#     as listed   666.7 s = 0.185 h    ->  rule-of-three bound 16.2/h
#     distinct    486.7 s = 0.135 h    ->  rule-of-three bound 22.2/h
#
# The false-alarm bound every tier is measured against is therefore 22/h and
# not 16. Tier-4 needs 7.5 h at its 0.40/h allowance.
#
# The fix is a content hash and not a deleted line. for_tier() drops a clip
# whose bytes it has already returned, so nothing is scored twice; the entries
# stay, with the truth beside them, because a list that quietly loses an entry
# is how the double count happened.
# ---------------------------------------------------------------------------

#: clips that are negatives only for tiers listening in the comb band
IN_BAND_ONLY = {"dev rotor 4m", "dev rotor 10m"}

#: envelope tiers may not be priced on phone audio
ENVELOPE_TIERS = {"t3"}

#: Livestock clips, listed in a manifest file beside the audio.
#
# Why this is a file and not more entries in CLIPS: the list above is
# recordings written down one at a time, and it should stay that way. The
# livestock recordings arrive in batches from a phone and their metadata -
# which converter ran, what the source sha256 was, when it was ingested - is
# the sort of thing that belongs beside the audio rather than in a source file.
#
# It is still a human act. Nothing writes the manifest except an explicit
# ingest step naming the files. Nothing here scans a directory, and a missing
# manifest is simply an empty list, so this code path changes nothing until
# livestock has been recorded.
REGISTERED_MANIFEST = CAP / "livestock_real" / "manifest.json"


def registered():
    """Ingested clips, in the same 4-tuple shape as CLIPS.

    Returns [] when the manifest is absent or unreadable rather than raising,
    because every caller of for_tier() treats a missing recording as an absent
    one already (the `p.exists()` filter below) and a half-read manifest that
    silently drops clips is worse than none at all - so entries whose audio is
    gone are simply not returned, exactly like the hard-coded ones."""
    import json
    if not REGISTERED_MANIFEST.exists():
        return []
    try:
        doc = json.loads(REGISTERED_MANIFEST.read_text())
    except Exception:                                          # noqa: BLE001
        return []
    clips = doc.get("clips", []) if isinstance(doc, dict) else list(doc)
    out = []
    for c in clips:
        out.append((c.get("name", c.get("path", "?")), c["path"],
                    c.get("prov", "phone-mono"),
                    c.get("why", "registered by ingest_livestock.py"),
                    c.get("family")))
    return out


def for_tier(tier):
    """The negatives `tier` may legitimately be priced against.

    tier is one of 'v1', 't2', 't3', 't4'. The two rules are enforced here so
    that using the wrong clip is a decision someone has to make rather than
    something that happens by not thinking about it."""
    rows = [(n, r, p, w, None) for n, r, p, w in CLIPS] + registered()
    out = []
    seen = {}                       # sha256 -> the name that claimed it first
    for name, rel, prov, why, family in rows:
        if tier in ENVELOPE_TIERS:
            if prov == "phone-mono":
                continue                  # AGC rewrites modulation depth
            if name in IN_BAND_ONLY:
                continue                  # these DO contain a rotor for T3
        p = CAP / rel
        if not p.exists():
            continue
        # Same bytes, same evidence. Scoring one recording twice does not make
        # a threshold twice as well tested; it only inflates the hours. See
        # Duplicates above.
        import hashlib
        h = hashlib.sha256(p.read_bytes()).hexdigest()
        if h in seen:
            continue
        seen[h] = name
        out.append({"name": name, "path": p, "prov": prov, "why": why,
                    "family": family})
    return out


def load(entry):
    import numpy as np
    p = Path(entry["path"])
    if p.suffix == ".npz":
        d = np.load(str(p))
        return np.stack([c.astype(np.float32) / 32767.0 for c in d["audio"]])
    import soundfile as sf
    x, sr = sf.read(str(p), dtype="float32", always_2d=True)
    assert sr == 16000, f"{p}: {sr} Hz"
    return x.T


def seconds(entry):
    return load(entry).shape[1] / 16000.0


def summary():
    rows = []
    for tier in ("v1", "t2", "t3", "t4"):
        cl = for_tier(tier)
        rows.append((tier, len(cl), sum(seconds(c) for c in cl)))
    return rows


if __name__ == "__main__":
    print(f"{'tier':<6} {'clips':>6} {'seconds':>9} {'hours':>7}")
    for tier, n, s in summary():
        print(f"{tier:<6} {n:6d} {s:9.0f} {s / 3600:7.3f}")
    print()
    for name, rel, prov, why in CLIPS:
        ok = (CAP / rel).exists()
        print(f"  {'ok ' if ok else 'MISSING'} {name:<24} {prov:<12} {rel}")
    reg = registered()
    print()
    print(f"  registered by ingest_livestock.py: {len(reg)}"
          + ("" if reg else "  (none recorded yet)"))
    for name, rel, prov, why, family in reg:
        ok = (CAP / rel).exists()
        print(f"  {'ok ' if ok else 'MISSING'} {name:<24} {prov:<12} "
              f"family={family}  {rel}")
