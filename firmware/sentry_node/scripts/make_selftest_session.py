#!/usr/bin/env python
"""
make_selftest_session.py - rebuild the historical session field_verdict.py is
proven against.

    python scripts/make_selftest_session.py

WHY IT EXISTS. field_verdict.py is trusted because it reproduces results this
project already knows: the 32.8 dB comb null, the room's 309 Hz hum, the
W_any ~ 11 envelope null, and zero events on real speech from all three tiers.
That proof needs a session to run on, and the session is 60 MB of audio - too
big to check in, and derivable from archives that ARE checked in.

So the session is rebuilt rather than stored. The real captures come from
captures/2026-08-13_1445_ldtest, mapped onto the field stage names; the
four-motor pair is synthesised, and is marked SYNTHETIC everywhere it appears
because it is the wash model scoring its own generator - it proves the tooling
runs, not that the physics holds.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
sys.path.insert(0, str(PROJ.parent.parent / "src"))
import synth as S                                              # noqa: E402

FS = 16000
SRC = PROJ / "captures" / "2026-08-13_1445_ldtest"
OUT = PROJ / "captures" / "2026-08-13_replay_selftest"

# The captures, mapped onto the field stage names. The rotor was a
# SINGLE bench motor at ~82 Hz shaft, so it maps naturally onto a solo stage.
MAP = {"quiet": "quiet_baseline", "rotor_4m_0": "solo_m1_hover",
       "rotor_10m_1": "solo_m2_hover", "talk": "speech_2m"}


def write(stage, audio, note, rep=None):
    np.savez_compressed(OUT / f"{stage}_raw.npz", audio=audio,
                        fs=np.int32(FS),
                        report=np.array(json.dumps(rep or {})))
    (OUT / f"{stage}_meta.json").write_text(json.dumps(
        {"stage": stage, "requested_s": round(audio.shape[1] / FS),
         "captured_s": round(audio.shape[1] / FS, 2),
         "file": f"{stage}_raw.npz",
         "dropped_blocks": (rep or {}).get("dropped", 0),
         "channel_rms": [], "note": note,
         "t": "2026-08-13T14:45:00"}, indent=2))
    print(f"  {stage:16} {audio.shape}  {audio.shape[1] / FS:6.1f} s  {note}")


def main():
    if not SRC.exists():
        print(f"  missing {SRC} - the 2026-08-13 archives are not here")
        return 2
    OUT.mkdir(parents=True, exist_ok=True)
    for src, stage in MAP.items():
        p = SRC / f"{src}_raw.npz"
        if not p.exists():
            print(f"  missing {p.name}, skipping")
            continue
        with np.load(p) as z:
            audio = z["audio"]
            rep = json.loads(str(z["report"])) if "report" in z else {}
        write(stage, audio, f"2026-08-13 {src}, replayed as {stage}", rep)

    for stage, spread in (("quad_matched", 0.0), ("quad_spread", 1.5)):
        y = S.drone_quad_wash(dur=90.0, rpm=8500.0, blades=3,
                              spread_pct=spread, seed=3)
        q = np.clip(np.asarray(y) * 12000.0, -32767, 32767).astype(np.int16)
        write(stage, np.stack([q] * 4),
              f"SYNTHETIC four-motor, spread {spread}% - proves the tooling, "
              f"NOT the physics")
    print(f"\n  session: {OUT}")
    print("  now:  python scripts/field_verdict.py "
          "captures/2026-08-13_replay_selftest")
    return 0


if __name__ == "__main__":
    sys.exit(main())
