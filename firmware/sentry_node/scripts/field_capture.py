#!/usr/bin/env python
"""
field_capture.py - ONE stage, ONE command, and the samples always survive.

    conda activate acoustic-detector
    cd ~/acoustic-detector/firmware/sentry_node

    python scripts/field_capture.py --selftest
    python scripts/field_capture.py --stage quiet_baseline --seconds 600
    python scripts/field_capture.py --stage solo_m1_hover  --seconds 60
    python scripts/field_capture.py --stage quad_matched   --seconds 120
    python scripts/field_capture.py --list

WHAT IT IS FOR. A capture session is the part of a field day that cannot be
redone later: the rig is packed, the wind has changed, the battery is flat. So
this tool does exactly one thing and does it robustly - it archives the four
channels the pipeline consumed, with an honest account of what the link
dropped, and it says PASS or FAIL in one line.

THERE IS NO ANALYSIS IN THIS PATH. Not a spectrum, not a score, not a verdict.
A failed analysis must never be able to lose a capture, and an analysis that
runs while the operator is standing over a spinning rig is an analysis nobody
reads. Everything interpretive lives in field_verdict.py, which runs afterwards
over the whole directory and can be run again at midnight, and again next week
with constants that do not exist yet.

WHY `Z` AND NOT `G`. Both run the SAME loop - same front_end, same combiner,
same back_end, same threshold; the `armed` flag gates only the actuators and
the PCM stream. `Z` additionally emits the exact int16 samples, each block
sequence-numbered. Per-frame scores answer only "did it alert, at today's
constants". The samples answer every question anyone asks afterwards. A side
benefit: `Z` does not sound the buzzer, so an alert cannot contaminate its own
recording.

DROPS ARE EXPECTED, NOT EXCEPTIONAL. 4 ch x 16 kHz x 2 B = 128 kB/s over
USB-Serial-JTAG. Only the longest unbroken run of sequence numbers is kept -
splicing across a gap would shift every later sample and read as an
inter-channel delay, which is the precise error the sequence numbers exist to
prevent. The drop report is written INSIDE the npz, next to the audio, so no
later analysis can pick up the samples while leaving their provenance behind.
"""
import argparse
import hashlib
import json
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
sys.path.insert(0, str(HERE))

import numpy as np                                             # noqa: E402
from capture_trace import open_serial, resolve_port            # noqa: E402
from field_test import run_guard, save_raw, say_raw            # noqa: E402
import profile_store as PS                                     # noqa: E402
import stage_vocab as SV                                       # noqa: E402

FS = 16000
GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
BOLD = "\033[1m"
OFF = "\033[0m"

# The stages the field card names. Free-form names are still allowed - this
# list only warns about a typo, and NOTHING here may refuse a capture: a
# capture that does not happen cannot be re-taken once the rig is packed.
#
# It lives in `stage_vocab.py` because the same table decides what
# `field_verdict.py` prints and what `family_field_analysis.py`'s adoption gate
# may score, and a stage list is only ever edited when a new stage appears -
# which is the one moment three copies of it drift apart.
CANONICAL = SV.CANONICAL


def device_config(port):
    """The `I` banner, reduced to the lines that describe what the detector
    believes, plus a short hash of them. A capture whose configuration is
    unknown is a capture nobody can quote six weeks later."""
    try:
        with open_serial(port, timeout=0.2) as s:
            s.reset_input_buffer()
            s.write(b"I\n")
            s.flush()
            time.sleep(2.5)
            buf = s.read(300000)
    except Exception as e:                                     # noqa: BLE001
        return {"error": str(e)}
    txt = [m.group().decode("ascii", "replace").strip()
           for m in re.finditer(rb"[ -~]{6,}", buf)]
    keep = [l for l in txt
            if l.startswith(("cfg v", "tier2:", "tier3:", "golden_threshold",
                             "deployment_default"))
            or "tau2_milli" in l or "tau3_milli" in l or "fire=" in l]
    h = hashlib.sha256("\n".join(keep).encode()).hexdigest()[:12]
    return {"lines": keep, "hash": h}


def channel_rms(audio):
    return [float(np.sqrt(np.mean(audio[c].astype(np.float64) ** 2)))
            for c in range(audio.shape[0])]


def capture(port, sess, stage, seconds, note, quiet=False):
    print(f"\n{BOLD}  stage {stage}   {seconds} s{OFF}")
    fr, al, pcm, interrupted = run_guard(port, seconds, f"{stage[:8]:<8}",
                                         None, raw=True)
    name, rep = save_raw(sess, stage, pcm)
    if not quiet:
        say_raw(rep)

    got_s = rep.get("seconds", 0.0)
    drops = rep.get("dropped", 0)
    ok_dur = got_s >= 0.95 * seconds
    ok_drop = (drops == 0)

    audio = None
    if name:
        with np.load(sess / name) as z:
            audio = z["audio"]
    rms = channel_rms(audio) if audio is not None else []

    side = {
        "stage": stage, "requested_s": seconds, "captured_s": round(got_s, 2),
        "file": name, "dropped_blocks": drops,
        "drop_rate": rep.get("drop_rate", 0.0),
        "runs": rep.get("runs", 0), "interrupted": bool(interrupted),
        "channel_rms": [round(v, 1) for v in rms],
        "device_alerts": len(al), "frames": len(fr),
        "note": note or "",
        "t": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "config": globals().get("_CFG", {}),
    }
    (sess / f"{stage}_meta.json").write_text(json.dumps(side, indent=2))
    PS.manifest_append(sess, {"stage": stage, **side})

    ok = ok_dur and ok_drop and name is not None
    tag = f"{GREEN}PASS{OFF}" if ok else f"{RED}FAIL{OFF}"
    print(f"\n  {tag}  {stage}: {got_s:.1f}/{seconds} s, {drops} dropped "
          f"blocks, rms {['%.0f' % v for v in rms]}")
    if not ok_dur:
        print(f"  {YELLOW}short capture - re-run it, or note the duration on "
              f"the paper log{OFF}")
    if not ok_drop:
        print(f"  {YELLOW}the link dropped blocks; only the longest unbroken "
              f"run was kept. The audio is still valid, it is just shorter "
              f"than you asked for.{OFF}")
    if rms and (min(rms) < 5 or max(rms) / max(min(rms), 1e-9) > 30):
        print(f"  {RED}channel levels look wrong - a dead or saturated "
              f"microphone makes every later number meaningless{OFF}")
        ok = False
    return ok, side


def selftest(port, sess):
    """The thirty-second version of 'will the capture session work at all'.

    Five seconds, zero drops, four channels with sane and comparable levels.
    Run it before the rig comes out, not after."""
    print(f"\n{BOLD}  SELF-TEST - 5 s, four channels, zero drops{OFF}")
    ok, side = capture(port, sess, "selftest", 5, "capture-path self-test",
                       quiet=True)
    rms = side["channel_rms"]
    if len(rms) != 4:
        print(f"  {RED}expected 4 channels, got {len(rms)}{OFF}")
        return False
    print(f"  per-channel rms: {rms}")
    spread = max(rms) / max(min(rms), 1e-9)
    print(f"  channel spread {spread:.1f}x "
          f"({'ok' if spread <= 6 else 'HIGH - check the wiring'})")
    return ok and spread <= 6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default=None)
    ap.add_argument("--seconds", type=int, default=60)
    ap.add_argument("--note", default=None,
                    help="operator note, archived with the capture")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--list", action="store_true",
                    help="what today's session already holds")
    ap.add_argument("--port", default=None)
    ap.add_argument("--session", default=None,
                    help="override the session directory")
    a = ap.parse_args()

    sess = Path(a.session) if a.session else PS.session_dir("field1")
    sess.mkdir(parents=True, exist_ok=True)

    if a.list:
        print(f"\n  session {sess}")
        caps = sorted(sess.glob("*_raw.npz"))
        if not caps:
            print("    (no captures yet)")
            return 0
        for p in caps:
            meta = sess / (p.name[:-len("_raw.npz")] + "_meta.json")
            if meta.exists():
                m = json.loads(meta.read_text())
                print(f"    {m['stage']:18} {m['captured_s']:6.1f}s  "
                      f"drops {m['dropped_blocks']:3d}  "
                      f"rms {m['channel_rms']}  {m['note'][:40]}")
            else:
                print(f"    {p.name}")
        return 0

    port = a.port or resolve_port(None)
    if not port or "no ESP32" in str(port):
        print(f"{RED}  no ESP32-S3 serial port found{OFF}")
        return 2

    globals()["_CFG"] = device_config(port)
    print(f"  session {sess}")
    print(f"  device config {globals()['_CFG'].get('hash', '?')}")

    if a.selftest:
        return 0 if selftest(port, sess) else 1

    if not a.stage:
        ap.error("give --stage NAME (or --selftest / --list)")
    if a.stage not in CANONICAL:
        near = SV.nearest(a.stage)
        hint = f" - did you mean '{near[0]}'?" if near else ""
        print(f"  {YELLOW}note: '{a.stage}' is not one of the canonical "
              f"stage names on the field card{hint}{OFF}")
        if SV.section(a.stage) is None:
            print(f"  {RED}and no section of field_verdict.py will print it. "
                  f"The samples will be archived\n  and the stage will be "
                  f"named as unrecognised, not analysed. A canonical name "
                  f"costs\n  nothing now and cannot be applied once the rig "
                  f"is packed.{OFF}")

    note = a.note
    if note is None:
        try:
            note = input("  operator note (throttle, distance, wind, "
                         "anything odd): ").strip()
        except (EOFError, KeyboardInterrupt):
            note = ""

    ok, _ = capture(port, sess, a.stage, a.seconds, note)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
