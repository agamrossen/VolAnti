#!/usr/bin/env python
"""
field_day.py - ONE COMMAND, from a connected board to a written calibration
report. Staged, resumable, per-board, and it does its own measuring, judging
and bookkeeping.

    conda activate acoustic-detector
    cd ~/acoustic-detector/firmware/sentry_node

    python scripts/field_day.py                 # the full staged run
    python scripts/field_day.py --quick         # S0-S3 health check, ~2 min
    python scripts/field_day.py --from S5       # resume at a stage
    python scripts/field_day.py --only S9       # one stage
    python scripts/field_day.py --provision     # NEW BOARD: build+flash first
    python scripts/field_day.py --list          # past sessions
    python scripts/field_day.py --report        # regenerate the report

WHAT IT IS FOR. Every number this project has ever quoted came from synthetic
audio. This wizard exists to replace that with measurement: what the real rotor
sounds like, what the real ambient looks like, and what threshold those two
facts actually justify.

HOW IT TALKS TO YOU. One instruction at a time, in plain English, with a
countdown where waiting is involved. It asks you to confirm what you SAW rather
than inferring it. Every stage ends with PASS / FAIL / SKIPPED and where the
data went.

Calibration output is a PROFILE on this Mac (calibration/<mac>.json), never
flash - so recalibrating never needs a rebuild and never forces a golden
re-proof.
"""
import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import profile_store as PS                                      # noqa: E402
import quad_analysis as qa                                      # noqa: E402
from capture_trace import open_serial, resolve_port             # noqa: E402
from trace_proto import parse, parse_stream                     # noqa: E402

BOLD, DIM = "\033[1m", "\033[2m"
RED, GREEN, YELLOW, CYAN = "\033[31m", "\033[32m", "\033[33m", "\033[36m"
OFF = "\033[0m"

FS = 16000
STAGE_ORDER = [f"S{i}" for i in range(13)]


# ===========================================================================
# presentation
# ===========================================================================
def head(stage, title, why):
    print()
    print("=" * 72)
    print(f"  {BOLD}{stage} — {title}{OFF}")
    print(f"  {DIM}{why}{OFF}")
    print("=" * 72)


def verdict(ok, stage, msg, where=None, skipped=False):
    tag = (f"{YELLOW}SKIPPED{OFF}" if skipped
           else (f"{GREEN}PASS{OFF}" if ok else f"{RED}FAIL{OFF}"))
    print(f"\n  {stage}: {tag}  {msg}")
    if where:
        print(f"  {DIM}data -> {where}{OFF}")
    return ok


def diagnose(lines):
    """P7: bounded diagnosis. Two or three likely causes, each with the exact
    next check. Never an improvised fix."""
    print(f"\n{RED}  most likely causes, in order:{OFF}")
    for i, (cause, check) in enumerate(lines, 1):
        print(f"    {i}. {cause}")
        print(f"       {DIM}check: {check}{OFF}")
    print(f"{RED}  stopping this stage. Capture the evidence above before "
          f"changing anything.{OFF}")


def ask_yn(q, default=None):
    suffix = " [y/n]" if default is None else (" [Y/n]" if default else " [y/N]")
    while True:
        try:
            r = input(f"  {BOLD}{q}{suffix} {OFF}").strip().lower()
        except EOFError:
            return bool(default)
        if not r and default is not None:
            return default
        if r in ("y", "yes"):
            return True
        if r in ("n", "no"):
            return False


def ask(q, default=""):
    try:
        r = input(f"  {BOLD}{q}{OFF}" + (f" [{default}] " if default else " "))
    except EOFError:
        return default
    return r.strip() or default


def countdown(msg, seconds):
    end = time.time() + seconds
    while True:
        left = end - time.time()
        if left <= 0:
            break
        m, s = divmod(int(left + 0.5), 60)
        sys.stdout.write(f"\r  {BOLD}{msg}{OFF} — {m:d}:{s:02d} remaining   ")
        sys.stdout.flush()
        time.sleep(0.25)
    sys.stdout.write("\r" + " " * 78 + "\r")


# ===========================================================================
# device helpers - every open goes through the DTR/RTS-safe opener (P8)
# ===========================================================================
def talk(port, cmd, listen_s=6.0, until=None):
    """Send one command, collect records for listen_s (or until `until(rec)`)."""
    out = []
    with open_serial(port, timeout=0.2) as s:
        s.write((cmd + "\n").encode())
        s.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < listen_s:
            c = s.read(65536)
            if not c:
                continue
            buf += c
            recs, _, used = parse_stream(bytes(buf))
            for r in recs:
                out.append(r)
                if until and until(r):
                    return out
            buf = buf[used:]
        try:
            s.write(b"\n")
            s.flush()
            t1 = time.time()
            while time.time() - t1 < 2.0:
                if not s.read(4096):
                    break
        except Exception:
            pass
    return out


def identity(port, listen_s=6.0):
    """The `I` text block. Returned raw plus a parsed dict."""
    raw = b""
    with open_serial(port, timeout=0.2) as s:
        s.write(b"I\n")
        s.flush()
        t0 = time.time()
        while time.time() - t0 < listen_s:
            raw += s.read(4096)
            if b"modes:" in raw:
                break
    txt = raw.decode("utf-8", "replace")
    info = {"raw": txt}
    for line in txt.splitlines():
        if "free_heap=" in line:
            for tok in line.replace(",", " ").split():
                if "=" in tok:
                    k, _, v = tok.partition("=")
                    info[k] = v
    return info


def chip_id(port):
    """MAC + chip type, straight from esptool. Refuses politely if the chip is
    not an ESP32-S3 - a plain ESP32 is a different part and out of scope."""
    try:
        r = subprocess.run(["esptool", "--port", port, "--after", "no-reset",
                            "chip-id"], capture_output=True, text=True,
                           timeout=60)
        txt = r.stdout + r.stderr
    except Exception as e:                                     # noqa: BLE001
        return None, None, f"esptool did not run: {e}"
    mac = chip = None
    for line in txt.splitlines():
        if line.strip().startswith("MAC:"):
            mac = line.split(":", 1)[1].strip()
        if "Chip type:" in line:
            chip = line.split(":", 1)[1].strip()
    return mac, chip, txt


# ===========================================================================
# stages
# ===========================================================================
def s0_identify(ctx):
    head("S0", "Connect and identify",
         "Who is this board, and is it the chip we support?")
    port = resolve_port(ctx["args"].port)
    ctx["port"] = port
    print(f"  port {port}")

    mac, chip, _ = chip_id(port)
    if chip and "ESP32-S3" not in chip:
        print(f"\n{RED}  This is a {chip}, not an ESP32-S3.{OFF}")
        print("  Everything in this project - the pin map, the I2S topology, "
              "the\n  SIMD FFT alignment - assumes an ESP32-S3-DevKitC-1 "
              "N16R8.")
        print("  Bring this board to the planning chat rather than adapting "
              "the tooling to it.")
        return verdict(False, "S0", "unsupported chip")
    ctx["mac"] = mac or "unknown"
    ctx["chip"] = chip or "?"
    print(f"  chip {ctx['chip']}   MAC {ctx['mac']}")

    info = identity(port)
    if "SENTRY-NODE" not in info["raw"] and "i2s pins" not in info["raw"]:
        diagnose([
            ("The board enumerates but the application never answers - this is "
             "the known board-1 pattern (native-USB application path dead, "
             "UART bridge fine).",
             "watch the boot with `idf.py -p $PORT monitor`. If it stops right "
             "after `entry 0x...`, use the board's OTHER USB-C connector "
             "(UART), or use the spare board."),
            ("The board is sitting inside a streaming mode and never read the "
             "command.",
             "press RST, then re-run."),
            ("No firmware, or the wrong firmware, is flashed.",
             "run with --provision to build and flash first."),
        ])
        return verdict(False, "S0", "no identity block")

    ctx["profile"] = PS.load(ctx["mac"])
    ctx["session"] = PS.session_dir(ctx["mac"])
    (ctx["session"] / "identity.txt").write_text(info["raw"])
    print(f"  free heap {info.get('free_heap', '?')}  "
          f"largest {info.get('largest', '?')}")
    return verdict(True, "S0", f"board {ctx['mac']} identified",
                   ctx["session"])


def s1_golden(ctx):
    head("S1", "Golden gate",
         "Prove the sealed detector is untouched before trusting anything else.")
    sess = ctx["session"]
    outs = []
    for i in range(4):
        out = sess / f"golden_g{i}.bin"
        r = subprocess.run(
            ["eim", "run",
             f"python scripts/capture_trace.py --index {i} --out {out}",
             "v6.0.2"], capture_output=True, text=True, cwd=str(HERE.parent))
        ok = "sentinel=yes" in (r.stdout + r.stderr)
        print(f"  vector {i}: {'captured' if ok else 'FAILED'}")
        if not ok:
            diagnose([
                ("The board stopped answering mid-capture.",
                 "re-run S0; if it is silent, `idf.py monitor` and read the boot."),
                ("A previous streaming mode is still running.",
                 "press RST and re-run this stage."),
            ])
            return verdict(False, "S1", f"vector {i} did not capture")
        outs.append(str(out))
    r = subprocess.run([sys.executable, "scripts/compare_trace.py"] + outs,
                       capture_output=True, text=True, cwd=str(HERE.parent))
    txt = r.stdout + r.stderr
    (sess / "golden_compare.txt").write_text(txt)
    ok = "STAGE 1a GOLDEN VECTOR: PASS" in txt
    print(txt.strip().splitlines()[-6:] and "\n".join(txt.strip().splitlines()[-6:]))
    if not ok:
        diagnose([
            ("A firmware change reached the detector after all.",
             "`git diff master -- main/detector.c main/detector.h "
             "main/generated/` must be empty."),
            ("The flashed image is not this build.",
             "re-flash and re-run."),
        ])
    return verdict(ok, "S1", "all four vectors, zero decision mismatches"
                   if ok else "golden gate FAILED - stop here",
                   sess / "golden_compare.txt")


def s2_peripherals(ctx):
    head("S2", "Peripherals",
         "Buzzer, motor, LED colour order, the snooze button, the display.")
    port, prof = ctx["port"], ctx["profile"]
    results = {}

    print(f"\n  {BOLD}Buzzer.{OFF}")
    talk(port, "U b1", 2.0)
    time.sleep(0.6)
    talk(port, "U b0", 2.0)
    results["buzzer"] = ask_yn("Did you hear one clean beep?")

    print(f"\n  {BOLD}Vibration motor.{OFF}")
    talk(port, "U v1", 2.0)
    time.sleep(0.8)
    talk(port, "U v0", 2.0)
    results["motor"] = ask_yn("Did the motor buzz once and stop?")

    print(f"\n  {BOLD}LED wire order.{OFF} The firmware sends RAW channel "
          f"values;\n  WS2812 parts ship in GRB and RGB and this board's order "
          f"is an observation.")
    seen = []
    for trip in ((255, 0, 0), (0, 255, 0), (0, 0, 255)):
        talk(port, f"U l {trip[0]} {trip[1]} {trip[2]}", 2.0)
        seen.append(ask(f"channel {trip} looked like which colour?", "?"))
    talk(port, "U l 255 255 255", 2.0)
    results["led_white_ok"] = ask_yn("And is it white now?")
    talk(port, "U l 0 0 0", 2.0)
    prof["led_wire_order"] = {"255,0,0": seen[0], "0,255,0": seen[1],
                              "0,0,255": seen[2]}

    print(f"\n  {BOLD}Snooze button.{OFF} This failed last session; it is the "
          f"direct test of the fix.")
    before = talk(port, "U s", 4.0)
    lvl0 = next((r["button_level"] for r in before if r["_kind"] == "sta"), None)
    print("  Now PRESS AND HOLD the button.")
    input("  Press Enter once you are holding it down... ")
    held = talk(port, "U s", 4.0)
    lvl1 = next((r["button_level"] for r in held if r["_kind"] == "sta"), None)
    print(f"  released reads {lvl0}, held reads {lvl1}   "
          f"(1 = up, 0 = pressed)")
    results["button"] = (lvl0 == 1 and lvl1 == 0)
    if not results["button"]:
        diagnose([
            ("The button is not actually reaching GPIO21 / GND.",
             "one leg to GPIO21, the other to GND. No external resistor - the "
             "internal pull-up does the work."),
            ("The fix did not take (status still reports a hardcoded value).",
             "confirm this image is the one built after the fix: the `I` "
             "output should list `G[thr_milli]`."),
        ])

    print(f"\n  {BOLD}E-paper.{OFF} Calibration pattern first.")
    talk(port, "U e t", 20.0)
    print("  Expect: a rectangular border, four labelled corner ticks, a "
          "centre cross,\n  and a row of 10 px blocks along the bottom.")
    all_visible = ask_yn("Are ALL four corners and the whole border visible?")
    if not all_visible:
        rot = ask("Try a rotation 0-3 (blank to skip)", "")
        if rot.isdigit():
            talk(port, f"U e t {rot}", 20.0)
            all_visible = ask_yn("Better? All four corners visible now?")
            prof["epaper_rotation"] = int(rot)
    else:
        prof.setdefault("epaper_rotation", 0)
    print(f"  {YELLOW}Please photograph the TEST screen once - it is the only "
          f"record of the panel's real usable area.{OFF}")
    talk(port, "U e r", 20.0)
    results["epaper"] = ask_yn("Does the LISTENING screen show the "
                               "sound-wave icon with LISTENING under it?")
    talk(port, "U e a", 20.0)
    results["epaper"] &= ask_yn("And the ALERT screen: triangle with ALERT "
                                "under it, fully inside the panel?")

    prof["peripherals"] = results
    PS.save(prof)
    ok = all(v for k, v in results.items())
    return verdict(ok, "S2",
                   "everything confirmed by eye" if ok else
                   f"failed: {[k for k, v in results.items() if not v]}",
                   PS.profile_path(ctx["mac"]))


def s3_mics(ctx):
    head("S3", "Microphones alive",
         "Four channels, zero I2S timeouts. This is the direct test of the "
         "clock-routing fix.")
    port, prof = ctx["port"], ctx["profile"]
    print("  Reading the quad meter for ~10 s. Stay reasonably quiet.")
    recs = talk(port, "Q", 12.0)
    qmt = [r for r in recs if r["_kind"] == "qmt"]
    if not qmt:
        diagnose([
            ("Neither bus is clocking - the most likely single cause is the "
             "GPIO4/GPIO5 clock routing.",
             "confirm this image contains the fix (`I` lists `G[thr_milli]`), "
             "then DMM for SCK/WS at all four mic pads."),
            ("The quad module could not claim I2S0 because a mono mode is "
             "still running.",
             "press RST and re-run this stage."),
        ])
        return verdict(False, "S3", "no meter records at all")

    last = qmt[-1]
    rms = np.array(last["rms"], float)
    dc = np.array(last["dc"], float)
    to = list(last["timeouts"])
    print(f"\n  {'channel':<10}{'rms':>10}{'dc':>10}")
    for c in range(4):
        print(f"  {qa.CH_NAMES[c]:<10}{rms[c]:10.1f}{dc[c]:10.1f}")
    print(f"  bus timeouts A={to[0]} B={to[1]}   "
          f"short reads {list(last['short_reads'])}")

    med = float(np.median(rms))
    dead = [c for c in range(4) if rms[c] < 0.10 * med or med == 0]
    dc_bad = [c for c in range(4) if abs(dc[c]) > 1500]
    spread_db = (20 * np.log10(np.maximum(rms, 1e-9) / max(med, 1e-9))
                 if med > 0 else np.zeros(4))
    outliers = [c for c in range(4) if abs(spread_db[c]) > 6.0]

    ok = (sum(to) == 0) and not dead
    if outliers:
        print(f"{YELLOW}  advisory: channel(s) "
              f"{[qa.CH_NAMES[c] for c in outliers]} sit more than 6 dB from "
              f"the median. Reported, not failed - but a capsule that is "
              f"quietly half-dead halves the array gain and nothing else "
              f"would notice.{OFF}")
    if dc_bad:
        print(f"{YELLOW}  advisory: |DC| over 1500 counts on "
              f"{[qa.CH_NAMES[c] for c in dc_bad]}. DC is measured, never "
              f"removed - but it sits inside the tonality gate.{OFF}")
    if not ok:
        diagnose([
            ("A bus is not clocking (timeouts) or a capsule is dead.",
             "DMM: SCK and WS present at all four mic pads; SD-A continuity "
             "GPIO6->M1->M2, SD-B GPIO7->M3->M4."),
            ("L/R strapping wrong on one mic of a pair (both drive the same "
             "slot, or neither drives one).",
             "M1 and M3 L/R to GND; M2 and M4 L/R to 3V3."),
        ])

    prof["channel_baseline"] = {
        "rms": rms.tolist(), "dc": dc.tolist(),
        "timeouts": to, "measured": datetime.now().isoformat(timespec="seconds")}
    PS.save(prof)
    PS.manifest_append(ctx["session"], {"stage": "S3", "rms": rms.tolist(),
                                        "dc": dc.tolist(), "timeouts": to})
    return verdict(ok, "S3", "four channels alive, zero timeouts" if ok else
                   "microphone check failed", PS.profile_path(ctx["mac"]))


def _s4_one_clap(ctx, tag):
    """One overhead clap -> (delays, overhead_check) or (None, None)."""
    sess = ctx["session"]
    out = sess / f"s4_clap{tag}"
    print(f"  {BOLD}capturing 5 s{OFF}")
    subprocess.run([sys.executable, "scripts/quad_parity.py", "capture",
                    "--seconds", "5", "--out", str(out)],
                   cwd=str(HERE.parent))
    time.sleep(0.2)
    print(f"  >>> {BOLD}CLAP NOW{OFF} (if you have not already)")
    npz = Path(str(out) + ".npz")
    if not npz.exists():
        return None, None, out
    ch = np.load(npz)["channels"]
    if ch.shape[1] < FS:
        return None, None, out
    centre = qa.find_transient(ch, skip_s=0.5)
    d = qa.channel_delays(ch, centre, win_ms=10.0)
    return d, qa.overhead_clap_check(d), out


def s4_coherence(ctx):
    """Overhead clap, N times, WITH A RESET BETWEEN EACH.

    One clap measures the offset. Only repetition ACROSS RESETS decides whether
    it is a constant, and that distinction is the whole gate: compensating for
    an offset (CX-D) is sound if it comes back the same after a power cycle and
    is actively harmful if it does not. So the stage records both numbers and
    a three-valued verdict - true / false / unknown - and CX-D stays off the
    shipped path until it reads true.
    """
    n_runs = max(1, int(ctx.get("s4_runs", 5)))
    head("S4", f"Clap x{n_runs} WITH RESETS: mapping and inter-bus offset",
         "One clap from directly above makes every capsule equidistant, so any "
         "delay measured is electrical, not acoustic. Repeating it across "
         "resets is what turns an observation into a constant.")
    prof, sess = ctx["profile"], ctx["session"]
    geom = ctx.get("geometry") or qa.GEOMETRY.name
    print(f"  geometry profile: {BOLD}{geom}{OFF}")
    print("  When it says CLAP, make ONE sharp clap about 1 m DIRECTLY ABOVE "
          "the board centre.")

    offsets, rows, last_out = [], [], None
    for i in range(n_runs):
        if i:
            print(f"\n{YELLOW}  >>> PRESS THE RESET BUTTON ON THE BOARD NOW "
                  f"<<<{OFF}")
            print("      (the point of this run is the reset - without one, "
                  "run 2 measures\n       the same DMA start as run 1 and "
                  "proves nothing)")
        input(f"  run {i + 1}/{n_runs}: press Enter when ready... ")
        d, over, out = _s4_one_clap(ctx, f"_{i}" if n_runs > 1 else "")
        last_out = out
        if d is None:
            print(f"{YELLOW}  run {i + 1}: no usable capture, skipping{OFF}")
            continue
        print("  delays vs M1 (samples): " +
              "  ".join(f"{qa.CH_NAMES[c]} {d[c]:+.3f}" for c in range(4)))
        print(f"  same-bus deltas |M2-M1|={over['ew_delta']:.3f}  "
              f"|M4-M3|={over['ns_delta']:.3f}  (tol {over['tol']})")
        offset = over["bus_offset_samples"]
        print(f"  {BOLD}run {i + 1}: INTER-BUS OFFSET = {offset:+.3f} "
              f"samples{OFF}")
        offsets.append(float(offset))
        rows.append({"run": i + 1, "delays": d.tolist(),
                     "offset": float(offset), "pass": bool(over["pass"])})

    if not offsets:
        return verdict(False, "S4", "no usable captures")

    det = qa.offset_determinism(offsets)
    constant = det["pass"] if len(offsets) >= 2 else None
    mean = det["mean"]
    print(f"\n  {BOLD}offsets: " + " ".join(f"{o:+.3f}" for o in offsets)
          + f"   spread {det['spread']:.3f} (tol {det['tol']}){OFF}")
    if constant is True:
        print(f"{GREEN}  CONSTANT across {len(offsets)} resets: mean "
              f"{mean:+.3f} samples. CX-D may be given device time.{OFF}")
    elif constant is False:
        print(f"{YELLOW}  NOT CONSTANT across resets (spread "
              f"{det['spread']:.3f} samples). That is a FINDING for planning - "
              f"the two RX engines can start on different WS edges, and the "
              f"answer is hardware-level start sync, not a host-side "
              f"compensation. CX-D stays off.{OFF}")
    else:
        print(f"{YELLOW}  only one usable run: constancy UNKNOWN. CX-D stays "
              f"off.{OFF}")

    prev = prof.get("inter_bus_offset_samples")
    if prev is not None and abs(prev - mean) > 0.25:
        print(f"{YELLOW}  the offset moved since last session "
              f"({prev:+.3f} -> {mean:+.3f}). Report it.{OFF}")
    prof["inter_bus_offset_samples"] = float(mean)
    PS.set_bus_offset(prof, mean, constant, len(offsets),
                      spread=det["spread"], geometry=geom)
    PS.save(prof)
    PS.manifest_append(sess, {"stage": "S4", "runs": rows,
                              "offsets": offsets,
                              "offset": float(mean),
                              "offset_constant": constant,
                              "offset_spread": det["spread"],
                              "geometry": geom})
    return verdict(bool(offsets and all(r["pass"] for r in rows)), "S4",
                   f"offset {mean:+.3f} samples, constant="
                   f"{constant}, {len(offsets)} runs", last_out)


def s5_quad_parity(ctx):
    head("S5", "QUAD PARITY — the critical gate",
         "Four channels through C and through the Python reference, diffed "
         "decision for decision. Until this passes, no four-mic number means "
         "anything.")
    sess = ctx["session"]
    secs = ctx["args"].parity_seconds
    print(f"  {secs:.0f} s of room audio: quiet, then talk, then a few claps "
          f"(the Stage-1b recipe).")
    input("  Press Enter to start... ")
    out = sess / "s5_quad_parity"
    subprocess.run([sys.executable, "scripts/quad_parity.py", "capture",
                    "--seconds", str(int(secs)), "--out", str(out)],
                   cwd=str(HERE.parent))
    r = subprocess.run([sys.executable, "scripts/quad_parity.py", "analyse",
                        "--in", str(out)], cwd=str(HERE.parent),
                       capture_output=True, text=True)
    txt = r.stdout + r.stderr
    print(txt)
    (sess / "s5_parity_report.txt").write_text(txt)
    ok = "QUAD PARITY: PASS" in txt
    if not ok:
        diagnose([
            ("The link dropped records, so the sample-exact premise broke.",
             "the report prints the drop rate; re-run with a shorter "
             "--parity-seconds."),
            ("The C and Python four-channel paths genuinely disagree.",
             "the comparator names the first divergent frame and field. Take "
             "that to planning - do NOT adjust anything to make it pass."),
        ])
    ctx["parity_ok"] = ok
    return verdict(ok, "S5", "C and Python agree on every frame" if ok
                   else "quad path is NOT trustworthy yet",
                   sess / "s5_parity_report.txt")


def _guard_capture(ctx, seconds, label, note=None):
    """Run G for `seconds`, collecting the per-frame trace. Shared by S6-S10."""
    port, sess = ctx["port"], ctx["session"]
    thr = PS.threshold_for(ctx["profile"])
    milli = int(round(thr * 1000))
    recs = []
    with open_serial(port, timeout=0.2) as s:
        s.write(f"G {milli}\n".encode())
        s.flush()
        buf = bytearray()
        t0 = time.time()
        last_draw = 0.0
        alerts = 0
        while time.time() - t0 < seconds:
            c = s.read(65536)
            if c:
                buf += c
                rs, _, used = parse_stream(bytes(buf))
                for r in rs:
                    recs.append(r)
                    if r["_kind"] == "alt":
                        alerts += 1
                        print(f"\n  {RED}ALERT #{alerts}  f0={r['f0_hz']:.1f} Hz"
                              f"  score={r['score']:.3f}{OFF}")
                buf = buf[used:]
            left = seconds - (time.time() - t0)
            if time.time() - last_draw > 0.5:
                last_draw = time.time()
                m, sec = divmod(int(left + 0.5), 60)
                sc = [r["score"] for r in recs[-40:] if r["_kind"] == "rec"]
                cur = sc[-1] if sc else 0.0
                sys.stdout.write(f"\r  {BOLD}{label}{OFF} — {m}:{sec:02d} left"
                                 f"   score {cur:6.3f}   alerts {alerts}   ")
                sys.stdout.flush()
        try:
            s.write(b"\n")
            s.flush()
            t1 = time.time()
            while time.time() - t1 < 3.0:
                if not s.read(4096):
                    break
        except Exception:
            pass
    print()
    frames = [r for r in recs if r["_kind"] == "rec"]
    scores = np.array([r["score"] for r in frames], float)
    path = sess / f"{label}_trace.npz"
    np.savez_compressed(path, score=scores,
                        f0=np.array([r["f0_hz"] for r in frames]),
                        chain=np.array([r["chain"] for r in frames]),
                        fired=np.array([r["fired"] for r in frames]))
    PS.manifest_append(sess, {"stage": label, "seconds": seconds,
                              "frames": len(frames), "alerts": alerts,
                              "threshold": thr, "note": note,
                              "file": str(path.name)})
    return scores, alerts, path


def s6_quiet(ctx):
    head("S6", "Quiet baseline",
         "What does this place look like to the detector when nothing is "
         "happening?")
    if not ctx.get("parity_ok", True):
        return verdict(False, "S6", "blocked: S5 quad parity has not passed",
                       skipped=True)
    mins = ctx["args"].quiet_minutes
    env = ask("Environment in one word (indoors/outdoors)", "indoors")
    wind = ask("Wind 0-3 (0 still, 3 gusty)", "0")
    print(f"\n  {BOLD}STAY QUIET for {mins:.0f} minutes.{OFF} Leave the room "
          f"if you can.")
    input("  Press Enter to start... ")
    scores, alerts, path = _guard_capture(ctx, mins * 60, "S6_quiet",
                                          note=f"{env}, wind {wind}")
    if not len(scores):
        return verdict(False, "S6", "no frames captured")
    dur = len(scores) * 512 / FS
    print(f"\n  frames {len(scores)}  ({dur:.0f} s)")
    print(f"  score: median {np.median(scores):.3f}  p99 "
          f"{np.percentile(scores, 99):.3f}  max {scores.max():.3f}")
    print(f"  alerts at the running threshold: {alerts}")
    print(f"{DIM}  A {mins:.0f}-minute sample characterises the score "
          f"DISTRIBUTION. It cannot establish a false-alarm RATE - for that "
          f"use --soak <hours>.{OFF}")
    ctx.setdefault("captures", {})["quiet"] = str(path)
    ctx["profile"]["ambient"] = {
        "median": float(np.median(scores)), "p99": float(np.percentile(scores, 99)),
        "max": float(scores.max()), "seconds": dur, "alerts": alerts,
        "env": env, "wind": wind}
    PS.save(ctx["profile"])
    return verdict(True, "S6", f"ambient characterised over {dur:.0f} s", path)


def s7_activity(ctx):
    head("S7", "Activity", "Speech and movement must not fire the alarm.")
    print("  Talk normally and move around near the device for 2 minutes.")
    input("  Press Enter to start... ")
    scores, alerts, path = _guard_capture(ctx, 120, "S7_activity")
    if not len(scores):
        return verdict(False, "S7", "no frames captured")
    thr = PS.threshold_for(ctx["profile"])
    print(f"\n  max score {scores.max():.3f} vs threshold {thr:.3f}  "
          f"(margin {thr - scores.max():+.3f})   alerts {alerts}")
    print(f"{DIM}  Speech close to the array is CORRELATED across all four "
          f"capsules, so the summed path sees it strongly - which is exactly "
          f"why 'it did not fire' is worth recording.{OFF}")
    return verdict(alerts == 0, "S7",
                   "no alerts on speech" if alerts == 0
                   else f"{alerts} alert(s) on speech - record what you did",
                   path)


def s9_rotor(ctx):
    head("S9", "ROTOR CHARACTERISATION",
         "The first real propeller this project has ever heard. This measures "
         "the number the PROVISIONAL 200-800 Hz band has waited for.")
    print(safety_block())
    if not ask_yn("Safety block read and the rig is secured?", False):
        return verdict(False, "S9", "not confirmed safe", skipped=True)

    steps = []
    print(f"\n  {BOLD}The device is its own tachometer.{OFF} Each step: set the "
          f"throttle,\n  read the f0 the device reports, and hold it there. "
          f"That makes steps\n  reproducible across sessions far better than a "
          f"knob position.")
    n = int(ask("How many throttle steps?", "5") or 5)
    for i in range(n):
        print(f"\n  {BOLD}Step {i + 1}/{n}{OFF}")
        knob = ask("Set the throttle and note the knob position", f"step{i+1}")
        input("  Press Enter when the rotor is steady... ")
        scores, alerts, path = _guard_capture(ctx, 25, f"S9_step{i+1}",
                                              note=f"knob={knob}")
        npz = np.load(path)
        f0 = npz["f0"]
        sc = npz["score"]
        # the f0 the detector locked onto while it was scoring highest
        top = np.argsort(sc)[-max(1, len(sc) // 10):]
        f0_hi = float(np.median(f0[top])) if len(top) else float("nan")
        steps.append({"step": i + 1, "knob": knob, "f0_hz": f0_hi,
                      "score_max": float(sc.max()) if len(sc) else 0.0,
                      "alerts": alerts, "file": path.name})
        print(f"  step {i+1}: f0 ~ {f0_hi:.1f} Hz   max score "
              f"{sc.max() if len(sc) else 0:.3f}   alerts {alerts}")

    print(f"\n  {BOLD}throttle step -> f0 -> score{OFF}")
    for st in steps:
        print(f"    {st['step']}  knob {st['knob']:<10} f0 {st['f0_hz']:7.1f} Hz"
              f"   max score {st['score_max']:6.3f}   alerts {st['alerts']}")

    f0s = [s["f0_hz"] for s in steps if s["f0_hz"] == s["f0_hz"]]
    ctx["profile"]["rotor"] = {"steps": steps}
    PS.save(ctx["profile"])
    if f0s:
        lo, hi = min(f0s), max(f0s)
        print(f"\n  {BOLD}MEASURED f0 RANGE: {lo:.0f} - {hi:.0f} Hz{OFF}")
        print(f"  predicted hover for a 7in 3-blade: 375-475 Hz")
        print(f"  PROVISIONAL priority band: 200-800 Hz")
        inside = (lo >= 200 and hi <= 800)
        if inside:
            print(f"{GREEN}  The measured range sits INSIDE the priority band."
                  f"{OFF}")
        else:
            print(f"{YELLOW}  The measured range falls OUTSIDE the band. That "
                  f"is a Stage-0-reopening finding and it now has the "
                  f"RECORDED evidence the rules require. It goes to the "
                  f"planning chat - not into tonight's code.{OFF}")
        ctx["profile"]["rotor"]["f0_range_hz"] = [lo, hi]
        ctx["profile"]["rotor"]["inside_priority_band"] = bool(inside)
        PS.save(ctx["profile"])
    return verdict(True, "S9", "rotor characterised", ctx["session"])


def safety_block():
    return f"""
{RED}{BOLD}  ⚠ SAFETY — read every line before the rotor spins{OFF}
    * props-up always; NEVER hand-held while armed
    * rig staked or ballasted; stand clear of the prop disc
    * first spin of a session PROPLESS at servo-tester minimum, then fit props
      and verify rotation direction (props are handed; reversed props change
      the harmonic amplitudes you are trying to measure)
    * BEC red-pin discipline when daisy-chaining ESCs
    * LiPo work OUTDOORS only - no loose lithium in the York labs, ever
    * keep the USB tether and the Mac out of the prop plane
"""


# ===========================================================================
# driver
# ===========================================================================
STAGES = {
    "S0": s0_identify, "S1": s1_golden, "S2": s2_peripherals,
    "S3": s3_mics, "S4": s4_coherence, "S5": s5_quad_parity,
    "S6": s6_quiet, "S7": s7_activity, "S9": s9_rotor,
}
QUICK = ["S0", "S1", "S2", "S3"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--from", dest="start", default=None)
    ap.add_argument("--only", default=None)
    ap.add_argument("--provision", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--parity-seconds", type=float, default=60.0)
    ap.add_argument("--quiet-minutes", type=float, default=5.0)
    # ---- the port ----------------------------------------
    ap.add_argument("--s4-runs", type=int, default=5,
                    help="overhead claps in S4, WITH A RESET "
                         "between each. Only repetition across "
                         "resets can decide whether the inter-bus "
                         "offset is a constant.")
    import geometry as _geom
    _geom.add_argument(ap)
    a = ap.parse_args()

    if a.list:
        if not PS.CAP_DIR.exists():
            print("no sessions yet")
            return 0
        for d in sorted(PS.CAP_DIR.iterdir()):
            if d.is_dir():
                n = len(PS.manifest_read(d))
                print(f"  {d.name}   {n} entries")
        return 0

    print(f"\n{BOLD}  SENTRY-NODE — FIELD DAY{OFF}")
    print(f"  {DIM}one command, staged. Every instruction in plain English; "
          f"every measurement archived.{OFF}")

    # GEOMETRY IS DATA: every bench expectation this run checks is
    # derived from the named profile, so PCB day is this flag.
    qa.use_geometry(a.geometry)
    print(f"  {DIM}geometry: {a.geometry} — "
          f"E-W {qa.MAX_DELAY_EW:.2f} / N-S {qa.MAX_DELAY_NS:.2f} "
          f"samples{OFF}")
    ctx = {"args": a, "captures": {},
           "s4_runs": a.s4_runs, "geometry": a.geometry}
    order = ([a.only] if a.only else
             (QUICK if a.quick else
              [s for s in STAGE_ORDER if s in STAGES]))
    if a.start:
        order = [s for s in order if STAGE_ORDER.index(s) >=
                 STAGE_ORDER.index(a.start)]
    # S0 always runs: everything downstream needs the port and the profile.
    if order and order[0] != "S0":
        order = ["S0"] + order

    results = {}
    for st in order:
        fn = STAGES.get(st)
        if not fn:
            continue
        try:
            ok = fn(ctx)
        except KeyboardInterrupt:
            print("\n  interrupted")
            break
        except Exception as e:                                  # noqa: BLE001
            print(f"{RED}  {st} raised {type(e).__name__}: {e}{OFF}")
            ok = False
        results[st] = ok
        if not ok and st in ("S0", "S1"):
            print(f"\n{RED}  {st} is a hard gate. Stopping.{OFF}")
            break

    print(f"\n{BOLD}  SUMMARY{OFF}")
    for st, ok in results.items():
        print(f"    {st}  {'PASS' if ok else 'FAIL'}")
    if ctx.get("session"):
        print(f"\n  session archive: {ctx['session']}")
        print(f"  profile:         {PS.profile_path(ctx.get('mac', '?'))}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
