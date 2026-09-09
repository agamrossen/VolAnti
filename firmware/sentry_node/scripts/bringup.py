#!/usr/bin/env python
"""
bringup.py - ONE guided run, five steps, plain English verdicts.

    conda activate acoustic-detector
    python scripts/bringup.py

Run it that way, NOT via `conda run`: that wrapper gives the child no terminal
on stdin, so the "press Enter" prompts cannot work (they fall back to a timed
countdown) and the live level bar redraws badly through its capture pipe.

It walks the whole Stage-1b bring-up in order and stops at the first step that
fails, telling you what the failure MEANS and what to check. Nothing here is
new device behaviour: every step drives a mode the firmware already has
(I / M / T / L / Y). The firmware is untouched.

  STEP 1  LINK      does the board answer at all?              -> PASS / FAIL
  STEP 2  MIC       does the microphone hear the room?         -> PASS / FAIL
  STEP 3  TONE      does a known 440 Hz land in the right bin?  -> PASS / FAIL
  STEP 4  LIVE      the real detector on real air              -> observation
  STEP 5  PARITY    C and Python agree decision-for-decision   -> PASS / FAIL

Steps 1-3 answer "is it wired right". Step 5 is the actual Stage-1b acceptance
test. Step 4 is not pass/fail: it reports what the detector did, and an alert
there means the comb detector fired on room audio - it does NOT mean a drone.

Options:
    --from N            start at step N (1-5), e.g. after fixing wiring
    --only N            run just step N
    --live-seconds S    length of step 4   (default 60)
    --parity-seconds S  length of step 5   (default 60)
    --port /dev/...     skip port auto-detection
"""
import argparse
import sys
import time
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import serial                                              # noqa: E402
from capture_trace import open_serial, resolve_port        # noqa: E402
from trace_proto import parse_stream                       # noqa: E402

BIN_HZ = 16000.0 / 2048.0          # 7.8125 Hz per FFT bin
TONE_HZ = 440.0
TONE_BIN = round(TONE_HZ / BIN_HZ)  # 56
# Ignore everything below ~100 Hz when judging the tone. The mic's DC
# offset is never removed (by design - the Python reference has no such
# stage), and its skirt dominates the lowest bins. The detector's lowest
# comb tooth is bin 8 and its priority band starts at 200 Hz, so nothing
# down there bears on whether the plumbing is right.
LF_CUTOFF_BIN = 13                  # 101.6 Hz

# int16 full scale. The device reports level in int16 units, never dBFS - a
# unit conversion on the device is a place for a bug to hide.
FULL = 32768.0


# ---------------------------------------------------------------------------
# presentation
# ---------------------------------------------------------------------------
def hdr(n, title, subtitle):
    print()
    print("=" * 72)
    print(f"  STEP {n}  -  {title}")
    print(f"  {subtitle}")
    print("=" * 72)


def verdict(ok, name, why, fix=None):
    print()
    print("-" * 72)
    print(f"  {name}: {'PASS' if ok else 'FAIL'}")
    print(f"  {why}")
    if not ok and fix:
        print()
        print("  What to check:")
        for line in fix:
            print(f"    - {line}")
    print("-" * 72)
    return ok


def bar(value, vmax, width=44):
    n = 0 if vmax <= 0 else int(width * min(1.0, value / vmax))
    return "#" * n + "-" * (width - n)


def pause(msg, fallback=5):
    """Wait for Enter, but NEVER die if there is no terminal on stdin.

    `conda run` does not attach a tty, so input() raises EOFError immediately.
    A bring-up script that aborts because nobody could press a key is useless,
    so fall back to a visible countdown and carry on."""
    try:
        input(f"  {msg} ")
        return
    except (EOFError, KeyboardInterrupt) as e:
        if isinstance(e, KeyboardInterrupt):
            raise
    print(f"  {msg}")
    print(f"  (no keyboard on stdin - starting automatically in {fallback} s;"
          f" run without `conda run` for the interactive version)")
    for k in range(fallback, 0, -1):
        sys.stdout.write(f"\r  starting in {k}... ")
        sys.stdout.flush()
        time.sleep(1.0)
    print()


# ---------------------------------------------------------------------------
# serial helpers
# ---------------------------------------------------------------------------
def open_port(port):
    """One shared opener - see capture_trace.open_serial. Opening with
    pyserial's default DTR/RTS holds the S3 in reset and the board looks dead,
    which is exactly the symptom that stalled this bring-up."""
    return open_serial(port, timeout=0.2)


def stop_mode(s):
    """Any byte ends a streaming mode; drain to its sentinel so the port is
    left clean for the next step."""
    try:
        s.write(b"\n")
        s.flush()
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if not s.read(4096):
                break
    except Exception:
        pass


def stream(s, cmd, seconds, on_record):
    """Send cmd, feed every decoded record to on_record until the sentinel or
    the time limit. Returns True if a sentinel arrived."""
    buf = bytearray()
    s.write((cmd + "\n").encode())
    s.flush()
    t0 = time.time()
    done = False
    while time.time() - t0 < seconds and not done:
        c = s.read(65536)
        if not c:
            continue
        buf += c
        recs, _, used = parse_stream(bytes(buf))
        for r in recs:
            if r["_kind"] == "end":
                done = True
            on_record(r, time.time() - t0)
        buf = buf[used:]               # keep a record split across reads
    if not done:
        stop_mode(s)
    return done


# ---------------------------------------------------------------------------
# STEP 1 - link
# ---------------------------------------------------------------------------
def step_link(port):
    hdr(1, "LINK", "Is the ESP32-S3 running our firmware and answering?")
    with open_port(port) as s:
        s.write(b"I\n")
        s.flush()
        t0 = time.time()
        raw = b""
        while time.time() - t0 < 6.0:
            raw += s.read(4096)
            if b"modes:" in raw:
                break
    txt = raw.decode("utf-8", "replace").strip()
    if txt:
        print(txt)
    ok = "SENTRY-NODE" in txt or "i2s pins" in txt
    return verdict(
        ok, "STEP 1 LINK",
        "The board identified itself, so firmware is running and the USB link "
        "carries data both ways."
        if ok else
        f"The board sent {len(raw)} bytes and never identified itself.",
        fix=[
            "Is the cable in the connector marked USB (not UART)?",
            "Only one board plugged in? Two make the port ambiguous.",
            "Watch the boot: eim run \"idf.py -p PORT monitor\" v6.0.2 - you "
            "want bootloader lines then SENTRY-NODE STAGE1A READY.",
            "If the boot log stops right after 'entry 0x...', that board's "
            "native USB cannot carry app output (this is what killed board 1).",
        ])


# ---------------------------------------------------------------------------
# STEP 2 - microphone level
# ---------------------------------------------------------------------------
def step_mic(port, quiet_s=6.0, loud_s=14.0):
    hdr(2, "MICROPHONE",
        "Can it hear the room? Watch the bar - it must grow when you speak.")
    print()
    print("  Two phases, no typing needed once it starts:")
    print(f"    1. STAY QUIET      ({quiet_s:.0f} s)  - measures the noise floor")
    print(f"    2. TALK AND CLAP   ({loud_s:.0f} s)  - must visibly move the bar")
    print()
    pause("Press Enter when you are ready to be quiet...")
    print()

    quiet, loud = [], []
    state = {"timeouts": 0, "short": 0, "peak_hold": 0, "phase": "QUIET"}

    def on_rec(r, t):
        if r["_kind"] != "met":
            return
        rms, pk = r["rms"], r["peak_abs"]
        state["timeouts"] = max(state["timeouts"], r["timeouts"])
        state["short"] = max(state["short"], r["short_reads"])
        state["peak_hold"] = max(state["peak_hold"], pk)
        if t < quiet_s:
            state["phase"] = "QUIET   "
            quiet.append(r)
        else:
            state["phase"] = "GO! TALK"
            loud.append(r)
        db = -99.0 if rms <= 0 else 20 * __import__("math").log10(rms / FULL)
        left = max(0.0, (quiet_s if t < quiet_s else quiet_s + loud_s) - t)
        sys.stdout.write(
            f"\r  {state['phase']} {left:4.1f}s  |{bar(rms, 3000.0)}|  "
            f"rms {rms:8.1f} ({db:6.1f} dBFS)  peak {pk:6d}   ")
        sys.stdout.flush()

    with open_port(port) as s:
        stream(s, "M", quiet_s + loud_s + 2.0, on_rec)
    print()

    if not quiet and not loud:
        return verdict(
            False, "STEP 2 MIC",
            "The board sent no level readings at all.",
            fix=["Re-run step 1 - the link itself may be down.",
                 "The firmware exits meter mode immediately if the very first "
                 "I2S read times out, which looks exactly like this."])

    q_rms = sorted(r["rms"] for r in quiet)[len(quiet) // 2] if quiet else 0.0
    l_rms = max((r["rms"] for r in loud), default=0.0)
    dc = sorted(r["dc_offset"] for r in quiet)[len(quiet) // 2] if quiet else 0.0
    pegged = all(r["peak_abs"] >= 32700 for r in (quiet + loud))

    print(f"  quiet rms (median) {q_rms:.1f}")
    print(f"  loudest rms        {l_rms:.1f}")
    print(f"  peak seen          {state['peak_hold']}")
    print(f"  DC offset          {dc:+.1f}   <- RECORD THIS (runbook "
          f"deliverable; never removed, only measured)")
    print(f"  I2S timeouts {state['timeouts']}   short reads {state['short']}")

    if state["timeouts"] > 0:
        return verdict(False, "STEP 2 MIC",
                       "The I2S bus timed out: the microphone is not clocking.",
                       fix=["SCK -> GPIO 4, WS -> GPIO 5, SD -> GPIO 6.",
                            "BCLK and WS are easy to swap - check both.",
                            "VDD -> 3V3 and GND -> GND actually seated."])
    if l_rms == 0.0 and q_rms == 0.0:
        return verdict(False, "STEP 2 MIC",
                       "Perfect digital silence - no data is arriving.",
                       fix=["SD not connected to GPIO 6.",
                            "L/R must go to GND. On 3V3 the mic drives the "
                            "other frame slot and the firmware reads silence."])
    if pegged:
        return verdict(False, "STEP 2 MIC",
                       "Every sample is at full scale - the data pin is "
                       "floating, not driven.",
                       fix=["SD is not actually connected to GPIO 6.",
                            "This is exactly the no-mic-attached signature."])
    if l_rms < 2.5 * max(q_rms, 1.0):
        return verdict(False, "STEP 2 MIC",
                       f"The level barely moved ({q_rms:.0f} -> {l_rms:.0f}). "
                       f"Data arrives but does not track the room.",
                       fix=["Did you actually speak/clap close to the mic?",
                            "L/R strapping: on 3V3 you can get a near-constant "
                            "value instead of true silence.",
                            "Check the mic port hole is not covered."])
    return verdict(True, "STEP 2 MIC",
                   f"The level tracked the room: {q_rms:.0f} quiet -> "
                   f"{l_rms:.0f} loud ({l_rms / max(q_rms, 1.0):.1f}x). "
                   f"The microphone is wired correctly and hearing.")


# ---------------------------------------------------------------------------
# STEP 3 - tone
# ---------------------------------------------------------------------------
def step_tone(port):
    hdr(3, "TONE",
        f"Does a known {TONE_HZ:.0f} Hz tone land in the right FFT bin?")
    print()
    print("  PLUMBING CHECK ONLY. It proves the samples arrive right way up "
          "and at")
    print("  the right rate. It says NOTHING about range, sensitivity, or "
          "drones.")
    print()
    print(f"  Play a {TONE_HZ:.0f} Hz sine from your phone, held 10-20 cm from "
          f"the mic.")
    print("  (Search '440 Hz tone' on YouTube, or any tone generator app.)")
    print()
    pause("Press Enter once the tone is playing...")
    print()

    got = {}

    def on_rec(r, t):
        if r["_kind"] == "ton":
            got.update(r)

    with open_port(port) as s:
        stream(s, "T 16", 20.0, on_rec)

    if not got:
        return verdict(False, "STEP 3 TONE", "No spectrum record came back.",
                       fix=["Re-run step 2 - the mic may have stopped."])

    raw_pk = got["peak_bin"]
    pairs = list(zip(got["top_bin"], got["top_mag"]))
    print(f"  averaged {got['n_avg']} frames, bin width {got['bin_hz']:.4f} Hz")
    print(f"  DC (bin 0) magnitude {got['dc_mag']:.4f}  "
          f"(measured, never removed)")
    print("  top bins:")
    for b, m in pairs:
        tag = "  <== loudest overall" if b == raw_pk else ""
        if b < LF_CUTOFF_BIN:
            tag += "   [low-frequency, ignored]"
        print(f"    bin {b:4d}  {b * BIN_HZ:8.1f} Hz   mag {m:10.4f}{tag}")

    # The peak that answers the question. Everything below LF_CUTOFF_BIN is
    # excluded on purpose: the INMP441's DC offset (measured in step 2, and
    # deliberately never removed anywhere in this project) smears across the
    # lowest bins and can dominate a quiet tone. The detector itself never
    # looks down there - its lowest comb tooth is bin 8 and the priority band
    # starts at 200 Hz - so a DC skirt says nothing about the plumbing.
    band = [(b, m) for b, m in pairs if b >= LF_CUTOFF_BIN]
    if not band:
        return verdict(False, "STEP 3 TONE",
                       "Every one of the strongest bins is below "
                       f"{LF_CUTOFF_BIN * BIN_HZ:.0f} Hz - the tone never "
                       f"made it into the top 8 at all.",
                       fix=["Play the tone louder, or hold it closer.",
                            "Confirm it really is 440 Hz."])
    pk, pk_mag = max(band, key=lambda x: x[1])
    print()
    print(f"  loudest bin above {LF_CUTOFF_BIN * BIN_HZ:.0f} Hz:  bin {pk} "
          f"= {pk * BIN_HZ:.1f} Hz   (expected bin {TONE_BIN} = "
          f"{TONE_BIN * BIN_HZ:.1f} Hz)")
    if raw_pk < LF_CUTOFF_BIN:
        print(f"  (bin {raw_pk} = {raw_pk * BIN_HZ:.1f} Hz was louder still, "
              f"but that is the mic's DC/rumble, not the tone.)")

    ok = abs(pk - TONE_BIN) <= 2
    half = abs(pk - 2 * TONE_BIN) <= 3
    return verdict(
        ok, "STEP 3 TONE",
        f"The tone landed in bin {pk}, where the arithmetic says it should. "
        f"Sample rate, windowing and FFT are all correct."
        if ok else
        (f"The peak is at bin {pk}, about double the expected {TONE_BIN} - "
         f"the sample rate is half what the firmware thinks."
         if half else
         f"The strongest in-band bin is {pk}, expected {TONE_BIN}."),
        fix=["Was the tone actually playing, and loud enough?",
             "Is it really 440 Hz? Some apps default elsewhere.",
             "A flat spectrum with no clear peak means the mic is not being "
             "heard - go back to step 2."])


# ---------------------------------------------------------------------------
# STEP 4 - live detector
# ---------------------------------------------------------------------------
def step_live(port, seconds):
    hdr(4, "LIVE DETECTOR",
        "The real algorithm on real air, at the deployment preset.")
    print()
    print("  Running HIGH_ALERT (threshold 1.70), the deployment default.")
    print("  An alert here means THE COMB DETECTOR FIRED ON ROOM AUDIO.")
    print("  It does NOT mean a drone. Note what caused any alert.")
    print()
    print(f"  {seconds:.0f} s. Talking and clapping are broadband with no comb,")
    print("  so they should NOT build a 6-frame chain. If they do, that is a")
    print("  finding worth writing down - not a threshold to tweak.")
    print()
    pause("Press Enter to start...")
    print()

    st = {"n": 0, "alerts": [], "peak": 0.0, "chain": 0, "last": 0.0}

    def on_rec(r, t):
        if r["_kind"] == "rec":
            st["n"] += 1
            st["peak"] = max(st["peak"], r["score"])
            st["chain"] = max(st["chain"], r["chain"])
            if t - st["last"] > 0.25:
                st["last"] = t
                sys.stdout.write(
                    f"\r  {seconds - t:5.1f}s  |{bar(r['score'], 3.0, 30)}|  "
                    f"score {r['score']:7.4f}  f0 {r['f0_hz']:7.1f}Hz  "
                    f"chain {r['chain']:2d}  alerts {len(st['alerts'])}   ")
                sys.stdout.flush()
        elif r["_kind"] == "alt":
            st["alerts"].append(r)
            print(f"\n  *** ALERT #{len(st['alerts'])}  t={r['t_s']:.2f}s  "
                  f"f0={r['f0_hz']:.1f}Hz  score={r['score']:.4f}  "
                  f"chain={r['chain']} ***")

    with open_port(port) as s:
        stream(s, "L", seconds, on_rec)
    print()
    print(f"  frames {st['n']}   peak score {st['peak']:.4f}   "
          f"longest chain {st['chain']}   alerts {len(st['alerts'])}")
    print()
    print("-" * 72)
    print("  STEP 4 LIVE: OBSERVATION (not pass/fail)")
    if st["alerts"]:
        print(f"  {len(st['alerts'])} alert(s) on room audio. Write down what "
              f"you were doing.")
    else:
        print("  No alerts on room audio, which is what a quiet room should "
              "give.")
    print("-" * 72)
    return True


# ---------------------------------------------------------------------------
# STEP 5 - parity (the actual Stage-1b acceptance test)
# ---------------------------------------------------------------------------
def step_parity(port, seconds, out):
    hdr(5, "MIC PARITY",
        "THE acceptance test: same real air through C and through Python.")
    print()
    print("  The board streams the samples it fed the pipeline AND its own")
    print("  per-frame trace. The host re-runs src/detector.py on exactly")
    print("  those samples and diffs decision for decision.")
    print()
    print(f"  {seconds:.0f} s: quiet room, then talk at it, then a few claps.")
    print("  It also writes a .wav - LISTEN TO IT. It is the first real audio")
    print("  this project has ever had.")
    print()
    pause("Press Enter to start the capture...")
    print()

    import mic_parity                                      # noqa: E402
    rc = mic_parity.phase_capture(
        SimpleNamespace(port=port, out=str(out), seconds=float(seconds)))
    if rc != 0:
        return verdict(False, "STEP 5 PARITY", "The capture itself failed.",
                       fix=["A gap in the sample stream is reported rather "
                            "than silently patched - re-run it.",
                            "Re-run step 2 to confirm the mic is still alive."])
    print()
    rc = mic_parity.phase_analyse(SimpleNamespace(inp=str(out)))
    ok = (rc == 0)
    return verdict(
        ok, "STEP 5 PARITY",
        "The C firmware and the Python reference agreed on every frame. "
        "Bring-up is accepted for microphone 1."
        if ok else
        "C and Python disagreed. The report above names the first divergent "
        "frame and field.",
        fix=["Do NOT touch the threshold to make this pass.",
             "compare_trace's report names the exact frame and field.",
             f"Listen to {out}.wav - the audio itself may explain it."])


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--from", dest="start", type=int, default=1)
    ap.add_argument("--only", type=int, default=None)
    ap.add_argument("--live-seconds", type=float, default=60.0)
    ap.add_argument("--parity-seconds", type=float, default=60.0)
    ap.add_argument("--out", default="captures/mic/bringup")
    a = ap.parse_args()

    try:
        port = resolve_port(a.port)
    except SystemExit as e:
        print(f"\n  Cannot find the board: {e}")
        print("  Is it plugged into the connector marked USB, and only one "
              "board attached?")
        return 2

    print()
    print("#" * 72)
    print("#  SENTRY-Node  -  guided bring-up, microphone 1")
    print(f"#  port {port}")
    print("#  Pins: BCLK=GPIO4  WS=GPIO5  SD=GPIO6  L/R=GND  VDD=3V3")
    print("#" * 72)

    steps = [
        (1, lambda: step_link(port)),
        (2, lambda: step_mic(port)),
        (3, lambda: step_tone(port)),
        (4, lambda: step_live(port, a.live_seconds)),
        (5, lambda: step_parity(port, a.parity_seconds, Path(a.out))),
    ]
    if a.only:
        steps = [s for s in steps if s[0] == a.only]
    else:
        steps = [s for s in steps if s[0] >= a.start]

    results = {}
    for n, fn in steps:
        ok = fn()
        results[n] = ok
        if not ok:
            print()
            print("#" * 72)
            print(f"#  STOPPED AT STEP {n}. Fix the above, then resume with:")
            print(f"#    conda run -n acoustic-detector python "
                  f"scripts/bringup.py --from {n}")
            print("#" * 72)
            return 1

    print()
    print("#" * 72)
    print("#  ALL STEPS COMPLETE")
    names = {1: "LINK", 2: "MIC", 3: "TONE", 4: "LIVE", 5: "PARITY"}
    for n in sorted(results):
        tag = "OBSERVED" if n == 4 else ("PASS" if results[n] else "FAIL")
        print(f"#    step {n}  {names[n]:7s} {tag}")
    print("#")
    print("#  Recorded for the log: the DC offset from step 2, the quiet-room")
    print("#  rms, the tone bin, any step-4 alerts, and the parity result.")
    print("#" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
