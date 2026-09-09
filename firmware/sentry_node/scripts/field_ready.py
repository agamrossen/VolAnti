#!/usr/bin/env python
"""
field_ready.py - the seven checks that gate the field day, in one command.

    conda activate acoustic-detector
    python scripts/field_ready.py

Run it that way, NOT via `conda run`: that wrapper gives the child no terminal
on stdin, so the "press Enter" and "which was loudest" prompts cannot work.

What this is for. Six checks are owed before a field session, and six things
owed is six things to forget on a hillside at seven in the morning - so they
are one command, in dependency order, and the run stops at the first hard
failure. F7 is a safety gate.

  F1  GOLDEN     the sealed detector still reproduces          HARD GATE
  F2  CONFIG     which tier pair will actually run             HARD GATE
  F3  REAL TIME  p99 frame time inside the 32 ms hop           HARD GATE
  F4  BUTTON     tap = 10 s snooze, hold = OFF and standby     operator
  F5  OUTPUTS    buzzer ladder, then the three screens         operator
  F6  CHARGER    the whole point: no laptop, no data cable     operator

F1-F3 and F7 are machine-judged. F4-F6 need eyes and ears, so they ask you and
record what you say - an operator check that answers itself is not a check.

F7 INDUCES a real alert by dropping v1's threshold to 0.300 for its duration
and restoring it afterwards, printing the configuration before and after. That
is a bench instrument, not a tuning change, and it is the only way to verify by
TEST rather than by reading that the device does not detect its own alarm - the
vibration motor is an ERM at 100-200 Hz, inside Tier-3's firing band.

WHY F1 IS FIRST AND HARD. The firmware changed since the golden gate last
passed. Until it passes again, nothing else measured on this board means
anything, because the thing being measured might not be the sealed detector.

Options:
    --from N / --only N     N in 1..7
    --seconds S             length of F3 (default 60)
    --port /dev/...         skip auto-detection
    --no-save               do not write a session directory
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
sys.path.insert(0, str(HERE))

from bringup import hdr, open_port, pause, stop_mode, verdict   # noqa: E402
from capture_trace import resolve_port                          # noqa: E402
from trace_proto import parse_stream                            # noqa: E402
import profile_store as PS                                      # noqa: E402

HOP_S = 512 / 16000.0
BUDGET_MS = 32.0

GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
BOLD = "\033[1m"
OFF = "\033[0m"


def ask(question, options):
    """A question with a fixed answer set. Returns the chosen key, or None if
    there is no terminal - in which case the step is recorded as UNJUDGED
    rather than silently passing."""
    print(f"  {BOLD}{question}{OFF}")
    for k, text in options:
        print(f"    [{k}] {text}")
    try:
        while True:
            a = input("  > ").strip().lower()
            if a in dict(options):
                return a
            print("  (one of: " + ", ".join(k for k, _ in options) + ")")
    except (EOFError, KeyboardInterrupt):
        print(f"  {YELLOW}no keyboard on stdin - recorded as UNJUDGED{OFF}")
        return None


def text_of(buf):
    """The printable runs in a binary trace. The device mixes plain text
    (banners, errors, the heap line) into the record stream on purpose."""
    return [m.group().decode("ascii", "replace").strip()
            for m in re.finditer(rb"[ -~]{6,}", buf)]


def talk(s, cmd, seconds, drain=True):
    """Send one line, read for `seconds`, return the raw bytes."""
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    s.flush()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        c = s.read(65536)
        if c:
            buf += c
    if drain:
        stop_mode(s)
        buf += s.read(200000)
    return bytes(buf)


# ---------------------------------------------------------------------------
# F1 - the golden gate
# ---------------------------------------------------------------------------
def f1_golden(ctx):
    hdr(1, "GOLDEN GATE",
        "The firmware changed. Until this passes, nothing else means anything.")
    sess = ctx["session"]
    outs = []
    for i in range(4):
        out = (sess / f"golden_g{i}.bin") if sess else Path(f"/tmp/g{i}.bin")
        r = subprocess.run(
            [sys.executable, "scripts/capture_trace.py",
             "--index", str(i), "--out", str(out)],
            capture_output=True, text=True, cwd=str(PROJ))
        ok = "sentinel=yes" in (r.stdout + r.stderr)
        print(f"  vector {i}: {'captured' if ok else RED + 'FAILED' + OFF}")
        if not ok:
            return verdict(False, "F1", f"vector {i} did not capture", [
                "the board stopped answering - press RST and re-run",
                "a streaming mode is still running - send a newline first",
                "autostart: the guard is running. Any line takes the console, "
                "so this should not happen - if it does, `U c autostart 0`",
            ])
        outs.append(str(out))

    r = subprocess.run([sys.executable, "scripts/compare_trace.py"] + outs,
                       capture_output=True, text=True, cwd=str(PROJ))
    txt = r.stdout + r.stderr
    if sess:
        (sess / "golden_compare.txt").write_text(txt)
    ok = "STAGE 1a GOLDEN VECTOR: PASS" in txt
    print("\n".join(txt.strip().splitlines()[-6:]))

    # THE DRIFT LINES, SURFACED. compare_trace prints one per vector in the
    # middle of its report and only the last six lines reach the terminal, so
    # the number the morning is asked to write down was in the saved file and
    # nowhere a human would see it. The bound is inherited from the port and
    # has to be re-anchored on this image; a number nobody reads cannot
    # re-anchor anything.
    drift = [l.strip() for l in txt.splitlines() if "drift bound" in l]
    if drift:
        worst = max((float(l.split()[2]) for l in drift), default=0.0)
        print(f"  worst score drift {worst:.3e} over {len(drift)} vectors "
              f"(warn 1.3e-04 / fail 1.3e-03)")
        print("  WRITE IT DOWN: it re-anchors DRIFT_WARN in "
              "scripts/compare_trace.py")
        ctx["results"]["F1_drift"] = worst

    ctx["results"]["F1"] = {"pass": ok}
    return verdict(ok, "F1",
                   "four vectors, zero decision mismatches - the sealed "
                   "detector is intact"
                   if ok else "GOLDEN GATE FAILED - stop, do not field this",
                   [] if ok else [
                       # detector.c and detector.h are THAWED BY AUTHORISATION
                       # since the design (the family {2,3} rule), so a plain
                       # diff against the branch point is expected to be
                       # non-empty and is no longer the check. The certificate
                       # re-freezes them by sha256 - that is the check.
                       "`python scripts/certify.py` check 1: detector.c and "
                       "detector.h are thawed by authorisation and re-frozen "
                       "by hash. If it says DRIFT, one of them was edited "
                       "beyond what was authorised",
                       "`git diff stage1b-out-quad-bringup -- "
                       "main/generated/` must be empty - the sealed constants "
                       "have NO authorised exception",
                       "the family rule ships OFF. `U c` must show "
                       "trk_family=0; if it shows 1, that is the variant "
                       "running, not the sealed tracker",
                       "the flashed image may not be this build - re-flash",
                   ])


# ---------------------------------------------------------------------------
# F2 - which pair will actually run
# ---------------------------------------------------------------------------
def f2_config(ctx):
    hdr(2, "CONFIGURATION",
        "All three tiers now fit. Which run is a choice, and it is stored.")
    with open_port(ctx["port"]) as s:
        info = talk(s, "I", 2.5, drain=False)
    lines = text_of(info)
    cfg = next((l for l in lines if l.startswith("cfg v")), None)
    if not cfg:
        return verdict(False, "F2", "the board did not answer `I`",
                       ["press RST and re-run", "check the cable carries data"])
    print(f"  {cfg}")
    kv = dict(p.split("=", 1) for p in cfg.split() if "=" in p)
    t2 = kv.get("t2") == "1"
    t3 = kv.get("t3") == "1"
    t4 = kv.get("t4") == "1"
    auto = kv.get("autostart") == "1"

    # THE DROP RULE IS GONE, and this is the line that used to encode it:
    # "both tiers were asked for and only two fit". Three fit now - the memory
    # came from halving the I2S DMA and the frame time from moving Tier-3's
    # update off Tier-2's frames. NEITHER HAS BEEN Measured ON AIR, which is
    # what F3 is for, so this gate says what is configured and F3 says whether
    # it holds.
    tiers = ["v1"] + [n for n, on in (("Tier-2", t2), ("Tier-3", t3),
                                      ("Tier-4", t4)) if on]
    pair = " + ".join(tiers)
    note = ""
    if t2 and t3:
        note = (f"{YELLOW}Three tiers at once has never run on air. F3 is the "
                f"gate.{OFF}\n"
                f"  If F3 fails: U c t3 0  ->  v1 + Tier-2, the calibrated "
                f"pair. That is the pre-committed ladder.")
    if t4:
        note += (f"\n  {YELLOW}Tier-4 is ON. It is calibrated but NOT "
                 f"certified - zero events in 487 s bounds its false-alarm "
                 f"rate at 16/h against an allowance of 0.40/h. Its alerts "
                 f"are data, not detections.{OFF}")
    if len(tiers) == 1:
        note = f"{YELLOW}Every added tier is off; v1 alone.{OFF}"
    print(f"\n  {BOLD}It will run: {pair}{OFF}")
    if note:
        print(f"  {note}")
    if not auto:
        print(f"  {RED}autostart is OFF - it will NOT guard on power-up. "
              f"Fix with: U c autostart 1{OFF}")

    # the protocol CONFIGURATION PROOF. Quoted explicitly rather than left implicit in
    # a cfg line, because "anything persisted a human did not type is a bug"
    # is only checkable against a list somebody can read.
    def find(pat, default="?"):
        m = re.search(pat, "\n".join(lines))
        return m.group(1) if m else default

    proof = {
        "pair": pair,
        "v1 threshold": find(r"deployment_default=\w+ \(([\d.]+)\)"),
        "tau2 (dormant unless T2 on)": find(r"tau2_milli=(\d+)"),
        "tau3": find(r"tau3_milli=(\d+)"),
        "tau4 (dormant unless T4 on)": find(r"tau4_milli=(\d+)"),
        "Tier-4 grid": find(r"grid=(\d+-\d+)Hz"),
        "Tier-3 firing band": find(r"fire=(\d+-\d+) Hz"),
        "Tier-2 / priority band": find(r"band=(\d+-\d+) Hz"),
        "Tier-2 warm-up ms": find(r"warmup_ms=(\d+)"),
        "exclusion list entries": find(r"excl=(\d+)"),
        "display rotation": find(r"rot=(\d+deg\w*)"),
        "autostart": kv.get("autostart", "?"),
        "settings version": kv.get("cfg", "?") if "cfg" in kv else
                            (cfg.split()[1] if len(cfg.split()) > 1 else "?"),
    }
    print()
    for k, v in proof.items():
        print(f"    {k:32} {v}")
    excl_ok = proof["exclusion list entries"] in ("0", "?")
    if not excl_ok:
        print(f"  {RED}the exclusion list is NOT empty - nothing should be "
              f"persisted that a human did not type{OFF}")

    # ------------------------------------------------------------------
    # THE FIELD-DEFAULTS AUDIT.
    #
    # `cfg` is what this board is doing. `ship` is what a BLANK board would do.
    # The claim the field image makes is that those two are the same line, so
    # the honest way to check it is to diff them and name every field that
    # differs - each difference is one thing a human still has to type after an
    # erase-flash, and the count of them is the claim's score.
    #
    # IT REPORTS, IT DOES NOT FAIL. A difference is not automatically wrong: the
    # no-battery bench mode is `batt=ABSENT lora=0` and is a deliberate operator
    # statement. What must never happen is a difference nobody NOTICED, so the
    # gate's job here is to make it impossible to miss rather than to refuse.
    # ------------------------------------------------------------------
    ship = next((l for l in lines if l.startswith("ship v")), None)
    drift = []
    if ship:
        skv = dict(p.split("=", 1) for p in ship.split() if "=" in p)
        for k in sorted(set(kv) | set(skv)):
            if kv.get(k) != skv.get(k):
                drift.append((k, skv.get(k, "-"), kv.get(k, "-")))
        print(f"\n  {BOLD}field-defaults audit{OFF}   (ship = a blank board; "
              f"cfg = this board)")
        print(f"  {ship.strip()}")
        if not drift:
            print(f"  {BOLD}no drift: an erased board comes up exactly like "
                  f"this one. Nothing to type.{OFF}")
        else:
            print(f"  {YELLOW}{len(drift)} field(s) differ - each is one thing "
                  f"a human types after an erase-flash:{OFF}")
            for k, shipped, running in drift:
                print(f"    {k:14} ships {shipped:>12}   running {running}")
    else:
        print(f"\n  {YELLOW}this image predates the field-defaults table "
              f"(no `ship` line from `I`) - the audit cannot run{OFF}")

    ctx["results"]["F2"] = {"pass": auto and excl_ok, "cfg": cfg,
                            "ship": ship, "defaults_drift": drift,
                            "pair": pair, "proof": proof}
    return verdict(auto and excl_ok, "F2",
                   f"configuration reads back, autostart on -> {pair}"
                   if (auto and excl_ok) else
                   ("autostart is off; a charger would give you a console, "
                    "not a guard" if not auto else
                    "the exclusion list is not empty"))


# ---------------------------------------------------------------------------
# F3 - real time, on real air
# ---------------------------------------------------------------------------
def f3_realtime(ctx):
    hdr(3, "REAL TIME",
        f"Every frame's work must fit the {BUDGET_MS:.0f} ms hop, on real audio.")
    secs = ctx["seconds"]
    print(f"  Running the standalone guard for {secs} s. Stay quiet-ish; this\n"
          f"  measures TIME, not detection.\n")
    with open_port(ctx["port"]) as s:
        buf = talk(s, "S", secs + 2)

    for line in text_of(buf):
        if line.startswith(("SENTRY standalone", "heap ", "NOTE", "WARN",
                            "ERR")):
            print(f"  {line}")

    recs, _, _ = parse_stream(buf)
    fr = [r for r in recs if r["_kind"] == "rec"]
    t2 = [r for r in recs if r["_kind"] == "t2r"]
    t3 = [r for r in recs if r["_kind"] == "t3r"]
    if not fr:
        return verdict(False, "F3", "no frames arrived", [
            "the guard could not start - the ERR line above names the stage",
            "a microphone bus may be dead: watch for the FAST RED LED flash",
        ])

    us = sorted(r["us_frame"] for r in fr)
    p50 = us[len(us) // 2] / 1000.0
    p99 = us[min(len(us) - 1, int(0.99 * len(us)))] / 1000.0
    mx = us[-1] / 1000.0
    over = sum(1 for u in us if u > BUDGET_MS * 1000)
    print(f"\n  frames {len(fr)}   Tier-2 steps {len(t2)}   "
          f"Tier-3 updates {len(t3)}")
    print(f"  frame time: median {p50:.1f} ms   {BOLD}p99 {p99:.1f} ms{OFF}   "
          f"max {mx:.1f} ms   of {BUDGET_MS:.0f} ms")
    print(f"  over budget: {over} frames ({100.0 * over / len(us):.1f}%)")
    if t3:
        c = sorted(r["us_t3"] for r in t3)
        print(f"  Tier-3 cost: median {c[len(c)//2]/1000:.2f} ms per update")

    ok = p99 < BUDGET_MS and over == 0
    ctx["results"]["F3"] = {"pass": ok, "p50_ms": p50, "p99_ms": p99,
                            "max_ms": mx, "frames": len(fr), "over": over}
    return verdict(ok, "F3",
                   f"p99 {p99:.1f} ms of {BUDGET_MS:.0f} ms, no frame over "
                   f"budget"
                   if ok else f"p99 {p99:.1f} ms and {over} frames over the "
                              f"{BUDGET_MS:.0f} ms hop - it is NOT real time",
                   [] if ok else [
                       "both tiers may be running - F2 says which",
                       "reference numbers: v1+Tier-2 p99 29.5 ms, "
                       "v1+Tier-3 p99 30.9 ms",
                   ])


# ---------------------------------------------------------------------------
# F4 - the button
# ---------------------------------------------------------------------------
def f4_button(ctx):
    hdr(4, "BUTTON",
        "A tap is 10 s of silence. A 2 s hold is OFF, and then you may unplug.")
    print("  A pin that never reaches ground here is a WIRING fault, not a\n"
          "  firmware one. A healthy capture records presses,\n"
          "  so it works - this confirms it still does, and that the new\n"
          "  behaviours are the ones you asked for.\n")

    with open_port(ctx["port"]) as s:
        pause("Press Enter, then TAP the button once...")
        buf = talk(s, "S", 14)
        recs, _, _ = parse_stream(buf)
        sta = [r for r in recs if r["_kind"] == "sta"]
        snoozed = [r for r in sta if r.get("alert_name") == "snoozed"]
        presses = max((r["press_count"] for r in sta), default=0)
        print(f"\n  press_count {presses}   STAT records showing SNOOZED: "
              f"{len(snoozed)}")
        if snoozed:
            left = max(r["snooze_remaining_ms"] for r in snoozed) / 1000.0
            print(f"  longest snooze window seen: {left:.1f} s "
                  f"(expected about 10)")

    seen = ask("Did the LED go SOLID BLUE for about ten seconds?",
               [("y", "yes - solid blue, then back to the slow green flash"),
                ("n", "no")])
    off = ask("Now HOLD the button for 2 s. Did the panel draw a bar and OFF?",
              [("y", "yes - a bold horizontal bar and the word OFF"),
               ("n", "no")])
    if off == "y":
        print("  Tap it again to bring it back before the next step.")
        pause("Press Enter when it is guarding again...")

    ok = bool(snoozed) and seen == "y" and off == "y"
    ctx["results"]["F4"] = {"pass": ok, "presses": int(presses),
                            "snoozed_records": len(snoozed),
                            "blue": seen, "off_screen": off}
    return verdict(ok, "F4",
                   "tap snoozes for 10 s with a blue LED; a hold draws OFF"
                   if ok else "the button surface did not behave as specified",
                   [] if ok else [
                       "no SNOOZED record: the press never reached GPIO21 - "
                       "check the switch to ground, `U gs` watches the pin",
                       "blue but no OFF screen: the panel may be faulted - "
                       "`U s` reports epaper_state",
                   ])


# ---------------------------------------------------------------------------
# F5 - buzzer and screens
# ---------------------------------------------------------------------------
def f5_outputs(ctx):
    hdr(5, "OUTPUTS",
        "Maximum volume is a property of the transducer. Your ear decides.")
    print("  Four one-second bursts: solid DC, then square waves at 2.0, 2.7\n"
          "  and 4.0 kHz. An ACTIVE buzzer is loudest on DC; a PASSIVE one is\n"
          "  nearly silent on DC and loudest near its resonance.\n")
    with open_port(ctx["port"]) as s:
        pause("Press Enter to play the ladder...")
        buf = talk(s, "U bt", 9)
        for line in text_of(buf):
            if "buzzer" in line:
                print(f"  {line}")

        which = ask("Which was loudest?",
                    [("1", "1 - DC (solid)"), ("2", "2 - PWM 2.0 kHz"),
                     ("3", "3 - PWM 2.7 kHz"), ("4", "4 - PWM 4.0 kHz")])
        cmd = {"1": "U bd0", "2": "U bd1 2000", "3": "U bd1 2700",
               "4": "U bd1 4000"}.get(which)
        if cmd:
            out = talk(s, cmd, 2.0, drain=False)
            for line in text_of(out):
                if "buzzer drive" in line:
                    print(f"  {line}")

        print("\n  Now the three screens. Each takes about 2 s to render.")
        screens = []
        for letter, name in (("o", "OFF - a bold bar and the word OFF"),
                             ("w", "ALERT / TIER 3 WASH - triangle and text"),
                             ("r", "LISTENING - the sound-wave icon")):
            talk(s, f"U e{letter}", 4.0, drain=False)
            a = ask(f"Did it render {name}, fully inside the visible area?",
                    [("y", "yes"), ("n", "no - clipped, mirrored or blank")])
            screens.append((name.split(" -")[0], a))

    ok = which is not None and all(a == "y" for _, a in screens)
    ctx["results"]["F5"] = {"pass": ok, "loudest": which,
                            "screens": dict(screens)}
    return verdict(ok, "F5",
                   f"drive saved ({cmd}); all three screens render"
                   if ok else "an output did not behave",
                   [] if ok else [
                       "a clipped or mirrored screen is a ROTATION, not a "
                       "bug: `U c rot 1` (0-3) and re-check - no rebuild",
                       "if the buzzer is inaudible on every step, the fault "
                       "is the transducer or its supply, not the firmware",
                   ])


# ---------------------------------------------------------------------------
# F6 - the whole point
# ---------------------------------------------------------------------------
def f6_charger(ctx):
    hdr(6, "THE CHARGER",
        "No laptop, no data. This is the one the build exists for.")
    print("  With no host draining the USB link the trace WRITES ARE DROPPED\n"
          "  rather than blocked. If the loop ever stalls on a charger and not\n"
          "  on a laptop, that is where to look - but you will see it here.\n")
    print(f"  {BOLD}Unplug from the laptop. Plug into the power bank with a\n"
          f"  charge-only cable.{OFF}\n")
    a = ask("Within about 2 s, does the LED start its slow green flash?",
            [("y", "yes"), ("n", "no")])
    if a == "y":
        print("\n  Leave it running for ten minutes and come back.")
        b = ask("Still flashing green, and the panel still says LISTENING?",
                [("y", "yes - it survived"), ("n", "no - it stopped")])
    else:
        b = "n"
    ok = a == "y" and b == "y"
    ctx["results"]["F6"] = {"pass": ok, "started": a, "survived": b}
    return verdict(ok, "F6",
                   "it guards on charger power alone - the build does what it "
                   "was for"
                   if ok else "it did not run on charger power",
                   [] if ok else [
                       "no green flash at all: the cable may be charge-only "
                       "AND under-powered, or autostart is off (F2)",
                       "started then stopped: capture it on the laptop with "
                       "`python scripts/run_device.py --attach`",
                   ])


def f7_freeze(ctx):
    hdr(7, "SELF-INTERFERENCE FREEZE",
        "The device must not detect its own alarm.")
    print("  The vibration motor is an ERM at 100-200 Hz bolted to the same\n"
          "  board - INSIDE Tier-3's 100-320 Hz firing band - and the buzzer\n"
          "  drives at 2-4 kHz, inside its high band. The rule is that while\n"
          "  any output runs, plus a 500 ms tail, Tier-3's tracker FREEZES:\n"
          "  neither hit nor miss, so the chain survives intact.\n")

    # WHY THIS NEEDS SOUND, which cost a bench hour to learn.
    #
    # The first version of this check just dropped v1's threshold to 0.300 and
    # expected room noise to fire it. Measured zero alerts in 35 s
    # at 0.300 in a quiet room. The threshold is NOT what stops v1 firing on
    # noise - the tracker is. It needs six consecutive frames whose argmax
    # stays within 2% (or an octave), and it needs the jitter gate: the median
    # frame-to-frame move of the RAW argmax must be under 0.4%. Room noise hops
    # all over the grid, so no chain ever forms, at any threshold.
    #
    # That is a good property, and it means an alert has to be INDUCED
    # acoustically. hover_425Hz.wav is a steady in-band tone and does it.
    stim = PROJ / "captures" / "stimulus" / "hover_425Hz.wav"
    have_audio = stim.exists() and shutil.which("afplay") is not None
    if have_audio:
        a = ask("Play the 425 Hz hover stimulus through the speakers? "
                "It needs to be audible to the mics.",
                [("y", "yes - play it (about 20 s, moderate volume)"),
                 ("n", "no - I will whistle a steady note instead"),
                 ("s", "skip F7")])
    else:
        print(f"  {YELLOW}no stimulus clip or no afplay - you will need to "
              f"make the noise{OFF}")
        a = ask("Ready to whistle a steady note for ~15 s?",
                [("n", "yes, I will make the noise"), ("s", "skip F7")])
    if a == "s" or a is None:
        print(f"  {YELLOW}F7 skipped - the freeze is UNVERIFIED on this "
              f"hardware{OFF}")
        ctx["results"]["F7"] = {"pass": False, "skipped": True}
        return False

    with open_port(ctx["port"]) as s:
        # Settle first: if the guard is running, the teardown takes a moment
        # and a command sent into it can be missed. This cost a failed run.
        s.write(b"\n")
        s.flush()
        time.sleep(2.0)
        s.read(400000)

        before = talk(s, "U c", 2.5, drain=False)
        old = next((l for l in text_of(before) if l.startswith("cfg v")), "")
        print(f"  before: {old}")
        m = re.search(r"thr1=(\d+)", old)
        thr1_was = m.group(1) if m else "0"

        for cmd in ("U c t2 0", "U c t3 1", "U c thr1 900"):
            talk(s, cmd, 2.5, drain=False)
        print(f"  {YELLOW}v1 threshold temporarily 0.900 - a bench "
              f"instrument, restored below{OFF}")

        s.reset_input_buffer()
        s.write(b"S\n")
        s.flush()
        proc = None
        if a == "y":
            time.sleep(4.0)              # let the floor settle first
            proc = subprocess.Popen(
                ["afplay", str(stim)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            print("  WHISTLE NOW - a steady note, for about 15 s")
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 34:
            c = s.read(65536)
            if c:
                buf += c
        if proc:
            proc.terminate()
        stop_mode(s)
        buf += s.read(400000)

        # restore FIRST, before any analysis can throw
        talk(s, f"U c thr1 {thr1_was}", 2.5, drain=False)
        after = talk(s, "U c", 2.5, drain=False)
        print(f"  after:  "
              f"{next((l for l in text_of(after) if l.startswith('cfg v')), '')}")

    recs, _, _ = parse_stream(bytes(buf))
    t3 = [r for r in recs if r["_kind"] == "t3r"]
    alerts = [r for r in recs if r["_kind"] == "alt"]
    frozen = [r for r in t3 if r["frozen"]]
    print(f"\n  v1 alerts {len(alerts)}   Tier-3 updates {len(t3)}   "
          f"frozen updates {len(frozen)}")

    if not alerts:
        return verdict(False, "F7", "no alert fired, so nothing was induced", [
            "the stimulus was not loud enough at the microphones - move the "
            "board closer to the speakers and re-run `--only 7`",
            "v1 needs a STEADY tone: six consecutive frames within 2% and a "
            "raw-argmax jitter under 0.4%. Noise will not do it at any "
            "threshold - that is measured, not assumed",
        ])
    if not t3:
        return verdict(False, "F7", "Tier-3 produced no updates",
                       ["the pair may have come up as v1+Tier-2 - see F2"])

    hits_in = [r["hits"] for r in frozen]
    ok_freeze = len(frozen) > 0
    ok_hits = (not hits_in) or (min(hits_in) == max(hits_in))
    tail = t3[-3:] if len(t3) >= 3 else t3
    ok_release = not all(r["frozen"] for r in tail)
    print(f"  hits across the frozen updates: "
          f"{sorted(set(hits_in)) if hits_in else 'n/a'}   "
          f"still frozen at the end: {all(r['frozen'] for r in tail)}")

    ok = ok_freeze and ok_hits and ok_release
    ctx["results"]["F7"] = {"pass": ok, "alerts": len(alerts),
                            "t3_updates": len(t3), "frozen": len(frozen),
                            "hits_span": [min(hits_in), max(hits_in)]
                                         if hits_in else None}
    return verdict(ok, "F7",
                   f"{len(frozen)} updates frozen during the alert, the chain "
                   f"survived, and the freeze released"
                   if ok else "the freeze did not behave as specified",
                   [] if ok else [
                       "no frozen updates: the standalone path is not passing "
                       "alert_ui_outputs_active() into t3_push_block",
                       "hits collapsed: it is INHIBITING, not freezing - a "
                       "frozen update must be neither hit nor miss",
                       "still frozen at the end: the 500 ms tail is not "
                       "expiring, which self-latches the tier",
                   ])


STEPS = [f1_golden, f2_config, f3_realtime, f4_button, f5_outputs, f6_charger,
         f7_freeze]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="start", type=int, default=1)
    ap.add_argument("--only", type=int, default=None)   # 1..7
    ap.add_argument("--seconds", type=int, default=60)
    ap.add_argument("--port", default=None)
    ap.add_argument("--no-save", action="store_true")
    a = ap.parse_args()

    port = a.port or resolve_port(None)
    if not port or "no ESP32" in str(port):
        print(f"{RED}  no ESP32-S3 serial port found - plug the board in{OFF}")
        return 2

    sess = None if a.no_save else PS.run_dir("field_ready")
    ctx = {"port": port, "seconds": a.seconds, "session": sess, "results": {}}

    print()
    print(f"  {BOLD}FIELD READINESS - the standalone build's six owed "
          f"checks{OFF}")
    print(f"  port {port}")
    if sess:
        print(f"  session {sess}")
    print("  F1-F3 are hard gates and stop the run. F4-F6 need your eyes.")

    idx = range(len(STEPS))
    if a.only:
        idx = [a.only - 1]
    else:
        idx = range(a.start - 1, len(STEPS))

    for i in idx:
        if not STEPS[i](ctx):
            if i < 3:
                print(f"\n  {RED}{BOLD}STOPPED at F{i + 1}. Fix it before "
                      f"anything below it is worth measuring.{OFF}")
                break
            print(f"\n  {YELLOW}F{i + 1} did not pass. Continuing - the "
                  f"remaining checks are independent.{OFF}")

    print()
    print("=" * 72)
    print(f"  {BOLD}SUMMARY{OFF}")
    for k in ("F1", "F2", "F3", "F4", "F5", "F6", "F7"):
        r = ctx["results"].get(k)
        if r is None:
            print(f"    {k}  not run")
        else:
            print(f"    {k}  {GREEN + 'PASS' + OFF if r['pass'] else RED + 'FAIL' + OFF}")
    hard = [ctx["results"].get(k, {}).get("pass") for k in ("F1", "F2", "F3")]
    if all(h is True for h in hard):
        print(f"\n  {GREEN}{BOLD}The hard gates pass. It is a detector, and it "
              f"is real time.{OFF}")
    print("=" * 72)

    if sess:
        entry = {"stage": "field_ready", "t": time.strftime("%FT%T"),
                 "port": port, **{k: v for k, v in ctx["results"].items()}}
        (sess / "field_ready.json").write_text(json.dumps(entry, indent=2))
        print(f"  saved {sess / 'field_ready.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
