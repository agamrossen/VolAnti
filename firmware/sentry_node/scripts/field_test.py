#!/usr/bin/env python
"""
field_test.py - THE SIMPLE ONE. It tells you what to do; you do it.

    conda activate acoustic-detector
    cd ~/acoustic-detector/firmware/sentry_node
    python scripts/field_test.py

No configuration questions, no y/n checklists. It checks the hardware itself,
then walks you through: be quiet -> talk and clap -> spin the propeller. It
measures everything and prints one table at the end.

    python scripts/field_test.py --quiet-secs 60 --rotor-secs 30
    python scripts/field_test.py --skip-checks       # straight to the audio
    python scripts/field_test.py --demo 90           # the milestone run
    python scripts/field_test.py --live              # just watch it work

The only thing it asks you to type is the distance before each rotor run, so
the table means something afterwards.

EVERY Measured CAPTURE ARCHIVES RAW 4-CHANNEL AUDIO by default, with exact
sequence-loss accounting (--no-raw opts out). This is not bookkeeping: per-frame
scores are only meaningful at the constants that produced them, whereas the
samples can be re-scored against any future detector, any threshold, and any
band. Every capture before threw the audio away, which is why the
rotor non-detection had to be argued from scores instead of settled from air.

  --demo  runs `Z`: raw audio archived, outputs SILENT. The capture.
  --live  runs `G`: outputs fire, per-frame trace saved, NO audio. The show.
"""
import argparse
import json
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
from trace_proto import parse_stream                            # noqa: E402

BOLD, DIM = "\033[1m", "\033[2m"
RED, GREEN, YELLOW, CYAN = "\033[31m", "\033[32m", "\033[33m", "\033[36m"
OFF = "\033[0m"
FS = 16000
HOP_S = 512 / FS


def say(msg):
    print(f"\n{BOLD}  {msg}{OFF}")


def wait_enter(msg="Press Enter when you are ready"):
    try:
        input(f"  {CYAN}{msg}...{OFF} ")
    except EOFError:
        pass


def show_off(port):
    """A visible parade of every output, in order, so you can SEE the whole
    device work before any measurement starts. No questions - just watch."""
    say("Checking the outputs — watch the device")

    print("  e-paper: LISTENING screen ...")
    talk_cmd(port, "U e r", 22)
    print("  e-paper: ALERT screen ...")
    talk_cmd(port, "U e a", 22)
    print("  e-paper: back to LISTENING ...")
    talk_cmd(port, "U e r", 22)

    print("  buzzer ...")
    talk_cmd(port, "U b1", 1.5); time.sleep(0.7); talk_cmd(port, "U b0", 1.5)
    print("  vibration motor ...")
    talk_cmd(port, "U v1", 1.5); time.sleep(0.9); talk_cmd(port, "U v0", 1.5)
    print("  LED: three channels, then white ...")
    for trip in ("255 0 0", "0 255 0", "0 0 255", "255 255 255"):
        talk_cmd(port, f"U l {trip}", 1.2)
        time.sleep(0.5)
    talk_cmd(port, "U l 0 0 0", 1.2)

    print(f"\n  {BOLD}Button: press and HOLD it now — 3 seconds.{OFF}")
    txt = talk_text(port, "U g", 6.0)
    if "NEVER went low" in txt:
        print(f"    {YELLOW}GPIO21 saw nothing. Scanning every free pin — "
              f"keep holding the button.{OFF}")
        txt += talk_text(port, "U G", 9.0)
    for ln in txt.splitlines():
        if "GPIO" in ln or "went low" in ln or "sees the switch" in ln \
                or "transitions" in ln or "bounce" in ln:
            print(f"    {ln.strip()}")
    print()


def talk_cmd(port, cmd, listen_s):
    with open_serial(port, timeout=0.2) as s:
        s.write((cmd + "\n").encode())
        s.flush()
        t0 = time.time()
        while time.time() - t0 < listen_s:
            c = s.read(4096)
            if not c:
                continue
            recs, _, _ = parse_stream(bytes(c))
            if any(r["_kind"] == "ack" for r in recs):
                return True
    return False


def talk_text(port, cmd, listen_s):
    """For diagnostics that answer in plain text (the GPIO watch)."""
    raw = b""
    with open_serial(port, timeout=0.2) as s:
        s.write((cmd + "\n").encode())
        s.flush()
        t0 = time.time()
        while time.time() - t0 < listen_s:
            raw += s.read(4096)
    return raw.decode("utf-8", "replace")


def check_hardware(port):
    """Everything the script can verify on its own. No questions asked."""
    say("Checking the hardware")
    ok = True

    raw = b""
    with open_serial(port, timeout=0.2) as s:
        s.write(b"I\n")
        s.flush()
        t0 = time.time()
        while time.time() - t0 < 6:
            raw += s.read(4096)
            if b"modes:" in raw:
                break
    if b"SENTRY-NODE" not in raw:
        print(f"  {RED}the board is not answering. Press RST and try again."
              f"{OFF}")
        return False
    print(f"  {GREEN}link OK{OFF}")

    # microphones: read the quad meter briefly
    recs = []
    with open_serial(port, timeout=0.2) as s:
        s.write(b"Q\n")
        s.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 6:
            c = s.read(65536)
            if not c:
                continue
            buf += c
            rs, _, used = parse_stream(bytes(buf))
            recs += rs
            buf = buf[used:]
        s.write(b"\n")
        s.flush()
        time.sleep(0.5)
        s.read(65536)

    qmt = [r for r in recs if r["_kind"] == "qmt"]
    if not qmt:
        print(f"  {RED}no microphone data at all.{OFF}")
        return False
    last = qmt[-1]
    rms, to = np.array(last["rms"], float), list(last["timeouts"])
    for c in range(4):
        state = (f"{GREEN}ok{OFF}" if rms[c] > 0
                 else f"{RED}SILENT{OFF}")
        print(f"  {qa.CH_NAMES[c]:<10} rms {rms[c]:8.1f}   {state}")
    print(f"  bus A timeouts {to[0]}   bus B timeouts {to[1]}")
    if to[0]:
        print(f"  {RED}bus A (GPIO6, M1+M2) is not clocking{OFF}")
        ok = False
    if to[1]:
        print(f"  {RED}bus B (GPIO7, M3+M4) is not clocking{OFF}")
        ok = False
    dead = [qa.CH_NAMES[c] for c in range(4) if rms[c] <= 0]
    if dead:
        print(f"  {RED}silent channels: {', '.join(dead)}{OFF}")
        ok = False
    if ok:
        print(f"  {GREEN}all four microphones alive{OFF}")
    return ok


def run_guard(port, seconds, label, thr_milli=None, raw=False):
    """Run the detector for `seconds`, with a live one-line readout.

    Returns (frames, alerts, pcm_records, interrupted). pcm_records is empty
    unless raw; `interrupted` is True if Ctrl-C ended the capture early.

    Ctrl-C KEEPS WHAT ARRIVED. A 120 s capture abandoned at 118 s is still 118
    seconds of evidence, and letting KeyboardInterrupt escape would throw away
    every frame and every sample already in hand - the same "the run existed
    only in the terminal" failure this tool was rewritten to end.

    raw=False -> `G`, the ARMED path: buzzer, motor, LED and e-paper all fire.
                 Per-frame records only. The samples are gone forever.
    raw=True  -> `Z`, which runs THE SAME LOOP: same front_end, same combiner,
                 same back_end, same threshold. In `run_quad_pipeline()` the
                 `armed` flag gates only the actuators and the PCM stream, so
                 the DECISIONS are identical - what changes is that `Z` also
                 emits the exact int16 samples the pipeline consumed, each
                 block sequence-numbered.

    Prefer raw for anything worth keeping. Per-frame scores answer only "did it
    alert, at today's constants"; the samples answer every question anyone asks
    afterwards, including questions about constants that do not exist yet. That
    is the difference between a run you can re-analyse and a run you can only
    remember. A side benefit: `Z` does not sound the buzzer, so an alert cannot
    contaminate its own recording.
    """
    if raw:
        if thr_milli:
            print(f"  {YELLOW}note: the firmware's Z command takes no "
                  f"threshold argument, so this raw capture runs at the "
                  f"deployment default 1.700, NOT {thr_milli / 1000:.3f}. "
                  f"The samples are threshold-free — re-score them offline at "
                  f"whatever threshold you want.{OFF}")
        # `Z 0` is not "zero seconds" to the firmware, it is "use the 30 s
        # default" (sentry_node.c:1352), so never let rounding produce one.
        cmd = f"Z {max(1, int(round(seconds)))}\n"
    else:
        cmd = f"G {thr_milli}\n" if thr_milli else "G\n"
    frames, alerts, pcm = [], [], []
    n_samp = 0
    saw_end = False
    interrupted = False

    # WHO DECIDES WHEN TO STOP, and why it differs by mode.
    # `G` is unbounded: the host ends it by sending a byte.
    # `Z` is bounded by the device, but run_quad_pipeline() breaks on
    # `stop_after_this` OR on any byte from the host (sentry_node.c:712). At
    # 128 kB/s the link can run behind real time, so a host stop at exactly
    # `seconds` would truncate a capture that is merely lagging - and the lost
    # tail would then be indistinguishable from link drops, an own goal
    # reported as a hardware limit. So in raw mode the host waits for the END
    # record and only forces a stop if the device goes silent for far too long.
    deadline = (seconds * 1.5 + 10.0) if raw else seconds
    with open_serial(port, timeout=0.2) as s:
        s.write(cmd.encode())
        s.flush()
        buf = bytearray()
        t0 = time.time()
        last = 0.0
        try:
            while time.time() - t0 < deadline:
                c = s.read(65536)
                if c:
                    buf += c
                    rs, _, used = parse_stream(bytes(buf))
                    for r in rs:
                        if r["_kind"] == "rec":
                            frames.append(r)
                        elif r["_kind"] == "alt":
                            alerts.append(r)
                            print(f"\n  {RED}{BOLD}ALERT  f0={r['f0_hz']:.0f} "
                                  f"Hz  score={r['score']:.2f}{OFF}")
                        elif r["_kind"] == "pcm4":
                            pcm.append(r)
                            n_samp += r["n_frames"]
                        elif r["_kind"] == "end":
                            saw_end = True
                    buf = buf[used:]
                if raw and saw_end:
                    break
                now = time.time()
                if now - last > 0.4:
                    last = now
                    left = max(0.0, seconds - (now - t0))
                    sc = frames[-1]["score"] if frames else 0.0
                    f0 = frames[-1]["f0_hz"] if frames else 0.0
                    m, sec = divmod(int(left + 0.5), 60)
                    tail = f"   rec {n_samp / FS:5.1f}s" if raw else ""
                    sys.stdout.write(
                        f"\r  {label}  {m}:{sec:02d} left   score {sc:6.2f}   "
                        f"f0 {f0:6.0f} Hz   alerts {len(alerts)}{tail}   ")
                    sys.stdout.flush()
        except KeyboardInterrupt:
            interrupted = True
            print(f"\n  {YELLOW}stopped early — keeping the "
                  f"{len(frames)} frames and {n_samp / FS:.1f}s already "
                  f"captured.{OFF}")

        if raw and not saw_end and not interrupted:
            print(f"\n  {YELLOW}the device never sent its END record — "
                  f"stopping it by hand. The capture may be short.{OFF}")
        if not saw_end:
            s.write(b"\n")
            s.flush()
        # Drain the tail: records already in flight are not loss, but they look
        # exactly like it in the sequence numbers if we close the port on them.
        try:
            t1 = time.time()
            while time.time() - t1 < (5 if raw else 3):
                c = s.read(65536)
                if not c:
                    break
                buf += c
                rs, _, used = parse_stream(bytes(buf))
                for r in rs:
                    if r["_kind"] == "rec":
                        frames.append(r)
                    elif r["_kind"] == "alt":
                        alerts.append(r)
                    elif r["_kind"] == "pcm4":
                        pcm.append(r)
                buf = buf[used:]
        except KeyboardInterrupt:
            interrupted = True      # a second Ctrl-C: still keep the data
    print()
    return frames, alerts, pcm, interrupted


def save_raw(sess, label, pcm_recs):
    """Archive the exact samples the pipeline consumed, with an honest account
    of what was lost getting them here. Returns (filename, report).

    4 ch x 16 kHz x 2 B = 128 kB/s over USB-Serial-JTAG is an UNPROVEN rate, so
    loss is expected rather than exceptional. `assemble_pcm4` returns only the
    longest contiguous run of sequence numbers: splicing across a gap would
    shift every later sample and read as an inter-channel delay, which is the
    precise error the sequence numbers exist to prevent. The drop report is
    written INSIDE the npz, next to the audio, so no later analysis can pick up
    the samples while leaving their provenance behind.
    """
    if not pcm_recs:
        return None, {"n_records": 0, "dropped": 0, "drop_rate": 0.0,
                      "runs": 0, "used_frames": 0}
    ch, rep = qa.assemble_pcm4(pcm_recs)
    p = sess / f"{label}_raw.npz"
    np.savez_compressed(p, audio=ch, fs=np.int32(FS),
                        report=np.array(json.dumps(rep, sort_keys=True)))
    rep["seconds"] = ch.shape[1] / FS
    rep["file"] = p.name
    rep["mb"] = p.stat().st_size / 1e6
    return str(p.name), rep


def say_raw(rep):
    """One line about the audio, and a LOUD one if anything was lost."""
    if not rep or not rep.get("n_records"):
        print(f"  {RED}NO RAW AUDIO CAPTURED — this run cannot be "
              f"re-analysed. Check that the firmware supports `Z`.{OFF}")
        return
    lost = rep.get("dropped", 0)
    if lost:
        print(f"  {YELLOW}raw: {rep['seconds']:.1f}s kept "
              f"({rep['mb']:.1f} MB) — but {lost} of {rep['expected']} blocks "
              f"were LOST ({rep['drop_rate'] * 100:.1f}%) in {rep['runs']} "
              f"fragments. Only the longest unbroken stretch is stored; the "
              f"link could not keep up.{OFF}")
    else:
        print(f"  {GREEN}raw: {rep['seconds']:.1f}s of 4-channel audio "
              f"({rep['mb']:.1f} MB), nothing lost{OFF}")


def summarise(frames, alerts, thr=1.70):
    """Aggregate AND diagnose. An alert needs `track_need` (6) consecutive
    frames above threshold whose f0 stays within 2% frame to frame. When no
    alert fires, the useful question is which of those two conditions failed -
    so both are measured here rather than left to guesswork."""
    if not frames:
        return {"frames": 0, "alerts": len(alerts)}
    sc = np.array([f["score"] for f in frames], float)
    f0 = np.array([f["f0_hz"] for f in frames], float)
    chain = np.array([f["chain"] for f in frames], int)
    above = sc >= thr

    # longest run of consecutive above-threshold frames
    best = run = 0
    for v in above:
        run = run + 1 if v else 0
        best = max(best, run)

    # f0 stability WHILE above threshold - the continuity rule's actual input
    f0_hi = f0[above]
    if len(f0_hi) > 1:
        step = np.abs(np.diff(f0_hi)) / np.maximum(f0_hi[:-1], 1.0)
        cont_ok = float(np.mean(step < 0.02))
    else:
        cont_ok = float("nan")

    top = np.argsort(sc)[-max(1, len(sc) // 10):]
    return {
        "frames": len(sc), "seconds": len(sc) * HOP_S,
        "score_median": float(np.median(sc)), "score_max": float(sc.max()),
        "score_p99": float(np.percentile(sc, 99)),
        "f0_at_loudest": float(np.median(f0[top])),
        "f0_median_above": float(np.median(f0_hi)) if len(f0_hi) else 0.0,
        "frames_above": int(above.sum()),
        "longest_run_above": int(best),
        "max_chain": int(chain.max()) if len(chain) else 0,
        "f0_continuity_frac": cont_ok,
        "alerts": len(alerts),
    }


def explain(s, thr=1.70):
    """Say, in one line, why it did or did not alert."""
    if s.get("frames", 0) == 0:
        return f"{RED}no data{OFF}"
    if s["alerts"]:
        return f"{GREEN}DETECTED{OFF}"
    if s["frames_above"] == 0:
        return (f"no alert: never crossed {thr:.2f} "
                f"(peak {s['score_max']:.2f})")
    if s["longest_run_above"] < 6:
        return (f"no alert: crossed {thr:.2f} on {s['frames_above']} frames "
                f"but the longest UNBROKEN run was {s['longest_run_above']} "
                f"(needs 6)")
    return (f"no alert: {s['longest_run_above']} consecutive frames above "
            f"threshold, but f0 was only stable on "
            f"{s['f0_continuity_frac'] * 100:.0f}% of them (needs <2% step) "
            f"— max chain reached {s['max_chain']}")


def save_trace(sess, label, frames):
    """Keep EVERY frame. The last session threw the per-frame data away and the
    question 'why did the propeller not alert' became unanswerable."""
    if not frames:
        return None
    p = sess / f"{label}_trace.npz"
    np.savez_compressed(
        p,
        score=np.array([f["score"] for f in frames], np.float32),
        f0=np.array([f["f0_hz"] for f in frames], np.float32),
        teeth=np.array([f["teeth"] for f in frames], np.int16),
        chain=np.array([f["chain"] for f in frames], np.int32),
        above=np.array([f["above_thr"] for f in frames], np.int8),
        accepted=np.array([f["cont_accepted"] for f in frames], np.int8))
    return str(p.name)


def live_watch(port, thr_milli=None, sess=None):
    """THE DEVICE, RUNNING. No stages, no prompts, no time limit.

    Walk around with the rotor, change the throttle, come closer, go further.
    The line below updates continuously and every alert is printed. Ctrl-C
    stops it. This is the mode for SEEING it work rather than measuring it.

    EVERY FRAME IS KEPT. On this mode produced the first real
    propeller alerts in the project's history - f0 423 Hz and 471 Hz - and saved
    nothing at all, so the milestone survived only as terminal scrollback and
    had to be written down from memory. A mode used for demonstrating is also,
    inevitably, the mode that is running when something happens for the first
    time. It archives now.

    It still cannot archive AUDIO: `G` is the armed path and the firmware emits
    PCM only when unarmed. For a run whose samples must survive, use --demo,
    which captures raw at the cost of the buzzer and the e-paper."""
    say("LIVE — the device is running now. Ctrl-C to stop.")
    print("  Move the rotor around, change the throttle, walk it closer.")
    print(f"  {DIM}The bar is the comb score. The threshold is the marker.{OFF}")
    if sess is None:
        print(f"  {YELLOW}no session directory — this run will NOT be saved."
              f"{OFF}")
    print()
    thr = (thr_milli / 1000.0) if thr_milli else 1.70
    cmd = f"G {thr_milli}\n" if thr_milli else "G\n"
    rec_frames, alert_recs = [], []
    frames = alerts = 0
    peak = 0.0
    try:
        with open_serial(port, timeout=0.2) as s:
            s.write(cmd.encode())
            s.flush()
            buf = bytearray()
            last = 0.0
            while True:
                c = s.read(65536)
                if c:
                    buf += c
                    rs, _, used = parse_stream(bytes(buf))
                    for r in rs:
                        if r["_kind"] == "rec":
                            frames += 1
                            rec_frames.append(r)
                            peak = max(peak, r["score"])
                            now = time.time()
                            if now - last > 0.15:
                                last = now
                                sc = r["score"]
                                w = 34
                                filled = int(w * min(1.0, sc / 3.0))
                                mark = int(w * thr / 3.0)
                                bar = "".join(
                                    "|" if i == mark else ("#" if i < filled
                                                           else "-")
                                    for i in range(w))
                                hot = RED if sc >= thr else GREEN
                                sys.stdout.write(
                                    f"\r  {hot}[{bar}]{OFF} {sc:5.2f}  "
                                    f"f0 {r['f0_hz']:6.0f}Hz  chain "
                                    f"{r['chain']:2d}  peak {peak:5.2f}  "
                                    f"alerts {alerts}   ")
                                sys.stdout.flush()
                        elif r["_kind"] == "alt":
                            alerts += 1
                            alert_recs.append(r)
                            print(f"\n  {RED}{BOLD}*** ALERT  f0="
                                  f"{r['f0_hz']:.0f} Hz  score="
                                  f"{r['score']:.2f}  ***{OFF}")
                    buf = buf[used:]
    except KeyboardInterrupt:
        print("\n  stopping...")
    finally:
        try:
            with open_serial(port, timeout=0.2) as s2:
                s2.write(b"\n")
                s2.flush()
                time.sleep(0.5)
                s2.read(65536)
        except Exception:
            pass
    print(f"\n  live: {frames} frames ({frames * HOP_S:.0f}s), peak score "
          f"{peak:.2f}, {alerts} alert(s)")

    out = {"frames": frames, "peak": peak, "alerts": alerts,
           "seconds": frames * HOP_S, "threshold": thr,
           "alert_f0_hz": [round(r["f0_hz"], 1) for r in alert_recs],
           "alert_score": [round(r["score"], 3) for r in alert_recs],
           "alert_chain": [int(r["chain"]) for r in alert_recs],
           "raw_audio": False}
    if alert_recs:
        report_f0(out["alert_f0_hz"])
    if sess is not None and rec_frames:
        out["trace"] = save_trace(sess, f"live_{datetime.now():%H%M%S}",
                                  rec_frames)
        PS.manifest_append(sess, {"stage": "live", **out})
        print(f"  {GREEN}saved {out['trace']}{OFF} — {frames} frames kept")
        print(f"  {DIM}per-frame scores only. --demo captures the audio.{OFF}")
    return out


def report_f0(f0s):
    """Say where the alerting f0 sits against the physics prediction. Two
    datapoints from one rig is not a validation of the band and this must not
    read as one - so it states the count as well as the verdict."""
    lo, hi = min(f0s), max(f0s)
    inside = [f for f in f0s if 375 <= f <= 475]
    print(f"\n  {BOLD}ALERTING f0: {lo:.0f} – {hi:.0f} Hz{OFF}"
          if lo != hi else f"\n  {BOLD}ALERTING f0: {lo:.0f} Hz{OFF}")
    print(f"  predicted for a 7in 3-blade at loaded hover: 375–475 Hz")
    print(f"  the PROVISIONAL priority band: 200–800 Hz")
    print(f"  {len(inside)} of {len(f0s)} alert(s) inside the hover "
          f"prediction.")
    print(f"  {DIM}{len(f0s)} datapoint(s), one rig, one day. This is support "
          f"for the band, not a validation of it.{OFF}")


def run_demo(port, sess, seconds, thr_milli=None):
    """THE MILESTONE RUN — the detection, repeatable, with the audio kept.

    On a dynamically-stimulated rotor produced this project's first
    real-propeller alerts and left no file. This mode reproduces that run and
    archives it: per-frame trace AND raw 4-channel audio.

    Why DYNAMIC stimulation is the instruction and not a suggestion: the same
    rotor, held at a steady speed on the same day, did not alert at all. The
    adaptive floor rises with tau = 6 s and whitening is S = log1p(mag/floor),
    so a tone that persists is absorbed into the floor it created. Varying the
    throttle keeps the floor chasing and never converged. That contrast IS the
    finding - it is not an inconvenience to be smoothed away by holding steady.

    The cost, stated plainly: this runs `Z`, so the buzzer, motor and e-paper
    stay silent. It is the capture, not the show. Use --live for the show."""
    say(f"DEMO — {seconds:.0f} seconds, recording everything.")
    print(f"{RED}  SAFETY: props up. Rig staked or ballasted. Never hand-held "
          f"while spinning.\n  Stand clear of the prop disc. Keep the Mac and "
          f"the USB lead out of the prop plane.{OFF}")
    print(f"\n  {BOLD}Vary the throttle the whole time.{OFF} A steady hold is "
          f"the one thing that does NOT work:")
    print("    - bring it up to hover-band throttle and hold ~5 s")
    print("    - sweep up and back down, repeatedly")
    print("    - move the rig closer and further, change its angle")
    print(f"  {DIM}Watch the f0 readout: 375–475 Hz is the predicted "
          f"loaded-hover blade pass.{OFF}")
    print(f"  {DIM}The outputs stay silent in this mode — that is expected, "
          f"and it keeps the buzzer out of the recording.{OFF}")
    wait_enter("Press Enter once it is spinning")

    fr, al, pcm, _ = run_guard(port, seconds, "DEMO    ", thr_milli, raw=True)
    s = summarise(fr, al)
    s["trace"] = save_trace(sess, "demo", fr)
    s["raw"], rep = save_raw(sess, "demo", pcm)
    s["raw_report"] = rep
    s["alert_f0_hz"] = [round(r["f0_hz"], 1) for r in al]
    s["alert_score"] = [round(r["score"], 3) for r in al]
    s["alert_chain"] = [int(r["chain"]) for r in al]
    PS.manifest_append(sess, {"stage": "demo", **s})

    print(f"\n  {explain(s)}")
    say_raw(rep)
    if al:
        report_f0(s["alert_f0_hz"])
    else:
        print(f"  {YELLOW}No alert this run. The audio is still archived — "
              f"re-score it offline with quad_reference.py before concluding "
              f"anything about the detector.{OFF}")
    return s


# ===========================================================================
# THE RIG PROTOCOL (the port, --rig)
#
# Tomorrow is the first day with a FOUR-MOTOR drone mimic - four threat-platform
# motors on 7" 3-blade props, individually throttled. It is the first chance
# this project has ever had to record the actual airframe, and every constant
# in the detector is currently set from physics and synthesis.
#
# So the protocol is written to answer the questions that cannot be answered
# offline, in the order that a day can actually be run, and EVERY stage archives
# RAW QUAD PCM. That last part is not a preference. On the run that
# produced this project's first real propeller alerts saved no audio, and the
# milestone survived as terminal scrollback. Traces answer "did it alert, at
# today's constants". Samples answer every question anyone asks afterwards,
# including questions about constants that do not exist yet.
#
# THE ONE STAGE TO PROTECT IF THE DAY RUNS SHORT: per-motor solo. Nothing else
# tells us where the energy actually IS. Analysis of the captures
# (see HANDOFF_OVERNIGHT_2.md) found the single rotor put +0.2 dB into the
# 200-800 Hz priority band and +12.6 dB into 3.2-7.8 kHz - i.e. the band the
# whole detector is aimed at may be aimed at the wrong place. A solo motor at
# three throttles, recorded raw, settles that.
# ===========================================================================

RIG_THROTTLES = ("LOW (just spinning)", "HOVER (the working point)",
                 "HIGH (near full)")


def _rig_capture(port, sess, label, seconds, prompt, safety=False,
                 thr_milli=None, results=None, extra=None):
    """One rig stage: prompt, capture RAW, archive, summarise. Ctrl-C safe -
    run_guard keeps whatever arrived, and everything is on disk before the
    next stage starts."""
    say(prompt)
    if safety:
        print(f"{RED}  SAFETY: props up. Rig staked or ballasted. Never "
              f"hand-held while spinning.\n  Stand clear of the prop disc. "
              f"Keep the Mac and the USB lead out of the prop plane.{OFF}")
    wait_enter()
    fr, al, pcm, stop = run_guard(port, seconds, f"{label[:8]:<8}",
                                  thr_milli, raw=True)
    s = summarise(fr, al)
    s["trace"] = save_trace(sess, label, fr)
    s["raw"], rep = save_raw(sess, label, pcm)
    s["raw_report"] = rep
    s["alert_f0_hz"] = [round(r["f0_hz"], 1) for r in al]
    s["alert_score"] = [round(r["score"], 3) for r in al]
    if extra:
        s.update(extra)
    PS.manifest_append(sess, {"stage": label, **s})
    if results is not None:
        results[label] = s
    print(f"  {explain(s)}")
    say_raw(rep)
    return s, stop


def run_rig(port, sess, a, thr_milli=None):
    """The guided four-motor rig day. Additive: it reuses run_guard, save_raw
    and the manifest exactly as every other stage does."""
    results = {}
    print(f"\n{BOLD}  THE RIG PROTOCOL{OFF}")
    print("  Ten stages. Every one archives raw 4-channel audio, so tonight's")
    print("  analysis can ask questions this protocol did not think of.")
    print(f"  {DIM}Ctrl-C at any point keeps everything captured so far and "
          f"moves on.{OFF}\n")

    # ---- 1. quiet -------------------------------------------------------
    _, stop = _rig_capture(
        port, sess, "quiet", a.rig_quiet,
        f"STAGE 1/10 - QUIET for {a.rig_quiet:.0f} s. Rig OFF, everyone still.",
        results=results)
    print(f"  {DIM}This is the bed every later stage is measured against, and "
          f"the false-alarm gate Tier-2 has to pass.{OFF}")
    if stop:
        return results

    # ---- 2. talk near the rig -------------------------------------------
    _, stop = _rig_capture(
        port, sess, "talk_near_rig", a.rig_talk,
        f"STAGE 2/10 - TALK for {a.rig_talk:.0f} s, standing where you will "
        f"stand all day.",
        results=results)
    print(f"  {DIM}Speech beat the real rotor in the detector's own metric on "
          f"the reference captures (chains of 26-28 against the rotor's 3). This is the "
          f"HARD negative gate for both tiers.{OFF}")
    if stop:
        return results

    # ---- 3. per-motor solo ----------------------------------------------
    print(f"\n{BOLD}  STAGE 3/10 - PER-MOTOR SOLO. THE MOST IMPORTANT STAGE "
          f"OF THE DAY.{OFF}")
    print("  One motor at a time, three throttles each. If the day runs short,")
    print("  protect this stage and drop the distance ladder.")
    print(f"  {DIM}It is the only stage that answers WHERE THE ENERGY IS. The "
          f"detector searches 200-800 Hz for the blade-pass comb; the one real "
          f"rotor recording in existence put +0.2 dB there and +12.6 dB above "
          f"3.2 kHz. If that repeats on the four-motor rig, the priority band "
          f"is aimed at the wrong place and that is the finding of the day."
          f"{OFF}")
    for m in range(1, 5):
        for ti, thr_name in enumerate(RIG_THROTTLES):
            label = f"solo_M{m}_{['low', 'hover', 'high'][ti]}"
            _, stop = _rig_capture(
                port, sess, label, a.rig_solo,
                f"  motor {m} of 4, throttle {thr_name} - "
                f"{a.rig_solo:.0f} s, HOLD IT STEADY.",
                safety=(m == 1 and ti == 0), results=results,
                extra={"motor": m, "throttle": thr_name})
            if stop:
                return results

    # ---- 4. all four, steady hover: THE ABSORPTION CASE ------------------
    _, stop = _rig_capture(
        port, sess, "quad_hover_steady", a.rig_steady,
        f"STAGE 4/10 - ALL FOUR MOTORS, STEADY HOVER THROTTLE, "
        f"{a.rig_steady:.0f} s. Do not vary it.",
        safety=True, results=results)
    print(f"  {DIM}THE case Tier-2 was built for. v1 is predicted to go quiet "
          f"after ~15 s as its 6 s floor absorbs the comb; Tier-2's floor is "
          f"an order of magnitude slower. Resist the urge to blip the "
          f"throttle - a steady hold is the measurement.{OFF}")
    if stop:
        return results

    # ---- 5. all four, dynamic -------------------------------------------
    _, stop = _rig_capture(
        port, sess, "quad_dynamic", a.rig_dynamic,
        f"STAGE 5/10 - ALL FOUR, VARY THE THROTTLE for "
        f"{a.rig_dynamic:.0f} s. Sweep up and down, move the rig.",
        safety=True, results=results)
    print(f"  {DIM}v1's territory: a moving comb the floor cannot chase. The "
          f"dynamic run is the only one that ever alerted.{OFF}")
    if stop:
        return results

    # ---- 6. punch-outs ---------------------------------------------------
    for i in range(a.rig_punches):
        _, stop = _rig_capture(
            port, sess, f"punch_{i}", a.rig_punch_secs,
            f"STAGE 6/10 - PUNCH-OUT {i + 1} of {a.rig_punches}: from hover, "
            f"snap to full for ~1 s, then back.",
            safety=(i == 0), results=results)
        if stop:
            return results

    # ---- 7. distance ladder ---------------------------------------------
    dists = [x.strip() for x in a.rig_distances.split(",") if x.strip()]
    for d in dists:
        for kind, secs in (("steady", a.rig_ladder_steady),
                           ("dynamic", a.rig_ladder_dynamic)):
            _, stop = _rig_capture(
                port, sess, f"ladder_{d}m_{kind}", secs,
                f"STAGE 7/10 - DISTANCE {d} m, {kind.upper()}, {secs:.0f} s.",
                safety=(d == dists[0] and kind == "steady"), results=results,
                extra={"distance_m": d, "kind": kind})
            if stop:
                return results

    # ---- 8. wind negative ------------------------------------------------
    if a.rig_wind:
        _, stop = _rig_capture(
            port, sess, "fan_wind", a.rig_wind,
            f"STAGE 8/10 - FAN WIND for {a.rig_wind:.0f} s, rig OFF. Point a "
            f"fan at the array from ~1 m.",
            results=results)
        print(f"  {DIM}THE ONLY STAGE THAT TESTS THE COHERENCE PREMISE. Wind "
              f"pressure at a capsule is locally generated and largely "
              f"uncorrelated between ports even 40 mm apart; an acoustic wave "
              f"is correlated. Measured on the reference captures, coherence "
              f"was 0.92-0.98 below 800 Hz for EVERYTHING - quiet, speech and "
              f"rotor alike - because indoors they are all acoustic. Without "
              f"this stage, coherence stays an untested idea.{OFF}")
        if stop:
            return results

    print(f"\n{GREEN}{BOLD}  RIG PROTOCOL COMPLETE.{OFF}")
    print(f"  {len(results)} stages archived in {sess}")
    print(f"  Next: python scripts/rig_report.py {sess}")
    return results


def finish(sess, results):
    """Write the session summary. Called on the normal path AND on Ctrl-C, so
    an abandoned session still leaves a readable index of what it captured.
    The per-capture files and manifest lines are already on disk by now - this
    only adds the summary that ties them together."""
    out = sess / f"fieldtest_{datetime.now():%H%M%S}.json"
    out.write_text(json.dumps(results, indent=2))
    print(f"\n  saved: {out}")
    print(f"{DIM}  Distances here are relative, for this rig on this day. The "
          f"single motor is about a quarter of the sound power of a four-motor "
          f"airframe, so treat every distance as a conservative floor.{OFF}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--quiet-secs", type=float, default=60)
    ap.add_argument("--talk-secs", type=float, default=30)
    ap.add_argument("--rotor-secs", type=float, default=30)
    ap.add_argument("--skip-checks", action="store_true")
    ap.add_argument("--distances", default="",
                    help="comma-separated metres for MEASURED rotor runs, "
                         "e.g. --distances 4,10,20. Omit to skip straight to "
                         "the live watch.")
    ap.add_argument("--live", action="store_true",
                    help="skip every measurement and just RUN the device "
                         "(armed: outputs fire. Trace saved, audio NOT — the "
                         "firmware streams PCM only when unarmed)")
    ap.add_argument("--demo", type=float, default=None, metavar="SECS",
                    help="the milestone run: SECS of dynamic rotor with RAW "
                         "AUDIO archived, e.g. --demo 90. Outputs stay silent.")
    ap.add_argument("--no-raw", action="store_true",
                    help="do not archive raw audio on the measured captures. "
                         "Saves disk and link bandwidth; throws away the only "
                         "artifact that outlives the current constants.")
    ap.add_argument("--threshold", type=float, default=None,
                    help="operating threshold, e.g. --threshold 1.45")
    # ---- the port: the four-motor rig protocol. ADDITIVE. --------------
    ap.add_argument("--rig", action="store_true",
                    help="the guided FOUR-MOTOR rig protocol: quiet, talk, "
                         "per-motor solo at three throttles, all-4 steady "
                         "hover (the absorption case), all-4 dynamic, "
                         "punch-outs, a distance ladder and a fan-wind "
                         "negative. Every stage archives raw quad PCM.")
    ap.add_argument("--rig-quiet", type=float, default=120)
    ap.add_argument("--rig-talk", type=float, default=60)
    ap.add_argument("--rig-solo", type=float, default=30)
    ap.add_argument("--rig-steady", type=float, default=120)
    ap.add_argument("--rig-dynamic", type=float, default=90)
    ap.add_argument("--rig-punches", type=int, default=5)
    ap.add_argument("--rig-punch-secs", type=float, default=15)
    ap.add_argument("--rig-distances", default="5,10,15,20",
                    help="metres for the distance ladder; add the lab max")
    ap.add_argument("--rig-ladder-steady", type=float, default=60)
    ap.add_argument("--rig-ladder-dynamic", type=float, default=30)
    ap.add_argument("--rig-wind", type=float, default=120,
                    help="fan-wind negative in seconds; 0 to skip. This is "
                         "the ONLY stage that tests the coherence premise.")
    import geometry as _geom
    _geom.add_argument(ap)
    a = ap.parse_args()

    port = resolve_port(a.port)
    print(f"\n{BOLD}  SENTRY-NODE — FIELD TEST{OFF}")
    print(f"  {DIM}port {port}. I will tell you what to do at each step.{OFF}")

    if not a.skip_checks and not check_hardware(port):
        print(f"\n{RED}  Hardware check failed — fix that before the audio "
              f"tests, or re-run with --skip-checks to continue anyway.{OFF}")
        return 1

    thr_milli = int(round(a.threshold * 1000)) if a.threshold else None
    raw = not a.no_raw
    sess = PS.run_dir("fieldtest")
    print(f"  {DIM}session: {sess}{OFF}")

    if a.rig:
        results = run_rig(port, sess, a, thr_milli)
        return finish(sess, results)

    if a.demo:
        run_demo(port, sess, a.demo, thr_milli)
        print(f"\n  saved in: {sess}")
        return 0

    if a.live:
        live_watch(port, thr_milli, sess)
        print(f"\n  saved in: {sess}")
        return 0

    results = {}

    if not a.skip_checks:
        show_off(port)

    # ---- 1. quiet -------------------------------------------------------
    say(f"STEP 1 of 3 — BE QUIET for {a.quiet_secs:.0f} seconds.")
    print("  Put the device where it will listen from. Then stay still and "
          "silent.")
    print(f"  {DIM}This measures what 'nothing happening' looks like here.{OFF}")
    wait_enter()
    fr, al, pcm, stop = run_guard(port, a.quiet_secs, "QUIET   ", raw=raw)
    results["quiet"] = summarise(fr, al)
    results["quiet"]["trace"] = save_trace(sess, "quiet", fr)
    if raw:
        results["quiet"]["raw"], rep = save_raw(sess, "quiet", pcm)
        results["quiet"]["raw_report"] = rep
    PS.manifest_append(sess, {"stage": "quiet", **results["quiet"]})
    print(f"  quiet: median {results['quiet'].get('score_median', 0):.2f}   "
          f"max {results['quiet'].get('score_max', 0):.2f}   "
          f"{explain(results['quiet'])}")
    if raw:
        say_raw(rep)
    if stop:
        return finish(sess, results)

    # ---- 2. talk --------------------------------------------------------
    say(f"STEP 2 of 3 — TALK AND CLAP for {a.talk_secs:.0f} seconds.")
    print("  Speak normally near the device, move around, clap a few times.")
    print(f"  {DIM}Speech has no propeller comb, so it should NOT alert.{OFF}")
    wait_enter()
    fr, al, pcm, stop = run_guard(port, a.talk_secs, "TALK    ", raw=raw)
    results["talk"] = summarise(fr, al)
    results["talk"]["trace"] = save_trace(sess, "talk", fr)
    if raw:
        results["talk"]["raw"], rep = save_raw(sess, "talk", pcm)
        results["talk"]["raw_report"] = rep
    PS.manifest_append(sess, {"stage": "talk", **results["talk"]})
    print(f"  talk: max {results['talk'].get('score_max', 0):.2f}   "
          f"{explain(results['talk'])}")
    if raw:
        say_raw(rep)
    if stop:
        return finish(sess, results)

    # ---- 3. rotor -------------------------------------------------------
    say("STEP 3 of 3 — THE PROPELLER.")
    print(f"{RED}  SAFETY: props up. Rig staked or ballasted. Never hand-held "
          f"while spinning.\n  Stand clear of the prop disc. Keep the Mac and "
          f"the USB lead out of the prop plane.{OFF}")
    runs = []
    dists = [x.strip() for x in a.distances.split(",") if x.strip()] \
        if a.distances else []
    for d in dists:
        say(f"Spin the propeller at {d} m for {a.rotor_secs:.0f} seconds.")
        print("  Get it to a steady speed, then hold it there.")
        wait_enter("Press Enter once it is spinning steadily")
        label = f"rotor_{d}m_{len(runs)}"
        fr, al, pcm, stop = run_guard(port, a.rotor_secs, f"ROTOR {d}m",
                                     raw=raw)
        s = summarise(fr, al)
        s["distance_m"] = d
        s["trace"] = save_trace(sess, label, fr)
        if raw:
            s["raw"], rep = save_raw(sess, label, pcm)
            s["raw_report"] = rep
        PS.manifest_append(sess, {"stage": "rotor", **s})
        runs.append(s)
        print(f"  {d} m: f0 {s.get('f0_median_above', 0) or s.get('f0_at_loudest', 0):.0f} Hz"
              f"   max score {s.get('score_max', 0):.2f}"
              f"   above-thr frames {s.get('frames_above', 0)}"
              f"   longest run {s.get('longest_run_above', 0)}")
        print(f"       {explain(s)}")
        if raw:
            say_raw(rep)
        if stop:
            results["rotor"] = runs
            return finish(sess, results)
    results["rotor"] = runs

    # ---- report ---------------------------------------------------------
    print(f"\n{BOLD}  RESULTS{OFF}")
    print(f"    quiet {a.quiet_secs:.0f}s : median score "
          f"{results['quiet'].get('score_median', 0):.2f}, "
          f"max {results['quiet'].get('score_max', 0):.2f}, "
          f"{results['quiet']['alerts']} alert(s)")
    print(f"    talking   : max score "
          f"{results['talk'].get('score_max', 0):.2f}, "
          f"{results['talk']['alerts']} alert(s)")
    if runs:
        print(f"\n    {'distance':>10}{'f0 Hz':>9}{'max score':>11}"
              f"{'alerts':>8}   detected")
        for r in runs:
            print(f"    {r['distance_m'] + ' m':>10}"
                  f"{r.get('f0_at_loudest', 0):9.0f}"
                  f"{r.get('score_max', 0):11.2f}{r['alerts']:8d}   "
                  f"{'YES' if r['alerts'] else 'no'}")
        f0s = [r.get("f0_at_loudest", 0) for r in runs if r["alerts"]]
        if f0s:
            lo, hi = min(f0s), max(f0s)
            print(f"\n  {BOLD}MEASURED ROTOR f0: {lo:.0f} - {hi:.0f} Hz{OFF}")
            print(f"  predicted for a 7in 3-blade at hover: 375-475 Hz")
            print(f"  the PROVISIONAL priority band: 200-800 Hz")
            if 200 <= lo and hi <= 800:
                print(f"  {GREEN}the measurement sits inside the band.{OFF}")
            else:
                print(f"  {YELLOW}the measurement falls OUTSIDE the band. "
                      f"That is a real finding - bring it to planning with "
                      f"this session's files.{OFF}")

    results["live"] = live_watch(port, thr_milli, sess)

    return finish(sess, results)


if __name__ == "__main__":
    sys.exit(main())
