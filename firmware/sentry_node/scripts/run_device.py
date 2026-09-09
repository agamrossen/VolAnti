#!/usr/bin/env python
"""
run_device.py - RUN THE WHOLE DEVICE. One command, Ctrl-C to stop.

    conda activate acoustic-detector
    cd ~/acoustic-detector/firmware/sentry_node
    python scripts/run_device.py

This is not a test of a part. It is the device behaving as a device: four
microphones, the committed detector at the deployment threshold, and every
output wired up - buzzer, vibration motor, white LED, e-paper - with the snooze
button silencing the outputs without ever gating detection.

THE ALARM MANAGES ITSELF. One detection gives 3 s of outputs and then 5 s of
enforced silence before anything can fire again (ALERT_MAX_MS / COOLDOWN_MS in
alert_ui.h). A continuous siren at arm's length is unusable; a burst is
unmissable and still lets you think. The button adds 10 s of silence on demand,
from any state, and a 2 s hold powers the device down with OFF on the panel.
None of it gates DETECTION - the records keep coming throughout, which is why
the trace is worth keeping even through a burst.

  Known deployment question, deliberately unresolved: 3 s on / 5 s off means a
  sustained real attack is silent 5 of every 8 seconds. Right for a bench,
  arguable for a hilltop. Not a decision this script gets to make.

EVERY FRAME IS ARCHIVED to captures/<date>_<time>_device/ unless --no-save.
This mode is the one left running while you experiment, so it is the one that
will be running the first time something surprising happens.

WHAT RUNS WHERE
  on the device   the alarm itself: buzzer, motor, LED, and the e-paper showing
                  LISTENING or ALERT. That is the product.
  in this terminal a live status line and a log of every alert, so you can see
                  WHY it did what it did.

THE FOUR MICROPHONES ARE SUMMED (a broadside beam - the existing combiner's
N>1 path). That is roughly +6 dB on a source arriving equally at all four
capsules, against uncorrelated noise. It is NOT steered beamforming and it
produces NO bearing.

HONEST LIMIT: every threshold in this project was calibrated on ONE microphone
and on synthetic audio. Summing four changes the signal-to-floor relationship
in a way that has never been measured. Treat what you see as qualitative until
there is a bench measurement with the real airframe.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import profile_store as PS                                         # noqa: E402
from capture_trace import open_serial, resolve_port                # noqa: E402
from trace_proto import parse_stream                               # noqa: E402

BOLD = "\033[1m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
DIM = "\033[2m"
OFF = "\033[0m"

HOP_S = 512 / 16000.0          # 32 ms


def bar(v, vmax=3.0, width=26):
    n = 0 if vmax <= 0 else int(width * min(1.0, max(0.0, v) / vmax))
    return "#" * n + "-" * (width - n)


def banner(port, thr):
    print()
    print(f"{BOLD}  SENTRY-NODE - full device run{OFF}")
    print(f"  port {port}")
    print(f"  4 microphones, summed (broadside). Threshold {thr:.4f} "
          f"(HIGH_ALERT, the deployment default).")
    print()
    print(f"  {BOLD}On the device:{OFF} buzzer + motor + white LED on alert; "
          f"e-paper shows LISTENING / ALERT.")
    print(f"  {BOLD}The alarm silences ITSELF:{OFF} 3 s of outputs, then 5 s "
          f"of enforced quiet")
    print(f"    before anything can fire again. You do not have to touch it.")
    print(f"  {BOLD}Snooze button:{OFF} 10 s of silence on demand, if 3 s is "
          f"still too much.")
    print(f"  {BOLD}Detection NEVER pauses{OFF} — not during the burst, not "
          f"during the cooldown,")
    print(f"    not while snoozed. Only the OUTPUTS are gated; the records "
          f"keep coming.")
    print(f"  {BOLD}Ctrl-C{OFF} to stop.")
    print()
    print(f"{DIM}  Speaker playback and bench sources prove plumbing only - no "
          f"range or detection-probability claim follows from anything you see "
          f"here.{OFF}")
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--quiet", action="store_true",
                    help="only print alerts and warnings, no status line")
    ap.add_argument("--profile", default=None,
                    help="'latest' or a board MAC: run at that board's "
                         "CALIBRATED threshold instead of the sealed default")
    ap.add_argument("--threshold", type=float, default=None,
                    help="explicit operating threshold, overrides --profile")
    ap.add_argument("--no-save", action="store_true",
                    help="do not archive the session (default is to keep "
                         "every frame under captures/)")
    # ---- the port: Tier-2. ALL OPTIONAL. -------------------------------
    # Bare `python scripts/run_device.py` still sends `G` and prints exactly
    # what it printed yesterday. Every line below is reached only when a flag
    # asks for it, because the whole point of the mode is that it is the one
    # left running unattended - and a mode whose output moved under it is a
    # mode whose history stops being comparable.
    ap.add_argument("--tier2", action="store_true",
                    help="send `H` instead of `G`: the same guard, plus the "
                         "slow-floor Tier-2 beside it. The alert is the OR "
                         "of the two tiers.")
    ap.add_argument("--tier3", action="store_true",
                    help="ALSO run Tier-3, the wash/envelope tier. READ THIS "
                         "BEFORE USING IT: on the only real propeller audio "
                         "this project owns Tier-3 is the ONLY tier that "
                         "detects anything at all, and its calibration FAILED "
                         "at 4.87 weighted FA/h against a 0.40 allowance. It "
                         "fires on fans, insects, HVAC and vehicles. Expect "
                         "nuisance alarms and read the tier on every one.")
    ap.add_argument("--thr3", type=float, default=None,
                    help="Tier-3 threshold tau3 (default 20.0, quoted against "
                         "a measured null of W=11.0)")
    ap.add_argument("--attach", action="store_true",
                    help="send NO command: attach to a device already running "
                         "its own standalone guard and just watch. This is the "
                         "field-build monitor.")
    ap.add_argument("--thr2", type=float, default=None,
                    help="Tier-2 threshold (default: the calibrated one "
                         "compiled into the firmware)")
    ap.add_argument("--cx", choices=["a", "b", "c", "d"], default="a",
                    help="combiner: a complex sum (SHIPPED DEFAULT) | "
                         "b incoherent RMS | c hybrid | d offset-compensated")
    ap.add_argument("--busoff", type=float, default=0.0,
                    help="CX-D only: measured bus-B lag in samples. Stays 0 "
                         "until the bench proves the offset is constant "
                         "across resets.")
    ap.add_argument("--t2-band", default=None, metavar="LO:HI",
                    help="Tier-2 search band in Hz, e.g. --t2-band 500:1600. "
                         "Runtime, so a BAND SHIFTED verdict on the rig day "
                         "is a flag rather than a rebuild. Default 200:800.")
    ap.add_argument("--exclude", action="append", default=[],
                    metavar="F0:TOL",
                    help="persistent-source exclusion band, e.g. "
                         "--exclude 347:8 (repeatable, max 4). Applies to "
                         "Tier-2 hit counting only; v1 never sees it and "
                         "nothing self-learns.")
    rate = ap.add_mutually_exclusive_group()
    rate.add_argument("--t2-half-rate", action="store_true",
                      help="run Tier-2 every second frame (same seconds of "
                           "integration, half the cost)")
    rate.add_argument("--t2-full-rate", action="store_true")
    a = ap.parse_args()

    port = resolve_port(a.port)

    # P1: calibration lives on the Mac. The threshold is passed to the device
    # at RUNTIME, so recalibrating never needs a rebuild and therefore never
    # forces a golden re-proof.
    thr = None
    src = "the sealed deployment default"
    if a.threshold:
        thr, src = a.threshold, "--threshold"
    elif a.profile:
        prof = (PS.latest() if a.profile == "latest"
                else PS.load(a.profile))
        if prof and prof.get("recommended_threshold"):
            thr = float(prof["recommended_threshold"])
            src = f"profile {prof['board_mac']} ({prof.get('updated', '?')})"
        else:
            print(f"  {YELLOW}no calibrated threshold in that profile - "
                  f"falling back to the sealed default{OFF}")
    if a.tier2:
        # H [thr1_milli] [thr2_milli] [cx] [busoff_milli] [rate] [lo] [hi]
        # 0 means "keep the compiled default" for every numeric argument.
        rate_arg = 2 if a.t2_half_rate else (1 if a.t2_full_rate else 0)
        blo = bhi = 0
        if a.t2_band:
            try:
                blo, bhi = (int(round(float(v)))
                            for v in a.t2_band.split(":", 1))
            except Exception:
                print(f"{RED}  --t2-band wants LO:HI in Hz, e.g. 500:1600{OFF}")
                return 2
        #... and an eighth: Tier-3's tau3 in milli-units, 0 = off.
        t3_arg = 0
        if a.tier3:
            t3_arg = int(round((a.thr3 if a.thr3 is not None else 20.0) * 1000))
        cmd = (f"H {0 if thr is None else int(round(thr * 1000))} "
               f"{0 if a.thr2 is None else int(round(a.thr2 * 1000))} "
               f"{a.cx} {int(round(a.busoff * 1000))} {rate_arg} "
               f"{blo} {bhi} {t3_arg}\n").encode()
    else:
        cmd = (b"G\n" if thr is None
               else f"G {int(round(thr * 1000))}\n".encode())
    if a.attach:
        # A standalone device is ALREADY guarding. Sending anything would stop
        # it and hand the console back, which is the opposite of watching it.
        cmd = b""

    st = {"frames": 0, "alerts": 0, "peak": 0.0, "chain": 0,
          "last_draw": 0.0, "thr": 1.70, "alert_state": "idle",
          "snooze_ms": 0, "presses": 0, "epd": "?", "started": time.time(),
          "no_data_warned": False}
    # EVERY FRAME IS KEPT. This is the mode people leave running while they
    # play, which makes it the mode that is running the first time something
    # unexpected happens. On the equivalent mode produced this
    # project's first real propeller alerts and archived nothing, so the
    # milestone survived as terminal scrollback and had to be reconstructed
    # from memory. Not again.
    rec_frames, alert_recs = [], []
    t2_frames, t2_alerts = [], []
    t3_frames, t3_alerts = [], []
    st.update(t2_alerts=0, t2_hits=0, t2_n2=0, t2_peak=0.0, t2_score=0.0,
              t2_f02=0.0, t2_kappa=float("nan"), t2_fired=0)
    st.update(t3_alerts=0, t3_W=0.0, t3_r=0.0, t3_hits=0, t3_n3=0,
              t3_peak=0.0, t3_frozen=0)
    sess = None if a.no_save else PS.run_dir("device")

    if thr:
        st["thr"] = thr
    banner(port, st["thr"])
    print(f"  threshold source: {src}\n")

    with open_serial(port, timeout=0.2) as s:
        # The exclusion list is a SEPARATE command that must land before the
        # guard starts, because the guard never returns until it is stopped.
        if a.exclude:
            try:
                pairs = [tuple(int(round(float(v))) for v in e.split(":", 1))
                         for e in a.exclude]
            except Exception:
                print(f"{RED}  --exclude wants F0:TOL in Hz, e.g. 347:8{OFF}")
                return 2
            if len(pairs) > 4:
                print(f"{RED}  at most 4 exclusion bands{OFF}")
                return 2
            line = "E %d %s\n" % (len(pairs),
                                  " ".join(f"{f} {t}" for f, t in pairs))
            s.write(line.encode())
            s.flush()
            time.sleep(0.3)
            s.read(4096)
            print("  excluding: " + ", ".join(f"{f}+-{t} Hz"
                                              for f, t in pairs))
        if cmd:
            s.write(cmd)
            s.flush()
        else:
            print(f"  {GREEN}attached{OFF} - watching a device that is "
                  f"running its own guard. Ctrl-C detaches WITHOUT stopping "
                  f"it.\n")
        buf = bytearray()
        try:
            while True:
                chunk = s.read(65536)
                if not chunk:
                    # 3 s with nothing at all means the device never started
                    if (not st["no_data_warned"] and st["frames"] == 0
                            and time.time() - st["started"] > 5.0):
                        st["no_data_warned"] = True
                        print(f"{RED}  !! no data. Either a microphone bus is "
                              f"not clocking, or the board is stuck in another "
                              f"mode. Try: press RST, then re-run.{OFF}")
                    continue
                buf += chunk
                recs, _, used = parse_stream(bytes(buf))
                for r in recs:
                    k = r["_kind"]

                    if k == "hdr":
                        st["thr"] = r["threshold"]

                    elif k == "rec":
                        st["frames"] += 1
                        rec_frames.append(r)
                        st["peak"] = max(st["peak"], r["score"])
                        st["chain"] = max(st["chain"], r["chain"])
                        now = time.time()
                        if not a.quiet and now - st["last_draw"] > 0.2:
                            st["last_draw"] = now
                            up = st["frames"] * HOP_S
                            tag = (f"{RED}ALERT{OFF}"
                                   if st["alert_state"] == "alerting"
                                   else (f"{YELLOW}snoozed "
                                         f"{st['snooze_ms'] / 1000:.0f}s{OFF}"
                                         if st["alert_state"] == "snoozed"
                                         else f"{GREEN}listening{OFF}"))
                            line = (f"\r  {up:7.1f}s "
                                    f"|{bar(r['score'], 3.0)}| "
                                    f"score {r['score']:6.3f}  "
                                    f"f0 {r['f0_hz']:7.1f}Hz  "
                                    f"chain {r['chain']:2d}"
                                    f"  alerts {st['alerts']:3d}  {tag}   ")
                            if a.tier2:
                                # Tier-2's own bar, its own count, and the
                                # coherence it is NOT gating on. Two tiers, two
                                # rows of evidence: an operator watching this
                                # must be able to say WHICH one is talking.
                                k = st["t2_kappa"]
                                line = (f"\r  {up:7.1f}s "
                                        f"|{bar(r['score'], 3.0)}| "
                                        f"v1 {r['score']:5.2f} "
                                        f"f0 {r['f0_hz']:6.1f} ch{r['chain']:2d}"
                                        f"   |{bar(st['t2_score'], 3.0, 16)}| "
                                        f"T2 {st['t2_score']:5.2f} "
                                        f"f0 {st['t2_f02']:6.1f} "
                                        f"{st['t2_hits']:3d}/{st['t2_n2']:<3d}"
                                        f" k{k:4.2f}"
                                        f"  A{st['alerts']}+{st['t2_alerts']}"
                                        f" {tag}  ")
                            if a.tier3:
                                # Tier-3's row: the statistic, the rate it
                                # won on, and whether it is FROZEN - which it
                                # is whenever the device's own buzzer or motor
                                # is running, because an ERM at 100-200 Hz is
                                # exactly what this tier hunts.
                                line = line.rstrip() + (
                                    f"  |T3 {st['t3_W']:5.1f} "
                                    f"r {st['t3_r']:5.1f} "
                                    f"{st['t3_hits']:2d}/{st['t3_n3']:<2d}"
                                    f"{' FRZ' if st['t3_frozen'] else ''}"
                                    f" A{st['t3_alerts']}  ")
                            sys.stdout.write(line)
                            sys.stdout.flush()

                    elif k == "alt":
                        st["alerts"] += 1
                        alert_recs.append(r)
                        print(f"\n{RED}{BOLD}  *** ALERT #{st['alerts']}  "
                              f"t={r['t_s']:.2f}s  f0={r['f0_hz']:.1f} Hz  "
                              f"score={r['score']:.3f}  chain={r['chain']} "
                              f"***{OFF}")
                        print(f"     buzzer + motor + white LED on, e-paper -> "
                              f"ALERT. Silences itself after 3 s and the panel "
                              f"goes back to LISTENING.")

                    elif k == "t2r":
                        t2_frames.append(r)
                        st["t2_score"] = r["score2"]
                        st["t2_f02"] = r["f02_hz"]
                        st["t2_hits"] = r["hits"]
                        st["t2_n2"] = r["n2"]
                        st["t2_kappa"] = r["kappa"]
                        st["t2_fired"] = r["fired2"]
                        st["t2_peak"] = max(st["t2_peak"], r["score2"])

                    elif k == "al2":
                        st["t2_alerts"] += 1
                        t2_alerts.append(r)
                        print(f"\n{RED}{BOLD}  *** TIER-2 ALERT "
                              f"#{st['t2_alerts']}  t={r['t_s']:.2f}s  "
                              f"f0={r['f02_hz']:.1f} Hz  "
                              f"score2={r['score2']:.3f}  "
                              f"hits={r['hits']}  kappa={r['kappa']:.2f} "
                              f"***{OFF}")
                        print(f"     Tier-2 integrated a STEADY source for "
                              f"~{r['hits'] * HOP_S:.1f} s. Same buzzer, same "
                              f"dismissal - the operator is not told which "
                              f"tier heard it.")

                    elif k == "t3r":
                        t3_frames.append(r)
                        st["t3_W"] = r["W"]
                        st["t3_r"] = r["r_hz"]
                        st["t3_hits"] = r["hits"]
                        st["t3_n3"] = r["n3"]
                        st["t3_frozen"] = r["frozen"]
                        st["t3_peak"] = max(st["t3_peak"], r["W"])

                    elif k == "al3":
                        st["t3_alerts"] += 1
                        t3_alerts.append(r)
                        print(f"\n{RED}{BOLD}  *** TIER-3 (WASH) ALERT "
                              f"#{st['t3_alerts']}  t={r['t_s']:.2f}s  "
                              f"rate={r['r_hz']:.1f} Hz  W={r['W']:.1f}  "
                              f"hits={r['hits']} ***{OFF}")
                        print(f"     A shaft rate of {r['r_hz']:.0f} Hz "
                              f"implies {r['r_hz'] * 3:.0f} Hz blade-pass on "
                              f"three blades. {YELLOW}A FAN IS THE SAME "
                              f"MACHINE{OFF}: this tier cannot separate a "
                              f"rotor from a fan, an insect or a vehicle.")

                    elif k == "sta":
                        prev = st["alert_state"]
                        st["alert_state"] = r["alert_name"]
                        st["snooze_ms"] = r["snooze_remaining_ms"]
                        st["epd"] = r["epaper_name"]
                        if r["press_count"] != st["presses"]:
                            st["presses"] = r["press_count"]
                            print(f"\n{YELLOW}  [button] press #{r['press_count']}"
                                  f"  -> {r['alert_name']}{OFF}")
                        if prev != r["alert_name"] and r["alert_name"] == "idle" \
                                and prev == "snoozed":
                            print(f"\n{GREEN}  [re-armed] snooze expired - "
                                  f"outputs will fire on the next detection"
                                  f"{OFF}")
                        if r["mode_chr"] == "G" and r["uptime_ms"] and \
                                st["frames"] == 0:
                            print(f"{RED}  !! device reports an I2S read "
                                  f"timeout - a microphone bus is not "
                                  f"clocking{OFF}")

                    elif k == "end":
                        print("\n  [device stopped]")
                        raise KeyboardInterrupt
                buf = buf[used:]

        except KeyboardInterrupt:
            print("\n  stopping...")
        finally:
            # ATTACHED MODE LEAVES THE DEVICE RUNNING. Detaching from a
            # standalone guard must not silence it: the whole point of the
            # field build is that the box keeps working when the laptop walks
            # away.
            if not a.attach:
                try:
                    s.write(b"\n")       # any byte stops the mode
                    s.flush()
                    t1 = time.time()
                    while time.time() - t1 < 3.0:
                        if not s.read(4096):
                            break
                except Exception:
                    pass

    run_s = st["frames"] * HOP_S
    print(f"\n{BOLD}  session summary{OFF}")
    print(f"    ran {run_s:.1f} s of audio ({st['frames']} frames)")
    print(f"    alerts {st['alerts']}   peak score {st['peak']:.3f}   "
          f"longest chain {st['chain']}   button presses {st['presses']}")
    # THE FRAME BUDGET. With three tiers running this stops being a curiosity:
    # the hop is 32 ms and everything - four FFTs, the v1 comb, Tier-2's second
    # floor, Tier-3's biquads and envelope transform - has to fit inside one.
    # us_frame is measured on the device around the whole per-frame chain, so
    # this is the number that says whether the field build is real-time. It
    # EXCLUDES Tier-3 on the frames where the tier does not update, which is
    # three frames in four, so read p99 and max, not the mean.
    if rec_frames:
        us = sorted(r["us_frame"] for r in rec_frames)
        p50 = us[len(us) // 2] / 1000
        p99 = us[min(len(us) - 1, int(0.99 * len(us)))] / 1000
        mx = us[-1] / 1000
        verdict = (f"{GREEN}fits{OFF}" if p99 < 32.0
                   else f"{RED}OVER THE 32 ms HOP{OFF}")
        print(f"    frame time: median {p50:.1f} ms  p99 {p99:.1f} ms  "
              f"max {mx:.1f} ms  of 32 ms  -> {verdict}")
    if t3_frames:
        Ws = sorted(r["W"] for r in t3_frames)
        print(f"    TIER-3: {len(t3_frames)} updates   alerts "
              f"{st['t3_alerts']}   peak W {st['t3_peak']:.1f}   "
              f"median W {Ws[len(Ws) // 2]:.1f}   "
              f"(null is 11.0, tau3 "
              f"{a.thr3 if a.thr3 is not None else 20.0:.1f})")
        us = sorted(r["us_t3"] for r in t3_frames)
        print(f"    TIER-3 cost: mean {sum(us) / len(us) / 1000:.2f} ms  p99 "
              f"{us[min(len(us) - 1, int(0.99 * len(us)))] / 1000:.2f} ms "
              f"per update (one update in four frames)")
        frz = sum(1 for r in t3_frames if r["frozen"])
        if frz:
            print(f"    TIER-3 frozen on {frz} of {len(t3_frames)} updates "
                  f"(the device's own outputs were running)")
    if a.tier2:
        ks = [r["kappa"] for r in t2_frames
              if r["kappa"] == r["kappa"]]          # drop NaN
        print(f"    TIER-2: {len(t2_frames)} steps   alerts "
              f"{st['t2_alerts']}   peak score2 {st['t2_peak']:.3f}   "
              f"kappa median "
              f"{(sorted(ks)[len(ks) // 2] if ks else float('nan')):.3f}")
        if t2_frames:
            us = sorted(r["us_t2"] for r in t2_frames)
            print(f"    TIER-2 cost: mean "
                  f"{sum(us) / len(us) / 1000:.2f} ms  p99 "
                  f"{us[min(len(us) - 1, int(0.99 * len(us)))] / 1000:.2f} ms"
                  f"   (of the 32.0 ms hop)")

    if sess is not None and rec_frames:
        stamp = time.strftime("%H%M%S")
        p = sess / f"device_{stamp}_trace.npz"
        np.savez_compressed(
            p,
            score=np.array([f["score"] for f in rec_frames], np.float32),
            f0=np.array([f["f0_hz"] for f in rec_frames], np.float32),
            teeth=np.array([f["teeth"] for f in rec_frames], np.int16),
            chain=np.array([f["chain"] for f in rec_frames], np.int32),
            above=np.array([f["above_thr"] for f in rec_frames], np.int8),
            accepted=np.array([f["cont_accepted"] for f in rec_frames],
                              np.int8),
            **({} if not t2_frames else {
                "t2_score": np.array([f["score2"] for f in t2_frames],
                                     np.float32),
                "t2_f0": np.array([f["f02_hz"] for f in t2_frames],
                                  np.float32),
                "t2_hits": np.array([f["hits"] for f in t2_frames], np.int32),
                "t2_fired": np.array([f["fired2"] for f in t2_frames],
                                     np.int8),
                "t2_kappa": np.array([f["kappa"] for f in t2_frames],
                                     np.float32),
                "t2_frame": np.array([f["frame"] for f in t2_frames],
                                     np.int32)}))
        entry = {"stage": "device_run_t2" if a.tier2 else "device_run",
                 "file": p.name,
                 "frames": st["frames"], "seconds": run_s,
                 "threshold": st["thr"], "alerts": st["alerts"],
                 "peak_score": st["peak"], "longest_chain": st["chain"],
                 "button_presses": st["presses"],
                 "us_frame_p99": (
                     sorted(r["us_frame"] for r in rec_frames)[
                         min(len(rec_frames) - 1,
                             int(0.99 * len(rec_frames)))]
                     if rec_frames else 0),
                 "alert_f0_hz": [round(r["f0_hz"], 1) for r in alert_recs],
                 "alert_score": [round(r["score"], 3) for r in alert_recs],
                 "alert_chain": [int(r["chain"]) for r in alert_recs]}
        if a.tier2:
            entry.update(
                tier2=True, cx=a.cx, busoff_samples=a.busoff,
                t2_steps=len(t2_frames), t2_alerts=st["t2_alerts"],
                t2_peak_score=round(st["t2_peak"], 3),
                t2_alert_f0_hz=[round(r["f02_hz"], 1) for r in t2_alerts],
                t2_alert_score=[round(r["score2"], 3) for r in t2_alerts],
                t2_alert_kappa=[round(r["kappa"], 3) for r in t2_alerts])
        if a.tier3 or t3_frames:
            entry.update(
                tier3=True,
                t3_updates=len(t3_frames), t3_alerts=st["t3_alerts"],
                t3_peak_W=round(st["t3_peak"], 2),
                t3_alert_rate_hz=[round(r["r_hz"], 1) for r in t3_alerts],
                t3_alert_W=[round(r["W"], 2) for r in t3_alerts])
        PS.manifest_append(sess, entry)
        print(f"    {GREEN}saved{OFF} {p}")

    if st["alerts"]:
        f0s = [r["f0_hz"] for r in alert_recs]
        lo, hi = min(f0s), max(f0s)
        inside = sum(1 for f in f0s if 375 <= f <= 475)
        print(f"\n    alerting f0: {lo:.0f}"
              + (f" - {hi:.0f} Hz" if hi != lo else " Hz"))
        print(f"    {inside} of {len(f0s)} inside the 375-475 Hz predicted "
              f"hover band (PROVISIONAL).")
        print(f"    {DIM}An alert means the comb detector fired. Write down "
              f"what was making noise - that is the evidence. The trace above "
              f"is the rest of it.{OFF}")
    else:
        print(f"    {DIM}No alert. The trace is still saved: a quiet run is "
              f"the negative evidence the false-alarm budget is made of.{OFF}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
