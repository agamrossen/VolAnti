#!/usr/bin/env python
"""
quad_tap.py - THE evidence tool for the four-microphone array.

    conda activate acoustic-detector
    python scripts/quad_tap.py --out captures/quad/run1
    python scripts/quad_tap.py --clap-only --runs 5 --out captures/quad/det

It answers three questions with numbers rather than opinions:

  1. MAPPING       does channel N really correspond to microphone N?
  2. BUS OFFSET    how many samples apart do the two I2S engines start?
  3. GEOMETRY      do the axes and signs match the as-built plus?

The analysis maths lives in quad_analysis.py and is proven against synthetic
signals in tests/test_quad_analysis.py, so a disagreement here is physical.

Every judgement prints PASS/FAIL with the measured number next to its
tolerance. Nothing is hidden behind a verdict.

GEOMETRY (as-built): +y = NORTH = M3 = the FORWARD / USB-cable edge, +x = EAST
= M2. E-W baseline 40.64 mm (1.90 samples), N-S 60.96 mm (2.84 samples).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import quad_analysis as qa                                        # noqa: E402
from capture_trace import open_serial, resolve_port               # noqa: E402
from trace_proto import MAGIC_END, parse                          # noqa: E402

BOLD = "\033[1m"
RED = "\033[31m"
GREEN = "\033[32m"
OFF = "\033[0m"


def verdict(ok, label, detail):
    tag = f"{GREEN}PASS{OFF}" if ok else f"{RED}FAIL{OFF}"
    print(f"  [{tag}] {label}")
    print(f"         {detail}")
    return ok


# ---------------------------------------------------------------------------
# capture
# ---------------------------------------------------------------------------
def capture(port, seconds, out: Path, quiet=False):
    """Drive `X <s>` and save the raw stream. Returns (channels, report)."""
    out.parent.mkdir(parents=True, exist_ok=True)
    budget = seconds + 30.0
    buf = bytearray()
    with open_serial(port, timeout=0.5) as s:
        s.write(f"X {int(seconds)}\n".encode())
        s.flush()
        t0 = time.time()
        last = t0
        while time.time() - t0 < budget:
            c = s.read(65536)
            if c:
                buf += c
                last = time.time()
                if MAGIC_END.to_bytes(4, "little") in buf[-8192:]:
                    time.sleep(0.2)
                    buf += s.read(s.in_waiting or 0)
                    break
            elif time.time() - last > 10.0 and buf:
                print("  stream went quiet", file=sys.stderr)
                break

    Path(str(out) + ".bin").write_bytes(bytes(buf))
    recs, bad = parse(bytes(buf))
    ch, report = qa.assemble_pcm4(recs)
    report["bad_chk"] = bad

    np.savez_compressed(str(out) + ".npz", channels=ch, fs=qa.FS)
    if not quiet:
        print(f"  wrote {out}.bin and {out}.npz")
        print(f"  records {report['n_records']}  dropped {report['dropped']}  "
              f"drop rate {report['drop_rate'] * 100:.2f}%  "
              f"contiguous runs {report['runs']}  "
              f"frames used {report.get('used_frames', 0)} "
              f"({report.get('used_frames', 0) / qa.FS:.1f}s)  bad_chk {bad}")
        if report["dropped"]:
            print(f"{RED}  !! link dropped records. The analysis uses only the "
                  f"longest contiguous run - it never splices across a gap, "
                  f"because a splice reads as an inter-channel delay.{OFF}")
    return ch, report


def countdown(msg, secs):
    for k in range(secs, 0, -1):
        sys.stdout.write(f"\r  {msg}  ... {k} ")
        sys.stdout.flush()
        time.sleep(1)
    sys.stdout.write("\r" + " " * 70 + "\r")


# ---------------------------------------------------------------------------
# protocols
# ---------------------------------------------------------------------------
FULL_SCRIPT = """
  GUIDED PROTOCOL - read it all before starting.

    0-2 s    settle. Stay quiet. (The first second is always discarded: the
             INMP441 start-up transient is a known full-scale burst.)
    2-4 s    TAP M1 (WEST) with a fingernail
    4-6 s    TAP M2 (EAST)
    6-8 s    TAP M3 (NORTH - the USB-cable edge)
    8-10 s   TAP M4 (SOUTH)
   10-13 s   ONE sharp clap, ~1 m DIRECTLY ABOVE the board centre
   13-16 s   ONE sharp clap from ~2 m due NORTH, at board height

  Overhead is deliberate: from directly above, all four capsules are
  equidistant to within about 0.02 samples, so ANY delay measured there is
  electrical (the inter-bus offset), cleanly separated from geometry.
"""

WINDOWS = [("TAP M1 (WEST)", 2.0, 4.0), ("TAP M2 (EAST)", 4.0, 6.0),
           ("TAP M3 (NORTH)", 6.0, 8.0), ("TAP M4 (SOUTH)", 8.0, 10.0)]
OVERHEAD = (10.0, 13.0)
NORTH = (13.0, 16.0)
FULL_SECONDS = 17


def slice_window(ch, t0, t1):
    return ch[:, int(t0 * qa.FS):int(t1 * qa.FS)]


def run_full(port, out: Path):
    print(FULL_SCRIPT)
    input("  Press Enter when you are ready to start the 17 s capture... ")
    print()
    print(f"  {BOLD}CAPTURING - follow the timings above.{OFF}")

    import threading
    prompts = [(0.0, "settle - stay quiet")] + \
              [(t0, name) for name, t0, _ in WINDOWS] + \
              [(OVERHEAD[0], "CLAP once, ~1 m DIRECTLY ABOVE the centre"),
               (NORTH[0], "CLAP once, ~2 m due NORTH, board height")]

    def prompter():
        start = time.time()
        for t, msg in prompts:
            wait = t - (time.time() - start)
            if wait > 0:
                time.sleep(wait)
            print(f"    >>> t={t:4.1f}s  {BOLD}{msg}{OFF}")

    th = threading.Thread(target=prompter, daemon=True)
    th.start()
    ch, report = capture(port, FULL_SECONDS, out)
    th.join(timeout=0.1)

    if ch.shape[1] < int(FULL_SECONDS * 0.5 * qa.FS):
        print(f"{RED}  capture too short to analyse "
              f"({ch.shape[1] / qa.FS:.1f}s usable){OFF}")
        return 1

    print(f"\n{BOLD}  ANALYSIS{OFF}\n")
    ok_all = True

    # 1 - mapping
    windows = [slice_window(ch, t0, t1) for _, t0, t1 in WINDOWS]
    ok, results = qa.check_mapping(windows)
    for r in results:
        got = r["got_name"] or "AMBIGUOUS/none"
        print(f"    window {r['expected_name']:<9} -> {got:<14} "
              f"peak/MAD {r['over_mad']:6.1f}  peak/next {r['over_other']:5.2f}")
    ok_all &= verdict(ok, "MAPPING  channel <-> microphone",
                      "all four taps attribute to the expected channel"
                      if ok else
                      "a tap landed on the wrong channel, or was ambiguous - "
                      "if the PAIRS are swapped, fix the four QUAD_CH_* "
                      "constants in source_i2s_quad.c")

    # 2 - overhead clap => bus offset
    seg = slice_window(ch, *OVERHEAD)
    centre = qa.find_transient(seg, skip_s=0.0)
    d_over = qa.channel_delays(seg, centre, win_ms=10.0)
    over = qa.overhead_clap_check(d_over)
    print(f"    delays vs M1 (samples): " +
          "  ".join(f"{qa.CH_NAMES[c]} {d_over[c]:+.3f}" for c in range(4)))
    ok_all &= verdict(
        over["pass"], "OVERHEAD CLAP  same-bus pairs agree",
        f"|M2-M1| = {over['ew_delta']:.3f}, |M4-M3| = {over['ns_delta']:.3f} "
        f"(tol {over['tol']})")
    offset = over["bus_offset_samples"]
    print(f"  {BOLD}  INTER-BUS OFFSET = {offset:+.3f} samples{OFF}   "
          f"(0 is the hope; any CONSTANT integer is fine - it is compensated "
          f"host-side by design)\n")

    # 3 - north clap => geometry and sign
    seg = slice_window(ch, *NORTH)
    centre = qa.find_transient(seg, skip_s=0.0)
    d_north = qa.channel_delays(seg, centre, win_ms=10.0)
    north = qa.north_clap_check(d_north, offset=offset)
    print(f"    delays vs M1 (samples): " +
          "  ".join(f"{qa.CH_NAMES[c]} {d_north[c]:+.3f}" for c in range(4)))
    ok_all &= verdict(
        north["pass"], "NORTH CLAP  geometry and sign",
        f"M4 lags M3 by {north['ns_delta']:+.3f} samples, expected "
        f"{north['expected']:.3f} +/- {north['tol']} "
        f"(sign {'correct' if north['sign_ok'] else 'INVERTED'}); "
        f"|M2-M1| = {north['ew_delta']:.3f} (tol {north['tol_ew']})")

    summary = {
        "mapping_pass": ok, "mapping": results,
        "overhead": over, "bus_offset_samples": offset,
        "north": north, "capture": report,
    }
    Path(str(out) + "_result.json").write_text(json.dumps(summary, indent=2))
    print(f"\n  wrote {out}_result.json")
    print(f"\n{BOLD}  QUAD TAP: {'PASS' if ok_all else 'FAIL'}{OFF}")
    return 0 if ok_all else 1


def run_clap_only(port, out: Path, runs: int):
    """Offset determinism across resets. The physical step is the point."""
    print(f"\n  OFFSET DETERMINISM - {runs} runs, board RESET between each.\n"
          f"  Each run: one clap ~1 m DIRECTLY ABOVE the board centre.\n")
    offsets = []
    for i in range(runs):
        print(f"  {BOLD}run {i + 1}/{runs}{OFF}")
        if i:
            print("    !! PHYSICAL: press the board's RST button now, wait for "
                  "it to come back.")
        input("    Press Enter when ready, then clap once when it says CLAP... ")
        countdown("capturing - get ready", 2)
        print(f"    >>> {BOLD}CLAP NOW (once, overhead){OFF}")
        ch, rep = capture(port, 4, Path(f"{out}_run{i + 1}"), quiet=True)
        if ch.shape[1] < qa.FS:
            print(f"{RED}    capture too short, skipping this run{OFF}")
            continue
        centre = qa.find_transient(ch, skip_s=0.5)
        d = qa.channel_delays(ch, centre, win_ms=10.0)
        off = qa.bus_offset(d)
        offsets.append(off)
        print(f"    offset {off:+.3f} samples   "
              f"(drop rate {rep['drop_rate'] * 100:.2f}%)")

    print("\n  offset table:")
    for i, o in enumerate(offsets, 1):
        print(f"    run {i}: {o:+.3f}")
    r = qa.offset_determinism(offsets)
    ok = verdict(r["pass"], "OFFSET DETERMINISM across resets",
                 f"spread {r['spread']:.3f} samples (tol {r['tol']}), "
                 f"mean {r['mean']:+.3f}")
    if not ok:
        print(f"\n{RED}  STOP - bring this to planning.{OFF}\n"
              f"  A varying offset means the two RX engines can start on "
              f"different WS edges. That is a design question, not a bench "
              f"fix, and no compensation constant should be invented here.")
    Path(str(out) + "_determinism.json").write_text(json.dumps(r, indent=2))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="captures/quad/tap")
    ap.add_argument("--port", default=None)
    ap.add_argument("--clap-only", action="store_true")
    ap.add_argument("--runs", type=int, default=5)
    # GEOMETRY IS DATA (the port, D12): the lead orders and the expected
    # sample delays this test checks come from the named profile, so a new
    # board is a flag rather than an edit here.
    import geometry as _geom
    _geom.add_argument(ap)
    a = ap.parse_args()

    qa.use_geometry(a.geometry)
    port = resolve_port(a.port)
    print(f"port {port}")
    print(f"geometry {a.geometry}: E-W {qa.MAX_DELAY_EW:.2f} / "
          f"N-S {qa.MAX_DELAY_NS:.2f} samples")
    out = Path(a.out)
    if a.clap_only:
        return run_clap_only(port, out, a.runs)
    return run_full(port, out)


if __name__ == "__main__":
    sys.exit(main())
