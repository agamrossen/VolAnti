#!/usr/bin/env python
"""
capture_trace.py - drive one golden-vector replay and save the raw stream.

Rules this script exists to obey:
  * the serial port is RESOLVED AT RUNTIME, every time. Never hardcoded, never
    remembered between runs - a device that re-enumerates on a different node
    must not silently capture nothing.
  * it EXITS. Nothing here blocks forever; there is a hard per-vector timeout
    (default 120 s) and the read loop ends at the sentinel record.

Usage:
  capture_trace.py --index 0 --out captures/run1/golden_strong_pos.bin
  capture_trace.py --list
"""
import argparse
import glob
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

sys.path.insert(0, str(Path(__file__).resolve().parent))
from trace_proto import MAGIC_END, parse  # noqa: E402

# ESP32-S3 native USB-Serial-JTAG enumerates with this VID:PID.
ESP_VIDPID = [(0x303A, 0x1001)]


def resolve_port(explicit=None):
    """Find the board now. Prefers a matching USB VID:PID, falls back to any
    usbmodem/usbserial node. Raises if the answer is ambiguous or absent."""
    if explicit:
        return explicit
    cands = [p.device for p in list_ports.comports()
             if (p.vid, p.pid) in ESP_VIDPID]
    if not cands:
        cands = sorted(set(glob.glob("/dev/cu.usbmodem*")
                           + glob.glob("/dev/cu.usbserial*")
                           + glob.glob("/dev/ttyUSB*")
                           + glob.glob("/dev/ttyACM*")))
    if not cands:
        raise SystemExit("no ESP32-S3 serial port found")
    if len(cands) > 1:
        raise SystemExit(f"ambiguous serial ports {cands}; pass --port")
    return cands[0]


def open_serial(port, timeout=0.2, settle=0.35):
    """Open the port WITHOUT resetting the board.

    pyserial asserts DTR and RTS at open. On the ESP32-S3 those lines drive
    EN and IO0, so a default open can hold the chip in reset for as long as the
    port is open - the board then sends exactly zero bytes and looks dead. The
    states must be set on the object BEFORE open() so they are applied as the
    port comes up, never toggled afterwards."""
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = timeout
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(settle)
    s.reset_input_buffer()
    s.reset_output_buffer()
    s.read(s.in_waiting or 0)          # drop any boot banner
    return s


def capture(port, cmd, timeout_s, settle=0.35):
    """Send one command, read until the sentinel or the hard timeout."""
    buf = bytearray()
    with open_serial(port, timeout=0.2, settle=settle) as s:
        s.write((cmd + "\n").encode())
        s.flush()
        t0 = time.time()
        last = t0
        while time.time() - t0 < timeout_s:
            chunk = s.read(65536)
            if chunk:
                buf += chunk
                last = time.time()
                if MAGIC_END.to_bytes(4, "little") in buf[-4096:]:
                    time.sleep(0.15)
                    buf += s.read(s.in_waiting or 0)
                    return bytes(buf), True
            elif time.time() - last > 10.0 and buf:
                break                       # stream died mid-run
    return bytes(buf), False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--index", type=int, default=None)
    ap.add_argument("--probe", default=None,
                    help="'from,to' frame range for forensic probe records")
    ap.add_argument("--out", default=None)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--list", action="store_true")
    # ---- the port. ADDITIVE: without it, this tool sends `R <i>` exactly as
    # it always has, and the capture is byte-for-byte the one the golden gate
    # has always judged. ------------------------------------------------
    ap.add_argument("--tier2", action="store_true",
                    help="send `K <i>` instead of `R <i>`: the same vector "
                         "through the same replay machinery, with Tier-2 "
                         "running beside it. Judge with "
                         "`compare_trace.py --tier2`.")
    ap.add_argument("--thr2", type=float, default=None,
                    help="K mode only: override Tier-2's threshold")
    ap.add_argument("--t2-full-rate", action="store_true",
                    help="K mode only: run Tier-2 on every frame")
    a = ap.parse_args()

    port = resolve_port(a.port)
    print(f"port {port}")

    if a.list or a.index is None:
        raw, _ = capture(port, "I", 10.0)
        sys.stdout.write(raw.decode("utf-8", "replace"))
        return 0

    cmd = f"R {a.index}"
    if a.probe:
        f, t = a.probe.split(",")
        cmd = f"P {a.index} {f} {t}"
    elif a.tier2:
        m2 = 0 if a.thr2 is None else int(round(a.thr2 * 1000))
        cmd = f"K {a.index} {m2} {1 if a.t2_full_rate else 0}"

    t0 = time.time()
    raw, saw_end = capture(port, cmd, a.timeout)
    dt = time.time() - t0
    recs, bad = parse(raw)
    hdr = next((r for r in recs if r["_kind"] == "hdr"), None)
    end = next((r for r in recs if r["_kind"] == "end"), None)
    n = sum(1 for r in recs if r["_kind"] == "rec")

    if a.out:
        p = Path(a.out)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(raw)

    name = hdr["name"] if hdr else "?"
    print(f"cmd '{cmd}'  {len(raw)} bytes in {dt:.1f}s  vector={name}  "
          f"frames={n}/{hdr['n_frames'] if hdr else '?'}  "
          f"sentinel={'yes' if end else 'NO'}  bad_chk={bad}")
    if not saw_end or end is None:
        print("FAIL no sentinel - capture incomplete", file=sys.stderr)
        return 2
    if n != hdr["n_frames"]:
        print(f"FAIL {hdr['n_frames'] - n} records lost", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
