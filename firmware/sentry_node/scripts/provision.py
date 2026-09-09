#!/usr/bin/env python
"""
provision.py - flash ONE new PCB rev A2 unit and prove what it is running.

    source ~/.espressif/tools/activate_idf_v6.0.2.sh
    python scripts/provision.py                 # flash + verify the attached board
    python scripts/provision.py --verify-only   # verify, write nothing

Why this exists rather than a bare `idf.py flash`: a flash is only finished
when the DEVICE says what it is running. This project has a written trap -
`app_init: Compile time` does NOT change across a reflash, because ESP-IDF has
no reason to recompile esp_app_desc.c - so the only proof is `ELF file SHA256`
against the local ELF. That check is the point of this script; the rest is the
fleet paperwork around it.

It ERASES before writing. A stored settings blob whose version already matches
never reaches the migration's force, so a unit that had been flashed once
before could come up carrying typed-in settings with no symptom except that
nothing changed. Erasing is how every unit is made to come up on the SHIPPED
defaults and nothing else.

Every unit is appended to captures/provisioned.csv.
"""
import argparse
import csv
import glob
import hashlib
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent                      # firmware/sentry_node
BUILD = ROOT / "build_pcb_a2"
LOG = ROOT / "captures" / "provisioned.csv"

ESP_VIDPID = [(0x303A, 0x1001)]

# The two units that have been to a field and carry settings somebody typed:
# N-73C7's threshold and t4=0, and both boards' rotation and mic calibration.
# This script ERASES, so plugging one of these in by mistake during a fleet run
# would silently throw that away. --force is the only way through.
PROTECTED = {"N-73C7", "N-F2CF"}

def _settings_version():
    """READ THE VERSION, DO NOT HARDCODE IT.

    This list used to spell "cfg v10" into eight regexes. The v11 bump then
    made every one of them fail on a unit that was in fact perfectly
    provisioned - a fleet tool that cries wolf on the day the firmware moves
    is worse than no tool. Same lesson as the settings migration list: the
    durable fix is to derive the constant, not to remember to edit it."""
    for line in (ROOT / "main" / "settings.h").read_text().splitlines():
        if line.startswith("#define SETTINGS_VERSION"):
            return "v" + line.split()[2].rstrip("u")
    raise SystemExit("SETTINGS_VERSION not found in main/settings.h")


V = _settings_version()
CFG = r"cfg " + V + " "

# What a correctly provisioned unit must report about itself. Each entry is
# (human name, regex over the device's own output).
EXPECT = [
    ("board is pcb-rev-a2",      r"board=pcb-rev-a2"),
    ("settings version " + V,    CFG),
    ("threshold 1.500",          CFG + r"thr1=1500\b"),
    ("alarm 7 s",                CFG + r".*\bburst=7000ms\b"),
    ("all four tiers armed",     CFG + r".*\bt2=1 t3=1 t4=1\b"),
    ("family {2,3} tracker on",  CFG + r".*\btrk_family=1\b"),
    ("voice/struck-note veto on", CFG + r".*\bveto=1\b"),
    ("near-field gate on",       CFG + r".*\bnf=1\b"),
    ("test mode off",            CFG + r".*\btest=0\b"),
    ("radio up at 13 dBm",       r"lora: up .*pa=BOOST 13 dBm"),
    ("tier schedule disjoint",   r"SCHED .*disjoint"),
    ("detector took the veto",   r"VETO voice/struck-note ON"),
]


def resolve_port(explicit=None):
    if explicit:
        return explicit
    c = [p.device for p in list_ports.comports() if (p.vid, p.pid) in ESP_VIDPID]
    if not c:
        c = sorted(set(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")))
    if not c:
        raise SystemExit("no ESP32-S3 serial port found - is the unit plugged in?")
    if len(c) > 1:
        raise SystemExit("more than one board attached (%s) - provision one at a "
                         "time, so the log cannot name the wrong unit" % ", ".join(c))
    return c[0]


def ports_now():
    c = [p.device for p in list_ports.comports() if (p.vid, p.pid) in ESP_VIDPID]
    if not c:
        c = sorted(set(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")))
    return sorted(c)


def wait_for_one(present=True, timeout=None):
    """Block until exactly one board is attached (or until none are)."""
    t0 = time.time()
    while True:
        n = len(ports_now())
        if present and n == 1:
            return ports_now()[0]
        if not present and n == 0:
            return None
        if timeout and time.time() - t0 > timeout:
            raise SystemExit("timed out waiting for a board")
        time.sleep(0.5)


def crc16_ccitt(b):
    """Exactly lora_crc16(): poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def esptool(port, *args, timeout=300):
    cmd = [sys.executable, "-m", "esptool", "--port", port] + list(args)
    r = subprocess.run(cmd, cwd=BUILD, capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        raise SystemExit("esptool %s failed:\n%s\n%s" % (args[0], r.stdout[-2000:], r.stderr[-2000:]))
    return r.stdout


def read_mac(port):
    out = esptool(port, "read-mac")
    m = re.findall(r"MAC:\s+([0-9a-f:]{17})", out)
    if not m:
        raise SystemExit("could not read the MAC:\n" + out)
    return m[-1]


def _drain(s, secs, stop=None):
    """Read hard for `secs`, or until `stop` matches what has arrived.

    Drains by in_waiting rather than fixed-size timed reads: the armed guard
    emits a continuous binary trace and a read that dawdles loses bytes, which
    presents as a healthy unit failing a check for a line it really printed."""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < secs:
        n = s.in_waiting
        buf += s.read(n if n else 1)
        if stop and re.search(stop, searchable(buf.decode("utf-8", "replace"))):
            break
    return buf


def capture_banner(port, secs=15.0):
    """Reset the board and listen to it boot. The banner is the only place the
    running image's SHA256 and the tier schedule are stated."""
    s = serial.Serial(port, 115200, timeout=0.05)
    s.dtr = False           # IO0 high: run, do not enter download mode
    s.rts = True            # EN low  -> in reset
    time.sleep(0.12)
    s.rts = False           # EN high -> running
    s.reset_input_buffer()
    raw = _drain(s, secs)
    s.close()
    return raw.decode("utf-8", "replace")


def capture_info(port, tries=3):
    """Ask the running guard what it is configured as.

    `I` is retried: a single line into a device that is mid-frame can go
    unanswered, and one missed line must not read as a misconfigured unit."""
    out = bytearray()
    for _ in range(tries):
        s = serial.Serial(port, 115200, timeout=0.05)
        time.sleep(0.25)
        s.reset_input_buffer()
        s.write(b"I\n")
        s.flush()
        # Stop on a COMPLETE ship line - a newline must have arrived after it.
        # Stopping at "ship v" truncated the line mid-flight and made a unit
        # whose cfg and ship are identical report that they differ.
        out += _drain(s, 6.0, stop=r"\bship v\d+ [^\n]*\n")
        s.close()
        if re.search(r"\bship v\d+ [^\n]*\n", searchable(out.decode("utf-8", "replace"))):
            break
    return out.decode("utf-8", "replace")


def searchable(raw):
    """The armed guard interleaves a BINARY trace stream with its text, so a
    text line arrives with binary glued to both ends and the newlines that
    separate them fall wherever the binary happens to contain 0x0a.

    A first version of this filtered raw.splitlines() by "mostly printable".
    That is luck, not parsing: when a long run of binary carried no 0x0a the
    real line was buried inside a 500-byte mostly-unprintable "line" and was
    dropped - which is how a healthy unit reported its tier schedule missing.

    Cutting on the binary instead is deterministic: every run of non-printable
    bytes becomes a line break, so each text fragment is a line whatever the
    stream did around it."""
    return re.sub(r"[^\x20-\x7e\n]+", "\n", raw)


def provision_one(a, port, want_sha, elf):
    print("port        %s" % port)
    print("image       %s" % elf)
    print("ELF sha256  %s" % want_sha)

    mac = read_mac(port)
    unit = "N-%04X" % crc16_ccitt(bytes.fromhex(mac.replace(":", "")))
    print("MAC         %s  ->  expect unit %s" % (mac, unit))

    if unit in PROTECTED and not a.verify_only and not a.force:
        print("\nSKIPPED  %s is a field unit and this script erases. Use --force "
              "only if you mean to lose its stored settings." % unit)
        return None

    if not a.verify_only:
        if not a.no_erase:
            print("erasing     ...", flush=True)
            esptool(port, "erase-flash")
        print("flashing    ...", flush=True)
        esptool(port, "-b", "460800", "--before", "default-reset", "--after",
                "hard-reset", "write-flash", "@flash_args")

    print("reading the device back ...", flush=True)
    banner = capture_banner(port)
    if "ELF file SHA256" not in banner:
        banner = capture_banner(port)      # one retry: a missed banner is not a fault
    blob = searchable(banner) + "\n" + searchable(capture_info(port))

    fails = []

    # THE flash proof. The compile time is not one - see the module docstring.
    m = re.search(r"ELF file SHA256:\s*([0-9a-f]+)", blob)
    if not m:
        fails.append("the device never printed its ELF SHA256")
    elif not want_sha.startswith(m.group(1).rstrip(".")):
        fails.append("running image is %s..., built image is %s..." %
                     (m.group(1)[:9], want_sha[:9]))

    if ("UNIT %s" % unit) not in blob:
        fails.append("device does not call itself %s" % unit)

    for name, pat in EXPECT:
        if not re.search(pat, blob):
            fails.append("not confirmed: " + name)

    cfg = re.search(r"\bcfg (v\d+ [^\n]*)\n", blob)
    ship = re.search(r"\bship (v\d+ [^\n]*)\n", blob)
    if not cfg or not ship:
        fails.append("`I` did not return both cfg and ship")
    elif cfg.group(1) != ship.group(1):
        fails.append("cfg and ship DIFFER - this unit needs settings typed into it")

    w4 = set(re.findall(r"T4R .*?W4=([0-9.]+)", blob))
    heap = re.search(r"heap armed: free=(\d+) largest=(\d+)", blob)

    print()
    for name, pat in EXPECT:
        print("  %-28s %s" % (name, "ok" if re.search(pat, blob) else "NOT SEEN"))
    if heap:
        print("  %-28s free=%s largest=%s" % ("heap armed", heap.group(1), heap.group(2)))
    print("  %-28s %d distinct W4 levels%s" % ("live audio", len(w4),
          " (audio is moving)" if len(w4) >= 3 else " - COULD NOT CONFIRM AUDIO"))
    if len(w4) < 3:
        fails.append("could not confirm live audio from the microphones")

    print()
    if fails:
        print("FAIL  %s" % unit)
        for f in fails:
            print("   - %s" % f)
    else:
        print("PASS  %s  running %s  on shipped defaults" % (unit, want_sha[:9]))

    LOG.parent.mkdir(parents=True, exist_ok=True)
    new = not LOG.exists()
    with LOG.open("a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(["when", "unit", "mac", "elf_sha256", "verdict",
                        "heap_free", "heap_largest", "label", "notes"])
        w.writerow([datetime.now().isoformat(timespec="seconds"), unit, mac,
                    want_sha, "FAIL" if fails else "PASS",
                    heap.group(1) if heap else "", heap.group(2) if heap else "",
                    a.label, "; ".join(fails)])

    # Leave it as the field gets it: freshly booted and arming itself.
    ser = serial.Serial(port, 115200, timeout=0.05)
    ser.dtr = False
    ser.rts = True
    time.sleep(0.12)
    ser.rts = False
    ser.close()
    return (unit, not fails)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--verify-only", action="store_true",
                    help="read and check the attached unit, write nothing")
    ap.add_argument("--no-erase", action="store_true",
                    help="skip the erase - only for a unit you flashed yourself today")
    ap.add_argument("--label", default="", help="free-text note for the log")
    ap.add_argument("--watch", action="store_true",
                    help="provision each board as it is plugged in, forever")
    ap.add_argument("--force", action="store_true",
                    help="erase even a unit on the protected field list")
    a = ap.parse_args()

    elf = BUILD / "sentry_node.elf"
    if not elf.exists():
        raise SystemExit("no build at %s - build first" % elf)
    want_sha = hashlib.sha256(elf.read_bytes()).hexdigest()

    if not a.watch:
        r = provision_one(a, resolve_port(a.port), want_sha, elf)
        print("logged to %s" % LOG)
        return 0 if (r is None or r[1]) else 1

    print("WATCH MODE - plug a unit in, wait for its verdict, unplug, repeat.")
    print("Ctrl-C to stop.  Protected from erasure: %s\n"
          % ", ".join(sorted(PROTECTED)))
    done, bad = [], []
    try:
        while True:
            print("waiting for a board ...", flush=True)
            port = wait_for_one(present=True)
            time.sleep(1.5)                     # let it finish enumerating
            print("-" * 66)
            r = provision_one(a, port, want_sha, elf)
            if r:
                (done if r[1] else bad).append(r[0])
            print("-" * 66)
            print("so far: %d passed%s" % (len(done),
                  ("  |  FAILED: " + ", ".join(bad)) if bad else ""))
            print("unplug this unit to continue ...", flush=True)
            wait_for_one(present=False)
            print()
    except KeyboardInterrupt:
        pass
    print("\npassed (%d): %s" % (len(done), ", ".join(done) or "-"))
    if bad:
        print("FAILED (%d): %s" % (len(bad), ", ".join(bad)))
    print("logged to %s" % LOG)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
