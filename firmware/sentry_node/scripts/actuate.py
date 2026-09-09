#!/usr/bin/env python
"""
actuate.py - drive the alert outputs one at a time, and read back what happened.

    conda activate acoustic-detector
    python scripts/actuate.py b1            buzzer on
    python scripts/actuate.py b0            buzzer off
    python scripts/actuate.py v1 | v0       vibration motor on / off
    python scripts/actuate.py led 255 0 0   LED raw channel values (WIRE order)
    python scripts/actuate.py epaper init|ready|alert|clear [--rot N]
    python scripts/actuate.py status        one STAT snapshot, in English
    python scripts/actuate.py drill 120     alert drill, STATs narrated live

LED VALUES ARE RAW WIRE ORDER, not colour names. WS2812 parts ship in GRB and
RGB orderings and which one is seated on this board is a bench observation:
send 255 0 0, look, then 0 255 0, then 0 0 255, and write down what you saw.
The alert colour is white, which is identical under either ordering, so this
discovery never forces a rebuild.

Every command prints the device's own ACK. Silence is never treated as success.
"""
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from capture_trace import open_serial, resolve_port                # noqa: E402
from trace_proto import parse_stream                               # noqa: E402

ACK_TIMEOUT_S = 4.0


def describe_stat(r):
    on = []
    if r["buzzer"]:
        on.append("buzzer")
    if r["motor"]:
        on.append("motor")
    if r["led_cmd"]:
        on.append(f"LED({r['led_c0']},{r['led_c1']},{r['led_c2']})")
    outputs = ", ".join(on) if on else "all outputs off"
    snooze = (f", snooze {r['snooze_remaining_ms'] / 1000:.0f}s left"
              if r["snooze_remaining_ms"] else "")
    return (f"  [{r['uptime_ms'] / 1000:8.1f}s] mode {r['mode_chr']}  "
            f"alert={r['alert_name']}{snooze}  {outputs}  "
            f"e-paper={r['epaper_name']}  button={'DOWN' if not r['button_level'] else 'up'}"
            f"  presses={r['press_count']}")


def send(port, cmd, listen_s=ACK_TIMEOUT_S, narrate=False):
    """Send one command line, print every record that comes back."""
    got_ack = False
    with open_serial(port, timeout=0.2) as s:
        s.write((cmd + "\n").encode())
        s.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < listen_s:
            chunk = s.read(65536)
            if not chunk:
                if got_ack and not narrate:
                    break
                continue
            buf += chunk
            recs, _, used = parse_stream(bytes(buf))
            for r in recs:
                k = r["_kind"]
                if k == "ack":
                    got_ack = True
                    flag = "" if r["status_name"] == "OK" else "   <-- !!"
                    print(f"  ACK  '{r['cmd']}'  {r['status_name']}{flag}")
                elif k == "sta":
                    print(describe_stat(r))
                elif k == "end":
                    print("  [sentinel] mode finished")
                    return got_ack
            buf = buf[used:]
            if got_ack and not narrate:
                break
        if not narrate:
            # drain politely so the next command starts clean
            s.write(b"\n")
            s.flush()
    if not got_ack:
        print("  !! no ACK from the device within "
              f"{listen_s:.0f}s - is it in a streaming mode? "
              "Send any byte to stop it, or reset the board.")
    return got_ack


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action")
    ap.add_argument("args", nargs="*")
    ap.add_argument("--rot", type=int, default=None,
                    help="e-paper rotation 0-3, applied at blit time")
    ap.add_argument("--port", default=None)
    a = ap.parse_args()

    port = resolve_port(a.port)
    print(f"port {port}")

    act = a.action
    if act in ("b1", "b0", "v1", "v0"):
        return 0 if send(port, f"U {act}") else 1

    if act == "led":
        if len(a.args) != 3:
            print("usage: actuate.py led <c0> <c1> <c2>   (0-255, WIRE order)")
            return 2
        c = " ".join(a.args)
        print(f"  LED raw wire values -> {c}   (record what colour you SEE)")
        return 0 if send(port, f"U l {c}") else 1

    if act == "epaper":
        if not a.args or a.args[0] not in ("init", "ready", "alert", "clear"):
            print("usage: actuate.py epaper init|ready|alert|clear [--rot N]")
            return 2
        letter = {"init": "i", "ready": "r", "alert": "a", "clear": "c"}[a.args[0]]
        cmd = f"U e {letter}"
        if a.rot is not None:
            cmd += f" {a.rot}"
        print("  e-paper work runs on core 1 and is queued; a full refresh "
              "takes ~2 s.")
        return 0 if send(port, cmd, listen_s=15.0, narrate=True) else 1

    if act == "status":
        return 0 if send(port, "U s") else 1

    if act == "drill":
        secs = int(a.args[0]) if a.args else 120
        print(f"  drill for {secs}s: alert -> press SNOOZE -> silence -> "
              f"auto re-arm at 45 s -> alert again")
        print("  (any key stops it early is NOT implemented here; let it run "
              "or reset the board)")
        return 0 if send(port, f"D {secs}", listen_s=secs + 10,
                         narrate=True) else 1

    print(f"unknown action '{act}'")
    return 2


if __name__ == "__main__":
    sys.exit(main())
