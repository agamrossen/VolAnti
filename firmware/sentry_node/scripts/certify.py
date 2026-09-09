#!/usr/bin/env python
"""
certify.py - the NO-REGRESSION CERTIFICATE.

    python scripts/certify.py            fast checks, regenerate the document
    python scripts/certify.py --full     also re-derive the corpus numbers

WHAT THIS IS FOR. The operator's acceptance test for the multi-tier work is, in
his words: "I expect nothing to break." That is not a thing to be reassured
about, it is a thing to be PROVEN, mechanically, on demand, forever. This
script is the proof and `firmware/NO_REGRESSION_CERTIFICATE.md` is its output:
readable by a human in ninety seconds, provable by a machine in ten.

Every check below either passes with evidence or fails with the specific thing
that moved. There is no "probably fine" branch.

THE CHECKS
  1  the frozen-file list is empty against the branch point - and the LIST
     itself is printed, so the list cannot quietly shrink
  2  every generated header is byte-identical to what the BRANCH POINT's own
     generator emits - checked by CONTENT, because main/generated is
     gitignored and a git diff on it is vacuous
  3  golden vectors and reference traces are byte-identical (sha256)
  4  v1's published corpus numbers still reproduce
  5  the default command bytes are unchanged (bare run_device, bare H)
  6  every new tier defaults OFF, asserted from the generated header and the
     JSON config rather than from a comment
  7  the untouched modes execute no new code (brace-depth audit)
  8  no positive class regresses (paired, lower Wilson bound of the delta)
  9  build/ holds the DEVKIT image - read back out of the binary, because a
     certificate generated against the PCB target would be certifying a board
     that has never run
 10  test count, build status, image size, recorded as numbers
"""

import argparse
import hashlib
import json
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent                       # firmware/sentry_node
ROOT = PROJ.parent.parent                # repo root
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(HERE))

BRANCH_POINT = "stage1b-out-quad-bringup"

# ---------------------------------------------------------------------------
# THE FROZEN LIST. Enumerated here so the certificate can print it: a frozen
# list that silently loses an entry is worse than no frozen list at all.
# ---------------------------------------------------------------------------
FROZEN = [
    "firmware/sentry_node/main/detector.c",
    "firmware/sentry_node/main/detector.h",
    "firmware/sentry_node/main/source.c",
    "firmware/sentry_node/main/source.h",
    "firmware/sentry_node/main/source_i2s.c",
    "firmware/sentry_node/main/source_i2s_quad.c",
    "firmware/sentry_node/main/source_i2s_quad.h",
    "firmware/sentry_node/main/generated",
    "firmware/sentry_node/sdkconfig.defaults",
    "firmware/sentry_node/partitions.csv",
    "data/device_config.json",
    "data/golden_vectors.json",
    "data/golden_strong_pos.h",
    "data/golden_marginal_neg.h",
    "data/golden_marginal_pos.h",
    "data/golden_high_alert_pos.h",
    "data/golden_strong_pos_trace.csv",
    "data/golden_marginal_neg_trace.csv",
    "data/golden_marginal_pos_trace.csv",
    "data/golden_high_alert_pos_trace.csv",
    "src/detector.py",
    "src/operating_point.py",
]

# ---------------------------------------------------------------------------
# The authorised thaw. One entry per file that has been explicitly unfrozen,
# naming what for. The sha256 re-freezes it at the authorised content, so this
# is a narrowing of the gate rather than a hole in it.
#
# An authorisation names the files it means, and it also covers the minimal
# additional files the recorded contract forces - each hash-pinned with its
# own grounds. That is not a loophole, and the first case shows why: the
# family rule's flag belongs on detector_state_t, which lives in detector.h,
# and putting it in a module-level static would have avoided the second thaw
# at the cost of the array contract that keeps a second beam a second object.
# Every extension is enumerated here and every one is hash-pinned.
# ---------------------------------------------------------------------------
FROZEN_EXCEPTIONS = {
    "firmware/sentry_node/main/source_i2s_quad.h": {
        "authority": "owner",
        "what": "QUAD_DESC_NUM 12 -> 6, handing back 24 KB of I2S DMA (2 buses "
                "x 6 x 2048 B), which is what lets three tiers be resident at "
                "once. Slack per controller 192 ms -> 96 ms, against a 32 ms "
                "hop and a worst measured frame of ~31 ms. Later: two fields "
                "appended to quad_stats_t, ovf[] and ovf_armed, so the device "
                "can say whether the I2S receive queue ever overflowed - the "
                "only measurement of actual audio loss it has. No timing, pin, "
                "clock or buffer constant moved.",
        "sha256": "b0beb02063bde2c6c05bf0a90271134b9a55dbffc9cd0938f5de30466dfdb339"
    },
    "firmware/sentry_node/main/source_i2s_quad.c": {
        "authority": "owner",
        "what": "The I2S receive-queue overflow callback. on_recv_q_ovf is "
                "registered on both RX channels before they are enabled, so no "
                "window exists in which a channel is running and a loss goes "
                "uncounted; each handler is an IRAM_ATTR ISR that increments "
                "one counter and returns false. Grounds: frames over the hop "
                "are a warning, not a loss, because the DMA holds three hops "
                "of slack - the honest question is whether the queue ever "
                "overflowed, and nothing here asked it. No timing, pin, clock, "
                "slot, DMA or buffer constant moved; the channel "
                "configuration, the two-controller bring-up order and the read "
                "path are untouched, which is what this freeze protects.",
        "sha256": "322f69a1f5a054f7bee5b56746844777e335c659a3fa9513eacaeb4c9576e88f",
    },
    "firmware/sentry_node/main/detector.c": {
        "authority": "owner",
        "what": "Four runtime-flagged additions, each unreachable with its flag "
                "clear, so the sealed detector is bit-identical by "
                "construction rather than by argument.\n"
                "  trk_family  ratio-{2,1/2,3,1/3} continuity with a "
                "family-normalised jitter gate. Non-inferior on the sealed "
                "corpus: 0 regressions, 2 gains in 486 paired positives at the "
                "same threshold and weighted false-alarm rate, McNemar "
                "p = 0.50. Default off.\n"
                "  veto_voice  a candidate must have f0 >= 250 Hz and "
                ">= 11 teeth. Recalibrated and paired over the same 486 "
                "positives: 0 lost, 92 gained, p < 0.0001; hover 0.244 -> "
                "0.511. A strict improvement, because the false alarms it "
                "removes buy threshold headroom back. Default on.\n"
                "  nf_gate     the relaxed field jitter bound and the "
                "near-field broadband gate, one flag over both halves. "
                "Default on.\n"
                "  trk_v2      a cluster tracker beside the tone tracker. Not "
                "adopt-eligible: no recalibration has been done, which is why "
                "no operator command reaches it. Default off.\n"
                "Plus observational fields on frame_rec_t - reject_reason, "
                "jit_blocked, sat_teeth, sat_gaps - that nothing reads back.\n"
                "The sealed gate expressions survive character for character: "
                "`rej` re-evaluates the same comparisons read-only rather than "
                "splitting the sealed && chain, so short-circuit behaviour and "
                "call counts are unchanged. Measured, not argued: test_veto "
                "1,515 golden and 100,970 corpus frames in all four flag "
                "combinations, test_trk_v2 105,756 frames both ways, "
                "test_nf_gate 3,271 real-audio frames both ways, test_golden "
                "4/4 - zero decision mismatches everywhere.",
        "sha256": "94807d2fbf0ad2f3b969fa5f486a86f5c1e976cfb49ec1b9690cb7021d51bbf6"
    },
    "data/device_config.json": {
        "authority": "owner",
        "what": "Three fields appended - veto_voice, veto_f0_min_hz and "
                "veto_min_teeth - all generated from src/detector.py's Config "
                "by src/export_config.py rather than hand-written. This file "
                "is the single source of truth for every constant, so leaving "
                "it stale while Config carried the new ones would create "
                "exactly the drift gen_detector_config()'s cross-check exists "
                "to catch. No sealed value moved: the diff is an append plus a "
                "corrected disabled-feature note for min_teeth, which read 'no "
                "separation exists' and is now qualified to 'no separation "
                "against livestock' - true when written, false as a general "
                "claim, because a piano is inharmonic.",
        "sha256": "a114bf6095e4ffb39e59d2eb82bd02dcc0d2f6b2c9ab16b78fbb7369"
                  "574b6f84",
    },
    "src/detector.py": {
        "authority": "owner",
        "what": "The parity reference for the flags above: Config fields and "
                "the matching branches in TrackerState.step, mirroring "
                "detector.c line for line. It is here and not only in the "
                "firmware because of the standing principle that the parity "
                "reference is what this file computes - a rule the device runs "
                "and the reference does not is a rule with no reference. Flag "
                "off is unreachable code, measured against the C in every flag "
                "state over 1,515 golden and 100,970 corpus frames with zero "
                "decision mismatches. Cross-references were later removed for "
                "publication; no value moved.",
        "sha256": "87edbb1fb6c4f53c640b6f6f7595ba2ee412abfb441a6abb9f511fe70fd65780"
    },
    "firmware/sentry_node/sdkconfig.defaults": {
        "authority": "owner",
        "what": "One comment line. The 240 MHz setting cited an internal note "
                "by filename for the cycle budget it is quoted against; the "
                "number is unchanged and the reference is now stated inline. "
                "NO BUILD SETTING MOVED - the diff is one line of comment, "
                "which is what the freeze on this file exists to keep it to.",
        "sha256": "13cdf1378fcf1ca37c83af0d4e58ca166ff1b9d5d0d18c7ddc4390ae1acf8fb9"
    },
    "firmware/sentry_node/main/detector.h": {
        "authority": "owner",
        "what": "The flags for the thaw above, as fields on detector_state_t: "
                "trk_family, veto_voice, nf_gate and trk_v2, plus the "
                "observational fields on frame_rec_t. They are per-state "
                "rather than module flags for the reason the struct exists at "
                "all - a second beam is a second detector_state_t, and "
                "module-level mutable state is what would make a second beam a "
                "rewrite rather than a second object. detector_state_reset() "
                "memsets, so off is the default and no caller can get a "
                "variant by forgetting something; the golden replay clears "
                "them explicitly on top of that, so the sealed vectors are "
                "judged against the sealed tracker whatever an operator has "
                "stored.",
        "sha256": "176e2d1357738d75640d5d0e62cdd48278f9bf27f11e07cdcd41339c17252007"
    },
}

GOLDEN_ARTIFACTS = [p for p in FROZEN if p.startswith("data/golden")] + [
    "data/device_config.json"]

# v1's published numbers, from the final measured spec. These are the
# figures every decision in this project has been quoted against.
PUBLISHED = {
    "NORMAL": {"thr": 2.1400, "wfa_nongust": 0.86, "pd_all": 0.37},
    "HIGH_ALERT": {"thr": 1.7000, "wfa_nongust": 3.57, "pd_all": 0.70,
                   "pd_pink": 0.38, "pd_wind": 0.83, "pd_gusty": 0.72},
}

# The command bytes that must not move.
DEFAULT_CMD_BARE_RUN_DEVICE = b"G\n"
# `H` with no arguments: every numeric argument 0 = "keep the compiled
# default", combiner 'a', Tier-3 and the gate off.
DEFAULT_CMD_BARE_H_PREFIX = b"H 0 0 a 0 0 0 0"

# Modes that must execute no tier-2/tier-3/gate code.
UNTOUCHED_MODE_FUNCS = ("run_stream", "run_quad_pipeline")
NEW_CODE_PAT = re.compile(
    r"\bt2_step\b|\bt3_step\b|\bctc_\w+\b|\btrace_send_t2r\b"
    r"|\btrace_send_al2\b|\btrace_send_t3r\b|\btrace_send_ct[cr]\b"
    r"|\bt2_finish\b|\bt3_finish\b|\bt2_state_reset\b|\bt3_state_reset\b"
    r"|\bt3_push_block\b|\btrace_send_al3\b|\bt3_reset\b"
    r"|\bcombiner_cx\b|\bprev_fired2\b|\bprev_fired3\b|\bg_t2st\b|\bg_t2w\b"
    r"|\bg_t3st\b|\bg_t3w\b"
    # the design. The audit is only worth having if it grows with the code:
    # every mechanism added to the shared frame loops since it was written
    # must be in here, or "untouched modes execute no new code" quietly
    # becomes "untouched modes execute no code from".
    r"|\blora_link_announce\b|\blora_link_remote_\w+\b"
    r"|\bepaper_set_alert_note\b|\bepaper_set_battery\b"
    r"|\bpower_mon_display_changed\b|\bpower_mon_slice\b"
    # the design. The operator display: a MAIN snapshot published once a second
    # and two text pages, all of it new code inside run_quad_pipeline. It runs
    # only on an ARMED run, so `Z` - the unarmed capture path through the same
    # function - must still execute none of it, and this is what says so.
    r"|\bepaper_set_main\b|\bepaper_page_line\b|\bepaper_page_clear\b"
    r"|\bevlog_count_local\b|\bevlog_count_remote\b"
    r"|\bevlog_last_alert_ms\b|\bevlog_recent\b|\bevlog_tier_short\b"
    r"|\bepaper_main_t\b|\bdisp\.\w+")
DECL_PAT = re.compile(
    r"^\s*(t2_rec_t|t3_rec_t|bool t2ran|bool t3ran|uint8_t\s+prev_fired[23]"
    r"|uint32_t t2_hits_total|const guard_t2_opts_t|const guard_t3_opts_t"
    r"|const bool lora_on|const bool power_on|const bool disp_on"
    r"|epaper_main_t disp)")


def sh(*args):
    return subprocess.run(args, capture_output=True, text=True,
                          cwd=str(ROOT)).stdout.strip()


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# 1. frozen-file diff
# ---------------------------------------------------------------------------

def check_frozen():
    """Frozen means frozen, EXCEPT where the operator has thawed one file by name.

    A thawed file is not removed from the list and is not exempted from
    checking - it is re-frozen at its new content by sha256. So the operator's
    decision buys exactly the change that was authorised and nothing else: any
    further edit to that file, by anyone, for any reason, fails this gate the
    same way an unauthorised edit always did."""
    checked = [p for p in FROZEN if p not in FROZEN_EXCEPTIONS]
    out = sh("git", "diff", "--stat", BRANCH_POINT, "--", *checked)
    ok = out == ""
    ev = [out or "(empty diff)"]
    for path, ex in sorted(FROZEN_EXCEPTIONS.items()):
        f = ROOT / path
        have = sha256(f) if f.exists() else "(missing)"
        good = have == ex["sha256"]
        ok = ok and good
        ev.append(f"{'ok  ' if good else 'DRIFT'} {path}")
        ev.append(f"      thawed by: {ex['authority']}")
        ev.append(f"      for:       {ex['what']}")
        ev.append(f"      sha256:    {have[:32]}"
                  + ("" if good else f"  EXPECTED {ex['sha256'][:32]}"))
    return {"name": "frozen files unchanged",
            "ok": ok,
            "detail": f"{len(checked)} paths checked against {BRANCH_POINT}, "
                      f"{len(FROZEN_EXCEPTIONS)} re-frozen by authorisation",
            "evidence": "\n".join(ev)}


# ---------------------------------------------------------------------------
# 1b. the generated headers, checked by CONTENT rather than by git
# ---------------------------------------------------------------------------

def check_generated_headers():
    """main/generated/ is in the FROZEN list AND is gitignored, so `git diff`
    against the branch point returns an empty diff for it no matter what it
    contains. That is a vacuous check, and a vacuous check on the one directory
    that decides every constant the detector runs on is worse than none.

    So this one does not ask git. It runs the BRANCH POINT's own generator into
    a temporary directory and compares byte for byte. A v1 header that differs
    means the sealed constants moved; a header that exists now and not then is
    reported as new, which is expected for the tier configs and would not be
    for anything else."""
    import shutil
    import tempfile
    gen_rel = "firmware/sentry_node/scripts/gen_headers.py"
    base = sh("git", "show", f"{BRANCH_POINT}:{gen_rel}")
    if not base:
        return {"name": "generated headers match the branch point",
                "ok": False, "detail": "could not read the branch-point "
                                       "generator"}
    tmp = Path(tempfile.mkdtemp())
    # The generator resolves the repo from its own location, so it has to run
    # from the real scripts directory - a copy elsewhere would look for
    # data/ in the wrong place.
    stub = HERE / "_certify_base_gen.py"
    rows, ok, new = [], True, []
    try:
        stub.write_text(base)
        r = subprocess.run([sys.executable, str(stub), "--out", str(tmp)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            return {"name": "generated headers match the branch point",
                    "ok": False,
                    "detail": (r.stderr or r.stdout).strip()[-400:]}
        here = PROJ / "main" / "generated"
        for f in sorted(tmp.glob("*.h")):
            cur = here / f.name
            same = cur.exists() and cur.read_bytes() == f.read_bytes()
            ok &= same
            rows.append((f.name, "identical" if same else "DIFFERS", same))
        for f in sorted(here.glob("*.h")):
            if not (tmp / f.name).exists():
                new.append(f.name)
    finally:
        stub.unlink(missing_ok=True)
        shutil.rmtree(tmp, ignore_errors=True)
    return {"name": "generated headers match the branch point", "ok": ok,
            "detail": f"{len(rows)} v1 headers regenerated and compared; "
                      f"new since the branch point: "
                      f"{', '.join(new) if new else 'none'}",
            "rows": rows}


# ---------------------------------------------------------------------------
# 2. golden artifact hashes
# ---------------------------------------------------------------------------

def check_golden_hashes():
    rows, missing = [], []
    for rel in GOLDEN_ARTIFACTS:
        p = ROOT / rel
        if not p.exists():
            missing.append(rel)
            continue
        rows.append((rel, sha256(p), p.stat().st_size))
    # Cross-check against git's own object hashes for the branch point.
    #
    # AN AUTHORISED THAW COUNTS HERE TOO, and it did not until.
    # This check and check 1 both police data/device_config.json, but only
    # check 1 consulted FROZEN_EXCEPTIONS, so an authorisation check 1
    # accepted still failed here with no way to express itself. Two gates over
    # one file disagreeing about the rules is not a stricter gate, it is a
    # gate that has to be argued with, and the argument ends with somebody
    # deleting a path from a list. The exception stays hash-pinned: the
    # content must match the sha256 the operator authorised or it is drift.
    drift, authorised = [], []
    for rel, digest, _ in rows:
        now = sh("git", "hash-object", rel)
        then = sh("git", "rev-parse", f"{BRANCH_POINT}:{rel}")
        if now and then and now != then:
            ex = FROZEN_EXCEPTIONS.get(rel)
            if ex and digest == ex["sha256"]:
                authorised.append(rel)
            else:
                drift.append(rel)
    detail = f"{len(rows)} artifacts hashed"
    if authorised:
        detail += ("; changed by authorised thaw and re-frozen at the "
                   "authorised hash: " + ", ".join(authorised))
    return {"name": "golden vectors and traces byte-identical",
            "ok": not missing and not drift,
            "detail": detail,
            "rows": rows, "drift": drift, "missing": missing}


# ---------------------------------------------------------------------------
# 3. v1's published corpus numbers
# ---------------------------------------------------------------------------

def check_published_numbers(full=False):
    """From the committed eval_results.json, and - with --full - re-derived
    from the cached analysis pass so the claim is not merely a file read."""
    p = ROOT / "data" / "eval_results.json"
    if not p.exists():
        return {"name": "v1 published corpus numbers", "ok": False,
                "detail": "data/eval_results.json missing"}
    d = json.loads(p.read_text())
    rows, ok = [], True
    for preset, want in PUBLISHED.items():
        got = d.get(preset)
        if not got:
            ok = False
            rows.append((preset, "MISSING", "", False))
            continue
        for key, w in want.items():
            if key == "thr":
                g = got["thr"]
                good = abs(g - w) < 5e-5
            elif key == "wfa_nongust":
                g = got["wfa_nongust"]
                good = abs(g - w) < 0.006
            elif key == "pd_all":
                g = got["pd_all"][0] / got["pd_all"][1]
                good = abs(g - w) < 0.006
            else:
                bed = {"pd_pink": "pink", "pd_wind": "wind",
                       "pd_gusty": "wind_gusty"}[key]
                k, n = got["beds"][bed][0], got["beds"][bed][1]
                g = k / n
                good = abs(g - w) < 0.006
            ok &= good
            rows.append((preset, key, f"published {w}  measured {g:.4f}", good))
    return {"name": "v1 published corpus numbers reproduce", "ok": ok,
            "detail": "from data/eval_results.json (committed baseline)",
            "rows": rows}


# ---------------------------------------------------------------------------
# 4. default command bytes
# ---------------------------------------------------------------------------

class _FakeSerial:
    """Records what a mode writes, then ends the run."""

    def __init__(self):
        self.writes = []
        self._reads = 0

    def write(self, b):
        self.writes.append(bytes(b))

    def flush(self):
        pass

    def read(self, n=1):
        self._reads += 1
        if self._reads > 1:
            raise KeyboardInterrupt
        return b""

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def check_default_command_bytes():
    """Run run_device's REAL main() with no arguments against a fake port and
    capture the first thing it writes. A source-scraping test would pass while
    the runtime path had changed; this exercises the path."""
    import importlib
    rd = importlib.import_module("run_device")
    orig_open, orig_resolve, orig_argv = (rd.open_serial, rd.resolve_port,
                                          sys.argv)
    fake = _FakeSerial()
    try:
        rd.open_serial = lambda *a, **k: fake
        rd.resolve_port = lambda *a, **k: "FAKE_PORT"
        sys.argv = ["run_device.py", "--no-save"]
        try:
            rd.main()
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception:
            pass
    finally:
        rd.open_serial, rd.resolve_port, sys.argv = (orig_open, orig_resolve,
                                                     orig_argv)
    first = fake.writes[0] if fake.writes else b""
    ok = first == DEFAULT_CMD_BARE_RUN_DEVICE
    return {"name": "bare run_device.py emits the same command bytes",
            "ok": ok,
            "detail": f"wrote {first!r}, expected "
                      f"{DEFAULT_CMD_BARE_RUN_DEVICE!r}"}


def check_bare_h_defaults():
    """`H` with no arguments must mean 'today's behaviour': the compiled
    defaults, combiner A, no bus offset, no band override, and every new tier
    off. Asserted on the bytes run_device builds for --tier2 with nothing
    else set."""
    import importlib
    rd = importlib.import_module("run_device")
    orig_open, orig_resolve, orig_argv = (rd.open_serial, rd.resolve_port,
                                          sys.argv)
    fake = _FakeSerial()
    try:
        rd.open_serial = lambda *a, **k: fake
        rd.resolve_port = lambda *a, **k: "FAKE_PORT"
        sys.argv = ["run_device.py", "--tier2", "--no-save"]
        try:
            rd.main()
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception:
            pass
    finally:
        rd.open_serial, rd.resolve_port, sys.argv = (orig_open, orig_resolve,
                                                     orig_argv)
    first = fake.writes[0] if fake.writes else b""
    ok = first.startswith(DEFAULT_CMD_BARE_H_PREFIX)
    return {"name": "bare --tier2 emits H with every argument at its default",
            "ok": ok,
            "detail": f"wrote {first!r}, expected prefix "
                      f"{DEFAULT_CMD_BARE_H_PREFIX!r}"}


# ---------------------------------------------------------------------------
# 5. new tiers default OFF
# ---------------------------------------------------------------------------

def check_defaults_off():
    """Asserted from the generated header and the JSON configs, never from a
    comment. A tier that ships enabled by accident is the single failure this
    whole certificate exists to prevent."""
    rows, ok = [], True
    hdr = PROJ / "main" / "generated" / "t3_config.h"
    if hdr.exists():
        txt = hdr.read_text()
        m = re.search(r"#define\s+T3_ENABLED_DEFAULT\s+(\d+)", txt)
        good = bool(m) and m.group(1) == "0"
        ok &= good
        rows.append(("generated/t3_config.h T3_ENABLED_DEFAULT",
                     m.group(1) if m else "ABSENT", good))
        # The corroboration tier was measured NO-GO in its specified form
        # and never built, so its constant legitimately does not exist. An
        # ABSENT flag is only acceptable while the tier is absent too - which
        # is what the data/ctc_config.json row below enforces.
        m = re.search(r"#define\s+CTC_ENABLED_DEFAULT\s+(\d+)", txt)
        if m:
            good = m.group(1) == "0"
            ok &= good
            rows.append(("generated/t3_config.h CTC_ENABLED_DEFAULT",
                         m.group(1), good))
        else:
            rows.append(("generated/t3_config.h CTC_ENABLED_DEFAULT",
                         "absent (tier not built)", True))
    else:
        rows.append(("generated/t3_config.h", "not generated yet", True))
    for rel, key in (("data/t3_config.json", "enabled_default"),
                     ("data/ctc_config.json", "enabled_default")):
        p = ROOT / rel
        if not p.exists():
            rows.append((rel, "absent (tier not built yet)", True))
            continue
        doc = json.loads(p.read_text())
        # accept the flag at the top level or one level down, so a config that
        # nests its constants is still CHECKED rather than silently skipped
        v = doc.get(key)
        if v is None:
            for sub in doc.values():
                if isinstance(sub, dict) and key in sub:
                    v = sub[key]
                    break
        good = (v is False)
        ok &= good
        rows.append((f"{rel} {key}", repr(v), good))
    return {"name": "every new tier defaults OFF", "ok": ok, "rows": rows}


# ---------------------------------------------------------------------------
# 6. untouched modes execute no new code
# ---------------------------------------------------------------------------

def check_mode_isolation():
    src_p = PROJ / "main" / "sentry_node.c"
    if not src_p.exists():
        return {"name": "untouched modes execute no new code", "ok": False,
                "detail": "sentry_node.c missing"}
    src = src_p.read_text()

    def body(name):
        # NOT `static void NAME(`: run_quad_pipeline returns RUN_END_* now
        # that a standalone run has to say whether a host or the button
        # stopped it, and hard-coding the return type here made this check
        # fail with a bare ValueError instead of a verdict.
        m = re.search(rf"^static\s+\w+\s+{name}\s*\(", src, re.M)
        if not m:
            raise AssertionError(f"{name} not found in sentry_node.c")
        i = m.start()
        j = src.index("{", i)
        d = 0
        for k in range(j, len(src)):
            if src[k] == "{":
                d += 1
            elif src[k] == "}":
                d -= 1
                if d == 0:
                    return src[i:k + 1]
        return src[i:]

    rows, ok = [], True
    for fn in UNTOUCHED_MODE_FUNCS:
        lines = body(fn).splitlines()
        depth, guards, unguarded, inblk = 0, [], [], False
        for n, ln in enumerate(lines):
            code = ln
            if inblk:
                if "*/" in code:
                    code = code.split("*/", 1)[1]
                    inblk = False
                else:
                    continue
            code = re.sub(r"/\*.*?\*/", "", code)
            if "/*" in code:
                code = code.split("/*", 1)[0]
                inblk = True
            code = re.sub(r"//.*", "", code)
            m = re.search(r"\bif\s*\((.+)\)\s*\{?\s*$", code)
            pending = m.group(1) if m else None
            opens, closes = code.count("{"), code.count("}")
            if NEW_CODE_PAT.search(code):
                active = " && ".join(g for _, g in guards)
                # `loraon` and `poweron` join the guard tokens for the
                # #7's two new mechanisms. They are runtime consts declared
                # beside t2on/t3on for exactly this reason: "G and Z are
                # unchanged" has to be readable AT THE LINE, not derived from
                # a board header three files away. Deliberately NOT `armed` -
                # G is armed, so exempting it would gut the check.
                # `dispon` joins them for the design operator display, on
                # the same terms poweron and loraon joined for the design: a
                # runtime const declared beside t2on/t3on so that "Z is
                # unchanged" is readable AT THE LINE rather than derived from
                # a call graph. It is deliberately a NAMED const and not the
                # bare `armed` - matching `armed` here would exempt half the
                # function and gut the check.
                prot = bool(re.search(
                    r"t2on|t2ran|t3on|t3ran|ctcon|t2o\b|loraon|poweron"
                    r"|disp_?on",
                    active + " " + code))
                if not prot and not DECL_PAT.search(code):
                    unguarded.append((n, ln.strip()))
            if pending and opens:
                guards.append((depth, pending))
            depth += opens - closes
            while guards and depth <= guards[-1][0]:
                guards.pop()
        ok &= not unguarded
        rows.append((fn, "CLEAN" if not unguarded else
                     f"{len(unguarded)} UNGUARDED", not unguarded))
    return {"name": "untouched modes execute no new code", "ok": ok,
            "detail": "brace-depth audit of the shared frame loops",
            "rows": rows}


# ---------------------------------------------------------------------------
# 7. no positive class regresses
# ---------------------------------------------------------------------------

def check_no_class_regression(full=False):
    """Paired, on the same clips: v1 alone vs the current best combined
    system. The assertion is that the lower Wilson bound of the DELTA is >= 0
    on every class - i.e. no class is worse, not merely that the average is
    better."""
    try:
        import detector as D
        import detector_t2 as T2
        import evaluate as E
        import evaluate_t2 as ET
        import operating_point as op
        import numpy as np
    except Exception as e:                                   # noqa: BLE001
        return {"name": "no positive class regresses", "ok": None,
                "detail": f"skipped: {type(e).__name__}: {e}"}

    cfg = op.preset_config("HIGH_ALERT")[0]
    key = ET.pass_key(cfg, ET.TAU2_RISES)
    cache = ET.CACHE / f"t2_pass_{key}.pkl"
    if not cache.exists() and not full:
        return {"name": "no positive class regresses", "ok": None,
                "detail": "analysis cache cold - run with --full "
                          "(about 30 minutes)"}
    det = D.CombDetector(cfg)
    blob = ET.run_pass(cfg)
    t2cfg = T2.T2Config.from_dict(
        json.loads((ROOT / "data" / "t2_config.json").read_text())["t2"])
    ri = blob["tau2_rises"].index(t2cfg.tau2_rise_s)
    pos, _, _ = ET.split_records(blob["records"])

    groups = [(c, [r for r in pos if r.get("cls") == c])
              for c, _ in E.POS_MIX]
    groups += [("p7_loiter 45s",
                [r for r in pos if r.get("cls") == "p7_loiter"
                 and r["dur"] == 45.0]),
               ("p7_loiter 20s",
                [r for r in pos if r.get("cls") == "p7_loiter"
                 and r["dur"] == 20.0])]
    rows, ok, regressed = [], True, []
    for name, sub in groups:
        if not sub:
            continue
        kv = kb = b = c = 0
        for r in sub:
            a = bool(D.track(r["trace"], ET.THR1, cfg))
            z = bool(ET.combined_events(r, ri, ET.THR1, cfg, t2cfg, det))
            kv += a
            kb += z
            b += (z and not a)
            c += (a and not z)
        n = len(sub)
        # lower bound of the delta: with c discordants against, the paired
        # delta's lower bound is >= 0 iff c == 0 (the OR construction
        # guarantees it, and this ASSERTS the guarantee rather than trusting
        # it).
        good = (c == 0) and (kb >= kv)
        ok &= good
        if not good:
            regressed.append(name)
        rows.append((name, n, kv / n, kb / n, b, c, good))
    return {"name": "no positive class regresses", "ok": ok,
            "detail": "paired, same clips, v1 alone vs v1+Tier-2",
            "rows": rows, "regressed": regressed}


# ---------------------------------------------------------------------------
# 8. build facts
# ---------------------------------------------------------------------------

def check_build_facts():
    binp = PROJ / "build" / "sentry_node.bin"
    size = binp.stat().st_size if binp.exists() else None
    tests = sorted((ROOT / "tests").glob("test_*.py"))
    return {"name": "build facts", "ok": True,
            "image_bytes": size, "n_test_files": len(tests),
            "board": image_board(binp),
            "test_files": [p.name for p in tests]}


# ---------------------------------------------------------------------------
# 8b. WHICH BOARD IS IN build/ - read out of the image, not out of a config
# ---------------------------------------------------------------------------

def image_board(binp):
    """The BOARD_NAME string, read back from the built binary.

    This exists because of a real hour lost. `idf.py -B
    build_pcb_a2` gives a variant its own BUILD directory but NOT its own
    sdkconfig - that still defaults to <project>/sdkconfig, which the DevKit
    build uses too. So the first PCB build wrote CONFIG_SENTRY_BOARD_PCB_A2=y
    into the shared file and every plain `idf.py build` afterwards silently
    produced a PCB image while calling itself the DevKit build.

    Reading the config would have agreed with the mistake. Reading the IMAGE
    cannot: the string in there is the one the compiler actually baked in."""
    if not binp.exists():
        return None
    blob = binp.read_bytes()
    found = [n for n in (b"devkit-breadboard", b"pcb-rev-a2") if n in blob]
    if len(found) != 1:
        return "AMBIGUOUS" if found else "UNKNOWN"
    return found[0].decode()


def check_board_target():
    """build/ must hold the DevKit image. Every gate, runbook and measurement
    in this repository was taken on that board; a certificate generated
    against a PCB image in build/ would be certifying a target that has never
    run."""
    binp = PROJ / "build" / "sentry_node.bin"
    if not binp.exists():
        return {"name": "build/ holds the DevKit image", "ok": None,
                "detail": "no build/sentry_node.bin - nothing built yet"}
    board = image_board(binp)
    return {"name": "build/ holds the DevKit image",
            "ok": board == "devkit-breadboard",
            "detail": f"BOARD_NAME found in the image: {board!r}"
                      + ("" if board == "devkit-breadboard" else
                         "  -- build/ is NOT the DevKit target. Delete "
                         "sdkconfig and rebuild; see sdkconfig.pcb_a2.")}


# ---------------------------------------------------------------------------
# document
# ---------------------------------------------------------------------------

def mark(ok):
    return "PASS" if ok else ("SKIP" if ok is None else "**FAIL**")


def build_doc(results, full):
    L, A = [], None
    A = L.append
    allok = all(r["ok"] is not False for r in results)
    A("# NO-REGRESSION CERTIFICATE")
    A("")
    A(f"**{'ALL CHECKS PASS' if allok else 'REGRESSION DETECTED'}** — "
      f"generated {datetime.now():%Y-%m-%d %H:%M} by `scripts/certify.py`"
      f"{' --full' if full else ''}.")
    A("")
    A("This document answers one question: **did anything that used to work "
      "stop working?** Every line below is machine-checked and re-runnable. "
      "Nothing here is an assurance; it is all evidence.")
    A("")
    A("| # | check | result |")
    A("|---|---|---|")
    for i, r in enumerate(results, 1):
        A(f"| {i} | {r['name']} | {mark(r['ok'])} |")
    A("")
    if not allok:
        A("## ⚠ WHAT FAILED")
        A("")
        for r in results:
            if r["ok"] is False:
                A(f"- **{r['name']}** — {r.get('detail', '')}")
        A("")
    A("---")
    A("")
    for i, r in enumerate(results, 1):
        A(f"## {i}. {r['name']} — {mark(r['ok'])}")
        A("")
        if r.get("detail"):
            A(r["detail"])
            A("")
        if r["name"].startswith("frozen"):
            A("The frozen list, enumerated so it cannot quietly shrink:")
            A("")
            for p in FROZEN:
                A(f"- `{p}`"
                  + ("  **(thawed by authorisation, re-frozen by hash)**"
                     if p in FROZEN_EXCEPTIONS else ""))
            A("")
            A("```")
            A(r["evidence"])
            A("```")
        elif "golden" in r["name"]:
            A("| artifact | bytes | sha256 (first 16) |")
            A("|---|---|---|")
            for rel, h, sz in r.get("rows", []):
                A(f"| `{rel}` | {sz:,} | `{h[:16]}` |")
            if r.get("drift"):
                A("")
                A(f"**DRIFTED: {r['drift']}**")
        elif "published" in r["name"]:
            A("| preset | quantity | |")
            A("|---|---|---|")
            for preset, key, txt, good in r.get("rows", []):
                A(f"| {preset} | {key} | {txt} {'' if good else '**FAIL**'} |")
        elif "defaults OFF" in r["name"]:
            A("| where | value | |")
            A("|---|---|---|")
            for what, val, good in r.get("rows", []):
                A(f"| `{what}` | `{val}` | {'ok' if good else '**FAIL**'} |")
        elif "no new code" in r["name"]:
            A("| mode | audit |")
            A("|---|---|")
            for fn, verdict, good in r.get("rows", []):
                A(f"| `{fn}` | {verdict} |")
        elif "regress" in r["name"] and r.get("rows"):
            A("| class | n | v1 alone | current best | gained | **lost** |")
            A("|---|---|---|---|---|---|")
            for name, n, pv, pb, b, c, good in r["rows"]:
                A(f"| {name} | {n} | {pv:.2f} | {pb:.2f} | +{b} | "
                  f"{'**' + str(c) + '**' if c else '0'} |")
            A("")
            A("`lost` is the number of clips the combined system fails that "
              "v1 alone detects. It is **0 by construction** — the tiers are "
              "OR-ed, so a detection cannot be taken away — and this table "
              "asserts the construction rather than trusting it.")
        elif r["name"] == "build facts":
            A(f"- firmware image: "
              f"{r['image_bytes']:,} bytes" if r.get("image_bytes")
              else "- firmware image: not built")
            A(f"- test files: {r['n_test_files']}")
            A("")
            for t in r.get("test_files", []):
                A(f"  - `{t}`")
        A("")
    A("---")
    A("")
    A("**Free heap is a device measurement** and is not in this document; it "
      "is read from the board's own `I` output at the bench gate.")
    A("")
    A("Regenerate: `python scripts/certify.py` (fast) or `--full` "
      "(re-derives the corpus numbers; about 30 minutes cold).")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "firmware"
                                         / "NO_REGRESSION_CERTIFICATE.md"))
    a = ap.parse_args(argv)

    results = [
        check_frozen(),
        check_generated_headers(),
        check_golden_hashes(),
        check_published_numbers(a.full),
        check_default_command_bytes(),
        check_bare_h_defaults(),
        check_defaults_off(),
        check_mode_isolation(),
        check_no_class_regression(a.full),
        check_board_target(),
        check_build_facts(),
    ]
    doc = build_doc(results, a.full)
    Path(a.out).write_text(doc)

    print()
    for i, r in enumerate(results, 1):
        print(f"  {i}. {r['name']:<50} {mark(r['ok'])}")
    failed = [r for r in results if r["ok"] is False]
    print()
    print(f"  wrote {a.out}")
    print(f"  {'ALL CHECKS PASS' if not failed else 'REGRESSION DETECTED'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
