"""
profile_store.py - the per-board calibration profile.

CALIBRATION LIVES ON THE MAC, NOT IN FLASH. The firmware stays generic; the
host passes runtime knobs from a JSON profile keyed by the board's MAC. The
consequence is the whole point: recalibrating never requires a rebuild, and
therefore never forces a golden re-proof.

Profiles live in `calibration/<mac>.json`. Sessions live in
`captures/<date>_<mac-suffix>/`.
"""
import json
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # firmware/sentry_node
CAL_DIR = ROOT / "calibration"
CAP_DIR = ROOT / "captures"

SCHEMA = "sentry-node/calibration-profile/1"


def _now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def mac_suffix(mac: str) -> str:
    return mac.replace(":", "")[-6:].lower() if mac else "unknown"


def profile_path(mac: str) -> Path:
    return CAL_DIR / f"{mac.replace(':', '').lower()}.json"


def load(mac: str) -> dict:
    p = profile_path(mac)
    if p.exists():
        return json.loads(p.read_text())
    return {
        "schema": SCHEMA,
        "board_mac": mac,
        "created": _now(),
        "updated": None,
        "sessions": [],
        # everything below is filled in by the wizard, and every key is
        # deliberately absent until Measured - a missing key means "not known",
        # never "assume the default".
    }


def save(prof: dict) -> Path:
    CAL_DIR.mkdir(parents=True, exist_ok=True)
    prof["updated"] = _now()
    p = profile_path(prof["board_mac"])
    p.write_text(json.dumps(prof, indent=2, sort_keys=True))
    return p


def latest() -> dict | None:
    """Most recently updated profile, for `run_device.py --profile latest`."""
    if not CAL_DIR.exists():
        return None
    files = sorted(CAL_DIR.glob("*.json"),
                   key=lambda f: f.stat().st_mtime, reverse=True)
    return json.loads(files[0].read_text()) if files else None


def session_dir(mac: str, when: datetime | None = None) -> Path:
    when = when or datetime.now()
    d = CAP_DIR / f"{when:%Y-%m-%d}_{mac_suffix(mac)}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def run_dir(mac: str, when: datetime | None = None) -> Path:
    """A capture directory unique to THIS RUN, not just this day.

    `session_dir` is per-day on purpose: field_day's stages resume into the
    same place, so S5 can find what S4 left. A capture tool is the opposite
    case - it reuses fixed labels ("quiet", "talk", "rotor_4m_0"), so a second
    run on the same day silently lands on top of the first.

    That is not hypothetical. On a 120 s series overwrote the 30 s
    series that a committed document cited as its evidence; the traces were
    only recoverable because they happened to be in git. Minutes in the
    directory name cost nothing and make the collision impossible.
    """
    when = when or datetime.now()
    d = CAP_DIR / f"{when:%Y-%m-%d_%H%M}_{mac_suffix(mac)}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def manifest_append(sess: Path, entry: dict):
    """One JSON object per line. Append-only: a crashed session still leaves
    every capture before it readable."""
    entry = {"t": _now(), **entry}
    with (sess / "manifest.jsonl").open("a") as f:
        f.write(json.dumps(entry, sort_keys=True) + "\n")


def manifest_read(sess: Path) -> list:
    p = sess / "manifest.jsonl"
    if not p.exists():
        return []
    return [json.loads(ln) for ln in p.read_text().splitlines() if ln.strip()]


def threshold_for(prof: dict | None, default: float = 1.70) -> float:
    """The operating threshold to run at. Falls back to the sealed deployment
    default when nothing has been calibrated - never to a guess."""
    if not prof:
        return default
    return float(prof.get("recommended_threshold") or default)


# ---------------------------------------------------------------------------
# the port additions. Both follow the same rule as everything above: a MISSING
# key means "not known", never "assume the default".
# ---------------------------------------------------------------------------

def set_bus_offset(prof: dict, samples: float, constant, n_runs: int,
                   spread: float = None, geometry: str = None) -> dict:
    """Record the measured inter-bus start offset AND whether it survives a
    reset.

    The two facts are stored together on purpose. An offset measured once is
    not a constant, it is an observation; compensating for it (CX-D) is only
    sound if it comes back the same after a power cycle. `constant` is
    True / False / None-for-unknown, and CX-D stays off the shipped path until
    it is True.
    """
    prof["bus_offset_samples"] = float(samples)
    prof["bus_offset_constant"] = (None if constant is None else bool(constant))
    prof["bus_offset_runs"] = int(n_runs)
    if spread is not None:
        prof["bus_offset_spread_samples"] = float(spread)
    if geometry:
        prof["geometry_profile"] = geometry
    prof["bus_offset_measured"] = _now()
    return prof


def set_excluded_f0(prof: dict, bands) -> dict:
    """Persistent-source exclusion bands, as [[centre_hz, tol_hz],...].

    SHIPS EMPTY AND STAYS EMPTY until a human puts a number here from a site
    baseline. There is no self-learning anywhere in this project: a device that
    teaches itself to ignore a frequency is a device that can be taught to
    ignore the threat. Tier-2 reads these; v1 never does.
    """
    bands = [[float(c), float(t)] for c, t in (bands or [])]
    if len(bands) > 4:
        raise ValueError("at most 4 exclusion bands (T2_MAX_EXCL)")
    prof["excluded_f0"] = bands
    prof["excluded_f0_set"] = _now()
    return prof


def excluded_f0(prof: dict | None):
    return [tuple(b) for b in (prof or {}).get("excluded_f0", [])]
