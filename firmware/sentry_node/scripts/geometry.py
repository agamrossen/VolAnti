"""
geometry.py - the array geometry, as DATA (the port, D12).

Every number the bench checks about WHERE the microphones are - which one
leads which for a clap from a named direction, by how many samples, and what
the alias-free band is - is derived here from `data/geometry_profiles.json`.
Nothing hardcodes 2.84 samples any more.

The point is PCB day. A new board is a new entry in the JSON and a --geometry
flag, not an edit to four scripts, each of which would be an opportunity to
update three of them.

CONVENTIONS, fixed once
-----------------------
  channel order      [M1 M2 M3 M4], everywhere in this project
  axes               +x = EAST, +y = NORTH (the FORWARD / USB-cable edge)
  azimuth            degrees CLOCKWISE FROM NORTH, so 0 = N, 90 = E, 180 = S
  elevation          degrees above the array plane; 90 = directly overhead
  delay sign         delay(a, b) > 0 means b LAGS a

Overhead (elevation 90) gives every baseline exactly zero acoustic delay. That
is not a curiosity, it is the entire design of the S4 test: whatever offset
survives an overhead clap is ELECTRICAL - two I2S RX engines starting at
different times - and nothing to do with where the microphones are.

DETECTION IS GEOMETRY-FREE. Nothing in this module feeds either detection
tier. It is read only by instruments.
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent.parent
PROFILES_JSON = ROOT / "data" / "geometry_profiles.json"

CH_ORDER = ("M1", "M2", "M3", "M4")


class Geometry:
    """One array profile plus everything derivable from it."""

    def __init__(self, doc, name=None):
        self.c = float(doc.get("c_sound_m_s", 343.0))
        self.fs = int(doc.get("fs", 16000))
        name = name or doc.get("default")
        if name not in doc["profiles"]:
            raise KeyError(f"unknown geometry profile {name!r}; have "
                           f"{sorted(doc['profiles'])}")
        p = doc["profiles"][name]
        self.name = name
        self.status = p.get("status", "")
        self.shape = p.get("shape", "")
        self.notes = p.get("notes", [])
        self.tol_mm = float(p.get("position_tolerance_mm", 0.5))
        self.pos_mm = {k: tuple(float(v) for v in p["positions_mm"][k])
                       for k in CH_ORDER}
        self.labels = {k: p.get("labels", {}).get(k, k) for k in CH_ORDER}
        self.bus = {k: p.get("bus", {}).get(k, "?") for k in CH_ORDER}

    # -- basic quantities -------------------------------------------------
    def baseline_mm(self, a, b):
        (ax, ay), (bx, by) = self.pos_mm[a], self.pos_mm[b]
        return math.hypot(ax - bx, ay - by)

    def max_delay_samples(self, a, b):
        """Largest possible acoustic delay across the a-b baseline."""
        return self.baseline_mm(a, b) * 1e-3 / self.c * self.fs

    def longest_baseline(self):
        best = max(((self.baseline_mm(a, b), a, b)
                    for i, a in enumerate(CH_ORDER)
                    for b in CH_ORDER[i + 1:]), key=lambda t: t[0])
        return {"mm": best[0], "pair": (best[1], best[2])}

    def half_wavelength_hz(self):
        """Frequency at which the LONGEST baseline is half a wavelength.

        Above this an off-axis source can reach two capsules in antiphase and
        an unweighted complex sum can CANCEL a tooth instead of adding it.
        The comb score reaches 7800 Hz, so this is not academic - it is the
        motivation for the CX-B / CX-C hybrid experiments."""
        d = self.longest_baseline()["mm"] * 1e-3
        return self.c / (2.0 * d)

    def alias_free_hz(self):
        """Spatial-aliasing limit for the SMALLEST inter-element spacing."""
        d = min(self.baseline_mm(a, b)
                for i, a in enumerate(CH_ORDER) for b in CH_ORDER[i + 1:])
        return self.c / (2.0 * d * 1e-3)

    # -- far-field arrival ------------------------------------------------
    def unit_vector(self, az_deg, el_deg=0.0):
        az, el = math.radians(az_deg), math.radians(el_deg)
        return (math.cos(el) * math.sin(az), math.cos(el) * math.cos(az))

    def delay_samples(self, a, b, az_deg, el_deg=0.0):
        """Delay of `b` relative to `a` for a far-field source at
        (az, el), in samples. Positive means b LAGS a.

        A plane wave from direction u reaches a capsule at p at time
        -(p.u)/c relative to the array centre, so the capsule FURTHER TOWARD
        the source arrives EARLIER and delay(a,b) = ((p_a - p_b).u)/c.
        """
        ux, uy = self.unit_vector(az_deg, el_deg)
        (ax, ay), (bx, by) = self.pos_mm[a], self.pos_mm[b]
        d_mm = (ax - bx) * ux + (ay - by) * uy
        return d_mm * 1e-3 / self.c * self.fs

    def arrival_samples(self, az_deg, el_deg=0.0):
        """Per-channel arrival delay in samples, referenced to the EARLIEST
        channel, so the returned values are >= 0. This is what the spatial
        mini-synth applies."""
        ux, uy = self.unit_vector(az_deg, el_deg)
        raw = {k: -(self.pos_mm[k][0] * ux + self.pos_mm[k][1] * uy)
               * 1e-3 / self.c * self.fs for k in CH_ORDER}
        first = min(raw.values())
        return {k: raw[k] - first for k in CH_ORDER}

    # -- the bench expectation table --------------------------------------
    def clap_expectations(self, tol_samples=0.5):
        """What each bench clap must show, derived rather than declared.

        `tol_samples` is the acceptance window. The overhead row is the one
        that matters most: it has zero acoustic delay on every pair BY
        CONSTRUCTION, so any residual is the inter-bus electrical offset.
        """
        rows = []
        for label, az, el in (("NORTH", 0.0, 0.0), ("EAST", 90.0, 0.0),
                              ("SOUTH", 180.0, 0.0), ("WEST", 270.0, 0.0),
                              ("OVERHEAD", 0.0, 90.0)):
            pairs = {}
            for i, a in enumerate(CH_ORDER):
                for b in CH_ORDER[i + 1:]:
                    pairs[f"{a}->{b}"] = round(
                        self.delay_samples(a, b, az, el), 4)
            arr = self.arrival_samples(az, el)
            order = [k for k, _ in sorted(arr.items(), key=lambda kv: kv[1])]
            rows.append({"clap": label, "az_deg": az, "el_deg": el,
                         "lead_order": order, "pair_delay_samples": pairs,
                         "tol_samples": tol_samples,
                         "electrical_only": label == "OVERHEAD"})
        return rows

    def summary(self):
        lb = self.longest_baseline()
        return (f"geometry {self.name} ({self.status})\n"
                f"  shape           {self.shape}\n"
                f"  positions mm    " + "  ".join(
                    f"{k}({self.labels[k]},bus {self.bus[k]})="
                    f"({self.pos_mm[k][0]:+.2f},{self.pos_mm[k][1]:+.2f})"
                    for k in CH_ORDER) + "\n"
                f"  longest baseline {lb['mm']:.2f} mm "
                f"({lb['pair'][0]}-{lb['pair'][1]}) -> lambda/2 at "
                f"{self.half_wavelength_hz():.0f} Hz\n"
                f"  alias-free to   {self.alias_free_hz():.0f} Hz\n"
                f"  tolerance       +-{self.tol_mm:.2f} mm")


def load(name=None, path=None):
    doc = json.loads((path or PROFILES_JSON).read_text())
    return Geometry(doc, name)


def names(path=None):
    doc = json.loads((path or PROFILES_JSON).read_text())
    return sorted(doc["profiles"]), doc.get("default")


def add_argument(parser, flag="--geometry"):
    """Give any bench tool the same --geometry flag, spelled the same way."""
    all_names, default = names()
    parser.add_argument(flag, default=default, choices=all_names,
                        help=f"array geometry profile (default {default})")
    return parser


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    add_argument(ap)
    a = ap.parse_args()
    g = load(a.geometry)
    print(g.summary())
    print()
    for row in g.clap_expectations():
        tag = "  [ELECTRICAL ONLY]" if row["electrical_only"] else ""
        print(f"  {row['clap']:<9} lead order "
              f"{' < '.join(row['lead_order'])}{tag}")
        for k, v in row["pair_delay_samples"].items():
            if abs(v) > 1e-6:
                print(f"      {k:<10} {v:+.2f} samples")
