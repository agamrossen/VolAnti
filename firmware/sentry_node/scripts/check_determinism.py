#!/usr/bin/env python
"""
check_determinism.py - are two captures of the same vector the same trace?

Compares every field of every record EXCEPT the timing probes (us_frame in
each record, and the us_* / total_us / max_us / p99_us fields of the sentinel).
Wall-clock timing is instrumentation, not part of the trace contract; requiring
it to repeat to the microsecond would make the check fail for reasons that say
nothing about the detector.

Everything else - including the score float32 bit patterns - must be identical.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from trace_proto import parse  # noqa: E402

SKIP_REC = {"us_frame", "chk", "_kind"}
SKIP_END = {"us_fft", "us_mag", "us_floor", "us_score", "total_us", "max_us",
            "p99_us", "chk", "_kind"}


def key(recs):
    out = []
    for r in recs:
        skip = SKIP_END if r["_kind"] == "end" else SKIP_REC
        out.append((r["_kind"],
                    tuple(sorted((k, v) for k, v in r.items()
                                 if k not in skip))))
    return out


def main():
    if len(sys.argv) < 3:
        raise SystemExit("usage: check_determinism.py A.bin B.bin [...]")
    base = None
    ok = True
    for path in sys.argv[1:]:
        recs, bad = parse(Path(path).read_bytes())
        k = key(recs)
        if base is None:
            base, bname = k, path
            print(f"reference {path}: {len(recs)} records")
            continue
        if k == base:
            print(f"  IDENTICAL  {path}  ({len(recs)} records)")
        else:
            ok = False
            n = sum(1 for a, b in zip(base, k) if a != b)
            first = next((i for i, (a, b) in enumerate(zip(base, k))
                          if a != b), None)
            print(f"  DIFFERS    {path}: {n} records differ from {bname}, "
                  f"first at index {first}")
            if first is not None and first < len(k):
                a = dict(base[first][1])
                b = dict(k[first][1])
                for f in sorted(set(a) | set(b)):
                    if a.get(f) != b.get(f):
                        print(f"      {f}: {a.get(f)!r} vs {b.get(f)!r}")
    print("DETERMINISM: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
