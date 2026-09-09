#!/usr/bin/env python
"""Print the ESP32-S3 serial port, resolved NOW. Never cached, never
hardcoded: the board may re-enumerate on a different node between runs, and a
stale path captures nothing while looking like it worked."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_trace import resolve_port  # noqa: E402

if __name__ == "__main__":
    print(resolve_port(None))
