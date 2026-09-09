#!/usr/bin/env python
"""
gen_epaper_screens.py - rasterise the e-paper screens to a C header.

    conda activate acoustic-detector
    python scripts/gen_epaper_screens.py

Writes:
    main/epaper_screens.h        checked in, so the ESP-IDF build NEVER
                                 depends on conda
    assets/preview_*.png         eyeball checks

FIVE SCREENS:
    LISTENING   sound-wave icon + "LISTENING"
    ALERT       warning triangle + "ALERT"
    ALERT_WASH  the same triangle + "ALERT" + "TIER 3 WASH" - Tier-3 fires on
                fans and insects too, so the panel says which tier heard it
    OFF         one bold horizontal bar + "OFF". e-paper keeps its image with
                no power, so a device unplugged while LISTENING would go on
                claiming to listen; the long-press draws this first
    TEST        a calibration pattern - border, centre cross, corner ticks and
                a 10 px ruler. If any of it is cut off or sits in a corrupted
                band, ONE photo of this screen tells us the panel's real usable
                geometry, instead of guessing from a description.

LAYOUT RULE, learned the hard way: everything lives inside a generous MARGIN
and is CENTRED. The first version drew edge-to-edge and assumed the panel was
exactly 122x250; on the real hardware part of it landed outside the visible
area and the text was clipped. Content now stays well inside, so a small
geometry mismatch costs whitespace instead of words.

GEOMETRY. The panel is a Waveshare 1.54 inch, 200 x 200 - SQUARE. This was
discovered from a photograph of the hardware: the build had been driving it with
the 2.13 inch V4 driver (122 x 250), which is why the top of the panel showed
noise (RAM never written) and the text ran off the edge. Square means the
landscape/portrait mapping disappears entirely; the blit is now identity plus an
optional rotation.

CONVENTION. 1 = white, 0 = black, matching the vendored driver's Clear().
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MAIN = HERE.parent / "main"
ASSETS = HERE.parent / "assets"

W, H = 200, 200                      # the panel is SQUARE (1.54in, 200x200)
ROW_BYTES = (W + 7) // 8             # 32
MARGIN = 16                          # keep EVERYTHING inside this


# ---------------------------------------------------------------------------
# a tiny 5x7 bitmap font, inlined so the generator needs no font files
# ---------------------------------------------------------------------------
FONT = {
    'A': ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    'B': ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    'C': ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    'D': ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    'E': ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    'G': ["01111", "10000", "10000", "10111", "10001", "10001", "01111"],
    'I': ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    'L': ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    'N': ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    'O': ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    'R': ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    'S': ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    'T': ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    'Y': ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    # Added for the standalone field build: OFF, and the Tier-3 annunciation.
    'F': ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    'H': ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    'M': ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    'P': ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    'U': ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    'W': ["10001", "10001", "10001", "10101", "10101", "11011", "10001"],
    '3': ["11111", "00010", "00100", "00010", "00001", "10001", "01110"],
    '4': ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    '0': ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    '1': ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    '2': ["01110", "10001", "00001", "00110", "01000", "10000", "11111"],
    '5': ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    '!': ["00100", "00100", "00100", "00100", "00100", "00000", "00100"],
    '-': ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    ' ': ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
}
GLYPH_W, GLYPH_H = 5, 7


def text_width(text, scale):
    return (GLYPH_W + 1) * scale * len(text) - scale


def draw_text(img, text, x0, y0, scale=1):
    cx = x0
    for ch in text.upper():
        glyph = FONT.get(ch)
        if glyph is None:
            cx += (GLYPH_W + 1) * scale
            continue
        for gy, row in enumerate(glyph):
            for gx, bit in enumerate(row):
                if bit == "1":
                    img[y0 + gy * scale: y0 + (gy + 1) * scale,
                        cx + gx * scale: cx + (gx + 1) * scale] = 0
        cx += (GLYPH_W + 1) * scale
    return cx


def centre_text(img, text, y, scale):
    x = (W - text_width(text, scale)) // 2
    assert x >= MARGIN, f"'{text}' at scale {scale} does not fit the margin"
    draw_text(img, text, x, y, scale)


def blank():
    return np.ones((H, W), np.uint8)


def disc(img, cx, cy, r):
    y, x = np.ogrid[:H, :W]
    img[(x - cx) ** 2 + (y - cy) ** 2 <= r * r] = 0


def ring(img, cx, cy, r, thick=3):
    y, x = np.ogrid[:H, :W]
    d = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    img[(d <= r + thick / 2) & (d >= r - thick / 2)] = 0


def arc_right(img, cx, cy, r, thick=3, spread=0.85):
    """A sound-wave arc opening to the RIGHT of (cx, cy)."""
    y, x = np.ogrid[:H, :W]
    d = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    on_ring = (d <= r + thick / 2) & (d >= r - thick / 2)
    # keep the right-hand sector only
    ang = np.arctan2(y - cy, x - cx)
    img[on_ring & (np.abs(ang) <= spread)] = 0


# ---------------------------------------------------------------------------
# the screens
# ---------------------------------------------------------------------------
def screen_listening():
    """Sound-wave icon on top, LISTENING beneath it. Both centred."""
    img = blank()
    # Icon band is y 16..66; text sits at 72..100. Both inside the margin.
    # The arcs span +/-0.85 rad, so their vertical reach is r*sin(0.85)=0.75r -
    # which is why the radii stop at 30 rather than 36. The first version used
    # 36 and poked out of the top margin; the margin test caught it.
    icon_cy = 78
    icon_cx = W // 2 - 22
    disc(img, icon_cx, icon_cy, 7)                 # the source dot
    for r in (20, 32, 44):                         # three waves
        arc_right(img, icon_cx, icon_cy, r, thick=4)

    scale = 3                                      # 9 chars -> 159 px wide
    t = "LISTENING"
    assert text_width(t, scale) <= W - 2 * MARGIN
    centre_text(img, t, 140, scale)
    return img


def screen_alert():
    """Warning triangle on top, ALERT beneath. Both centred, inside margins."""
    img = blank()
    cx, cy, half = W // 2, 74, 46
    apex, left, right = (cx, cy - half), (cx - half, cy + half), (cx + half, cy + half)

    def line(p, q, thick=4):
        n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1]))) + 1
        for i in range(n):
            t = i / max(n - 1, 1)
            x = int(round(p[0] + (q[0] - p[0]) * t))
            y = int(round(p[1] + (q[1] - p[1]) * t))
            img[max(0, y - thick // 2): y + thick // 2 + 1,
                max(0, x - thick // 2): x + thick // 2 + 1] = 0

    line(apex, left)
    line(left, right)
    line(right, apex)
    img[cy - 14: cy + 12, cx - 3: cx + 4] = 0      # the bang
    img[cy + 18: cy + 26, cx - 3: cx + 4] = 0

    # scale 5, not 6: at scale 6 the glyphs are 42 px tall and, sitting below
    # the triangle, ran past the bottom margin. The margin test caught it.
    scale = 5                                      # 5 chars -> 145 px wide
    t = "ALERT"
    assert text_width(t, scale) <= W - 2 * MARGIN
    assert 145 + GLYPH_H * scale <= H - MARGIN, "ALERT text runs past the margin"
    centre_text(img, t, 145, scale)
    return img


def screen_alert_wash():
    """Tier-3's alert. Same warning triangle, and it says WHICH TIER heard it.

    Tier-3 fires on fans, insects and vehicles as readily as on a rotor
    (T3_VERDICT.md: 4.87 weighted FA/h against a 0.40 allowance), so a field
    note that cannot say which tier raised an alarm is worth very little. The
    buzzer is the same buzzer - the operator gets one alarm and one dismissal -
    but the panel and the LED disambiguate it afterwards.
    """
    img = blank()
    cx, cy, half = W // 2, 66, 38
    apex, left, right = (cx, cy - half), (cx - half, cy + half), (cx + half, cy + half)

    def line(p, q, thick=4):
        n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1]))) + 1
        for i in range(n):
            t = i / max(n - 1, 1)
            x = int(round(p[0] + (q[0] - p[0]) * t))
            y = int(round(p[1] + (q[1] - p[1]) * t))
            img[max(0, y - thick // 2): y + thick // 2 + 1,
                max(0, x - thick // 2): x + thick // 2 + 1] = 0

    line(apex, left)
    line(left, right)
    line(right, apex)
    img[cy - 12: cy + 10, cx - 3: cx + 4] = 0      # the bang
    img[cy + 15: cy + 22, cx - 3: cx + 4] = 0

    centre_text(img, "ALERT", 118, 4)              # 116 px wide
    centre_text(img, "TIER 3 WASH", 156, 2)        # 130 px wide
    assert 156 + GLYPH_H * 2 <= H - MARGIN, "TIER 3 WASH runs past the margin"
    return img


def screen_off():
    """OFF. One bold horizontal bar, and the word under it.

    THE REASON THIS SCREEN EXISTS: e-paper holds its last image with no power
    at all. A device unplugged while listening keeps showing LISTENING on a
    shelf for weeks, which is a lie that looks exactly like the truth. The long
    press draws this before the outputs go down, so an unpowered panel says
    unpowered.
    """
    img = blank()
    bar_h = 14
    y = H // 2 - 26
    img[y: y + bar_h, MARGIN + 8: W - MARGIN - 8] = 0
    centre_text(img, "OFF", H // 2 + 6, 5)         # 87 px wide
    assert H // 2 + 6 + GLYPH_H * 5 <= H - MARGIN, "OFF runs past the margin"
    return img


def screen_test():
    """Calibration pattern. Photograph it and we know the real geometry.

    * outer border exactly on the MARGIN
    * corner ticks labelled by quadrant
    * centre cross
    * a ruler of 10 px blocks along the bottom
    """
    img = blank()
    m = MARGIN
    img[m:m + 2, m:W - m] = 0                      # top
    img[H - m - 2:H - m, m:W - m] = 0              # bottom
    img[m:H - m, m:m + 2] = 0                      # left
    img[m:H - m, W - m - 2:W - m] = 0              # right

    for (x, y, lbl) in ((m + 6, m + 6, "1"), (W - m - 20, m + 6, "2"),
                        (m + 6, H - m - 20, "5"), (W - m - 20, H - m - 20, "0")):
        img[y:y + 12, x:x + 12] = 0
        draw_text(img, lbl, x + 16, y + 2, scale=1)

    cx, cy = W // 2, H // 2
    img[cy - 1:cy + 1, cx - 22:cx + 22] = 0
    img[cy - 22:cy + 22, cx - 1:cx + 1] = 0

    for i in range(0, 16):                         # 10 px ruler
        if i % 2 == 0:
            img[H - m - 12:H - m - 4, m + 8 + i * 10: m + 18 + i * 10] = 0

    centre_text(img, "TEST", cy - 60, 3)
    return img


# ---------------------------------------------------------------------------
def pack(img):
    assert img.shape == (H, W), img.shape
    out = bytearray()
    for y in range(H):
        row = img[y]
        for bx in range(ROW_BYTES):
            byte = 0
            for bit in range(8):
                x = bx * 8 + bit
                v = int(row[x]) if x < W else 1
                byte |= (v & 1) << (7 - bit)
            out.append(byte)
    return bytes(out)


def c_array(name, data):
    lines = [f"static const uint8_t {name}[{len(data)}] = {{"]
    for i in range(0, len(data), 16):
        lines.append("    " + ", ".join(f"0x{b:02X}" for b in data[i:i + 16]) + ",")
    lines.append("};")
    return "\n".join(lines)


def save_png(img, path):
    import struct
    import zlib
    h, w = img.shape
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw.extend((img[y] * 255).astype(np.uint8).tobytes())

    def chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload +
                struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
           + chunk(b"IEND", b""))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def render_rotated(img, rot):
    """What the PANEL shows at quadrant `rot` (clockwise), mirroring
    main/epaper.c panel_to_logical() EXACTLY. Preview only - the packed
    header is always the unrotated artwork, because the device rotates at
    blit time from a runtime setting."""
    out = np.ones_like(img)
    for py in range(H):
        for px in range(W):
            if rot == 1:
                lx, ly = py, (W - 1) - px
            elif rot == 2:
                lx, ly = (W - 1) - px, (H - 1) - py
            elif rot == 3:
                lx, ly = (H - 1) - py, px
            else:
                lx, ly = px, py
            out[py, px] = img[ly, lx]
    return out


def main():
    screens = {
        "listening": screen_listening(),
        "alert": screen_alert(),
        "alert_wash": screen_alert_wash(),
        "off": screen_off(),
        "test": screen_test(),
    }
    body = "\n\n".join(c_array(f"epaper_screen_{k}", pack(v))
                       for k, v in screens.items())
    hdr = f"""/*
 * epaper_screens.h - GENERATED by scripts/gen_epaper_screens.py. Do not edit.
 *
 * Five 1-bit screens authored LANDSCAPE at {W}x{H} (the panel is {H}x{W}
 * portrait; the firmware blits between them with a runtime rotation).
 *
 * Everything is CENTRED inside a {MARGIN} px margin. The first version drew
 * edge-to-edge and assumed exact panel dimensions; on real hardware the text
 * clipped. Margins mean a geometry mismatch costs whitespace, not words.
 *
 * epaper_screen_test is a CALIBRATION pattern: border on the margin, labelled
 * corner ticks, centre cross, 10 px ruler. Photograph it to learn the panel's
 * true usable area.
 *
 * Convention: 1 = white, 0 = black. Rows MSB-first, {ROW_BYTES} bytes/row,
 * {H} rows.
 */
#pragma once

#include <stdint.h>

#define EPAPER_SCREEN_W          {W}
#define EPAPER_SCREEN_H          {H}
#define EPAPER_SCREEN_ROW_BYTES  {ROW_BYTES}
#define EPAPER_SCREEN_BYTES      {ROW_BYTES * H}
#define EPAPER_SCREEN_MARGIN     {MARGIN}

{body}
"""
    out = MAIN / "epaper_screens.h"
    out.write_text(hdr)
    # Rotated previews: the operator's default is a quarter turn CLOCKWISE and
    # nobody can photograph the panel tonight, so this is the only way to look
    # at what the device will actually show.
    for k, v in screens.items():
        save_png(render_rotated(v, 1), ASSETS / f"preview_rot90_{k}.png")
    for k, v in screens.items():
        save_png(v, ASSETS / f"preview_{k}.png")
    print(f"wrote {out} ({out.stat().st_size} bytes)")
    for k in screens:
        print(f"  preview: assets/preview_{k}.png")
    print(f"  {W}x{H} landscape, {ROW_BYTES} B/row, {ROW_BYTES * H} B/screen, "
          f"margin {MARGIN} px")
    return 0


if __name__ == "__main__":
    sys.exit(main())
