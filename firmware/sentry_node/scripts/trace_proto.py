"""
trace_proto.py - the wire format, in one place, shared by capture and compare.

Must stay in lockstep with main/trace.h. Every record carries a magic word and
an FNV-1a check field, so the parser resynchronises on a magic word rather than
trusting a byte offset: a stray log byte costs one record, not the run.
"""
import struct

MAGIC_HDR = 0xA5C31A0F
MAGIC_REC = 0x5EC0DE01
MAGIC_PRB = 0x9B10BE55
MAGIC_END = 0xE0F5EA1D
MAGIC_MET = 0x3E7E12A0      # meter record
MAGIC_ALT = 0xA1E27F00      # alert record
MAGIC_PCM = 0x9C40DA7A      # raw int16 samples, VARIABLE length
MAGIC_TON = 0x70E4B142      # averaged spectrum peak (tone plumbing check)
# --- alert-output + quad-acquisition build -------------------------------
# Additive record types. VERSION deliberately stays 1: the parser skips magics
# it does not know, so old tools read new captures and new tools read old ones.
MAGIC_ACK = 0x0ACC0DE5      # command acknowledgement
MAGIC_STA = 0x57A75AFE      # device status snapshot
MAGIC_QMT = 0x9AD4E7E5      # quad meter: 4 channels + 2 buses
MAGIC_PC4 = 0x9C4D4A7A      # quad PCM, VARIABLE length
# --- Tier-2 build (the port) ---------------------------------------------
# Additive again, and VERSION deliberately STILL 1. A bump would claim old
# tools cannot read these captures, which is false: the parser resynchronises
# on magic words and skips unknown ones, so a Stage-1b tool reads a Tier-2
# capture and simply sees no T2 records. tests/test_t2_records.py asserts that
# against a real pre-Tier-2 capture rather than trusting the claim.
MAGIC_T2R = 0x72E12A05      # Tier-2 per-frame record
MAGIC_AL2 = 0xA1E27F02      # Tier-2 alert
MAGIC_T3R = 0x73E13A05      # Tier-3 per-update record (SPARSE: 1 frame in 4)
MAGIC_AL3 = 0xA1E27F03      # Tier-3 alert
VERSION = 1

ACK_STATUS = {0: "OK", 1: "NOT_IMPLEMENTED", 2: "FAULT", 3: "BAD_ARG"}
EPD_STATE = {0: "uninit", 1: "ready", 2: "busy", 3: "faulted"}
ALERT_STATE = {0: "idle", 1: "alerting", 2: "snoozed", 3: "cooldown"}

# little-endian, packed - '<' plus no alignment padding
HDR = struct.Struct("<II32sIIdIIIIIII")
REC = struct.Struct("<IIdfHddHBBHBBiBBII")
END = struct.Struct("<IIBIfiIIIQIIQQQQBI")
PRB = struct.Struct("<III8H8f8d8fdddBBHfI")
MET = struct.Struct("<IIQIddiiiIII")
ALT = struct.Struct("<IIddfiII")
TON = struct.Struct("<IIIdd8H8fdI")
PCM_HDR = struct.Struct("<IIIH")     # magic, seq, first_index, n
ACK = struct.Struct("<I16sII")
STA = struct.Struct("<IIBBBBBBBBIBBHI")
QMT = struct.Struct("<IIQI4d4d4i4i4i2I2I2II")
PC4_HDR = struct.Struct("<IIIHH")    # magic, seq, base_frame, n_frames, n_ch
T2R = struct.Struct("<IIdfdHHBBBBiiifII")
AL2 = struct.Struct("<IIddfifII")
# Tier-3. Doubles throughout: the wash statistic and its rate are computed in
# double on the device and a float32 round trip would lose the last digit that
# distinguishes two adjacent 1 Hz rate candidates.
T3R = struct.Struct("<IIdddddiiiBBBBII")
AL3 = struct.Struct("<IIdddiII")

HDR_FIELDS = ("magic version name n_samples n_frames threshold fs n_fft hop "
              "n_bins n_f0 rec_size chk").split()
REC_FIELDS = ("magic frame t_s score f0_bin f0_hz f0_raw_hz teeth floor_fast "
              "reanch n_held_bins above_thr cont_accepted chain fired "
              "is_octave us_frame chk").split()
END_FIELDS = ("magic n_frames verdict_fires n_events peak_score longest_chain "
              "n_floor_fast n_reanch n_held_frames total_us max_us p99_us "
              "us_fft us_mag us_floor us_score chain_overflow chk").split()
PRB_FIELDS = ("magic frame win_checksum probe_bin probe_mag probe_floor "
              "probe_S e e_slow flat rising fast argmax_bin argmax_score "
              "chk").split()
MET_FIELDS = ("magic seq t_us n_samples rms dc_offset peak_abs vmin vmax "
              "short_reads timeouts chk").split()
ALT_FIELDS = ("magic frame t_s f0_hz score chain n_events chk").split()
TON_FIELDS = ("magic n_avg peak_bin peak_hz bin_hz top_bin top_mag dc_mag "
              "chk").split()
ACK_FIELDS = ("magic cmd status chk").split()
STA_FIELDS = ("magic uptime_ms mode buzzer motor led_cmd led_c0 led_c1 led_c2 "
              "alert_state snooze_remaining_ms epaper_state button_level "
              "press_count chk").split()
QMT_FIELDS = ("magic seq t_us n_samples rms dc peak vmin vmax timeouts "
              "short_reads frames_total chk").split()
# array widths for the QMT record, by field
QMT_WIDTH = {"rms": 4, "dc": 4, "peak": 4, "vmin": 4, "vmax": 4,
             "timeouts": 2, "short_reads": 2, "frames_total": 2}
T2R_FIELDS = ("magic frame t_s score2 f02_hz f02_row teeth2 hit fired2 "
              "excluded pad hits n2 track_age kappa us_t2 chk").split()
AL2_FIELDS = ("magic frame t_s f02_hz score2 hits kappa n_events chk").split()
T3R_FIELDS = ("magic frame t_s r_hz W r_any_hz W_any hits n3 track_age hit "
              "fired3 frozen pad us_t3 chk").split()
AL3_FIELDS = ("magic frame t_s r_hz W hits n_events chk").split()

ALL_MAGIC = {MAGIC_HDR: ("hdr", HDR), MAGIC_REC: ("rec", REC),
             MAGIC_PRB: ("prb", PRB), MAGIC_END: ("end", END),
             MAGIC_MET: ("met", MET), MAGIC_ALT: ("alt", ALT),
             MAGIC_TON: ("ton", TON), MAGIC_ACK: ("ack", ACK),
             MAGIC_STA: ("sta", STA), MAGIC_QMT: ("qmt", QMT),
             MAGIC_T2R: ("t2r", T2R), MAGIC_AL2: ("al2", AL2),
             MAGIC_T3R: ("t3r", T3R), MAGIC_AL3: ("al3", AL3)}


def fnv1a(b):
    h = 2166136261
    for c in b:
        h ^= c
        h = (h * 16777619) & 0xFFFFFFFF
    return h


def _unflatten(kind, vals):
    """PRB has fixed-size arrays; struct flattens them, so rebuild the shape."""
    if kind == "qmt":
        d, i = {}, 0
        for f in QMT_FIELDS:
            n = QMT_WIDTH.get(f, 1)
            d[f] = list(vals[i:i + n]) if n > 1 else vals[i]
            i += n
        return d
    if kind == "ack":
        d = dict(zip(ACK_FIELDS, vals))
        d["cmd"] = d["cmd"].split(b"\0")[0].decode("ascii", "replace")
        d["status_name"] = ACK_STATUS.get(d["status"], f"?{d['status']}")
        return d
    if kind == "sta":
        d = dict(zip(STA_FIELDS, vals))
        d["mode_chr"] = chr(d["mode"]) if d["mode"] else "-"
        d["epaper_name"] = EPD_STATE.get(d["epaper_state"], "?")
        d["alert_name"] = ALERT_STATE.get(d["alert_state"], "?")
        return d
    if kind == "ton":
        d, i = {}, 0
        for f in TON_FIELDS:
            n = 8 if f in ("top_bin", "top_mag") else 1
            d[f] = list(vals[i:i + n]) if n > 1 else vals[i]
            i += n
        return d
    if kind != "prb":
        names = {"hdr": HDR_FIELDS, "rec": REC_FIELDS, "end": END_FIELDS,
                 "met": MET_FIELDS, "alt": ALT_FIELDS,
                 "t2r": T2R_FIELDS, "al2": AL2_FIELDS,
                 "t3r": T3R_FIELDS, "al3": AL3_FIELDS}[kind]
        d = dict(zip(names, vals))
        if kind == "hdr":
            d["name"] = d["name"].split(b"\0")[0].decode()
        return d
    i = 0
    d = {}
    for f in PRB_FIELDS:
        n = 8 if f in ("probe_bin", "probe_mag", "probe_floor", "probe_S") else 1
        d[f] = list(vals[i:i + n]) if n > 1 else vals[i]
        i += n
    return d


def parse(buf):
    """Whole-blob parse. Returns (records, n_bad_chk)."""
    recs, bad, _ = parse_stream(buf)
    return recs, bad


def parse_stream(buf):
    """Incremental parse. Returns (records, n_bad_chk, n_bytes_consumed).

    A live reader MUST use this and keep buf[consumed:] for the next chunk: a
    record straddling a read boundary is otherwise silently dropped, which
    looks exactly like the device skipping frames.

    PCM records are VARIABLE length: the header carries the sample count, so
    the end cannot be found without reading it first. Everything else is fixed
    size and resynchronises on its magic word."""
    out, bad, i, n, done = [], 0, 0, len(buf), 0
    while i + 4 <= n:
        magic = struct.unpack_from("<I", buf, i)[0]
        if magic == MAGIC_PC4:
            if i + PC4_HDR.size > n:
                break
            _, seq, base, nfr, nch = PC4_HDR.unpack_from(buf, i)
            n_samp = nfr * nch
            total = PC4_HDR.size + 2 * n_samp + 4
            if i + total > n:
                break
            body = buf[i:i + PC4_HDR.size + 2 * n_samp]
            chk = struct.unpack_from("<I", buf, i + PC4_HDR.size + 2 * n_samp)[0]
            if chk != fnv1a(body):
                bad += 1
                i += 1
                continue
            out.append({"_kind": "pcm4", "seq": seq, "base_frame": base,
                        "n_frames": nfr, "n_ch": nch,
                        "samples": struct.unpack_from(f"<{n_samp}h", buf,
                                                      i + PC4_HDR.size)})
            i += total
            done = i
            continue
        if magic == MAGIC_PCM:
            if i + PCM_HDR.size > n:
                break
            _, seq, first, cnt = PCM_HDR.unpack_from(buf, i)
            total = PCM_HDR.size + 2 * cnt + 4
            if i + total > n:
                break
            body = buf[i:i + PCM_HDR.size + 2 * cnt]
            chk = struct.unpack_from("<I", buf, i + PCM_HDR.size + 2 * cnt)[0]
            if chk != fnv1a(body):
                bad += 1
                i += 1
                continue
            out.append({"_kind": "pcm", "seq": seq, "first_index": first,
                        "n": cnt,
                        "samples": struct.unpack_from(f"<{cnt}h", buf,
                                                      i + PCM_HDR.size)})
            i += total
            done = i
            continue
        hit = ALL_MAGIC.get(magic)
        if hit is None:
            i += 1
            continue
        kind, S = hit
        if i + S.size > n:
            break
        raw = buf[i:i + S.size]
        d = _unflatten(kind, S.unpack(raw))
        if d["chk"] != fnv1a(raw[:-4]):
            bad += 1
            i += 1                      # not a real record: keep scanning
            continue
        d["_kind"] = kind
        out.append(d)
        i += S.size
        done = i
    return out, bad, done
