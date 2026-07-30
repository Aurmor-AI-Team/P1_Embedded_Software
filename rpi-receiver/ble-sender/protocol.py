"""Wire protocol + framing for the Aurmor BLE biometric stream.

Pure constants and helpers — intentionally free of any BlueZ/D-Bus imports so
this module (and replay.py) run on any machine, including the macOS ``--stdout``
dry-run path.

These UUIDs and the binary-v1 framing are the contract the React Native receiver
(react-native-ble-plx, a BLE *central*) mirrors in
``aurmor-sports-mobile/features/ble-stream/protocol.ts``.
"""
from __future__ import annotations

import json
import struct
from typing import Dict, Iterator, List, Optional, Tuple

# 128-bit custom UUIDs (newly minted for this project; keep in sync with the app).
SERVICE_UUID = "5a8e0000-9b1a-4c7d-8e2f-1f3a5b7c9d10"
META_UUID = "5a8e0001-9b1a-4c7d-8e2f-1f3a5b7c9d10"      # read   -> JSON descriptor
DATA_UUID = "5a8e0002-9b1a-4c7d-8e2f-1f3a5b7c9d10"      # notify -> binary-v1 byte stream
CONTROL_UUID = "5a8e0003-9b1a-4c7d-8e2f-1f3a5b7c9d10"   # write  -> start/stop/restart/forget [wid]
WIFI_CREDS_UUID = "5a8e0004-9b1a-4c7d-8e2f-1f3a5b7c9d10"  # read -> JSON {ssid,password,ip,port,pi_id}
WEARABLES_UUID = "5a8e0005-9b1a-4c7d-8e2f-1f3a5b7c9d10"   # read -> JSON {"active":[{"wid","node"}]}
IMPACTS_UUID = "5a8e0006-9b1a-4c7d-8e2f-1f3a5b7c9d10"     # read -> JSON per-athlete impact summary
ROSTER_UUID = "5a8e0007-9b1a-4c7d-8e2f-1f3a5b7c9d10"      # read/write -> JSON roster (app owns this)

DEFAULT_DEVICE_NAME = "aurmor-rpi"
DEFAULT_CHUNK_SIZE = 180         # safe after MTU negotiation (notification <= MTU-3)
SCHEMA_VERSION = 2               # 2 = binary-v1 framing (was 1 = NDJSON)

# The receiver broadcasts which wearables are live (heard over its WiFi in the
# last few seconds) as manufacturer-specific data in its BLE advertisement, so
# the app's passive scan can show a BLE-silent provisioned ESP32 as detected
# WITHOUT connecting (a connection each poll is what prompted the iOS pairing
# dialog). Mirrored in aurmor-sports-mobile/features/esp32-provisioning/protocol.ts.
# 0xFFFF is the Bluetooth-SIG "internal/test" company id. Payload after the
# company id: [version=1, wid-bitmask] (bit i set => wid i+1 is live).
PRESENCE_MFG_ID = 0xFFFF
PRESENCE_VERSION = 2      # v1 was an 8-bit wid bitmask; see below

# --------------------------------------------------------------------------- #
# Squad capacity.
#
# 30 athletes x up to 6 devices = 180 wearables. Those numbers are enforced in
# roster.py, but they are NOT free — see the budgets below, which are what the
# receiver actually schedules against.
# --------------------------------------------------------------------------- #
MAX_ATHLETES = 30
MAX_DEVICES_PER_ATHLETE = 6
MAX_DEVICES = MAX_ATHLETES * MAX_DEVICES_PER_ATHLETE      # 180

# Aggregate UDP packet budget for the whole squad. A 49-byte telemetry frame
# costs ~168 us of 2.4 GHz airtime once preamble, SIFS, ACK, DIFS and backoff
# are counted, so the medium tops out near 5,900 pps with one station and
# roughly 2,400 pps once ~180 stations are contending. 2,000 leaves headroom
# for HELLO traffic, retries and the AP's own overhead.
#
#   180 devices @ 100 Hz = 18,000 pps  -> 9x over
#   180 devices @  10 Hz =  1,800 pps  -> fits
#    30 devices @  60 Hz =  1,800 pps  -> fits
UDP_PACKET_BUDGET_PPS = 2000
TELEMETRY_MIN_HZ = 2
TELEMETRY_MAX_HZ = 100

# Airtime for one 49-byte telemetry frame at 2.4 GHz / 802.11n MCS7, including
# preamble, SIFS, ACK, DIFS and average backoff. Kept here (rather than
# imported from schedule.py) so the governor has no extra dependency.
FRAME_AIRTIME_US = 168.0

# Govern on channel DUTY, not packet count.
#
# A flat pps cap is the wrong control variable: 2000 pps works out to ~33%
# channel occupancy at ANY device count, which is right at the edge where CSMA
# starts to degrade — 30 stations at 33% duty already carry a ~44% chance that
# a given transmission overlaps another. Targeting duty instead keeps the same
# margin whether there are 5 devices or 180.
CHANNEL_DUTY_TARGET = 0.20


def telemetry_rate_hz(n_devices: int,
                      duty_target: float = CHANNEL_DUTY_TARGET,
                      budget_pps: int = UDP_PACKET_BUDGET_PPS) -> int:
    """Per-device telemetry rate that keeps the channel near ``duty_target``.

    The pps cap is retained as a second bound so a very small squad cannot be
    handed a rate that swamps the receiver even though the air is quiet.
    """
    if n_devices <= 0:
        return TELEMETRY_MAX_HZ
    by_duty = duty_target * 1e6 / (n_devices * FRAME_AIRTIME_US)
    by_pps = budget_pps / n_devices
    hz = int(min(by_duty, by_pps))
    return max(TELEMETRY_MIN_HZ, min(TELEMETRY_MAX_HZ, hz))


def channel_duty(n_devices: int, hz: int) -> float:
    """Fraction of channel time n devices occupy at hz each."""
    return n_devices * hz * FRAME_AIRTIME_US / 1e6


# BLE uplink budget. bluezero pushes each notification through a D-Bus round
# trip, so a few hundred records/second is the practical ceiling regardless of
# connection interval. Forwarding 180 nodes at full rate is not a tuning
# problem, it is two orders of magnitude out.
BLE_RECORD_BUDGET_PER_S = 200
# Only a handful of nodes stream at full rate: the ones the app is actually
# displaying. Everything else is covered by the batched summary.
FOCUS_MAX_NODES = 4
FOCUS_MAX_HZ = 25


def presence_manufacturer_data(active_wids, athletes_live: int = 0,
                               roster_revision: int = 0) -> list:
    """Payload bytes for the presence advertisement.

    V1 packed live wearables into an 8-bit bitmask, which capped the receiver
    at 8 devices. 180 devices would need 23 bytes of bitmask, and a legacy
    31-byte advertisement has nowhere near that once the device name is in it.

    V2 therefore advertises COUNTS instead of identities:
        [version, n_devices_live, n_athletes_live, roster_revision & 0xFF]
    That is enough for the app's passive scan to show "receiver X, 27 athletes
    live" without connecting. The identities come from the Wearables
    characteristic after connecting, where there is no 31-byte ceiling.
    """
    wids = list(active_wids)
    return [PRESENCE_VERSION,
            min(len(wids), 255),
            min(athletes_live, 255),
            roster_revision & 0xFF]


def presence_manufacturer_data_v1(active_wids) -> list:
    """The legacy 8-wid bitmask, kept so an un-updated app still sees something.
    Devices with wid > 8 are invisible to it — which is exactly why v2 exists."""
    bitmask = 0
    for wid in active_wids:
        if 1 <= wid <= 8:
            bitmask |= 1 << (wid - 1)
    return [1, bitmask & 0xFF]

# Constant / redundant columns dropped from each sample to save BLE bytes.
_DROP_COLUMNS = {"timestamp_iso", "label", "version", "present_mask_hex"}
# Identity columns emitted first, in this order.
_LEAD_COLUMNS = ("node", "t_s", "round")


def _coerce(text: str):
    """Convert a CSV cell to int/float when possible, else leave it a string."""
    text = text.strip()
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def encode_sample(row: Dict[str, str]) -> Dict[str, object]:
    """Turn one CSV row (str -> str) into a compact, JSON-ready sample dict."""
    out: Dict[str, object] = {}
    for key in _LEAD_COLUMNS:
        if key in row:
            out[key] = _coerce(row[key])
    for key, value in row.items():
        if key in _DROP_COLUMNS or key in _LEAD_COLUMNS:
            continue
        out[key] = _coerce(value)
    return out


def sample_to_ndjson(sample: Dict[str, object]) -> bytes:
    """Compact JSON + trailing newline (the NDJSON frame delimiter), as UTF-8."""
    return (json.dumps(sample, separators=(",", ":")) + "\n").encode("utf-8")


def pose_to_ndjson(t_s: float, tran, quats) -> bytes:
    """One IK pose line: SMPL root translation + per-joint local quaternions.

    Shape: {"type":"pose","t_s":<s>,"tran":[x,y,z],"q":[[w,x,y,z], … 24 …]}.
    Tagged with "type" so the receiver routes it apart from biometric samples
    (which are identified by "node"). Shares the Data characteristic + framing.
    """
    obj = {"type": "pose", "t_s": t_s, "tran": tran, "q": quats}
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")


def chunk_bytes(data: bytes, size: int) -> Iterator[bytes]:
    """Yield ``data`` in <=size byte slices (one BLE notification each)."""
    if size < 1:
        raise ValueError("chunk size must be >= 1")
    for i in range(0, len(data), size):
        yield data[i:i + size]


# --------------------------------------------------------------------------- #
# binary-v1 wire format (the Data characteristic; replaces NDJSON).
#
# Why: NDJSON repeats every field NAME on every sample (~39/s), which dominates
# the byte cost. A fixed positional binary frame drops the names (order is
# implicit, published once via Meta) and packs each value as a fixed-width
# number, cutting the link ~5-6x. The app reconstructs the *same* JS objects, so
# the JSON later uploaded to the DB is unchanged in shape (values carry each
# field's scale resolution; see FIELD_SPECS).
#
# Framing (length-prefixed records, then chunk_bytes as before):
#   record = msg_type:u8 | length:u16(LE) | payload[length]
# Sample payload (MSG_SAMPLE):
#   node_idx:u8 | t_s_ms:u32 | <fields in the node's layout order, each per spec>
# Pose payload (MSG_POSE):
#   t_s_ms:u32 | tran:3*i16(/1000) | 24 joints * 4*i16 quaternion(/10000)
# Meta payload (MSG_META):
#   the same JSON descriptor as the Meta characteristic, as UTF-8 bytes. Sent
#   over the Data stream (chunked + framed) because it exceeds the 512-byte GATT
#   attribute limit; the app reads it here before decoding samples.
# --------------------------------------------------------------------------- #

MSG_SAMPLE = 1
MSG_POSE = 2
MSG_META = 3
MSG_IMPACT = 4   # one discrete head-impact event (see encode_impact_binary)
MSG_SUMMARY = 5  # ONE record covering the whole squad (see encode_summary_binary)

POSE_TRAN_SCALE = 1000      # i16: +/-32.767 m
POSE_QUAT_SCALE = 10000     # i16: +/-3.2767 (quaternion components live in [-1,1])

# Curated (type, scale) for well-bounded float sensor fields -> compact int16/u16.
# Encode = round(value*scale); decode = raw/scale. Any field NOT listed here is
# resolved at startup by value type: int -> ("i32", 1) exact; float -> ("i32",
# 1000) exact to 3 decimals; str -> ("str", 0). This keeps every wire field
# (not just UI ones) so the recorder's JSON.stringify upload stays equivalent.
FIELD_SPECS: Dict[str, Tuple[str, int]] = {
    "distance_m": ("u16", 1000),
    "ax_g": ("i16", 1000), "ay_g": ("i16", 1000), "az_g": ("i16", 1000),
    "gx_dps": ("i16", 10), "gy_dps": ("i16", 10), "gz_dps": ("i16", 10),
    "hx_g": ("i16", 1000), "hy_g": ("i16", 1000), "hz_g": ("i16", 1000),
    "imu_temp_c": ("i16", 100),
}

# struct format + clamp range per fixed-width numeric type.
_INT_TYPES = {
    "i16": ("<h", -32768, 32767),
    "u16": ("<H", 0, 65535),
    "i32": ("<i", -2147483648, 2147483647),
    "u32": ("<I", 0, 4294967295),
}


def resolve_spec(field: str, value: object) -> Tuple[str, int]:
    """(type, scale) for ``field``: curated if known, else inferred from value."""
    if field in FIELD_SPECS:
        return FIELD_SPECS[field]
    if isinstance(value, bool) or isinstance(value, int):
        return ("i32", 1)          # exact integer
    if isinstance(value, float):
        return ("i32", 1000)       # exact to 3 decimals
    return ("str", 0)


def _pack_value(typ: str, scale: int, value: object) -> bytes:
    if typ == "str":
        raw = str(value).encode("utf-8")[:255]
        return bytes([len(raw)]) + raw
    if typ == "f32":
        return struct.pack("<f", float(value) if value is not None else 0.0)
    fmt, lo, hi = _INT_TYPES[typ]
    n = int(round((float(value) if value is not None else 0.0) * scale))
    return struct.pack(fmt, max(lo, min(hi, n)))


def frame_record(msg_type: int, payload: bytes) -> bytes:
    """Prefix ``payload`` with [msg_type:u8][length:u16] for self-delimited framing."""
    return struct.pack("<BH", msg_type, len(payload)) + payload


def encode_sample_binary(sample: Dict[str, object], node_idx: int,
                         layout: List[str],
                         field_specs: Dict[str, Tuple[str, int]]) -> bytes:
    """Pack one sample as a MSG_SAMPLE payload (no framing/chunking)."""
    parts = [bytes([node_idx & 0xFF]),
             _pack_value("u32", 1000, sample.get("t_s", 0))]
    for field in layout:
        typ, scale = field_specs[field]
        parts.append(_pack_value(typ, scale, sample.get(field)))
    return b"".join(parts)


def encode_pose_binary(t_s: float, tran, quats) -> bytes:
    """Pack one IK pose as a MSG_POSE payload (no framing/chunking)."""
    parts = [_pack_value("u32", 1000, t_s)]
    for c in tran:
        parts.append(_pack_value("i16", POSE_TRAN_SCALE, c))
    for q in quats:
        for c in q:
            parts.append(_pack_value("i16", POSE_QUAT_SCALE, c))
    return b"".join(parts)


# Fields carried by the ESP32's UDP packet, in wire order (see udp_source.py /
# wifi_udp_tx.cpp). The 10 IMU fields (curated in FIELD_SPECS above) plus 4 mock
# biometric fields the app renders as Heart rate / SpO2 / Respiration / HRV. A
# real head sensor has no biometrics; the mock playback fills these from the
# chest (ECG) and wrist (PPG) reference data so the session screen shows them.
LIVE_IMU_FIELDS: List[str] = [
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "hx_g", "hy_g", "hz_g",
    "imu_temp_c",
]
LIVE_BIO_FIELDS: List[str] = [
    "ecg_hr_bpm",     # Heart rate (bpm)
    "ppg_spo2_pct",   # SpO2 (%)
    "resp_rate_bpm",  # Respiration (breaths/min)
    "ecg_rmssd_ms",   # HRV RMSSD (ms)
]
# Compact (type, scale) for the bio fields (kept out of the shared FIELD_SPECS
# so the CSV-replay encoding is untouched).
_LIVE_BIO_SPECS: Dict[str, Tuple[str, int]] = {
    "ecg_hr_bpm": ("u16", 1),
    "ppg_spo2_pct": ("u16", 100),
    "resp_rate_bpm": ("u16", 1),
    "ecg_rmssd_ms": ("u16", 1),
}


# --------------------------------------------------------------------------- #
# v2 telemetry: the firmware no longer streams raw axes. In ALERTS/LIVE modes it
# sends running impact aggregates, and every discrete hit arrives separately as
# an ALERT (see MSG_IMPACT below). These are the v2 sample fields.
# --------------------------------------------------------------------------- #
LIVE_AGG_FIELDS: List[str] = [
    "impact_count",          # cumulative hits over threshold since boot
    "impact_threshold_g",    # threshold in force on the device
    "impact_accum_g",        # sum of peak g over all counted hits
    "all_time_peak_g",       # largest peak seen since boot
    "imu_temp_c",
]
_LIVE_AGG_SPECS: Dict[str, Tuple[str, int]] = {
    "impact_count": ("u16", 1),
    "impact_threshold_g": ("u16", 100),
    "impact_accum_g": ("u32", 100),
    "all_time_peak_g": ("u16", 100),
    "imu_temp_c": ("i16", 100),
}


def build_live_protocol_meta(nodes: List[str], schema: str = "agg"):
    """Decode tables for the live UDP source: every node shares one layout.

    ``schema="agg"`` (default) publishes the v2 impact-aggregate layout;
    ``schema="raw"`` publishes the original raw-IMU layout for legacy boards.

    Same return shape as build_protocol_meta (field_specs, layouts, node_layout)
    so encode_sample_binary and the app's decoder work unchanged.
    """
    field_specs: Dict[str, List] = {}
    if schema == "raw":
        lead = LIVE_IMU_FIELDS
        for f in LIVE_IMU_FIELDS:
            field_specs[f] = list(FIELD_SPECS[f])
    else:
        lead = LIVE_AGG_FIELDS
        for f in LIVE_AGG_FIELDS:
            field_specs[f] = list(_LIVE_AGG_SPECS[f])
    for f in LIVE_BIO_FIELDS:
        field_specs[f] = list(_LIVE_BIO_SPECS[f])
    layouts = [list(lead) + LIVE_BIO_FIELDS]
    node_layout = [0] * len(nodes)
    return field_specs, layouts, node_layout


# --------------------------------------------------------------------------- #
# MSG_IMPACT — one discrete head impact, from one athlete.
#
# Impacts are NOT samples: they are sparse, individually meaningful, and each
# one must reach the app exactly once. They ride the same Data characteristic
# and the same length-prefixed framing, but carry their own fixed layout so a
# decoder needs no Meta lookup — an impact must stay decodable even if the app
# reconnected and has not re-read Meta yet.
#
# Payload:
#   wid:u8 | seq:u32 | t_s_ms:u32 | epoch_ms:u64 | severity:u8 | mode:u8 |
#   xport:u8 | <IMPACT_FIELDS per IMPACT_SPECS> | player_name:len-prefixed str
# --------------------------------------------------------------------------- #
IMPACT_FIELDS: List[str] = [
    "peak_g", "threshold_g",
    "hx_g", "hy_g", "hz_g",
    "gx_dps", "gy_dps", "gz_dps",
    "rot_dps", "dur_ms",
]
IMPACT_SPECS: Dict[str, Tuple[str, int]] = {
    "peak_g": ("u16", 100),        # 0 - 655 g
    "threshold_g": ("u16", 100),
    "hx_g": ("i16", 100), "hy_g": ("i16", 100), "hz_g": ("i16", 100),
    "gx_dps": ("i16", 10), "gy_dps": ("i16", 10), "gz_dps": ("i16", 10),
    "rot_dps": ("u16", 10),        # resultant rotational rate
    "dur_ms": ("u16", 1),
}


IMPACT_FLAG_HEAD = 0x01          # impact came from a head-position sensor
IMPACT_FLAG_UNATTRIBUTED = 0x02  # wearable was not assigned to any athlete


def encode_impact_binary(event: Dict[str, object]) -> bytes:
    """Pack one impact event as a MSG_IMPACT payload (no framing/chunking).

    Carries the athlete AND the body position, because an athlete may wear
    several sensors and a wrist hit must never be rendered as a head impact.
    ``flags`` marks head-vs-body and attributed-vs-not so the app can decide
    without re-deriving anything from the position string.
    """
    flags = 0
    if event.get("is_head"):
        flags |= IMPACT_FLAG_HEAD
    if not event.get("athlete_id"):
        flags |= IMPACT_FLAG_UNATTRIBUTED
    parts = [
        struct.pack("<H", int(event.get("wid", 0)) & 0xFFFF),
        struct.pack("<I", int(event.get("seq", 0)) & 0xFFFFFFFF),
        _pack_value("u32", 1000, event.get("t_s", 0)),
        struct.pack("<Q", int(event.get("epoch_ms", 0))),
        bytes([int(event.get("severity", 0)) & 0xFF,
               int(event.get("mode", 0)) & 0xFF,
               int(event.get("xport", 0)) & 0xFF,
               flags & 0xFF]),
    ]
    for field in IMPACT_FIELDS:
        typ, scale = IMPACT_SPECS[field]
        parts.append(_pack_value(typ, scale, event.get(field)))
    parts.append(_pack_value("str", 0, event.get("athlete_id") or ""))
    parts.append(_pack_value("str", 0, event.get("athlete") or ""))
    parts.append(_pack_value("str", 0, event.get("position") or ""))
    return b"".join(parts)


def impact_meta() -> Dict[str, object]:
    """Published in Meta so the app can decode MSG_IMPACT records."""
    return {
        "fields": IMPACT_FIELDS,
        "specs": {k: list(v) for k, v in IMPACT_SPECS.items()},
        "severity_names": ["light", "moderate", "severe"],
        "flags": {"head": IMPACT_FLAG_HEAD,
                  "unattributed": IMPACT_FLAG_UNATTRIBUTED},
        # Trailing length-prefixed UTF-8 strings, in this order.
        "trailing_strings": ["athlete_id", "athlete", "position"],
        "wid_bytes": 2,
    }


# --------------------------------------------------------------------------- #
# MSG_SUMMARY — the whole squad in ONE record.
#
# The obvious design is one summary record per node. At 180 nodes and 1 Hz that
# is 180 notifications/second, which is at bluezero's practical ceiling before
# any impact or focused telemetry gets a look in. Batching the squad into a
# single length-prefixed record makes it ~13 chunked notifications per second
# instead, and it arrives as one consistent snapshot rather than 180 that tear
# across each other.
#
# Payload:
#   count:u16 | entry[count]
# entry (13 bytes):
#   node_idx:u16 | flags:u8 | head_impacts:u16 | body_impacts:u16 |
#   peak_g:u16(/100) | age_ms:u16 | rate_hz:u8 | mode:u8
# --------------------------------------------------------------------------- #
SUMMARY_ENTRY = struct.Struct("<HBHHHHBB")

SUMMARY_FLAG_HEAD = 0x01          # this node is a head sensor
SUMMARY_FLAG_LIVE = 0x02          # heard from within the staleness window
SUMMARY_FLAG_UNATTRIBUTED = 0x04  # device not assigned to an athlete
SUMMARY_FLAG_FOCUSED = 0x08       # streaming at full rate right now


def encode_summary_binary(entries: List[Dict[str, object]]) -> bytes:
    """Pack the squad rollup as a MSG_SUMMARY payload (no framing/chunking)."""
    out = [struct.pack("<H", len(entries))]
    for e in entries:
        out.append(SUMMARY_ENTRY.pack(
            int(e.get("node_idx", 0)) & 0xFFFF,
            int(e.get("flags", 0)) & 0xFF,
            min(int(e.get("head_impacts", 0)), 65535),
            min(int(e.get("body_impacts", 0)), 65535),
            min(int(round(float(e.get("peak_g", 0.0)) * 100)), 65535),
            min(int(e.get("age_ms", 65535)), 65535),
            min(int(e.get("rate_hz", 0)), 255),
            int(e.get("mode", 0)) & 0xFF))
    return b"".join(out)


def summary_meta() -> Dict[str, object]:
    """Published in Meta so the app can decode MSG_SUMMARY records."""
    return {
        "entry_format": "<HBHHHHBB",
        "entry_fields": ["node_idx", "flags", "head_impacts", "body_impacts",
                         "peak_g", "age_ms", "rate_hz", "mode"],
        "scales": {"peak_g": 100},
        "flags": {"head": SUMMARY_FLAG_HEAD, "live": SUMMARY_FLAG_LIVE,
                  "unattributed": SUMMARY_FLAG_UNATTRIBUTED,
                  "focused": SUMMARY_FLAG_FOCUSED},
    }


def migrate_legacy_roster(cfg: Dict[str, object]) -> Optional[dict]:
    """Convert an old flat ``{"roster": {"1": "A. Rivera"}}`` config block into a
    roster document. Returns None when there is nothing to migrate.

    The old shape assumed one wearable per person; every migrated device is
    given the HEAD position, which is the only safe assumption for a head-impact
    product — an unknown position must not be silently treated as a head sensor
    later on.
    """
    raw = cfg.get("roster") or {}
    if not raw:
        return None
    athletes = []
    for key, entry in raw.items():
        try:
            wid = int(key)
        except (TypeError, ValueError):
            continue
        if isinstance(entry, str):
            entry = {"player": entry}
        name = entry.get("player") or entry.get("name") or f"Athlete {wid}"
        athletes.append({
            "id": f"legacy-{wid}",
            "name": name,
            "team": entry.get("team", ""),
            "devices": {str(wid): entry.get("position", "HEAD")},
        })
    return {"athletes": athletes} if athletes else None