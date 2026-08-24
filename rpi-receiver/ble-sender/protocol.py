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
from typing import Dict, Iterator, List, Tuple

# 128-bit custom UUIDs (newly minted for this project; keep in sync with the app).
SERVICE_UUID = "5a8e0000-9b1a-4c7d-8e2f-1f3a5b7c9d10"
META_UUID = "5a8e0001-9b1a-4c7d-8e2f-1f3a5b7c9d10"      # read   -> JSON descriptor
DATA_UUID = "5a8e0002-9b1a-4c7d-8e2f-1f3a5b7c9d10"      # notify -> binary-v1 byte stream
CONTROL_UUID = "5a8e0003-9b1a-4c7d-8e2f-1f3a5b7c9d10"   # write  -> start/stop/restart/forget [wid]
WIFI_CREDS_UUID = "5a8e0004-9b1a-4c7d-8e2f-1f3a5b7c9d10"  # read -> JSON {ssid,password,ip,port,pi_id}
WEARABLES_UUID = "5a8e0005-9b1a-4c7d-8e2f-1f3a5b7c9d10"   # read -> JSON {"active":[{"wid","node"}]}

DEFAULT_DEVICE_NAME = "aurmor-rpi"
DEFAULT_CHUNK_SIZE = 180         # safe after MTU negotiation (notification <= MTU-3)
SCHEMA_VERSION = 2               # 2 = binary-v1 framing (was 1 = NDJSON)

# The receiver broadcasts which wearables are live (heard over its WiFi in the
# last few seconds) as manufacturer-specific data in its BLE advertisement, so
# the app's passive scan can show a BLE-silent provisioned ESP32 as detected
# WITHOUT connecting (a connection each poll is what prompted the iOS pairing
# dialog). Mirrored in aurmor-sports-mobile/features/esp32-provisioning/protocol.ts.
# 0xFFFF is the Bluetooth-SIG "internal/test" company id. Payload after the
# company id: [version=2, count, wid0_lo, wid0_hi, …] — an explicit list of live
# wids as u16 LE. (v1 was an 8-bit bitmask capped at wids 1–8; MAC-derived wids
# are 16-bit, so the list form replaces it.)
PRESENCE_MFG_ID = 0xFFFF
PRESENCE_VERSION = 2
# Cap so the manufacturer data fits a single BLE advertisement (company id +
# [version, count] + 2 bytes/wid). 10 wids => 24 bytes, within the ~29-byte
# manufacturer-data budget of a legacy advertisement.
PRESENCE_MAX_WIDS = 10


def presence_manufacturer_data(active_wids) -> list:
    """Payload bytes ([version, count, wid0_lo, wid0_hi, …]) for the presence
    advertisement — an explicit u16-LE list of live wids."""
    wids = sorted({int(w) & 0xFFFF for w in active_wids})[:PRESENCE_MAX_WIDS]
    out = [PRESENCE_VERSION, len(wids)]
    for wid in wids:
        out.append(wid & 0xFF)
        out.append((wid >> 8) & 0xFF)
    return out

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
MSG_IMPACT = 4      # one discrete head impact (see encode_impact_binary)

# Impact detection threshold, in g. The wearable owns this constant
# (impact_det.h IMPACT_THRESHOLD_G) and puts it on every record; this copy is a
# default for anything that has to render before the first impact arrives.
IMPACT_THRESHOLD_G = 20.0
IMPACT_G_SCALE = 100        # u16/u32 hundredths of a g

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


def encode_impact_binary(impact: Dict[str, object], node_idx: int) -> bytes:
    """Pack one impact as a MSG_IMPACT payload (no framing/chunking).

    21 bytes, byte-identical to what ble_stream.cpp emits when a phone reads
    straight off a wearable:

        node_idx:u8 | seq:u16 | t_ms:u32 | peak_g:u16/100 | threshold_g:u16/100
        | dur_ms:u16 | count:u16 | max_g:u16/100 | sum_g:u32/100

    The layout is FIXED and deliberately absent from FIELD_SPECS. Impacts are
    sparse and individually meaningful, so one must stay decodable by a client
    that reconnected and has not re-read Meta yet — and keeping it out of the
    live layout is what lets the pinned Meta/sample fixtures stay unchanged.

    ``count``/``max_g``/``sum_g`` are the wearable's own running totals since
    ITS boot, not since the session. They are carried for loss detection only:
    a receiver can tell "3 impacts happened" from "3 impacts arrived".
    """
    return b"".join([
        bytes([node_idx & 0xFF]),
        _pack_value("u16", 1, impact.get("seq", 0)),
        _pack_value("u32", 1, impact.get("t_ms", 0)),
        _pack_value("u16", IMPACT_G_SCALE, impact.get("peak_g", 0.0)),
        _pack_value("u16", IMPACT_G_SCALE, impact.get("threshold_g", IMPACT_THRESHOLD_G)),
        _pack_value("u16", 1, impact.get("dur_ms", 0)),
        _pack_value("u16", 1, impact.get("count", 0)),
        _pack_value("u16", IMPACT_G_SCALE, impact.get("max_g", 0.0)),
        _pack_value("u32", IMPACT_G_SCALE, impact.get("sum_g", 0.0)),
    ])


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


def build_live_protocol_meta(nodes: List[str]):
    """Decode tables for the live UDP source: every node shares one IMU+bio
    layout.

    Same return shape as build_protocol_meta (field_specs, layouts, node_layout)
    so encode_sample_binary and the app's decoder work unchanged.
    """
    field_specs: Dict[str, List] = {f: list(FIELD_SPECS[f]) for f in LIVE_IMU_FIELDS}
    for f in LIVE_BIO_FIELDS:
        field_specs[f] = list(_LIVE_BIO_SPECS[f])
    layouts = [LIVE_IMU_FIELDS + LIVE_BIO_FIELDS]
    node_layout = [0] * len(nodes)
    return field_specs, layouts, node_layout


def build_protocol_meta(frames, nodes: List[str]):
    """Derive the decode tables published in Meta from the loaded frames.

    Returns ``(field_specs, layouts, node_layout)`` where ``field_specs`` maps
    every field name -> [type, scale], ``layouts`` is the de-duplicated list of
    ordered field-name lists (excluding the header keys ``node``/``t_s``), and
    ``node_layout[i]`` indexes ``layouts`` for ``nodes[i]``.
    """
    node_fields: Dict[str, List[str]] = {}
    rep: Dict[str, object] = {}
    for frame in frames:
        for node, sample in frame.samples:
            if node not in node_fields:
                node_fields[node] = [k for k in sample.keys()
                                     if k not in ("node", "t_s")]
            for key, value in sample.items():
                rep.setdefault(key, value)

    field_specs: Dict[str, List] = {}
    for fields in node_fields.values():
        for field in fields:
            if field not in field_specs:
                typ, scale = resolve_spec(field, rep.get(field))
                field_specs[field] = [typ, scale]

    layouts: List[List[str]] = []
    layout_index: Dict[tuple, int] = {}
    node_layout: List[int] = []
    for node in nodes:
        fields = node_fields.get(node, [])
        key = tuple(fields)
        if key not in layout_index:
            layout_index[key] = len(layouts)
            layouts.append(fields)
        node_layout.append(layout_index[key])
    return field_specs, layouts, node_layout
