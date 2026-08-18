"""Round-trip + upload-fidelity tests for the binary-v1 wire format.

Run: ``python3 test_binary_protocol.py`` (stdlib only; no BLE/bluezero needed).

The critical check is *upload fidelity*: the mobile app decodes a binary frame
back into an object and the session recorder uploads ``JSON.stringify`` of it. So
the bar is that our decode, re-serialized as compact JSON, matches what the app
*currently* uploads — which is ``JSON.stringify(JSON.parse(line))`` (note: JS
canonicalises 1.0 -> 1, so we compare against the canonicalised line, not the raw
Pi NDJSON line).
"""
from __future__ import annotations

import hashlib
import hmac
import json
import struct
import sys

import protocol
import replay


# --------------------------------------------------------------------------- #
# A reference decoder that mirrors the TypeScript app (features/ble-stream).
# --------------------------------------------------------------------------- #
def _read_value(view: memoryview, off: int, typ: str, scale: int):
    if typ == "i16":
        return struct.unpack_from("<h", view, off)[0] / scale, off + 2
    if typ == "u16":
        return struct.unpack_from("<H", view, off)[0] / scale, off + 2
    if typ == "i32":
        return struct.unpack_from("<i", view, off)[0] / scale, off + 4
    if typ == "u32":
        return struct.unpack_from("<I", view, off)[0] / scale, off + 4
    if typ == "f32":
        return struct.unpack_from("<f", view, off)[0], off + 4
    if typ == "str":
        n = view[off]
        return bytes(view[off + 1:off + 1 + n]).decode("utf-8"), off + 1 + n
    raise ValueError(typ)


def decode_sample(payload: bytes, meta: dict) -> dict:
    view = memoryview(payload)
    node_idx = view[0]
    t_ms = struct.unpack_from("<I", view, 1)[0]
    out = {"node": meta["nodes"][node_idx], "t_s": t_ms / 1000}
    off = 5
    for field in meta["layouts"][meta["node_layout"][node_idx]]:
        typ, scale = meta["field_specs"][field]
        value, off = _read_value(view, off, typ, scale)
        out[field] = value
    return out


def decode_pose(payload: bytes) -> dict:
    view = memoryview(payload)
    t_ms = struct.unpack_from("<I", view, 0)[0]
    tran = [struct.unpack_from("<h", view, 4 + 2 * i)[0] / 1000 for i in range(3)]
    q = []
    off = 10
    while off + 8 <= len(payload):
        q.append([struct.unpack_from("<h", view, off + 2 * i)[0] / 10000
                  for i in range(4)])
        off += 8
    return {"type": "pose", "t_s": t_ms / 1000, "tran": tran, "q": q}


def iter_records(stream: bytes):
    """Reassemble length-prefixed records exactly like BinaryFrameAssembler."""
    off = 0
    while off + 3 <= len(stream):
        msg_type, length = struct.unpack_from("<BH", stream, off)
        body = stream[off + 3:off + 3 + length]
        assert len(body) == length, "truncated record"
        yield msg_type, body
        off += 3 + length


# --------------------------------------------------------------------------- #
# JS-canonicalised JSON: numbers go through float64 and 1.0 prints as "1".
# --------------------------------------------------------------------------- #
def js_canonical(obj) -> str:
    def norm(v):
        if isinstance(v, float) and v.is_integer():
            return int(v)
        if isinstance(v, dict):
            return {k: norm(x) for k, x in v.items()}
        if isinstance(v, list):
            return [norm(x) for x in v]
        return v
    return json.dumps(norm(obj), separators=(",", ":"))


# --------------------------------------------------------------------------- #
# ESP32 direct-stream fixtures.
#
# A solo session skips the Pi entirely: the phone connects to the wearable and
# the BOARD serves this same binary-v1 contract (peq0-v1-head-tests/components/
# peripherals/ble_stream.cpp). That firmware is hand-written C mirroring the
# encoder below, in another repo, with no shared code — so these two fixtures are
# the only thing standing between a scale typo and a silently wrong sample.
#
# Both are byte-exact on purpose: if either side is edited, this fails.
# --------------------------------------------------------------------------- #

# What ble_stream.cpp's build_meta() emits for a board whose wid is 0xAAAA.
ESP32_META_JSON = (
    '{"exercise":"live-ble","period_ms":100,"fps":10.00,"frames":0,'
    '"nodes":["AAAA"],"chunk_size":180,"framing":"binary-v1","schema":2,'
    '"field_specs":{'
    '"ax_g":["i16",1000],"ay_g":["i16",1000],"az_g":["i16",1000],'
    '"gx_dps":["i16",10],"gy_dps":["i16",10],"gz_dps":["i16",10],'
    '"hx_g":["i16",1000],"hy_g":["i16",1000],"hz_g":["i16",1000],'
    '"imu_temp_c":["i16",100],'
    '"ecg_hr_bpm":["u16",1],"ppg_spo2_pct":["u16",100],'
    '"resp_rate_bpm":["u16",1],"ecg_rmssd_ms":["u16",1]},'
    '"layouts":[["ax_g","ay_g","az_g",'
    '"gx_dps","gy_dps","gz_dps",'
    '"hx_g","hy_g","hz_g","imu_temp_c",'
    '"ecg_hr_bpm","ppg_spo2_pct","resp_rate_bpm","ecg_rmssd_ms"]],'
    '"node_layout":[0]}'
)

# The reference sample the byte fixture below encodes.
ESP32_SAMPLE = {
    "node": "AAAA", "t_s": 1234.567,
    "ax_g": 1.5, "ay_g": -0.25, "az_g": 0.0,
    "gx_dps": 100.0, "gy_dps": -45.5, "gz_dps": 0.0,
    "hx_g": 2.0, "hy_g": -3.5, "hz_g": 0.0,
    "imu_temp_c": 25.5,
    # Zero from a real sensor, exactly as over UDP; the mock fills them.
    "ecg_hr_bpm": 0, "ppg_spo2_pct": 0, "resp_rate_bpm": 0, "ecg_rmssd_ms": 0,
}

# One full MSG_SAMPLE record as ble_stream.cpp puts it on the wire:
# msg_type=1 | len=33 (0x0021 LE) | node_idx=0 | t_ms=1234567 | 10 i16 | 4 u16.
ESP32_SAMPLE_RECORD = bytes([
    0x01, 0x21, 0x00,                     # msg_type, length (u16 LE)
    0x00,                                 # node_idx
    0x87, 0xD6, 0x12, 0x00,               # t_ms = 1234567 (u32 LE)
    0xDC, 0x05,                           # ax_g   1.5    * 1000 =  1500
    0x06, 0xFF,                           # ay_g  -0.25   * 1000 =  -250
    0x00, 0x00,                           # az_g   0.0
    0xE8, 0x03,                           # gx_dps 100.0  *   10 =  1000
    0x39, 0xFE,                           # gy_dps -45.5  *   10 =  -455
    0x00, 0x00,                           # gz_dps 0.0
    0xD0, 0x07,                           # hx_g   2.0    * 1000 =  2000
    0x54, 0xF2,                           # hy_g  -3.5    * 1000 = -3500
    0x00, 0x00,                           # hz_g   0.0
    0xF6, 0x09,                           # imu_temp_c 25.5 * 100 = 2550
    0x00, 0x00,                           # ecg_hr_bpm
    0x00, 0x00,                           # ppg_spo2_pct
    0x00, 0x00,                           # resp_rate_bpm
    0x00, 0x00,                           # ecg_rmssd_ms
])


# The enrolment challenge vector, shared with the other two repos.
#
# A claimed wearable only accepts WiFi credentials (or serves its stream) over a
# connection that answered HMAC-SHA256(secret, nonce). Three independent
# implementations have to agree: the app's hand-rolled one
# (features/esp32-provisioning/hmac.ts), mbedtls on the board
# (components/peripherals/ble_auth.cpp), and hashlib here.
#
# The algorithm is standard; what actually breaks across languages is the
# FRAMING — hashing the hex TEXT instead of the decoded bytes yields a valid-
# looking HMAC that the board rejects every single time, with no clue why.
AUTH_SECRET_HEX = "000102030405060708090a0b0c0d0e0f"
AUTH_NONCE_HEX = "a1b2c3d4e5f60718293a4b5c6d7e8f90"
AUTH_RESPONSE_HEX = "87aa1655fc85974cb343b827ef88fb11fc81db7c3c485230bb02a59004a0a46c"


def check_auth_challenge_fixture() -> None:
    got = hmac.new(bytes.fromhex(AUTH_SECRET_HEX),
                   bytes.fromhex(AUTH_NONCE_HEX),
                   hashlib.sha256).hexdigest()
    assert got == AUTH_RESPONSE_HEX, (
        f"enrolment challenge vector drifted:\n  got:  {got}\n  want: {AUTH_RESPONSE_HEX}"
    )
    # Hashing the hex text instead of the bytes must NOT accidentally match.
    wrong = hmac.new(AUTH_SECRET_HEX.encode(), AUTH_NONCE_HEX.encode(),
                     hashlib.sha256).hexdigest()
    assert wrong != AUTH_RESPONSE_HEX
    print("enrolment challenge fixture OK (bytes, not hex text)")


def check_esp32_meta_fixture() -> None:
    """The board's hard-coded descriptor must carry the SAME decode tables the
    Pi publishes for a live wearable — otherwise the app decodes the board's
    samples against the wrong scales and reads plausible but wrong numbers."""
    meta = json.loads(ESP32_META_JSON)
    field_specs, layouts, node_layout = protocol.build_live_protocol_meta(["AAAA"])

    assert meta["framing"] == "binary-v1", meta["framing"]
    assert meta["schema"] == protocol.SCHEMA_VERSION, meta["schema"]
    assert meta["nodes"] == ["AAAA"], meta["nodes"]
    assert meta["field_specs"] == field_specs, (
        f"ESP32 field_specs drifted from the Pi's:\n"
        f"  board: {meta['field_specs']}\n  pi:    {field_specs}"
    )
    assert meta["layouts"] == layouts, (
        f"ESP32 layout drifted:\n  board: {meta['layouts']}\n  pi:    {layouts}"
    )
    assert meta["node_layout"] == node_layout, meta["node_layout"]
    # The board sizes its notification buffer from this; the app never uses it,
    # but a mismatch means the firmware is chunking against a stale assumption.
    assert meta["chunk_size"] == 180, meta["chunk_size"]
    print("ESP32 Meta fixture OK (decode tables identical to the Pi's)")


def check_esp32_stream_fixture() -> None:
    """The board's bytes for a known sample must be byte-identical to the Pi's,
    and decode back to the original values."""
    field_specs, layouts, _ = protocol.build_live_protocol_meta(["AAAA"])
    expected = protocol.frame_record(
        protocol.MSG_SAMPLE,
        protocol.encode_sample_binary(ESP32_SAMPLE, 0, layouts[0], field_specs),
    )
    assert ESP32_SAMPLE_RECORD == expected, (
        f"ESP32 wire bytes differ from the Pi encoder:\n"
        f"  board: {ESP32_SAMPLE_RECORD.hex(' ')}\n"
        f"  pi:    {bytes(expected).hex(' ')}"
    )

    # And it survives the app's reassembly at a hostile chunk boundary.
    meta = json.loads(ESP32_META_JSON)
    stream = b"".join(protocol.chunk_bytes(ESP32_SAMPLE_RECORD, 5))
    (msg_type, body), = list(iter_records(stream))
    assert msg_type == protocol.MSG_SAMPLE, msg_type
    decoded = decode_sample(body, meta)
    for key, want in ESP32_SAMPLE.items():
        got = decoded[key]
        if isinstance(want, str):
            assert got == want, f"{key}: {got!r} != {want!r}"
        else:
            assert abs(float(got) - float(want)) <= 1e-6, f"{key}: {got} != {want}"
    print(f"ESP32 sample fixture OK ({len(ESP32_SAMPLE_RECORD)}B record, "
          f"byte-identical to the Pi encoder)")


def main() -> int:
    check_auth_challenge_fixture()
    check_esp32_meta_fixture()
    repo_root = __import__("pathlib").Path(__file__).resolve().parent.parent
    data_dir = repo_root / "mock-csv" / "10_pushups_biometric_data_simulation"
    frames, nodes = replay.load_frames(data_dir)
    field_specs, layouts, node_layout = protocol.build_protocol_meta(frames, nodes)
    meta = {"nodes": nodes, "field_specs": field_specs,
            "layouts": layouts, "node_layout": node_layout}

    node_index = {n: i for i, n in enumerate(nodes)}
    chunk_size = 180

    checked = 0
    json_bytes = 0
    bin_bytes = 0
    mismatches = 0

    for frame in frames:
        for node, sample in frame.samples:
            idx = node_index[node]
            layout = layouts[node_layout[idx]]
            payload = protocol.encode_sample_binary(sample, idx, layout, field_specs)
            record = protocol.frame_record(protocol.MSG_SAMPLE, payload)

            # Reassemble across deliberately tiny chunk boundaries.
            stream = b"".join(protocol.chunk_bytes(record, 7))
            (msg_type, body), = list(iter_records(stream))
            assert msg_type == protocol.MSG_SAMPLE

            decoded = decode_sample(body, meta)

            # What the app uploads today vs. what it would upload now.
            current = js_canonical(json.loads(
                protocol.sample_to_ndjson(sample).decode().strip()))
            new = js_canonical(decoded)

            checked += 1
            json_bytes += len(protocol.sample_to_ndjson(sample))
            bin_bytes += len(record)

            if json.loads(current).keys() != json.loads(new).keys():
                print("KEY MISMATCH:\n  old:", current, "\n  new:", new)
                mismatches += 1
                continue
            # Values must match within each field's scale resolution.
            old_obj, new_obj = json.loads(current), json.loads(new)
            for k in old_obj:
                a, b = old_obj[k], new_obj[k]
                if isinstance(a, str):
                    ok = a == b
                else:
                    typ, scale = ("u32", 1000) if k == "t_s" else \
                        field_specs.get(k, ("i32", 1))
                    tol = 1.0 / scale if scale else 0
                    ok = abs(float(a) - float(b)) <= tol + 1e-9
                if not ok:
                    print(f"VALUE MISMATCH node={node} {k}: {a} != {b}")
                    mismatches += 1

    check_esp32_stream_fixture()

    # Pose round-trip (synthetic SMPL frame: 24 joint quaternions + translation).
    tran = [0.123, -1.045, 2.5]
    quats = [[1.0, 0.0, 0.0, 0.0]] * 23 + [[0.7071, 0.7071, 0.0, 0.0]]
    pose_payload = protocol.encode_pose_binary(0.255, tran, quats)
    pose_record = protocol.frame_record(protocol.MSG_POSE, pose_payload)
    (mt, body), = list(iter_records(b"".join(protocol.chunk_bytes(pose_record, 13))))
    assert mt == protocol.MSG_POSE
    pose = decode_pose(body)
    assert pose["t_s"] == 0.255, pose["t_s"]
    assert all(abs(a - b) <= 1e-3 for a, b in zip(pose["tran"], tran)), pose["tran"]
    assert all(abs(a - b) <= 1e-4 for qa, qb in zip(pose["q"], quats)
               for a, b in zip(qa, qb)), "pose quaternion mismatch"
    print("pose round-trip OK (tran<=1e-3, quat<=1e-4)")

    print(f"checked {checked} samples across {len(frames)} frames, "
          f"{len(nodes)} nodes")
    print(f"field types in use: "
          f"{sorted({tuple(v) for v in field_specs.values()})}")
    print(f"size: NDJSON={json_bytes}B  binary-v1={bin_bytes}B  "
          f"ratio={json_bytes / bin_bytes:.2f}x")
    print(f"mismatches: {mismatches}")
    if mismatches:
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
