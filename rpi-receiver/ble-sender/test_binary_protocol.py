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


def main() -> int:
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
