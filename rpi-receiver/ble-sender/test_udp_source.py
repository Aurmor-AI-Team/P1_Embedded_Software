"""Loopback tests for the live UDP source + wifi_ap config/nmcli handling.

Run: ``python3 test_udp_source.py`` (stdlib only; no BLE/bluezero/nmcli needed).
Style matches test_binary_protocol.py: plain asserts, exit code 0 on success.
"""
from __future__ import annotations

import json
import socket
import struct
import sys
import tempfile
import time
from pathlib import Path

import protocol
import udp_source
import wifi_ap
from test_binary_protocol import decode_sample, iter_records
from udp_source import (FORGET, HELLO, IMU_BODY, HDR, MSG_FORGET, MSG_IMU,
                        MSG_WELCOME, UdpImuSource, VERSION, WELCOME)


def make_imu(wid: int, seq: int, t_ms: int, values=None) -> bytes:
    # 10 IMU floats + 4 bio floats (hr, spo2, resp, hrv).
    values = values or (0.1, -0.2, -1.0, 1.5, -2.5, 3.5, 0.01, 0.02, -0.99, 25.5,
                        88.0, 97.0, 18.0, 42.0)
    return HDR.pack(MSG_IMU, VERSION, wid) + IMU_BODY.pack(seq, t_ms, *values)


def wait_for(predicate, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def run_source_tests() -> int:
    failures = 0
    src = UdpImuSource(port=0, pi_id=42, max_queue=8)
    src.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.bind(("127.0.0.1", 0))
    client.settimeout(2.0)
    dest = ("127.0.0.1", src.port)

    # HELLO -> WELCOME echoes the nonce and carries our pi_id.
    client.sendto(HELLO.pack(udp_source.MSG_HELLO, VERSION, 1, 0xDEADBEEF), dest)
    data, _ = client.recvfrom(64)
    mt, ver, wid, nonce, pi_id = WELCOME.unpack(data)
    assert (mt, ver, wid, nonce, pi_id) == (MSG_WELCOME, VERSION, 1, 0xDEADBEEF, 42), \
        (mt, ver, wid, nonce, pi_id)
    print("HELLO -> WELCOME OK")

    # IMU from a mapped wid becomes a HEAD sample with the live field names.
    client.sendto(make_imu(1, seq=7, t_ms=10_000), dest)
    assert wait_for(lambda: len(src._queue) >= 1), "IMU sample never queued"
    (sample,) = src.drain()
    assert sample["node"] == "HEAD", sample
    assert sample["t_s"] == 0.0, sample  # first packet defines the baseline
    assert list(sample.keys()) == (
        ["node", "t_s"] + protocol.LIVE_IMU_FIELDS + protocol.LIVE_BIO_FIELDS), \
        list(sample.keys())
    assert abs(sample["az_g"] + 1.0) < 1e-6 and abs(sample["imu_temp_c"] - 25.5) < 1e-4
    # Bio fields carried through from the packet.
    assert sample["ecg_hr_bpm"] == 88.0 and sample["ppg_spo2_pct"] == 97.0
    assert sample["resp_rate_bpm"] == 18.0 and sample["ecg_rmssd_ms"] == 42.0

    # Later packet: t_s is the offset from the baseline.
    client.sendto(make_imu(1, seq=8, t_ms=10_255), dest)
    assert wait_for(lambda: len(src._queue) >= 1)
    (sample,) = src.drain()
    assert sample["t_s"] == 0.255, sample["t_s"]
    print("IMU -> sample mapping + t_s baseline OK")

    # A second board (wid 2) maps to its own node — streams never collide.
    client.sendto(make_imu(2, seq=1, t_ms=500), dest)
    assert wait_for(lambda: len(src._queue) >= 1)
    (sample,) = src.drain()
    assert sample["node"] == "WA", sample
    assert src.nodes == ["HEAD", "WA", "WD", "WE"], src.nodes
    print("multi-board wid->node mapping OK")

    # Unknown wid: handshake works but no sample is queued.
    client.sendto(make_imu(99, seq=1, t_ms=5), dest)
    time.sleep(0.15)
    assert src.drain() == [], "unmapped wid must not be streamed"
    print("unmapped wid ignored OK")

    # Board reboot (t_ms jumps far backwards): t_s stays monotonic.
    client.sendto(make_imu(1, seq=9, t_ms=100), dest)
    assert wait_for(lambda: len(src._queue) >= 1)
    (sample,) = src.drain()
    assert sample["t_s"] > 0.255, f"t_s went backwards after reboot: {sample['t_s']}"
    print(f"reboot re-base OK (t_s={sample['t_s']})")

    # Overflow: deque keeps only the newest max_queue samples.
    for i in range(20):
        client.sendto(make_imu(1, seq=100 + i, t_ms=200_000 + i * 10), dest)
    assert wait_for(lambda: len(src._queue) == 8)
    time.sleep(0.1)
    drained = src.drain()
    assert len(drained) == 8, len(drained)
    assert drained[-1]["t_s"] == max(s["t_s"] for s in drained)
    print("overflow drop-oldest OK")

    # Active wearables: both boards heard recently, age out together, unmapped
    # wids never appear (presence source for the app's devices tab).
    assert src.active_wearables() == {1: "HEAD", 2: "WA"}, src.active_wearables()
    assert src.active_wearables(max_age_s=0.0) == {}
    assert 99 not in src.active_wearables(max_age_s=3600)
    print("active wearables tracking OK")

    # Presence advertisement payload: [version, bitmask]; the app prepends the
    # 2-byte company id and reads bit (wid-1). Mirror that decode here.
    assert protocol.presence_manufacturer_data([]) == [1, 0]
    assert protocol.presence_manufacturer_data([1]) == [1, 0b0000_0001]
    assert protocol.presence_manufacturer_data([1, 3]) == [1, 0b0000_0101]
    assert protocol.presence_manufacturer_data([9]) == [1, 0], "wid>8 ignored"
    print("presence manufacturer data OK")

    # FORGET goes to the wid's last-seen address with our pi_id.
    assert src.send_forget(1, retries=2, interval_s=0.01) is True
    data, _ = client.recvfrom(64)
    mt, ver, wid, pi_id = FORGET.unpack(data)
    assert (mt, ver, wid, pi_id) == (MSG_FORGET, VERSION, 1, 42), (mt, ver, wid, pi_id)
    assert src.send_forget(55) is False, "unknown wid must report undeliverable"
    print("FORGET layout + addressing OK")

    src.stop()
    client.close()
    return failures


def run_live_meta_tests() -> int:
    """A live-mode sample (IMU + bio) must round-trip through the app decoder."""
    nodes = ["HEAD"]
    field_specs, layouts, node_layout = protocol.build_live_protocol_meta(nodes)
    assert layouts == [protocol.LIVE_IMU_FIELDS + protocol.LIVE_BIO_FIELDS], layouts
    assert node_layout == [0]

    sample = {"node": "HEAD", "t_s": 1.275,
              "ax_g": 0.101, "ay_g": -0.202, "az_g": -1.0,
              "gx_dps": 1.5, "gy_dps": -2.5, "gz_dps": 3.5,
              "hx_g": 0.01, "hy_g": 0.02, "hz_g": -0.99,
              "imu_temp_c": 25.53,
              "ecg_hr_bpm": 116, "ppg_spo2_pct": 96.5,
              "resp_rate_bpm": 22, "ecg_rmssd_ms": 44}
    payload = protocol.encode_sample_binary(sample, 0, layouts[0], field_specs)
    record = protocol.frame_record(protocol.MSG_SAMPLE, payload)
    stream = b"".join(protocol.chunk_bytes(record, 13))
    meta = {"nodes": nodes, "field_specs": field_specs,
            "layouts": layouts, "node_layout": node_layout}
    (mt, body), = list(iter_records(stream))
    assert mt == protocol.MSG_SAMPLE
    decoded = decode_sample(body, meta)
    assert decoded["node"] == "HEAD" and decoded["t_s"] == 1.275, decoded
    for field in protocol.LIVE_IMU_FIELDS + protocol.LIVE_BIO_FIELDS:
        typ, scale = field_specs[field]
        assert abs(decoded[field] - sample[field]) <= 1.0 / scale + 1e-9, \
            (field, decoded[field], sample[field])
    print("live meta round-trip (IMU+bio) through reference decoder OK")
    return 0


def run_wifi_ap_tests() -> int:
    # Config generation: a unique per-Pi identity + secrets, created once and
    # re-read verbatim.
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "receiver_config.json"
        cfg = wifi_ap.load_config(path)
        suffix = cfg["ap_ssid"].rsplit("-", 1)[-1]
        assert cfg["ap_ssid"] == f"aurmor-pi-{suffix}"
        assert cfg["ap_ssid"] != "aurmor-pi-ap", "SSID must be unique per Pi"
        assert cfg["receiver_name"] == f"aurmor-rpi-{suffix}", "name shares the SSID suffix"
        assert len(cfg["ap_password"]) >= 8, "WPA2 needs >= 8 chars"
        assert 1 <= cfg["pi_id"] <= 0xFFFFFFFF
        assert cfg["udp_port"] == 5005
        again = wifi_ap.load_config(path)
        assert again == cfg, "identity + secrets must be stable across loads"
        on_disk = json.loads(path.read_text())
        assert on_disk == cfg
    print("receiver config generation OK (unique identity)")

    # A config still carrying the legacy shared SSID is migrated to a unique one.
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "receiver_config.json"
        path.write_text(json.dumps({"ap_ssid": "aurmor-pi-ap"}))
        cfg = wifi_ap.load_config(path)
        assert cfg["ap_ssid"] != "aurmor-pi-ap", "legacy shared SSID must be migrated"
        assert cfg["receiver_name"].startswith("aurmor-rpi-")
    print("legacy SSID migration OK")

    # ensure_ap drives nmcli: add when absent, modify when present, then up.
    calls = []

    class FakeResult:
        def __init__(self, stdout=""):
            self.returncode = 0
            self.stdout = stdout
            self.stderr = ""

    def fake_run(argv, capture_output=True, text=True, **kwargs):
        calls.append(argv)
        if argv[1:4] == ["-t", "-f", "NAME"]:
            return FakeResult(stdout="lo\npreconfigured\n")
        return FakeResult()

    cfg = {"ap_ssid": "aurmor-pi-3f48", "ap_password": "secret123", "pi_id": 7,
           "udp_port": 5005, "ap_con_name": "aurmor-ap", "ap_ifname": "wlan0",
           "ap_channel": 6, "receiver_name": "aurmor-rpi-3f48"}
    real_run = wifi_ap.subprocess.run
    wifi_ap.subprocess.run = fake_run
    try:
        assert wifi_ap.ensure_ap(cfg) is True
    finally:
        wifi_ap.subprocess.run = real_run

    assert calls[0][:4] == ["nmcli", "-t", "-f", "NAME"]
    add = calls[1]
    assert add[:4] == ["nmcli", "connection", "add", "type"], add
    for expected in ("802-11-wireless.hidden", "ipv4.method", "wifi-sec.psk",
                     "802-11-wireless.mode", "802-11-wireless.channel",
                     "wifi-sec.proto", "wifi-sec.pairwise", "wifi-sec.pmf"):
        assert expected in add, f"missing {expected} in nmcli add"
    assert add[add.index("802-11-wireless.hidden") + 1] == "yes"
    assert add[add.index("ipv4.method") + 1] == "shared"
    assert add[add.index("802-11-wireless.channel") + 1] == "6"
    assert add[add.index("wifi-sec.proto") + 1] == "rsn"
    assert add[add.index("wifi-sec.pmf") + 1] == "disable"
    assert calls[2] == ["nmcli", "connection", "up", "aurmor-ap"], calls[2]
    print("ensure_ap nmcli argv OK (hidden AP, shared ipv4, add->up)")

    # Already configured + active: the whole check must stay read-only (no
    # add/modify/up), so unprivileged runs succeed after a one-time sudo setup.
    calls.clear()

    def fake_run_active(argv, capture_output=True, text=True, **kwargs):
        calls.append(argv)
        if argv[1:4] == ["-t", "-f", "NAME"]:  # both the list and --active query
            return FakeResult(stdout="lo\naurmor-ap\n")
        if argv[1] == "-g":  # settings probe: ssid, hidden, mode, channel
            return FakeResult(stdout="aurmor-pi-3f48\nyes\nap\n6\n")
        raise AssertionError(f"unexpected nmcli write call: {argv}")

    wifi_ap.subprocess.run = fake_run_active
    try:
        assert wifi_ap.ensure_ap(cfg) is True
    finally:
        wifi_ap.subprocess.run = real_run
    assert not any(a[1] == "connection" and a[2] in ("add", "modify", "up")
                   for a in calls), calls
    print("ensure_ap read-only fast path OK (already configured + active)")

    # creds JSON matches the characteristic contract.
    creds = json.loads(wifi_ap.wifi_creds_json(cfg))
    assert creds == {"ssid": "aurmor-pi-3f48", "password": "secret123",
                     "ip": "10.42.0.1", "port": 5005, "pi_id": 7,
                     "receiver_name": "aurmor-rpi-3f48"}, creds
    print("wifi creds JSON OK")
    return 0


def main() -> int:
    failures = run_source_tests()
    failures += run_live_meta_tests()
    failures += run_wifi_ap_tests()
    if failures:
        print(f"FAILURES: {failures}")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
