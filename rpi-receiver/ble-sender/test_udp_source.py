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
from udp_source import (ALERT, ALERT_ACK, FORGET, HELLO, IMU_BODY, HDR,
                        MSG_ALERT, MSG_ALERT_ACK, MSG_FORGET, MSG_IMU,
                        MSG_WELCOME, UdpImuSource, VERSION, WELCOME)


def make_imu(wid: int, seq: int, t_ms: int, values=None) -> bytes:
    # 10 IMU floats + 4 bio floats (hr, spo2, resp, hrv).
    values = values or (0.1, -0.2, -1.0, 1.5, -2.5, 3.5, 0.01, 0.02, -0.99, 25.5,
                        88.0, 97.0, 18.0, 42.0)
    return HDR.pack(MSG_IMU, VERSION, wid) + IMU_BODY.pack(seq, t_ms, *values)


def make_alert(wid: int, seq: int, t_ms: int, peak_g: float = 41.23,
               threshold_g: float = 20.0, sum_g: float = 88.41,
               max_g: float = 41.23, count: int = 3, dur_ms: int = 18) -> bytes:
    return ALERT.pack(MSG_ALERT, VERSION, wid, seq, t_ms,
                      peak_g, threshold_g, sum_g, max_g, count, dur_ms)


def wait_for(predicate, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def run_source_tests() -> int:
    failures = 0
    # Explicit body-position map keeps the legacy HEAD/WA/WD/WE labels for these
    # assertions; the live default (no map) is covered by run_dynamic_node_tests.
    src = UdpImuSource(port=0, pi_id=42, max_queue=8, wid_to_node=udp_source.WID_TO_NODE)
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

    # A wid not in the explicit map still streams — under its MAC-derived hex
    # node (99 == 0x0063). No board is dropped now that identity is the wid.
    client.sendto(make_imu(99, seq=1, t_ms=5), dest)
    assert wait_for(lambda: len(src._queue) >= 1)
    (sample,) = src.drain()
    assert sample["node"] == "0063", sample
    print("unknown-to-map wid streams under hex node OK")

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

    # Active wearables: every board heard recently appears (wid -> node),
    # including ones outside the explicit map (they use the hex node). Presence
    # source for the app's devices tab.
    active = src.active_wearables()
    assert active.get(1) == "HEAD" and active.get(2) == "WA" and active.get(99) == "0063", active
    assert src.active_wearables(max_age_s=0.0) == {}
    print("active wearables tracking OK")

    # Presence advertisement payload: [version=2, count, wid_lo, wid_hi, …]; the
    # app prepends the 2-byte company id and reads the u16-LE list. Mirror here.
    assert protocol.presence_manufacturer_data([]) == [2, 0]
    assert protocol.presence_manufacturer_data([1]) == [2, 1, 1, 0]
    assert protocol.presence_manufacturer_data([0x5AF3]) == [2, 1, 0xF3, 0x5A]
    assert protocol.presence_manufacturer_data([2, 1]) == [2, 2, 1, 0, 2, 0]  # sorted
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


def run_alert_tests() -> int:
    """Impact alerts: acked always, deduped, and on the sample timeline."""
    failures = 0
    src = UdpImuSource(port=0, pi_id=42, max_queue=8,
                       wid_to_node=udp_source.WID_TO_NODE)
    src.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.bind(("127.0.0.1", 0))
    client.settimeout(2.0)
    dest = ("127.0.0.1", src.port)

    # Struct layouts must match wifi_udp_tx.cpp, which asserts the same sizes.
    assert ALERT.size == 34, ALERT.size
    assert ALERT_ACK.size == 8, ALERT_ACK.size

    # A sample first, so the impact has an established timeline to land on.
    client.sendto(make_imu(1, seq=1, t_ms=10_000), dest)
    assert wait_for(lambda: len(src._queue) >= 1)
    src.drain()

    # ALERT -> ACK echoing the seq, plus one queued impact record.
    client.sendto(make_alert(1, seq=5, t_ms=11_000), dest)
    data, _ = client.recvfrom(64)
    mt, ver, wid, seq = ALERT_ACK.unpack(data)
    assert (mt, ver, wid, seq) == (MSG_ALERT_ACK, VERSION, 1, 5), (mt, ver, wid, seq)
    assert wait_for(lambda: len(src._queue) >= 1)
    (impact,) = src.drain()
    assert impact["type"] == "impact", impact
    assert impact["node"] == "HEAD", impact
    assert abs(impact["peak_g"] - 41.23) < 1e-3, impact
    assert abs(impact["threshold_g"] - 20.0) < 1e-3, impact
    assert impact["seq"] == 5 and impact["dur_ms"] == 18, impact
    assert impact["count"] == 3, impact
    # Same clock as the samples: t_ms 11000 with t0 10000 -> 1.0 s.
    assert abs(impact["t_s"] - 1.0) < 1e-3, impact
    print("ALERT -> ACK + queued impact OK")

    # A retransmit (the board never heard our ack) must be acked AGAIN but not
    # queued twice — otherwise a lost ack inflates the athlete's impact count.
    client.sendto(make_alert(1, seq=5, t_ms=11_000), dest)
    data, _ = client.recvfrom(64)
    assert ALERT_ACK.unpack(data)[3] == 5
    time.sleep(0.1)
    assert src.drain() == [], "duplicate alert must not queue a second record"
    print("duplicate ALERT re-acked but deduped OK")

    # A genuinely new impact still gets through.
    client.sendto(make_alert(1, seq=6, t_ms=12_000, peak_g=55.5), dest)
    client.recvfrom(64)
    assert wait_for(lambda: len(src._queue) >= 1)
    (impact2,) = src.drain()
    assert impact2["seq"] == 6 and abs(impact2["peak_g"] - 55.5) < 1e-3, impact2
    print("subsequent ALERT queued OK")

    # Ordering with the sample stream is preserved (one queue, not two).
    client.sendto(make_imu(1, seq=2, t_ms=13_000), dest)
    time.sleep(0.05)
    client.sendto(make_alert(1, seq=7, t_ms=13_500), dest)
    client.recvfrom(64)
    time.sleep(0.05)
    client.sendto(make_imu(1, seq=3, t_ms=14_000), dest)
    assert wait_for(lambda: len(src._queue) >= 3)
    time.sleep(0.05)
    kinds = [r.get("type", "sample") for r in src.drain()]
    assert kinds == ["sample", "impact", "sample"], kinds
    print("alert/sample ordering preserved OK")

    # After a board reboot the impact must land on the REBASED timeline, next to
    # the samples around it — not back near zero.
    client.sendto(make_imu(1, seq=0, t_ms=100), dest)      # t_ms jumps back
    assert wait_for(lambda: len(src._queue) >= 1)
    time.sleep(0.05)
    (post_reboot,) = src.drain()
    client.sendto(make_alert(1, seq=1, t_ms=200), dest)
    client.recvfrom(64)
    assert wait_for(lambda: len(src._queue) >= 1)
    (impact3,) = src.drain()
    assert impact3["t_s"] >= post_reboot["t_s"], (impact3, post_reboot)
    print("post-reboot ALERT rebased onto the sample timeline OK")

    src.stop()
    client.close()
    return failures


def run_impact_requeue_tests() -> int:
    """A subscriber that discards a stale backlog must keep the impacts."""
    failures = 0
    src = UdpImuSource(port=0, pi_id=42, max_queue=16)
    src._queue.append({"node": "AAAA", "t_s": 1.0, "ax_g": 0.1})
    src._queue.append({"type": "impact", "node": "AAAA", "t_s": 1.5, "seq": 1,
                       "peak_g": 30.0})
    src._queue.append({"node": "AAAA", "t_s": 2.0, "ax_g": 0.2})
    src._queue.append({"type": "impact", "node": "AAAA", "t_s": 2.5, "seq": 2,
                       "peak_g": 40.0})

    stale = src.drain()
    held = [s for s in stale if s.get("type") == "impact"]
    src.requeue(held)

    kept = src.drain()
    assert [r["seq"] for r in kept] == [1, 2], kept
    assert all(r.get("type") == "impact" for r in kept), kept
    print("impacts survive a subscribe-time backlog discard OK")
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
        if argv[1] == "-s":  # privileged psk probe — matches the config
            return FakeResult(stdout="secret123\n")
        if argv[1] == "-g" and "security" in argv[2]:
            return FakeResult(stdout="wpa-psk\nrsn\nccmp\nccmp\n1 (disable)\n")
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

    # PSK DRIFT: the profile still carries our SSID (it is derived from the board
    # serial and survives a config regeneration) but a stale password. Left
    # alone, wearables associate and then die in the 4-way handshake — ESP32
    # disconnect reason 15 — which reads like a firmware fault. ensure_ap must
    # notice and rewrite the profile instead of reporting success.
    calls.clear()

    def fake_run_drifted(argv, capture_output=True, text=True, **kwargs):
        calls.append(argv)
        if argv[1:4] == ["-t", "-f", "NAME"]:
            return FakeResult(stdout="lo\naurmor-ap\n")
        if argv[1] == "-s":
            return FakeResult(stdout="a-different-old-password\n")
        if argv[1] == "-g" and "security" in argv[2]:
            return FakeResult(stdout="wpa-psk\nrsn\nccmp\nccmp\n1 (disable)\n")
        if argv[1] == "-g":
            return FakeResult(stdout="aurmor-pi-3f48\nyes\nap\n6\n")
        return FakeResult()

    wifi_ap.subprocess.run = fake_run_drifted
    try:
        assert wifi_ap.ensure_ap(cfg) is True
    finally:
        wifi_ap.subprocess.run = real_run
    modify = [a for a in calls if a[1:3] == ["connection", "modify"]]
    assert modify, f"drifted psk must trigger a reconfigure, got {calls}"
    assert modify[0][modify[0].index("wifi-sec.psk") + 1] == "secret123"
    assert any(a[1:3] == ["connection", "up"] for a in calls), calls
    print("ensure_ap repairs a drifted AP password OK")

    # Unprivileged: the psk is withheld (blank), so drift cannot be ruled out.
    # That must NOT trigger a rewrite we have no permission to make — the
    # existing profile is left alone and the operator is told how to check.
    calls.clear()

    def fake_run_blind(argv, capture_output=True, text=True, **kwargs):
        calls.append(argv)
        if argv[1:4] == ["-t", "-f", "NAME"]:
            return FakeResult(stdout="lo\naurmor-ap\n")
        if argv[1] == "-s":
            return FakeResult(stdout="")          # withheld without privileges
        if argv[1] == "-g" and "security" in argv[2]:
            return FakeResult(stdout="wpa-psk\nrsn\nccmp\nccmp\n1 (disable)\n")
        if argv[1] == "-g":
            return FakeResult(stdout="aurmor-pi-3f48\nyes\nap\n6\n")
        raise AssertionError(f"unexpected nmcli write call: {argv}")

    wifi_ap.subprocess.run = fake_run_blind
    try:
        assert wifi_ap.ensure_ap(cfg) is True
    finally:
        wifi_ap.subprocess.run = real_run
    assert not any(a[1] == "connection" and a[2] in ("add", "modify", "up")
                   for a in calls), calls
    print("ensure_ap stays read-only when the psk is unreadable OK")

    # An AP left on NetworkManager's defaults (WPA2/WPA3 transition) is up,
    # active and correctly named — and invisible to ESP32 stations, which ask
    # for WIFI_AUTH_WPA2_PSK and get NO_AP_FOUND. ensure_ap must repin it.
    calls.clear()

    def fake_run_unpinned(argv, capture_output=True, text=True, **kwargs):
        calls.append(argv)
        if argv[1:4] == ["-t", "-f", "NAME"]:
            return FakeResult(stdout="lo\naurmor-ap\n")
        if argv[1] == "-s":
            return FakeResult(stdout="secret123\n")
        if argv[1] == "-g" and "security" in argv[2]:
            return FakeResult(stdout="wpa-psk\n\n\n\n0 (default)\n")  # NM defaults
        if argv[1] == "-g":
            return FakeResult(stdout="aurmor-pi-3f48\nyes\nap\n6\n")
        return FakeResult()

    wifi_ap.subprocess.run = fake_run_unpinned
    try:
        assert wifi_ap.ensure_ap(cfg) is True
    finally:
        wifi_ap.subprocess.run = real_run
    modify = [a for a in calls if a[1:3] == ["connection", "modify"]]
    assert modify, f"unpinned security must trigger a reconfigure, got {calls}"
    assert modify[0][modify[0].index("wifi-sec.proto") + 1] == "rsn"
    assert modify[0][modify[0].index("wifi-sec.pmf") + 1] == "disable"
    print("ensure_ap repins a WPA3-transition AP OK")

    # creds JSON matches the characteristic contract.
    creds = json.loads(wifi_ap.wifi_creds_json(cfg))
    assert creds == {"ssid": "aurmor-pi-3f48", "password": "secret123",
                     "ip": "10.42.0.1", "port": 5005, "pi_id": 7,
                     "receiver_name": "aurmor-rpi-3f48"}, creds
    print("wifi creds JSON OK")
    return 0


def run_dynamic_node_tests() -> int:
    """Live default (no explicit map): any MAC-derived wid streams under its hex
    node (== the board's serial suffix), so same-position boards from different
    people never collide."""
    src = UdpImuSource(port=0, pi_id=7)  # no wid_to_node -> live/dynamic
    src.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(2.0)
    dest = ("127.0.0.1", src.port)
    try:
        # Two different boards that would BOTH be "chest" in the old kind-based
        # scheme now carry distinct MAC-derived wids -> distinct hex nodes.
        client.sendto(make_imu(0x5AF3, seq=1, t_ms=0), dest)
        client.sendto(make_imu(0x91B2, seq=1, t_ms=0), dest)
        assert wait_for(lambda: len(src._queue) >= 2), "MAC-wid samples never queued"
        nodes = {s["node"] for s in src.drain()}
        assert nodes == {"5AF3", "91B2"}, nodes
        active = src.active_wearables()
        assert active.get(0x5AF3) == "5AF3" and active.get(0x91B2) == "91B2", active
        assert protocol.presence_manufacturer_data(active.keys()) == \
            [2, 2, 0xF3, 0x5A, 0xB2, 0x91], "presence lists both MAC-wids (sorted)"
        print("dynamic MAC-wid nodes (no collision for same position) OK")
    finally:
        src.stop()
        client.close()
    return 0


def run_ble_sender_dynamic_meta_tests() -> int:
    """Live sender registers a per-board node the first time it streams and
    re-sends Meta, so two same-position boards decode as distinct nodes."""
    from ble_sender import BleSender, build_meta

    class FakeChar:
        def __init__(self):
            self.writes = []

        def set_value(self, v):
            self.writes.append(bytes(v))

    class FakeSource:
        def __init__(self, batches):
            self._batches = list(batches)

        def drain(self):
            return self._batches.pop(0) if self._batches else []

    def mk(node):
        s = {"node": node, "t_s": 0.0}
        for f in protocol.LIVE_IMU_FIELDS:
            s[f] = 0.5
        for f in protocol.LIVE_BIO_FIELDS:
            s[f] = 60.0
        return s

    field_specs, layouts, node_layout = protocol.build_live_protocol_meta([])
    meta0 = build_meta("live-udp", [], [], 180, field_specs, layouts, node_layout,
                       period_ms=100)
    # Two chest boards from different people -> distinct MAC-derived hex nodes.
    src = FakeSource([[mk("5AF3"), mk("91B2")]])
    sender = BleSender([], meta0, "x", None, 180, 1.0, False, False,
                       [], field_specs, layouts, node_layout,
                       source=src, live_period_ms=100)
    sender.data_char = FakeChar()
    sender.running = True
    sender._emit_live()

    assert sender.nodes == ["5AF3", "91B2"], sender.nodes
    assert sender.node_index == {"5AF3": 0, "91B2": 1}, sender.node_index

    stream = b"".join(sender.data_char.writes)
    records = list(iter_records(stream))
    metas = [body for mt, body in records if mt == protocol.MSG_META]
    samples = [body for mt, body in records if mt == protocol.MSG_SAMPLE]
    assert metas, "Meta must be (re)sent when a node registers"
    latest = json.loads(metas[-1].decode())
    assert latest["nodes"] == ["5AF3", "91B2"], latest["nodes"]
    assert [decode_sample(b, latest)["node"] for b in samples] == ["5AF3", "91B2"]
    print("ble_sender dynamic node registration + Meta re-send OK")
    return 0


def main() -> int:
    failures = run_source_tests()
    failures += run_alert_tests()
    failures += run_impact_requeue_tests()
    failures += run_dynamic_node_tests()
    failures += run_ble_sender_dynamic_meta_tests()
    failures += run_live_meta_tests()
    failures += run_wifi_ap_tests()
    if failures:
        print(f"FAILURES: {failures}")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
