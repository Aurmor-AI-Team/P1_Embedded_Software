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
from test_binary_protocol import _read_value, decode_sample, iter_records
import impact_store as impact_store_mod
import roster as roster_mod
import schedule as schedule_mod
import session as session_mod
from udp_source import (ALERT_ACK, ALERT_BODY, CONFIG, FORGET, HELLO, IMU_BODY,
                        IMU_BODY_V2, HDR, MSG_ALERT, MSG_ALERT_ACK, MSG_FORGET,
                        MSG_CONFIG, MSG_IMU, MSG_WELCOME, UdpImuSource, VERSION,
                        VERSION_V2, WELCOME)


def make_imu(wid: int, seq: int, t_ms: int, values=None) -> bytes:
    # 10 IMU floats + 4 bio floats (hr, spo2, resp, hrv).
    values = values or (0.1, -0.2, -1.0, 1.5, -2.5, 3.5, 0.01, 0.02, -0.99, 25.5,
                        88.0, 97.0, 18.0, 42.0)
    return HDR.pack(MSG_IMU, VERSION, wid) + IMU_BODY.pack(seq, t_ms, *values)


def make_imu_v2(wid: int, seq: int, t_ms: int, count=3, thr=20.0, accum=91.5,
                peak=41.2, temp=25.5, mode=6) -> bytes:
    """v2 telemetry: impact aggregates instead of raw axes."""
    return HDR.pack(MSG_IMU, VERSION_V2, wid) + IMU_BODY_V2.pack(
        seq, t_ms, count, thr, accum, peak, temp, 0.0, 0.0, 0.0, 0.0, mode)


def make_alert(wid: int, seq: int, t_ms: int, peak=41.2, thr=20.0,
               dur=120, mode=6, xport=2) -> bytes:
    return HDR.pack(MSG_ALERT, VERSION_V2, wid) + ALERT_BODY.pack(
        seq, t_ms, peak, thr, 0.5, -1.2, 40.9, 120.0, -80.0, 15.0,
        dur, mode, xport)


def wait_for(predicate, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def run_source_tests() -> int:
    failures = 0
    # The historical rig (one athlete, four sensors) is now just a roster.
    legacy = roster_mod.Roster.from_legacy_map(
        {1: "HEAD", 2: "WA", 3: "WD", 4: "WE"})
    src = UdpImuSource(port=0, pi_id=42, max_queue=8, roster=legacy)
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
    assert wait_for(lambda: len(src._latest) >= 1), "IMU sample never queued"
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
    assert wait_for(lambda: len(src._latest) >= 1)
    (sample,) = src.drain()
    assert sample["t_s"] == 0.255, sample["t_s"]
    print("IMU -> sample mapping + t_s baseline OK")

    # A second board (wid 2) maps to its own node — streams never collide.
    client.sendto(make_imu(2, seq=1, t_ms=500), dest)
    assert wait_for(lambda: len(src._latest) >= 1)
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
    assert wait_for(lambda: len(src._latest) >= 1)
    (sample,) = src.drain()
    assert sample["t_s"] > 0.255, f"t_s went backwards after reboot: {sample['t_s']}"
    print(f"reboot re-base OK (t_s={sample['t_s']})")

    # Backpressure: telemetry now coalesces per node rather than filling a
    # shared FIFO, so a burst leaves exactly one (newest) sample for the node.
    for i in range(20):
        client.sendto(make_imu(1, seq=100 + i, t_ms=200_000 + i * 10), dest)
    time.sleep(0.2)
    drained = src.drain()
    assert len(drained) == 1, f"expected 1 coalesced sample, got {len(drained)}"
    assert abs(drained[0]["t_s"] - src._last_t_s[1]) < 1e-6, drained
    print("per-node coalescing keeps the newest sample OK")

    # Active wearables: both boards heard recently, age out together, unmapped
    # wids never appear (presence source for the app's devices tab).
    assert src.active_wearables() == {1: "HEAD", 2: "WA"}, src.active_wearables()
    assert src.active_wearables(max_age_s=0.0) == {}
    assert 99 not in src.active_wearables(max_age_s=3600)
    print("active wearables tracking OK")

    # Presence advertisement payload: [version, bitmask]; the app prepends the
    # 2-byte company id and reads bit (wid-1). Mirror that decode here.
    assert protocol.presence_manufacturer_data_v1([]) == [1, 0]
    assert protocol.presence_manufacturer_data_v1([1]) == [1, 0b0000_0001]
    assert protocol.presence_manufacturer_data_v1([1, 3]) == [1, 0b0000_0101]
    assert protocol.presence_manufacturer_data_v1([9]) == [1, 0], "wid>8 invisible"
    # v2 replaces the bitmask with counts, which is what lets >8 devices exist.
    assert protocol.presence_manufacturer_data([1, 2], athletes_live=2,
                                               roster_revision=3) == [2, 2, 2, 3]
    print("presence manufacturer data (v1 bitmask + v2 counts) OK")

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
    field_specs, layouts, node_layout = protocol.build_live_protocol_meta(
        nodes, schema="raw")
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




def full_squad(n_athletes=30, per_athlete=6):
    positions = ["HEAD", "WA", "WD", "WE", "LSHIN", "RSHIN"][:per_athlete]
    return {"athletes": [
        {"id": f"a{i}", "name": f"Player {i}", "team": "Home",
         "devices": {str(i * per_athlete + d + 1): p
                     for d, p in enumerate(positions)}}
        for i in range(n_athletes)]}


def run_session_tests() -> int:
    """30 head sensors, one per athlete, one group session."""
    squad = roster_mod.Roster()
    for i in range(30):
        squad.add_athlete(f"Player {i:02d}", athlete_id=f"a{i:02d}",
                          team="Home", number=i + 1)
    assert len(squad.unassigned_athletes()) == 30
    assert squad.nodes() == [], "athletes with no sensor create no nodes"
    print("squad of 30 registered without any sensors OK")

    for i in range(29):
        squad.assign_head(i + 1, f"a{i:02d}")
    assert len(squad.head_wids()) == 29
    assert [a.name for a in squad.unassigned_athletes()] == ["Player 29"]
    print("check-in: 29 sensors handed out, 1 athlete flagged OK")

    # Re-handing a helmet to a different athlete moves it rather than erroring.
    squad.assign_head(1, "a29")
    assert squad.lookup(1)["athlete_id"] == "a29", squad.lookup(1)
    assert [a.name for a in squad.unassigned_athletes()] == ["Player 00"]
    squad.assign_head(1, "a00")
    print("re-handing a sensor moves it between athletes OK")

    # 29-30 devices must land comfortably inside the airtime budget.
    n = len(squad.assigned_wids())
    hz = protocol.telemetry_rate_hz(n)
    duty = protocol.channel_duty(n, hz)
    assert duty <= protocol.CHANNEL_DUTY_TARGET + 0.01, duty
    assert hz >= 20, f"{n} head sensors should support a useful rate, got {hz}"
    print(f"governor: {n} sensors -> {hz} Hz, {duty*100:.1f}% duty OK")

    store = impact_store_mod.ImpactStore(roster=squad)
    group = session_mod.GroupSession(squad, store)
    info = group.start(name="test-session", now=0.0)
    assert info["athletes"] == 30 and info["with_sensor"] == 29, info
    assert info["without_sensor"] == ["Player 29"], info
    print("session start reports the athlete with no sensor OK")

    class Src:
        now = 0.0
        def active_wearables(self, max_age_s=10.0):
            live = {w: f"a{w-1:02d}/HEAD" for w in range(1, 30)}
            live.pop(5, None)                            # never turned on
            if 600 <= self.now <= 900:                   # 5 min dropout
                live.pop(12, None)
            return live

    src = Src()
    for t in range(1, 1801):
        src.now = t
        group.tick(src, now=float(t))

    for wid, peak in ((1, 44.0), (12, 52.0)):
        info_w = squad.lookup(wid)
        store.record({"wid": wid, "seq": wid, "t_s": 5.0, "peak_g": peak,
                      "threshold_g": 20.0, "hx_g": 1.0, "hy_g": 2.0, "hz_g": 3.0,
                      "gx_dps": 10.0, "gy_dps": 20.0, "gz_dps": 30.0,
                      "dur_ms": 120, "mode": 6, "xport": 2,
                      **{k: v for k, v in info_w.items() if k in (
                          "athlete_id", "athlete", "team", "position",
                          "is_head", "node")}})

    rep = group.end(now=1800.0)
    rows = {r["athlete"]: r for r in rep["rows"]}

    # The property this whole module exists for: an impact count recorded
    # through a blind spot must NOT read as a trustworthy zero-or-one.
    dropped = rows["Player 11"]                # wid 12
    assert dropped["status"] == session_mod.STATUS_PARTIAL, dropped
    assert dropped["gap_count"] == 1 and dropped["longest_gap_s"] > 250, dropped
    assert dropped["head_impacts"] == 1
    assert not dropped["count_reliable"], "a blind spot must invalidate the count"

    never = rows["Player 04"]                  # wid 5
    assert never["status"] == session_mod.STATUS_NO_DATA, never
    assert never["head_impacts"] == 0 and not never["count_reliable"], \
        "0 impacts from a sensor that never connected must not read as 'no hits'"

    nosensor = rows["Player 29"]
    assert nosensor["status"] == session_mod.STATUS_UNASSIGNED, nosensor
    assert not nosensor["count_reliable"]

    good = rows["Player 00"]
    assert good["status"] == session_mod.STATUS_MONITORED and good["count_reliable"]
    assert good["head_impacts"] == 1
    print("coverage separates 'no impacts' from 'not monitoring' OK")

    assert rep["monitored"] == 27 and rep["partial"] == 1, rep
    assert rep["no_data"] == 1 and rep["unassigned"] == 1, rep
    assert rep["total_head_impacts"] == 2
    print(f"session report: {rep['monitored']} monitored, {rep['partial']} partial, "
          f"{rep['no_data']} no-data, {rep['unassigned']} unassigned OK")

    text = group.text_report()
    assert "count unreliable" in text and "Player 11" in text
    print("text report flags unreliable counts inline OK")
    store.close()
    return 0


def run_scale_tests() -> int:
    """30 athletes x 6 devices = 180 wearables, and the budgets that implies."""
    squad = roster_mod.Roster()
    squad.apply(full_squad())
    cap = roster_mod.capacity(squad)
    assert cap["athletes"] == 30 and cap["devices"] == 180, cap
    assert cap["head_sensors"] == 30, cap
    assert len(squad.nodes()) == 180
    assert len(set(squad.nodes())) == 180, "node ids must stay unique at scale"
    print("30 athletes x 6 devices = 180 unique nodes OK")

    for bad, why in (
        ({"athletes": [{"id": f"x{i}", "name": f"P{i}",
                        "devices": {str(i + 1): "HEAD"}} for i in range(31)]},
         "31 athletes"),
        ({"athletes": [{"id": "a", "name": "P", "devices": {
            str(w): p for w, p in zip(range(1, 8),
                                      ["HEAD", "WA", "WD", "WE", "LSHIN",
                                       "RSHIN", "BACK"])}}]},
         "7 devices on one athlete"),
    ):
        try:
            roster_mod.Roster().apply(bad)
        except roster_mod.RosterError:
            pass
        else:
            print(f"FAIL: accepted over-capacity roster ({why})")
            return 1
    print("capacity limits enforced OK")

    # The airtime governor must land the whole squad inside the budget.
    for n, floor in ((1, 100), (30, 60), (180, 10)):
        hz = protocol.telemetry_rate_hz(n)
        assert n * hz <= protocol.UDP_PACKET_BUDGET_PPS, (n, hz)
        assert hz >= protocol.TELEMETRY_MIN_HZ
        print(f"  governor: {n:3} device(s) -> {hz:3} Hz "
              f"(~{n * hz} pps of {protocol.UDP_PACKET_BUDGET_PPS})")
    assert protocol.telemetry_rate_hz(180) < protocol.telemetry_rate_hz(30)
    print("airtime governor keeps the squad inside budget OK")

    # Presence must survive >8 wearables. v1's bitmask silently lost them.
    v2 = protocol.presence_manufacturer_data(range(1, 181), athletes_live=30,
                                             roster_revision=7)
    assert v2 == [2, 180, 30, 7], v2
    v1 = protocol.presence_manufacturer_data_v1(range(1, 181))
    assert v1 == [1, 0xFF], v1     # wid 9..180 are simply invisible to v1
    print("presence v2 reports 180 devices / 30 athletes (v1 could show 8) OK")

    store = impact_store_mod.ImpactStore(roster=squad)
    src = UdpImuSource(port=0, pi_id=42, roster=squad, impact_store=store)
    src.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.bind(("127.0.0.1", 0))
    client.settimeout(2.0)
    dest = ("127.0.0.1", src.port)

    # FAIRNESS: this is why the shared FIFO had to go. One device sending 200
    # packets must not evict the quiet ones — with a bounded shared deque the
    # loud device's backlog is exactly what pushed everyone else out.
    for i in range(200):
        client.sendto(make_imu_v2(1, seq=i, t_ms=1000 + i * 5), dest)
    for wid in range(2, 21):
        client.sendto(make_imu_v2(wid, seq=1, t_ms=1000), dest)
    assert wait_for(lambda: len(src.drain_impacts()) == 0 and len(src._latest) >= 20,
                    timeout_s=3.0), f"only {len(src._latest)} nodes represented"
    samples = src.drain()
    nodes = {s["node"] for s in samples}
    assert len(nodes) >= 20, f"quiet devices starved: {len(nodes)} nodes"
    assert len(samples) == len(nodes), "coalescing must yield one sample per node"
    print(f"fairness: 1 loud + 19 quiet devices -> all {len(nodes)} represented OK")

    # FOCUS: only the displayed nodes get full-rate records, capped.
    chosen = src.set_focus([squad.lookup(w)["node"] for w in range(1, 9)])
    assert len(chosen) == protocol.FOCUS_MAX_NODES, chosen
    for i in range(30):
        client.sendto(make_imu_v2(1, seq=500 + i, t_ms=9000 + i * 10), dest)
    assert wait_for(lambda: len(src._focus_q) >= 10, timeout_s=3.0)
    focus_nodes = {s["node"] for s in src.drain_focus()}
    assert focus_nodes <= set(chosen), focus_nodes
    src.drain()
    print(f"focus capped at {protocol.FOCUS_MAX_NODES} nodes, full rate OK")

    # Batched squad summary: one record for 180 nodes, not 180 records.
    entries = [{"node_idx": i, "flags": protocol.SUMMARY_FLAG_HEAD | protocol.SUMMARY_FLAG_LIVE,
                "head_impacts": i % 7, "body_impacts": 0, "peak_g": 20.0 + i * 0.1,
                "age_ms": 120, "rate_hz": 10, "mode": 6} for i in range(180)]
    payload = protocol.encode_summary_binary(entries)
    record = protocol.frame_record(protocol.MSG_SUMMARY, payload)
    chunks = list(protocol.chunk_bytes(record, protocol.DEFAULT_CHUNK_SIZE))
    (mt, body), = list(iter_records(b"".join(chunks)))
    assert mt == protocol.MSG_SUMMARY
    (count,) = struct.unpack_from("<H", body, 0)
    assert count == 180, count
    last = protocol.SUMMARY_ENTRY.unpack_from(body, 2 + 179 * protocol.SUMMARY_ENTRY.size)
    assert last[0] == 179 and abs(last[4] / 100 - (20.0 + 179 * 0.1)) < 0.01, last
    print(f"squad summary: 180 nodes in 1 record / {len(chunks)} notifications "
          f"(vs 180 records) OK")

    # Liveness must not lie. `_last_seen.get(wid, 0.0) > cutoff` reported every
    # rostered wearable as LIVE for the first 10 s of uptime, because monotonic()
    # starts near zero and 0.0 beats a negative cutoff.
    quiet = UdpImuSource(port=0, pi_id=1, roster=squad)
    assert quiet.active_wearables() == {}, quiet.active_wearables()
    quiet._touch(1, ("10.0.0.9", 5006))
    assert quiet.active_wearables() == {1: squad.lookup(1)["node"]}
    print("silent wearables are not reported live OK")

    # Slot plan: every device gets a non-overlapping window wide enough to
    # actually transmit in, and athletes sub-slot inside their own window.
    plan = schedule_mod.slot_plan(squad, hz=10)
    assert len(plan) == 180, len(plan)
    offsets = sorted(p["offset_us"] for p in plan.values())
    assert len(set(offsets)) == 180, "slot offsets must be unique"
    gap = min(b - a for a, b in zip(offsets, offsets[1:]))
    airtime = schedule_mod.airtime_us(49, contended=False)
    assert gap > airtime, f"slot gap {gap} us cannot fit a {airtime:.0f} us frame"
    # Sensors on one athlete share that athlete's window.
    a0 = {w: plan[w]["offset_us"] for w in range(1, 7)}
    a1 = {w: plan[w]["offset_us"] for w in range(7, 13)}
    assert max(a0.values()) < min(a1.values()), "athlete windows must not overlap"
    print(f"slot plan: 180 devices, {gap} us apart, "
          f"{airtime:.0f} us of airtime each OK")

    # Scheduling must not change the airtime budget verdict, only improve duty.
    flat_duty = schedule_mod.flat(30, 6, 10, scheduled=False)["duty"]
    sched_duty = schedule_mod.flat(30, 6, 10, scheduled=True)["duty"]
    assert sched_duty < flat_duty * 0.7, (flat_duty, sched_duty)
    print(f"scheduling cuts channel duty {flat_duty*100:.1f}% -> "
          f"{sched_duty*100:.1f}% (backoff removed) OK")

    # Config downlink: the governor's rate AND slot reach the wearable.
    client.sendto(HELLO.pack(udp_source.MSG_HELLO, VERSION_V2, 1, 1), dest)
    client.recvfrom(64)
    assert src.send_config(1, policy=0, threshold_g=25.0, rate_hz=10,
                           slot_us=3333, frame_us=100000)
    data, _ = client.recvfrom(64)
    mt, ver, wid, policy, flags, rate_hz, thr, slot, frame = CONFIG.unpack(data)
    assert (mt, ver, wid, policy, rate_hz) == (MSG_CONFIG, VERSION_V2, 1, 0, 10), \
        (mt, ver, wid, policy, rate_hz)
    assert flags == (udp_source.CONFIG_FLAG_THRESHOLD | udp_source.CONFIG_FLAG_RATE
                     | udp_source.CONFIG_FLAG_SLOT), flags
    assert abs(thr - 25.0) < 1e-6
    assert (slot, frame) == (3333, 100000), (slot, frame)
    print("MSG_CONFIG downlink (policy + threshold + rate + slot) OK")

    src.stop()
    client.close()
    store.close()
    return 0


def run_roster_tests() -> int:
    """The roster model: multi-sensor athletes, validation, stable indices."""
    r = roster_mod.Roster()
    assert r.is_empty(), "a fresh receiver must monitor nobody"
    assert r.nodes() == [] and r.lookup(1) is None
    print("empty by default OK")

    r.apply({"athletes": [
        {"id": "a1", "name": "A. Rivera", "team": "Home", "number": 7,
         "devices": {"1": "HEAD", "5": "WA", "6": "WD"}},
        {"id": "a2", "name": "K. Osei", "team": "Home",
         "devices": {"2": "HEAD"}},
    ]})
    assert r.head_wids() == [1, 2], r.head_wids()
    assert sorted(r.assigned_wids()) == [1, 2, 5, 6]
    one = r.lookup(1)
    assert one["athlete"] == "A. Rivera" and one["is_head"], one
    five = r.lookup(5)
    assert five["athlete_id"] == "a1" and not five["is_head"], five
    assert one["node"] != five["node"], "one athlete's sensors need distinct nodes"
    print("multi-sensor athlete OK (3 devices, 1 head)")

    # Two athletes both wearing HEAD must not collapse onto one node — that is
    # how you attribute a concussion to the wrong person.
    assert r.lookup(1)["node"] != r.lookup(2)["node"]
    print("head nodes distinct across athletes OK")

    # Stable indices: adding an athlete must never renumber existing nodes.
    before = {n: i for i, n in enumerate(r.nodes())}
    r.apply({"athletes": [
        {"id": "a1", "name": "A. Rivera", "devices": {"1": "HEAD", "5": "WA", "6": "WD"}},
        {"id": "a2", "name": "K. Osei", "devices": {"2": "HEAD"}},
        {"id": "a3", "name": "M. Lund", "devices": {"3": "HEAD"}},
    ]})
    after = {n: i for i, n in enumerate(r.nodes())}
    assert all(after[n] == i for n, i in before.items()), (before, after)
    print("append-only node indices OK (mid-session edits stay decodable)")

    for bad, why in (
        ({"athletes": [{"name": "X", "devices": {"1": "HEAD"}},
                       {"name": "Y", "devices": {"1": "HEAD"}}]},
         "one wearable on two athletes"),
        ({"athletes": [{"name": "X", "devices": {"1": "HEAD", "2": "HEAD"}}]},
         "two devices on one position"),
        ({"athletes": [{"name": "", "devices": {}}]}, "athlete with no name"),
        ({"athletes": [{"name": "X", "devices": {"1": ""}}]}, "device with no position"),
        ({"athletes": "nope"}, "athletes not a list"),
    ):
        try:
            roster_mod.Roster().apply(bad)
        except roster_mod.RosterError:
            pass
        else:
            print(f"FAIL: accepted invalid roster ({why})")
            return 1
    print("roster validation rejects bad edits OK")

    # Persistence round-trip, including index stability across a restart.
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "roster.json"
        a = roster_mod.Roster(path)
        a.apply({"athletes": [{"id": "a1", "name": "A. Rivera",
                               "devices": {"1": "HEAD", "5": "WA"}}]})
        nodes_before = a.nodes()
        b = roster_mod.Roster(path).load()
        assert b.nodes() == nodes_before, (b.nodes(), nodes_before)
        assert b.lookup(5)["position"] == "WA"
        print("roster persists across restart with stable indices OK")
    return 0


def run_impact_tests() -> int:
    """Impact path: discovery, attribution, head-vs-body, dedupe, hot-swap."""
    squad = roster_mod.Roster()
    store = impact_store_mod.ImpactStore(roster=squad)
    src = UdpImuSource(port=0, pi_id=42, max_queue=8, roster=squad,
                       impact_store=store)
    src.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.bind(("127.0.0.1", 0))
    client.settimeout(2.0)
    dest = ("127.0.0.1", src.port)

    # --- with an EMPTY roster, wearables must still be discoverable -------- #
    client.sendto(HELLO.pack(udp_source.MSG_HELLO, VERSION_V2, 4, 0xABCD), dest)
    client.recvfrom(64)                      # unassigned boards are still welcomed
    client.sendto(make_imu_v2(4, seq=1, t_ms=1000, peak=12.0), dest)
    assert wait_for(lambda: src.unassigned_wearables()), "no discovery list"
    (found,) = src.unassigned_wearables()
    assert found["wid"] == 4, found
    assert src.drain() == [], "unassigned telemetry must not be streamed"
    print("empty roster: wearable discoverable but not streamed OK")

    # An impact from an UNASSIGNED board is still recorded — losing a real head
    # impact because setup was incomplete is not an acceptable failure.
    client.sendto(make_alert(4, seq=1, t_ms=1500, peak=55.0), dest)
    client.recvfrom(64)
    assert wait_for(lambda: len(src._impacts) >= 1), "unassigned impact dropped"
    (event,) = src.drain_impacts()
    assert event["athlete_id"] is None and event["wid"] == 4, event
    assert event["is_head"] is False, "unknown position must not count as head"
    print("unassigned impact recorded, not counted as head OK")

    # --- the app assigns devices at runtime -------------------------------- #
    squad.apply({"athletes": [
        {"id": "a1", "name": "A. Rivera", "team": "Home",
         "devices": {"1": "HEAD", "5": "WD"}},
        {"id": "a2", "name": "K. Osei", "team": "Home", "devices": {"2": "HEAD"}},
    ]})
    src.set_roster(squad)
    store.set_roster(squad)
    assert src.nodes == squad.nodes()
    print("roster hot-swap OK")

    # Telemetry now flows, tagged with the right athlete + position.
    client.sendto(make_imu_v2(5, seq=2, t_ms=2000), dest)
    assert wait_for(lambda: len(src._latest) >= 1), "assigned telemetry never queued"
    (sample,) = src.drain()
    assert sample["node"] == squad.lookup(5)["node"], sample
    assert list(sample.keys()) == (
        ["node", "t_s"] + protocol.LIVE_AGG_FIELDS + protocol.LIVE_BIO_FIELDS)
    print("assigned telemetry routed to the athlete's node OK")

    # --- head vs body on the SAME athlete ---------------------------------- #
    client.sendto(make_alert(1, seq=11, t_ms=3000, peak=45.0), dest)   # head
    data, _ = client.recvfrom(64)
    mt, ver, wid, seq = ALERT_ACK.unpack(data)
    assert (mt, ver, wid, seq) == (MSG_ALERT_ACK, VERSION_V2, 1, 11)
    client.sendto(make_alert(5, seq=12, t_ms=3100, peak=61.0), dest)   # wrist
    client.recvfrom(64)
    assert wait_for(lambda: len(src._impacts) >= 2)
    head, wrist = src.drain_impacts()
    assert head["is_head"] and head["position"] == "HEAD", head
    assert not wrist["is_head"] and wrist["position"] == "WD", wrist
    assert head["athlete_id"] == wrist["athlete_id"] == "a1"

    rivera = next(a for a in store.athletes() if a["athlete_id"] == "a1")
    assert rivera["head_impacts"] == 1, rivera
    assert rivera["body_impacts"] == 1, rivera
    assert abs(rivera["head_peak_g"] - 45.0) < 1e-3, rivera
    assert len(rivera["devices"]) == 2, rivera["devices"]
    print("head and body impacts counted separately on one athlete OK")

    # A harder wrist hit must NOT raise the head peak.
    assert rivera["head_peak_g"] < 61.0, "wrist impact leaked into head totals"
    print("wrist impact excluded from head totals OK")

    # --- retransmit dedupe -------------------------------------------------- #
    for _ in range(3):
        client.sendto(make_alert(1, seq=11, t_ms=3000, peak=45.0), dest)
        client.recvfrom(64)          # every retransmit is still acked
    time.sleep(0.15)
    assert src.drain_impacts() == [], "retransmit must not duplicate"
    rivera = next(a for a in store.athletes() if a["athlete_id"] == "a1")
    assert rivera["head_impacts"] == 1, rivera
    print("retransmit dedupe OK (acked every time, recorded once)")

    # --- rostered-but-quiet athletes still appear --------------------------- #
    ids = {a["athlete_id"] for a in store.athletes()}
    assert "a2" in ids, "a rostered athlete with no impacts must still be listed"
    osei = next(a for a in store.athletes() if a["athlete_id"] == "a2")
    assert osei["head_impacts"] == 0 and len(osei["devices"]) == 1
    print("rostered-but-quiet athlete listed OK")

    summary = store.summary(pi_id=42)
    assert summary["head_impacts"] == 1, summary
    assert summary["body_impacts"] == 2, summary   # wrist + the unassigned one
    assert summary["duplicates_suppressed"] == 3, summary
    print("squad summary OK")

    # --- MSG_IMPACT round-trip through the app's decoder -------------------- #
    payload = protocol.encode_impact_binary(head)
    record = protocol.frame_record(protocol.MSG_IMPACT, payload)
    (mt, body), = list(iter_records(b"".join(protocol.chunk_bytes(record, 11))))
    assert mt == protocol.MSG_IMPACT
    dec = decode_impact(body)
    assert dec["athlete_id"] == "a1" and dec["athlete"] == "A. Rivera", dec
    assert dec["position"] == "HEAD" and dec["is_head"] and not dec["unattributed"]
    assert abs(dec["peak_g"] - 45.0) <= 0.01, dec["peak_g"]
    print("MSG_IMPACT round-trip carries athlete + position OK")

    unattr = decode_impact(protocol.encode_impact_binary(event))
    assert unattr["unattributed"] and not unattr["is_head"], unattr
    print("unattributed impact flagged for the app OK")

    # Unknown length must be reported, not silently swallowed.
    client.sendto(HDR.pack(MSG_IMU, VERSION_V2, 1) + b"\x00" * 20, dest)
    time.sleep(0.15)
    assert src._bad_len, "short IMU packet must be counted, not ignored"
    print("bad-length packets counted OK")

    src.stop()
    client.close()
    store.close()
    return 0


def decode_impact(payload: bytes) -> dict:
    """Reference decoder for MSG_IMPACT — mirrors what the app must implement."""
    view = memoryview(payload)
    # wid is u16: 180 devices fit in a byte, but the wire header already carries
    # wid as u16 and widening a shipped field later is far worse than now.
    out = {"wid": struct.unpack_from("<H", view, 0)[0]}
    out["seq"] = struct.unpack_from("<I", view, 2)[0]
    out["t_s"] = struct.unpack_from("<I", view, 6)[0] / 1000
    out["epoch_ms"] = struct.unpack_from("<Q", view, 10)[0]
    out["severity"], out["mode"] = view[18], view[19]
    out["xport"], flags = view[20], view[21]
    out["is_head"] = bool(flags & protocol.IMPACT_FLAG_HEAD)
    out["unattributed"] = bool(flags & protocol.IMPACT_FLAG_UNATTRIBUTED)
    off = 22
    for field in protocol.IMPACT_FIELDS:
        typ, scale = protocol.IMPACT_SPECS[field]
        value, off = _read_value(view, off, typ, scale)
        out[field] = value
    for name in ("athlete_id", "athlete", "position"):
        out[name], off = _read_value(view, off, "str", 0)
    return out


def main() -> int:
    failures = run_source_tests()
    failures += run_live_meta_tests()
    failures += run_session_tests()
    failures += run_scale_tests()
    failures += run_roster_tests()
    failures += run_impact_tests()
    failures += run_wifi_ap_tests()
    if failures:
        print(f"FAILURES: {failures}")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())