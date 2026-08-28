"""Loopback tests for the emulated ESP32 fleet (mock_receiver.py).

Runs the REAL UdpImuSource against real MockBoard threads over 127.0.0.1, so
these cover the actual UDP round trip: HELLO/WELCOME, mode relay, the impact
ack, and FORGET. No BLE, no bluezero, no hardware.

Run: ``python3 test_mock_receiver.py`` (stdlib only; exit code 0 on success).
Style matches test_udp_source.py: plain asserts, one runner per area.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import ble_sender
import mock_receiver
import protocol
import udp_source
from mock_receiver import MOCK_COUNT, MOCK_WID_BASE, MockFleet
from udp_source import MODES, UdpImuSource


def wait_for(predicate, timeout_s: float = 4.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def start_fleet(count=3, mode="idle", period_ms=60, impact_every=0.0):
    """A source plus a small fleet pointed at it, both already running."""
    source = UdpImuSource(port=0, pi_id=77, max_queue=4096)
    source.start()
    fleet = MockFleet(count, MOCK_WID_BASE, ("127.0.0.1", source.port),
                      mode=mode, period_ms=period_ms,
                      impact_every=impact_every, rows=None)
    fleet.start()
    return source, fleet


def teardown(source, fleet) -> None:
    fleet.stop()
    source.stop()


def samples_of(records, node=None):
    out = [r for r in records if r.get("type") != "impact"]
    return [r for r in out if node is None or r["node"] == node] if node else out


def impacts_of(records):
    return [r for r in records if r.get("type") == "impact"]


# --------------------------------------------------------------------------- #
def run_idle_presence_tests() -> int:
    """Idle boards send no telemetry, but must still be PRESENT: the app shows
    them as detected from active_wearables(), which HELLO alone sustains."""
    failures = 0
    source, fleet = start_fleet(count=3, mode="idle")
    try:
        assert wait_for(lambda: len(source.active_wearables()) == 3), \
            f"idle boards must check in: {source.active_wearables()}"
        active = source.active_wearables()
        assert sorted(active) == fleet.wids, (sorted(active), fleet.wids)
        assert sorted(active.values()) == ["E001", "E002", "E003"], active
        assert wait_for(lambda: source.modes() ==
                        {w: "idle" for w in fleet.wids}), source.modes()

        source.drain()          # clear anything queued during start-up
        time.sleep(0.5)
        assert source.drain() == [], "an idle board must put nothing on the wire"
        print("mock fleet idle presence OK")
    except AssertionError as exc:
        print(f"FAIL idle presence: {exc}")
        failures += 1
    finally:
        teardown(source, fleet)
    return failures


def run_mode_relay_tests() -> int:
    """send_mode reaches exactly the board it names, and only that board starts
    streaming — the same per-wid addressing the app relies on."""
    failures = 0
    source, fleet = start_fleet(count=3, mode="idle")
    try:
        assert wait_for(lambda: len(source.active_wearables()) == 3)
        target = fleet.wids[1]
        assert source.send_mode(target, "live", retries=2, interval_s=0.05)
        assert wait_for(lambda: source.modes().get(target) == "live"), source.modes()

        source.drain()
        time.sleep(0.5)
        records = source.drain()
        nodes = {r["node"] for r in samples_of(records)}
        assert nodes == {"E002"}, f"only the named board may stream: {nodes}"

        # And back to idle: it must fall silent again.
        assert source.send_mode(target, "idle", retries=2, interval_s=0.05)
        assert wait_for(lambda: source.modes().get(target) == "idle")
        source.drain()
        time.sleep(0.4)
        assert samples_of(source.drain()) == [], "idle must stop the stream"
        print("mock fleet mode relay OK")
    except AssertionError as exc:
        print(f"FAIL mode relay: {exc}")
        failures += 1
    finally:
        teardown(source, fleet)
    return failures


def run_impact_tests() -> int:
    """Impacts reach the receiver, get acked (so the board stops retransmitting),
    and are queued exactly once."""
    failures = 0
    source, fleet = start_fleet(count=2, mode="live", impact_every=0.5)
    try:
        assert wait_for(lambda: len(source.active_wearables()) == 2)
        collected = []
        assert wait_for(lambda: (collected.extend(source.drain())
                                 or len(impacts_of(collected)) >= 2), 8.0), \
            f"expected impacts, got {len(impacts_of(collected))}"
        impacts = impacts_of(collected)
        for impact in impacts:
            assert impact["peak_g"] > impact["threshold_g"], impact
            assert impact["node"] in ("E001", "E002"), impact
        # Each (node, seq) exactly once: the board retransmits until acked, so a
        # duplicate here means the receiver's dedupe is not working.
        keys = [(i["node"], i["seq"]) for i in impacts]
        assert len(keys) == len(set(keys)), f"duplicate impacts: {keys}"
        # The ack got back: nothing is still awaiting one.
        assert wait_for(lambda: all(not b._unacked for b in fleet.boards)), \
            [b._unacked for b in fleet.boards]
        print("mock fleet impacts + ack/dedupe OK")
    except AssertionError as exc:
        print(f"FAIL impacts: {exc}")
        failures += 1
    finally:
        teardown(source, fleet)
    return failures


def run_held_impact_tests() -> int:
    """An idle board still DETECTS impacts and holds them, so switching out of
    idle replays what happened while it was quiet (mirrors the firmware)."""
    failures = 0
    source, fleet = start_fleet(count=1, mode="idle", impact_every=0.4)
    try:
        wid = fleet.wids[0]
        assert wait_for(lambda: len(source.active_wearables()) == 1)
        assert wait_for(lambda: len(fleet.boards[0]._held) >= 2, 4.0), \
            "idle must hold impacts, not drop them"
        source.drain()
        assert impacts_of(source.drain()) == [], "idle must transmit nothing"

        held = len(fleet.boards[0]._held)
        assert source.send_mode(wid, "alerts", retries=2, interval_s=0.05)
        collected = []
        assert wait_for(lambda: (collected.extend(source.drain())
                                 or len(impacts_of(collected)) >= held), 4.0), \
            f"held impacts must be replayed: {len(impacts_of(collected))} < {held}"
        # alerts mode is impacts WITHOUT telemetry.
        assert samples_of(collected) == [], "alerts mode must not stream samples"
        print("mock fleet held-impact replay + alerts mode OK")
    except AssertionError as exc:
        print(f"FAIL held impacts: {exc}")
        failures += 1
    finally:
        teardown(source, fleet)
    return failures


def run_forget_tests() -> int:
    """FORGET makes a board leave the network — it stops streaming and ages out
    of active_wearables(), which is what the app's release-to-Bluetooth watches."""
    failures = 0
    source, fleet = start_fleet(count=2, mode="live")
    try:
        assert wait_for(lambda: len(source.active_wearables()) == 2)
        target = fleet.wids[0]
        assert source.send_forget(target, retries=2, interval_s=0.05)
        assert wait_for(lambda: fleet.boards[0].forgotten), "board ignored FORGET"
        assert wait_for(lambda: not fleet.boards[0].is_alive())

        source.drain()
        time.sleep(0.4)
        nodes = {r["node"] for r in samples_of(source.drain())}
        assert "E001" not in nodes, f"a forgotten board must go quiet: {nodes}"
        assert "E002" in nodes, f"the rest of the fleet must keep going: {nodes}"
        # Presence ages out on the receiver's own 10 s window; check the rule
        # rather than waiting it out.
        assert target not in source.active_wearables(max_age_s=0.2)
        print("mock fleet FORGET OK")
    except AssertionError as exc:
        print(f"FAIL forget: {exc}")
        failures += 1
    finally:
        teardown(source, fleet)
    return failures


def run_contract_tests() -> int:
    """The reserved wid block must stay decodable by the app, which derives each
    board's wid from its serial suffix (mock-devices.ts / protocol.ts)."""
    failures = 0
    try:
        assert MOCK_COUNT == 10, "must match MOCK_ESP32_COUNT in mock-devices.ts"
        # Deliberately NOT tied to PRESENCE_MAX_WIDS. The fleet does not fit one
        # presence advertisement and does not need to: mock_receiver turns the
        # presence broadcast off, because the app seeds these wids itself. An
        # earlier version of this assertion claimed the opposite and was how the
        # oversized-advertisement bug slipped through.
        assert MOCK_COUNT > protocol.PRESENCE_MAX_WIDS
        nodes = [udp_source.node_for_wid(MOCK_WID_BASE + i) for i in range(MOCK_COUNT)]
        assert nodes[0] == "E001" and nodes[-1] == "E00A", nodes
        assert len(set(nodes)) == MOCK_COUNT
        for node in nodes:
            assert len(node) == 4 and all(c in "0123456789ABCDEF" for c in node), node
        # Every wid fits the u16 the wire header uses.
        assert MOCK_WID_BASE + MOCK_COUNT - 1 <= 0xFFFF
        print("mock wid-block contract OK")
    except AssertionError as exc:
        print(f"FAIL contract: {exc}")
        failures += 1
    return failures


def run_presence_off_tests() -> int:
    """mock_receiver must NOT broadcast presence.

    Ten live wids overflow the 31-byte advertisement, BlueZ then refuses to
    register it, and the receiver becomes invisible to every scan in the app —
    so there is no receiver to pair and no group session to start. This is the
    regression test for exactly that.
    """
    failures = 0
    source, fleet = start_fleet(count=MOCK_COUNT, mode="idle")
    try:
        assert wait_for(lambda: len(source.active_wearables()) == MOCK_COUNT, 8.0), \
            source.active_wearables()

        # BleSender.__init__ imports no BlueZ, so the real thing can be built
        # here and asked what it would advertise for this fleet.
        sender = ble_sender.BleSender(
            [], b"{}", "aurmor-rpi-test", None, 180, 1.0, False, False,
            fleet.nodes, {}, [[]], [0] * MOCK_COUNT, source=source)
        # A real receiver still broadcasts presence; only mock mode opts out.
        assert sender.presence_adv_enabled is True
        # Every rotation offset must stay inside the 31-byte advertisement.
        for offset in range(MOCK_COUNT * 2):
            sender._presence_offset = offset
            size = protocol._ADV_FIXED_BYTES - 2 + len(sender._presence_data())
            assert size <= protocol.ADV_PAYLOAD_BYTES, \
                f"offset {offset} -> {size} B advertisement (over 31)"

        # The guarantee that fixes the reported bug: mock_receiver turns the
        # broadcast off outright, so nothing is spent on it at all.
        src = (Path(__file__).resolve().parent / "mock_receiver.py").read_text()
        assert "sender.presence_adv_enabled = False" in src, \
            "mock_receiver must disable the presence advertisement"
        print("mock receiver presence-adv disabled OK")
    except AssertionError as exc:
        print(f"FAIL presence off: {exc}")
        failures += 1
    finally:
        teardown(source, fleet)
    return failures


def run_csv_mode_tests() -> int:
    """`mock` mode replays the HEAD CSV, offset per board so the fleet is not in
    lockstep. Skipped when the mock-csv folder isn't present."""
    rows = mock_receiver.load_head_mock_rows(mock_receiver.DEFAULT_CSV)
    if not rows:
        print("mock CSV replay SKIPPED (mock-csv not present)")
        return 0
    failures = 0
    source = UdpImuSource(port=0, pi_id=77, max_queue=4096)
    source.start()
    fleet = MockFleet(2, MOCK_WID_BASE, ("127.0.0.1", source.port),
                      mode="mock", period_ms=40, impact_every=0.0, rows=rows)
    fleet.start()
    try:
        collected = []
        assert wait_for(lambda: (collected.extend(source.drain())
                                 or len({r["node"] for r in samples_of(collected)}) == 2))
        assert fleet.boards[0].row_offset != fleet.boards[1].row_offset, \
            "boards must not replay the CSV in lockstep"
        first = {r["node"]: r for r in samples_of(collected)}
        assert first["E001"]["ax_g"] != first["E002"]["ax_g"], first
        print("mock CSV replay OK")
    except AssertionError as exc:
        print(f"FAIL csv mode: {exc}")
        failures += 1
    finally:
        teardown(source, fleet)
    return failures


def main() -> int:
    failures = run_contract_tests()
    failures += run_presence_off_tests()
    failures += run_idle_presence_tests()
    failures += run_mode_relay_tests()
    failures += run_impact_tests()
    failures += run_held_impact_tests()
    failures += run_forget_tests()
    failures += run_csv_mode_tests()
    if failures:
        print(f"FAILURES: {failures}")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
