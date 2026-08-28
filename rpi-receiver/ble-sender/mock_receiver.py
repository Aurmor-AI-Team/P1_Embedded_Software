#!/usr/bin/env python3
"""Receiver with a fleet of EMULATED ESP32 wearables — run INSTEAD of ble_sender.py.

Testing a group session needs ten wearables. This stands ten of them up in
software so the whole group path — device picker, per-participant assignment,
working-mode control, impacts, per-node de-multiplexing, session teardown — can
be exercised with nothing but a Raspberry Pi and a phone.

Nothing on the data path is faked. The real ``UdpImuSource`` and the real
``BleSender`` run unmodified; the emulated boards genuinely send IMU/HELLO/ALERT
packets over the loopback and genuinely answer WELCOME/MODE/FORGET. The only
fiction is that ten boards exist::

    mock_receiver.py
      ├─ wifi_ap.load_config()        -> this Pi's real receiver_name / pi_id
      ├─ UdpImuSource(port, pi_id)    -> unmodified receiver code
      ├─ BleSender(source=that, …)    -> unmodified GATT server
      └─ MockFleet: N × MockBoard thread
            HELLO + MODE_RPT every 2 s  ──▶ 127.0.0.1:<udp_port>
            IMU 68 B every --period-ms  ──▶
            ALERT + 600 ms retransmit   ──▶
                                        ◀── WELCOME / MODE / FORGET / ALERT_ACK

The boards occupy a RESERVED wearable-id block, 0xE001-0xE00A, which the app
mirrors in ``features/devices/mock-devices.ts`` as the serials
``aurmor-mibs-E001`` … ``aurmor-mibs-E00A``. Both sides must agree on that block
and nothing else: every other part of the contract is the existing one.

  On the Pi:   sudo systemctl stop aurmor-receiver
               sudo systemctl start aurmor-receiver-mock
  By hand:     python3 mock_receiver.py
  Dry run:     python3 mock_receiver.py --stdout --mode live --count 3 --verbose

This and ble_sender.py are mutually exclusive — both claim the Bluetooth adapter
and the UDP port.
"""
from __future__ import annotations

import argparse
import csv
import math
import random
import socket
import sys
import threading
import time
from pathlib import Path

import ble_sender
import protocol
import udp_source
import wifi_ap
from udp_source import (ALERT, ALERT_ACK, FORGET, HDR, HELLO, IMU_BODY, MODE,
                        MODE_NAMES, MODE_RPT, MODES, MSG_ALERT, MSG_ALERT_ACK,
                        MSG_FORGET, MSG_HELLO, MSG_IMU, MSG_MODE, MSG_MODE_RPT,
                        MSG_WELCOME, VERSION, WELCOME, UdpImuSource)

# --------------------------------------------------------------------------- #
# The reserved mock wearable-id block.
#
# Real wids are MAC-derived (the last two bytes of the board's MAC, which is
# also its serial suffix), so no value is formally reserved. This block is
# picked to be recognisable in a log and is documented on both sides; it only
# matters that the app's mock-devices.ts uses the SAME base and count, or the
# phone will attribute samples to devices that never appear.
# --------------------------------------------------------------------------- #
MOCK_WID_BASE = 0xE001
MOCK_COUNT = 10                       # == MOCK_ESP32_COUNT in mock-devices.ts

# Emulated-board timings, mirroring wifi_udp_tx.cpp.
HELLO_INTERVAL_S = 2.0
ALERT_RETRY_S = 0.6
ALERT_MAX_TRIES = 6
# Loop granularity. Deliberately shorter than the sample period so a MODE or
# FORGET is picked up promptly rather than a whole frame later.
TICK_S = 0.05

# CSV replay source for `mock` mode: the same merge the firmware's embedded mock
# does — HEAD IMU, chest ECG (hr/resp/hrv) and wrist PPG (spo2). Note this file
# lives in ble-sender/, one level shallower than tools/fake_esp32_sender.py.
MOCK_DIR = Path(__file__).resolve().parent.parent / "mock-csv" / \
    "10_squats_clean_biometric_data_simulation"
DEFAULT_CSV = MOCK_DIR / "HEAD_Head_main.csv"
IMU_FIELDS = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
              "hx_g", "hy_g", "hz_g", "imu_temp_c")

IMPACT_THRESHOLD_G = 20.0


def _column(path: Path, name: str):
    if not path.exists():
        return None
    with path.open(newline="") as handle:
        return [float(row[name]) for row in csv.DictReader(handle)]


def load_head_mock_rows(path: Path):
    """14-value rows (10 IMU + hr/spo2/resp/hrv) for `mock` mode, or None."""
    if not path.exists():
        print(f"# {path} not found — 'mock' mode will use synthetic data",
              file=sys.stderr)
        return None
    with path.open(newline="") as handle:
        imu = [tuple(float(row[f]) for f in IMU_FIELDS)
               for row in csv.DictReader(handle)]
    hr = _column(MOCK_DIR / "WA_Chest.csv", "ecg_hr_bpm")
    spo2 = _column(MOCK_DIR / "WD_L_Wrist.csv", "ppg_spo2_pct")
    resp = _column(MOCK_DIR / "WA_Chest.csv", "resp_rate_bpm")
    hrv = _column(MOCK_DIR / "WA_Chest.csv", "ecg_rmssd_ms")
    rows = []
    for i, imu_row in enumerate(imu):
        rows.append(imu_row + (
            hr[i] if hr else 92.0,
            spo2[i] if spo2 else 97.0,
            resp[i] if resp else 20.0,
            hrv[i] if hrv else 44.0,
        ))
    return rows


class MockBoard(threading.Thread):
    """One emulated ESP32 wearable, speaking the real UDP protocol.

    Working modes mirror ``wearable_mode_t`` (peq0-v1-head-tests/src/app_ctrl.h)
    and the grammar in the app's ``wearableMode.ts``:

      idle    nothing on the wire. Impacts are still DETECTED and held, so
              switching away from idle replays what happened while it was quiet.
      live    procedural telemetry, distinct per board.
      alerts  the same detector with telemetry off — impacts only.
      mock    the HEAD mock CSV on a loop.

    HELLO + MODE_RPT go out every 2 s in every mode, including idle: that is
    what keeps the board present in ``active_wearables()`` (and so "detected" in
    the app) when it is sending nothing else.
    """

    def __init__(self, wid: int, index: int, count: int, dest, *,
                 mode: str = "idle", period_ms: int = 255,
                 impact_every: float = 30.0, rows=None, verbose: bool = False):
        super().__init__(daemon=True, name=f"mock-board-{wid:04X}")
        self.wid = wid
        self.index = index
        self.count = count
        self.node = udp_source.node_for_wid(wid)
        self.dest = dest
        self.mode = MODES[mode]
        self.period_s = max(0.01, period_ms / 1000.0)
        self.impact_every = impact_every
        self.rows = rows
        self.verbose = verbose

        # Per-board personality, so ten participants do not look identical on
        # screen. Seeded from the wid: the same board reads the same every run.
        self.rng = random.Random(wid)
        self.phase = index * 0.37
        self.hr_base = 88.0 + 3.0 * index
        self.spo2_base = 96.0 + (index % 3)
        self.resp_base = 16.0 + (index % 5)
        self.hrv_base = 40.0 + 2.0 * index
        self.row_offset = (index * 977) % len(rows) if rows else 0

        self._stop = threading.Event()
        self._sock = None
        self._seq = 0
        self._t0 = time.monotonic()
        # Impact bookkeeping, matching the board's running totals since boot.
        self._impact_seq = 0
        self._impact_count = 0
        self._impact_sum = 0.0
        self._impact_max = 0.0
        self._unacked = {}    # seq -> [packet, last_tx, tries]
        self._held = []       # packets detected while idle, replayed on exit
        self.forgotten = False

    # -- lifecycle ---------------------------------------------------------- #
    def stop(self) -> None:
        self._stop.set()

    def _t_ms(self, now: float) -> int:
        return int((now - self._t0) * 1000)

    # -- data --------------------------------------------------------------- #
    def _live_values(self, t: float):
        """Procedural telemetry. Accel stays near 1 g so nothing here is ever
        mistaken for a real impact — those are injected explicitly below."""
        ph = t * 2.0 + self.phase
        breath = math.sin(t * 2 * math.pi * self.resp_base / 60.0)
        hr = self.hr_base + 6 * math.sin(t / 12 + self.phase) + 2 * breath \
            + self.rng.gauss(0, 0.6)
        spo2 = min(100.0, self.spo2_base + 0.4 * math.sin(t / 20 + self.phase)
                   + self.rng.gauss(0, 0.15))
        return (
            0.08 * math.sin(ph) + self.rng.gauss(0, 0.01),
            0.08 * math.cos(ph * 0.7) + self.rng.gauss(0, 0.01),
            -1.0 + 0.05 * math.sin(ph * 2),
            12.0 * math.sin(ph), 8.0 * math.cos(ph * 1.3), 4.0 * math.sin(ph * 0.5),
            0.2 * math.sin(ph * 0.2), 0.3 * math.cos(ph * 0.2), -0.9,
            25.0 + 0.5 * math.sin(t / 60) + 0.1 * self.index,
            hr, spo2,
            self.resp_base + 1.2 * math.sin(t / 25 + self.phase),
            self.hrv_base + 5 * math.sin(t / 18 + self.phase) + self.rng.gauss(0, 1.0),
        )

    def _csv_values(self):
        row = self.rows[(self.row_offset + self._seq) % len(self.rows)]
        return row

    def _values(self, t: float):
        if self.mode == MODES["mock"] and self.rows:
            return self._csv_values()
        return self._live_values(t)

    # -- transmit ----------------------------------------------------------- #
    def _send(self, packet: bytes) -> None:
        try:
            self._sock.sendto(packet, self.dest)
        except OSError as exc:
            print(f"# [{self.node}] send failed: {exc}", file=sys.stderr)

    def _send_sample(self, now: float) -> None:
        t_ms = self._t_ms(now)
        values = self._values(now - self._t0)
        self._send(HDR.pack(MSG_IMU, VERSION, self.wid)
                   + IMU_BODY.pack(self._seq, t_ms, *values))
        self._seq += 1
        if self.verbose:
            print(f"# [{self.node}] seq={self._seq} hr={values[10]:.0f} "
                  f"spo2={values[11]:.1f}", file=sys.stderr)

    def _detect_impact(self, now: float) -> None:
        """Build one impact record. The detector runs in EVERY mode — in idle
        the record is held rather than dropped, which is what makes switching
        out of idle replay the hits that happened while it was quiet."""
        peak_g = self.rng.uniform(25.0, 60.0)
        dur_ms = self.rng.randint(12, 25)
        self._impact_seq += 1
        self._impact_count += 1
        self._impact_sum += peak_g
        self._impact_max = max(self._impact_max, peak_g)
        packet = ALERT.pack(MSG_ALERT, VERSION, self.wid, self._impact_seq,
                            self._t_ms(now), peak_g, IMPACT_THRESHOLD_G,
                            self._impact_sum, self._impact_max,
                            self._impact_count, dur_ms)
        if self.mode == MODES["idle"]:
            self._held.append(packet)
            print(f"# [{self.node}] impact #{self._impact_seq} {peak_g:.1f}g "
                  f"held (idle)", file=sys.stderr)
            return
        self._transmit_impact(packet, now)

    def _transmit_impact(self, packet: bytes, now: float) -> None:
        seq = ALERT.unpack(packet)[3]
        self._unacked[seq] = [packet, now, 1]
        self._send(packet)
        print(f"# [{self.node}] IMPACT #{seq} {ALERT.unpack(packet)[5]:.1f}g sent",
              file=sys.stderr)

    def _flush_held(self, now: float) -> None:
        if not self._held:
            return
        print(f"# [{self.node}] replaying {len(self._held)} impact(s) held "
              f"while idle", file=sys.stderr)
        for packet in self._held:
            self._transmit_impact(packet, now)
        self._held.clear()

    def _retransmit(self, now: float) -> None:
        # The board resends until the Pi acks, so the emulation must too — it is
        # what exercises the receiver's ack/dedupe path.
        for seq, entry in list(self._unacked.items()):
            if now - entry[1] < ALERT_RETRY_S:
                continue
            if entry[2] >= ALERT_MAX_TRIES:
                print(f"# [{self.node}] IMPACT #{seq} unacked after "
                      f"{ALERT_MAX_TRIES} tries — giving up", file=sys.stderr)
                del self._unacked[seq]
                continue
            entry[1] = now
            entry[2] += 1
            self._send(entry[0])

    # -- receive ------------------------------------------------------------ #
    def _set_mode(self, code: int, now: float) -> None:
        name = MODE_NAMES.get(code)
        if name is None or code == self.mode:
            return
        was_idle = self.mode == MODES["idle"]
        self.mode = code
        print(f"# [{self.node}] mode -> {name}", file=sys.stderr)
        if was_idle:
            self._flush_held(now)

    def _receive(self, now: float) -> None:
        while True:
            try:
                data, _addr = self._sock.recvfrom(64)
            except BlockingIOError:
                return
            except OSError:
                return
            if len(data) < HDR.size:
                continue
            msg_type = data[0]
            if msg_type == MSG_WELCOME and len(data) == WELCOME.size:
                continue  # handshake completed; nothing for us to do
            if msg_type == MSG_ALERT_ACK and len(data) == ALERT_ACK.size:
                self._unacked.pop(ALERT_ACK.unpack(data)[3], None)
            elif msg_type == MSG_MODE and len(data) == MODE.size:
                self._set_mode(MODE.unpack(data)[4], now)
            elif msg_type == MSG_FORGET and len(data) == FORGET.size:
                # A real board erases its credentials and leaves the network; it
                # then ages out of active_wearables() after 10 s, which is the
                # signal the app's release-to-Bluetooth flow looks for.
                print(f"# [{self.node}] FORGET — leaving the network",
                      file=sys.stderr)
                self.forgotten = True
                self._stop.set()

    # -- main loop ---------------------------------------------------------- #
    def run(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._t0 = time.monotonic()
        next_hello = 0.0
        next_sample = 0.0
        # Stagger the first impact per board, or all ten fire in the same second.
        next_impact = (self._t0 + self.impact_every * (0.3 + self.index / self.count)
                       if self.impact_every > 0 else float("inf"))
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if now >= next_hello:
                    self._send(HELLO.pack(MSG_HELLO, VERSION, self.wid, self._seq))
                    # The mode report rides alongside every HELLO as a separate
                    # message, exactly as the firmware sends it.
                    self._send(MODE_RPT.pack(MSG_MODE_RPT, VERSION, self.wid, self.mode))
                    next_hello = now + HELLO_INTERVAL_S
                if self.mode in (MODES["live"], MODES["mock"]) and now >= next_sample:
                    self._send_sample(now)
                    next_sample = now + self.period_s
                if now >= next_impact:
                    self._detect_impact(now)
                    next_impact = now + self.impact_every
                self._retransmit(now)
                self._receive(now)
                self._stop.wait(TICK_S)
        finally:
            if self._sock is not None:
                self._sock.close()


class MockFleet:
    """The emulated boards, started and stopped together."""

    def __init__(self, count: int, wid_base: int, dest, **board_kwargs):
        self.wids = [wid_base + i for i in range(count)]
        self.boards = [
            MockBoard(wid, i, count, dest, **board_kwargs)
            for i, wid in enumerate(self.wids)
        ]

    @property
    def nodes(self):
        """Node labels for every board, so Meta is complete before the first
        sample arrives (rather than growing via BleSender._ensure_node)."""
        return [board.node for board in self.boards]

    def start(self) -> None:
        for board in self.boards:
            board.start()
        print(f"# mock fleet: {len(self.boards)} boards "
              f"{self.nodes[0]}…{self.nodes[-1]} started", file=sys.stderr)

    def stop(self) -> None:
        for board in self.boards:
            board.stop()
        for board in self.boards:
            board.join(timeout=1.0)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--count", type=int, default=MOCK_COUNT,
                   help="emulated wearables (default: %(default)s; the app's "
                        "mock-devices.ts expects this many)")
    p.add_argument("--wid-base", type=lambda s: int(s, 0), default=MOCK_WID_BASE,
                   help="first wearable id (default: 0x%(default)04X); MUST match "
                        "MOCK_WID_BASE in the app's mock-devices.ts")
    p.add_argument("--mode", choices=tuple(MODES), default="idle",
                   help="working mode the boards start in (default: %(default)s, "
                        "as a real board boots); the app changes it from the "
                        "session screen")
    p.add_argument("--period-ms", type=int, default=255,
                   help="per-board sample period (default: %(default)s ms, the "
                        "firmware's cadence)")
    p.add_argument("--impact-every", type=float, default=30.0, metavar="SECONDS",
                   help="synthetic head impact per board this often, staggered "
                        "across the fleet (0 = never; default: %(default)s)")
    p.add_argument("--csv", default=str(DEFAULT_CSV),
                   help="HEAD mock CSV replayed in 'mock' mode")
    p.add_argument("--ap", action="store_true",
                   help="also bring up the hidden WiFi AP. Off by default: no "
                        "emulated board needs it, and leaving it down keeps this "
                        "script from touching NetworkManager at all")
    p.add_argument("--config", default=None,
                   help="receiver config JSON (default: receiver_config.json "
                        "next to this script)")
    p.add_argument("--live-period-ms", type=int, default=100,
                   help="BLE queue drain interval (default: %(default)s ms)")
    p.add_argument("--chunk-size", type=int, default=protocol.DEFAULT_CHUNK_SIZE,
                   help="max notification payload in bytes (default: %(default)s)")
    p.add_argument("--name", default=None,
                   help="BLE advertised name (default: this Pi's receiver_name "
                        "from its config — the app's identity gate rejects "
                        "anything else)")
    p.add_argument("--adapter", default=None,
                   help="Bluetooth adapter address (default: first available)")
    p.add_argument("--stdout", action="store_true",
                   help="dry run: stream NDJSON to stdout, no BLE and no config "
                        "file (works on macOS)")
    p.add_argument("--verbose", action="store_true", help="log samples to stderr")
    return p.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    if args.count < 1:
        raise SystemExit("--count must be >= 1")
    # The advertisement cap (protocol.PRESENCE_MAX_WIDS) is irrelevant here —
    # mock mode does not broadcast presence at all. What matters is that the
    # app's mock-devices.ts knows exactly MOCK_COUNT serials from MOCK_WID_BASE;
    # boards outside that block stream to nodes no participant can be assigned.
    if args.wid_base != MOCK_WID_BASE or args.count != MOCK_COUNT:
        print(f"# warning: fleet is 0x{args.wid_base:04X}+{args.count}, not the "
              f"0x{MOCK_WID_BASE:04X}+{MOCK_COUNT} the app's mock-devices.ts "
              f"expects — change both sides or the phone shows devices that "
              f"never send anything", file=sys.stderr)

    if args.stdout:
        # No config: load_config() would MINT A NEW RECEIVER IDENTITY when the
        # file is absent (new SSID, password and pi_id), orphaning every real
        # wearable provisioned to this Pi. A dry run must never risk that, and
        # port 0 also keeps it clear of a receiver already running.
        cfg = {"pi_id": 1, "udp_port": 0, "receiver_name": protocol.DEFAULT_DEVICE_NAME}
        creds = b"{}"
    else:
        cfg_path = Path(args.config).expanduser() if args.config \
            else wifi_ap.DEFAULT_CONFIG_PATH
        cfg = wifi_ap.load_config(cfg_path)
        if args.ap:
            wifi_ap.ensure_ap(cfg)
        creds = wifi_ap.wifi_creds_json(cfg)

    source = UdpImuSource(cfg["udp_port"], cfg["pi_id"], verbose=args.verbose)
    source.start()

    rows = load_head_mock_rows(Path(args.csv))
    fleet = MockFleet(args.count, args.wid_base,
                      ("127.0.0.1", source.port),
                      mode=args.mode, period_ms=args.period_ms,
                      impact_every=args.impact_every, rows=rows,
                      verbose=args.verbose)
    fleet.start()

    try:
        if args.stdout:
            ble_sender.run_stdout_live(source, args.live_period_ms)
            return

        # Pre-seed the node tables with every board, so Meta is complete at the
        # first subscribe instead of being re-sent as each board checks in.
        nodes = fleet.nodes
        field_specs, layouts, node_layout = protocol.build_live_protocol_meta(nodes)
        meta = ble_sender.build_meta("live-udp", [], nodes, args.chunk_size,
                                     field_specs, layouts, node_layout,
                                     period_ms=args.live_period_ms)
        name = args.name or cfg["receiver_name"]
        sender = ble_sender.BleSender(
            [], meta, name, args.adapter,
            args.chunk_size, 1.0, False, args.verbose,
            nodes, field_specs, layouts, node_layout,
            source=source, live_period_ms=args.live_period_ms,
            wifi_creds_bytes=creds)
        # No presence broadcast. Ten live wids need 26 bytes of manufacturer
        # data, and with Flags + the 128-bit service UUID already taking 21 of
        # the 31 an advertisement holds, BlueZ rejects the whole thing — the
        # receiver then advertises NOTHING and the app cannot see it to pair or
        # to start a session. There is nothing to gain by paying those bytes
        # either: the app seeds the simulated fleet's wids itself (MOCK_WIDS in
        # features/devices/mock-devices.ts), so it already knows they are live.
        sender.presence_adv_enabled = False
        print(f"# MOCK RECEIVER — {args.count} emulated wearables, no real "
              f"hardware. Do not run alongside ble_sender.py.", file=sys.stderr)
        print("# presence-adv: off (the app seeds the simulated fleet itself)",
              file=sys.stderr)
        sender.run()
    finally:
        fleet.stop()
        source.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
