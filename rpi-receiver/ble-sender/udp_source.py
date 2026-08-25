"""Live IMU source: receive ESP32-C6 UDP packets and queue them as samples.

Runs a daemon thread with a UDP socket (same wire format as
``udp_imu_receiver/udp_imu_receiver.py``, which stays around as a standalone
debug tool). ``BleSender`` drains the queue on its GLib tick and streams the
samples over BLE in place of the CSV replay.

No BLE imports here, so this module (and its loopback tests) run anywhere.

Message framing — all messages start with a 4-byte header:
    uint8  msg_type   (1=IMU, 2=HELLO, 3=WELCOME, 4=FORGET)
    uint8  version
    uint16 wearable_id

IMU     (68 bytes, wearable->Pi) = header + uint32 seq, uint32 t_ms, 14 floats
                                   (ax,ay,az, gx,gy,gz, hx,hy,hz, temp_c,
                                    hr, spo2, resp, hrv)  -- the 4 bio values
                                   are 0 from a real sensor, filled by the mock
HELLO   (8 bytes, wearable->Pi)  = header + uint32 nonce
WELCOME (12 bytes, Pi->wearable) = header + uint32 nonce + uint32 pi_id
FORGET  (8 bytes, Pi->wearable)  = header + uint32 pi_id
    wearable_id addresses the target board (0 = any); the board erases its
    stored WiFi credentials and drops off the network ("unpair").
"""
from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from collections import deque
from typing import Dict, List, Optional

import protocol

MSG_IMU, MSG_HELLO, MSG_WELCOME, MSG_FORGET = 1, 2, 3, 4
MSG_ALERT, MSG_ALERT_ACK = 5, 6
MSG_MODE, MSG_MODE_RPT = 7, 8
VERSION = 1

# Working modes, mirrored from wearable_mode_t in
# peq0-v1-head-tests/src/app_ctrl.h. The NAMES are the control-characteristic
# grammar the app writes ("mode alerts 3"); the NUMBERS are what goes on the
# UDP wire. Both sides of that mapping live here.
MODES = {"idle": 0, "live": 1, "alerts": 2, "mock": 3}
MODE_NAMES = {v: k for k, v in MODES.items()}

HDR = struct.Struct("<BBH")         # msg_type, version, wearable_id  (4)
IMU_BODY = struct.Struct("<II14f")  # seq, t_ms, 10 IMU + 4 bio floats (64)
HELLO = struct.Struct("<BBHI")      # header + nonce                  (8)
WELCOME = struct.Struct("<BBHII")   # header + nonce + pi_id          (12)
FORGET = struct.Struct("<BBHI")     # header + pi_id                  (8)
# One head impact: header + seq, t_ms, peak, threshold, sum, max, count, dur.
ALERT = struct.Struct("<BBHIIffffIH")   # (34)
ALERT_ACK = struct.Struct("<BBHI")      # header + acked seq          (8)
MODE = struct.Struct("<BBHIB")          # header + pi_id + mode       (9)
MODE_RPT = struct.Struct("<BBHB")       # header + mode               (5)

IMU_SIZE = HDR.size + IMU_BODY.size  # 68
HELLO_SIZE = HELLO.size              # 8

# These must match alert_packet_t / alert_ack_packet_t in wifi_udp_tx.cpp, which
# carries the same assertion. A silent layout drift here decodes an impact into
# garbage rather than failing, so assert it at import.
assert ALERT.size == 34, f"ALERT layout drifted: {ALERT.size} != 34"
assert ALERT_ACK.size == 8, f"ALERT_ACK layout drifted: {ALERT_ACK.size} != 8"
assert MODE.size == 9, f"MODE layout drifted: {MODE.size} != 9"
assert MODE_RPT.size == 5, f"MODE_RPT layout drifted: {MODE_RPT.size} != 5"

# Legacy fixed body-position map, kept for mock/replay (CSV) which still uses
# body-position node names. The LIVE source no longer defaults to it: each board
# streams under its own MAC-derived wid (= its serial suffix), and the node label
# is that wid as 4-hex-digit, so same-position boards from different people never
# collide. Pass this map explicitly to keep the old HEAD/WA/WD/WE labels.
#   1 = HEAD (other)   2 = WA (chest)   3 = WD (left wrist)   4 = WE (right wrist)
WID_TO_NODE: Dict[int, str] = {1: "HEAD", 2: "WA", 3: "WD", 4: "WE"}


def node_for_wid(wid: int, overrides: Optional[Dict[int, str]] = None) -> str:
    """Body node for a wearable id: an explicit override (mock/legacy body
    positions) if present, else the wid as 4-hex-digit — which equals the
    board's serial suffix (aurmor-mibs-XXXX), so the app can attribute the
    node's samples to a specific device by matching that suffix."""
    if overrides and wid in overrides:
        return overrides[wid]
    return f"{wid:04X}"

# t_ms is the board's monotonic uptime clock. If it jumps back by more than
# this, the board rebooted and we re-baseline its t_s instead of going negative.
_REBOOT_JUMP_MS = 5000
# Gap inserted between the pre- and post-reboot timelines (one 255 ms frame).
_REBASE_GAP_S = 0.255


class UdpImuSource:
    """Background UDP receiver: HELLO/WELCOME handshake + IMU sample queue."""

    def __init__(self, port: int, pi_id: int,
                 wid_to_node: Optional[Dict[int, str]] = None,
                 max_queue: int = 1024, verbose: bool = False):
        self.port = port
        self.pi_id = pi_id
        # Live default: no fixed map — each board uses its MAC-derived hex node.
        # Mock/replay passes an explicit body-position map.
        self.wid_to_node = dict(wid_to_node) if wid_to_node is not None else {}
        self.verbose = verbose

        self._queue: deque = deque(maxlen=max_queue)
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._last_addr: Dict[int, tuple] = {}   # wid -> last source address
        self._last_seen: Dict[int, float] = {}   # wid -> monotonic time of last packet
        self._t0_ms: Dict[int, int] = {}         # wid -> first-seen t_ms
        self._rebase_s: Dict[int, float] = {}    # wid -> offset added after reboots
        self._last_t_s: Dict[int, float] = {}    # wid -> last emitted t_s
        self._last_alert_seq: Dict[int, int] = {}  # wid -> last impact seq queued
        self._modes: Dict[int, str] = {}         # wid -> working mode it reports
        self._dropped = 0
        self._last_drop_log = 0.0

    def _node_for(self, wid: int) -> str:
        return node_for_wid(wid, self.wid_to_node)

    @property
    def nodes(self) -> List[str]:
        # Any explicit override nodes, plus a node for every board heard from.
        seen = {self._node_for(wid) for wid in self._last_seen}
        return sorted(set(self.wid_to_node.values()) | seen)

    def active_wearables(self, max_age_s: float = 10.0) -> Dict[int, str]:
        """Wearables heard from within ``max_age_s`` (wid -> node).

        Boards HELLO every 2 s whenever they are on the network, so a healthy
        idle wearable never ages out. Served over the Wearables characteristic
        so the app can show presence for BLE-silent (provisioned) boards.
        """
        cutoff = time.monotonic() - max_age_s
        return {wid: self._node_for(wid) for wid, seen in self._last_seen.items()
                if seen > cutoff}

    # -- lifecycle ----------------------------------------------------------- #
    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.port))
        self.port = sock.getsockname()[1]  # resolve port 0 (tests) to the real one
        sock.settimeout(0.5)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True,
                                        name="udp-imu-source")
        self._thread.start()
        print(f"# UDP source listening on 0.0.0.0:{self.port} "
              f"(pi_id={self.pi_id}, nodes={self.nodes})", file=sys.stderr)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    # -- consumption --------------------------------------------------------- #
    def drain(self) -> List[dict]:
        """Pop every queued sample (oldest first)."""
        out: List[dict] = []
        while True:
            try:
                out.append(self._queue.popleft())
            except IndexError:
                return out

    def requeue(self, items: List[dict]) -> None:
        """Put drained records back at the FRONT, preserving their order.

        Used when a subscriber discards a stale sample backlog but must keep the
        impacts that were mixed into it.
        """
        for item in reversed(items):
            self._queue.appendleft(item)

    # -- unpair -------------------------------------------------------------- #
    def send_forget(self, wid: Optional[int] = None,
                    retries: int = 5, interval_s: float = 0.2) -> bool:
        """Tell wearable ``wid`` (or every known one) to forget its WiFi.

        Returns False when no matching board has ever sent us a packet, so
        there is no address to deliver to (the app treats that as best-effort
        failure and tells the user about the manual button reset).
        """
        if wid is not None:
            targets = {wid: self._last_addr.get(wid)}
        else:
            targets = dict(self._last_addr)
        targets = {w: a for w, a in targets.items() if a is not None}
        if not targets or self._sock is None:
            print(f"# forget: no known address for wid={wid if wid is not None else 'any'}",
                  file=sys.stderr)
            return False
        for i in range(retries):
            for w, addr in targets.items():
                pkt = FORGET.pack(MSG_FORGET, VERSION, w, self.pi_id)
                try:
                    self._sock.sendto(pkt, addr)
                except OSError as exc:
                    print(f"# forget send failed: {exc}", file=sys.stderr)
                    return False
            if i + 1 < retries:
                time.sleep(interval_s)
        print(f"# forget sent to {sorted(targets)} ({retries}x)", file=sys.stderr)
        return True

    # -- working mode -------------------------------------------------------- #
    def send_mode(self, wid: int, mode, retries: int = 5,
                  interval_s: float = 0.2) -> bool:
        """Tell wearable ``wid`` which working mode to run.

        This is the group-session path: the board is on our WiFi with its BLE
        off, so the app cannot reach it directly and asks us to relay. ``mode``
        is a name from MODES or the number itself.

        A wid is REQUIRED — deliberately. The bare-``forget`` broadcast was
        removed because one unauthenticated write could drop a whole team
        mid-session, and "silence every board at once" is the same hazard. The
        app sends one call per board instead; a squad-wide toggle is a UI
        affordance, not a wire broadcast.

        Fire-and-forget with repeats, like send_forget: the board reports what
        it actually settled on in its MODE_RPT (see modes()), so there is
        nothing to ack.
        """
        code = MODES.get(mode) if isinstance(mode, str) else int(mode)
        if code is None or code not in MODE_NAMES:
            print(f"# mode: unknown mode {mode!r}", file=sys.stderr)
            return False
        addr = self._last_addr.get(wid)
        if addr is None or self._sock is None:
            print(f"# mode: no known address for wid={wid}", file=sys.stderr)
            return False
        pkt = MODE.pack(MSG_MODE, VERSION, wid, self.pi_id, code)
        for i in range(retries):
            try:
                self._sock.sendto(pkt, addr)
            except OSError as exc:
                print(f"# mode send failed: {exc}", file=sys.stderr)
                return False
            if i + 1 < retries:
                time.sleep(interval_s)
        print(f"# mode {MODE_NAMES[code]} sent to wid={wid} ({retries}x)",
              file=sys.stderr)
        return True

    def modes(self) -> Dict[int, str]:
        """wid -> the working mode each board says it is in.

        Sourced from MODE_RPT, not from what we commanded: a board reboots into
        idle and its BOOT button can start a demo without us knowing.
        """
        return dict(self._modes)

    # -- receive loop -------------------------------------------------------- #
    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return  # socket closed under us during stop()
            if len(data) < HDR.size:
                continue
            msg_type, version, wid = HDR.unpack_from(data, 0)
            if version != VERSION:
                continue

            if msg_type == MSG_HELLO and len(data) == HELLO_SIZE:
                self._last_addr[wid] = addr
                self._last_seen[wid] = time.monotonic()
                _, _, _, nonce = HELLO.unpack(data)
                reply = WELCOME.pack(MSG_WELCOME, VERSION, wid, nonce, self.pi_id)
                self._sock.sendto(reply, addr)
                print(f"# [{addr[0]}] HELLO wid={wid} -> WELCOME (pi_id={self.pi_id})",
                      file=sys.stderr)
                continue

            if msg_type == MSG_MODE_RPT and len(data) == MODE_RPT.size:
                # Rides alongside every HELLO. Counts as liveness too: a board
                # in alerts mode may send no telemetry for minutes.
                self._last_addr[wid] = addr
                self._last_seen[wid] = time.monotonic()
                code = MODE_RPT.unpack(data)[3]
                name = MODE_NAMES.get(code)
                if name is not None and self._modes.get(wid) != name:
                    print(f"# [{addr[0]}] wid={wid} mode={name}", file=sys.stderr)
                self._modes[wid] = name or f"?{code}"
                continue

            if msg_type == MSG_IMU and len(data) == IMU_SIZE:
                self._last_addr[wid] = addr
                self._last_seen[wid] = time.monotonic()
                # Every board streams — its node is its MAC-derived hex wid (or an
                # explicit override for mock/replay). No board is dropped now that
                # identity comes from the wid itself rather than a fixed 4-slot map.
                self._enqueue(wid, self._node_for(wid), data)
                continue

            if msg_type == MSG_ALERT and len(data) == ALERT.size:
                self._last_addr[wid] = addr
                self._last_seen[wid] = time.monotonic()
                seq = ALERT.unpack(data)[3]
                # ACK FIRST, and ack duplicates too. The board retransmits until
                # it hears one, so a dropped ack — the common case — otherwise
                # means it resends the same impact forever.
                self._sock.sendto(ALERT_ACK.pack(MSG_ALERT_ACK, VERSION, wid, seq), addr)
                if self._last_alert_seq.get(wid) == seq:
                    continue        # duplicate: our earlier ack was lost
                self._last_alert_seq[wid] = seq
                self._enqueue_impact(wid, self._node_for(wid), data)

    def _t_s_for(self, wid: int, t_ms: int) -> float:
        """Board uptime -> session timeline, surviving a mid-session reboot.

        Shared by samples and impacts so both land on ONE timeline: an impact
        rebased differently from the surrounding samples would be replayed at
        the wrong moment.
        """
        t0 = self._t0_ms.get(wid)
        if t0 is None or t_ms + _REBOOT_JUMP_MS < t0:
            if t0 is not None:  # board rebooted: keep t_s monotonic
                self._rebase_s[wid] = self._last_t_s.get(wid, 0.0) + _REBASE_GAP_S
            self._t0_ms[wid] = t0 = t_ms
        t_s = round((t_ms - t0) / 1000.0 + self._rebase_s.get(wid, 0.0), 3)
        self._last_t_s[wid] = t_s
        return t_s

    def _enqueue_impact(self, wid: int, node: str, data: bytes) -> None:
        (_type, _ver, _wid, seq, t_ms,
         peak_g, threshold_g, sum_g, max_g, count, dur_ms) = ALERT.unpack(data)

        # Impacts are never dropped for queue pressure the way samples are.
        # There are at most a handful per session and each one is the reason
        # this product exists; a full queue means the phone is behind, not that
        # this record is disposable.
        self._queue.append({
            "type": "impact",
            "node": node,
            "t_s": self._t_s_for(wid, t_ms),
            "seq": seq,
            "peak_g": round(peak_g, 3),
            "threshold_g": round(threshold_g, 3),
            "dur_ms": dur_ms,
            # Board-side running totals since ITS boot — loss detection only.
            "count": count,
            "max_g": round(max_g, 3),
            "sum_g": round(sum_g, 3),
        })
        print(f"# [{node}] IMPACT #{seq} {peak_g:.1f}g (>{threshold_g:.0f}g) "
              f"dur={dur_ms}ms board_count={count}", file=sys.stderr)

    def _enqueue(self, wid: int, node: str, data: bytes) -> None:
        (_seq, t_ms,
         ax, ay, az, gx, gy, gz, hx, hy, hz, temp_c,
         hr, spo2, resp, hrv) = IMU_BODY.unpack_from(data, HDR.size)

        t_s = self._t_s_for(wid, t_ms)

        if len(self._queue) == self._queue.maxlen:
            self._dropped += 1
            now = time.monotonic()
            if now - self._last_drop_log >= 5.0:
                print(f"# UDP queue full — dropped {self._dropped} samples so far "
                      f"(no BLE subscriber draining?)", file=sys.stderr)
                self._last_drop_log = now
        sample = {"node": node, "t_s": t_s}
        for field, value in zip(protocol.LIVE_IMU_FIELDS,
                                (ax, ay, az, gx, gy, gz, hx, hy, hz, temp_c)):
            sample[field] = round(value, 6)
        # Bio fields (Heart rate / SpO2 / Respiration / HRV): filled from CSV by
        # the mock, and 0 from a real IMU (which has no bio channels — a real
        # HEAD node would show 0s here until a chest/wrist sensor is simulated).
        for field, value in zip(protocol.LIVE_BIO_FIELDS, (hr, spo2, resp, hrv)):
            sample[field] = round(value, 3)
        self._queue.append(sample)
        if self.verbose:
            print(f"# [{node}] t_s={t_s:.3f} a=({ax:+.3f},{ay:+.3f},{az:+.3f})g "
                  f"hr={hr:.0f} spo2={spo2:.0f}", file=sys.stderr)
