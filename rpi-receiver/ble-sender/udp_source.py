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
VERSION = 1

HDR = struct.Struct("<BBH")         # msg_type, version, wearable_id  (4)
IMU_BODY = struct.Struct("<II14f")  # seq, t_ms, 10 IMU + 4 bio floats (64)
HELLO = struct.Struct("<BBHI")      # header + nonce                  (8)
WELCOME = struct.Struct("<BBHII")   # header + nonce + pi_id          (12)
FORGET = struct.Struct("<BBHI")     # header + pi_id                  (8)

IMU_SIZE = HDR.size + IMU_BODY.size  # 52
HELLO_SIZE = HELLO.size              # 8

# Which body node each wearable_id maps to. Only mapped IDs are streamed.
# The app assigns a wid per ESP32 from the body position picked at pairing
# (see ESP32_KIND_MAP in the app's esp32-provisioning/protocol.ts) — this table
# MUST agree with it so multiple boards land on distinct, non-colliding nodes.
#   1 = HEAD (other)   2 = WA (chest)   3 = WD (left wrist)   4 = WE (right wrist)
WID_TO_NODE: Dict[int, str] = {1: "HEAD", 2: "WA", 3: "WD", 4: "WE"}

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
        self.wid_to_node = dict(WID_TO_NODE if wid_to_node is None else wid_to_node)
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
        self._dropped = 0
        self._last_drop_log = 0.0

    @property
    def nodes(self) -> List[str]:
        return sorted(set(self.wid_to_node.values()))

    def active_wearables(self, max_age_s: float = 10.0) -> Dict[int, str]:
        """Mapped wearables heard from within ``max_age_s`` (wid -> node).

        Boards HELLO every 2 s whenever they are on the network, so a healthy
        idle wearable never ages out. Served over the Wearables characteristic
        so the app can show presence for BLE-silent (provisioned) boards.
        """
        cutoff = time.monotonic() - max_age_s
        return {wid: node for wid, node in self.wid_to_node.items()
                if self._last_seen.get(wid, 0.0) > cutoff}

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

            if msg_type == MSG_IMU and len(data) == IMU_SIZE:
                self._last_addr[wid] = addr
                self._last_seen[wid] = time.monotonic()
                node = self.wid_to_node.get(wid)
                if node is None:
                    continue  # unmapped board; handshake still answered above
                self._enqueue(wid, node, data)

    def _enqueue(self, wid: int, node: str, data: bytes) -> None:
        (_seq, t_ms,
         ax, ay, az, gx, gy, gz, hx, hy, hz, temp_c,
         hr, spo2, resp, hrv) = IMU_BODY.unpack_from(data, HDR.size)

        t0 = self._t0_ms.get(wid)
        if t0 is None or t_ms + _REBOOT_JUMP_MS < t0:
            if t0 is not None:  # board rebooted: keep t_s monotonic
                self._rebase_s[wid] = self._last_t_s.get(wid, 0.0) + _REBASE_GAP_S
            self._t0_ms[wid] = t0 = t_ms
        t_s = round((t_ms - t0) / 1000.0 + self._rebase_s.get(wid, 0.0), 3)
        self._last_t_s[wid] = t_s

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
