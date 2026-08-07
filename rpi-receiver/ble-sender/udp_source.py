"""Live wearable source: receive ESP32-C6 UDP packets and queue them as samples.

Runs a daemon thread with a UDP socket. ``BleSender`` drains the queues on its
GLib tick and streams to the mobile app over BLE.

No BLE imports here, so this module (and its loopback tests) run anywhere.

Message framing — all messages start with a 4-byte header:
    uint8  msg_type   (1=IMU, 2=HELLO, 3=WELCOME, 4=FORGET, 5=ALERT, 6=ALERT_ACK)
    uint8  version
    uint16 wearable_id

IMU v1  (68 bytes) = header + u32 seq, u32 t_ms, 14 floats
                     (ax,ay,az, gx,gy,gz, hx,hy,hz, temp_c, hr,spo2,resp,hrv)
IMU v2  (49 bytes) = header + u32 seq, u32 t_ms, u32 impact_count,
                     f impact_threshold, f impact_accumulator, f all_time_peak_g,
                     f temp_c, f hr,spo2,resp,hrv, u8 mode
ALERT   (48 bytes, wearable->Pi)  = header + u32 seq, u32 t_ms, f peak_g,
                     f threshold_g, 3f h_xyz, 3f g_xyz, u16 dur_ms, u8 mode, u8 xport
ALERT_ACK (8 bytes, Pi->wearable) = header + u32 seq   [MUST be sent or the
                     wearable retransmits; see wifi_udp_tx.cpp]
HELLO   (8 bytes)  = header + u32 nonce
WELCOME (12 bytes) = header + u32 nonce + u32 pi_id
FORGET  (8 bytes)  = header + u32 pi_id

WHY BOTH IMU VERSIONS: the firmware moved from streaming raw IMU axes to
streaming impact aggregates, but this receiver still gated on the v1 length
(``len(data) == IMU_SIZE``) with no else branch — so every packet from current
firmware was silently discarded while HELLO/WELCOME kept answering, making a
totally dead link look healthy. Both are accepted so old and new boards can run
side by side through a rollout, and anything unrecognised is now logged.

WHO A PACKET BELONGS TO is decided by the roster (roster.py), which the app owns
and edits at runtime. The receiver starts with NO athletes and NO device
assignments; until the app populates it, wearables are "unassigned" — still
handshaken, still discoverable, and their impacts are still recorded (see
_handle_alert) so a forgotten assignment never loses a head impact.
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
import roster as roster_mod
import schedule

MSG_IMU, MSG_HELLO, MSG_WELCOME, MSG_FORGET = 1, 2, 3, 4
MSG_ALERT, MSG_ALERT_ACK = 5, 6

VERSION = 1                      # legacy wire version (raw-IMU boards)
VERSION_V2 = 2                   # impact-aggregate + alert boards
ACCEPTED_VERSIONS = (VERSION, VERSION_V2)

HDR = struct.Struct("<BBH")          # msg_type, version, wearable_id   (4)
IMU_BODY = struct.Struct("<II14f")   # v1: seq, t_ms, 10 IMU + 4 bio     (64)
IMU_BODY_V1 = IMU_BODY
IMU_BODY_V2 = struct.Struct("<III4f4fB")  # seq,t_ms,count,4f agg,4f bio,mode (45)
ALERT_BODY = struct.Struct("<II8fHBB")    # seq,t_ms,peak,thr,h3,g3,dur,mode,xp (44)
HELLO = struct.Struct("<BBHI")       # header + nonce                    (8)
WELCOME = struct.Struct("<BBHII")    # header + nonce + pi_id            (12)
FORGET = struct.Struct("<BBHI")      # header + pi_id                    (8)
ALERT_ACK = struct.Struct("<BBHI")   # header + seq                      (8)
# Downlink config, Pi -> wearable. Lets the receiver throttle the whole squad
# to fit the airtime budget without reflashing anything.
#   policy: 0=ALERTS 1=LIVE 2=MOCK, 255 = leave alone
#   flags:  bit0 apply threshold_g, bit1 apply rate_hz
#   slot_us / frame_us: transmit offset inside a superframe. Staggering the
#   squad removes the CSMA backoff from every frame (~30% -> ~18% channel duty
#   at 180 devices) and stops 180 radios racing for the same instant.
CONFIG = struct.Struct("<BBHBBHfII")  # hdr + policy,flags,rate,thresh,slot,frame (20)
MSG_CONFIG = 7
CONFIG_FLAG_THRESHOLD = 0x01
CONFIG_FLAG_RATE = 0x02
CONFIG_FLAG_SLOT = 0x04
POLICY_KEEP = 255

IMU_SIZE = HDR.size + IMU_BODY_V1.size      # 68
IMU_SIZE_V2 = HDR.size + IMU_BODY_V2.size   # 49
ALERT_SIZE = HDR.size + ALERT_BODY.size     # 48
HELLO_SIZE = HELLO.size                     # 8

# The historical single-athlete layout, kept only so an existing rig can be
# expressed as a roster (Roster.from_legacy_map). NOT a default any more.
LEGACY_WID_TO_NODE: Dict[int, str] = {1: "HEAD", 2: "WA", 3: "WD", 4: "WE"}

_REBOOT_JUMP_MS = 5000
_REBASE_GAP_S = 0.255

# How long an unassigned wearable stays in the discovery list after going quiet.
UNASSIGNED_TTL_S = 60.0


class UdpImuSource:
    """Background UDP receiver: handshake, telemetry queue, and impact alerts."""

    def __init__(self, port: int, pi_id: int,
                 roster: Optional[roster_mod.Roster] = None,
                 max_queue: int = 1024, verbose: bool = False,
                 impact_store=None):
        self.port = port
        self.pi_id = pi_id
        # Default: monitor nobody. The app populates this.
        self.roster = roster if roster is not None else roster_mod.empty_roster()
        self.verbose = verbose
        self.store = impact_store

        # TELEMETRY: one slot per node, newest wins.
        #
        # A single shared FIFO does not survive a squad. At 180 devices the
        # queue fills with whoever transmits most, and a quiet athlete's sample
        # is evicted by a chatty one's backlog — drop-oldest silently becomes
        # drop-whoever-is-least-active. Coalescing per node bounds memory by
        # roster size instead of by traffic, and guarantees every node is
        # represented on every drain regardless of how fast its neighbours send.
        #
        # Intermediate samples are lost by design: the app renders at ~10 Hz and
        # full-fidelity data belongs in the telemetry log, not the BLE link.
        self._latest: Dict[str, dict] = {}
        self._latest_lock = threading.Lock()
        self._rx_count: Dict[int, int] = {}      # wid -> packets received
        self._rx_window_start = time.monotonic()

        # FOCUS: the handful of nodes the app is actually displaying get a real
        # FIFO at full rate. Everything else is covered by the batched summary.
        self._focus: set = set()
        self._focus_q: deque = deque(maxlen=max_queue)

        # Impacts get their own UNBOUNDED queue. Telemetry is disposable and
        # coalescing is correct for it; a dropped head impact is the failure
        # this feature exists to prevent, so it must never share a bound with a
        # chatty telemetry stream.
        self._impacts: deque = deque()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._last_addr: Dict[int, tuple] = {}
        self._last_seen: Dict[int, float] = {}
        self._t0_ms: Dict[int, int] = {}
        self._rebase_s: Dict[int, float] = {}
        self._last_t_s: Dict[int, float] = {}
        self._modes: Dict[int, int] = {}       # wid -> last reported app_mode_t
        self._peak_g: Dict[int, float] = {}    # wid -> last reported all-time peak
        self._dropped = 0
        self._last_drop_log = 0.0
        self._bad_len: Dict[int, int] = {}
        self._last_badlen_log = 0.0

    # -- roster -------------------------------------------------------------- #
    @property
    def nodes(self) -> List[str]:
        return self.roster.nodes()

    def set_roster(self, roster: roster_mod.Roster) -> None:
        """Hot-swap the roster (the app edited it). Safe mid-stream: node
        indices are append-only, so samples already in flight stay decodable."""
        self.roster = roster

    def active_wearables(self, max_age_s: float = 10.0) -> Dict[int, str]:
        """Assigned wearables heard from recently (wid -> node)."""
        # NOTE the explicit None check. `_last_seen.get(wid, 0.0) > cutoff` is
        # wrong: monotonic() starts near zero at boot, so for the first
        # max_age_s of uptime the 0.0 default beats a negative cutoff and every
        # rostered wearable reports as LIVE before any device has transmitted.
        # A head-impact system claiming to monitor athletes it cannot hear is
        # the worst failure this code can have.
        cutoff = time.monotonic() - max_age_s
        out: Dict[int, str] = {}
        for wid in self.roster.assigned_wids():
            seen = self._last_seen.get(wid)
            if seen is not None and seen > cutoff:
                info = self.roster.lookup(wid)
                if info:
                    out[wid] = info["node"]
        return out

    def unassigned_wearables(self, max_age_s: float = UNASSIGNED_TTL_S) -> List[dict]:
        """Boards on the network that no athlete owns yet.

        This is the discovery list the app needs to build a roster at all: with
        an empty roster every wearable lands here, and the user assigns each one
        to an athlete and a body position.
        """
        cutoff = time.monotonic() - max_age_s
        assigned = set(self.roster.assigned_wids())
        now = time.monotonic()
        out = []
        for wid, seen in sorted(self._last_seen.items()):
            if wid in assigned or seen <= cutoff:
                continue
            out.append({
                "wid": wid,
                "age_s": round(now - seen, 1),
                "ip": self._last_addr.get(wid, ("?",))[0],
                "mode": self._modes.get(wid),
                "peak_g": round(self._peak_g.get(wid, 0.0), 2),
            })
        return out

    def modes(self) -> Dict[int, int]:
        """wid -> last reported device mode (app_mode_t), for the app's UI."""
        return dict(self._modes)

    # -- lifecycle ----------------------------------------------------------- #
    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.port))
        self.port = sock.getsockname()[1]
        sock.settimeout(0.5)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True,
                                        name="udp-imu-source")
        self._thread.start()
        n = len(self.roster.assigned_wids())
        print(f"# UDP source listening on 0.0.0.0:{self.port} (pi_id={self.pi_id}, "
              f"{n} assigned wearable(s))"
              + ("" if n else " — roster empty, waiting for the app"),
              file=sys.stderr)

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
        """Most recent sample per node, one each. Bounded by roster size."""
        with self._latest_lock:
            out = list(self._latest.values())
            self._latest.clear()
        return out

    def drain_focus(self) -> List[dict]:
        """Full-rate samples for the focused nodes only."""
        out: List[dict] = []
        while True:
            try:
                out.append(self._focus_q.popleft())
            except IndexError:
                return out

    def set_focus(self, nodes) -> List[str]:
        """Nodes the app is displaying, which stream at full rate.

        Capped at FOCUS_MAX_NODES: this is the one path that can outrun the BLE
        link, so it is bounded here rather than trusted to the caller.
        """
        nodes = list(dict.fromkeys(nodes))[:protocol.FOCUS_MAX_NODES]
        self._focus = set(nodes)
        self._focus_q.clear()
        return nodes

    @property
    def focus(self) -> List[str]:
        return sorted(self._focus)

    # -- rate governor ------------------------------------------------------- #
    def rate_stats(self) -> dict:
        """Observed aggregate packet rate vs the airtime budget."""
        now = time.monotonic()
        window = max(now - self._rx_window_start, 1e-6)
        total = sum(self._rx_count.values())
        return {
            "devices_heard": len(self._rx_count),
            "packets": total,
            "window_s": round(window, 1),
            "pps": round(total / window, 1),
            "budget_pps": protocol.UDP_PACKET_BUDGET_PPS,
            "target_hz": self.target_rate_hz(),
        }

    def reset_rate_stats(self) -> None:
        self._rx_count.clear()
        self._rx_window_start = time.monotonic()

    def target_rate_hz(self) -> int:
        """Per-device telemetry rate that keeps the squad inside the budget."""
        n = max(len(self.roster.assigned_wids()), 1)
        return protocol.telemetry_rate_hz(n)

    def send_config(self, wid: int, policy: Optional[int] = None,
                    threshold_g: Optional[float] = None,
                    rate_hz: Optional[int] = None,
                    slot_us: Optional[int] = None,
                    frame_us: Optional[int] = None) -> bool:
        """Push mode/threshold/rate to one wearable over UDP.

        This is how the squad stays inside the airtime budget: the receiver
        knows how many devices are assigned, the wearable does not.
        """
        addr = self._last_addr.get(wid)
        if addr is None or self._sock is None:
            return False
        flags = 0
        if threshold_g is not None:
            flags |= CONFIG_FLAG_THRESHOLD
        if rate_hz is not None:
            flags |= CONFIG_FLAG_RATE
        if slot_us is not None and frame_us:
            flags |= CONFIG_FLAG_SLOT
        pkt = CONFIG.pack(MSG_CONFIG, VERSION_V2, wid,
                          POLICY_KEEP if policy is None else policy,
                          flags,
                          int(rate_hz or 0),
                          float(threshold_g or 0.0),
                          int(slot_us or 0),
                          int(frame_us or 0))
        try:
            self._sock.sendto(pkt, addr)
        except OSError as exc:
            print(f"# config send to wid={wid} failed: {exc}", file=sys.stderr)
            return False
        return True

    def govern(self, policy: Optional[int] = None,
               scheduled: bool = True) -> dict:
        """Push the budgeted telemetry rate — and a transmit slot — to every
        assigned wearable.

        Slots are keyed on the ATHLETE, so adding a sensor to one athlete does
        not reshuffle anybody else's timing mid-session. Each athlete's devices
        sub-slot inside their own window, which means a device only ever has to
        avoid colliding with its own athlete's sensors, never the whole squad.
        """
        hz = self.target_rate_hz()
        wids = self.roster.assigned_wids()
        plan = schedule.slot_plan(self.roster, hz) if scheduled else {}
        sent = 0
        for wid in wids:
            slot = plan.get(wid)
            sent += self.send_config(
                wid, policy=policy, rate_hz=hz,
                slot_us=slot["offset_us"] if slot else None,
                frame_us=slot["frame_us"] if slot else None)
        widths = {p["slot_us"] for p in plan.values()} or {0}
        print(f"# governor: {len(wids)} device(s) -> {hz} Hz each "
              f"(~{hz * max(len(wids), 1)} pps, budget "
              f"{protocol.UDP_PACKET_BUDGET_PPS})"
              + (f", slots {min(widths)} us wide" if plan else ", unscheduled")
              + f"; configured {sent}", file=sys.stderr)
        return {"rate_hz": hz, "configured": sent, "scheduled": bool(plan),
                "slot_us": min(widths)}

    def drain_impacts(self) -> List[dict]:
        """Pop every queued impact (oldest first). Never lossy."""
        out: List[dict] = []
        while True:
            try:
                out.append(self._impacts.popleft())
            except IndexError:
                return out

    # -- unpair -------------------------------------------------------------- #
    def send_forget(self, wid: Optional[int] = None,
                    retries: int = 5, interval_s: float = 0.2) -> bool:
        """Tell wearable ``wid`` to forget its WiFi.

        NOTE: wid=None targets EVERY known board — with a squad that unpairs
        everyone. Callers should pass an explicit wid.
        """
        if wid is not None:
            targets = {wid: self._last_addr.get(wid)}
        else:
            print("# forget: wid=None targets EVERY paired wearable",
                  file=sys.stderr)
            targets = dict(self._last_addr)
        targets = {w: a for w, a in targets.items() if a is not None}
        if not targets or self._sock is None:
            print(f"# forget: no known address for wid="
                  f"{wid if wid is not None else 'any'}", file=sys.stderr)
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
                return
            if len(data) < HDR.size:
                continue
            msg_type, version, wid = HDR.unpack_from(data, 0)
            if version not in ACCEPTED_VERSIONS:
                continue

            if msg_type == MSG_HELLO and len(data) == HELLO_SIZE:
                self._touch(wid, addr)
                _, _, _, nonce = HELLO.unpack(data)
                # Answer every board, assigned or not — an unassigned wearable
                # must still get on the network to be discoverable at all.
                self._sock.sendto(
                    WELCOME.pack(MSG_WELCOME, version, wid, nonce, self.pi_id), addr)
                unassigned = self.roster.lookup(wid) is None
                if self.verbose or unassigned:
                    print(f"# [{addr[0]}] HELLO wid={wid} v{version} -> WELCOME"
                          + (" (unassigned)" if unassigned else ""),
                          file=sys.stderr)
                continue

            if msg_type == MSG_ALERT:
                if len(data) != ALERT_SIZE:
                    self._log_bad_len(msg_type, len(data), ALERT_SIZE)
                    continue
                self._touch(wid, addr)
                self._handle_alert(wid, version, data, addr)
                continue

            if msg_type == MSG_IMU:
                self._touch(wid, addr)
                info = self.roster.lookup(wid)
                if len(data) == IMU_SIZE_V2:
                    self._enqueue_v2(wid, info, data)
                elif len(data) == IMU_SIZE:
                    if info is not None:
                        self._enqueue_v1(wid, info, data)
                else:
                    self._log_bad_len(msg_type, len(data),
                                      f"{IMU_SIZE} or {IMU_SIZE_V2}")
                continue

            self._log_bad_len(msg_type, len(data), "known type")

    def _touch(self, wid: int, addr) -> None:
        self._last_addr[wid] = addr
        self._last_seen[wid] = time.monotonic()
        self._rx_count[wid] = self._rx_count.get(wid, 0) + 1

    def _log_bad_len(self, msg_type: int, got, want) -> None:
        """Loudly. A silent length mismatch is exactly how the live path died."""
        self._bad_len[got] = self._bad_len.get(got, 0) + 1
        now = time.monotonic()
        if now - self._last_badlen_log >= 5.0:
            self._last_badlen_log = now
            print(f"# DROPPED msg_type={msg_type} len={got} (expected {want}) "
                  f"— firmware/receiver wire formats disagree. "
                  f"Counts by length: {self._bad_len}", file=sys.stderr)

    # -- timeline ------------------------------------------------------------ #
    def _t_s(self, wid: int, t_ms: int) -> float:
        t0 = self._t0_ms.get(wid)
        if t0 is None or t_ms + _REBOOT_JUMP_MS < t0:
            if t0 is not None:
                self._rebase_s[wid] = self._last_t_s.get(wid, 0.0) + _REBASE_GAP_S
            self._t0_ms[wid] = t0 = t_ms
        t_s = round((t_ms - t0) / 1000.0 + self._rebase_s.get(wid, 0.0), 3)
        self._last_t_s[wid] = t_s
        return t_s

    def _push_sample(self, sample: dict) -> None:
        node = sample["node"]
        with self._latest_lock:
            self._latest[node] = sample
        if node in self._focus:
            if len(self._focus_q) == self._focus_q.maxlen:
                self._dropped += 1
                now = time.monotonic()
                if now - self._last_drop_log >= 5.0:
                    print(f"# focus queue full — dropped {self._dropped} samples "
                          f"(no BLE subscriber draining?)", file=sys.stderr)
                    self._last_drop_log = now
            self._focus_q.append(sample)

    # -- impact alerts ------------------------------------------------------- #
    def _handle_alert(self, wid: int, version: int, data: bytes, addr) -> None:
        (seq, t_ms, peak_g, threshold_g,
         hx, hy, hz, gx, gy, gz, dur_ms, mode, xport) = \
            ALERT_BODY.unpack_from(data, HDR.size)

        # ACK FIRST, unconditionally, assigned or not. The wearable retransmits
        # every 600 ms up to 6 times until acked; acking only after a successful
        # record turns one store hiccup into six duplicate transmissions on air.
        try:
            self._sock.sendto(ALERT_ACK.pack(MSG_ALERT_ACK, version, wid, seq), addr)
        except OSError as exc:
            print(f"# alert ack failed for wid={wid} seq={seq}: {exc}",
                  file=sys.stderr)

        self._modes[wid] = mode
        info = self.roster.lookup(wid)
        event = {
            "seq": seq,
            "t_s": self._t_s(wid, t_ms),
            "peak_g": round(peak_g, 3),
            "threshold_g": round(threshold_g, 2),
            "hx_g": round(hx, 3), "hy_g": round(hy, 3), "hz_g": round(hz, 3),
            "gx_dps": round(gx, 2), "gy_dps": round(gy, 2), "gz_dps": round(gz, 2),
            "dur_ms": dur_ms,
            "mode": mode,
            "xport": xport,
            "wid": wid,
        }
        if info is not None:
            event.update({
                "athlete_id": info["athlete_id"],
                "athlete": info["athlete"],
                "team": info["team"],
                "position": info["position"],
                "is_head": info["is_head"],
                "node": info["node"],
            })
        else:
            # UNASSIGNED. Record it anyway. Losing a genuine head impact because
            # nobody had assigned the device yet is not an acceptable failure
            # mode; the app can attribute it retroactively.
            event.update({
                "athlete_id": None,
                "athlete": f"Unassigned wearable {wid}",
                "team": "",
                "position": "",
                "is_head": False,
                "node": f"unassigned/{wid}",
            })

        if self.store is not None:
            full = self.store.record(event)
            if full is None:
                if self.verbose:
                    print(f"# duplicate alert wid={wid} seq={seq} (retransmit)",
                          file=sys.stderr)
                return          # already recorded and already forwarded
            event = full

        self._impacts.append(event)
        where = (f"{event['athlete']} {event['position']}" if info
                 else f"UNASSIGNED wid={wid}")
        print(f"# IMPACT {where} seq={seq} peak={peak_g:.1f}g dur={dur_ms}ms "
              f"thr={threshold_g:.0f}g"
              + ("" if event.get("is_head") else "  [not a head sensor]"),
              file=sys.stderr)

    # -- telemetry ----------------------------------------------------------- #
    def _enqueue_v1(self, wid: int, info: dict, data: bytes) -> None:
        (_seq, t_ms,
         ax, ay, az, gx, gy, gz, hx, hy, hz, temp_c,
         hr, spo2, resp, hrv) = IMU_BODY_V1.unpack_from(data, HDR.size)

        sample = {"node": info["node"], "t_s": self._t_s(wid, t_ms)}
        for field, value in zip(protocol.LIVE_IMU_FIELDS,
                                (ax, ay, az, gx, gy, gz, hx, hy, hz, temp_c)):
            sample[field] = round(value, 6)
        for field, value in zip(protocol.LIVE_BIO_FIELDS, (hr, spo2, resp, hrv)):
            sample[field] = round(value, 3)
        self._push_sample(sample)

    def _enqueue_v2(self, wid: int, info: Optional[dict], data: bytes) -> None:
        (_seq, t_ms, impact_count,
         threshold_g, accumulator_g, all_time_peak_g, temp_c,
         hr, spo2, resp, hrv, mode) = IMU_BODY_V2.unpack_from(data, HDR.size)

        self._modes[wid] = mode
        self._peak_g[wid] = all_time_peak_g
        if info is None:
            return   # unassigned: counted for discovery, not streamed to the app

        sample = {"node": info["node"], "t_s": self._t_s(wid, t_ms)}
        for field, value in zip(protocol.LIVE_AGG_FIELDS,
                                (impact_count, threshold_g, accumulator_g,
                                 all_time_peak_g, temp_c)):
            sample[field] = round(float(value), 4)
        for field, value in zip(protocol.LIVE_BIO_FIELDS, (hr, spo2, resp, hrv)):
            sample[field] = round(value, 3)
        self._push_sample(sample)
        if self.verbose:
            print(f"# [{info['node']}] t_s={sample['t_s']:.3f} "
                  f"impacts={impact_count} peak={all_time_peak_g:.1f}g "
                  f"mode={mode}", file=sys.stderr)