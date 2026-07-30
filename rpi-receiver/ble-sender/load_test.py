#!/usr/bin/env python3
"""Squad load test: how many wearables can this receiver actually carry?

The software path is not the limit — measured at ~150k packets/s of parsing on
a laptop core and zero loss at 18,000 pps over loopback, roughly 9x the airtime
budget. What actually limits a squad is the WiFi link: how many stations the AP
will associate, and how much 2.4 GHz airtime 180 contending radios can share.

Neither of those can be measured in a container. Run this on the real hardware,
over the real AP, with the clients spread across at least two machines.

    # on the Pi (or wherever ble_sender would run)
    python3 loadtest.py server --port 5005

    # on one or more OTHER machines joined to the same AP
    python3 loadtest.py client --host 10.42.0.1 --port 5005 \\
        --devices 60 --first-wid 1 --rate 11 --seconds 60

    # a second client machine continues the wid range
    python3 loadtest.py client --host 10.42.0.1 --devices 60 --first-wid 61 ...

The server prints per-device loss derived from sequence gaps, so you can see
whether loss is uniform (airtime saturation) or concentrated on a few devices
(association trouble, or one radio in a bad spot).

Stdlib only. Does not import BLE or the receiver's own modules, so it runs on a
bare Pi image.
"""
from __future__ import annotations

import argparse
import signal
import socket
import struct
import sys
import threading
import time
from collections import defaultdict

HDR = struct.Struct("<BBH")
IMU_BODY_V2 = struct.Struct("<III4f4fB")
ALERT_BODY = struct.Struct("<II8fHBB")
HELLO = struct.Struct("<BBHI")
WELCOME = struct.Struct("<BBHII")
ALERT_ACK = struct.Struct("<BBHI")

MSG_IMU, MSG_HELLO, MSG_WELCOME = 1, 2, 3
MSG_ALERT, MSG_ALERT_ACK = 5, 6
VERSION_V2 = 2

IMU_SIZE = HDR.size + IMU_BODY_V2.size      # 49
ALERT_SIZE = HDR.size + ALERT_BODY.size     # 48


# --------------------------------------------------------------------------- #
# Server: stand in for the receiver and measure what arrives.
# --------------------------------------------------------------------------- #
def run_server(args) -> int:
    # systemd, `timeout`, and Ctrl-C should all print the summary rather than
    # discard a run you just spent a minute measuring.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # A big receive buffer separates "the network dropped it" from "we were too
    # slow to drain it" — without this you cannot tell the two apart.
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)
    print(f"# listening on 0.0.0.0:{args.port}, rcvbuf="
          f"{sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)} B", file=sys.stderr)

    first_seq, last_seq, got = {}, {}, defaultdict(int)
    alerts = defaultdict(int)
    hellos = defaultdict(int)
    bad_len = defaultdict(int)
    started = None
    last_report = time.monotonic()
    window_pkts = 0

    try:
        while True:
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                data = None
            now = time.monotonic()

            if data:
                if started is None:
                    started = now
                window_pkts += 1
                if len(data) < HDR.size:
                    continue
                msg_type, _ver, wid = HDR.unpack_from(data, 0)

                if msg_type == MSG_HELLO and len(data) == HELLO.size:
                    hellos[wid] += 1
                    _, _, _, nonce = HELLO.unpack(data)
                    sock.sendto(WELCOME.pack(MSG_WELCOME, VERSION_V2, wid,
                                             nonce, args.pi_id), addr)
                elif msg_type == MSG_IMU and len(data) == IMU_SIZE:
                    seq = IMU_BODY_V2.unpack_from(data, HDR.size)[0]
                    got[wid] += 1
                    first_seq.setdefault(wid, seq)
                    last_seq[wid] = seq
                elif msg_type == MSG_ALERT and len(data) == ALERT_SIZE:
                    seq = ALERT_BODY.unpack_from(data, HDR.size)[0]
                    alerts[wid] += 1
                    # Ack immediately, exactly like the real receiver — an
                    # unacked alert retransmits 6x and inflates the load.
                    sock.sendto(ALERT_ACK.pack(MSG_ALERT_ACK, VERSION_V2,
                                               wid, seq), addr)
                else:
                    bad_len[len(data)] += 1

            if now - last_report >= args.report:
                pps = window_pkts / (now - last_report)
                print(f"# {len(got):3} device(s) streaming | "
                      f"{pps:7,.0f} pps | {sum(alerts.values()):4} alert(s)",
                      file=sys.stderr)
                window_pkts = 0
                last_report = now
    except KeyboardInterrupt:
        pass

    if not got:
        print("\nno telemetry received.", file=sys.stderr)
        return 1

    elapsed = max((time.monotonic() - started), 1e-6)
    print(f"\n=== {elapsed:.1f} s, {len(got)} device(s) ===")
    total_exp = total_got = 0
    worst = []
    for wid in sorted(got):
        expected = last_seq[wid] - first_seq[wid] + 1
        received = got[wid]
        loss = 100.0 * (1 - received / expected) if expected else 0.0
        total_exp += expected
        total_got += received
        worst.append((loss, wid, received, expected))
    agg_loss = 100.0 * (1 - total_got / total_exp) if total_exp else 0.0
    print(f"aggregate: {total_got:,} / {total_exp:,} packets "
          f"({agg_loss:.2f}% loss), {total_got/elapsed:,.0f} pps sustained")
    print(f"alerts:    {sum(alerts.values())} from {len(alerts)} device(s)")
    if bad_len:
        print(f"unparsed lengths: {dict(bad_len)}")

    worst.sort(reverse=True)
    print("\nworst 10 devices by loss:")
    for loss, wid, received, expected in worst[:10]:
        print(f"  wid {wid:4}  {received:6,}/{expected:6,}  {loss:6.2f}%")

    # Uniform loss means the medium is saturated; concentrated loss usually
    # means association or RF trouble on specific radios.
    losses = [w[0] for w in worst]
    spread = max(losses) - min(losses)
    print(f"\nloss spread across devices: {spread:.1f} points")
    if agg_loss < 1:
        print("VERDICT: link is carrying this load.")
    elif spread > 20:
        print("VERDICT: loss is CONCENTRATED — suspect association limits, "
              "a struggling AP, or specific radios out of range. Check "
              "`iw dev wlan0 station dump | grep -c Station` on the AP.")
    else:
        print("VERDICT: loss is UNIFORM — the channel is saturated. Lower the "
              "per-device rate or split the squad across receivers.")
    return 0


# --------------------------------------------------------------------------- #
# Client: pretend to be N wearables.
# --------------------------------------------------------------------------- #
def run_client(args) -> int:
    dest = (args.host, args.port)
    stop = threading.Event()
    sent = defaultdict(int)
    welcomed = set()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    sock.settimeout(0.5)

    def listener():
        while not stop.is_set():
            try:
                data, _ = sock.recvfrom(256)
            except (socket.timeout, OSError):
                continue
            if len(data) >= HDR.size:
                mt, _v, wid = HDR.unpack_from(data, 0)
                if mt == MSG_WELCOME:
                    welcomed.add(wid)

    threading.Thread(target=listener, daemon=True).start()

    wids = list(range(args.first_wid, args.first_wid + args.devices))
    print(f"# simulating wid {wids[0]}..{wids[-1]} at {args.rate} Hz each "
          f"(~{len(wids) * args.rate:,} pps offered) -> {args.host}:{args.port}",
          file=sys.stderr)

    for wid in wids:
        sock.sendto(HELLO.pack(MSG_HELLO, VERSION_V2, wid, wid), dest)
        time.sleep(0.002)          # don't slam the AP with 180 simultaneous joins
    time.sleep(1.0)
    print(f"# {len(welcomed)}/{len(wids)} device(s) got a WELCOME", file=sys.stderr)

    period = 1.0 / args.rate
    t0 = time.perf_counter()
    deadline = t0 + args.seconds
    next_sweep = t0
    seq = 0
    next_hello = t0 + 2.0
    alerts_sent = 0

    try:
        while time.perf_counter() < deadline:
            now = time.perf_counter()
            t_ms = int((now - t0) * 1000) + 1000
            for wid in wids:
                pkt = HDR.pack(MSG_IMU, VERSION_V2, wid) + IMU_BODY_V2.pack(
                    seq, t_ms, 0, 20.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0, 0.0, 6)
                try:
                    sock.sendto(pkt, dest)
                    sent[wid] += 1
                except OSError:
                    pass

            # Occasional impact bursts: this is the traffic shape that matters,
            # since alerts retransmit until acked.
            if args.alert_every and seq and seq % args.alert_every == 0:
                for wid in wids[::max(1, len(wids) // 8)]:
                    a = HDR.pack(MSG_ALERT, VERSION_V2, wid) + ALERT_BODY.pack(
                        alerts_sent, t_ms, 45.0, 20.0, 1.0, 2.0, 44.0,
                        100.0, -50.0, 20.0, 120, 6, 2)
                    try:
                        sock.sendto(a, dest)
                    except OSError:
                        pass
                alerts_sent += 1

            if now >= next_hello:
                next_hello = now + 2.0
                for wid in wids:
                    try:
                        sock.sendto(HELLO.pack(MSG_HELLO, VERSION_V2, wid, seq), dest)
                    except OSError:
                        pass

            seq += 1
            next_sweep += period
            delay = next_sweep - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -1.0:
                # We cannot keep up locally; the measurement would blame the
                # network for a client-side shortfall.
                print("# WARNING: client is behind schedule — reduce --devices "
                      "or --rate, or add another client machine", file=sys.stderr)
                next_sweep = time.perf_counter()
    except KeyboardInterrupt:
        pass

    stop.set()
    total = sum(sent.values())
    elapsed = time.perf_counter() - t0
    print(f"\nsent {total:,} telemetry packets from {len(wids)} device(s) "
          f"in {elapsed:.1f}s = {total/elapsed:,.0f} pps offered")
    print(f"alert bursts: {alerts_sent}")
    print("compare against the server's received count for real loss.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    s = sub.add_parser("server", help="receive and measure")
    s.add_argument("--port", type=int, default=5005)
    s.add_argument("--pi-id", type=int, default=42)
    s.add_argument("--report", type=float, default=5.0,
                   help="seconds between progress lines")

    c = sub.add_parser("client", help="simulate wearables")
    c.add_argument("--host", required=True)
    c.add_argument("--port", type=int, default=5005)
    c.add_argument("--devices", type=int, default=30)
    c.add_argument("--first-wid", type=int, default=1,
                   help="so several client machines can cover one wid range")
    c.add_argument("--rate", type=int, default=11,
                   help="telemetry Hz per device (the governor's value)")
    c.add_argument("--seconds", type=float, default=60.0)
    c.add_argument("--alert-every", type=int, default=0,
                   help="fire an impact burst every N sweeps (0 = never)")

    args = p.parse_args(argv)
    return run_server(args) if args.mode == "server" else run_client(args)


if __name__ == "__main__":
    sys.exit(main())