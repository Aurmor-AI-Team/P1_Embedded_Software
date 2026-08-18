#!/usr/bin/env python3
"""
udp_imu_receiver.py — receive IMU packets from XIAO ESP32-C6 wearables over UDP,
and answer their connection-test handshake.

Run on the Raspberry Pi (or any machine on the same network):

    python3 udp_imu_receiver.py --port 5005 --pi-id 7

Then provision each wearable over BLE with this machine's IP, the same port,
a unique wearable ID, and (optionally) the expected Pi ID 7.

Message framing — all messages start with a 4-byte header:
    uint8  msg_type   (1=IMU, 2=HELLO, 3=WELCOME)
    uint8  version
    uint16 wearable_id

IMU     (68 bytes) = header + uint32 seq, uint32 t_ms, 14 floats
                     (ax,ay,az, gx,gy,gz, hx,hy,hz, temp_c, hr,spo2,resp,hrv)
HELLO   (8 bytes, wearable->Pi)  = header + uint32 nonce
WELCOME (12 bytes, Pi->wearable) = header + uint32 nonce + uint32 pi_id
"""
import argparse
import socket
import struct
import time

MSG_IMU, MSG_HELLO, MSG_WELCOME = 1, 2, 3
VERSION = 1

HDR      = struct.Struct("<BBH")        # msg_type, version, wearable_id  (4)
IMU_BODY = struct.Struct("<II14f")      # seq, t_ms, 10 IMU + 4 bio floats (64)
HELLO    = struct.Struct("<BBHI")       # header + nonce                  (8)
WELCOME  = struct.Struct("<BBHII")      # header + nonce + pi_id          (12)

IMU_SIZE   = HDR.size + IMU_BODY.size   # 52
HELLO_SIZE = HELLO.size                 # 8


def main():
    ap = argparse.ArgumentParser(description="ESP32-C6 IMU UDP receiver + handshake")
    ap.add_argument("--host", default="0.0.0.0", help="interface to bind (default: all)")
    ap.add_argument("--port", type=int, default=5005, help="UDP port (default: 5005)")
    ap.add_argument("--pi-id", type=int, default=1, help="this Pi's ID sent in WELCOME")
    ap.add_argument("--quiet", action="store_true", help="only print stats, not every packet")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    print(f"Listening on {args.host}:{args.port} | pi_id={args.pi_id}")

    # Per-wearable stats: wid -> dict(last_seq, dropped, received)
    stats = {}
    t0 = time.time()

    while True:
        data, addr = sock.recvfrom(256)
        if len(data) < HDR.size:
            continue
        msg_type, version, wid = HDR.unpack_from(data, 0)

        if msg_type == MSG_HELLO and len(data) == HELLO_SIZE:
            _, _, _, nonce = HELLO.unpack(data)
            reply = WELCOME.pack(MSG_WELCOME, VERSION, wid, nonce, args.pi_id)
            sock.sendto(reply, addr)
            print(f"[{addr[0]}] HELLO from wearable {wid} -> WELCOME (pi_id={args.pi_id})")
            continue

        if msg_type == MSG_IMU and len(data) == IMU_SIZE:
            (seq, t_ms,
             ax, ay, az, gx, gy, gz, hx, hy, hz, temp_c,
             _hr, _spo2, _resp, _hrv) = IMU_BODY.unpack_from(data, HDR.size)

            st = stats.setdefault(wid, {"last_seq": None, "dropped": 0, "received": 0})
            if st["last_seq"] is not None:
                gap = (seq - st["last_seq"]) & 0xFFFFFFFF
                if gap > 1:
                    st["dropped"] += gap - 1
            st["last_seq"] = seq
            st["received"] += 1

            if not args.quiet:
                h = (hx * hx + hy * hy + hz * hz) ** 0.5
                print(f"[{addr[0]}] wid={wid:>5} seq={seq:>8} t={t_ms:>9}ms | "
                      f"a=({ax:+.3f},{ay:+.3f},{az:+.3f})g | "
                      f"g=({gx:+.1f},{gy:+.1f},{gz:+.1f})dps | "
                      f"|h|={h:5.2f}g | {temp_c:4.1f}C")
            continue

        # Unknown / wrong-size packet — ignore.

        now = time.time()
        if now - t0 >= 5.0:
            for wid, st in stats.items():
                total = st["received"] + st["dropped"]
                rate = st["received"] / (now - t0)
                loss = (100.0 * st["dropped"] / total) if total else 0.0
                print(f"--- wid={wid}: {rate:6.1f} pkt/s | "
                      f"received={st['received']} dropped={st['dropped']} ({loss:.2f}%) ---")
                st["received"] = 0
                st["dropped"] = 0
            t0 = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
