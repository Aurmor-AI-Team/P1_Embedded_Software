#!/usr/bin/env python3
"""Pretend to be an ESP32-C6 wearable: send HEAD IMU packets over UDP.

Dev tool for exercising ble_sender.py's live mode without hardware:

    terminal A:  python3 ble_sender.py --source udp --stdout --no-ap
    terminal B:  python3 tools/fake_esp32_sender.py

Sends the HEAD mock CSV rows (falling back to a synthetic sine wave) as
52-byte IMU packets at the CSV cadence, HELLOs periodically, and prints any
WELCOME / FORGET packets the Pi sends back. Exits on FORGET, like the board.
"""
from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import time
from pathlib import Path

MSG_IMU, MSG_HELLO, MSG_WELCOME, MSG_FORGET = 1, 2, 3, 4
VERSION = 1
IMU = struct.Struct("<BBHII14f")   # 10 IMU + 4 bio (hr, spo2, resp, hrv)
HELLO = struct.Struct("<BBHI")
WELCOME = struct.Struct("<BBHII")
FORGET = struct.Struct("<BBHI")

IMU_FIELDS = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
              "hx_g", "hy_g", "hz_g", "imu_temp_c")
MOCK_DIR = Path(__file__).resolve().parents[2] / "mock-csv" / "10_squats_clean_biometric_data_simulation"
DEFAULT_CSV = MOCK_DIR / "HEAD_Head_main.csv"


def _col(path: Path, name: str):
    if not path.exists():
        return None
    with path.open(newline="") as handle:
        return [float(row[name]) for row in csv.DictReader(handle)]


def load_rows(path: Path):
    """Merge HEAD IMU with chest (HR/resp/HRV) and wrist (SpO2) into 14-value
    rows, matching what the firmware mock embeds."""
    if not path.exists():
        print(f"# {path} not found — using synthetic data", file=sys.stderr)
        return None
    with path.open(newline="") as handle:
        imu = [tuple(float(row[f]) for f in IMU_FIELDS) for row in csv.DictReader(handle)]
    hr = _col(MOCK_DIR / "WA_Chest.csv", "ecg_hr_bpm")
    spo2 = _col(MOCK_DIR / "WD_L_Wrist.csv", "ppg_spo2_pct")
    resp = _col(MOCK_DIR / "WA_Chest.csv", "resp_rate_bpm")
    hrv = _col(MOCK_DIR / "WA_Chest.csv", "ecg_rmssd_ms")
    rows = []
    for i, imu_row in enumerate(imu):
        bio = (hr[i] if hr else 92.0, spo2[i] if spo2 else 97.0,
               resp[i] if resp else 20.0, hrv[i] if hrv else 44.0)
        rows.append(imu_row + bio)
    return rows


def synth_row(i: int):
    phase = i * 0.1
    return (math.sin(phase), math.cos(phase), -1.0,
            10 * math.sin(phase), 0.0, 0.0,
            math.sin(phase), math.cos(phase), -1.0,
            25.0 + math.sin(phase / 10),
            92.0, 97.0, 20.0, 44.0)  # hr, spo2, resp, hrv


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--wid", type=int, default=1)
    ap.add_argument("--period-ms", type=int, default=255)
    ap.add_argument("--csv", default=str(DEFAULT_CSV))
    args = ap.parse_args()

    rows = load_rows(Path(args.csv))
    dest = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.0)
    sock.setblocking(False)

    print(f"# sending wid={args.wid} -> {dest} every {args.period_ms} ms "
          f"({'csv ' + str(len(rows)) + ' rows, looped' if rows else 'synthetic'})",
          file=sys.stderr)

    seq = 0
    t0 = time.monotonic()
    last_hello = 0.0
    while True:
        now = time.monotonic()
        if now - last_hello >= 2.0:
            sock.sendto(HELLO.pack(MSG_HELLO, VERSION, args.wid, seq), dest)
            last_hello = now

        values = rows[seq % len(rows)] if rows else synth_row(seq)
        t_ms = int((now - t0) * 1000)
        sock.sendto(IMU.pack(MSG_IMU, VERSION, args.wid, seq, t_ms, *values), dest)
        seq += 1

        try:
            data, addr = sock.recvfrom(64)
            if len(data) >= 4:
                msg_type = data[0]
                if msg_type == MSG_WELCOME and len(data) == WELCOME.size:
                    _, _, wid, nonce, pi_id = WELCOME.unpack(data)
                    print(f"# WELCOME from {addr[0]}: wid={wid} pi_id={pi_id}",
                          file=sys.stderr)
                elif msg_type == MSG_FORGET and len(data) == FORGET.size:
                    _, _, wid, pi_id = FORGET.unpack(data)
                    print(f"# FORGET from {addr[0]}: wid={wid} pi_id={pi_id} "
                          f"— exiting (as the board would)", file=sys.stderr)
                    return
        except BlockingIOError:
            pass

        time.sleep(args.period_ms / 1000.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped", file=sys.stderr)
