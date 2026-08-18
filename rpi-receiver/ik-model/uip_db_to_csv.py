"""Convert a UIP-DB sequence window to the mock-csv schema.

UIP-DB provides per-sensor world-frame accel (m/s², gravity-removed) and
3x3 rotation matrices. Mock-csv expects per-sensor body-frame accel (g,
gravity included), gyro (dps), and magnetometer (g) — same channels the
rpi-receiver actually emits. This script inverts UIP's preprocessing:

  glb_acc + R   →  body accel (g, with gravity)
  R history     →  gyro (omega = vee(R^T dR/dt), in dps)
  R + world mag →  body magnetometer reading (normalized)
  vuwb[:,i,head] → distance_m

Writes 6 CSVs (WD, WE, WF, WG, HEAD, WA) into --out-dir.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import torch

# UIP-DB sensor order. (lw, rw, lk, rk, head, root)
UIP_ORDER = ["lw", "rw", "lk", "rk", "head", "root"]
HEAD_IDX = UIP_ORDER.index("head")

# Mock-csv filename + node id + label + present_mask per slot.
SLOT_TO_CSV = {
    "lw":   ("WD_L_Wrist",     "WD",   "L_Wrist",   "0x03"),
    "rw":   ("WE_R_Wrist",     "WE",   "R_Wrist",   "0x03"),
    "lk":   ("WF_L_Knee",      "WF",   "L_Knee",    "0x21"),
    "rk":   ("WG_R_Knee",      "WG",   "R_Knee",    "0x21"),
    "head": ("HEAD_Head_main", "HEAD", "Head_main", "0x09"),
    "root": ("WA_Chest",       "WA",   "Chest",     "0x55"),  # pelvis -> chest slot
}
GRAVITY = 9.80665

# Synthetic magnetic-north vector in world frame, normalized.
# Mid-latitude inclination ~60 deg. Mock-csv mag values are unit-magnitude;
# we follow that convention.
_MAG_INCL_DEG = 60.0
WORLD_MAG = np.array([
    np.cos(np.radians(_MAG_INCL_DEG)),
    0.0,
    -np.sin(np.radians(_MAG_INCL_DEG)),
])
WORLD_MAG = WORLD_MAG / np.linalg.norm(WORLD_MAG)


def world_accel_to_body_g(acc_world: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Undo UIP's preprocessing: add gravity back, rotate world->body, /9.81."""
    a = acc_world.copy()
    a[:, 2] += GRAVITY                              # gravity points -Z; reading is +Z
    R_T = np.transpose(R, (0, 2, 1))
    a_body = np.einsum("nij,nj->ni", R_T, a)
    return a_body / GRAVITY


def gyro_from_rot(R: np.ndarray, dt: float) -> np.ndarray:
    """Body-frame angular velocity (dps) from a rotation-matrix time series."""
    n = R.shape[0]
    dR = np.empty_like(R)
    dR[1:-1] = (R[2:] - R[:-2]) / (2.0 * dt)
    dR[0]    = (R[1]  - R[0])   / dt
    dR[-1]   = (R[-1] - R[-2])  / dt
    R_T = np.transpose(R, (0, 2, 1))
    S = R_T @ dR
    omega = np.stack([S[:, 2, 1], S[:, 0, 2], S[:, 1, 0]], axis=1)
    return np.degrees(omega)


def mag_body(R: np.ndarray) -> np.ndarray:
    """Body-frame reading of WORLD_MAG (rotate world->body)."""
    R_T = np.transpose(R, (0, 2, 1))
    return np.einsum("nij,j->ni", R_T, WORLD_MAG)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--db", type=Path, default=Path("/tmp/uip_db_listing/test.pt"))
    ap.add_argument("--seq", type=int, default=9)
    ap.add_argument("--start", type=int, default=1686, help="first frame to convert")
    ap.add_argument("--frames", type=int, default=1200, help="frame count")
    ap.add_argument("--out-dir", type=Path, required=True)
    args = ap.parse_args()

    print(f"loading {args.db}")
    d = torch.load(str(args.db), map_location="cpu", weights_only=False)
    n_total = d["acc"][args.seq].shape[0]
    s = max(0, min(args.start, n_total - 1))
    e = min(s + args.frames, n_total)
    n = e - s
    print(f"seq {args.seq}: {n} frames from [{s}, {e}) → {args.out_dir}")

    acc_world = d["acc"][args.seq][s:e].numpy()                 # [N, 6, 3]
    ori = d["ori"][args.seq][s:e].numpy()                        # [N, 6, 3, 3]
    uwb = d["vuwb"][args.seq][s:e].numpy()                       # [N, 6, 6]

    dt = 1.0 / 60.0
    t_s = np.arange(n) * dt
    t0 = datetime(2025, 6, 1, 15, 0, 0)
    columns = ["timestamp_iso", "t_s", "round", "node", "label",
               "present_mask_hex", "version", "node_seq", "distance_m",
               "ax_g", "ay_g", "az_g",
               "gx_dps", "gy_dps", "gz_dps",
               "hx_g", "hy_g", "hz_g",
               "imu_temp_c"]

    args.out_dir.mkdir(parents=True, exist_ok=True)

    for slot_idx, slot in enumerate(UIP_ORDER):
        fname, node, label, mask = SLOT_TO_CSV[slot]
        R = ori[:, slot_idx]
        a_g  = world_accel_to_body_g(acc_world[:, slot_idx], R)
        g_dps = gyro_from_rot(R, dt)
        m_g  = mag_body(R)
        # distance_m: row of UWB matrix from this sensor to the head
        if slot == "head":
            dist = np.zeros(n)
        else:
            dist = uwb[:, slot_idx, HEAD_IDX]

        out_path = args.out_dir / f"{fname}.csv"
        with open(out_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(columns)
            for i in range(n):
                ts = (t0 + timedelta(seconds=float(t_s[i]))).isoformat(timespec="milliseconds")
                w.writerow([
                    ts, f"{t_s[i]:.3f}", i, node, label, mask, 2, i,
                    f"{dist[i]:.3f}",
                    f"{a_g[i,0]:+.3f}",  f"{a_g[i,1]:+.3f}",  f"{a_g[i,2]:+.3f}",
                    f"{g_dps[i,0]:+.1f}", f"{g_dps[i,1]:+.1f}", f"{g_dps[i,2]:+.1f}",
                    f"{m_g[i,0]:+.3f}",  f"{m_g[i,1]:+.3f}",  f"{m_g[i,2]:+.3f}",
                    "29.0",
                ])
        print(f"  wrote {out_path}  ({n} rows)")

    print()
    print("Sanity check — run the same pipeline on this converted dataset:")
    print(f"  python3 apply_ik.py --input {args.out_dir} --no-resample")


if __name__ == "__main__":
    main()
