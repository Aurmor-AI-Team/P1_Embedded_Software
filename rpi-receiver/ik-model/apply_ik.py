"""Apply UltraInertialPoser (UIP) to mock-csv biometric streams.

Reads the per-node CSVs in a mock-csv dataset directory, fuses 9-DoF IMU
samples into world-frame orientation, builds the tensors UIP expects, then
calls UIP's predict() and writes per-frame joint rotations + root translation
in time order.

Usage:
    python apply_ik.py                          # process both squats + pushups
    python apply_ik.py --input "<dataset_dir>"  # process one dataset
    python apply_ik.py --dry-run                # build tensors, skip the model
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

GRAVITY = 9.80665

# UIP sensor order from modules/dataset/preprocess.py (imu_mask comment):
#   lw, rw, lk, rk, head, root
UIP_SENSOR_ORDER = ["lw", "rw", "lk", "rk", "head", "root"]
HEAD_INDEX = UIP_SENSOR_ORDER.index("head")

# Map UIP slots to the CSV file basenames in mock-csv/<dataset>/.
NODE_FILE_FOR_SLOT = {
    "lw":   "WD_L_Wrist.csv",
    "rw":   "WE_R_Wrist.csv",
    "lk":   "WF_L_Knee.csv",
    "rk":   "WG_R_Knee.csv",
    "head": "HEAD_Head_main.csv",
    "root": "WA_Chest.csv",
}

DEFAULT_DATASETS = [
    "10 squats_biometric_data_simulation",
    "10_pushups_biometric_data_simulation",
]

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent              # rpi-receiver/
MOCK_CSV_DIR = REPO_ROOT / "mock-csv"


# ---------------------------------------------------------------------------
# CSV loading + alignment
# ---------------------------------------------------------------------------

def discover_inputs(input_arg: Optional[str]) -> List[Path]:
    if input_arg:
        p = Path(input_arg).expanduser().resolve()
        if not p.is_dir():
            raise FileNotFoundError(f"--input not a directory: {p}")
        return [p]
    return [MOCK_CSV_DIR / name for name in DEFAULT_DATASETS]


@dataclass
class NodeFrame:
    slot: str
    t_s: np.ndarray          # [N]
    acc_g: np.ndarray        # [N, 3]
    gyro_dps: np.ndarray     # [N, 3]
    mag_g: np.ndarray        # [N, 3]
    distance_m: np.ndarray   # [N]


def _read_one_node(csv_path: Path, slot: str) -> NodeFrame:
    df = pd.read_csv(csv_path)
    needed = ["t_s", "ax_g", "ay_g", "az_g",
              "gx_dps", "gy_dps", "gz_dps",
              "hx_g", "hy_g", "hz_g"]
    for col in needed:
        if col not in df.columns:
            raise ValueError(f"{csv_path.name}: missing column {col!r}")
    dist = (df["distance_m"].to_numpy(np.float64)
            if "distance_m" in df.columns
            else np.zeros(len(df), dtype=np.float64))
    return NodeFrame(
        slot=slot,
        t_s=df["t_s"].to_numpy(np.float64),
        acc_g=df[["ax_g", "ay_g", "az_g"]].to_numpy(np.float64),
        gyro_dps=df[["gx_dps", "gy_dps", "gz_dps"]].to_numpy(np.float64),
        mag_g=df[["hx_g", "hy_g", "hz_g"]].to_numpy(np.float64),
        distance_m=dist,
    )


def load_node_csvs(dataset_dir: Path) -> Dict[str, NodeFrame]:
    nodes = {}
    for slot, fname in NODE_FILE_FOR_SLOT.items():
        path = dataset_dir / fname
        if not path.is_file():
            raise FileNotFoundError(f"missing CSV {path}")
        nodes[slot] = _read_one_node(path, slot)
    return nodes


def align_on_time(nodes: Dict[str, NodeFrame]) -> Tuple[np.ndarray, Dict[str, NodeFrame]]:
    """Intersect the per-node time vectors and resample each node to that grid.

    The mock CSVs share a common round clock, so this is essentially a sanity
    pass — but we tolerate small jitter by linear-interpolating onto the
    intersection grid (the densest grid common to all nodes).
    """
    t_min = max(n.t_s[0] for n in nodes.values())
    t_max = min(n.t_s[-1] for n in nodes.values())
    base = next(iter(nodes.values())).t_s
    t = base[(base >= t_min) & (base <= t_max)]

    out = {}
    for slot, n in nodes.items():
        out[slot] = NodeFrame(
            slot=slot,
            t_s=t,
            acc_g=_interp_2d(t, n.t_s, n.acc_g),
            gyro_dps=_interp_2d(t, n.t_s, n.gyro_dps),
            mag_g=_interp_2d(t, n.t_s, n.mag_g),
            distance_m=np.interp(t, n.t_s, n.distance_m),
        )
    return t, out


def _interp_2d(t_new: np.ndarray, t_old: np.ndarray, y: np.ndarray) -> np.ndarray:
    return np.stack([np.interp(t_new, t_old, y[:, k]) for k in range(y.shape[1])], axis=1)


def resample(t: np.ndarray, nodes: Dict[str, NodeFrame], target_hz: float
             ) -> Tuple[np.ndarray, Dict[str, NodeFrame]]:
    if target_hz <= 0:
        return t, nodes
    t0, t1 = t[0], t[-1]
    n_new = max(2, int(round((t1 - t0) * target_hz)) + 1)
    t_new = np.linspace(t0, t1, n_new)
    out = {}
    for slot, n in nodes.items():
        out[slot] = NodeFrame(
            slot=slot,
            t_s=t_new,
            acc_g=_interp_2d(t_new, n.t_s, n.acc_g),
            gyro_dps=_interp_2d(t_new, n.t_s, n.gyro_dps),
            mag_g=_interp_2d(t_new, n.t_s, n.mag_g),
            distance_m=np.interp(t_new, n.t_s, n.distance_m),
        )
    return t_new, out


# ---------------------------------------------------------------------------
# Orientation fusion (Madgwick MARG)
# ---------------------------------------------------------------------------

def fuse_orientation(acc_g: np.ndarray, gyro_dps: np.ndarray, mag_g: np.ndarray,
                     dt: float, beta: float = 0.041) -> np.ndarray:
    """Madgwick MARG filter. Returns quaternions [N, 4] in (w, x, y, z) order.

    Pure-numpy fallback so the script has no hard dependency on `ahrs`.
    """
    n = acc_g.shape[0]
    q = np.zeros((n, 4), dtype=np.float64)
    q[0] = (1.0, 0.0, 0.0, 0.0)

    gyro_rad = np.deg2rad(gyro_dps)
    # Acc/mag direction only; magnitudes drop out after normalization.
    for i in range(1, n):
        q[i] = _madgwick_step(q[i - 1], acc_g[i], gyro_rad[i], mag_g[i], dt, beta)
    return q


def _madgwick_step(q, a, g, m, dt, beta):
    qw, qx, qy, qz = q
    ax, ay, az = a
    gx, gy, gz = g
    mx, my, mz = m

    norm_a = math.sqrt(ax * ax + ay * ay + az * az)
    norm_m = math.sqrt(mx * mx + my * my + mz * mz)
    if norm_a == 0.0 or norm_m == 0.0:
        # Fall back to gyro-only integration.
        return _normalize_q(_q_integrate(q, g, dt))
    ax, ay, az = ax / norm_a, ay / norm_a, az / norm_a
    mx, my, mz = mx / norm_m, my / norm_m, mz / norm_m

    # Reference magnetic field in earth frame.
    hx = 2 * mx * (0.5 - qy * qy - qz * qz) + 2 * my * (qx * qy - qw * qz) + 2 * mz * (qx * qz + qw * qy)
    hy = 2 * mx * (qx * qy + qw * qz) + 2 * my * (0.5 - qx * qx - qz * qz) + 2 * mz * (qy * qz - qw * qx)
    bx = math.sqrt(hx * hx + hy * hy)
    bz = 2 * mx * (qx * qz - qw * qy) + 2 * my * (qy * qz + qw * qx) + 2 * mz * (0.5 - qx * qx - qy * qy)

    # Gradient descent corrective step.
    f1 = 2 * (qx * qz - qw * qy) - ax
    f2 = 2 * (qw * qx + qy * qz) - ay
    f3 = 2 * (0.5 - qx * qx - qy * qy) - az
    f4 = 2 * bx * (0.5 - qy * qy - qz * qz) + 2 * bz * (qx * qz - qw * qy) - mx
    f5 = 2 * bx * (qx * qy - qw * qz) + 2 * bz * (qw * qx + qy * qz) - my
    f6 = 2 * bx * (qw * qy + qx * qz) + 2 * bz * (0.5 - qx * qx - qy * qy) - mz

    j11, j12, j13, j14 = -2 * qy, 2 * qz, -2 * qw, 2 * qx
    j21, j22, j23, j24 = 2 * qx, 2 * qw, 2 * qz, 2 * qy
    j31, j32 = 0.0, -4 * qx
    j33, j34 = -4 * qy, 0.0
    j41 = -2 * bz * qy
    j42 = 2 * bz * qz
    j43 = -4 * bx * qy - 2 * bz * qw
    j44 = -4 * bx * qz + 2 * bz * qx
    j51 = -2 * bx * qz + 2 * bz * qx
    j52 = 2 * bx * qy + 2 * bz * qw
    j53 = 2 * bx * qx + 2 * bz * qz
    j54 = -2 * bx * qw + 2 * bz * qy
    j61 = 2 * bx * qy
    j62 = 2 * bx * qz - 4 * bz * qx
    j63 = 2 * bx * qw - 4 * bz * qy
    j64 = 2 * bx * qx

    sw = j11 * f1 + j21 * f2 + j31 * f3 + j41 * f4 + j51 * f5 + j61 * f6
    sx = j12 * f1 + j22 * f2 + j32 * f3 + j42 * f4 + j52 * f5 + j62 * f6
    sy = j13 * f1 + j23 * f2 + j33 * f3 + j43 * f4 + j53 * f5 + j63 * f6
    sz = j14 * f1 + j24 * f2 + j34 * f3 + j44 * f4 + j54 * f5 + j64 * f6
    sw, sx, sy, sz = _normalize_q((sw, sx, sy, sz))

    # Quaternion derivative from gyro, minus gradient step.
    dqw = 0.5 * (-qx * gx - qy * gy - qz * gz) - beta * sw
    dqx = 0.5 * (qw * gx + qy * gz - qz * gy) - beta * sx
    dqy = 0.5 * (qw * gy - qx * gz + qz * gx) - beta * sy
    dqz = 0.5 * (qw * gz + qx * gy - qy * gx) - beta * sz

    return _normalize_q((qw + dqw * dt, qx + dqx * dt, qy + dqy * dt, qz + dqz * dt))


def _q_integrate(q, g, dt):
    qw, qx, qy, qz = q
    gx, gy, gz = g
    dqw = 0.5 * (-qx * gx - qy * gy - qz * gz)
    dqx = 0.5 * (qw * gx + qy * gz - qz * gy)
    dqy = 0.5 * (qw * gy - qx * gz + qz * gx)
    dqz = 0.5 * (qw * gz + qx * gy - qy * gx)
    return (qw + dqw * dt, qx + dqx * dt, qy + dqy * dt, qz + dqz * dt)


def _normalize_q(q):
    qw, qx, qy, qz = q
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (qw / n, qx / n, qy / n, qz / n)


def gravity_only_rotmat(acc_g: np.ndarray) -> np.ndarray:
    """Per-frame orientation from gravity alone (yaw fixed at 0).

    Rotation that maps the body axis pointing "up" (= -accel_unit) to world +Z.
    Pitch/roll come from the accel; yaw is undetermined and left zero.
    """
    n = acc_g.shape[0]
    norms = np.linalg.norm(acc_g, axis=1, keepdims=True)
    body_up = -acc_g / np.maximum(norms, 1e-9)         # [N, 3]
    world_up = np.array([0.0, 0.0, 1.0])
    v = np.cross(body_up, world_up)                    # rotation axis * sin(theta)
    s = np.linalg.norm(v, axis=1)                      # sin(theta)
    c = body_up @ world_up                             # cos(theta)

    R = np.tile(np.eye(3), (n, 1, 1))
    K = np.zeros((n, 3, 3))
    K[:, 0, 1] = -v[:, 2]; K[:, 0, 2] = +v[:, 1]
    K[:, 1, 0] = +v[:, 2]; K[:, 1, 2] = -v[:, 0]
    K[:, 2, 0] = -v[:, 1]; K[:, 2, 1] = +v[:, 0]

    safe = s > 1e-6
    factor = np.zeros(n)
    factor[safe] = (1 - c[safe]) / (s[safe] ** 2)
    R = R + K + factor[:, None, None] * (K @ K)

    # If body_up is anti-parallel to world_up (upside down), rotate 180° about X.
    flip = c < -0.9999
    if flip.any():
        R180 = np.diag([1.0, -1.0, -1.0])
        R[flip] = R180
    return R


def identity_rotmat(n_frames: int) -> np.ndarray:
    return np.tile(np.eye(3), (n_frames, 1, 1))


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """q: [N, 4] (w, x, y, z) → [N, 3, 3]."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = q.shape[0]
    R = np.empty((n, 3, 3), dtype=np.float64)
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


# ---------------------------------------------------------------------------
# Tensor assembly
# ---------------------------------------------------------------------------

def to_world_acc(acc_g: np.ndarray, R: np.ndarray) -> np.ndarray:
    """acc_g [N, 3] in body frame (g) → world-frame, gravity-removed (m/s^2).

    UIP's predict() expects the linear acceleration the body would experience
    sans gravity, in the world frame.
    """
    acc_ms2 = acc_g * GRAVITY                       # [N, 3]
    acc_world = np.einsum("nij,nj->ni", R, acc_ms2)  # rotate body→world
    acc_world[:, 2] -= GRAVITY                      # remove gravity along +Z
    return acc_world


def build_glb_uwb(head_dists_per_slot: Dict[str, np.ndarray]) -> np.ndarray:
    """Build [N, 6, 6] symmetric UWB distance matrix.

    CSV only gives each node's distance to the head, so we fill the head
    row/column and leave the other off-diagonals at 0.
    """
    n = len(next(iter(head_dists_per_slot.values())))
    M = np.zeros((n, 6, 6), dtype=np.float64)
    for slot, d in head_dists_per_slot.items():
        i = UIP_SENSOR_ORDER.index(slot)
        if i == HEAD_INDEX:
            continue
        M[:, i, HEAD_INDEX] = d
        M[:, HEAD_INDEX, i] = d
    return M


def assemble_tensors(t: np.ndarray, nodes: Dict[str, NodeFrame],
                     orientation: str = "madgwick"
                     ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (glb_acc [N,6,3], glb_rot [N,6,3,3], glb_uwb [N,6,6])."""
    n = len(t)
    if n < 2:
        raise ValueError("need at least 2 samples to estimate dt")
    dt = float(np.median(np.diff(t)))

    glb_acc = np.zeros((n, 6, 3), dtype=np.float64)
    glb_rot = np.zeros((n, 6, 3, 3), dtype=np.float64)
    head_dists = {}

    for slot, nf in nodes.items():
        i = UIP_SENSOR_ORDER.index(slot)
        if orientation == "madgwick":
            q = fuse_orientation(nf.acc_g, nf.gyro_dps, nf.mag_g, dt)
            R = quat_to_rotmat(q)
        elif orientation == "gravity-only":
            R = gravity_only_rotmat(nf.acc_g)
        elif orientation == "identity":
            R = identity_rotmat(n)
        else:
            raise ValueError(f"unknown orientation mode: {orientation}")
        glb_rot[:, i] = R
        glb_acc[:, i] = to_world_acc(nf.acc_g, R)
        head_dists[slot] = nf.distance_m

    glb_uwb = build_glb_uwb(head_dists)
    return glb_acc, glb_rot, glb_uwb


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def load_uip_model(uip_repo: Path, ckpt: Path, smpl_dir: Path, device: str):
    """Import UIP from a local clone and load the pretrained checkpoint."""
    try:
        import torch  # noqa: F401
    except ImportError as e:
        raise RuntimeError(
            "PyTorch is required. See ik-model/README.md for setup steps."
        ) from e

    if not uip_repo.is_dir():
        raise FileNotFoundError(
            f"UIP repo not found at {uip_repo}.\n"
            f"  git clone https://github.com/eth-siplab/UltraInertialPoser {uip_repo}"
        )
    if not ckpt.is_file():
        raise FileNotFoundError(
            f"Checkpoint not found at {ckpt}.\n"
            f"  Download from the UIP Google Drive linked in their README."
        )
    if not smpl_dir.is_dir():
        raise FileNotFoundError(
            f"SMPL data dir not found at {smpl_dir}.\n"
            f"  Place basicmodel_m_lbs_10_207_0_v1.0.0.pkl under {smpl_dir}."
        )

    sys.path.insert(0, str(uip_repo))
    # pybullet has no arm64-mac wheel and its source build fails. It is only
    # used by modules/utils.py::set_pose(), which predict() never calls. Stub it.
    if "pybullet" not in sys.modules:
        import types
        sys.modules["pybullet"] = types.ModuleType("pybullet")
    # chumpy (used to load the SMPL pkl) imports deprecated numpy aliases.
    # Re-add them as builtins so chumpy can import under numpy >= 1.20.
    for _name in ("bool", "int", "float", "complex", "object", "str"):
        if not hasattr(np, _name):
            setattr(np, _name, getattr(__builtins__, _name) if isinstance(__builtins__, type(sys)) else __builtins__[_name])
    if not hasattr(np, "unicode"):
        np.unicode = str
    try:
        uip_module = importlib.import_module("modules.model.uip")
        UIP = getattr(uip_module, "UIP")
    except Exception as e:
        raise RuntimeError(
            "Failed to import UIP. Make sure the conda env from "
            "UltraInertialPoser/environment.yml is active and rbdl is installed."
        ) from e

    args_path = uip_repo / "config" / "model_args.json"
    if args_path.is_file():
        with open(args_path) as fh:
            model_args_dict = json.load(fh)
    else:
        model_args_dict = {}
    model_args_dict["device"] = device  # override; checkpoint trained on cuda
    model_args = argparse.Namespace(**model_args_dict)

    import torch
    # UIP's config has SMPL/urdf paths as strings relative to its repo root,
    # so we briefly chdir into the repo to instantiate the network.
    prev_cwd = os.getcwd()
    os.chdir(str(uip_repo))
    try:
        net = UIP(model_args)
        state = torch.load(str(ckpt), map_location=device)
        if isinstance(state, dict):
            for k in ("net", "model", "state_dict"):
                if k in state and isinstance(state[k], dict):
                    state = state[k]
                    break
        net.load_state_dict(state)
        net.eval()
        net.to(device)
    finally:
        os.chdir(prev_cwd)
    return net


def run_inference(net, glb_acc, glb_rot, glb_uwb, device: str):
    import torch
    acc_t = torch.as_tensor(glb_acc, dtype=torch.float32, device=device)
    rot_t = torch.as_tensor(glb_rot, dtype=torch.float32, device=device)
    uwb_t = torch.as_tensor(glb_uwb, dtype=torch.float32, device=device)
    init_pose = torch.eye(3, dtype=torch.float32, device=device).expand(1, 24, 3, 3).contiguous()
    offset = torch.zeros(6, 3, dtype=torch.float32, device=device)

    with torch.no_grad():
        out = net.predict(acc_t, rot_t, init_pose, glb_uwb=uwb_t, offset=offset)
    pose_p, tran_p = out[0], out[1]
    return pose_p.cpu().numpy(), tran_p.cpu().numpy()


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def stream_results(t_s: np.ndarray, pose_p: np.ndarray, tran_p: np.ndarray,
                   out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    npz_path = out_dir / "pose_seq.npz"
    np.savez_compressed(npz_path, t_s=t_s, pose_rotmat=pose_p, tran=tran_p)

    csv_path = out_dir / "pose_seq.csv"
    n_joints = pose_p.shape[1]
    cols = ["t_s", "tran_x", "tran_y", "tran_z"]
    cols += [f"j{j}_r{a}{b}" for j in range(n_joints) for a in range(3) for b in range(3)]

    flat = pose_p.reshape(pose_p.shape[0], -1)
    rows = np.concatenate([t_s[:, None], tran_p, flat], axis=1)

    last_print = -1.0
    with open(csv_path, "w") as fh:
        fh.write(",".join(cols) + "\n")
        for i in range(rows.shape[0]):
            ti = float(t_s[i])
            fh.write(",".join(f"{v:.6f}" for v in rows[i]) + "\n")
            if ti - last_print >= 1.0:
                print(f"  t={ti:6.2f}s  tran=({tran_p[i, 0]:+.3f},"
                      f" {tran_p[i, 1]:+.3f}, {tran_p[i, 2]:+.3f})")
                last_print = ti

    print(f"  wrote {npz_path}")
    print(f"  wrote {csv_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def slugify(name: str) -> str:
    return name.replace(" ", "_").lower()


def process_dataset(dataset_dir: Path, args, net) -> None:
    print(f"[{dataset_dir.name}] loading 6 IMU CSVs")
    nodes = load_node_csvs(dataset_dir)
    t, nodes = align_on_time(nodes)
    src_hz = 1.0 / float(np.median(np.diff(t)))
    print(f"[{dataset_dir.name}] {len(t)} frames @ {src_hz:.2f} Hz "
          f"(t: {t[0]:.2f} → {t[-1]:.2f} s)")

    if not args.no_resample:
        t, nodes = resample(t, nodes, args.target_hz)
        print(f"[{dataset_dir.name}] resampled to {len(t)} frames @ {args.target_hz:.2f} Hz")

    print(f"[{dataset_dir.name}] assembling tensors (orientation={args.orientation})")
    t0 = time.time()
    glb_acc, glb_rot, glb_uwb = assemble_tensors(t, nodes, orientation=args.orientation)
    print(f"  glb_acc {glb_acc.shape}  glb_rot {glb_rot.shape}  glb_uwb {glb_uwb.shape}"
          f"  ({time.time() - t0:.2f}s)")

    if args.dry_run or net is None:
        print(f"[{dataset_dir.name}] dry run — skipping model inference")
        return

    print(f"[{dataset_dir.name}] running UIP.predict()")
    t0 = time.time()
    pose_p, tran_p = run_inference(net, glb_acc, glb_rot, glb_uwb, args.device)
    print(f"  pose {pose_p.shape}  tran {tran_p.shape}  ({time.time() - t0:.2f}s)")

    out_dir = Path(args.out_dir) / slugify(dataset_dir.name) / args.orientation
    stream_results(t, pose_p, tran_p, out_dir)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input", type=str, default=None,
                    help="Single dataset dir. Default: process both squats + pushups.")
    ap.add_argument("--uip-repo", type=Path, default=SCRIPT_DIR / "UltraInertialPoser")
    ap.add_argument("--ckpt", type=Path, default=SCRIPT_DIR / "weights" / "uip.pt")
    ap.add_argument("--smpl-dir", type=Path, default=SCRIPT_DIR / "data")
    ap.add_argument("--out-dir", type=Path, default=SCRIPT_DIR / "results")
    ap.add_argument("--target-hz", type=float, default=60.0)
    ap.add_argument("--no-resample", action="store_true")
    ap.add_argument("--device", choices=["cpu", "cuda", "mps"], default="cpu")
    ap.add_argument("--orientation",
                    choices=["madgwick", "gravity-only", "identity"],
                    default="madgwick",
                    help="How to derive per-IMU world-frame rotations.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Build tensors and exit; do not load or run the model.")
    args = ap.parse_args()

    datasets = discover_inputs(args.input)

    net = None
    if not args.dry_run:
        net = load_uip_model(args.uip_repo, args.ckpt, args.smpl_dir, args.device)

    for ds in datasets:
        process_dataset(ds, args, net)


if __name__ == "__main__":
    main()
