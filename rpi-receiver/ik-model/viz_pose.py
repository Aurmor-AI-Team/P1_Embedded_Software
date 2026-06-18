"""Animate the 24-joint SMPL skeleton from an apply_ik.py NPZ.

Usage:
    python viz_pose.py results/10_squats_biometric_data_simulation/pose_seq.npz
    python viz_pose.py <npz> --save out.mp4 --fps 30 --stride 2 --no-tran

The pose comes from UIP as per-frame [24, 3, 3] rotation matrices in *local*
joint frames. We walk the SMPL kinematic tree to get world-space joint
positions, then draw bones with matplotlib's 3D animation.
"""

from __future__ import annotations

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np

# Standard SMPL 24-joint parent table.
SMPL_PARENTS = np.array([
    -1,  0,  0,  0,  1,  2,  3,  4,  5,  6,  7,  8,
     9,  9,  9, 12, 13, 14, 16, 17, 18, 19, 20, 21,
], dtype=np.int32)


def _shim_numpy_for_chumpy():
    """SMPL .pkl files load via chumpy, which uses deprecated np aliases."""
    for name in ("bool", "int", "float", "complex", "object", "str"):
        if not hasattr(np, name):
            setattr(np, name, __builtins__[name] if isinstance(__builtins__, dict)
                    else getattr(__builtins__, name))
    if not hasattr(np, "unicode"):
        np.unicode = str


def load_rest_joints(smpl_pkl: Path) -> np.ndarray:
    """Return [24, 3] rest-pose joint positions in SMPL canonical frame."""
    _shim_numpy_for_chumpy()
    with open(smpl_pkl, "rb") as fh:
        data = pickle.load(fh, encoding="latin1")
    v_template = np.asarray(data["v_template"], dtype=np.float64)   # [6890, 3]
    J_regressor = data["J_regressor"]                                # sparse or dense
    if hasattr(J_regressor, "toarray"):
        J_regressor = J_regressor.toarray()
    return np.asarray(J_regressor, dtype=np.float64) @ v_template     # [24, 3]


def forward_kinematics(pose_rotmat: np.ndarray, tran: np.ndarray,
                       J_rest: np.ndarray) -> np.ndarray:
    """Walk the kinematic tree; return per-frame world joint positions [N, 24, 3]."""
    n_frames = pose_rotmat.shape[0]
    out = np.zeros((n_frames, 24, 3), dtype=np.float64)
    R_world = np.empty((24, 3, 3), dtype=np.float64)

    for f in range(n_frames):
        R_local = pose_rotmat[f]                  # [24, 3, 3]
        # Root.
        R_world[0] = R_local[0]
        out[f, 0] = tran[f]
        for j in range(1, 24):
            p = SMPL_PARENTS[j]
            R_world[j] = R_world[p] @ R_local[j]
            offset = J_rest[j] - J_rest[p]
            out[f, j] = out[f, p] + R_world[p] @ offset
    return out


def animate(joints_world: np.ndarray, fps: float, save: str | None,
            no_tran: bool) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    if no_tran:
        joints_world = joints_world - joints_world[:, [0], :]

    # Set up bones from the parent table.
    bones = [(j, SMPL_PARENTS[j]) for j in range(1, 24)]

    # Centered, square axes.
    mean = joints_world.reshape(-1, 3).mean(axis=0)
    radius = float(np.abs(joints_world - mean).max()) * 1.05
    if radius < 1e-3:
        radius = 1.0

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")

    # Initial frame.
    pts = joints_world[0]
    scat = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=20, c="tab:blue")
    lines = []
    for (j, p) in bones:
        line, = ax.plot([pts[j, 0], pts[p, 0]],
                        [pts[j, 1], pts[p, 1]],
                        [pts[j, 2], pts[p, 2]], c="black", lw=1.5)
        lines.append(line)
    title = ax.set_title("frame 0")

    def set_limits(center):
        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)

    set_limits(mean)

    def update(i):
        pts = joints_world[i]
        scat._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
        for line, (j, p) in zip(lines, bones):
            line.set_data_3d([pts[j, 0], pts[p, 0]],
                             [pts[j, 1], pts[p, 1]],
                             [pts[j, 2], pts[p, 2]])
        title.set_text(f"frame {i}  /  {joints_world.shape[0] - 1}")
        return [scat, *lines, title]

    anim = FuncAnimation(fig, update, frames=joints_world.shape[0],
                         interval=1000.0 / fps, blit=False)

    if save:
        try:
            anim.save(save, fps=fps, dpi=120)
            print(f"saved {save}")
        except Exception as e:
            print(f"save failed ({e}); falling back to interactive window")
            plt.show()
    else:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("npz", type=Path, help="pose_seq.npz produced by apply_ik.py")
    ap.add_argument("--smpl", type=Path,
                    default=Path(__file__).resolve().parent / "data"
                    / "basicmodel_m_lbs_10_207_0_v1.0.0.pkl")
    ap.add_argument("--save", type=str, default=None,
                    help="optional output (.mp4 / .gif); otherwise show window")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--stride", type=int, default=2,
                    help="render every Nth frame (default 2 → 30 fps from 60 Hz data)")
    ap.add_argument("--no-tran", action="store_true",
                    help="center on pelvis; ignore root translation")
    args = ap.parse_args()

    if not args.npz.is_file():
        sys.exit(f"NPZ not found: {args.npz}")
    if not args.smpl.is_file():
        sys.exit(f"SMPL pkl not found: {args.smpl}")

    print(f"loading {args.npz}")
    data = np.load(args.npz)
    pose_rotmat = data["pose_rotmat"][::args.stride]
    tran = data["tran"][::args.stride]
    t_s = data["t_s"][::args.stride]
    print(f"  frames {pose_rotmat.shape[0]}, t {t_s[0]:.2f} → {t_s[-1]:.2f} s")

    print("loading SMPL rest-pose joints")
    J_rest = load_rest_joints(args.smpl)

    print("forward kinematics")
    joints_world = forward_kinematics(pose_rotmat, tran, J_rest)

    print(f"animating @ {args.fps} fps")
    animate(joints_world, args.fps, args.save, args.no_tran)


if __name__ == "__main__":
    main()
