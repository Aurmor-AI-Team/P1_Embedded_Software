"""Run UIP on one sequence from the official UIP-DB test set.

Skips the CSV pipeline — UIP-DB tensors are already in the format predict()
wants. Writes the result in the same NPZ schema apply_ik.py uses, so
viz_pose.py works on it without changes.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from apply_ik import load_uip_model, stream_results, SCRIPT_DIR


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--db", type=Path, default=Path("/tmp/uip_db_listing/test.pt"))
    ap.add_argument("--seq", type=int, default=9, help="sequence index in test.pt")
    ap.add_argument("--start", type=int, default=0, help="first frame to include")
    ap.add_argument("--frames", type=int, default=2000,
                    help="cap frames to keep runtime sane (UIP physics opt is per-frame)")
    ap.add_argument("--ckpt", type=Path, default=SCRIPT_DIR / "weights" / "uip.pt")
    ap.add_argument("--uip-repo", type=Path, default=SCRIPT_DIR / "UltraInertialPoser")
    ap.add_argument("--smpl-dir", type=Path, default=SCRIPT_DIR / "data")
    ap.add_argument("--device", choices=["cpu", "cuda", "mps"], default="cpu")
    ap.add_argument("--out-dir", type=Path,
                    default=SCRIPT_DIR / "results" / "uip_db")
    args = ap.parse_args()

    print(f"loading {args.db}")
    d = torch.load(str(args.db), map_location="cpu", weights_only=False)
    n_total = d["acc"][args.seq].shape[0]
    s = max(0, min(args.start, n_total - 1))
    e = min(s + args.frames, n_total)
    print(f"sequence {args.seq}: {n_total} frames total, using [{s}, {e}) → {e - s} frames")
    print(f"  source: {d['fnames'][args.seq]}")

    glb_acc = d["acc"][args.seq][s:e].float()                  # [N, 6, 3]
    glb_rot = d["ori"][args.seq][s:e].float()                  # [N, 6, 3, 3]
    glb_uwb = d["vuwb"][args.seq][s:e].float()                 # [N, 6, 6]
    offset = d["offset"][args.seq].float()                     # [6, 3]
    # init_pose: build identity (T-pose) for joint rotations
    init_pose = torch.eye(3).expand(1, 24, 3, 3).contiguous().float()

    print(f"  glb_acc {tuple(glb_acc.shape)}  glb_rot {tuple(glb_rot.shape)}"
          f"  glb_uwb {tuple(glb_uwb.shape)}  offset {tuple(offset.shape)}")

    net = load_uip_model(args.uip_repo, args.ckpt, args.smpl_dir, args.device)

    print(f"running UIP.predict() on {e - s} frames…")
    t0 = time.time()
    with torch.no_grad():
        out = net.predict(
            glb_acc.to(args.device),
            glb_rot.to(args.device),
            init_pose.to(args.device),
            glb_uwb=glb_uwb.to(args.device),
            offset=offset.to(args.device),
        )
    pose_p, tran_p = out[0].cpu().numpy(), out[1].cpu().numpy()
    print(f"  pose {pose_p.shape}  tran {tran_p.shape}  ({time.time() - t0:.1f}s)")

    t_s = np.arange(e - s) / 60.0 + (s / 60.0)  # UIP-DB is 60 Hz
    tag = f"seq{args.seq:02d}_f{s}-{e}"
    stream_results(t_s, pose_p, tran_p, args.out_dir / tag)


if __name__ == "__main__":
    main()
