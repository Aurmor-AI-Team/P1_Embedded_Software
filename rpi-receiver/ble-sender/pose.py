"""Load a precomputed IK pose sequence (UIP `pose_seq.csv`) and serve a pose for
any biometric frame timestamp.

The CSV (from ik-model/apply_ik.py) has columns:
    t_s, tran_x, tran_y, tran_z, j0_r00 … j23_r22
i.e. root translation + one row-major 3x3 rotation matrix per SMPL joint. We
convert each matrix to a quaternion [w, x, y, z] up front so the wire payload is
compact and render-friendly. No numpy — stdlib only, so it runs anywhere.
"""
from __future__ import annotations

import bisect
import csv
import math
from pathlib import Path
from typing import List, Tuple


def rotmat_to_quat(m: List[List[float]]) -> List[float]:
    """3x3 row-major rotation matrix -> unit quaternion [w, x, y, z]."""
    (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = m[0], m[1], m[2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    return [w, x, y, z]


class PoseSequence:
    """Time-indexed joint quaternions + root translation, with nearest lookup."""

    def __init__(self, t_s: List[float], quats: List[List[List[float]]],
                 trans: List[List[float]]):
        self.t_s = t_s
        self.quats = quats
        self.trans = trans

    @property
    def count(self) -> int:
        return len(self.t_s)

    @property
    def n_joints(self) -> int:
        return len(self.quats[0]) if self.quats else 0

    def nearest(self, t: float) -> Tuple[List[float], List[List[float]]]:
        """Return (tran[3], quats[n_joints][4]) for the row closest to `t`."""
        i = bisect.bisect_left(self.t_s, t)
        if i <= 0:
            j = 0
        elif i >= len(self.t_s):
            j = len(self.t_s) - 1
        else:
            j = i if (self.t_s[i] - t) < (t - self.t_s[i - 1]) else i - 1
        return self.trans[j], self.quats[j]


def load_pose_csv(path) -> PoseSequence:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"pose file not found: {path}")
    t_s: List[float] = []
    quats: List[List[List[float]]] = []
    trans: List[List[float]] = []
    with path.open(newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        n_joints = (len(header) - 4) // 9
        for row in reader:
            vals = [float(v) for v in row]
            t_s.append(vals[0])
            trans.append([round(vals[1], 4), round(vals[2], 4), round(vals[3], 4)])
            jq: List[List[float]] = []
            for j in range(n_joints):
                b = 4 + j * 9
                m = [vals[b:b + 3], vals[b + 3:b + 6], vals[b + 6:b + 9]]
                jq.append([round(c, 4) for c in rotmat_to_quat(m)])
            quats.append(jq)
    return PoseSequence(t_s, quats, trans)
