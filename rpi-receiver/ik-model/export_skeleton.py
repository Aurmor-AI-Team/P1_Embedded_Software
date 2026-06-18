"""Export the SMPL skeleton (kinematic tree + rest-pose joints) the app needs for
forward kinematics, as a small JSON asset.

The per-frame stream carries only joint *rotations*; to draw bones the app also
needs the fixed bone structure: each joint's parent and its rest (T-pose,
betas=0) position. We read those straight from the SMPL body model and write
them into the mobile app.

Run once on a machine that has the SMPL .pkl (the same one apply_ik.py uses):
    python export_skeleton.py [smpl.pkl] [out.json]
"""
from __future__ import annotations

import json
import pickle
import sys
from pathlib import Path

import numpy as np

# chumpy (used to unpickle the SMPL .pkl) imports numpy aliases removed in
# modern numpy. Re-add them so the pickle loads (same shim as apply_ik.py).
for _name, _typ in (("bool", bool), ("int", int), ("float", float),
                    ("complex", complex), ("object", object),
                    ("str", str), ("unicode", str)):
    if not hasattr(np, _name):
        setattr(np, _name, _typ)

SCRIPT_DIR = Path(__file__).resolve().parent              # .../rpi-receiver/ik-model
MAXIMA_ROOT = SCRIPT_DIR.parents[2]                        # .../maxima
DEFAULT_PKL = SCRIPT_DIR / "data" / "basicmodel_m_lbs_10_207_0_v1.0.0.pkl"
DEFAULT_OUT = (MAXIMA_ROOT / "aurmor-sports-mobile" / "features" / "skeleton"
               / "smplSkeleton.json")

# SMPL's 24 joints, in model order (matches pose_seq.csv j0…j23).
SMPL_JOINT_NAMES = [
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
    "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
    "neck", "left_collar", "right_collar", "head", "left_shoulder",
    "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    "left_hand", "right_hand",
]


def main() -> None:
    pkl = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PKL
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_OUT

    with open(pkl, "rb") as fh:
        model = pickle.load(fh, encoding="latin1")

    j_regressor = model["J_regressor"]                 # sparse [24, 6890]
    v_template = np.asarray(model["v_template"])       # [6890, 3]
    kintree = np.asarray(model["kintree_table"])       # [2, 24]

    rest_joints = np.asarray(j_regressor.dot(v_template))  # [24, 3] (betas=0)

    # Row 0 is each joint's parent; the root uses a uint32 sentinel -> -1.
    parents = []
    for p in kintree[0].tolist():
        parents.append(-1 if int(p) < 0 or int(p) > 23 else int(p))
    parents[0] = -1

    n = rest_joints.shape[0]
    skeleton = {
        "parents": parents,
        "joints": [[round(float(v), 5) for v in row] for row in rest_joints.tolist()],
        "names": SMPL_JOINT_NAMES[:n],
    }

    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as fh:
        json.dump(skeleton, fh)
    print(f"wrote {out}")
    print(f"  joints  : {n}")
    print(f"  parents : {parents}")


if __name__ == "__main__":
    main()
