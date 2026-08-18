# ik-model — apply UltraInertialPoser to mock-csv streams

[apply_ik.py](apply_ik.py) turns the 10-node biometric simulations under
[../mock-csv/](../mock-csv/) into a full-body pose sequence (SMPL 24-joint
rotations + root translation, per frame) by running the
[UltraInertialPoser (UIP)](https://github.com/eth-siplab/UltraInertialPoser)
model.

## What the script does

1. Reads the 6 CSVs that map to UIP's IMU slots:
   `WD_L_Wrist, WE_R_Wrist, WF_L_Knee, WG_R_Knee, HEAD_Head_main, WA_Chest`
   (the chest stands in for the pelvis/root because the mock rig has no
   pelvis node).
2. Aligns them on `t_s`, optionally upsamples from 3.92 Hz to 60 Hz.
3. Fuses each node's accel + gyro + mag with a Madgwick MARG filter to get
   a world-frame rotation matrix per IMU.
4. Rotates the body-frame accel into the world frame and removes gravity,
   converting g → m/s².
5. Builds the `[N, 6, 6]` UWB distance matrix from each node's
   `distance_m` (head row/column only; rest = 0).
6. Calls `UIP.predict(glb_acc, glb_rot, init_pose, glb_uwb=…, offset=…)`
   once on the whole sequence (the model is RNN/GNN-based and needs
   temporal context).
7. Walks the returned `pose [N, 24, 3, 3]` and `tran [N, 3]` in time order
   and writes `pose_seq.npz` + `pose_seq.csv` per dataset.

## Setup

```bash
pip install -r requirements.txt

# Clone the UIP repo next to this script.
git clone https://github.com/eth-siplab/UltraInertialPoser

# Install rbdl (build from source — see UIP README).
# Download the SMPL body model (license-gated):
#   https://smpl.is.tue.mpg.de/  → basicmodel_m_lbs_10_207_0_v1.0.0.pkl
mkdir -p data && cp <path>/basicmodel_m_lbs_10_207_0_v1.0.0.pkl data/

# Download the pretrained checkpoint:
#   https://drive.google.com/drive/folders/151pmZSRl_bEu5eJgu1V9SdjE-x7FtACz
mkdir -p weights && cp <path>/uip.pt weights/uip.pt
```

## Run

```bash
# Default: process both squats + pushups.
python apply_ik.py

# Process one dataset.
python apply_ik.py --input "../mock-csv/10 squats_biometric_data_simulation"

# Sanity-check the data pipeline without the model (no checkpoint needed):
python apply_ik.py --dry-run
```

CLI flags:

| Flag             | Default                                      | Notes                                                              |
|------------------|----------------------------------------------|--------------------------------------------------------------------|
| `--input`        | both `mock-csv/` subdirs                     | Single dataset directory                                           |
| `--uip-repo`     | `ik-model/UltraInertialPoser`                | Local clone of the UIP repo                                        |
| `--ckpt`         | `ik-model/weights/uip.pt`                    | Pretrained checkpoint                                              |
| `--smpl-dir`     | `ik-model/data`                              | Directory holding the SMPL `.pkl`                                  |
| `--out-dir`      | `ik-model/results`                           | Per-dataset subfolder will be created                              |
| `--target-hz`    | `60.0`                                       | Resample target before inference                                   |
| `--no-resample`  | off                                          | Feed the raw 3.92 Hz timeline                                      |
| `--device`       | `cpu`                                        | `cpu` / `cuda` / `mps`                                             |
| `--dry-run`      | off                                          | Build tensors only, skip the model                                 |

## Output

For each input dataset, `<out-dir>/<dataset_slug>/` contains:

- `pose_seq.npz` — `{ t_s: [N], pose_rotmat: [N, 24, 3, 3], tran: [N, 3] }`
- `pose_seq.csv` — one row per frame, columns
  `t_s, tran_x, tran_y, tran_z, j0_r00 … j23_r22`.

To render an SMPL avatar from the NPZ, point UIP's
`visualizer/visualize_result.py` at the file.

## Caveats

- The mock CSVs sample at ~3.92 Hz, far below UIP's training rate. Pose
  quality at native rate is unreliable; upsampling to 60 Hz is the
  default but does not invent information.
- The mock rig has no pelvis IMU; we substitute the chest. Expect root
  translation in particular to be approximate.
- UWB pairwise distances are not in the CSV. We populate only the head
  row/column from `distance_m`; UIP runs but its UWB term is degraded.
