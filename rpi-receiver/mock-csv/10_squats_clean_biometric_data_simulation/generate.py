"""Generate a clean, biomechanically consistent 10-rep squat dataset.

Same CSV schema and node set as the original
    mock-csv/10 squats_biometric_data_simulation/
but the per-frame signals are derived analytically from a single squat-depth
function instead of being independently noised. The result is:

  * a perfectly regular 3.0 s/rep cadence (10 reps, t = 5 s … 35 s)
  * deterministic kinematic curves (depth → distance_m, accel, gyro, mag)
  * physiology that tracks the workout (HR ramp, EMG bursts on the ASCENT,
    resp rate climb, SpO2 dip, BIA steady)
  * 255 frames @ 3.92 Hz like the rest of mock-csv/

Run from this directory:
    python3 generate.py
"""

from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path
import math
import numpy as np

OUT_DIR = Path(__file__).resolve().parent
SEED = 20260605

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
DT = 0.255                                  # 255 ms per UWB round
N = 255                                     # 255 rounds → 65.025 s
T = np.arange(N) * DT                       # 0 … 64.770 s
T0 = datetime(2026, 6, 5, 15, 0, 0)

SETUP_END = 5.0                             # standing setup
SQUAT_START = 5.0
REP_PERIOD = 3.0
N_REPS = 10
SQUAT_END = SQUAT_START + REP_PERIOD * N_REPS  # 35 s

rng = np.random.default_rng(SEED)


def noise(scale: float, n: int = N) -> np.ndarray:
    return rng.normal(0.0, scale, size=n)


# ---------------------------------------------------------------------------
# Squat kinematics — one master "depth" curve drives every sensor
# ---------------------------------------------------------------------------
def squat_depth_and_rate(t: np.ndarray):
    """depth ∈ [0, 1] (0 standing, 1 deep squat), and its time-derivative.

    Each rep is a smooth (1 - cos) bowl: descent for the first half,
    ascent for the second half. depth_dot < 0 during descent, > 0 ascent.
    """
    in_sq = (t >= SQUAT_START) & (t < SQUAT_END)
    tau = np.where(in_sq, (t - SQUAT_START) % REP_PERIOD, 0.0) / REP_PERIOD  # 0..1
    omega = 2 * math.pi / REP_PERIOD
    depth = np.where(in_sq, 0.5 * (1 - np.cos(2 * math.pi * tau)), 0.0)
    # d depth / dt; >0 on descent, <0 on ascent (then flipped below for "rate")
    depth_dot = np.where(in_sq, 0.5 * omega * np.sin(2 * math.pi * tau), 0.0)
    return in_sq, depth, depth_dot, tau


IN_SQ, DEPTH, DEPTH_DOT, TAU = squat_depth_and_rate(T)

# Convenience: descent mask (going down) and ascent mask (coming up)
DESCENT = IN_SQ & (TAU < 0.5)
ASCENT = IN_SQ & (TAU >= 0.5)


# ---------------------------------------------------------------------------
# Physiology — slow envelopes over the 65 s session
# ---------------------------------------------------------------------------
def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


# Activity envelope: 0 at rest, ramps to 1 over the first ~half of squats,
# stays high, then decays during recovery.
ramp_in = smoothstep((T - SQUAT_START) / 12.0)
ramp_out = 1.0 - smoothstep((T - SQUAT_END) / 25.0)
ACTIVITY = np.clip(np.minimum(ramp_in, ramp_out), 0.0, 1.0)
ACTIVITY[T < SQUAT_START] = 0.0

HR = 64.0 + 50.0 * ACTIVITY                                # 64 → 114 bpm
RMSSD = 55.0 - 19.0 * ACTIVITY                             # 55 → 36 ms
RESP = 14.0 + 12.0 * ACTIVITY                              # 14 → 26 br/min
SPO2 = 98.0 - 2.0 * ACTIVITY                               # 98 → 96 %
PPG_Q = 90.0 - 8.0 * ACTIVITY                              # quality dips
EEG_ALPHA_GAIN = 1.0 - 0.35 * ACTIVITY                     # alpha suppressed
EEG_BETA_GAIN = 1.0 + 0.65 * ACTIVITY                      # beta rises


# ---------------------------------------------------------------------------
# IMU helpers
# ---------------------------------------------------------------------------
def gravity_in_body(pitch_deg: np.ndarray, mount_yaw_deg: float = 0.0):
    """Return (ax, ay, az) in body-g when the segment is pitched forward
    by `pitch_deg` about its medial-lateral axis.

    Standing: ay = -1g (Y is "up" out of the body), ax = az = 0.
    A forward pitch about the lateral (body Z) axis turns gravity into:
        g_body = (sin θ, -cos θ, 0)
    A non-zero mount yaw splits the forward (X) component into X and Z so
    both axes pick up a piece of the tilt — matches how the original
    simulation's chest/knee sensors record motion on ax AND az together.
    """
    θ = np.deg2rad(pitch_deg)
    φ = math.radians(mount_yaw_deg)
    s, c = np.sin(θ), np.cos(θ)
    ax = s * math.cos(φ)
    az = s * math.sin(φ)
    ay = -c
    return ax, ay, az


def gyro_from_pitch(pitch_deg: np.ndarray, mount_yaw_deg: float = 0.0):
    """Angular velocity about the lateral axis from d(pitch)/dt, split by
    the mount yaw between gx, gy, gz the same way gravity is split."""
    # numerical derivative of pitch_deg (already in deg) → dps
    rate = np.gradient(pitch_deg, DT)
    φ = math.radians(mount_yaw_deg)
    gx = rate * math.sin(φ) * 0.05      # tiny lateral wobble
    gy = rate * 0.0
    gz = rate * 1.0                     # primary axis carries the rotation
    return gx, gy, gz


def write_csv(name: str, columns: list[str], data: dict):
    path = OUT_DIR / name
    n = len(data[columns[0]])
    rows = []
    for i in range(n):
        cells = []
        for col in columns:
            v = data[col][i]
            if isinstance(v, str):
                cells.append(v)
            elif isinstance(v, (int, np.integer)):
                cells.append(str(int(v)))
            elif col in ("ax_g", "ay_g", "az_g", "hx_g", "hy_g", "hz_g"):
                cells.append(f"{v:+.3f}")
            elif col in ("gx_dps", "gy_dps", "gz_dps"):
                cells.append(f"{v:+.1f}")
            elif col == "distance_m":
                cells.append(f"{v:.3f}")
            elif col == "imu_temp_c":
                cells.append(f"{v:.2f}")
            elif col.startswith("eeg_") or col.startswith("emg_"):
                cells.append(f"{v:.3f}")
            elif col.startswith("bia_"):
                cells.append(f"{v:.2f}")
            else:
                cells.append(f"{v}")
        rows.append(",".join(cells))
    with open(path, "w") as fh:
        fh.write(",".join(columns) + "\n")
        fh.write("\n".join(rows) + "\n")
    print(f"  wrote {path.name}  ({n} rows)")


# ---------------------------------------------------------------------------
# Shared columns for every CSV
# ---------------------------------------------------------------------------
TIMESTAMPS = [(T0 + timedelta(seconds=float(t))).strftime("%Y-%m-%dT%H:%M:%S.")
              + f"{int((float(t) % 1) * 1000):03d}" for t in T]
T_S = [f"{t:.3f}" for t in T]
ROUND = list(range(N))
SEQ = list(range(N))
VERSION = [2] * N
TEMP = (29.00 + 0.10 * np.sin(2 * math.pi * T / 30.0) + noise(0.04)).round(2)


def base_imu(node: str, label: str, mask: str):
    return dict(
        timestamp_iso=TIMESTAMPS,
        t_s=T_S,
        round=ROUND,
        node=[node] * N,
        label=[label] * N,
        present_mask_hex=[mask] * N,
        version=VERSION,
        node_seq=SEQ,
        imu_temp_c=TEMP,
    )


# ---------------------------------------------------------------------------
# Per-node signal models
# ---------------------------------------------------------------------------
def make_chest():
    """Torso pitches ~22° forward at the bottom so gz peaks at ±25 dps
    (README spec). Distance to head stays near the standing chest→head
    offset (~0.30 m) since both drop together."""
    pitch = 22.0 * DEPTH
    ax, ay, az = gravity_in_body(pitch, mount_yaw_deg=45.0)
    ax = ax + noise(0.010); ay = ay + noise(0.010); az = az + noise(0.010)
    gx, gy, gz = gyro_from_pitch(pitch, mount_yaw_deg=45.0)
    gx = gx + noise(0.6); gy = noise(0.6); gz = gz + noise(0.8)
    hx = ax * 0.95 + noise(0.020)
    hy = ay * 0.97 + noise(0.020)
    hz = az * 0.95 + noise(0.020)
    distance = 0.300 - 0.020 * DEPTH + noise(0.010)

    cols = ["timestamp_iso", "t_s", "round", "node", "label", "present_mask_hex",
            "version", "node_seq", "distance_m",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
            "hx_g", "hy_g", "hz_g", "imu_temp_c",
            "resp_rate_bpm", "ecg_hr_bpm", "ecg_rmssd_ms", "ecg_flags",
            "bia_resistance_ohm", "bia_reactance_ohm", "bia_phase_deg",
            "bia_impedance_ohm"]
    data = base_imu("WA", "Chest", "0x55")
    data["distance_m"] = distance
    data["ax_g"] = ax; data["ay_g"] = ay; data["az_g"] = az
    data["gx_dps"] = gx; data["gy_dps"] = gy; data["gz_dps"] = gz
    data["hx_g"] = hx; data["hy_g"] = hy; data["hz_g"] = hz
    data["resp_rate_bpm"] = np.round(RESP).astype(int)
    data["ecg_hr_bpm"] = np.round(HR + noise(0.8)).astype(int)
    data["ecg_rmssd_ms"] = np.round(RMSSD + noise(1.5)).astype(int)
    data["ecg_flags"] = [0] * N
    data["bia_resistance_ohm"] = (509.0 + 0.5 * np.sin(2 * math.pi * T / 12.0)
                                  + noise(0.5))
    data["bia_reactance_ohm"] = 63.2 + noise(0.2)
    data["bia_phase_deg"] = 7.10 + noise(0.04)
    data["bia_impedance_ohm"] = np.sqrt(data["bia_resistance_ohm"] ** 2
                                        + data["bia_reactance_ohm"] ** 2)
    write_csv("WA_Chest.csv", cols, data)


def make_elbow(side: str):
    """Arms swing slightly forward as a counterbalance (≤15°)."""
    pitch = 15.0 * DEPTH
    ax, ay, az = gravity_in_body(pitch, mount_yaw_deg=20.0)
    ax = ax + noise(0.020); ay = ay + noise(0.020); az = az + noise(0.020)
    gx, gy, gz = gyro_from_pitch(pitch, mount_yaw_deg=20.0)
    gx = gx + noise(0.8); gy = noise(0.8); gz = gz + noise(1.2)
    hx = ax * 0.94 + noise(0.020)
    hy = ay * 0.96 + noise(0.020)
    hz = az * 0.94 + noise(0.020)
    distance = 0.500 - 0.030 * DEPTH + noise(0.012)

    cols = ["timestamp_iso", "t_s", "round", "node", "label", "present_mask_hex",
            "version", "node_seq", "distance_m",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
            "hx_g", "hy_g", "hz_g", "imu_temp_c"]
    node = "WB" if side == "L" else "WC"
    label = f"{side}_Elbow"
    data = base_imu(node, label, "0x01")
    data["distance_m"] = distance
    data["ax_g"] = ax; data["ay_g"] = ay; data["az_g"] = az
    data["gx_dps"] = gx; data["gy_dps"] = gy; data["gz_dps"] = gz
    data["hx_g"] = hx; data["hy_g"] = hy; data["hz_g"] = hz
    write_csv(f"{node}_{label}.csv", cols, data)


def make_wrist(side: str):
    """Wrist swings further forward than elbow (~25°). Carries PPG."""
    pitch = 25.0 * DEPTH
    ax, ay, az = gravity_in_body(pitch, mount_yaw_deg=30.0)
    ax = ax + noise(0.018); ay = ay + noise(0.018); az = az + noise(0.018)
    gx, gy, gz = gyro_from_pitch(pitch, mount_yaw_deg=30.0)
    gx = gx + noise(1.0); gy = noise(1.0); gz = gz + noise(1.5)
    # The wrist sees a small steady offset on ax even at rest (the watch face
    # is slightly tilted relative to the forearm).
    ax = ax + 0.17
    hx = ax * 0.95 + noise(0.020)
    hy = ay * 0.97 + noise(0.020)
    hz = az * 0.95 + noise(0.020)
    distance = 0.600 - 0.035 * DEPTH + noise(0.014)

    cols = ["timestamp_iso", "t_s", "round", "node", "label", "present_mask_hex",
            "version", "node_seq", "distance_m",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
            "hx_g", "hy_g", "hz_g", "imu_temp_c",
            "ppg_hr_bpm", "ppg_spo2_pct", "ppg_quality"]
    node = "WD" if side == "L" else "WE"
    label = f"{side}_Wrist"
    data = base_imu(node, label, "0x03")
    data["distance_m"] = distance
    data["ax_g"] = ax; data["ay_g"] = ay; data["az_g"] = az
    data["gx_dps"] = gx; data["gy_dps"] = gy; data["gz_dps"] = gz
    data["hx_g"] = hx; data["hy_g"] = hy; data["hz_g"] = hz
    data["ppg_hr_bpm"] = np.round(HR + noise(0.8)).astype(int)
    data["ppg_spo2_pct"] = np.round(SPO2 + noise(0.2)).astype(int)
    data["ppg_quality"] = np.round(PPG_Q + noise(2.0)).astype(int)
    write_csv(f"{node}_{label}.csv", cols, data)


def make_knee(side: str):
    """Shin/upper-tibia sensor sees a ~110° rotation envelope so that
    gz peaks at ±115 dps (README spec). EMG bursts on the ASCENT
    (concentric phase of the quad), lighter on the eccentric descent."""
    pitch = 110.0 * DEPTH
    ax, ay, az = gravity_in_body(pitch, mount_yaw_deg=55.0)
    ax = ax + noise(0.015); ay = ay + noise(0.015); az = az + noise(0.020)
    gx, gy, gz = gyro_from_pitch(pitch, mount_yaw_deg=55.0)
    gx = gx + noise(1.5); gy = noise(1.5); gz = gz + noise(3.0)
    hx = ax * 0.94 + noise(0.020)
    hy = ay * 0.97 + noise(0.020)
    hz = az * 0.94 + noise(0.020)
    distance = 0.850 - 0.200 * DEPTH + noise(0.012)

    # Quadriceps EMG envelope.
    # Ascent (concentric, knee extending): big burst ~ 0.64 peak.
    # Descent (eccentric, controlling the drop): half-amplitude burst.
    # Shape = |sin(2π τ)| × per-phase gain, gated by IN_SQ.
    base_burst = np.abs(np.sin(2 * math.pi * TAU))
    gain = np.where(ASCENT, 0.64, np.where(DESCENT, 0.22, 0.0))
    emg_quad = base_burst * gain
    # Resting tone outside squats
    rest_tone = 0.02 + 0.005 * np.sin(2 * math.pi * T / 7.0)
    emg_ch0 = np.where(IN_SQ, emg_quad, 0.0) + rest_tone + np.abs(noise(0.012))
    emg_ch1 = np.where(IN_SQ, emg_quad * 0.92, 0.0) + rest_tone + np.abs(noise(0.012))
    emg_ch2 = rest_tone + np.abs(noise(0.012))
    emg_ch3 = rest_tone * 1.2 + np.abs(noise(0.014))

    cols = ["timestamp_iso", "t_s", "round", "node", "label", "present_mask_hex",
            "version", "node_seq", "distance_m",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
            "hx_g", "hy_g", "hz_g", "imu_temp_c",
            "emg_rms_ch0", "emg_rms_ch1", "emg_rms_ch2", "emg_rms_ch3"]
    node = "WF" if side == "L" else "WG"
    label = f"{side}_Knee"
    data = base_imu(node, label, "0x21")
    data["distance_m"] = distance
    data["ax_g"] = ax; data["ay_g"] = ay; data["az_g"] = az
    data["gx_dps"] = gx; data["gy_dps"] = gy; data["gz_dps"] = gz
    data["hx_g"] = hx; data["hy_g"] = hy; data["hz_g"] = hz
    data["emg_rms_ch0"] = emg_ch0
    data["emg_rms_ch1"] = emg_ch1
    data["emg_rms_ch2"] = emg_ch2
    data["emg_rms_ch3"] = emg_ch3
    write_csv(f"{node}_{label}.csv", cols, data)


def make_ankle(side: str):
    """Foot stays planted; ankle pitch tracks shin slightly (~20°).
    Distance to head is the biggest mover: 1.44 m → 1.13 m."""
    pitch = 20.0 * DEPTH
    ax, ay, az = gravity_in_body(pitch, mount_yaw_deg=25.0)
    ax = ax + noise(0.012); ay = ay + noise(0.012); az = az + noise(0.015)
    gx, gy, gz = gyro_from_pitch(pitch, mount_yaw_deg=25.0)
    gx = gx + noise(1.2); gy = noise(1.2); gz = gz + noise(2.0)
    hx = ax * 0.94 + noise(0.020)
    hy = ay * 0.97 + noise(0.020)
    hz = az * 0.94 + noise(0.020)
    distance = 1.440 - 0.310 * DEPTH + noise(0.013)

    cols = ["timestamp_iso", "t_s", "round", "node", "label", "present_mask_hex",
            "version", "node_seq", "distance_m",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
            "hx_g", "hy_g", "hz_g", "imu_temp_c"]
    node = "WH" if side == "L" else "WI"
    label = f"{side}_Ankle"
    data = base_imu(node, label, "0x01")
    data["distance_m"] = distance
    data["ax_g"] = ax; data["ay_g"] = ay; data["az_g"] = az
    data["gx_dps"] = gx; data["gy_dps"] = gy; data["gz_dps"] = gz
    data["hx_g"] = hx; data["hy_g"] = hy; data["hz_g"] = hz
    write_csv(f"{node}_{label}.csv", cols, data)


def make_head():
    """Head pitches forward only a few degrees (gaze stays roughly forward).
    No distance_m column — head is the UWB reference."""
    pitch = 5.0 * DEPTH
    ax, ay, az = gravity_in_body(pitch, mount_yaw_deg=10.0)
    ax = ax + noise(0.012); ay = ay + noise(0.012); az = az + noise(0.012)
    gx, gy, gz = gyro_from_pitch(pitch, mount_yaw_deg=10.0)
    gx = gx + noise(0.8); gy = noise(0.8); gz = gz + noise(0.9)
    hx = ax * 0.97 + noise(0.020)
    hy = ay * 0.98 + noise(0.020)
    hz = az * 0.97 + noise(0.020)

    cols = ["timestamp_iso", "t_s", "round", "node", "label", "present_mask_hex",
            "version", "node_seq",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
            "hx_g", "hy_g", "hz_g", "imu_temp_c",
            "eeg_delta", "eeg_theta", "eeg_alpha", "eeg_beta", "eeg_gamma"]
    data = base_imu("HEAD", "Head_main", "0x09")
    data["ax_g"] = ax; data["ay_g"] = ay; data["az_g"] = az
    data["gx_dps"] = gx; data["gy_dps"] = gy; data["gz_dps"] = gz
    data["hx_g"] = hx; data["hy_g"] = hy; data["hz_g"] = hz
    data["eeg_delta"] = 0.350 + noise(0.012)
    data["eeg_theta"] = 0.200 + noise(0.010)
    data["eeg_alpha"] = 0.220 * EEG_ALPHA_GAIN + noise(0.012)
    data["eeg_beta"] = 0.145 * EEG_BETA_GAIN + noise(0.010)
    data["eeg_gamma"] = 0.080 + noise(0.010)
    write_csv("HEAD_Head_main.csv", cols, data)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    print(f"Generating clean 10-squat dataset in {OUT_DIR}")
    make_head()
    make_chest()
    make_elbow("L"); make_elbow("R")
    make_wrist("L"); make_wrist("R")
    make_knee("L"); make_knee("R")
    make_ankle("L"); make_ankle("R")
    print("done.")


if __name__ == "__main__":
    main()
