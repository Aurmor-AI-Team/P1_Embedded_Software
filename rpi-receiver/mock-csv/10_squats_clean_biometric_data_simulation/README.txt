CLEAN 10-SQUAT BIOMETRIC SIMULATION — analytic, biomechanically consistent
===========================================================================
Same CSV schema, node set, and sampling rate as
    ../10 squats_biometric_data_simulation/
but every per-frame value is derived from a single "squat depth" function so
that accel, gyro, mag, UWB distance, EMG and HR are physically coherent
(e.g. knee gz peak == d(knee_pitch)/dt, not an independent random walk).

GENERATOR
  generate.py  — re-run any time to refresh the CSVs in place.

TIMELINE  (255 rounds × 255 ms = 65.025 s @ 3.92 Hz)
  0–5 s     standing setup
  5–35 s    10 squats, 3.0 s/rep (1.5 s descent + 1.5 s ascent)
  35–65 s   recovery

SQUAT MODEL
  per-rep phase τ ∈ [0, 1]
  depth(τ) = 0.5 × (1 − cos(2π τ))          0 standing, 1 bottom
  every segment pitches forward by  pitch_max × depth, with pitch_max:
      head 5°, chest 22°, elbow 15°, wrist 25°, knee 110°, ankle 20°
  ⇒  gyro = d(pitch)/dt  is automatically consistent with the accel
      (gravity in body frame), and the README targets hit naturally:
        knee  gz peak ≈ ±115 dps    chest gz peak ≈ ±25 dps
  UWB distance to head shrinks as the body drops:
        ankle  1.44 → 1.13 m        knee   0.85 → 0.65 m
        wrist  0.60 → 0.57 m        chest  0.30 → 0.28 m

PHYSIOLOGY  (envelope tracks activity, ramps in over ~12 s then decays)
  HR (PPG + ECG)   64 → 114 bpm
  RMSSD            55 → 36 ms
  Resp rate        14 → 26 br/min
  SpO2             98 → 96 %
  Quad EMG (WF/WG ch0+ch1)   bursts ≈ 0.64 peak on the ASCENT
                              ≈ 0.22 peak on the eccentric descent
  EEG (HEAD)       alpha suppressed (×0.65), beta lifted (×1.65) during reps
  BIA (chest)      ≈ 7.1° phase, steady (healthy adult)

DIFFERENCES vs the original 10-squat set
  - rep cadence is exactly 3.0 s with no jitter (10 clean reps)
  - knee accel/gyro now physically agree (the original gz peak 115 dps
    implied ~110° rotation, but the accel range only showed ~40°)
  - EMG bursts are gated to the ASCENT phase, not just to the squat window
  - HR follows a smooth ramp/decay rather than per-frame noise
  - all 10 nodes share one squat-depth signal, so distance_m, gyro, accel
    on every node line up frame-for-frame
  - keeps the original column order, dtypes and value formatting

NODE / SENSOR PLACEMENT (present_mask, hex)
  WA Chest    0x55  IMU + ECG + RESP + BIA
  WB L_Elbow  0x01  IMU only
  WC R_Elbow  0x01  IMU only
  WD L_Wrist  0x03  IMU + PPG (HR/SpO2)
  WE R_Wrist  0x03  IMU + PPG
  WF L_Knee   0x21  IMU + EMG (quadriceps)
  WG R_Knee   0x21  IMU + EMG (quadriceps)
  WH L_Ankle  0x01  IMU only
  WI R_Ankle  0x01  IMU only
  HEAD (main) 0x09  IMU + EEG
