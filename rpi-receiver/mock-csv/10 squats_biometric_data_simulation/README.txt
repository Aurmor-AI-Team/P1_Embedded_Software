BODYWEIGHT SQUAT BIOMETRIC SIMULATION — 10 reps, healthy adult
==============================================================
One CSV per body-worn UWB node. Values are ENGINEERING UNITS (what the main
recovers via bio_get_*), NOT on-wire fixed-point. Sampled at the UWB round
rate: 255 ms (3.92 Hz), 255 rows = 65 s.

TIMELINE
  0–5 s     standing / setup (baseline)
  5–35 s    10 squats, 3.0 s/rep (continuous; slower cadence than pushups)
  35–65 s   recovery

HOW THIS DIFFERS FROM THE PUSHUP SET (same physical rig, adapted placement)
  - Upright posture: gravity sits on the body-VERTICAL axis (ay ≈ -1 g),
    not horizontal as in the prone pushup.
  - KNEES are the prime movers: gz swings ±115 dps vs the torso's ±25 dps.
  - The whole body (incl. head/main) drops each rep, so ankle/knee distance
    to the head SHRINKS at the bottom (ankle: ~1.44 m → ~1.13 m).
  - EMG is on the QUADRICEPS (knee nodes WF/WG), bursting on the ASCENT
    (concentric), with a lighter eccentric burst on the way down.
  - Elbow nodes carry IMU only here (arms only counterbalance in a squat).

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

COLUMNS — identical schema/units to the pushup set:
  timestamp_iso,t_s,round,node,label,present_mask_hex,version,node_seq,
  distance_m, ax_g ay_g az_g, gx_dps gy_dps gz_dps, hx_g hy_g hz_g, imu_temp_c,
  ppg_hr_bpm ppg_spo2_pct ppg_quality, resp_rate_bpm,
  ecg_hr_bpm ecg_rmssd_ms ecg_flags, emg_rms_ch0..ch3,
  eeg_delta..eeg_gamma, bia_resistance/reactance/phase/impedance_ohm

PHYSIOLOGY (sanity ranges)
  HR        64 → ~114 bpm (slightly higher than pushups; large muscle mass)
  RMSSD     ~55 → ~36 ms
  Resp      14 → ~26 br/min
  SpO2      98 → ~96 %
  Quad EMG  bursts to ~0.64 on each ascent
  Knee gz   ±115 dps per rep; torso gz ±25 dps
  BIA phase ~7.1° steady (healthy adult)

Derived metrics only — no raw waveforms (don't fit the ~4 Hz frame).
