PUSHUP BIOMETRIC SIMULATION — 10 reps, healthy adult
=====================================================
One CSV per body-worn UWB node. Values are ENGINEERING UNITS (what the main
recovers via the bio_get_* accessors), NOT the on-wire fixed-point integers.
Sampled at the UWB round rate: 255 ms (3.92 Hz), 236 rows = 60 s.

TIMELINE
  0–5 s    setup / plank hold (baseline)
  5–30 s   10 pushups, 2.5 s/rep (continuous)
  30–60 s  recovery

NODE / SENSOR PLACEMENT (present_mask in each file, hex)
  WA Chest    0x55  IMU + ECG + RESP + BIA      (torso hub)
  WB L_Elbow  0x21  IMU + EMG (triceps)
  WC R_Elbow  0x21  IMU + EMG (triceps)
  WD L_Wrist  0x03  IMU + PPG (HR/SpO2)
  WE R_Wrist  0x03  IMU + PPG (HR/SpO2)
  WF/WG Knee  0x01  IMU only
  WH/WI Ankle 0x01  IMU only
  HEAD (main) 0x09  IMU + EEG (band powers)

COLUMNS (present only when that node carries the sensor)
  timestamp_iso, t_s, round, node, label, present_mask_hex, version, node_seq
  distance_m                         (peripherals only; computed by main, not on wire)
  ax_g ay_g az_g                     low-g accel (g)
  gx_dps gy_dps gz_dps               gyro (deg/s)
  hx_g hy_g hz_g                     high-g accel (g)
  imu_temp_c                         LSM6 die temp (°C)
  ppg_hr_bpm ppg_spo2_pct ppg_quality
  resp_rate_bpm
  ecg_hr_bpm ecg_rmssd_ms ecg_flags  (HRV = RMSSD; flags 0 = normal)
  emg_rms_ch0..ch3                   normalized RMS (ch0 = active triceps)
  eeg_delta..eeg_gamma               relative band powers (sum ≈ 1)
  bia_resistance_ohm bia_reactance_ohm bia_phase_deg bia_impedance_ohm

PHYSIOLOGY EMBEDDED (sanity ranges)
  HR        62 → ~106 bpm, lagged rise, slow recovery
  RMSSD     ~59 → ~40 ms  (HRV falls with exertion)
  Resp      14 → ~24 br/min
  SpO2      98 → ~96 % dip mid-set
  EMG       bursts to ~0.56 on each press (concentric) phase
  Elbow gz  ±95 dps per flexion/extension cycle
  BIA phase ~7.0° steady (healthy adult, ~50 kHz)
  IMU       gravity vector per pose + rep-cadence oscillation (0.4 Hz)

NOTE: derived metrics only — no raw ECG/EMG/EEG/PPG waveforms (those don't fit
the ~4 Hz frame; log raw on-node and fuse offline if needed).
