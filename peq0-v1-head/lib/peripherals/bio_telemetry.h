/*! ----------------------------------------------------------------------------
 * @file    bio_telemetry.h
 * @brief   Extensible per-node biometric telemetry payload carried inside the
 *          UWB DS-TWR Final frame.
 *
 * This struct REPLACES the bare lsm6_sample_t that the peripheral currently
 * memcpy's into the Final frame. Both the peripheral (which fills it) and the
 * main (which extracts it) include this header, so the wire format is defined
 * in exactly one place.
 *
 * DESIGN NOTES
 * ------------
 *  - Channel budget: the ranging Final frame is ~3.9 Hz/node and, in standard
 *    PHR mode (dwt_config_t.phrMode = DWT_PHRMODE_STD), the whole frame is
 *    capped at 127 bytes incl. FCS. After the 10-byte header + 15 bytes of
 *    DS-TWR timestamps, ~100 bytes remain. A fully-populated bio_telemetry_t
 *    below is ~56 bytes, so there is generous headroom. If you ever exceed
 *    ~100 bytes, switch the config to DWT_PHRMODE_EXT (up to 1023 bytes) at a
 *    small air-time cost.
 *
 *  - DERIVED metrics only. This pipe carries computed values (HR, breathing
 *    rate, EEG band energies, ECG HR/HRV, EMG RMS, BIA R/Xc/phase), NOT raw
 *    waveforms. Raw ECG/EMG/EEG/PPG/BIA sweep streams are 0.5-4 KB/s each and
 *    do not fit. If you need raw signals for the biomech/IK model, log them
 *    on-node (flash/SD) timestamped against the beacon round counter and fuse
 *    offline.
 *
 *  - present_mask: each node sets only the bits for sensors it actually has.
 *    The main reads the mask before trusting any field. Adding a new sensor =
 *    add a BIO_HAS_* bit + fields; old firmware ignores unknown bits. Bump
 *    BIO_TELEM_VERSION on any layout change so a mixed fleet can be detected.
 *    NOTE: appending fields also changes sizeof(bio_telemetry_t), hence the
 *    Final frame length, so a version mismatch is also a hard length mismatch
 *    at the RX check -- you MUST reflash every board together after a bump.
 *
 *  - Fixed-point: all multi-byte fields are little-endian (native ESP32, and
 *    matches the ts_to_frame/ts_from_frame helpers already in uwb_ranging.c).
 *    Scales are chosen to cover each sensor's range within int16/uint16.
 * ---------------------------------------------------------------------------
 */
#ifndef BIO_TELEMETRY_H
#define BIO_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BIO_TELEM_VERSION  2   /* bumped: added BIA (R/Xc/phase) -> reflash all */

/* ---- present_mask bits (uint16, room for 16 sensor classes) ------------- */
#define BIO_HAS_IMU   (1u << 0)   /* low-g accel + gyro + high-g accel + temp */
#define BIO_HAS_PPG   (1u << 1)   /* optical: HR, SpO2, signal quality        */
#define BIO_HAS_RESP  (1u << 2)   /* breathing rate                           */
#define BIO_HAS_EEG   (1u << 3)   /* band energies (delta..gamma)             */
#define BIO_HAS_ECG   (1u << 4)   /* ECG-derived HR, HRV, status flags        */
#define BIO_HAS_EMG   (1u << 5)   /* per-channel RMS                          */
#define BIO_HAS_BIA   (1u << 6)   /* bioimpedance: resistance, reactance, phase */
/* bits 7..15 reserved for future sensor classes                            */

#define BIO_EEG_BANDS     5       /* delta, theta, alpha, beta, gamma         */
#define BIO_EMG_CHANNELS  4       /* adjust to your hardware                  */

/* ---- fixed-point scales (value = raw * scale; encode raw = value / scale) */
#define BIO_ACCEL_LSB_PER_G    2048.0f   /* int16, +/-16 g  (32768/16)        */
#define BIO_GYRO_LSB_PER_DPS      8.0f   /* int16, +/-4096 dps                */
#define BIO_HIGHG_LSB_PER_G     256.0f   /* int16, +/-128 g  (covers +/-80g)  */
#define BIO_TEMP_LSB_PER_C      100.0f   /* int16, deg C * 100                */
#define BIO_EEG_LSB_PER_UNIT   1000.0f   /* uint16, relative power 0..1 -> 0..1000 */
#define BIO_EMG_LSB_PER_UNIT   1000.0f   /* int16, normalized RMS * 1000 (define your unit) */
#define BIO_BIA_LSB_PER_OHM      10.0f   /* uint16, ohms * 10  -> 0..6553.5 ohm, 0.1 ohm res */
#define BIO_BIA_LSB_PER_DEG     100.0f   /* uint16, deg * 100  -> 0..655.35 deg, 0.01 deg res */

/* Sentinels for "field present in struct but value not available". */
#define BIO_U8_NA   0xFFu
#define BIO_U16_NA  0xFFFFu

#pragma pack(push, 1)
typedef struct {
    uint8_t  version;        /* = BIO_TELEM_VERSION                          */
    uint8_t  node_seq;       /* per-node rolling counter (drop/dup detection)*/
    uint16_t present_mask;   /* OR of BIO_HAS_* for the fields below         */

    /* ---- IMU (BIO_HAS_IMU) ---- */
    int16_t  accel_g[3];     /* low-g, BIO_ACCEL_LSB_PER_G                    */
    int16_t  gyro_dps[3];    /* BIO_GYRO_LSB_PER_DPS                          */
    int16_t  highg_g[3];     /* high-g, BIO_HIGHG_LSB_PER_G                   */
    int16_t  imu_temp;       /* BIO_TEMP_LSB_PER_C                            */

    /* ---- PPG (BIO_HAS_PPG) ---- */
    uint8_t  ppg_hr_bpm;     /* 0..254, BIO_U8_NA if unavailable             */
    uint8_t  ppg_spo2_pct;   /* 0..100,  BIO_U8_NA if unavailable            */
    uint8_t  ppg_quality;    /* 0..100 signal quality / perfusion proxy      */

    /* ---- Respiration (BIO_HAS_RESP) ---- */
    uint8_t  resp_rate_bpm;  /* breaths/min, BIO_U8_NA if unavailable        */

    /* ---- EEG band energies (BIO_HAS_EEG) ---- */
    uint16_t eeg_band[BIO_EEG_BANDS];  /* relative power, BIO_EEG_LSB_PER_UNIT */

    /* ---- ECG-derived (BIO_HAS_ECG) ---- */
    uint8_t  ecg_hr_bpm;     /* beats/min from ECG, BIO_U8_NA if unavailable */
    uint16_t ecg_rmssd_ms;   /* HRV (RMSSD), milliseconds                     */
    uint8_t  ecg_flags;      /* bit0 arrhythmia, bit1 lead-off, ...          */

    /* ---- EMG (BIO_HAS_EMG) ---- */
    int16_t  emg_rms[BIO_EMG_CHANNELS]; /* per-channel RMS, BIO_EMG_LSB_PER_UNIT */

    /* ---- Bioimpedance (BIO_HAS_BIA) ----
     * Single-frequency BIA (report at your phase-angle reference frequency,
     * conventionally 50 kHz). R and Xc are the measured components; phase
     * angle phi = atan(Xc / R) * 180/pi, carried directly so the main does no
     * trig. Impedance magnitude |Z| = sqrt(R^2 + Xc^2) is recovered on the
     * main from R and Xc (see bio_get_bia_impedance_ohm). For multi-frequency
     * BIA, add a second {R,Xc,phase} triplet under a new present bit rather
     * than overloading these. */
    uint16_t bia_resistance_ohm; /* R,  BIO_BIA_LSB_PER_OHM, BIO_U16_NA if N/A */
    uint16_t bia_reactance_ohm;  /* Xc, BIO_BIA_LSB_PER_OHM, BIO_U16_NA if N/A */
    uint16_t bia_phase_deg;      /* phi, BIO_BIA_LSB_PER_DEG, BIO_U16_NA if N/A */
} bio_telemetry_t;
#pragma pack(pop)

/* Keep the whole Final frame inside standard-PHR budget:
 *   header(10) + 3 timestamps(15) + sizeof(bio_telemetry_t) + FCS(2) <= 127. */
_Static_assert(sizeof(bio_telemetry_t) <= 100,
               "bio_telemetry_t too large for standard-PHR Final frame; "
               "trim fields or switch to DWT_PHRMODE_EXT");

/* ------------------------------------------------------------------------- */
/* Encode/decode helpers (optional, but keep scale logic in one place).      */
/* ------------------------------------------------------------------------- */

static inline int16_t bio_clamp16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
}

static inline uint16_t bio_clampu16(float v)
{
    if (v < 0.0f)       return 0;
    if (v > 65535.0f)   return 65535;
    return (uint16_t)(v + 0.5f);
}

/* Fill the IMU portion from engineering units. Pass the same low-g/high-g/gyro
 * values you already read in lsm6_read_sample(). */
static inline void bio_set_imu(bio_telemetry_t *t,
                               const float accel_g[3],
                               const float gyro_dps[3],
                               const float highg_g[3],
                               float temp_c)
{
    for (int i = 0; i < 3; i++) {
        t->accel_g[i] = bio_clamp16(accel_g[i] * BIO_ACCEL_LSB_PER_G);
        t->gyro_dps[i] = bio_clamp16(gyro_dps[i] * BIO_GYRO_LSB_PER_DPS);
        t->highg_g[i] = bio_clamp16(highg_g[i] * BIO_HIGHG_LSB_PER_G);
    }
    t->imu_temp = bio_clamp16(temp_c * BIO_TEMP_LSB_PER_C);
    t->present_mask |= BIO_HAS_IMU;
}

/* Decode an IMU axis back to engineering units on the main side. */
static inline float bio_get_accel_g(const bio_telemetry_t *t, int axis)
{ return t->accel_g[axis] / BIO_ACCEL_LSB_PER_G; }
static inline float bio_get_gyro_dps(const bio_telemetry_t *t, int axis)
{ return t->gyro_dps[axis] / BIO_GYRO_LSB_PER_DPS; }
static inline float bio_get_highg_g(const bio_telemetry_t *t, int axis)
{ return t->highg_g[axis] / BIO_HIGHG_LSB_PER_G; }
static inline float bio_get_temp_c(const bio_telemetry_t *t)
{ return t->imu_temp / BIO_TEMP_LSB_PER_C; }

/* ---- Bioimpedance decoders ---- */
static inline float bio_get_bia_resistance_ohm(const bio_telemetry_t *t)
{ return t->bia_resistance_ohm / BIO_BIA_LSB_PER_OHM; }
static inline float bio_get_bia_reactance_ohm(const bio_telemetry_t *t)
{ return t->bia_reactance_ohm / BIO_BIA_LSB_PER_OHM; }
static inline float bio_get_bia_phase_deg(const bio_telemetry_t *t)
{ return t->bia_phase_deg / BIO_BIA_LSB_PER_DEG; }
/* Magnitude recovered from the two components -- no extra bytes on the wire. */
static inline float bio_get_bia_impedance_ohm(const bio_telemetry_t *t)
{
    float r = bio_get_bia_resistance_ohm(t);
    float x = bio_get_bia_reactance_ohm(t);
    return sqrtf(r * r + x * x);
}

#ifdef __cplusplus
}
#endif

#endif /* BIO_TELEMETRY_H */
