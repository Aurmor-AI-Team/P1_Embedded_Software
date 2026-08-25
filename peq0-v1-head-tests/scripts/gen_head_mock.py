"""Generate src/head_mock_data.h from the HEAD mock CSV (IMU columns only).

The board plays this back over UDP on a BOOT short-press (see
components/peripherals/mock_playback.cpp). Only the 10 IMU fields that fit the
52-byte UDP packet are embedded; the CSV's EEG columns are dropped.

SYNTHETIC IMPACTS
-----------------
The source capture is someone standing still: its high-g resultant never leaves
1 g, so playing it back demonstrates nothing about head-impact detection. This
script splices in a few synthetic impacts (see IMPACT_SPLICES) so the mock
exercises the whole pipeline end to end.

They are generated here rather than edited into the CSV on purpose: the CSV is a
real capture shared with the receiver's replay tests, and a file that is part
real and part fabricated is a trap for whoever reads it next.

Because a head impact lasts ~15 ms and the capture is sampled every 255 ms, an
impact CANNOT be one row at the base cadence — that would claim a 255 ms
contact. So each row now carries its own `dt_ms` (the delay until the next row)
and each spliced impact is a short burst sampled at IMPACT_STEP_MS, giving the
detector a real rise/peak/decay to peak-hold over.

Runs two ways:
  - standalone:            python3 scripts/gen_head_mock.py
  - PlatformIO pre-script: extra_scripts = pre:scripts/gen_head_mock.py

Idempotent: the header embeds the source CSV's sha256 and is only rewritten
when the CSV changes. If the CSV is missing but a header exists, the existing
header is kept (so builds don't require the sibling rpi-receiver checkout).
Override the CSV location with the HEAD_MOCK_CSV env var.
"""
from __future__ import annotations

import csv
import hashlib
import os
import sys
from pathlib import Path

# Under PlatformIO this runs as an SCons pre-script: __file__ is undefined and
# the project root comes from the build environment instead.
try:
    Import("env")  # type: ignore[name-defined]  # noqa: F821
    PROJECT_DIR = Path(env["PROJECT_DIR"])  # type: ignore[name-defined]  # noqa: F821
except NameError:
    PROJECT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_CSV = (PROJECT_DIR.parent / "rpi-receiver" / "mock-csv"
               / "10_squats_clean_biometric_data_simulation" / "HEAD_Head_main.csv")
# Lives inside the peripherals component: mock_playback.cpp is its only consumer.
HEADER_PATH = PROJECT_DIR / "components" / "peripherals" / "head_mock_data.h"

IMU_FIELDS = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
              "hx_g", "hy_g", "hz_g", "imu_temp_c")

# Bump when the splice or the emitted layout changes: the header is otherwise
# only regenerated when the source CSV's hash changes, so a generator edit alone
# would silently keep the stale header.
GENERATOR_VERSION = 2

# Synthetic impacts: (row index to splice before, peak g, label).
# Peaks straddle the app's display buckets (<40 light, 40-60 moderate, >=60
# severe) so the demo shows all three, and the LAST one is deliberately not the
# largest — that is what distinguishes the "last impact" tile from "peak".
IMPACT_SPLICES = (
    (40, 26.0, "light"),
    (120, 64.0, "severe"),
    (200, 45.0, "moderate"),
)
IMPACT_STEP_MS = 2      # sampling inside the burst
IMPACT_PULSE_MS = 16    # half-sine contact duration, a plausible head impact
# A real impact spins the head as well as decelerating it. Kept under the
# i16 x10 wire scale's 3276.7 dps ceiling.
IMPACT_PEAK_DPS = 1500.0
# The +/-16 g low-g accelerometer rails during a real impact; the high-g channel
# is the one that stays meaningful, which is why detection reads it.
LOW_G_RAIL = 16.0

# The head node has no biometrics, so the mock borrows them from the chest (ECG:
# heart rate, respiration, HRV) and wrist (PPG: SpO2) reference files in the same
# exercise folder, merged by row. Resting defaults are used if they're missing.
BIO_SOURCES = (
    ("hr", "WA_Chest.csv", "ecg_hr_bpm", 92.0),
    ("spo2", "WD_L_Wrist.csv", "ppg_spo2_pct", 97.0),
    ("resp", "WA_Chest.csv", "resp_rate_bpm", 20.0),
    ("hrv", "WA_Chest.csv", "ecg_rmssd_ms", 44.0),
)


def sha_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def float_literal(v: float) -> str:
    s = f"{v:g}"
    if "." not in s and "e" not in s:
        s += ".0"  # "0f" is not a valid C literal; "0.0f" is
    return s + "f"


def impact_burst(base_row, peak_g: float):
    """A half-sine contact pulse, sampled every IMPACT_STEP_MS.

    Returns rows in the same shape as the CSV's IMU tuple. The high-g channel
    carries the real waveform; the low-g channel rails at +/-16 g the way the
    hardware does, so the mock also demonstrates WHY detection reads high-g.
    """
    import math
    ax, ay, az, gx, gy, gz, hx, hy, hz, temp = base_row
    # Direction of the blow: mostly lateral with a forward component, unit-ised
    # so the resultant equals the requested peak.
    dx, dy, dz = 0.60, -0.75, 0.28
    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    dx, dy, dz = dx / norm, dy / norm, dz / norm

    rows = []
    steps = IMPACT_PULSE_MS // IMPACT_STEP_MS
    for i in range(steps + 1):
        frac = (i * IMPACT_STEP_MS) / IMPACT_PULSE_MS
        mag = peak_g * math.sin(math.pi * frac)
        spin = IMPACT_PEAK_DPS * math.sin(math.pi * frac)
        rows.append((
            # Low-g rails; it is saturated and useless during the event.
            max(-LOW_G_RAIL, min(LOW_G_RAIL, ax + mag * dx)),
            max(-LOW_G_RAIL, min(LOW_G_RAIL, ay + mag * dy)),
            max(-LOW_G_RAIL, min(LOW_G_RAIL, az + mag * dz)),
            round(gx + spin * dx, 1),
            round(gy + spin * dy, 1),
            round(gz + spin * dz, 1),
            # High-g: the channel the detector actually watches. The baseline
            # (gravity) stays under it, which is why it reads ~1 g at rest.
            round(hx + mag * dx, 3),
            round(hy + mag * dy, 3),
            round(hz + mag * dz, 3),
            temp,
        ))
    return rows


def splice_impacts(rows, dts):
    """Insert the synthetic impact bursts, returning (rows, dt_ms, notes).

    `dts[i]` is the delay AFTER row i. A burst inherits the base row's slot:
    the row it is spliced before keeps its own timing, and the burst's own
    steps run at IMPACT_STEP_MS, so total playback length is unchanged apart
    from the few ms each burst adds.
    """
    out_rows, out_dts, notes = [], [], []
    splice_at = {idx: (peak, label) for idx, peak, label in IMPACT_SPLICES}
    for i, row in enumerate(rows):
        if i in splice_at:
            peak, label = splice_at[i]
            burst = impact_burst(row, peak)
            notes.append(f"row {len(out_rows)}: {peak:g} g ({label})")
            for b in burst:
                out_rows.append(b)
                out_dts.append(IMPACT_STEP_MS)
        out_rows.append(row)
        out_dts.append(dts[i])
    return out_rows, out_dts, notes


def existing_stamp(header: Path) -> tuple[str, str] | None:
    """(source sha, generator version) recorded in an existing header."""
    if not header.exists():
        return None
    sha = version = None
    for line in header.read_text().splitlines():
        if line.startswith("// source-sha256:"):
            sha = line.split(":", 1)[1].strip()
        elif line.startswith("// generator-version:"):
            version = line.split(":", 1)[1].strip()
    return (sha, version) if sha else None


def _read_bio_column(csv_dir: Path, filename: str, column: str):
    path = csv_dir / filename
    if not path.exists():
        return None
    with path.open(newline="") as handle:
        return [float(row[column]) for row in csv.DictReader(handle)]


def generate(csv_path: Path, header: Path) -> None:
    rows = []
    t_values = []
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append(tuple(float(row[f]) for f in IMU_FIELDS))
            t_values.append(float(row["t_s"]))
    if len(rows) < 2:
        raise SystemExit(f"{csv_path} has too few rows ({len(rows)})")
    cadence_ms = round((t_values[1] - t_values[0]) * 1000)
    # Per-row delay to the NEXT row. Uniform in the capture, but the spliced
    # impact bursts run far faster, so playback can no longer use one period.
    dts = [max(1, round((t_values[i + 1] - t_values[i]) * 1000))
           for i in range(len(rows) - 1)]
    dts.append(cadence_ms)

    # Merge biometrics from the sibling node files (defaults when absent).
    bio_cols = []
    for _key, filename, column, default in BIO_SOURCES:
        col = _read_bio_column(csv_path.parent, filename, column)
        bio_cols.append((col, default))

    # Biometrics are indexed against the ORIGINAL rows, so resolve them before
    # splicing and let each burst inherit its base row's values (a 16 ms impact
    # does not change anyone's heart rate).
    bio_rows = [tuple(col[i] if col and i < len(col) else default
                      for col, default in bio_cols)
                for i in range(len(rows))]
    splice_at = {idx for idx, _peak, _label in IMPACT_SPLICES}
    spliced_bio = []
    for i in range(len(rows)):
        if i in splice_at:
            spliced_bio.extend([bio_rows[i]] * (IMPACT_PULSE_MS // IMPACT_STEP_MS + 1))
        spliced_bio.append(bio_rows[i])

    rows, dts, notes = splice_impacts(rows, dts)
    assert len(rows) == len(spliced_bio) == len(dts), (len(rows), len(spliced_bio), len(dts))

    lines = [
        "// Auto-generated by scripts/gen_head_mock.py — do not edit.",
        f"// source: {csv_path.name} + chest/wrist biometrics",
        f"// source-sha256: {sha_of(csv_path)}",
        f"// generator-version: {GENERATOR_VERSION}",
        "//",
        "// SYNTHETIC head impacts spliced in by the generator (the source",
        "// capture never exceeds 1 g, so it demonstrates nothing on its own):",
    ]
    lines += [f"//   {n}" for n in notes]
    lines += [
        f"// Each is a {IMPACT_PULSE_MS} ms half-sine pulse sampled every",
        f"// {IMPACT_STEP_MS} ms. dt_ms is per row BECAUSE of them: an impact",
        f"// cannot be one row at the {cadence_ms} ms base cadence without",
        "// claiming a contact that lasted that long.",
        "#pragma once",
        "",
        f"#define HEAD_MOCK_ROWS {len(rows)}",
        f"#define HEAD_MOCK_CADENCE_MS {cadence_ms}",
        "",
        "typedef struct {",
        "    float ax_g, ay_g, az_g;",
        "    float gx_dps, gy_dps, gz_dps;",
        "    float hx_g, hy_g, hz_g;",
        "    float imu_temp_c;",
        "    float hr, spo2, resp, hrv;  // mock biometrics (chest/wrist)",
        "    uint16_t dt_ms;             // delay until the NEXT row",
        "} head_mock_row_t;",
        "",
        "static const head_mock_row_t HEAD_MOCK_DATA[HEAD_MOCK_ROWS] = {",
    ]
    for i, values in enumerate(rows):
        formatted = ", ".join(float_literal(v) for v in values + spliced_bio[i])
        lines.append(f"    {{{formatted}, {dts[i]}}},")
    lines.append("};")
    header.write_text("\n".join(lines) + "\n")
    print(f"gen_head_mock: wrote {header} ({len(rows)} rows @ {cadence_ms} ms "
          f"base, {len(notes)} spliced impacts, IMU + bio)")


def main() -> None:
    csv_path = Path(os.environ.get("HEAD_MOCK_CSV", DEFAULT_CSV))
    if not csv_path.exists():
        if HEADER_PATH.exists():
            print(f"gen_head_mock: {csv_path} not found — keeping existing "
                  f"{HEADER_PATH.name}", file=sys.stderr)
            return
        raise SystemExit(
            f"gen_head_mock: {csv_path} not found and no {HEADER_PATH} exists. "
            f"Check out rpi-receiver next to P1_Embedded_Software or set "
            f"HEAD_MOCK_CSV to a HEAD node CSV.")
    if existing_stamp(HEADER_PATH) == (sha_of(csv_path), str(GENERATOR_VERSION)):
        return  # up to date; keep the build quiet
    generate(csv_path, HEADER_PATH)


main()
