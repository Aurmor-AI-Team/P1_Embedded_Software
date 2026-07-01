"""Batch workout gesture analysis: recognize → count → score → time a repeated
gesture against a reference, from a recorded Aurmor session.

Assumption (agreed up front): the user is *attempting the known reference
exercise*, so this is few-shot **template matching** (DTW) plus
**peak-detection rep counting** — no trained classifier, no dataset. A reference
session of a few reps is enough to build a template and an accept threshold.

Feature sources share one analysis core. The core (segment → per-rep feature →
DTW → score → time) is source-agnostic: it consumes a 1-D segmentation signal +
an (N, D) feature matrix, so sources differ only in their loader/feature builder:
  * **IMU**   — raw accel/gyro of the prime body node (load_session_csv / ndjson).
  * **pose**  — per-joint flexion angles from the IK skeleton (load_pose_csv).
  * **video** — per-joint angles from a pose estimator (see video_pose.py); any
                non-"imu" Session routes through the generic joint-angle path.

Wire-format field names/units mirror ble-sender/protocol.py; the pose CSV layout
mirrors ble-sender/pose.py. DTW here is a small dependency-free numpy DP — for
production swap in tslearn / dtaidistance (faster, plus DTW-barycenter templates).
"""
from __future__ import annotations

import glob
import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
from scipy.signal import detrend, find_peaks

# --------------------------------------------------------------------------- #
# Common in-memory form
# --------------------------------------------------------------------------- #


@dataclass
class Session:
    """A recorded session as a shared time base + named 1-D channels.

    For IMU, channels are keyed ``"<NODE>.<col>"`` (e.g. ``"WF.gz_dps"``). For
    pose/video, channels are keyed by joint (e.g. ``"j4_angle"`` or
    ``"left_knee"``), each a per-frame angle in radians.
    """

    t: np.ndarray                       # (N,) seconds, ascending
    channels: Dict[str, np.ndarray]     # name -> (N,) float
    source: str = "imu"
    meta: dict = field(default_factory=dict)

    @property
    def fs(self) -> float:
        """Median sample rate (Hz)."""
        dt = np.median(np.diff(self.t))
        return float(1.0 / dt) if dt > 0 else 0.0


# SMPL 24-joint order (mirrors export_skeleton.py). Index → name.
SMPL_JOINTS = [
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
    "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
    "neck", "left_collar", "right_collar", "head", "left_shoulder",
    "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    "left_hand", "right_hand",
]

# IMU axes packed into the per-rep DTW feature vector for the prime node.
_IMU_FEAT_AXES = ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]
# Axes considered when auto-picking the 1-D segmentation signal.
_IMU_SIGNAL_AXES = _IMU_FEAT_AXES


# --------------------------------------------------------------------------- #
# Loaders
# --------------------------------------------------------------------------- #


def load_session_csv(folder: str) -> Session:
    """Load an IMU session from a mock-csv folder (one CSV per body node).

    All nodes share the same ``t_s`` timeline (UWB round rate ~3.92 Hz); we take
    the time base from the first file and attach every node's numeric columns.
    """
    paths = sorted(glob.glob(os.path.join(folder, "*.csv")))
    if not paths:
        raise FileNotFoundError(f"no CSVs in {folder}")
    t_ref: Optional[np.ndarray] = None
    channels: Dict[str, np.ndarray] = {}
    for path in paths:
        # Key by the wire node *code* (what the BLE stream / NDJSON carries), not
        # the filename, so templates built from CSV interoperate with the live
        # NDJSON path. The filename's first token is that code:
        # "WC_R_Elbow.csv" -> "WC", "HEAD_Head_main.csv" -> "HEAD".
        node = os.path.splitext(os.path.basename(path))[0].split("_")[0]
        df = pd.read_csv(path)
        if "t_s" not in df.columns:
            continue
        t = df["t_s"].to_numpy(dtype=float)
        if t_ref is None:
            t_ref = t
        n = min(len(t_ref), len(t))
        for axis in _IMU_SIGNAL_AXES:
            if axis in df.columns:
                channels[f"{node}.{axis}"] = df[axis].to_numpy(dtype=float)[:n]
    assert t_ref is not None
    n = min([len(t_ref)] + [len(v) for v in channels.values()]) if channels else len(t_ref)
    channels = {k: v[:n] for k, v in channels.items()}
    return Session(t=t_ref[:n], channels=channels, source="imu",
                   meta={"folder": folder, "nodes": _nodes_from_channels(channels)})


def load_pose_csv(path: str) -> Session:
    """Load a pose session from an IK ``pose_seq.csv`` (root tran + 24 joint 3x3
    rotation matrices). Each joint becomes a flexion-angle channel ``jXX_angle``
    = the joint's local rotation magnitude in radians (``acos((trace-1)/2)``),
    which reads as flexion/extension for hinge joints (knee/elbow).
    """
    df = pd.read_csv(path)
    t = df["t_s"].to_numpy(dtype=float)
    n_joints = (df.shape[1] - 4) // 9
    channels: Dict[str, np.ndarray] = {}
    for j in range(n_joints):
        base = 4 + j * 9
        m = df.iloc[:, base:base + 9].to_numpy(dtype=float)  # (N, 9) row-major
        trace = m[:, 0] + m[:, 4] + m[:, 8]                   # r00 + r11 + r22
        cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        channels[f"j{j}_angle"] = np.arccos(cos_theta)
    return Session(t=t, channels=channels, source="pose",
                   meta={"path": path, "n_joints": n_joints})


def load_session_ndjson(lines: Sequence[str] | str) -> Tuple[Session, Optional[Session]]:
    """Parse the uploaded NDJSON session frames — **the real cloud input**.

    Each line is either a biometric sample (has ``node``) or a pose frame
    (``type == 'pose'``). Returns ``(imu_session, pose_session_or_None)``.
    """
    if isinstance(lines, str):
        lines = [ln for ln in lines.splitlines() if ln.strip()]

    imu_rows: Dict[float, Dict[str, float]] = {}
    pose_t: List[float] = []
    pose_ch: Dict[str, List[float]] = {}

    for ln in lines:
        try:
            obj = json.loads(ln)
        except json.JSONDecodeError:
            continue
        if obj.get("type") == "pose":
            t = float(obj["t_s"])
            pose_t.append(t)
            for j, q in enumerate(obj.get("q", [])):
                w = max(-1.0, min(1.0, float(q[0])))
                pose_ch.setdefault(f"j{j}_angle", []).append(2.0 * math.acos(abs(w)))
        elif "node" in obj:
            t = float(obj["t_s"])
            row = imu_rows.setdefault(t, {})
            node = obj["node"]
            for axis in _IMU_SIGNAL_AXES:
                if axis in obj and isinstance(obj[axis], (int, float)):
                    row[f"{node}.{axis}"] = float(obj[axis])

    imu_t = np.array(sorted(imu_rows), dtype=float)
    keys = sorted({k for r in imu_rows.values() for k in r})
    imu_channels = {
        k: np.array([imu_rows[t].get(k, np.nan) for t in imu_t], dtype=float) for k in keys
    }
    imu_session = Session(t=imu_t, channels=imu_channels, source="imu",
                          meta={"nodes": _nodes_from_channels(imu_channels)})

    pose_session = None
    if pose_t:
        pose_session = Session(t=np.array(pose_t, dtype=float),
                               channels={k: np.array(v, dtype=float) for k, v in pose_ch.items()},
                               source="pose")
    return imu_session, pose_session


def _nodes_from_channels(channels: Dict[str, np.ndarray]) -> List[str]:
    return sorted({k.split(".")[0] for k in channels if "." in k})


# --------------------------------------------------------------------------- #
# Signal selection (periodicity via autocorrelation)
# --------------------------------------------------------------------------- #


def _periodicity(x: np.ndarray, fs: float,
                 min_period_s: float = 0.8, max_period_s: float = 6.0) -> Tuple[float, float]:
    """Score how periodic ``x`` is. Returns ``(strength, period_s)``.

    Detrends, autocorrelates, and looks for the strongest *peak* at a lag inside a
    plausible rep period. ``strength`` in [0, 1] (autocorr peak height); higher =
    cleaner repetition. ``period_s`` is the dominant rep period in seconds.
    """
    x = np.asarray(x, dtype=float)
    if np.any(~np.isfinite(x)):
        x = np.nan_to_num(x, nan=float(np.nanmean(x)) if np.any(np.isfinite(x)) else 0.0)
    if x.std() < 1e-9 or len(x) < 8:
        return 0.0, 0.0
    x = detrend(x)
    ac = np.correlate(x, x, mode="full")[len(x) - 1:]
    ac /= ac[0]
    lo = max(1, int(min_period_s * fs))
    hi = min(len(ac) - 1, int(max_period_s * fs))
    if hi <= lo:
        return 0.0, 0.0
    window = ac[lo:hi]
    # Require a genuine autocorrelation *peak* (local max) at rep scale. A
    # monotonically decaying window means the channel just varies slowly — it has
    # no rep-period structure, so it shouldn't win signal selection.
    peaks, _ = find_peaks(window)
    if len(peaks) == 0:
        return 0.0, 0.0
    k = peaks[int(np.argmax(window[peaks]))]
    lag = lo + k
    return float(ac[lag]), float(lag / fs)


def select_signal(session: Session, override: Optional[str] = None) -> Tuple[str, np.ndarray, float, int]:
    """Pick the clearest periodic channel for rep segmentation.

    Returns ``(channel_name, oriented_signal, period_s, orient_sign)``. The
    signal is detrended and oriented so the dominant excursion is positive (one
    peak per rep). ``override`` forces a channel (e.g. ``"WF.gz_dps"``).

    If the session declares ``meta["primary_signals"]``, only those channels are
    considered — used by the video front-end to keep segmentation on joint
    *angles* (exercise-specific) rather than coordinates (which also swing during
    walking, creating false reps).
    """
    fs = session.fs
    if override:
        if override not in session.channels:
            raise ValueError(f"override channel {override!r} not in session")
        name, period = override, _periodicity(session.channels[override], fs)[1]
    else:
        primary = [c for c in session.meta.get("primary_signals", []) if c in session.channels]
        candidates = primary or list(session.channels)
        # Rank by periodicity × range-of-motion: the prime mover both repeats
        # cleanly AND swings hard. Periodicity alone favours smooth, near-static
        # joints (neck/spine) over the elbow/knee actually driving the rep.
        name, best_score, period = None, -1.0, 0.0
        for cand in candidates:
            x = session.channels[cand]
            strength, p = _periodicity(x, fs)
            rom = float(np.nanstd(x))
            score = strength * rom
            if score > best_score:
                name, best_score, period = cand, score, p
        if name is None:
            raise ValueError("no usable channel found")
    raw = detrend(np.nan_to_num(session.channels[name]))
    orient = 1 if abs(raw.max()) >= abs(raw.min()) else -1
    return name, raw * orient, period, orient


# --------------------------------------------------------------------------- #
# Rep segmentation
# --------------------------------------------------------------------------- #


@dataclass
class Rep:
    start: int          # index of valley before the peak
    peak: int
    end: int            # index of valley after the peak
    t_start: float
    t_end: float

    @property
    def duration(self) -> float:
        return self.t_end - self.t_start


def segment_reps(signal: np.ndarray, t: np.ndarray, period_s: float, fs: float) -> List[Rep]:
    """Split an oriented periodic ``signal`` into reps via peak detection.

    Peaks (one per rep) gate the count; valleys (minima) on either side give the
    rep's start/end so durations are full down-up cycles. ``distance``/
    ``prominence`` are derived from the cadence + signal spread.
    """
    if period_s <= 0:
        period_s = max(1.0, len(signal) / max(fs, 1.0) / 10.0)
    dist = max(1, int(0.6 * period_s * fs))
    prom = 0.4 * np.std(signal)
    peaks, _ = find_peaks(signal, distance=dist, prominence=prom)
    valleys, _ = find_peaks(-signal, distance=dist, prominence=prom)
    candidates: List[Tuple[Rep, float]] = []
    for i, p in enumerate(peaks):
        # Bound this rep strictly between the neighbouring peaks so windows never
        # overlap and a sparse valley can't stretch a rep across the whole set.
        prev_p = int(peaks[i - 1]) if i > 0 else 0
        next_p = int(peaks[i + 1]) if i + 1 < len(peaks) else len(signal) - 1
        before = valleys[(valleys < p) & (valleys > prev_p)]
        after = valleys[(valleys > p) & (valleys < next_p)]
        s = int(before[-1]) if len(before) else (prev_p + int(p)) // 2 if i > 0 else max(0, int(p) - dist)
        e = int(after[0]) if len(after) else (int(p) + next_p) // 2 if i + 1 < len(peaks) else min(len(signal) - 1, int(p) + dist)
        if e <= s:
            continue
        # A real rep is roughly one cadence cycle. Windows far outside that are
        # rest/baseline gaps (standing & recovery) that a stray peak stretched
        # across — not reps. Gate on duration vs the period.
        dur = float(t[e] - t[s])
        if period_s > 0 and not (0.4 * period_s <= dur <= 2.0 * period_s):
            continue
        # Rep amplitude (peak rise above the flanking valleys). Used below to drop
        # tiny fluctuations between real reps (e.g. standing pauses that still
        # produce a small local max) — those aren't reps.
        amp = float(signal[int(p)] - 0.5 * (signal[s] + signal[e]))
        candidates.append((Rep(start=s, peak=int(p), end=e,
                               t_start=float(t[s]), t_end=float(t[e])), amp))
    if not candidates:
        return []
    # Amplitude gate: keep reps whose excursion is a real fraction of the typical
    # rep, so a half-squat / standing wobble between reps doesn't get counted.
    med_amp = float(np.median([a for _, a in candidates]))
    return [r for r, a in candidates if a >= 0.4 * med_amp]


# --------------------------------------------------------------------------- #
# Per-rep feature matrix
# --------------------------------------------------------------------------- #


def _rep_rom(channel: np.ndarray, rep: Rep) -> float:
    """Range-of-motion (max−min) of a channel within a rep window."""
    seg = np.nan_to_num(channel[rep.start:rep.end + 1])
    return float(np.ptp(seg)) if len(seg) else 0.0


def feature_keys(session: Session, signal_key: str, top_k: int = 6) -> List[str]:
    """Channels packed into the per-rep DTW feature vector.

    IMU: the prime node's 6 IMU axes (rich, local to the mover). pose/video: the
    ``top_k`` most periodic joint-angle channels (always including the signal
    joint), so the gesture's shape — not just one angle — drives the match.
    """
    if session.source == "imu":
        node = signal_key.split(".")[0]
        return [f"{node}.{ax}" for ax in _IMU_FEAT_AXES if f"{node}.{ax}" in session.channels]
    fs = session.fs
    # Match on the declared primary channels (joint *angles*) when available.
    # Angles are body-size-invariant — a 110° knee bend is 110° on anyone — so a
    # different person's correct rep stays close. Coordinate channels, even
    # torso-normalized, encode proportions/stance and reintroduce person-dependence.
    primary = [c for c in session.meta.get("primary_signals", []) if c in session.channels]
    pool = primary or list(session.channels)
    scored = [(k, _periodicity(session.channels[k], fs)[0] * float(np.nanstd(session.channels[k])))
              for k in pool]
    scored.sort(key=lambda kv: kv[1], reverse=True)
    best = scored[0][1] if scored else 0.0
    # Keep only channels that genuinely repeat with the gesture (within ~0.4× of
    # the prime mover's score), capped at top_k. This drops incidental, viewpoint-
    # variable motion (arms/torso in a squat) that would otherwise inflate DTW.
    keys = [k for k, sc in scored if best > 0 and sc >= 0.4 * best][:top_k]
    if not keys:
        keys = [scored[0][0]] if scored else []
    if signal_key not in keys:
        keys = [signal_key] + keys[: top_k - 1]
    return keys


def rep_matrix(session: Session, rep: Rep, keys: List[str], length: int,
               mean: Optional[np.ndarray] = None, std: Optional[np.ndarray] = None) -> np.ndarray:
    """(length, D) feature matrix for one rep: per-channel resample to a fixed
    length, then z-normalize with the supplied (reference) stats."""
    cols = []
    src_idx = np.linspace(0.0, 1.0, rep.end - rep.start + 1)
    dst_idx = np.linspace(0.0, 1.0, length)
    for k in keys:
        seg = np.nan_to_num(session.channels[k][rep.start:rep.end + 1])
        cols.append(np.interp(dst_idx, src_idx, seg))
    m = np.stack(cols, axis=1)
    if mean is not None and std is not None:
        m = (m - mean) / std
    return m


# --------------------------------------------------------------------------- #
# DTW (dependency-free; swap for tslearn/dtaidistance in production)
# --------------------------------------------------------------------------- #


def dtw_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Length-normalized DTW distance between two (L, D) sequences, Euclidean
    local cost. O(L²·D); microseconds at L≈32-48."""
    na, nb = len(a), len(b)
    cost = np.full((na + 1, nb + 1), np.inf)
    cost[0, 0] = 0.0
    for i in range(1, na + 1):
        ai = a[i - 1]
        for j in range(1, nb + 1):
            d = np.linalg.norm(ai - b[j - 1])
            cost[i, j] = d + min(cost[i - 1, j], cost[i, j - 1], cost[i - 1, j - 1])
    return float(cost[na, nb] / (na + nb))


# --------------------------------------------------------------------------- #
# Template (reference) + analysis
# --------------------------------------------------------------------------- #


@dataclass
class Template:
    source: str
    signal_key: str
    orient: int
    feat_keys: List[str]
    length: int
    period_s: float
    mean: np.ndarray
    std: np.ndarray
    exemplars: List[np.ndarray]     # one or more (length, D) reference reps
    threshold: float                # DTW accept threshold (to the nearest exemplar)
    scale: float                    # score scale (dist at which score ≈ 50)
    n_ref_reps: int
    cadence_hz: float
    signal_rom: float = 0.0         # reference median range-of-motion of the signal channel


def fit(reference: Session, length: int = 32, override_signal: Optional[str] = None) -> Template:
    """Build a reference template + accept threshold from a few-rep session."""
    fs = reference.fs
    signal_key, signal, period, orient = select_signal(reference, override_signal)
    reps = segment_reps(signal, reference.t, period, fs)
    if len(reps) < 2:
        raise ValueError(f"reference has too few reps ({len(reps)}) to build a template")
    keys = feature_keys(reference, signal_key)

    # Normalization stats over all reference reps (per channel).
    raw_mats = [rep_matrix(reference, r, keys, length) for r in reps]
    stacked = np.concatenate(raw_mats, axis=0)
    mean = stacked.mean(axis=0)
    std = stacked.std(axis=0)
    std[std < 1e-9] = 1.0
    mats = [(m - mean) / std for m in raw_mats]

    # Medoid = rep with the smallest total DTW distance to the others.
    n = len(mats)
    dmat = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            dmat[i, j] = dmat[j, i] = dtw_distance(mats[i], mats[j])
    medoid_i = int(np.argmin(dmat.sum(axis=1)))
    ref_dists = np.array([dmat[medoid_i, j] for j in range(n) if j != medoid_i])

    med = float(np.median(ref_dists))
    mad = float(np.median(np.abs(ref_dists - med))) or (med * 0.5 + 1e-6)
    threshold = med + 3.0 * 1.4826 * mad          # robust 3-sigma-ish gate
    scale = max(threshold, med + 1e-6)            # dist where score ≈ 50

    signal_rom = float(np.median([_rep_rom(reference.channels[signal_key], r) for r in reps]))
    return Template(
        source=reference.source, signal_key=signal_key, orient=orient, feat_keys=keys,
        length=length, period_s=period, mean=mean, std=std, exemplars=[mats[medoid_i]],
        threshold=float(threshold), scale=float(scale), n_ref_reps=n,
        cadence_hz=float(1.0 / period) if period else 0.0, signal_rom=signal_rom,
    )


def fit_multi(sessions: List[Session], length: int = 48,
              override_signal: Optional[str] = None) -> Template:
    """Build a **multi-reference** template from several reference sessions.

    Pools every rep from every reference clip into one exemplar set; a test rep is
    later matched to its *nearest* exemplar. This is the few-shot way to cover
    natural variation (different people, camera viewpoints): the reference set must
    *contain* the modes you want to accept — you can only recognize variation you
    have shown. Signal + feature channels are chosen by aggregate periodicity×ROM
    across all sessions so they're consistent; the accept threshold is set from the
    spread of nearest-neighbour distances *among* the reference exemplars.
    """
    if not sessions:
        raise ValueError("no reference sessions")
    shared = set(sessions[0].channels)
    for s in sessions[1:]:
        shared &= set(s.channels)
    if not shared:
        raise ValueError("reference sessions share no channels")

    def agg(k: str) -> float:
        return float(sum(_periodicity(s.channels[k], s.fs)[0] * np.nanstd(s.channels[k])
                         for s in sessions))

    # Restrict both the segmentation signal and the DTW features to the declared
    # primary (joint-angle) channels: angles are exercise-specific (vs coordinate
    # channels that swing in walking) and body-size-invariant (vs coordinates that
    # encode proportions). This is what keeps a different person's correct rep close.
    primary = set()
    for s in sessions:
        primary |= set(s.meta.get("primary_signals", []))
    pool = [k for k in shared if k in primary] or list(shared)
    scored = sorted(((k, agg(k)) for k in pool), key=lambda kv: kv[1], reverse=True)
    best = scored[0][1] if scored else 0.0
    signal_key = override_signal or scored[0][0]
    keys = [k for k, sc in scored if best > 0 and sc >= 0.4 * best][:6]
    if signal_key not in keys:
        keys = [signal_key] + keys[:5]

    periods: List[float] = []
    seg = []
    for s in sessions:
        _, sig, per, _ = select_signal(s, signal_key)
        if per > 0:
            periods.append(per)
        seg.append((s, sig, per))
    period = float(np.median(periods)) if periods else 2.0

    raw_mats: List[np.ndarray] = []
    roms: List[float] = []
    for s, sig, per in seg:
        reps = segment_reps(sig, s.t, per if per > 0 else period, s.fs)
        for r in reps:
            raw_mats.append(rep_matrix(s, r, keys, length))
            roms.append(_rep_rom(s.channels[signal_key], r))
    if len(raw_mats) < 2:
        raise ValueError(f"too few reference reps across sessions ({len(raw_mats)})")

    stacked = np.concatenate(raw_mats, axis=0)
    mean = stacked.mean(axis=0)
    std = stacked.std(axis=0)
    std[std < 1e-9] = 1.0
    exemplars = [(m - mean) / std for m in raw_mats]

    # Threshold from how close each reference rep sits to its *nearest other*
    # exemplar — i.e. the natural good-to-good spread of the reference set.
    n = len(exemplars)
    nn = np.array([min(dtw_distance(exemplars[i], exemplars[j]) for j in range(n) if j != i)
                   for i in range(n)])
    med = float(np.median(nn))
    mad = float(np.median(np.abs(nn - med))) or (med * 0.5 + 1e-6)
    threshold = med + 3.0 * 1.4826 * mad
    scale = max(threshold, med + 1e-6)

    return Template(
        source=sessions[0].source, signal_key=signal_key, orient=1, feat_keys=keys,
        length=length, period_s=period, mean=mean, std=std, exemplars=exemplars,
        threshold=float(threshold), scale=float(scale), n_ref_reps=n,
        cadence_hz=float(1.0 / period) if period else 0.0,
        signal_rom=float(np.median(roms)) if roms else 0.0,
    )


def _score(dist: float, scale: float) -> float:
    """Map a DTW distance to a 0–100 similarity score: 100 at 0, ~50 at ``scale``."""
    return float(max(0.0, min(100.0, 100.0 * 0.5 ** (dist / scale)))) if scale > 0 else 0.0


def analyze(test: Session, template: Template) -> dict:
    """Recognize + count + score + time reps in ``test`` against ``template``.

    Recognition collapses into the similarity test (gesture is assumed to be the
    reference): a rep is *recognized* iff its DTW distance < the template
    threshold. Returns a JSON-able summary.
    """
    # Rep features read raw channel values (orientation-independent), so only the
    # 1-D segmentation signal needs orienting — select_signal handles that per-session.
    fs = test.fs
    # Segment the test on ITS OWN cadence, not the reference's. The reference
    # defines the good-rep *shape* (the exemplars); it must not dictate the test's
    # tempo — otherwise a reference clip with an off cadence (e.g. one containing
    # walking) breaks rep detection for every test video. Fall back to the
    # reference period only if the test's own cadence can't be estimated.
    name, signal, own_period, _ = select_signal(test, template.signal_key)
    period = own_period if own_period > 0 else template.period_s
    reps = segment_reps(signal, test.t, period, fs)

    # Depth gate: a real rep moves the prime joint a real fraction of the
    # reference's range. z-normalized DTW alone can't tell a deep squat from a
    # shallow bob (it rescales amplitude away), so a small-amplitude candidate
    # (a standing wobble / walking step) is dropped here before scoring.
    reps = [r for r in reps
            if template.signal_rom <= 0
            or _rep_rom(test.channels[name], r) >= 0.5 * template.signal_rom]

    rep_out: List[dict] = []
    recognized = 0
    active = 0.0
    scores: List[float] = []
    durations: List[float] = []
    for idx, r in enumerate(reps):
        m = rep_matrix(test, r, template.feat_keys, template.length, template.mean, template.std)
        dist = min(dtw_distance(m, ex) for ex in template.exemplars)  # nearest exemplar
        ok = dist < template.threshold
        sc = _score(dist, template.scale)
        if ok:
            recognized += 1
            active += r.duration
            scores.append(sc)
            durations.append(r.duration)
        rep_out.append({
            "index": idx, "t_start": round(r.t_start, 3), "t_end": round(r.t_end, 3),
            "duration_s": round(r.duration, 3), "dtw_distance": round(dist, 4),
            "score": round(sc, 1), "recognized": ok,
        })

    return {
        "source": template.source,
        "signal_channel": name,
        "cadence_hz": round(template.cadence_hz, 3),
        "sample_rate_hz": round(fs, 2),
        "reference": {"n_reps": template.n_ref_reps, "accept_threshold": round(template.threshold, 4)},
        "test": {
            "candidate_reps": len(reps),
            "recognized_reps": recognized,
            "rejected_reps": len(reps) - recognized,
            "total_time_s": round(float(test.t[-1] - test.t[0]), 3) if len(test.t) else 0.0,
            "active_time_s": round(active, 3),
            "avg_score": round(float(np.mean(scores)), 1) if scores else 0.0,
            "avg_rep_time_s": round(float(np.mean(durations)), 3) if durations else 0.0,
            "reps": rep_out,
        },
    }
