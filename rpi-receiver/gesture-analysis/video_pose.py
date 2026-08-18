"""Video front-end for the gesture-analysis core.

Turns a workout video into the same ``Session`` object the IMU/pose pipeline
already consumes, so `fit()`/`analyze()` in gesture_analysis.py run **unchanged**:

    video.mp4 ─► MediaPipe Pose ─► 3D world landmarks ─► joint angles ─► Session

Why joint angles (not raw landmarks): an angle at the knee/elbow is inherently
scale-, translation-, and (using MediaPipe's 3D *world* landmarks) largely
viewpoint-invariant. So a reference clip filmed at a different distance/angle
still compares fairly — exactly the property we relied on for the IMU path.

MediaPipe is a *keypoint detector*: it returns the positions of 33 body points
per frame; the joint angles below are computed here with plain trig. Prototype
backend = `mediapipe` Tasks `PoseLandmarker` (CPU). Production can swap in
YOLO11-pose / RTMPose (GPU) or a 3D/SMPL model — only this file changes; the
analysis core does not.

The Tasks API needs a downloaded model bundle (`pose_landmarker_*.task`). Place it
at ``models/pose_landmarker_full.task`` (see README) or pass ``model_path=``.
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.signal import butter, filtfilt

from gesture_analysis import Session

_DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "models", "pose_landmarker_full.task")

# MediaPipe Pose landmark indices (BlazePose 33-point topology).
_L = {
    "l_shoulder": 11, "r_shoulder": 12, "l_elbow": 13, "r_elbow": 14,
    "l_wrist": 15, "r_wrist": 16, "l_hip": 23, "r_hip": 24,
    "l_knee": 25, "r_knee": 26, "l_ankle": 27, "r_ankle": 28,
}

# Joint angle = angle at B in the triple (A, B, C). We emit **bilateral** channels
# (a visibility-aware mean of the left & right side) rather than separate L/R, so
# the signal is side- and viewpoint-agnostic: in a side view the occluded far limb
# is dropped and the near one is used; in a frontal view both average. Without
# this, a template keyed on (say) right_knee fails to match a clip filmed from the
# other side / more frontally — the real failure we saw on the sample clips.
_BILATERAL: Dict[str, Tuple[Tuple[str, str, str], Tuple[str, str, str]]] = {
    "knee":     (("l_hip", "l_knee", "l_ankle"),       ("r_hip", "r_knee", "r_ankle")),
    "hip":      (("l_shoulder", "l_hip", "l_knee"),    ("r_shoulder", "r_hip", "r_knee")),
    "elbow":    (("l_shoulder", "l_elbow", "l_wrist"), ("r_shoulder", "r_elbow", "r_wrist")),
    "shoulder": (("l_elbow", "l_shoulder", "l_hip"),   ("r_elbow", "r_shoulder", "r_hip")),
}

# Distal joints whose **canonical-frame position** we track (bilateral). Joint
# angles are already rotation-invariant, so they don't benefit from a canonical
# frame — but coordinates do: expressing a joint in a body-fixed frame makes its
# trajectory the same whether the camera is to the left, right, or front. We keep
# the vertical ("up", along the spine) and forward components, torso-normalized.
_COORD_JOINTS: Dict[str, Tuple[str, str]] = {
    "knee":  ("l_knee", "r_knee"),
    "ankle": ("l_ankle", "r_ankle"),
    "wrist": ("l_wrist", "r_wrist"),
    "elbow": ("l_elbow", "r_elbow"),
}

_VIS_THRESHOLD = 0.5      # landmark visibility below this is treated as missing


def _side_angle(pts: np.ndarray, vis: np.ndarray, tri: Tuple[str, str, str]) -> float:
    """Angle for one side, or NaN if any of its 3 landmarks is low-visibility."""
    ia, ib, ic = _L[tri[0]], _L[tri[1]], _L[tri[2]]
    if min(vis[ia], vis[ib], vis[ic]) < _VIS_THRESHOLD:
        return np.nan
    return _angle(pts[ia], pts[ib], pts[ic])


def _body_frame(pts: np.ndarray, vis: np.ndarray):
    """Canonical body coordinate frame from the torso landmarks.

    Returns ``(R, pelvis, torso_len)`` where ``R``'s rows are the body axes in
    world coords — right (l→r hip), up (pelvis→neck), forward (up×right) — or
    None if the torso isn't well-tracked. Rotating world points by ``R`` makes
    them invariant to where the camera sits relative to the person.
    """
    req = [_L["l_hip"], _L["r_hip"], _L["l_shoulder"], _L["r_shoulder"]]
    if min(vis[i] for i in req) < _VIS_THRESHOLD:
        return None
    pelvis = 0.5 * (pts[_L["l_hip"]] + pts[_L["r_hip"]])
    neck = 0.5 * (pts[_L["l_shoulder"]] + pts[_L["r_shoulder"]])
    up = neck - pelvis
    torso = float(np.linalg.norm(up))
    if torso < 1e-6:
        return None
    up = up / torso
    right = pts[_L["r_hip"]] - pts[_L["l_hip"]]
    right = right - np.dot(right, up) * up           # orthogonalize against up
    nr = np.linalg.norm(right)
    if nr < 1e-6:
        return None
    right = right / nr
    forward = np.cross(up, right)
    return np.stack([right, up, forward]), pelvis, torso


def _canon_side(pts: np.ndarray, vis: np.ndarray, idx: int, frame) -> Optional[np.ndarray]:
    """One landmark in the canonical body frame (torso-normalized), or None."""
    if frame is None or vis[idx] < _VIS_THRESHOLD:
        return None
    R, pelvis, torso = frame
    return R @ (pts[idx] - pelvis) / torso           # [right, up, forward]


def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """Angle at vertex ``b`` (radians) formed by points a-b-c."""
    ba, bc = a - b, c - b
    nba, nbc = np.linalg.norm(ba), np.linalg.norm(bc)
    if nba < 1e-9 or nbc < 1e-9:
        return np.nan
    cos = np.clip(np.dot(ba, bc) / (nba * nbc), -1.0, 1.0)
    return float(np.arccos(cos))


def _torso_lean(pts: np.ndarray, vis: np.ndarray) -> float:
    """Angle of the torso (hip-midpoint → shoulder-midpoint) from vertical."""
    if min(vis[_L["l_shoulder"]], vis[_L["r_shoulder"]],
           vis[_L["l_hip"]], vis[_L["r_hip"]]) < _VIS_THRESHOLD:
        return np.nan
    sh = (pts[_L["l_shoulder"]] + pts[_L["r_shoulder"]]) / 2.0
    hip = (pts[_L["l_hip"]] + pts[_L["r_hip"]]) / 2.0
    v = sh - hip
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.nan
    # Angle from the world vertical (MediaPipe world y axis points down).
    cos = np.clip(abs(v[1]) / n, -1.0, 1.0)
    return float(np.arccos(cos))


def _interp_nans(x: np.ndarray) -> np.ndarray:
    """Linear-interpolate NaN gaps (from low-visibility frames); edge-fill ends."""
    x = x.copy()
    nan = ~np.isfinite(x)
    if nan.all():
        return np.zeros_like(x)
    idx = np.arange(len(x))
    x[nan] = np.interp(idx[nan], idx[~nan], x[~nan])
    return x


def _smooth(x: np.ndarray, fs: float, cutoff_hz: float = 4.0) -> np.ndarray:
    """Zero-phase Butterworth low-pass; video angle series are jittery at 30 fps."""
    if len(x) < 13 or fs <= 0:
        return x
    nyq = fs / 2.0
    wn = min(0.99, cutoff_hz / nyq)
    if wn <= 0:
        return x
    b, a = butter(2, wn)
    return filtfilt(b, a, x)


def load_session_video(path: str, every_n: int = 1, model_path: Optional[str] = None,
                       smooth_cutoff_hz: float = 4.0) -> Session:
    """Run MediaPipe Pose over ``path`` and return a joint-angle ``Session``.

    ``every_n`` subsamples frames (>=2 to speed up long clips). ``model_path``
    overrides the default ``models/pose_landmarker_full.task`` bundle.
    """
    import cv2                       # imported lazily so `import video_pose` is cheap
    import mediapipe as mp
    from mediapipe.tasks.python import vision
    from mediapipe.tasks.python.core.base_options import BaseOptions

    model_path = model_path or _DEFAULT_MODEL
    if not os.path.isfile(model_path):
        raise FileNotFoundError(
            f"pose model not found: {model_path}\n"
            "Download it, e.g.:\n  curl -fsSL -o models/pose_landmarker_full.task "
            "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
            "pose_landmarker_full/float16/latest/pose_landmarker_full.task")

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video: {path}")
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    eff_fps = src_fps / max(1, every_n)

    times: List[float] = []
    raw: Dict[str, List[float]] = {k: [] for k in _BILATERAL}
    raw["torso_lean"] = []
    for j in _COORD_JOINTS:                            # canonical-frame coords
        raw[f"{j}_up"] = []
        raw[f"{j}_fwd"] = []

    options = vision.PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    landmarker = vision.PoseLandmarker.create_from_options(options)
    frame_idx = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame_idx % every_n == 0:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                ts_ms = int(frame_idx / src_fps * 1000)
                res = landmarker.detect_for_video(mp_image, ts_ms)
                times.append(frame_idx / src_fps)
                if res.pose_world_landmarks:
                    lm = res.pose_world_landmarks[0]      # person 0 (num_poses=1)
                    pts = np.array([[p.x, p.y, p.z] for p in lm], dtype=float)
                    vis = np.array([p.visibility for p in lm], dtype=float)
                    for name, (left, right) in _BILATERAL.items():
                        sides = [_side_angle(pts, vis, left), _side_angle(pts, vis, right)]
                        sides = [s for s in sides if np.isfinite(s)]
                        raw[name].append(float(np.mean(sides)) if sides else np.nan)
                    raw["torso_lean"].append(_torso_lean(pts, vis))
                    # Canonical-frame coordinates (view-invariant), bilateral mean.
                    frame = _body_frame(pts, vis)
                    for j, (left, right) in _COORD_JOINTS.items():
                        cs = [_canon_side(pts, vis, _L[s], frame) for s in (left, right)]
                        cs = [c for c in cs if c is not None]
                        if cs:
                            m = np.mean(cs, axis=0)
                            raw[f"{j}_up"].append(float(m[1]))
                            raw[f"{j}_fwd"].append(float(m[2]))
                        else:
                            raw[f"{j}_up"].append(np.nan)
                            raw[f"{j}_fwd"].append(np.nan)
                else:
                    for name in raw:
                        raw[name].append(np.nan)
            frame_idx += 1
    finally:
        landmarker.close()
        cap.release()

    if len(times) < 8:
        raise ValueError(f"too few usable frames in {path} ({len(times)})")

    t = np.array(times, dtype=float)
    channels: Dict[str, np.ndarray] = {}
    for name, vals in raw.items():
        series = _interp_nans(np.array(vals, dtype=float))
        channels[name] = _smooth(series, eff_fps, smooth_cutoff_hz)

    return Session(t=t, channels=channels, source="video",
                   meta={"path": path, "src_fps": src_fps, "eff_fps": eff_fps,
                         "frames": len(times),
                         # Prefer joint *angles* for rep segmentation: they're
                         # exercise-specific (a squat bends the knee; walking
                         # barely does), unlike coordinate channels (ankle_fwd
                         # swings in walking too → false reps).
                         "primary_signals": list(_BILATERAL)})
