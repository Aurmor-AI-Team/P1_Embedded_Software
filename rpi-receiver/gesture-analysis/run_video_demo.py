#!/usr/bin/env python3
"""Demo CLI for the video gesture-analysis prototype.

Builds a reference template from one workout clip and evaluates another, using
MediaPipe pose → joint angles → the existing DTW core. Mirrors run_demo.py (IMU).

Drop clips in ./clips and run, e.g.:

  # Same exercise: expect recognized reps ≈ visible reps, high scores.
  python run_video_demo.py --reference clips/squats_ref.mp4 --test clips/squats_test.mp4

  # Cross-exercise rejection: reps fall past the threshold (low scores).
  python run_video_demo.py --reference clips/squats_ref.mp4 --test clips/pushups.mp4

  # 30 fps clips: reps span ~60-90 frames, so a longer DTW length helps.
  python run_video_demo.py --reference ref.mp4 --test test.mp4 --length 48 --every-n 1 --json
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import gesture_analysis as ga
from video_pose import load_session_video


def _print_summary(s: dict) -> None:
    t = s["test"]
    print(f"\n  ── VIDEO ── (signal joint: {s['signal_channel']}, "
          f"{s['sample_rate_hz']} fps, cadence {s['cadence_hz']} Hz, {s['analysis_ms']} ms)")
    print(f"     reference reps: {s['reference']['n_reps']}   "
          f"accept threshold: {s['reference']['accept_threshold']}")
    print(f"     recognized: {t['recognized_reps']}/{t['candidate_reps']}   "
          f"rejected: {t['rejected_reps']}   avg score: {t['avg_score']}")
    print(f"     avg rep time: {t['avg_rep_time_s']} s   "
          f"active: {t['active_time_s']} s / {t['total_time_s']} s total")
    rows = ", ".join(
        f"#{r['index']} {r['duration_s']}s/{r['score']}{'' if r['recognized'] else '✗'}"
        for r in t["reps"]
    )
    print(f"     reps: {rows}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reference", required=True, nargs="+",
                    help="one or more reference clips (multiple → multi-reference template)")
    ap.add_argument("--test", required=True, help="test clip to evaluate")
    ap.add_argument("--length", type=int, default=48, help="DTW resample length per rep (default 48)")
    ap.add_argument("--every-n", type=int, default=1, help="process every Nth frame (speed up long clips)")
    ap.add_argument("--model", default=None,
                    help="path to a pose_landmarker .task bundle (default models/pose_landmarker_full.task)")
    ap.add_argument("--signal", default=None, help="force the segmentation joint (e.g. left_knee)")
    ap.add_argument("--tolerance", type=float, default=1.0,
                    help="loosen/tighten the accept threshold (×). >1 admits more cross-person style variation")
    ap.add_argument("--json", action="store_true", help="also dump the full summary as JSON")
    args = ap.parse_args()

    print(f"reference = {args.reference!r}   test = {args.test!r}")

    refs = []
    for path in args.reference:
        print(f"  extracting pose (reference: {path})…")
        s = load_session_video(path, every_n=args.every_n, model_path=args.model)
        print(f"    {s.meta['frames']} frames @ {s.meta['eff_fps']:.1f} fps")
        refs.append(s)
    print("  extracting pose (test)…")
    test = load_session_video(args.test, every_n=args.every_n,
                              model_path=args.model)
    print(f"    {test.meta['frames']} frames @ {test.meta['eff_fps']:.1f} fps")

    t0 = time.perf_counter()
    if len(refs) > 1:
        template = ga.fit_multi(refs, length=args.length, override_signal=args.signal)
    else:
        template = ga.fit(refs[0], length=args.length, override_signal=args.signal)
    # Loosen the form-tolerance gate (depth gate already excludes non-squats).
    template.threshold *= args.tolerance
    template.scale *= args.tolerance
    summary = ga.analyze(test, template)
    summary["analysis_ms"] = round((time.perf_counter() - t0) * 1000, 1)

    _print_summary(summary)
    if args.json:
        print("\n" + json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
