"""Load the mock biometric CSVs into a time-ordered list of frames.

No BLE imports here either, so this runs anywhere (including the macOS dry-run).
Every node shares the same ``t_s`` timeline, so rows are merged by timestamp:
one *frame* = every node's sample at a given ``t_s``.
"""
from __future__ import annotations

import csv
from collections import namedtuple
from pathlib import Path
from typing import List, Tuple

from protocol import encode_sample

# samples: list of (node_id, sample_dict), sorted by node for determinism.
Frame = namedtuple("Frame", ["t_s", "samples"])


def load_frames(data_dir) -> Tuple[List[Frame], List[str]]:
    """Read every ``*.csv`` in ``data_dir`` and return ``(frames, node_ids)``."""
    data_dir = Path(data_dir)
    csv_files = sorted(data_dir.glob("*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"no CSV files found in {data_dir}")

    by_ts: dict = {}
    nodes = set()
    for path in csv_files:
        with path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                node = (row.get("node") or path.stem.split("_")[0]).strip()
                t_s = round(float(row["t_s"]), 3)
                by_ts.setdefault(t_s, {})[node] = encode_sample(row)
                nodes.add(node)

    frames = [Frame(t_s, sorted(by_ts[t_s].items())) for t_s in sorted(by_ts)]
    return frames, sorted(nodes)


def period_ms(frames: List[Frame]) -> int:
    """Inter-frame period in milliseconds (derived from the first two frames)."""
    if len(frames) >= 2:
        return round((frames[1].t_s - frames[0].t_s) * 1000)
    return 0
