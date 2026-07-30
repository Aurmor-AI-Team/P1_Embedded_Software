"""Group session: 30 head sensors, one per athlete, for one training or match.

Two lifetimes that were previously conflated:

  SQUAD      the athletes themselves — names, numbers, team. Persists forever
             in roster.json, edited occasionally.
  SESSION    one training or match: who turned up, which sensor each person
             was handed, when it started and stopped, and what happened.
             A different helmet every week is normal.

The thing this module exists for
--------------------------------
**"0 impacts" and "the sensor was offline" must never look the same.**

A per-athlete impact count on its own is dangerously ambiguous. If a sensor
dropped off the network for twenty minutes, any hit in that window was never
recorded, and a report showing "0 impacts" for that athlete is actively
misleading — it reads as "nothing happened" when it means "we weren't looking".

So every session tracks per-athlete COVERAGE: the fraction of session time the
receiver could actually hear that athlete's sensor, plus each individual gap.
The report states coverage first and the impact count second, and an athlete
below the coverage floor is reported as UNRELIABLE rather than as zero.

Stdlib only, same as the rest of the receiver.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import roster as roster_mod

# A device is considered "heard" if a packet arrived within this window. The
# firmware HELLOs every 2 s even in ALERTS mode with no telemetry at all, so
# 6 s tolerates two consecutive lost keepalives before declaring a gap.
HEARD_WINDOW_S = 6.0

# Gaps shorter than this are noise (a missed beacon, a body blocking the
# antenna for a moment) and are not worth reporting individually.
MIN_GAP_S = 5.0

# Coverage below this makes an athlete's impact count untrustworthy.
COVERAGE_FLOOR = 0.95

STATUS_MONITORED = "monitored"
STATUS_PARTIAL = "partial"
STATUS_NO_DATA = "no-data"
STATUS_UNASSIGNED = "unassigned"


class AthleteCoverage:
    """Liveness accounting for one athlete's sensor over one session."""

    __slots__ = ("athlete_id", "name", "number", "wid",
                 "covered_s", "sampled_s", "gaps", "_gap_start", "first_seen")

    def __init__(self, athlete_id: str, name: str, number, wid: Optional[int]):
        self.athlete_id = athlete_id
        self.name = name
        self.number = number
        self.wid = wid
        self.covered_s = 0.0
        self.sampled_s = 0.0
        self.gaps: List[dict] = []
        self._gap_start: Optional[float] = None
        self.first_seen: Optional[float] = None

    def sample(self, now: float, dt: float, heard: bool) -> None:
        self.sampled_s += dt
        if heard:
            if self.first_seen is None:
                self.first_seen = now
            self.covered_s += dt
            if self._gap_start is not None:
                self._close_gap(now)
        elif self._gap_start is None:
            self._gap_start = now

    def _close_gap(self, now: float) -> None:
        length = now - self._gap_start
        if length >= MIN_GAP_S:
            self.gaps.append({"start_s": round(self._gap_start, 1),
                              "end_s": round(now, 1),
                              "length_s": round(length, 1)})
        self._gap_start = None

    def finalise(self, now: float) -> None:
        if self._gap_start is not None:
            self._close_gap(now)

    @property
    def coverage(self) -> float:
        return (self.covered_s / self.sampled_s) if self.sampled_s else 0.0

    def status(self) -> str:
        if self.wid is None:
            return STATUS_UNASSIGNED
        if self.first_seen is None:
            return STATUS_NO_DATA
        return STATUS_MONITORED if self.coverage >= COVERAGE_FLOOR else STATUS_PARTIAL

    def as_dict(self) -> dict:
        longest = max((g["length_s"] for g in self.gaps), default=0.0)
        return {
            "athlete_id": self.athlete_id,
            "athlete": self.name,
            "number": self.number,
            "wid": self.wid,
            "status": self.status(),
            "coverage": round(self.coverage, 4),
            "coverage_pct": round(self.coverage * 100, 1),
            "gaps": self.gaps,
            "gap_count": len(self.gaps),
            "longest_gap_s": longest,
            "monitored_s": round(self.covered_s, 1),
        }


class GroupSession:
    """One training or match, across a squad of one-sensor-per-athlete.

    ``tick()`` is called from the BLE sender's timer; it samples which sensors
    the receiver can currently hear and accumulates coverage. Everything else
    is bookkeeping around it.
    """

    def __init__(self, roster: roster_mod.Roster, store,
                 log_dir: Optional[Path] = None):
        self._lock = threading.Lock()
        self._roster = roster
        self._store = store
        self._log_dir = Path(log_dir) if log_dir else None
        self.name: Optional[str] = None
        self.started_at: Optional[float] = None     # wall clock
        self._started_mono: Optional[float] = None
        self.ended_at: Optional[float] = None
        self._last_tick: Optional[float] = None
        self._coverage: Dict[str, AthleteCoverage] = {}
        self._present: List[str] = []

    # -- lifecycle ----------------------------------------------------------- #
    @property
    def active(self) -> bool:
        return self._started_mono is not None and self.ended_at is None

    @property
    def elapsed_s(self) -> float:
        if self._started_mono is None:
            return 0.0
        end = self._last_tick or time.monotonic()
        return max(end - self._started_mono, 0.0)

    def start(self, name: Optional[str] = None,
              present: Optional[List[str]] = None,
              now: Optional[float] = None) -> dict:
        """Begin a session.

        ``present`` limits it to the athletes who actually turned up; the
        default is everyone in the squad who has a device assigned. Athletes in
        the squad WITHOUT a device are still listed, as ``unassigned`` — a
        person nobody handed a sensor to is exactly what a pre-session check
        needs to surface.
        """
        with self._lock:
            self.name = name or time.strftime("%Y%m%dT%H%M%S")
            self.started_at = time.time()
            self._started_mono = time.monotonic() if now is None else now
            self._last_tick = self._started_mono
            self.ended_at = None
            self._coverage.clear()

            athletes = self._roster.athletes()
            chosen = ([a for a in athletes if a.id in set(present)]
                      if present is not None else athletes)
            self._present = [a.id for a in chosen]
            for a in chosen:
                wids = a.head_wids() or sorted(a.devices)
                self._coverage[a.id] = AthleteCoverage(
                    a.id, a.name, a.number, wids[0] if wids else None)

            self._store.new_session(self.name)
            unassigned = [c.name for c in self._coverage.values() if c.wid is None]
            return {
                "session": self.name,
                "athletes": len(self._coverage),
                "with_sensor": sum(1 for c in self._coverage.values()
                                   if c.wid is not None),
                "without_sensor": unassigned,
            }

    def tick(self, source, now: Optional[float] = None) -> None:
        """Sample sensor liveness. Cheap; call once a second or so.

        ``now`` is injectable so coverage accounting can be tested against a
        simulated clock instead of wall time — a dropout that only shows up
        after twenty real minutes is a dropout nobody will ever write a test
        for.
        """
        if not self.active:
            return
        now = time.monotonic() if now is None else now
        with self._lock:
            dt = now - (self._last_tick or now)
            self._last_tick = now
            if dt <= 0:
                return
            heard = set(source.active_wearables(max_age_s=HEARD_WINDOW_S))
            elapsed = now - self._started_mono
            for cov in self._coverage.values():
                cov.sample(elapsed, dt, cov.wid in heard)

    def end(self, now: Optional[float] = None) -> dict:
        """Close the session and produce the report."""
        with self._lock:
            if self._started_mono is None:
                return {"error": "no session running"}
            now = time.monotonic() if now is None else now
            elapsed = now - self._started_mono
            for cov in self._coverage.values():
                cov.finalise(elapsed)
            self.ended_at = time.time()
        report = self.report()
        self._write(report)
        return report

    # -- reporting ----------------------------------------------------------- #
    def report(self) -> dict:
        with self._lock:
            covs = list(self._coverage.values())
        impacts = {}
        if self._store is not None:
            for a in self._store.athletes():
                impacts[a["athlete_id"]] = a

        rows = []
        for cov in sorted(covs, key=lambda c: (c.name.lower(), c.athlete_id)):
            row = cov.as_dict()
            hit = impacts.get(cov.athlete_id, {})
            row["head_impacts"] = hit.get("head_impacts", 0)
            row["head_peak_g"] = hit.get("head_peak_g", 0.0)
            # The whole point: an impact count is only meaningful if we were
            # actually listening. Say so in the row itself so no downstream
            # consumer has to remember to cross-check coverage.
            row["count_reliable"] = row["status"] == STATUS_MONITORED
            rows.append(row)

        monitored = sum(1 for r in rows if r["status"] == STATUS_MONITORED)
        partial = sum(1 for r in rows if r["status"] == STATUS_PARTIAL)
        nodata = sum(1 for r in rows if r["status"] == STATUS_NO_DATA)
        unassigned = sum(1 for r in rows if r["status"] == STATUS_UNASSIGNED)
        return {
            "session": self.name,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "duration_s": round(self.elapsed_s, 1),
            "active": self.active,
            "athletes": len(rows),
            "monitored": monitored,
            "partial": partial,
            "no_data": nodata,
            "unassigned": unassigned,
            "total_head_impacts": sum(r["head_impacts"] for r in rows),
            "coverage_floor": COVERAGE_FLOOR,
            "rows": rows,
        }

    def _write(self, report: dict) -> None:
        if self._log_dir is None:
            return
        try:
            self._log_dir.mkdir(parents=True, exist_ok=True)
            path = self._log_dir / f"session-{self.name}.json"
            path.write_text(json.dumps(report, indent=2) + "\n")
            print(f"# session report -> {path}", flush=True)
        except OSError as exc:
            print(f"# session: could not write report ({exc})", flush=True)

    def text_report(self) -> str:
        """Human-readable summary, for the console and for a printed handover."""
        r = self.report()
        out = [
            f"Session {r['session']}  ({r['duration_s']:.0f} s, "
            f"{r['athletes']} athlete(s))",
            f"  monitored {r['monitored']}   partial {r['partial']}   "
            f"no-data {r['no_data']}   unassigned {r['unassigned']}",
            f"  total head impacts: {r['total_head_impacts']}",
            "",
            f"  {'athlete':<22} {'wid':>4} {'cov':>6} {'gaps':>5} "
            f"{'impacts':>8}  status",
        ]
        for row in r["rows"]:
            flag = "" if row["count_reliable"] else "  <-- count unreliable"
            wid = str(row["wid"]) if row["wid"] is not None else "-"
            out.append(f"  {row['athlete'][:22]:<22} {wid:>4} "
                       f"{row['coverage_pct']:>5.1f}% {row['gap_count']:>5} "
                       f"{row['head_impacts']:>8}  {row['status']}{flag}")
        return "\n".join(out)