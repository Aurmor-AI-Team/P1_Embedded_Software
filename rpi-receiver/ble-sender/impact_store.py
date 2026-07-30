"""Per-athlete impact history: aggregate, persist, and replay impact events.

The wearables are the source of truth for *detection* (the ESP32 peak-holds and
debounces); this module is the source of truth for *history*. It exists for one
requirement: an impact must survive the app not being connected. A phone goes
out of BLE range, locks, backgrounds, or the receiver reboots mid-game. None of
that may lose a recorded impact, so every event is appended to a JSONL file the
moment it arrives over UDP, before any BLE work is attempted.

Two distinctions this module is careful about:

* **Head vs body.** An athlete may wear several sensors. A wrist sensor reading
  45 g is an arm swing; a head sensor reading 45 g is the event this product
  exists to catch. They are counted separately and ``head_impacts`` never
  includes body sensors.

* **Attributed vs unattributed.** A wearable that no athlete owns yet still gets
  its impacts recorded, under a synthetic ``unassigned:<wid>`` key. Dropping a
  real head impact because someone had not finished setting up the roster is not
  an acceptable failure mode.

Stdlib only, same as the rest of the receiver.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional

import roster as roster_mod

# Severity buckets by peak resultant linear acceleration, in g.
#
# These are DISPLAY buckets, not a medical classification. Concussion risk
# depends on linear acceleration, rotational velocity, duration and impact
# location together, and no single g threshold is diagnostic. They exist so the
# app can colour a list; raw peak_g and rot_dps always travel with the event.
SEVERITY_LIGHT = 0
SEVERITY_MODERATE = 1
SEVERITY_SEVERE = 2

SEVERITY_BOUNDS_G = (40.0, 60.0)
SEVERITY_NAMES = ("light", "moderate", "severe")


def classify(peak_g: float) -> int:
    lo, hi = SEVERITY_BOUNDS_G
    if peak_g >= hi:
        return SEVERITY_SEVERE
    if peak_g >= lo:
        return SEVERITY_MODERATE
    return SEVERITY_LIGHT


def unattributed_key(wid: int) -> str:
    return f"unassigned:{wid}"


class DeviceState:
    """Totals for one wearable on one body position."""

    __slots__ = ("wid", "position", "impacts", "peak_g", "accum_g", "last_t_s")

    def __init__(self, wid: int, position: str):
        self.wid = wid
        self.position = position
        self.impacts = 0
        self.peak_g = 0.0
        self.accum_g = 0.0
        self.last_t_s: Optional[float] = None

    def as_dict(self) -> dict:
        return {
            "wid": self.wid,
            "position": self.position,
            "position_label": roster_mod.position_label(self.position),
            "is_head": roster_mod.is_head_position(self.position),
            "impacts": self.impacts,
            "peak_g": round(self.peak_g, 2),
            "avg_g": round(self.accum_g / self.impacts, 2) if self.impacts else 0.0,
            "last_t_s": self.last_t_s,
        }


class AthleteState:
    """Running totals for one athlete across every sensor they wear."""

    __slots__ = ("athlete_id", "name", "team", "number", "devices",
                 "head_impacts", "body_impacts", "head_peak_g", "head_accum_g",
                 "last_seen", "attributed")

    def __init__(self, athlete_id: str, name: str, team: str = "",
                 number: Optional[int] = None, attributed: bool = True):
        self.athlete_id = athlete_id
        self.name = name
        self.team = team
        self.number = number
        self.attributed = attributed
        self.devices: Dict[int, DeviceState] = {}
        self.head_impacts = 0
        self.body_impacts = 0
        self.head_peak_g = 0.0
        self.head_accum_g = 0.0
        self.last_seen: Optional[float] = None

    def as_dict(self) -> dict:
        return {
            "athlete_id": self.athlete_id,
            "athlete": self.name,
            "team": self.team,
            "number": self.number,
            "attributed": self.attributed,
            # Head totals are the headline numbers. Body impacts are reported
            # separately and deliberately never folded in.
            "head_impacts": self.head_impacts,
            "head_peak_g": round(self.head_peak_g, 2),
            "head_avg_g": (round(self.head_accum_g / self.head_impacts, 2)
                           if self.head_impacts else 0.0),
            "body_impacts": self.body_impacts,
            "devices": [self.devices[w].as_dict() for w in sorted(self.devices)],
            "age_s": (round(time.monotonic() - self.last_seen, 1)
                      if self.last_seen is not None else None),
        }


class ImpactStore:
    """Thread-safe multi-athlete impact history.

    Written from the UDP receive thread, read from the GLib/BLE thread, so every
    public method takes the lock. ``record`` does a file append while holding it
    — a few hundred microseconds, and impacts arrive at most a few per second
    per athlete even in a heavy collision sport.
    """

    def __init__(self, roster: Optional[roster_mod.Roster] = None,
                 log_path: Optional[Path] = None,
                 session: Optional[str] = None,
                 backlog: int = 1024,
                 fsync_interval_s: float = 0.5):
        self._lock = threading.Lock()
        self._roster = roster if roster is not None else roster_mod.empty_roster()
        self._athletes: Dict[str, AthleteState] = {}
        self._events: deque = deque(maxlen=backlog)
        self._seen: Dict[int, set] = {}      # wid -> seqs already recorded
        self._log_path = Path(log_path) if log_path else None
        self._log_fh = None
        self._session = session or time.strftime("%Y%m%dT%H%M%S")
        self._total = 0
        self._head_total = 0
        self._duplicates = 0
        # An fsync costs ~4 ms on SSD and 10-50 ms on a Pi's SD card, and
        # record() runs on the UDP receive thread. Doing it inline meant a
        # collision that fired 20 alerts at once could block the socket for
        # most of a second and drop other athletes' packets. Durability now
        # runs on its own thread; see _flusher.
        self._fsync_interval_s = fsync_interval_s
        self._dirty = threading.Event()
        self._closing = threading.Event()
        self._flusher = None
        self._open_log()
        if self._log_fh is not None:
            self._flusher = threading.Thread(target=self._flush_loop, daemon=True,
                                             name="impact-fsync")
            self._flusher.start()

    # -- lifecycle ----------------------------------------------------------- #
    def _open_log(self) -> None:
        if self._log_path is None:
            return
        try:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_fh = self._log_path.open("a", encoding="utf-8")
        except OSError as exc:
            print(f"# impact-store: cannot open {self._log_path} ({exc}) — "
                  f"events kept in memory only", flush=True)
            self._log_fh = None

    def _flush_loop(self) -> None:
        while not self._closing.is_set():
            if self._dirty.wait(self._fsync_interval_s):
                self._dirty.clear()
                self._fsync()

    def _fsync(self) -> None:
        fh = self._log_fh
        if fh is None:
            return
        try:
            os.fsync(fh.fileno())
        except (OSError, ValueError):
            pass

    @property
    def session(self) -> str:
        with self._lock:
            return self._session

    def set_roster(self, roster: roster_mod.Roster) -> None:
        """Point at the updated roster and refresh names on existing state."""
        with self._lock:
            self._roster = roster
            for a in roster.athletes():
                st = self._athletes.get(a.id)
                if st is not None:
                    st.name, st.team, st.number = a.name, a.team, a.number

    def new_session(self, name: Optional[str] = None) -> str:
        """Start a fresh session: totals reset, the JSONL keeps everything."""
        with self._lock:
            self._session = name or time.strftime("%Y%m%dT%H%M%S")
            self._athletes.clear()
            self._events.clear()
            self._seen.clear()
            self._total = self._head_total = 0
            return self._session

    def close(self) -> None:
        self._closing.set()
        self._dirty.set()
        if self._flusher is not None:
            self._flusher.join(timeout=2.0)
            self._flusher = None
        with self._lock:
            self._fsync()
            if self._log_fh is not None:
                try:
                    self._log_fh.close()
                except OSError:
                    pass
                self._log_fh = None

    # -- ingest -------------------------------------------------------------- #
    def _state_for(self, event: dict) -> AthleteState:
        aid = event.get("athlete_id")
        if aid:
            st = self._athletes.get(aid)
            if st is None:
                st = AthleteState(aid, event.get("athlete") or aid,
                                  event.get("team") or "", None, attributed=True)
                self._athletes[aid] = st
            return st
        key = unattributed_key(int(event.get("wid", 0)))
        st = self._athletes.get(key)
        if st is None:
            st = AthleteState(key, event.get("athlete") or key,
                              "", None, attributed=False)
            self._athletes[key] = st
        return st

    def record(self, event: dict) -> Optional[dict]:
        """Record one impact. Returns the enriched event, or None if duplicate.

        The wearable retransmits an alert until it is acked, so the SAME impact
        legitimately arrives several times — dedupe on (wid, seq) or one hit
        shows up three times in the app's list.
        """
        wid = int(event.get("wid", 0))
        seq = int(event.get("seq", 0))
        with self._lock:
            seen = self._seen.setdefault(wid, set())
            if seq in seen:
                self._duplicates += 1
                return None
            seen.add(seq)
            if len(seen) > 4096:
                for old in sorted(seen)[:1024]:
                    seen.discard(old)

            peak = float(event.get("peak_g", 0.0))
            position = str(event.get("position") or "")
            is_head = bool(event.get("is_head"))

            st = self._state_for(event)
            st.last_seen = time.monotonic()
            dev = st.devices.get(wid)
            if dev is None:
                dev = DeviceState(wid, position)
                st.devices[wid] = dev
            dev.position = position or dev.position
            dev.impacts += 1
            dev.accum_g += peak
            dev.peak_g = max(dev.peak_g, peak)
            dev.last_t_s = event.get("t_s")

            if is_head:
                st.head_impacts += 1
                st.head_accum_g += peak
                st.head_peak_g = max(st.head_peak_g, peak)
                self._head_total += 1
            else:
                st.body_impacts += 1

            rot = math.sqrt(sum(float(event.get(k, 0.0)) ** 2
                                for k in ("gx_dps", "gy_dps", "gz_dps")))

            full = dict(event)
            full.update({
                "session": self._session,
                "severity": classify(peak),
                "rot_dps": round(rot, 1),
                "epoch_ms": int(time.time() * 1000),
                "athlete_index": (st.head_impacts if is_head else st.body_impacts),
                "position_label": roster_mod.position_label(position),
            })
            self._events.append(full)
            self._total += 1

            # Append-and-flush BEFORE anything downstream can fail. This is the
            # whole reason the store exists.
            #
            # write + flush hands the bytes to the kernel, which is cheap and
            # already survives a process crash. The fsync that survives a power
            # cut is handed to the flusher thread — EXCEPT for severe impacts,
            # which are worth blocking the receive loop a few milliseconds for.
            if self._log_fh is not None:
                try:
                    self._log_fh.write(json.dumps(full, separators=(",", ":")) + "\n")
                    self._log_fh.flush()
                    if full["severity"] >= SEVERITY_SEVERE:
                        self._fsync()
                    else:
                        self._dirty.set()
                except OSError as exc:
                    print(f"# impact-store: write failed ({exc})", flush=True)
            return full

    # -- consumption --------------------------------------------------------- #
    def backlog(self, limit: int = 0) -> List[dict]:
        """Every event of the current session, oldest first (for a fresh app)."""
        with self._lock:
            events = list(self._events)
        return events[-limit:] if limit else events

    def athletes(self) -> List[dict]:
        """Per-athlete summary. Rostered athletes appear even with zero impacts,
        so the app's list matches the squad the user set up."""
        with self._lock:
            for a in self._roster.athletes():
                st = self._athletes.get(a.id)
                if st is None:
                    st = AthleteState(a.id, a.name, a.team, a.number)
                    self._athletes[a.id] = st
                for wid, position in a.devices.items():
                    if wid not in st.devices:
                        st.devices[wid] = DeviceState(wid, position)
            attributed = [s for s in self._athletes.values() if s.attributed]
            loose = [s for s in self._athletes.values() if not s.attributed]
        order = sorted(attributed, key=lambda s: (s.name.lower(), s.athlete_id))
        return [s.as_dict() for s in order] + \
               [s.as_dict() for s in sorted(loose, key=lambda s: s.athlete_id)]

    def summary(self, pi_id: int = 0) -> dict:
        with self._lock:
            total, head, dupes = self._total, self._head_total, self._duplicates
            session, revision = self._session, self._roster.revision
        return {
            "pi_id": pi_id,
            "session": session,
            "roster_revision": revision,
            "total_impacts": total,
            "head_impacts": head,
            "body_impacts": total - head,
            "duplicates_suppressed": dupes,
            "severity_bounds_g": list(SEVERITY_BOUNDS_G),
            "severity_names": list(SEVERITY_NAMES),
            "athletes": self.athletes(),
        }