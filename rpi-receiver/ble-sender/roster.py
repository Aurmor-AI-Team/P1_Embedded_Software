"""Athlete roster: who is being monitored, and which wearables are on them.

The receiver used to carry a flat, hard-coded ``wid -> node`` table describing
ONE athlete wearing four sensors. This replaces it with the shape the product
actually needs:

    athlete  --< device (wearable_id, body position)

An athlete may wear one sensor or several; a wearable belongs to at most one
athlete. Nothing is assumed at startup — the receiver boots with an EMPTY
roster and the app populates it, so a fresh Pi monitors nobody until told to.

Two things here are load-bearing:

* **Stable node indices.** The binary sample frame identifies a node by a
  one-byte index into the ``nodes`` list published in Meta. The roster is now
  editable at runtime, so if indices were derived by sorting, adding an athlete
  mid-session would renumber existing nodes and any sample already in flight
  would decode as the wrong person. Indices are therefore append-only for the
  life of a session and persisted with the roster.

* **HEAD is not just another position.** A wrist sensor reading 45 g is an arm
  swing; a head sensor reading 45 g is the thing this product exists to catch.
  ``is_head_device()`` keeps those from being counted together.

Stdlib only, same as the rest of the receiver.
"""
from __future__ import annotations

import json
import re
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

ROSTER_SCHEMA = 1

# Squad capacity. Mirrors protocol.MAX_* but duplicated here so roster.py stays
# import-light and usable standalone; a mismatch is caught by the tests.
MAX_ATHLETES = 30
MAX_DEVICES_PER_ATHLETE = 6
MAX_DEVICES = MAX_ATHLETES * MAX_DEVICES_PER_ATHLETE   # 180
MAX_WEARABLE_ID = 0xFFFF     # the wire header carries wid as u16

# Canonical body positions. Codes match the app's ESP32_KIND_MAP so a device
# assigned at pairing lands on the same code here. Unknown codes are accepted
# (the app may add positions before the Pi knows about them) but are treated as
# non-head. TO-DO - add more body positions.
POSITIONS: Dict[str, str] = {
    "HEAD": "Head",
    "WA": "Chest",
    "WD": "Left wrist",
    "WE": "Right wrist",
    "BACK": "Upper back",
    "WAIST": "Waist",
    "LSHIN": "Left shin",
    "RSHIN": "Right shin",
}

# Positions whose impacts count as HEAD impacts. Everything else is recorded as
# a body impact and kept out of the head-impact totals.
HEAD_POSITIONS = {"HEAD"}

_SLUG_RE = re.compile(r"[^a-zA-Z0-9]+")


def is_head_position(position: str) -> bool:
    return (position or "").upper() in HEAD_POSITIONS


def position_label(code: str) -> str:
    return POSITIONS.get((code or "").upper(), code or "?")


def _slug(text: str, fallback: str) -> str:
    s = _SLUG_RE.sub("-", (text or "").strip().lower()).strip("-")
    return s or fallback


class RosterError(ValueError):
    """Raised when the app sends a roster that cannot be applied."""


class Athlete:
    __slots__ = ("id", "name", "team", "number", "devices", "node_overrides")

    def __init__(self, id: str, name: str, team: str = "",
                 number: Optional[int] = None,
                 devices: Optional[Dict[int, str]] = None,
                 node_overrides: Optional[Dict[int, str]] = None):
        self.id = id
        self.name = name
        self.team = team
        self.number = number
        self.devices: Dict[int, str] = dict(devices or {})
        # Optional explicit node id per wearable. Lets a single-athlete rig keep
        # its historical flat node names ("HEAD", "WA") instead of "id/POS".
        self.node_overrides: Dict[int, str] = dict(node_overrides or {})

    def head_wids(self) -> List[int]:
        return sorted(w for w, p in self.devices.items() if is_head_position(p))

    def as_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "team": self.team,
            "number": self.number,
            "devices": {
                str(w): ({"position": p, "node": self.node_overrides[w]}
                         if w in self.node_overrides else p)
                for w, p in sorted(self.devices.items())
            },
        }


class Roster:
    """Thread-safe athlete/device assignment with stable node indices.

    Read from the UDP receive thread on every packet, written from the BLE
    thread when the app edits it, so every accessor takes the lock.
    """

    def __init__(self, path: Optional[Path] = None):
        self._lock = threading.RLock()
        self._path = Path(path) if path else None
        self._athletes: Dict[str, Athlete] = {}
        self._by_wid: Dict[int, Tuple[str, str, str]] = {}  # wid -> (athlete_id, position, node)
        self._node_index: Dict[str, int] = {}             # node id -> stable index
        self._nodes: List[str] = []                       # index -> node id
        self._revision = 0

    # -- lookup -------------------------------------------------------------- #
    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    def is_empty(self) -> bool:
        with self._lock:
            return not self._by_wid

    def athletes(self) -> List[Athlete]:
        with self._lock:
            return [self._athletes[a] for a in sorted(self._athletes)]

    def head_wids(self) -> List[int]:
        """Wearables in a head position, across every athlete."""
        with self._lock:
            return sorted(w for w, e in self._by_wid.items()
                          if is_head_position(e[1]))

    def assigned_wids(self) -> List[int]:
        with self._lock:
            return sorted(self._by_wid)

    def lookup(self, wid: int) -> Optional[dict]:
        """Everything the receive path needs for one packet, or None if the
        wearable is not assigned to anybody."""
        with self._lock:
            entry = self._by_wid.get(wid)
            if entry is None:
                return None
            athlete_id, position, node = entry
            a = self._athletes[athlete_id]
            return {
                "wid": wid,
                "athlete_id": athlete_id,
                "athlete": a.name,
                "team": a.team,
                "number": a.number,
                "position": position,
                "is_head": is_head_position(position),
                "node": node,
                "node_idx": self._node_index.get(node, 0),
            }

    def nodes(self) -> List[str]:
        """Node ids in stable index order (index i == nodes()[i])."""
        with self._lock:
            return list(self._nodes)

    def node_meta(self) -> List[dict]:
        """Parallel to nodes(): who and where each node is, for the app."""
        with self._lock:
            out = []
            owner = {e[2]: (e[0], e[1]) for e in self._by_wid.values()}
            for node in self._nodes:
                athlete_id, position = owner.get(node, node.partition("/")[::2])
                a = self._athletes.get(athlete_id)
                out.append({
                    "node": node,
                    "athlete_id": athlete_id,
                    "athlete": a.name if a else "(removed)",
                    "team": a.team if a else "",
                    "number": a.number if a else None,
                    "position": position,
                    "position_label": position_label(position),
                    "is_head": is_head_position(position),
                    "wid": next((w for w, e in self._by_wid.items()
                                 if e[2] == node), None),
                })
            return out

    # -- mutation ------------------------------------------------------------ #
    def apply(self, doc: dict) -> dict:
        """Replace the roster from an app-supplied document. Validates first and
        applies nothing on error, so a malformed edit can't half-apply and leave
        athletes partially unmonitored."""
        athletes, by_wid = _parse(doc)
        with self._lock:
            self._athletes = athletes
            self._by_wid = by_wid
            # Append-only index assignment: an existing node keeps its index
            # forever, a new one goes on the end. Never renumber.
            for wid in sorted(by_wid):
                _athlete_id, _position, node = by_wid[wid]
                if node not in self._node_index:
                    self._node_index[node] = len(self._nodes)
                    self._nodes.append(node)
            self._revision += 1
            self.save()
            return self.as_dict()

    def assign(self, wid: int, athlete_id: str, position: str) -> dict:
        """Attach one wearable to one athlete (the app's incremental path)."""
        with self._lock:
            doc = self.as_dict()
            for a in doc["athletes"]:
                a["devices"] = {w: p for w, p in a["devices"].items()
                                if int(w) != wid}
                if a["id"] == athlete_id:
                    a["devices"][str(wid)] = position.strip().upper()
            if not any(a["id"] == athlete_id for a in doc["athletes"]):
                raise RosterError(f"unknown athlete id {athlete_id!r}")
            return self.apply(doc)

    @classmethod
    def from_legacy_map(cls, wid_to_node: Dict[int, str], name: str = "Athlete 1",
                        path: Optional[Path] = None) -> "Roster":
        """One athlete wearing several sensors, using the historical flat node
        names. This is the pre-roster deployment expressed in the new model."""
        r = cls(path)
        devices = {str(w): {"position": n, "node": n}
                   for w, n in wid_to_node.items()}
        r.apply({"athletes": [{"id": "athlete-1", "name": name,
                               "devices": devices}]})
        return r

    def assign_head(self, wid: int, athlete_id: str) -> dict:
        """Hand one head sensor to one athlete — the group check-in path.

        This is the whole assignment flow for a one-sensor-per-athlete session:
        tap an athlete, tap a device off the unassigned list. Reassigning a
        wid moves it, so re-handing a helmet to a different player mid-session
        does the obvious thing rather than erroring.
        """
        return self.assign(wid, athlete_id, "HEAD")

    def add_athlete(self, name: str, athlete_id: str = "", team: str = "",
                    number=None) -> dict:
        """Register an athlete with no device yet (squad setup)."""
        doc = self.as_dict()
        aid = athlete_id or _slug(name, f"athlete{len(doc['athletes'])}")
        if any(a["id"] == aid for a in doc["athletes"]):
            raise RosterError(f"athlete id {aid!r} already exists")
        doc["athletes"].append({"id": aid, "name": name, "team": team,
                                "number": number, "devices": {}})
        return self.apply(doc)

    def unassigned_athletes(self) -> List["Athlete"]:
        """Squad members nobody has handed a sensor to. A pre-session check
        should be loud about these — an athlete with no device is invisible."""
        return [a for a in self.athletes() if not a.devices]

    def unassign(self, wid: int) -> dict:
        with self._lock:
            doc = self.as_dict()
            for a in doc["athletes"]:
                a["devices"] = {w: p for w, p in a["devices"].items()
                                if int(w) != wid}
            return self.apply(doc)

    # -- persistence --------------------------------------------------------- #
    def as_dict(self) -> dict:
        with self._lock:
            return {
                "schema": ROSTER_SCHEMA,
                "revision": self._revision,
                "athletes": [a.as_dict() for a in self.athletes()],
                "node_index": dict(self._node_index),
            }

    def save(self) -> None:
        if self._path is None:
            return
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_suffix(self._path.suffix + ".tmp")
            tmp.write_text(json.dumps(self.as_dict(), indent=2) + "\n")
            tmp.replace(self._path)          # atomic: never a truncated roster
        except OSError as exc:
            print(f"# roster: save to {self._path} failed ({exc})", flush=True)

    def load(self) -> "Roster":
        """Restore a saved roster. A missing or unreadable file is not an error:
        the receiver is supposed to start empty."""
        if self._path is None or not self._path.is_file():
            return self
        try:
            doc = json.loads(self._path.read_text())
        except (OSError, ValueError) as exc:
            print(f"# roster: {self._path} unreadable ({exc}) — starting empty",
                  flush=True)
            return self
        saved_index = doc.get("node_index") or {}
        try:
            self.apply(doc)
        except RosterError as exc:
            print(f"# roster: saved file invalid ({exc}) — starting empty",
                  flush=True)
            return self
        # Restore the previous index assignment so node indices survive a
        # restart; anything new keeps the index apply() just handed it.
        with self._lock:
            if saved_index:
                merged = dict(saved_index)
                for node in self._nodes:
                    merged.setdefault(node, len(merged))
                size = max(merged.values()) + 1 if merged else 0
                nodes = [""] * size
                for node, idx in merged.items():
                    if 0 <= idx < size:
                        nodes[idx] = node
                self._node_index = merged
                self._nodes = nodes
        return self


def capacity(roster: "Roster") -> dict:
    """What the squad currently costs, for the app's setup screen."""
    athletes = roster.athletes()
    devices = roster.assigned_wids()
    return {
        "athletes": len(athletes),
        "max_athletes": MAX_ATHLETES,
        "devices": len(devices),
        "max_devices": MAX_DEVICES,
        "max_devices_per_athlete": MAX_DEVICES_PER_ATHLETE,
        "head_sensors": len(roster.head_wids()),
        "athletes_without_head_sensor": [
            a.id for a in athletes if not a.head_wids()],
    }


def node_id(athlete_id: str, position: str) -> str:
    """Globally unique node identifier. Two athletes both wearing a HEAD sensor
    must not collapse onto one node — that is how you attribute a concussion to
    the wrong person."""
    return f"{athlete_id}/{position}"


def _parse(doc: dict) -> Tuple[Dict[str, Athlete], Dict[int, Tuple[str, str, str]]]:
    """Validate an app-supplied roster document. Raises RosterError."""
    if not isinstance(doc, dict):
        raise RosterError("roster must be a JSON object")
    raw = doc.get("athletes")
    if raw is None:
        raise RosterError("roster needs an 'athletes' array")
    if not isinstance(raw, list):
        raise RosterError("'athletes' must be an array")

    if len(raw) > MAX_ATHLETES:
        raise RosterError(
            f"{len(raw)} athletes exceeds the {MAX_ATHLETES}-athlete limit — "
            f"split the squad across receivers")

    athletes: Dict[str, Athlete] = {}
    by_wid: Dict[int, Tuple[str, str, str]] = {}

    for i, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise RosterError(f"athletes[{i}] must be an object")
        name = str(entry.get("name") or "").strip()
        if not name:
            raise RosterError(f"athletes[{i}] needs a name")
        aid = str(entry.get("id") or "").strip() or _slug(name, f"athlete{i}")
        if aid in athletes:
            raise RosterError(f"duplicate athlete id {aid!r}")

        devices: Dict[int, str] = {}
        overrides: Dict[int, str] = {}
        seen_positions = set()
        for key, spec in (entry.get("devices") or {}).items():
            try:
                wid = int(key)
            except (TypeError, ValueError):
                raise RosterError(f"{name}: device key {key!r} is not a wearable id")
            if not 1 <= wid <= MAX_WEARABLE_ID:
                raise RosterError(
                    f"{name}: wearable id {wid} out of range 1-{MAX_WEARABLE_ID}")
            node_override = None
            if isinstance(spec, dict):
                node_override = str(spec.get("node") or "").strip() or None
                spec = spec.get("position")
            position = str(spec or "").strip().upper()
            if not position:
                raise RosterError(f"{name}: wearable {wid} needs a body position")
            if wid in by_wid:
                other = by_wid[wid][0]
                raise RosterError(
                    f"wearable {wid} assigned to both {other!r} and {aid!r} — "
                    f"a device can only be on one athlete")
            if position in seen_positions:
                raise RosterError(
                    f"{name}: two devices assigned to {position} — "
                    f"positions must be unique per athlete")
            seen_positions.add(position)
            devices[wid] = position
            if node_override:
                overrides[wid] = node_override
            by_wid[wid] = (aid, position, node_override or node_id(aid, position))

        if len(devices) > MAX_DEVICES_PER_ATHLETE:
            raise RosterError(
                f"{name}: {len(devices)} devices exceeds the "
                f"{MAX_DEVICES_PER_ATHLETE}-per-athlete limit")

        number = entry.get("number")
        if number is not None:
            try:
                number = int(number)
            except (TypeError, ValueError):
                number = None
        athletes[aid] = Athlete(aid, name, str(entry.get("team") or ""),
                                number, devices, overrides)

    if len(by_wid) > MAX_DEVICES:
        raise RosterError(
            f"{len(by_wid)} devices exceeds the {MAX_DEVICES}-device limit")
    return athletes, by_wid


def empty_roster(path: Optional[Path] = None) -> Roster:
    """A receiver monitors nobody until the app says otherwise."""
    return Roster(path)