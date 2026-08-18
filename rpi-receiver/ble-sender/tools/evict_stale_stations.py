#!/usr/bin/env python3
"""Evict "ghost" WiFi stations from the receiver's AP.

When an ESP32 loses power it can't send a graceful disconnect, so the AP keeps a
stale association for its MAC. On reboot the board's re-auth collides with that
ghost and the AP silently drops it (disconnect reason 2, "previous auth no
longer valid") until the entry is removed.

Inactivity-based eviction does NOT work here: the board retries auth every ~2.5 s
and each attempt resets the station's inactivity timer, so it never looks idle.
Instead we watch **data packets** — a live board sends a HELLO every 2 s (real
data → rx packets climb), while a ghost passes zero data (only auth/mgmt frames,
which don't count). A station that passes no data for a full window is a ghost.

Needs CAP_NET_ADMIN (run as root / via the systemd unit).
"""
import os
import re
import subprocess
import sys
import time

IFACE = os.environ.get("AURMOR_AP_IFACE", "wlan0")
INTERVAL_S = float(os.environ.get("AURMOR_EVICT_INTERVAL_S", "5"))
WINDOW_S = float(os.environ.get("AURMOR_EVICT_WINDOW_S", "10"))
# A station passing fewer than this many data packets across the window is a
# ghost. A live board HELLOs every 2 s (~5 packets / 10 s), so it never trips.
MIN_PACKETS = int(os.environ.get("AURMOR_EVICT_MIN_PKTS", "2"))

_STATION = re.compile(r"^Station ([0-9a-fA-F:]{17})")


def station_rx_packets():
    """{mac: rx_packet_count} for every associated station."""
    try:
        out = subprocess.run(["iw", "dev", IFACE, "station", "dump"],
                             capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"evict-stale-stations: iw dump failed: {exc}", file=sys.stderr)
        return {}
    stations = {}
    mac = None
    for line in out.stdout.splitlines():
        m = _STATION.match(line.strip())
        if m:
            mac = m.group(1).lower()
            stations[mac] = 0
        elif mac and "rx packets:" in line:
            try:
                stations[mac] = int(line.split(":", 1)[1].strip())
            except ValueError:
                pass
    return stations


def evict(mac):
    subprocess.run(["iw", "dev", IFACE, "station", "del", mac],
                   capture_output=True, text=True)


def main():
    print(f"evict-stale-stations: iface={IFACE} interval={INTERVAL_S}s "
          f"window={WINDOW_S}s min_pkts={MIN_PACKETS}", file=sys.stderr)
    # mac -> (rx_packets_at_window_start, monotonic_time_at_window_start)
    baseline = {}
    while True:
        now = time.monotonic()
        current = station_rx_packets()
        for mac, pkts in current.items():
            base = baseline.get(mac)
            if base is None:
                baseline[mac] = (pkts, now)
                continue
            base_pkts, base_t = base
            if now - base_t < WINDOW_S:
                continue
            if pkts - base_pkts < MIN_PACKETS:
                print(f"evict-stale-stations: removing ghost {mac} "
                      f"(+{pkts - base_pkts} data pkts in {now - base_t:.0f}s)",
                      file=sys.stderr)
                evict(mac)
                baseline.pop(mac, None)
            else:
                baseline[mac] = (pkts, now)  # still live — start a new window
        # Forget stations that are no longer associated.
        for mac in [m for m in baseline if m not in current]:
            baseline.pop(mac, None)
        time.sleep(INTERVAL_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
