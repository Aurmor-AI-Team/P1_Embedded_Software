"""Environment self-check for the receiver: is this Pi actually able to work?

Every outage this project has had on a Pi looked identical from the outside —
"the app can't see the wearable" — while the real cause was one of a handful of
environment problems that are individually silent:

  * the service can't manage NetworkManager, so the AP is never created or
    repaired and ensure_ap just logs and carries on;
  * the AP is up but not pinned to WPA2-CCMP, so ESP32 stations filter it out
    during the scan and report NO_AP_FOUND against an access point you can see
    perfectly well from a laptop;
  * the AP's password no longer matches receiver_config.json, so wearables
    associate and then die in the WPA handshake;
  * receiver_config.json went missing, so the Pi silently minted a new identity
    and every previously-provisioned wearable now targets an AP that is gone;
  * no BlueZ pairing agent, so iOS re-asks to pair on every single connection.

Run `python3 ble_sender.py --selftest` to get each of those as an explicit
PASS/FAIL with the command that fixes it. Exits non-zero if anything is broken,
so it can gate a deployment.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import wifi_ap

OK, WARN, FAIL = "PASS", "WARN", "FAIL"


def _run(argv: list[str]) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(argv, capture_output=True, text=True)
    except FileNotFoundError:
        return subprocess.CompletedProcess(argv, 127, "", "not found")


class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[str, str, str]] = []

    def add(self, status: str, what: str, detail: str = "") -> None:
        self.rows.append((status, what, detail))

    @property
    def failed(self) -> bool:
        return any(s == FAIL for s, _, _ in self.rows)

    def render(self) -> str:
        width = max(len(w) for _, w, _ in self.rows)
        out = []
        for status, what, detail in self.rows:
            out.append(f"[{status}] {what.ljust(width)}  {detail}".rstrip())
        return "\n".join(out)


def _check_nmcli(rep: Report) -> bool:
    r = _run(["nmcli", "-t", "-f", "NAME", "connection", "show"])
    if r.returncode != 0:
        rep.add(FAIL, "NetworkManager", f"nmcli unusable: {r.stderr.strip() or r.returncode}")
        return False
    rep.add(OK, "NetworkManager", "nmcli responds")
    return True


def _check_privileges(rep: Report, con: str) -> None:
    """Can we actually REPAIR the AP, or only look at it?

    This is the one that silently sank every other fix: ensure_ap detects drift
    correctly and then cannot act on it.
    """
    r = _run(["nmcli", "-s", "-g", "802-11-wireless-security.psk",
              "connection", "show", con])
    if r.returncode == 0 and r.stdout.strip():
        rep.add(OK, "NM privileges", "can read connection secrets (repairs will work)")
        return
    import getpass
    who = getpass.getuser()
    rep.add(FAIL, "NM privileges",
            f"as '{who}': cannot read connection secrets, so the AP cannot be "
            f"created or repaired.\n        This reflects WHOEVER RAN THIS — if "
            f"the service runs as root it is unaffected. Check:\n"
            f"          grep ^User= /etc/systemd/system/*.service\n"
            f"        Fix: drop the User= line (run as root), or tools/install.sh.")


def _check_config(rep: Report, cfg_path: Path) -> dict | None:
    if not cfg_path.exists():
        if wifi_ap.first_boot_pending():
            # A stripped image, booting for the first time. The service mints
            # the identity in ExecStart — which never happens if this returns
            # FAIL, because the unit gates ExecStart on this very check.
            rep.add(WARN, "receiver identity",
                    f"{cfg_path} not created yet — first boot after imaging; "
                    f"the service generates one on start.")
            return None
        rep.add(FAIL, "receiver identity",
                f"{cfg_path} missing — starting the service now would mint a "
                f"NEW identity\n        (new password + pi_id), orphaning every "
                f"provisioned wearable. Restore it from a backup.")
        return None
    try:
        cfg = json.loads(cfg_path.read_text())
    except (ValueError, UnicodeDecodeError) as exc:
        rep.add(FAIL, "receiver identity", f"{cfg_path} is not valid JSON: {exc}")
        return None
    rep.add(OK, "receiver identity",
            f"{cfg['ap_ssid']} / {cfg['receiver_name']} / pi_id={cfg['pi_id']}")
    return cfg


def _check_ap(rep: Report, cfg: dict) -> None:
    con = cfg["ap_con_name"]
    listing = _run(["nmcli", "-t", "-f", "NAME", "connection", "show"])
    if con not in listing.stdout.splitlines():
        if wifi_ap.first_boot_pending():
            rep.add(WARN, "AP profile",
                    f"'{con}' not built yet — first boot after imaging; "
                    f"the service creates it from the config on start.")
            return
        rep.add(FAIL, "AP profile",
                f"'{con}' does not exist — wearables get NO_AP_FOUND.\n"
                f"        Fix: sudo systemctl restart the service with "
                f"privileges, or run tools/install.sh")
        return

    active = _run(["nmcli", "-t", "-f", "NAME", "connection", "show", "--active"])
    if con not in active.stdout.splitlines():
        rep.add(FAIL, "AP profile", f"'{con}' exists but is not active. "
                                    f"Fix: sudo nmcli connection up {con}")
        return
    rep.add(OK, "AP profile", f"'{con}' active")

    # The invisible one: correct name, up, and filtered out by every ESP32.
    if wifi_ap._security_pinned(con):
        rep.add(OK, "AP security", "pinned to WPA2-CCMP (visible to ESP32 stations)")
    else:
        rep.add(FAIL, "AP security",
                "NOT pinned to plain WPA2-CCMP. ESP32 stations filter a "
                "WPA3-transition AP out\n        during the scan and report "
                "NO_AP_FOUND. Restart with privileges to repin.")

    psk = wifi_ap._psk_matches(cfg)
    if psk is True:
        rep.add(OK, "AP password", "matches receiver_config.json")
    elif psk is False:
        rep.add(FAIL, "AP password",
                "differs from receiver_config.json — wearables associate then "
                "fail the WPA\n        handshake (ESP32 reason 15). Restart "
                "with privileges to repair.")
    else:
        rep.add(WARN, "AP password", "not readable without privileges — cannot verify")


def _check_bluetooth(rep: Report) -> None:
    r = _run(["bluetoothctl", "show"])
    if r.returncode != 0 or "Controller" not in r.stdout:
        rep.add(FAIL, "Bluetooth", "no controller reported by bluetoothctl")
        return
    line = next((l for l in r.stdout.splitlines() if l.startswith("Controller")), "")
    rep.add(OK, "Bluetooth", line.strip())

    # A pairing agent is what lets iOS bond ONCE instead of asking every time.
    #
    # Asked via a marker the agent writes, NOT `busctl tree org.bluez`: the agent
    # object is exported by ble_sender's own bus connection, so it never appears
    # in org.bluez's tree and that check warned on healthy receivers.
    import pairing_agent
    if pairing_agent.is_registered():
        rep.add(OK, "Pairing agent", "registered (phones bond once)")
        return
    running = _run(["systemctl", "is-active", "--quiet", "ble_sender"]).returncode == 0 \
        or _run(["systemctl", "is-active", "--quiet", "aurmor-receiver"]).returncode == 0
    if running:
        # `systemctl restart` returns as soon as the process is SPAWNED, but the
        # service still has to bring up the AP and BLE before it registers. Run
        # the self-test straight after a restart — which is exactly what anyone
        # does — and a perfectly healthy receiver reports a scary FAIL. Give it a
        # moment rather than teaching people to ignore this tool.
        import time
        for _ in range(12):
            time.sleep(1)
            if pairing_agent.is_registered():
                rep.add(OK, "Pairing agent", "registered (phones bond once)")
                return
        rep.add(FAIL, "Pairing agent",
                "service is running but no agent registered — iOS will re-ask to "
                "pair on\n        EVERY connection and pairing will never "
                "complete. Check the log for\n        '# could not register "
                "pairing agent'.")
    else:
        rep.add(WARN, "Pairing agent",
                "not registered — expected while the service is stopped "
                "(it registers its own on start).")


def _check_modules(rep: Report) -> None:
    """Catch deployment skew: a module that failed to copy takes the whole
    service down at import time with nothing but status=1/FAILURE."""
    missing = []
    for name in ("protocol", "replay", "pose", "wifi_ap", "udp_source",
                 "pairing_agent"):
        try:
            __import__(name)
        except Exception as exc:  # noqa: BLE001 - reporting, not handling
            missing.append(f"{name} ({type(exc).__name__}: {exc})")
    if missing:
        rep.add(FAIL, "python modules", "; ".join(missing))
    else:
        rep.add(OK, "python modules", "all import cleanly")


def run(cfg_path: Path) -> int:
    rep = Report()
    _check_modules(rep)
    if _check_nmcli(rep):
        cfg = _check_config(rep, Path(cfg_path))
        if cfg:
            _check_privileges(rep, cfg["ap_con_name"])
            _check_ap(rep, cfg)
    _check_bluetooth(rep)

    print(rep.render())
    if rep.failed:
        print("\nSELF-TEST FAILED — the receiver will not work correctly as configured.",
              file=sys.stderr)
        return 1
    print("\nSelf-test passed.")
    return 0
