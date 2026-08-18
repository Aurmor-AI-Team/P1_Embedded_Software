"""Hidden WiFi access point on the Pi, managed through NetworkManager (nmcli).

The receiver permanently hosts a hidden AP; ESP32 wearables join it after the
mobile app forwards the credentials over BLE (see the WIFI_CREDS
characteristic in ble_sender.py). Requires Raspberry Pi OS Bookworm or any
system where nmcli manages wlan0.

Credentials and identity live in ``receiver_config.json`` next to this file,
generated with random secrets on first run so every Pi gets its own.
"""
from __future__ import annotations

import json
import secrets
import subprocess
import sys
from pathlib import Path

# NetworkManager assigns this gateway address to the AP interface when the
# connection uses ipv4.method=shared.
AP_IP = "10.42.0.1"

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "receiver_config.json"

# The old shared default. Several receivers side by side must NOT share an SSID
# (the ESP32 would treat them as one roaming network and pick by signal, not
# identity), so a config still carrying this literal is migrated to a unique one.
_LEGACY_SHARED_SSID = "aurmor-pi-ap"

_DEFAULTS = {
    "ap_con_name": "aurmor-ap",
    "ap_ifname": "wlan0",
    # Channels 1/6/11 are active-scannable in every regulatory domain. An
    # auto-picked 12/13 is passive-scan-only for world-domain ESP32s, and a
    # hidden AP can't be found by passive scan at all.
    "ap_channel": 6,
    "udp_port": 5005,
}


def _pi_serial_suffix():
    """Last 4 hex chars of the Raspberry Pi's unique board serial, or None off-Pi.

    The board serial is stable, globally unique per unit, readable before any
    network interface comes up, and immune to MAC randomization — a better
    identity source than the WiFi MAC.
    """
    serial = ""
    # Device-tree node (preferred): a NUL-terminated ASCII serial string.
    try:
        raw = Path("/sys/firmware/devicetree/base/serial-number").read_bytes()
        serial = raw.split(b"\x00", 1)[0].decode("ascii", "ignore")
    except OSError:
        pass
    if not serial:
        # Fallback: the "Serial" line in /proc/cpuinfo.
        try:
            for line in Path("/proc/cpuinfo").read_text().splitlines():
                if line.lower().startswith("serial"):
                    serial = line.split(":", 1)[1]
                    break
        except OSError:
            pass
    serial = "".join(c for c in serial.lower() if c in "0123456789abcdef")
    return serial[-4:] if len(serial) >= 4 else None


def _unique_suffix() -> str:
    # Pi board serial is stable + human-recognizable; random is the fallback
    # (e.g. on a dev host). Either way it is stored once in receiver_config.json.
    return _pi_serial_suffix() or secrets.token_hex(2)


def load_config(path: Path = DEFAULT_CONFIG_PATH) -> dict:
    """Read the receiver config, creating it with generated secrets + a unique
    per-Pi identity if absent."""
    path = Path(path)
    existed = path.exists()
    if existed:
        try:
            cfg = json.loads(path.read_text())
        except (ValueError, UnicodeDecodeError) as exc:
            # Deliberately fatal. Falling through to `cfg = {}` here would mint a
            # BRAND NEW identity — new SSID, password and pi_id — silently
            # orphaning every wearable already provisioned to this receiver and
            # the receiver's own registration in the app. Losing the file is
            # recoverable from a backup; regenerating over it is not.
            raise SystemExit(
                f"{path} exists but is not valid JSON ({exc}).\n"
                f"Refusing to generate a new receiver identity over it — that "
                f"would change this Pi's SSID, password and pi_id, and every "
                f"wearable provisioned to it would stop connecting.\n"
                f"Restore it from {path.with_suffix('.json.bak')} or a backup, "
                f"or delete it deliberately to start a new identity."
            ) from exc
    else:
        cfg = {}
    changed = False
    for key, value in _DEFAULTS.items():
        if key not in cfg:
            cfg[key] = value
            changed = True

    # Per-Pi identity: a unique hidden SSID (so the ESP32 targets exactly this
    # receiver) and a matching human-readable receiver name, sharing one suffix.
    ssid = cfg.get("ap_ssid")
    if not ssid or ssid == _LEGACY_SHARED_SSID:
        suffix = _unique_suffix()
        cfg["ap_ssid"] = f"aurmor-pi-{suffix}"
        cfg["receiver_name"] = f"aurmor-rpi-{suffix}"
        changed = True
    elif not cfg.get("receiver_name"):
        cfg["receiver_name"] = f"aurmor-rpi-{ssid.rsplit('-', 1)[-1]}"
        changed = True

    if not cfg.get("ap_password"):
        cfg["ap_password"] = secrets.token_urlsafe(9)  # 12 chars, WPA2-valid
        changed = True
    if not cfg.get("pi_id"):
        cfg["pi_id"] = secrets.randbelow(0xFFFFFFFE) + 1  # nonzero u32
        changed = True
    if changed:
        # Keep the previous file. The values in here ARE this receiver's
        # identity, so an accidental overwrite is what turns "the config went
        # missing" into "every wearable and the app now point at a receiver that
        # no longer exists" — with the old secrets gone and no way back.
        if existed:
            try:
                path.with_suffix(".json.bak").write_text(path.read_text())
            except OSError as exc:  # noqa: BLE001 - a backup is a nicety
                print(f"# could not back up {path}: {exc}", file=sys.stderr)

        path.write_text(json.dumps(cfg, indent=2) + "\n")
        if existed:
            print(f"# wrote receiver config -> {path}", file=sys.stderr)
        else:
            # A fresh identity. Say so unmistakably: anything provisioned
            # against the previous one is now pointing at an AP that is gone,
            # and the symptom (associate, then WPA handshake timeout, ESP32
            # disconnect reason 15) looks nothing like "the config was missing".
            print(f"# ---------------------------------------------------------\n"
                  f"# NEW RECEIVER IDENTITY generated ({path} was missing):\n"
                  f"#   SSID   {cfg['ap_ssid']}\n"
                  f"#   name   {cfg['receiver_name']}\n"
                  f"#   pi_id  {cfg['pi_id']}\n"
                  f"# Wearables provisioned to the PREVIOUS identity can no "
                  f"longer join, and the app's registered receiver no longer\n"
                  f"# matches. If this Pi worked before, restore the old config "
                  f"instead of continuing. If the AP profile still exists with\n"
                  f"# the old password, delete it so it is rebuilt:\n"
                  f"#   sudo nmcli connection delete {cfg['ap_con_name']}\n"
                  f"# ---------------------------------------------------------",
                  file=sys.stderr)
    return cfg


def wifi_creds_json(cfg: dict) -> bytes:
    """The WIFI_CREDS characteristic payload the app forwards to the ESP32."""
    return json.dumps({
        "ssid": cfg["ap_ssid"],
        "password": cfg["ap_password"],
        "ip": AP_IP,
        "port": cfg["udp_port"],
        "pi_id": cfg["pi_id"],
        "receiver_name": cfg["receiver_name"],
    }, separators=(",", ":")).encode("utf-8")


def _nmcli(args: list, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(["nmcli", *args], capture_output=True, text=True, **kwargs)


_PRIV_HINT = ("creating/modifying the AP profile needs privileges: run once "
              "with sudo, or grant them permanently (usermod -aG netdev + a "
              "polkit rule; see README.md)")


def _psk_matches(cfg: dict):
    """Whether the live profile's PSK is the one in our config.

    True / False / None, where None means "not privileged enough to tell".

    This exists because the SSID is NOT sufficient evidence. It is derived from
    the Pi's board serial, so it survives a config regeneration unchanged — but
    ap_password is random and does not. Regenerate receiver_config.json (which
    happens whenever the file goes missing or unreadable) and you get an AP whose
    SSID still matches while its key no longer does. Wearables are then handed a
    password the live AP will not accept: they associate fine and die in the
    4-way handshake, which shows up as ESP32 disconnect reason 15 after a
    reason 201, and looks for all the world like a firmware bug.

    Reading it needs privileges (`nmcli -s`), hence the None case.
    """
    r = _nmcli(["-s", "-g", "802-11-wireless-security.psk",
                "connection", "show", cfg["ap_con_name"]])
    if r.returncode != 0:
        return None
    live = r.stdout.strip()
    if not live:
        return None          # blank == withheld, not "no password set"
    return live == cfg["ap_password"]


def _security_pinned(con: str) -> bool:
    """True when the profile pins plain WPA2-CCMP.

    Checked explicitly because it is invisible in every other way: an AP left on
    NetworkManager's defaults is a WPA2/WPA3 transition AP, and an ESP32 station
    asks for WIFI_AUTH_WPA2_PSK, so the driver FILTERS THAT AP OUT DURING THE
    SCAN. The board then reports NO_AP_FOUND (reason 201) and never reaches
    auth — i.e. an access point that is up, active, correctly named and
    completely invisible to our own hardware.

    These are not secrets, so this works unprivileged.
    """
    r = _nmcli(["-g", "802-11-wireless-security.key-mgmt,"
                "802-11-wireless-security.proto,"
                "802-11-wireless-security.pairwise,"
                "802-11-wireless-security.group,"
                "802-11-wireless-security.pmf",
                "connection", "show", con])
    if r.returncode != 0:
        return True   # can't tell — don't churn a profile on a failed probe
    got = [line.strip().lower() for line in r.stdout.splitlines()]
    while len(got) < 5:
        got.append("")
    key_mgmt, proto, pairwise, group, pmf = got[:5]
    # nmcli renders pmf as "1 (disable)" in some versions and "disable" in others.
    pmf_ok = "disable" in pmf or pmf.startswith("1")
    return (key_mgmt == "wpa-psk" and proto == "rsn"
            and pairwise == "ccmp" and group == "ccmp" and pmf_ok)


def _settings_match(cfg: dict) -> bool:
    """True when the existing profile already carries our SSID/AP/hidden/channel
    settings, our WPA2 pinning, AND — where we are allowed to look — our
    password. A mismatch sends ensure_ap down the reconfigure path, which is the
    repair."""
    r = _nmcli(["-g", "802-11-wireless.ssid,802-11-wireless.hidden,"
                "802-11-wireless.mode,802-11-wireless.channel",
                "connection", "show", cfg["ap_con_name"]])
    if r.returncode != 0 or r.stdout.splitlines() != [
            cfg["ap_ssid"], "yes", "ap", str(cfg["ap_channel"])]:
        return False

    if not _security_pinned(cfg["ap_con_name"]):
        print(f"# wifi-ap: '{cfg['ap_con_name']}' is not pinned to plain "
              f"WPA2-CCMP — ESP32 stations filter a WPA3-transition AP out "
              f"during the scan and report NO_AP_FOUND. Reconfiguring it.",
              file=sys.stderr)
        return False

    psk = _psk_matches(cfg)
    if psk is False:
        print(f"# wifi-ap: '{cfg['ap_con_name']}' has a DIFFERENT password than "
              f"receiver_config.json — wearables would associate and then fail "
              f"the WPA handshake. Reconfiguring it.", file=sys.stderr)
        return False
    if psk is None:
        print(f"# wifi-ap: cannot read '{cfg['ap_con_name']}' password without "
              f"privileges, so password drift can't be ruled out. If wearables "
              f"associate but never get an IP, run:\n"
              f"#   sudo nmcli -s -g 802-11-wireless-security.psk connection "
              f"show {cfg['ap_con_name']}\n"
              f"#   sudo nmcli connection delete {cfg['ap_con_name']}   "
              f"# then restart this service to rebuild it",
              file=sys.stderr)
    return True


def _is_active(con: str) -> bool:
    r = _nmcli(["-t", "-f", "NAME", "connection", "show", "--active"])
    return r.returncode == 0 and con in r.stdout.splitlines()


def ensure_ap(cfg: dict) -> bool:
    """Idempotently create + activate the hidden AP profile. Unprivileged runs
    are read-only when the profile already matches the config (at most a
    `connection up`). Never raises: on any failure it logs and returns False
    so BLE streaming still starts."""
    con = cfg["ap_con_name"]
    try:
        existing = _nmcli(["-t", "-f", "NAME", "connection", "show"])
        if existing.returncode != 0:
            print(f"# wifi-ap: nmcli unavailable ({existing.stderr.strip()}) — "
                  f"no access point; is NetworkManager running?", file=sys.stderr)
            return False
        exists = con in existing.stdout.splitlines()

        if exists and _settings_match(cfg):
            if _is_active(con):
                print(f"# wifi-ap: hidden AP '{cfg['ap_ssid']}' already active "
                      f"on {cfg['ap_ifname']} ({AP_IP})", file=sys.stderr)
                return True
        else:
            settings = [
                "802-11-wireless.mode", "ap",
                "802-11-wireless.band", "bg",
                "802-11-wireless.channel", str(cfg["ap_channel"]),
                "802-11-wireless.hidden", "yes",
                "802-11-wireless.ssid", cfg["ap_ssid"],
                "ipv4.method", "shared",
                "wifi-sec.key-mgmt", "wpa-psk",
                "wifi-sec.psk", cfg["ap_password"],
                # Pin plain WPA2-CCMP: NM's default lets wpa_supplicant offer a
                # WPA3-SAE transition AP, which ESP32 stations reject with
                # disconnect reason 211 (no AP with compatible security).
                "wifi-sec.proto", "rsn",
                "wifi-sec.pairwise", "ccmp",
                "wifi-sec.group", "ccmp",
                "wifi-sec.pmf", "disable",
                "connection.autoconnect", "yes",
            ]
            if exists:
                result = _nmcli(["connection", "modify", con, *settings])
            else:
                result = _nmcli(["connection", "add", "type", "wifi",
                                 "ifname", cfg["ap_ifname"], "con-name", con,
                                 *settings])
            if result.returncode != 0:
                print(f"# wifi-ap: configuring '{con}' failed: "
                      f"{result.stderr.strip()} — {_PRIV_HINT}", file=sys.stderr)
                return False

        up = _nmcli(["connection", "up", con])
        if up.returncode != 0:
            print(f"# wifi-ap: activating '{con}' failed: {up.stderr.strip()} — "
                  f"try `sudo nmcli con up {con}`", file=sys.stderr)
            return False

        print(f"# wifi-ap: hidden AP '{cfg['ap_ssid']}' up on "
              f"{cfg['ap_ifname']} ({AP_IP})", file=sys.stderr)
        return True
    except FileNotFoundError:
        print("# wifi-ap: nmcli not found — no access point. Install "
              "NetworkManager or run with --no-ap.", file=sys.stderr)
        return False
    except Exception as exc:  # AP is best-effort; the BLE stream must come up
        print(f"# wifi-ap: unexpected error: {exc}", file=sys.stderr)
        return False
